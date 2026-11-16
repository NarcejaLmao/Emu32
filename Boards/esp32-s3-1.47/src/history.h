// Persistent recently-played history and Recent Games menu.
// Included by menus.h after shared UI helpers.
#pragma once

// Persistent recently-played list. Paths are stored one per line in /system/recents.txt.
static const int RECENTS_MAX = 12;
static char recentPaths[RECENTS_MAX][ROM_PATH_LEN];
static int recentCount = 0;
static char recentLaunchPath[ROM_PATH_LEN] = "";
// Prevent a transient/partial read from being mistaken for an empty history
// and then persisted over the user's existing recent-game list.
static bool recentLoadSafe = false;

// SD-card paths are case-insensitive; treat differently-cased spellings of
// the same ROM as one history entry. Keep this ASCII-only for FAT filenames.
static bool recentPathEquals(const char *a, const char *b) {
  if (!a || !b) return a == b;
  while (*a && *b) {
    unsigned char ca = (unsigned char)*a++;
    unsigned char cb = (unsigned char)*b++;
    if (ca >= 'A' && ca <= 'Z') ca = (unsigned char)(ca + ('a' - 'A'));
    if (cb >= 'A' && cb <= 'Z') cb = (unsigned char)(cb + ('a' - 'A'));
    if (ca != cb) return false;
  }
  return *a == *b;
}
static bool recentLoad() {
  recentCount = 0;
  recentLoadSafe = false;
  if (!sdMount()) return false;
  File f = SD.open("/system/recents.txt", FILE_READ);
  // A power loss during the temp-file rename can leave only the backup.
  // Restore it before giving up so the user does not lose saved entries.
  if (!f && SD.exists("/system/recents.bak")) {
    if (SD.rename("/system/recents.bak", "/system/recents.txt"))
      f = SD.open("/system/recents.txt", FILE_READ);
    if (!f) f = SD.open("/system/recents.bak", FILE_READ);
  }
  if (!f) {
    if (!SD.exists("/system/recents.txt") && !SD.exists("/system/recents.bak"))
      recentLoadSafe = true;  // A genuinely new card has an empty history.
    return recentLoadSafe;
  }
  const size_t expectedBytes = f.size();
  bool malformed = false;
  // Track valid entries beyond the visible limit so a capped list cannot
  // erase older recent-game entries when saved back to the SD card.
  bool overflowed = false;
  // Consume the entire file even after the in-memory list reaches capacity.
  // Otherwise valid extra/duplicate lines leave the file position short of EOF
  // and are misclassified as an SD read failure.
  while (f.available()) {
    char line[ROM_PATH_LEN]; size_t n = f.readBytesUntil('\n', line, sizeof(line)-1); line[n] = 0;
    // Reject embedded NUL bytes: otherwise a corrupted line such as
    // "/roms/nes/game.nes\\0junk" is silently accepted as a different ROM path.
    if (memchr(line, '\0', n) != nullptr) { malformed = true; continue; }
    // readBytesUntil() may stop at the buffer limit without consuming the
    // rest of an oversized line. Drain it so a tail fragment cannot become
    // a separate, misleading recent-game path.
    if (n == sizeof(line)-1 && f.available() && f.peek() != '\n' && f.peek() != '\r') {
      malformed = true;
      while (f.available()) { int c = f.read(); if (c < 0 || c == '\n') break; }
      continue;
    }
    // Text editors may add a UTF-8 BOM to the first line. Strip it so the
    // first saved ROM path is not silently discarded as an invalid path.
    if (recentCount == 0 && n >= 3 &&
        (uint8_t)line[0] == 0xEF && (uint8_t)line[1] == 0xBB && (uint8_t)line[2] == 0xBF) {
      memmove(line, line + 3, n - 3);
      n -= 3;
      line[n] = 0;
    }
    while (n && (line[n-1]=='\r' || line[n-1]==' ')) line[--n] = 0;
    if (!n) continue;  // Ignore harmless blank lines.
    if (strncasecmp(line, "/roms/", 6) != 0 || n >= sizeof(recentPaths[0])) {
      // A non-empty line outside the ROM directory is not a valid ROM path and may be a
      // truncated/corrupt entry. Do not silently drop it and later overwrite
      // the history file with only the entries that happened to parse.
      malformed = true;
      continue;
    }
    bool duplicate = false;
    for (int i=0; i<recentCount; ++i) if (recentPathEquals(recentPaths[i], line)) { duplicate=true; break; }
    if (!duplicate) {
      if (recentCount < RECENTS_MAX)
        strlcpy(recentPaths[recentCount++], line, sizeof(recentPaths[0]));
      else
        overflowed = true;
    }
  }
  const size_t consumedBytes = f.position();
  f.close();
  if (consumedBytes != expectedBytes) {
    recentCount = 0;
    Serial.printf("RECENTS: incomplete read (%u/%u bytes); refusing to overwrite history\n",
                  (unsigned)consumedBytes, (unsigned)expectedBytes);
    return false;
  }
  if (malformed) {
    // Keep readable entries available for display, but block recentSave():
    // rewriting this partial view would permanently discard damaged entries.
    Serial.println("RECENTS: malformed path data; refusing to overwrite history");
    return false;
  }
  if (overflowed) {
    // The menu displays only RECENTS_MAX entries; never overwrite the full
    // history file with this incomplete in-memory view.
    Serial.printf("RECENTS: more than %d entries; refusing to overwrite history\n", RECENTS_MAX);
    return false;
  }
  recentLoadSafe = true;
  return true;
}
static bool recentSave() {
  if (!recentLoadSafe) return false;
  // Keep the previous history until the replacement file is fully written.
  // Removing recents.txt before rename meant a failed rename silently erased
  // the user's entire recent-games list.
  if (!sdMount()) return false;
  SD.mkdir("/system");
  const char *tmp = "/system/recents.tmp";
  const char *dst = "/system/recents.txt";
  const char *bak = "/system/recents.bak";
  // FILE_WRITE may append on Arduino SD. Do not continue if a stale
  // temporary file cannot be removed; otherwise old entries can be retained.
  if (SD.exists(tmp) && !SD.remove(tmp)) {
    Serial.println("RECENTS: cannot clear temporary file");
    return false;
  }

  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return false;
  bool written = true;
  for (int i=0; i<recentCount; ++i) {
    const size_t len = strlen(recentPaths[i]);
    if (f.print(recentPaths[i]) != len || f.write('\n') != 1) {
      written = false;
      break;
    }
  }
  f.flush();
  f.close();
  if (!written) {
    SD.remove(tmp);
    return false;
  }

  // Rotate the old file out of the way, then roll it back if installing the
  // new file fails. SD.rename() does not reliably replace existing files.
  const bool hadOld = SD.exists(dst);
  // Do not discard a recoverable backup when the main file is already
  // missing (for example after an interrupted earlier save).
  const char *bakPrev = "/system/recents.bak.prev";
  const bool hadBackup = SD.exists(bak);
  // Preserve the last recovery copy until the current primary is moved aside.
  SD.remove(bakPrev);
  if (hadBackup && !SD.rename(bak, bakPrev)) { SD.remove(tmp); return false; }
  if (hadOld && !SD.rename(dst, bak)) {
    if (hadBackup) SD.rename(bakPrev, bak);
    SD.remove(tmp);
    return false;
  }
  if (!SD.rename(tmp, dst)) {
    if (hadOld) SD.rename(bak, dst);
    if (hadBackup) SD.rename(bakPrev, bak);
    SD.remove(tmp);
    return false;
  }
  // Only retire recovery copies after the replacement is safely installed.
  SD.remove(bakPrev);
  SD.remove(bak);
  return true;
}
static void recentRecord(const char *path) {
  // Do not store a truncated path that could point to a different ROM.
  if (!path || path[0] != '/' || strlen(path) >= sizeof(recentPaths[0]) || !sdMount()) return;
  if (!recentLoad()) return;
  // Keep the in-memory list consistent with the SD card if the atomic save fails.
  char previousPaths[RECENTS_MAX][ROM_PATH_LEN];
  const int previousCount = recentCount;
  memcpy(previousPaths, recentPaths, sizeof(previousPaths));
  int found=-1;
  for (int i=0; i<recentCount; ++i) if (recentPathEquals(recentPaths[i], path)) { found=i; break; }
  if (found >= 0) for (int i=found; i>0; --i) strlcpy(recentPaths[i], recentPaths[i-1], sizeof(recentPaths[i]));
  else {
    int last = recentCount < RECENTS_MAX ? recentCount++ : RECENTS_MAX-1;
    for (int i=last; i>0; --i) strlcpy(recentPaths[i], recentPaths[i-1], sizeof(recentPaths[i]));
  }
  strlcpy(recentPaths[0], path, sizeof(recentPaths[0]));
  if (!recentSave()) {
    recentCount = previousCount;
    memcpy(recentPaths, previousPaths, sizeof(recentPaths));
  }
}
static int recentSystemForPath(const char *path) {
  if (!path) return -1;
  const size_t pathLen = strlen(path);
  for (int i=0; i<SYS_COUNT; ++i) {
    const size_t n = strlen(sysDir[i]);
    // Check the length before indexing: malformed or stale short paths must
    // not make the case-insensitive prefix comparison read past the NUL.
    if (pathLen <= n) continue;
    // FAT paths are case-insensitive, so a path saved with different letter
    // casing must still resolve to its system folder when opened from Recents.
    size_t j = 0;
    while (j < n) {
      unsigned char a = (unsigned char)path[j];
      unsigned char b = (unsigned char)sysDir[i][j];
      if (a >= 'A' && a <= 'Z') a = (unsigned char)(a + ('a' - 'A'));
      if (b >= 'A' && b <= 'Z') b = (unsigned char)(b + ('a' - 'A'));
      if (a != b) break;
      ++j;
    }
    if (j != n || path[n] != '/') continue;
    // Recent entries are ROMs from the system folder root, never directories.
    // Reject malformed paths so a stale entry cannot select the wrong core.
    const char *name = path + n + 1;
    if (!name[0] || strchr(name, '/') || strchr(name, '\\') ||
        !strcmp(name, ".") || !strcmp(name, "..")) continue;
    return i;
  }
  return -1;
}
static int recentMenu() {
  recentLoad();
  int sel=0, top=0; const int visible = max(1, (LCD_H-58)/18);
  uint32_t t0=millis(); do { readPad(); delay(10); } while (padState && millis()-t0<1200);
  uint8_t prev=padState; uint32_t nextRep=0;
  bool dirty = true;
  for (;;) {
    readPad(); bleTick(); uint8_t s=padState, pressed=s & ~prev; prev=s; uint32_t now=millis();
    if (pressed & 0x02) return -1;
    if (menuConfirmPressed(pressed, 11) && recentCount>0) {
      int sys=recentSystemForPath(recentPaths[sel]);
      if (sys>=0) { strlcpy(recentLaunchPath,recentPaths[sel],sizeof(recentLaunchPath)); return sys; }
    }
    int step=0;
    if (pressed & 0x50) { step=-1; nextRep=now+350; }
    else if (pressed & 0xA0) { step=1; nextRep=now+350; }
    else if ((s&0x50) && (int32_t)(now-nextRep)>=0) { step=-1; nextRep=now+90; }
    else if ((s&0xA0) && (int32_t)(now-nextRep)>=0) { step=1; nextRep=now+90; }
    if (step && recentCount) {
      const int oldSel = sel, oldTop = top;
      sel=(sel+step+recentCount)%recentCount;
      if(sel<top)top=sel;
      if(sel>=top+visible)top=sel-visible+1;
      if (sel != oldSel || top != oldTop) dirty = true;
    }
    if (dirty) {
    lcdFill(0,0,LCD_W,LCD_H,uiBg());
    lcdTextRow(0,22,"RECENT GAMES",2,uiWhite(),uiPanel()); lcdFill(0,22,LCD_W,2,uiAccent());
    if (!recentCount) lcdTextRow(LCD_H/2-8,18,"NO RECENT GAMES",1,uiMuted(),uiBg(),(LCD_W-textW("NO RECENT GAMES",1))/2);
    for (int i=top; i<recentCount && i<top+visible; ++i) {
      const char *base=strrchr(recentPaths[i],'/'); base=base?base+1:recentPaths[i];
      char label[48]; strlcpy(label,base,sizeof(label));
      const int y=27+(i-top)*18; uint16_t bg=i==sel?uiBlue():uiPanel();
      lcdTextRow(y,17,label,1,i==sel?uiWhite():uiMuted(),bg);
      if(i==sel) lcdFill(0,y,3,17,uiAccent());
    }
    lcdTextRow(LCD_H-18,18,"A PLAY   B BACK",1,uiMuted(),uiPanel());
    dirty = false;
    }
    delay(16);
  }
}
