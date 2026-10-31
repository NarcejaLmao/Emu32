// Persistent recently-played history and Recent Games menu.
// Included by menus.h after shared UI helpers.
#pragma once

// Persistent recently-played list. Paths are stored one per line in /system/recents.txt.
static const int RECENTS_MAX = 12;
static char recentPaths[RECENTS_MAX][ROM_PATH_LEN];
static int recentCount = 0;
static char recentLaunchPath[ROM_PATH_LEN] = "";
static void recentLoad() {
  recentCount = 0;
  if (!sdMount()) return;
  File f = SD.open("/system/recents.txt", FILE_READ);
  // A power loss during the temp-file rename can leave only the backup.
  // Restore it before giving up so the user does not lose saved entries.
  if (!f && SD.exists("/system/recents.bak")) {
    if (SD.rename("/system/recents.bak", "/system/recents.txt"))
      f = SD.open("/system/recents.txt", FILE_READ);
    if (!f) f = SD.open("/system/recents.bak", FILE_READ);
  }
  if (!f) return;
  while (f.available() && recentCount < RECENTS_MAX) {
    char line[ROM_PATH_LEN]; size_t n = f.readBytesUntil('\n', line, sizeof(line)-1); line[n] = 0;
    // readBytesUntil() may stop at the buffer limit without consuming the
    // rest of an oversized line. Drain it so a tail fragment cannot become
    // a separate, misleading recent-game path.
    if (n == sizeof(line)-1 && f.available() && f.peek() != '\n' && f.peek() != '\r') {
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
    if (!n || line[0] != '/' || n >= sizeof(recentPaths[0])) continue;
    bool duplicate = false;
    for (int i=0; i<recentCount; ++i) if (!strcmp(recentPaths[i], line)) { duplicate=true; break; }
    if (!duplicate) strlcpy(recentPaths[recentCount++], line, sizeof(recentPaths[0]));
  }
  f.close();
}
static void recentSave() {
  // Keep the previous history until the replacement file is fully written.
  // Removing recents.txt before rename meant a failed rename silently erased
  // the user's entire recent-games list.
  if (!sdMount()) return;
  SD.mkdir("/system");
  const char *tmp = "/system/recents.tmp";
  const char *dst = "/system/recents.txt";
  const char *bak = "/system/recents.bak";
  SD.remove(tmp);

  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return;
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
    return;
  }

  // Rotate the old file out of the way, then roll it back if installing the
  // new file fails. SD.rename() does not reliably replace existing files.
  const bool hadOld = SD.exists(dst);
  // Do not discard a recoverable backup when the main file is already
  // missing (for example after an interrupted earlier save).
  if (hadOld) {
    SD.remove(bak);
    if (!SD.rename(dst, bak)) {
      SD.remove(tmp);
      return;
    }
  }
  if (!SD.rename(tmp, dst)) {
    if (hadOld) SD.rename(bak, dst);
    SD.remove(tmp);
    return;
  }
  // Only retire a stale backup after the replacement is safely installed.
  SD.remove(bak);
}
static void recentRecord(const char *path) {
  // Do not store a truncated path that could point to a different ROM.
  if (!path || path[0] != '/' || strlen(path) >= sizeof(recentPaths[0]) || !sdMount()) return;
  recentLoad();
  int found=-1;
  for (int i=0; i<recentCount; ++i) if (!strcmp(recentPaths[i], path)) { found=i; break; }
  if (found >= 0) for (int i=found; i>0; --i) strlcpy(recentPaths[i], recentPaths[i-1], sizeof(recentPaths[i]));
  else {
    int last = recentCount < RECENTS_MAX ? recentCount++ : RECENTS_MAX-1;
    for (int i=last; i>0; --i) strlcpy(recentPaths[i], recentPaths[i-1], sizeof(recentPaths[i]));
  }
  strlcpy(recentPaths[0], path, sizeof(recentPaths[0]));
  recentSave();
}
static int recentSystemForPath(const char *path) {
  if (!path) return -1;
  for (int i=0; i<SYS_COUNT; ++i) {
    const size_t n = strlen(sysDir[i]);
    if (strncmp(path, sysDir[i], n) != 0 || path[n] != '/') continue;
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
