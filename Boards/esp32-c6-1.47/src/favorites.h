// Persistent favorites and Favorites menu. Included by menus.h after shared UI helpers.
#pragma once

// Persistent favorites. Full ROM paths are stored one per line in /system/favorites.txt.
#define FAVORITES_MAX 48
#define FAVORITE_PATH_LEN 192
static char favoritePaths[FAVORITES_MAX][FAVORITE_PATH_LEN];
static int favoriteCount = 0;

static void favoriteLoad() {
  favoriteCount = 0;
  if (!sdMount()) return;
  File f = SD.open("/system/favorites.txt", FILE_READ);
  // A power loss during the temp-file rename can leave only the backup.
  // Restore it before giving up so the user does not lose saved entries.
  if (!f && SD.exists("/system/favorites.bak")) {
    if (SD.rename("/system/favorites.bak", "/system/favorites.txt"))
      f = SD.open("/system/favorites.txt", FILE_READ);
    if (!f) f = SD.open("/system/favorites.bak", FILE_READ);
  }
  if (!f) return;
  while (f.available() && favoriteCount < FAVORITES_MAX) {
    char line[FAVORITE_PATH_LEN];
    size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = 0;
    while (n && (line[n-1] == '\r' || line[n-1] == ' ')) line[--n] = 0;
    if (!n || line[0] != '/') continue;
    bool duplicate = false;
    for (int i=0; i<favoriteCount; ++i) {
      if (!strcmp(favoritePaths[i], line)) { duplicate = true; break; }
    }
    if (!duplicate) strlcpy(favoritePaths[favoriteCount++], line, sizeof(favoritePaths[0]));
  }
  f.close();
}

static bool favoriteSave() {
  if (!sdMount()) return false;
  SD.mkdir("/system");
  const char *tmp = "/system/favorites.tmp";
  const char *dst = "/system/favorites.txt";
  const char *bak = "/system/favorites.bak";
  SD.remove(tmp);
  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return false;
  bool written = true;
  for (int i=0; i<favoriteCount; ++i) {
    const size_t len = strlen(favoritePaths[i]);
    if (f.print(favoritePaths[i]) != len || f.write('\n') != 1) { written = false; break; }
  }
  f.flush();
  f.close();
  if (!written) { SD.remove(tmp); return false; }
  const bool hadOld = SD.exists(dst);
  // Preserve an existing backup if the primary file is missing. It may be
  // the only recoverable copy left by a previous interrupted save.
  if (hadOld) {
    SD.remove(bak);
    if (!SD.rename(dst, bak)) { SD.remove(tmp); return false; }
  }
  if (!SD.rename(tmp, dst)) {
    if (hadOld) SD.rename(bak, dst);
    SD.remove(tmp);
    return false;
  }
  // The new primary is installed; the older backup can now be retired.
  SD.remove(bak);
  return true;
}

static int favoriteIndex(const char *path) {
  if (!path) return -1;
  for (int i=0; i<favoriteCount; ++i) if (!strcmp(favoritePaths[i], path)) return i;
  return -1;
}

static bool favoriteToggle(const char *path) {
  if (!path || path[0] != '/') return false;
  const int found = favoriteIndex(path);
  if (found >= 0) {
    for (int i=found; i<favoriteCount-1; ++i)
      strlcpy(favoritePaths[i], favoritePaths[i+1], sizeof(favoritePaths[i]));
    --favoriteCount;
  } else {
    if (favoriteCount >= FAVORITES_MAX) return false;
    strlcpy(favoritePaths[favoriteCount++], path, sizeof(favoritePaths[0]));
  }
  return favoriteSave();
}



static int favoriteSystemForPath(const char *path) {
  if (!path) return -1;
  for (int i=0; i<SYS_COUNT; ++i) {
    const size_t n = strlen(sysDir[i]);
    if (!strncmp(path, sysDir[i], n) && path[n]=='/') return i;
  }
  return -1;
}

static int favoritesMenu() {
  favoriteLoad();
  int sel = 0, top = 0;
  const int visible = max(1, (LCD_H - 58) / 18);
  uint32_t t0 = millis(); do { readPad(); delay(10); } while (padState && millis() - t0 < 1200);
  uint8_t prev = padState;
  uint16_t prevBleButtons = bleBtn;
  uint32_t nextRep = 0;
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev; prev = s;
    const uint16_t buttons = bleBtn;
    const bool yPressed = (buttons & XB_Y) && !(prevBleButtons & XB_Y);
    prevBleButtons = buttons;
    const uint32_t now = millis();
    if (pressed & 0x02) return -1;
    if (yPressed && favoriteCount > 0) {
      for (int i=sel; i<favoriteCount-1; ++i)
        strlcpy(favoritePaths[i], favoritePaths[i+1], sizeof(favoritePaths[i]));
      --favoriteCount;
      if (sel >= favoriteCount && sel > 0) --sel;
      if (top > sel) top = sel;
      favoriteSave();
      dirty = true;
    }
    if (menuConfirmPressed(pressed, 5) && favoriteCount > 0) {
      const int sys = favoriteSystemForPath(favoritePaths[sel]);
      if (sys >= 0) {
        strlcpy(recentLaunchPath, favoritePaths[sel], sizeof(recentLaunchPath));
        return sys;
      }
    }
    int step = 0;
    if (pressed & 0x50) { step = -1; nextRep = now + 350; }
    else if (pressed & 0xA0) { step = 1; nextRep = now + 350; }
    else if ((s & 0x50) && (int32_t)(now-nextRep) >= 0) { step = -1; nextRep = now + 90; }
    else if ((s & 0xA0) && (int32_t)(now-nextRep) >= 0) { step = 1; nextRep = now + 90; }
    if (step && favoriteCount) {
      const int oldSel = sel, oldTop = top;
      sel = (sel + step + favoriteCount) % favoriteCount;
      if (sel < top) top = sel;
      if (sel >= top + visible) top = sel - visible + 1;
      if (sel != oldSel || top != oldTop) dirty = true;
    }
    if (dirty) {
      lcdFill(0, 0, LCD_W, LCD_H, uiBg());
      lcdTextRow(0, 22, "FAVORITES", 2, uiWhite(), uiPanel());
      lcdFill(0, 22, LCD_W, 2, uiAccent());
      if (!favoriteCount) {
        lcdTextRow(LCD_H/2-12, 18, "NO FAVORITES YET", 1, uiMuted(), uiBg(),
                   (LCD_W-textW("NO FAVORITES YET",1))/2);
        lcdTextRow(LCD_H/2+8, 18, "PRESS Y ON A GAME", 1, uiMuted(), uiBg(),
                   (LCD_W-textW("PRESS Y ON A GAME",1))/2);
      }
      for (int i=top; i<favoriteCount && i<top+visible; ++i) {
        const char *base = strrchr(favoritePaths[i], '/'); base = base ? base+1 : favoritePaths[i];
        char label[48]; strlcpy(label, base, sizeof(label));
        const int y = 27 + (i-top)*18;
        const uint16_t bg = i == sel ? uiBlue() : uiPanel();
        lcdTextRow(y, 17, label, 1, i==sel ? uiWhite() : uiMuted(), bg);
        if (i == sel) lcdFill(0, y, 3, 17, uiAccent());
      }
      lcdTextRow(LCD_H-18, 18, "A PLAY  Y REMOVE  B BACK", 1, uiMuted(), uiPanel());
      dirty = false;
    }
    delay(16);
  }
}
