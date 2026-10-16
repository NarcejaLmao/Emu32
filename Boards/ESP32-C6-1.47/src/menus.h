// Menu rendering and navigation. SD/ROM discovery helpers are in sd_rom.h.
#pragma once

static int textW(const char *s, int scale) { return (int)strlen(s) * 4 * scale - scale; }

static const uint16_t UI_BG       = 0x1010; // near-black navy, RGB565 byte-swapped via col565 helpers below
static uint16_t uiBg()      { return col565(10, 14, 24); }
static uint16_t uiPanel()   { return col565(22, 30, 46); }
static uint16_t uiPanel2()  { return col565(16, 22, 35); }
static uint16_t uiAccent()  { return col565(45, 220, 190); }
static uint16_t uiBlue()    { return col565(38, 100, 210); }
static uint16_t uiWhite()   { return col565(245, 249, 255); }
static uint16_t uiMuted()   { return col565(145, 160, 180); }
static uint16_t uiWarn()    { return col565(255, 190, 90); }
static int menuVisibleRows() { const int listTop = MENU_TOP + 20; const int available = (LCD_H - 18 - listTop) / MENU_ROW_H; return available > 0 ? available : 1; }

static void menuDraw(int sel, int top, const char *status) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  char title[48];
  snprintf(title, sizeof(title), "%s LIBRARY", sysShort[curSys]);
  lcdTextRow(0, 22, title, 2, uiWhite(), uiPanel());
  lcdFill(0, 22, LCD_W, 2, uiAccent());
  char info[48];
  snprintf(info, sizeof(info), "%d GAMES  |  %d/%d", romCount, romCount ? sel + 1 : 0, romCount);
  lcdTextRow(24, 15, info, 1, uiMuted(), uiBg());
  const int listTop = MENU_TOP + 20;
  for (int i = 0; i < menuVisibleRows(); i++) {
    const int idx = top + i, y = listTop + i * MENU_ROW_H;
    if (y >= LCD_H - 18) break;
    if (idx >= romCount) {
      lcdTextRow(y, MENU_ROW_H, "", 2, 0, uiBg());
      continue;
    }
    char nm[40]; baseName(romNames[idx], nm, sizeof(nm));
    const bool selected = idx == sel;
    lcdTextRow(y, MENU_ROW_H, nm, 2, selected ? uiWhite() : uiMuted(), selected ? uiBlue() : ((i & 1) ? uiPanel2() : uiBg()));
    if (selected) lcdFill(0, y, 3, MENU_ROW_H, uiAccent());
    else lcdFill(0, y + MENU_ROW_H - 1, LCD_W, 1, uiPanel());
  }
  char foot[48];
  if (status && status[0]) snprintf(foot, sizeof(foot), "A PLAY   B BACK   %s", status);
  else snprintf(foot, sizeof(foot), "A PLAY   B BACK   UP/DN MOVE");
  lcdTextRow(LCD_H - 18, 18, foot, 1, uiMuted(), uiPanel());
  lcdFill(0, LCD_H - 18, LCD_W, 1, uiAccent());
}

// Game list of curSys. Returns the index of the chosen ROM, or -1 when the
// player pressed SELECT to go back to the main menu. Works with the GPIO
// buttons and with a paired BLE gamepad (BOOT starts pairing, same as in-game).
static int romMenu() {
  // An empty library is a valid browser state: show instructions and wait for
  // an explicit back action instead of constraining a selection to -1.
  if (romCount <= 0) {
    uint32_t emptyWait = millis();
    do { readPad(); delay(10); } while (padState && millis() - emptyWait < 1500);
    uint8_t emptyPrev = padState;
    for (;;) {
      readPad();
      bleTick();
      const uint8_t pressed = padState & ~emptyPrev;
      emptyPrev = padState;
      if ((pressed & 0x02) || ((pressed & 0x04) && !(bleBtn & XB_RB))) return -1;
      lcdFill(0, 0, LCD_W, LCD_H, uiBg());
      char title[48];
      snprintf(title, sizeof(title), "%s LIBRARY", sysShort[curSys]);
      lcdTextRow(0, 22, title, 2, uiWhite(), uiPanel());
      lcdFill(0, 22, LCD_W, 2, uiAccent());
      lcdTextRow(24, 15, "0 GAMES  |  0/0", 1, uiMuted(), uiBg());
      lcdTextRow(LCD_H / 2 - 22, 24, "NO GAMES FOUND", 2, uiWhite(), uiPanel());
      lcdTextRow(LCD_H / 2 + 5, 18, "ADD ROMS TO:", 1, uiMuted(), uiBg());
      char pathLine[48];
      strlcpy(pathLine, sysDir[curSys], sizeof(pathLine));
      lcdTextRow(LCD_H / 2 + 22, 18, pathLine, 1, uiAccent(), uiBg());
      lcdTextRow(LCD_H - 18, 18, "B BACK   SELECT BACK", 1, uiMuted(), uiPanel());
      delay(50);
    }
  }

  int sel = constrain(menuSel[curSys], 0, romCount - 1), top = 0;
  lcdFill(0, 0, LCD_W, LCD_H, 0);

  uint32_t t0 = millis();                        // wait for buttons to be released
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1500);

  uint8_t  prev = padState;
  uint32_t nextRep = 0;
  int      lastStatus = -1;
  bool     dirty = true;

  for (;;) {
    readPad();
    bleTick();
    if (rotateHotkey()) dirty = true;                   // SELECT+RB: rotate (hold RB, then SELECT)
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    const uint32_t now = millis();

    // B (or SELECT, unless RB is held for the rotate hotkey): back to main menu
    if ((pressed & 0x02) || ((pressed & 0x04) && !(bleBtn & XB_RB))) { menuSel[curSys] = sel; return -1; }
    if (pressed & 0x09) { menuSel[curSys] = sel; return sel; }         // A (or START): play

    uint8_t src = 0; bool fresh = false;
    if (pressed & 0xF0)                                 { src = pressed; fresh = true; nextRep = now + 400; }
    else if ((s & 0xF0) && (int32_t)(now - nextRep) >= 0) { src = s; nextRep = now + 80; }   // auto-repeat
    int step = 0;
    if (src & 0x10) step = -1; else if (src & 0x20) step = 1;
    else if (src & 0x40) step = -menuVisibleRows(); else if (src & 0x80) step = menuVisibleRows();
    if (step) {
      int ns = sel + step;
      if (fresh && (step == 1 || step == -1)) {      // wrap around on a fresh press
        if (ns < 0) ns = romCount - 1; else if (ns >= romCount) ns = 0;
      }
      ns = constrain(ns, 0, romCount - 1);
      if (ns != sel) { sel = ns; dirty = true; }
    }

    if (sel < top) top = sel;
    if (sel >= top + menuVisibleRows()) top = sel - menuVisibleRows() + 1;

    const int status = bleStatus();
    if (dirty || status != lastStatus) {
      menuDraw(sel, top, status == 1 ? " PAIRING" : status == 2 ? " BT" : status == 3 ? " BT FAIL" : "");
      lastStatus = status; dirty = false;
    }
    delay(16);
  }
}

// ---- SD-card battery save manager -------------------------------------------
#define SAVE_MANAGER_MAX 48
#define SAVE_MANAGER_PATH 128
struct SaveManagerEntry { char path[SAVE_MANAGER_PATH]; char label[48]; };
static SaveManagerEntry saveManagerEntries[SAVE_MANAGER_MAX];
static int saveManagerCount = 0;

static bool saveManagerHasSuffix(const char *name, const char *suffix) {
  size_t n = strlen(name), z = strlen(suffix);
  return n > z && strcasecmp(name + n - z, suffix) == 0;
}

static void saveManagerScanDir(const char *dir, const char *systemLabel) {
  File folder = SD.open(dir);
  if (!folder || !folder.isDirectory()) { if (folder) folder.close(); return; }
  for (File f = folder.openNextFile(); f && saveManagerCount < SAVE_MANAGER_MAX; f = folder.openNextFile()) {
    if (!f.isDirectory()) {
      const char *name = f.name();
      const char *base = strrchr(name, '/'); base = base ? base + 1 : name;
      if (saveManagerHasSuffix(base, ".sav")) {
        SaveManagerEntry &e = saveManagerEntries[saveManagerCount];
        strlcpy(e.path, dir, sizeof(e.path));
        strlcat(e.path, "/", sizeof(e.path));
        strlcat(e.path, base, sizeof(e.path));
        snprintf(e.label, sizeof(e.label), "%s %s", systemLabel, base);
        saveManagerCount++;
      }
    }
    f.close();
  }
  folder.close();
}

// Copy src to a verified temporary file, then atomically promote it to dst.
// Used only for the small save files, never for ROM streaming.
static bool saveManagerCopyVerified(const char *src, const char *dst) {
  char tmp[SAVE_MANAGER_PATH + 8];
  if (snprintf(tmp, sizeof(tmp), "%s.tmp", dst) >= (int)sizeof(tmp)) return false;
  File in = SD.open(src, FILE_READ);
  if (!in) return false;
  const size_t expected = in.size();
  if (!expected || expected > 256 * 1024) { in.close(); return false; }
  SD.remove(tmp);
  File out = SD.open(tmp, FILE_WRITE);
  if (!out) { in.close(); return false; }
  uint8_t buf[512]; size_t copied = 0;
  while (copied < expected) {
    size_t want = expected - copied; if (want > sizeof(buf)) want = sizeof(buf);
    size_t got = in.read(buf, want);
    if (!got || out.write(buf, got) != got) break;
    copied += got;
  }
  in.close(); out.flush(); out.close();
  File verify = SD.open(tmp, FILE_READ);
  bool ok = copied == expected && verify && verify.size() == expected;
  if (verify) verify.close();
  if (!ok) { SD.remove(tmp); return false; }
  SD.remove(dst);
  if (!SD.rename(tmp, dst)) { SD.remove(tmp); return false; }
  return true;
}

static bool saveManagerRestoreBackup(const char *path) {
  char bak[SAVE_MANAGER_PATH + 8], tmp[SAVE_MANAGER_PATH + 8], old[SAVE_MANAGER_PATH + 8];
  if (snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak) ||
      snprintf(tmp, sizeof(tmp), "%s.restore", path) >= (int)sizeof(tmp) ||
      snprintf(old, sizeof(old), "%s.old", path) >= (int)sizeof(old)) return false;
  // Copy the backup to a verified staging file first, so the source backup
  // remains intact until the replacement is safely staged.
  if (!saveManagerCopyVerified(bak, tmp)) return false;
  SD.remove(old);
  const bool hadMain = SD.exists(path);
  if (hadMain && !SD.rename(path, old)) { SD.remove(tmp); return false; }
  if (!SD.rename(tmp, path)) {
    if (hadMain) SD.rename(old, path);
    SD.remove(tmp); return false;
  }
  // Keep the former primary as the new backup after a successful restore.
  if (hadMain) { SD.remove(bak); SD.rename(old, bak); }
  else SD.remove(bak);
  return true;
}

static void saveManagerDraw(int sel, int top, const char *status) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  char header[40]; snprintf(header, sizeof(header), "SAVE MANAGER  %02d", saveManagerCount);
  lcdTextRow(0, 23, header, 2, uiWhite(), uiPanel());
  lcdFill(0, 23, LCD_W, 2, uiAccent());
  lcdTextRow(25, 14, "BATTERY SAVE FILES", 1, uiMuted(), uiBg());
  if (!saveManagerCount) {
    lcdTextRow(LCD_H / 2 - 20, 22, "NO SAVES FOUND", 2, uiWhite(), uiPanel());
    lcdTextRow(LCD_H / 2 + 5, 16, "PLAY A GAME FIRST", 1, uiMuted(), uiBg());
    lcdTextRow(LCD_H - 18, 18, "B BACK", 1, uiMuted(), uiPanel());
    return;
  }
  const int listTop = 43;
  for (int row = 0; row < menuVisibleRows(); ++row) {
    int idx = top + row, y = listTop + row * MENU_ROW_H;
    if (y >= LCD_H - 18) break;
    if (idx >= saveManagerCount) { lcdTextRow(y, MENU_ROW_H, "", 2, 0, uiBg()); continue; }
    char label[40]; strlcpy(label, saveManagerEntries[idx].label, sizeof(label));
    if (strlen(label) > (size_t)((LCD_W > LCD_H) ? 32 : 19)) label[(LCD_W > LCD_H) ? 31 : 18] = 0;
    const bool selected = idx == sel;
    lcdTextRow(y, MENU_ROW_H, label, 2, selected ? uiWhite() : uiMuted(), selected ? uiBlue() : ((row & 1) ? uiPanel2() : uiBg()));
    if (selected) lcdFill(0, y, 3, MENU_ROW_H, uiAccent());
    else lcdFill(0, y + MENU_ROW_H - 1, LCD_W, 1, uiPanel());
  }
  lcdTextRow(LCD_H - 18, 18, status && status[0] ? status : "A MANAGE   B BACK", 1, uiMuted(), uiPanel());
  lcdFill(0, LCD_H - 18, LCD_W, 1, uiAccent());
}

static void saveManagerActions(int entry) {
  SaveManagerEntry &e = saveManagerEntries[entry];
  char bak[SAVE_MANAGER_PATH + 8];
  if (snprintf(bak, sizeof(bak), "%s.bak", e.path) >= (int)sizeof(bak)) return;
  int sel = 0; uint8_t prev = padState; bool dirty = true;
  uint32_t t0 = millis(); do { readPad(); delay(10); } while (padState && millis()-t0 < 1000);
  prev = padState;
  for (;;) {
    readPad(); bleTick();
    uint8_t s = padState, pressed = s & ~prev; prev = s;
    if (rotateHotkey()) dirty = true;
    if (pressed & 0x02) return;
    if (pressed & 0x50) { sel = (sel + 2) % 3; dirty = true; }
    if (pressed & 0xA0) { sel = (sel + 1) % 3; dirty = true; }
    if (dirty) {
      lcdFill(0, 0, LCD_W, LCD_H, uiBg());
      lcdTextRow(0, 24, "SAVE OPTIONS", 2, uiWhite(), uiPanel());
      lcdFill(0, 24, LCD_W, 2, uiAccent());
      char label[40]; strlcpy(label, e.label, sizeof(label));
      if (strlen(label) > (size_t)((LCD_W > LCD_H) ? 32 : 19)) label[(LCD_W > LCD_H) ? 31 : 18] = 0;
      lcdTextRow(29, 20, label, 2, uiMuted(), uiBg());
      const char *items[3] = { "RESTORE BACKUP", "CREATE BACKUP", "BACK" };
      const int baseY = (LCD_H > LCD_W) ? 100 : 56;
      const int rowH = (LCD_H > LCD_W) ? 28 : 24;
      for (int i = 0; i < 3; i++) {
        int y = baseY + i * rowH;
        lcdTextRow(y, rowH, items[i], 2, i == sel ? uiWhite() : uiMuted(), i == sel ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel()));
        if (i == sel) lcdFill(0, y, 3, rowH, uiAccent());
      }
      lcdTextRow(LCD_H - 18, 18, "A SELECT   B BACK", 1, uiMuted(), uiPanel());
      dirty = false;
    }
    if (pressed & 0x09) {
      if (sel == 2) return;
      bool ok = false;
      if (sel == 0) ok = saveManagerRestoreBackup(e.path);
      else {
        if (SD.exists(e.path)) ok = saveManagerCopyVerified(e.path, bak);
      }
      showMessage(ok ? "SAVE OPERATION OK" : "SAVE OPERATION FAILED",
                  sel == 0 ? "BACKUP RESTORE" : "BACKUP CREATE",
                  ok ? col565(0,80,0) : col565(110,0,0));
      delay(1000); dirty = true;
    }
    delay(16);
  }
}

static void saveManagerMenu() {
  if (!sdMount()) { showMessage("NO SD CARD", "CHECK CARD", col565(110,0,0)); delay(1000); return; }
  sdInitFolders();
  saveManagerCount = 0;
  saveManagerScanDir("/saves/gb", "GB");
  saveManagerScanDir("/saves/gbc", "GBC");
  int sel = 0, top = 0; uint8_t prev = padState; bool dirty = true;
  uint32_t t0 = millis(); do { readPad(); delay(10); } while (padState && millis()-t0 < 1000);
  prev = padState;
  for (;;) {
    readPad(); bleTick();
    uint8_t s = padState, pressed = s & ~prev; prev = s;
    if (rotateHotkey()) dirty = true;
    if (pressed & 0x02) return;
    if (saveManagerCount && (pressed & 0x09)) { saveManagerActions(sel); dirty = true; }
    int step = (pressed & 0x10) ? -1 : (pressed & 0x20) ? 1 : (pressed & 0x40) ? -menuVisibleRows() : (pressed & 0x80) ? menuVisibleRows() : 0;
    if (step && saveManagerCount) { sel = constrain(sel + step, 0, saveManagerCount-1); dirty = true; }
    if (sel < top) top = sel; if (sel >= top + menuVisibleRows()) top = sel - menuVisibleRows() + 1;
    if (dirty) { saveManagerDraw(sel, top, ""); dirty = false; }
    delay(16);
  }
}

// ---- main menu: choose NES or GAME BOY ---------------------------------------
static void sysMenuDraw(int sel, int status) {
  const bool land = LCD_W > LCD_H;
  static char failTxt[24];
  if (status == 3) snprintf(failTxt, sizeof failTxt, "BT FAIL: %s", bleFail);
  const char *st = (status == 1) ? "PAIRING BLUETOOTH..." : (status == 2) ? "BLUETOOTH CONNECTED" : (status == 3) ? failTxt : "BOOT BUTTON: PAIR GAMEPAD";
  const char *extra = "SAVE MANAGER";
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  if (!land) {
    const int tw = textW("EMU32", 6);
    lcdTextRow(15, 36, "EMU32", 6, uiWhite(), uiBg(), (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 53, tw, 3, uiAccent());
    lcdTextRow(58, 16, "RETRO GAME STATION", 1, uiMuted(), uiBg(), (LCD_W - textW("RETRO GAME STATION", 1)) / 2);
    lcdTextRow(77, 15, "SELECT SYSTEM", 1, uiWarn(), uiPanel());
    for (int i = 0; i < SYS_COUNT + 1; i++) {
      const int y = 96 + i * 37;
      const char *name = i == SYS_COUNT ? extra : sysName[i];
      const char *sub = i == SYS_COUNT ? "BACKUP AND RESTORE" : sysSub[i];
      const uint16_t bg = i == sel ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel());
      const uint16_t fg = i == sel ? uiWhite() : uiMuted();
      lcdTextRow(y, 22, name, i == SYS_COUNT ? 2 : 3, fg, bg, (LCD_W - textW(name, i == SYS_COUNT ? 2 : 3)) / 2);
      lcdTextRow(y + 22, 12, sub, 1, fg, bg, (LCD_W - textW(sub, 1)) / 2);
      if (i == sel) lcdFill(0, y, 3, 34, uiAccent());
      lcdFill(0, y + 34, LCD_W, 3, uiBg());
    }
    lcdTextRow(286, 14, st, 1, status == 3 ? uiWarn() : uiMuted(), uiBg(), (LCD_W - textW(st, 1)) / 2);
    lcdTextRow(303, 15, "A SELECT   B BACK", 1, uiWhite(), uiPanel());
  } else {
    const int tw = textW("EMU32", 3);
    lcdTextRow(0, 21, "EMU32", 3, uiWhite(), uiBg(), (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 20, tw, 2, uiAccent());
    lcdTextRow(22, 14, "SELECT SYSTEM", 1, uiWarn(), uiPanel());
    for (int i = 0; i < SYS_COUNT + 1; i++) {
      const int y = 39 + i * 22;
      const char *name = i == SYS_COUNT ? extra : sysName[i];
      const uint16_t bg = i == sel ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel());
      lcdTextRow(y, 18, name, 2, i == sel ? uiWhite() : uiMuted(), bg, (LCD_W - textW(name, 2)) / 2);
      if (i == sel) lcdFill(0, y, 3, 18, uiAccent());
    }
    lcdTextRow(151, 10, st, 1, status == 3 ? uiWarn() : uiMuted(), uiBg(), (LCD_W - textW(st, 1)) / 2);
    lcdTextRow(161, 11, "A OK / B BACK", 1, uiWhite(), uiPanel());
  }
}

// Returns SYS_NES, SYS_GB, SYS_GBC or SYS_A2600.
static int sysMenu() {
  int sel = constrain(sysSel, 0, SYS_COUNT);
  lcdFill(0, 0, LCD_W, LCD_H, 0);

  uint32_t t0 = millis();                        // wait for buttons to be released
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1500);

  uint8_t prev = padState;
  int     lastStatus = -1;
  bool    dirty = true;

  for (;;) {
    readPad();
    bleTick();
    if (rotateHotkey()) dirty = true;                        // SELECT+RB: rotate the display
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;

    if (pressed & 0x09) {
      if (sel == SYS_COUNT) { saveManagerMenu(); dirty = true; }
      else { sysSel = sel; return sel; }
    }        // A (or START): select

    int step = (pressed & 0x50) ? -1 : (pressed & 0xA0) ? 1 : 0;   // UP/LEFT, DOWN/RIGHT
    if (step) { sel = (sel + step + SYS_COUNT + 1) % (SYS_COUNT + 1); dirty = true; }

    const int status = bleStatus();
    if (dirty || status != lastStatus) {
      sysMenuDraw(sel, status);
      lastStatus = status; dirty = false;
    }
    delay(16);
  }
}

