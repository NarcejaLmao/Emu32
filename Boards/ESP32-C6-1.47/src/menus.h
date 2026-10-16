// Menu rendering and navigation. SD/ROM discovery helpers are in sd_rom.h.
#pragma once

static int textW(const char *s, int scale) { return (int)strlen(s) * 4 * scale - scale; }

static void menuDraw(int sel, int top, const char *status) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%s %d/%d%s", sysShort[curSys], sel + 1, romCount, status);
  lcdTextRow(0, 18, buf, 2, col565(255, 255, 255), col565(0, 60, 120));
  for (int i = 0; i < MENU_ROWS; i++) {
    const int idx = top + i, y = MENU_TOP + i * MENU_ROW_H;
    if (idx >= romCount) { lcdTextRow(y, MENU_ROW_H, "", 2, 0, 0); continue; }
    char nm[40];
    baseName(romNames[idx], nm, sizeof(nm));
    if (idx == sel) lcdTextRow(y, MENU_ROW_H, nm, 2, col565(255, 255, 255), col565(40, 90, 210));
    else            lcdTextRow(y, MENU_ROW_H, nm, 2, col565(190, 190, 190), 0);
  }
}

// Game list of curSys. Returns the index of the chosen ROM, or -1 when the
// player pressed SELECT to go back to the main menu. Works with the GPIO
// buttons and with a paired BLE gamepad (BOOT starts pairing, same as in-game).
static int romMenu() {
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
    else if (src & 0x40) step = -MENU_ROWS; else if (src & 0x80) step = MENU_ROWS;
    if (step) {
      int ns = sel + step;
      if (fresh && (step == 1 || step == -1)) {      // wrap around on a fresh press
        if (ns < 0) ns = romCount - 1; else if (ns >= romCount) ns = 0;
      }
      ns = constrain(ns, 0, romCount - 1);
      if (ns != sel) { sel = ns; dirty = true; }
    }

    if (sel < top) top = sel;
    if (sel >= top + MENU_ROWS) top = sel - MENU_ROWS + 1;

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
  lcdFill(0, 0, LCD_W, LCD_H, 0);
  char header[40]; snprintf(header, sizeof(header), "SAVE MANAGER %d", saveManagerCount);
  lcdTextRow(2, 18, header, 2, col565(255,255,255), col565(0,60,120));
  if (!saveManagerCount) {
    lcdTextRow(LCD_H / 2 - 12, 18, "NO BATTERY SAVES", 2, col565(220,220,220), 0);
    lcdTextRow(LCD_H - 22, 14, "B = BACK", 2, col565(130,130,130), 0);
    return;
  }
  for (int row = 0; row < MENU_ROWS; ++row) {
    int idx = top + row; int y = MENU_TOP + row * MENU_ROW_H;
    if (idx >= saveManagerCount) { lcdTextRow(y, MENU_ROW_H, "", 2, 0, 0); continue; }
    char label[40]; strlcpy(label, saveManagerEntries[idx].label, sizeof(label));
    if (strlen(label) > 19) { label[18] = '~'; label[19] = 0; }
    lcdTextRow(y, MENU_ROW_H, label, 2,
      idx == sel ? col565(255,255,255) : col565(185,185,185),
      idx == sel ? col565(40,90,210) : 0);
  }
  lcdTextRow(LCD_H - 18, 14, status && status[0] ? status : "A = MANAGE  B = BACK", 2, col565(120,120,120), 0);
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
      lcdFill(0,0,LCD_W,LCD_H,0);
      lcdTextRow(12,18,"SAVE ACTIONS",2,col565(255,255,255),col565(0,60,120));
      char label[40]; strlcpy(label,e.label,sizeof(label)); if (strlen(label)>19) label[19]=0;
      lcdTextRow(42,16,label,2,col565(160,160,160),0);
      const char *items[3] = { "RESTORE BACKUP", "CREATE BACKUP", "BACK" };
      for (int i=0;i<3;i++) lcdTextRow(92+i*30,20,items[i],2,
        i==sel?col565(255,255,255):col565(180,180,180),i==sel?col565(40,90,210):0);
      lcdTextRow(LCD_H-20,14,"A = SELECT  B = BACK",2,col565(110,110,110),0);
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
    int step = (pressed & 0x10) ? -1 : (pressed & 0x20) ? 1 : (pressed & 0x40) ? -MENU_ROWS : (pressed & 0x80) ? MENU_ROWS : 0;
    if (step && saveManagerCount) { sel = constrain(sel + step, 0, saveManagerCount-1); dirty = true; }
    if (sel < top) top = sel; if (sel >= top + MENU_ROWS) top = sel - MENU_ROWS + 1;
    if (dirty) { saveManagerDraw(sel, top, ""); dirty = false; }
    delay(16);
  }
}

// ---- main menu: choose NES or GAME BOY ---------------------------------------
static void sysMenuDraw(int sel, int status) {
  const uint16_t white = col565(255,255,255), grey = col565(150,150,150);
  const uint16_t blue = col565(40,90,210), dark = col565(32,32,40);
  const bool land = LCD_W > LCD_H;
  static char failTxt[24];
  if (status == 3) snprintf(failTxt, sizeof failTxt, "FAIL: %s", bleFail);
  const char *st = (status == 1) ? "PAIRING..." : (status == 2) ? "BT PAD CONNECTED" : (status == 3) ? failTxt : "BOOT = PAIR BT PAD";
  const char *hint = "A = OK   B = BACK";
  const char *extra = "SAVE MANAGER";
  if (!land) {
    const int tw = textW("EMU32", 6);
    lcdTextRow(18,34,"EMU32",6,white,0,(LCD_W-tw)/2);
    lcdFill((LCD_W-tw)/2,58,tw,3,blue);
    lcdTextRow(67,14,"CHOOSE A SYSTEM",2,grey,0,(LCD_W-textW("CHOOSE A SYSTEM",2))/2);
    for (int i=0;i<SYS_COUNT+1;i++) {
      const int y = 86 + i*39;
      const char *name = i == SYS_COUNT ? extra : sysName[i];
      const char *sub = i == SYS_COUNT ? "BACKUP / RESTORE SAVES" : sysSub[i];
      const uint16_t bg = i==sel ? blue : dark, fg = i==sel ? white : grey;
      lcdTextRow(y,22,name, i==SYS_COUNT ? 2 : 3,fg,bg,(LCD_W-textW(name,i==SYS_COUNT?2:3))/2);
      lcdTextRow(y+22,12,sub,1,fg,bg,(LCD_W-textW(sub,1))/2);
      lcdFill(0,y+34,LCD_W,5,0);
    }
    lcdTextRow(292,12,st,1,grey,0,(LCD_W-textW(st,1))/2);
    lcdTextRow(307,12,hint,1,col565(90,90,90),0,(LCD_W-textW(hint,1))/2);
  } else {
    const int tw=textW("EMU32",3);
    lcdTextRow(1,20,"EMU32",3,white,0,(LCD_W-tw)/2);
    lcdFill((LCD_W-tw)/2,21,tw,2,blue);
    for (int i=0;i<SYS_COUNT+1;i++) {
      const int y=25+i*22; const char *name=i==SYS_COUNT?extra:sysName[i];
      const uint16_t bg=i==sel?blue:dark, fg=i==sel?white:grey;
      lcdTextRow(y,18,name,2,fg,bg,(LCD_W-textW(name,2))/2);
      lcdFill(0,y+18,LCD_W,4,0);
    }
    lcdTextRow(141,12,st,1,grey,0,(LCD_W-textW(st,1))/2);
    lcdTextRow(157,12,hint,1,col565(90,90,90),0,(LCD_W-textW(hint,1))/2);
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

