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

// ---- main menu: choose NES or GAME BOY ---------------------------------------
static void sysMenuDraw(int sel, int status) {
  const uint16_t white = col565(255, 255, 255), grey = col565(150, 150, 150);
  const uint16_t blue  = col565(40, 90, 210),   dark = col565(32, 32, 40);
  const bool land = LCD_W > LCD_H;

  static char failTxt[24];
  if (status == 3) snprintf(failTxt, sizeof failTxt, "FAIL: %s", bleFail);
  const char *st = (status == 1) ? "PAIRING..." : (status == 2) ? "BT PAD CONNECTED" : (status == 3) ? failTxt : "BOOT = PAIR BT PAD";
  const char *hint = "A = OK   B = BACK";

  if (!land) {
    const int tw = textW("EMU32", 6);
    lcdTextRow(24, 34, "EMU32", 6, white, 0, (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 64, tw, 3, blue);
    lcdTextRow(78, 14, "CHOOSE A SYSTEM", 2, grey, 0, (LCD_W - textW("CHOOSE A SYSTEM", 2)) / 2);

    for (int i = 0; i < SYS_COUNT; i++) {                // 4 entries, 46 px pitch (GAME BOY COLOR needs scale 3)
      const int y = 100 + i * 46;
      const uint16_t bg = (i == sel) ? blue : dark, fg = (i == sel) ? white : grey;
      lcdTextRow(y,      26, sysName[i], 3, fg, bg, (LCD_W - textW(sysName[i], 3)) / 2);
      lcdTextRow(y + 26, 14, sysSub[i],  2, fg, bg, (LCD_W - textW(sysSub[i], 2)) / 2);
      lcdFill(0, y + 40, LCD_W, 6, 0);
    }
    lcdTextRow(290, 14, st, 2, grey, 0, (LCD_W - textW(st, 2)) / 2);
    lcdTextRow(306, 14, hint, 2, col565(90, 90, 90), 0, (LCD_W - textW(hint, 2)) / 2);
  } else {                                           // 320 x 172: compact, one line per system
    const int tw = textW("EMU32", 3);
    lcdTextRow(2, 20, "EMU32", 3, white, 0, (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 22, tw, 2, blue);
    for (int i = 0; i < SYS_COUNT; i++) {                // 4 entries, 26 px pitch
      const int y = 28 + i * 26;
      const uint16_t bg = (i == sel) ? blue : dark, fg = (i == sel) ? white : grey;
      lcdTextRow(y, 22, sysName[i], 3, fg, bg, (LCD_W - textW(sysName[i], 3)) / 2);
      lcdFill(0, y + 22, LCD_W, 4, 0);
    }
    lcdTextRow(136, 14, st, 2, grey, 0, (LCD_W - textW(st, 2)) / 2);
    lcdTextRow(152, 14, hint, 2, col565(90, 90, 90), 0, (LCD_W - textW(hint, 2)) / 2);
  }
}

// Returns SYS_NES, SYS_GB, SYS_GBC or SYS_A2600.
static int sysMenu() {
  int sel = constrain(sysSel, 0, SYS_COUNT - 1);
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

    if (pressed & 0x09) { sysSel = sel; return sel; }        // A (or START): select

    int step = (pressed & 0x50) ? -1 : (pressed & 0xA0) ? 1 : 0;   // UP/LEFT, DOWN/RIGHT
    if (step) { sel = (sel + step + SYS_COUNT) % SYS_COUNT; dirty = true; }

    const int status = bleStatus();
    if (dirty || status != lastStatus) {
      sysMenuDraw(sel, status);
      lastStatus = status; dirty = false;
    }
    delay(16);
  }
}

