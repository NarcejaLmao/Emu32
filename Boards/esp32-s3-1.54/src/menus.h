// Menu rendering and navigation. SD/ROM discovery helpers are in sd_rom.h.
#pragma once

static int textW(const char *s, int scale) { return (int)strlen(s) * 4 * scale - scale; }

// Debounce menu confirm actions in addition to edge-detecting button presses.
// This protects destructive/file actions from noisy GPIO or BLE input snapshots.
static bool menuConfirmPressed(uint8_t pressed, uint8_t context = 0) {
  if (!(pressed & 0x09)) return false; // A or START
  // Debounce independently per interaction context. A confirmation in one
  // screen must not suppress a valid press immediately after entering another.
  static uint32_t lastConfirmMs[12] = {};
  static bool haveConfirmed[12] = {};
  if (context >= 12) context = 0;
  const uint32_t now = millis();
  if (haveConfirmed[context] && (uint32_t)(now - lastConfirmMs[context]) < 180) return false;
  lastConfirmMs[context] = now;
  haveConfirmed[context] = true;
  return true;
}

static const uint16_t UI_BG       = 0x1010; // near-black navy, RGB565 byte-swapped via col565 helpers below
static uint16_t uiBg()      { return col565(10, 14, 24); }
static uint16_t uiPanel()   { return col565(22, 30, 46); }
static uint16_t uiPanel2()  { return col565(16, 22, 35); }
static uint16_t uiAccent()  { return col565(45, 220, 190); }
static uint16_t uiBlue()    { return col565(38, 100, 210); }
static uint16_t uiWhite()   { return col565(245, 249, 255); }
static uint16_t uiMuted()   { return col565(145, 160, 180); }
static uint16_t uiWarn()    { return col565(255, 190, 90); }
// Layout classes: tall (172x320 style), wide (320x172 style) and square (240x240).
// Square uses the same side-by-side list + box art layout as wide, and its own main menu.
static bool uiWide() { return LCD_W >= LCD_H; }
static int menuLabelMax() { return max(8, LCD_W / 8 - 2); }   // scale-2 characters that fit on a row
static int menuListTop() { return uiWide() ? 42 : 119; }
static int menuListWidth() { return uiWide() ? LCD_W - 98 : LCD_W; }

// Fit ROM labels to the actual drawable row width. Landscape reserves the
// right-hand column for box art, so LCD_W alone overestimates available space.
static void menuFitLabel(char *label, size_t capacity, int rowWidth, int scale = 2, int textX = 8) {
  if (!label || capacity == 0 || rowWidth <= textX) return;
  int maxChars = (rowWidth - textX - 3 * scale) / (4 * scale) + 1;
  if (maxChars <= 0) { label[0] = '\0'; return; }
  // The display width can exceed the destination buffer; never write past it.
  if ((size_t)maxChars >= capacity) maxChars = (int)capacity - 1;
  const size_t len = strlen(label);
  if (len <= (size_t)maxChars) return;
  if (maxChars >= 4) {
    label[maxChars - 3] = '.';
    label[maxChars - 2] = '.';
    label[maxChars - 1] = '.';
    label[maxChars] = '\0';
  } else {
    label[maxChars] = '\0';
  }
}
static int menuVisibleRows() { const int listTop = menuListTop(); const int available = (LCD_H - 18 - listTop) / MENU_ROW_H; return available > 0 ? available : 1; }

// Draw a row into a clipped horizontal region. This lets landscape mode reserve
// a narrow right-hand column for selected-game box art without covering rows.
static void menuTextRowRegion(int x0, int y, int w, int h, const char *s, int scale,
                              uint16_t fg, uint16_t bg, int textX = 8) {
  if (w <= 0 || h <= 0) return;
  const int len = strlen(s), textH = 5 * scale, top = (h - textH) / 2;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x0, y, x0 + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int ly = 0; ly < h; ++ly) {
    for (int x = 0; x < w; ++x) fillBuf[x] = bg;
    if (ly >= top && ly < top + textH) {
      const int r = (ly - top) / scale;
      for (int i = 0; i < len; ++i) {
        const uint16_t g = glyph(s[i]);
        const int gx = textX + i * 4 * scale;
        if (gx + 3 * scale > w) break;
        if (!g) continue;
        for (int c = 0; c < 3; ++c)
          if ((g >> (14 - (r * 3 + c))) & 1)
            for (int k = 0; k < scale; ++k) fillBuf[gx + c * scale + k] = fg;
      }
    }
    SPI.writeBytes((uint8_t *)fillBuf, w * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// Compact battery status badge for the top-right corner. S3 Waveshare LCD boards
// route the battery divider to GPIO1 (VADC is approximately VBAT / 2). The C6
// board variant has no confirmed battery-sense pin in this firmware, so show --%.
#if defined(CONFIG_IDF_TARGET_ESP32S3) && __has_include("soc/usb_serial_jtag_reg.h")
#include "soc/soc.h"
#include "soc/usb_serial_jtag_reg.h"
#define UI_HAVE_USB_SOF 1
#endif

static int gBatMv = 0;   // last computed battery voltage (mV), 0 = unknown

// Battery sense configuration. If the percentage is wrong, open the serial monitor
// (115200) and look at the "BAT:" lines, then override these two values.
#ifndef BAT_ADC_PIN
#define BAT_ADC_PIN 1          // GPIO wired to the battery divider
#endif
#ifndef BAT_DIV_RATIO_X10
#define BAT_DIV_RATIO_X10 0    // 0 = auto-detect (2x or 3x), else 20 = VBAT/2, 30 = VBAT/3, 40 = VBAT/4...
#endif

static int uiBatteryPercent() {
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C6)
  static uint32_t lastRead = 0;
  static bool have = false;
  static int cached = -1;
  if (!have || millis() - lastRead > 5000) {
    have = true;
    lastRead = millis();
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);   // full 0..~3.1V range
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) { sum += analogReadMilliVolts(BAT_ADC_PIN); delayMicroseconds(200); }
    const uint32_t pinMv = sum / 16U;

    // Pick the divider ratio. A Li-ion cell is ~3.0-4.25 V, so a 1/2 divider puts the
    // pin at ~1.5-2.1 V and a 1/3 divider at ~1.0-1.4 V: the ranges don't overlap.
    uint32_t ratio10 = BAT_DIV_RATIO_X10;
    if (ratio10 == 0) ratio10 = (pinMv >= 1450) ? 20 : 30;
    const uint32_t mv = pinMv * ratio10 / 10U;
    gBatMv = (pinMv < 700) ? 0 : (int)mv;
    if (Serial.availableForWrite() > 64) Serial.printf("BAT: pin=%umV ratio=%u.%u vbat=%umV\n", (unsigned)pinMv,
                  (unsigned)(ratio10 / 10), (unsigned)(ratio10 % 10), (unsigned)mv);

    if (pinMv < 700 || mv > 4600) {
      cached = -1;        // pin floating/wrong or out of range: show --% rather than a fake 0
    } else {
      // Typical single-cell Li-ion discharge curve (approximate).
      static const uint16_t curveMv[]  = {3300, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200};
      static const uint8_t  curvePct[] = {   0,    5,   15,   30,   50,   65,   80,   92,  100};
      if (mv <= curveMv[0]) cached = 0;
      else if (mv >= curveMv[8]) cached = 100;
      else {
        int i = 0;
        while (i < 7 && mv >= curveMv[i + 1]) i++;
        cached = curvePct[i] + (int)((mv - curveMv[i]) * (curvePct[i + 1] - curvePct[i]) /
                                     (curveMv[i + 1] - curveMv[i]));
      }
    }
  }
  return cached;
#else
  return -1;
#endif
}

// True when USB power is present / the battery is on a charger.
// 1) USB: the ESP32-S3 USB controller counts host SOF frames (1 kHz) only while a host/charger
//    is connected, so a changing frame counter means the cable is plugged in.
// 2) Fallback: a Li-ion cell sitting at >= ~4.18 V is almost certainly on a charger.
static bool uiBatteryCharging() {
  static uint32_t lastCheck = 0, lastFrame = 0;
  static bool cached = false, have = false;
  const uint32_t now = millis();
  if (have && now - lastCheck < 1000) return cached;
  bool usb = false;
#ifdef UI_HAVE_USB_SOF
  // Compare the SOF frame counter with the value from the previous check (~1 s ago) instead of
  // busy-waiting with delay(3) inside the menu draw path. SOF runs at 1 kHz while a host is attached.
  const uint32_t f = REG_READ(USB_SERIAL_JTAG_FRAM_NUM_REG) & 0x7FF;
  usb = have && (f != lastFrame);
  lastFrame = f;
#endif
  have = true; lastCheck = now;
  uiBatteryPercent();   // make sure gBatMv is fresh
  cached = usb || gBatMv >= 4180;
  return cached;
}

// 7x8 lightning bolt, drawn to the left of the battery icon.
static void uiDrawBolt(int x, int y) {
  static const uint8_t rows[8] = { 0x06, 0x0C, 0x18, 0x3F, 0x0C, 0x18, 0x30, 0x60 };
  const uint16_t c = col565(255, 214, 10);
  for (int r = 0; r < 8; r++) {
    int col = 0;
    while (col < 7) {
      if ((rows[r] >> (6 - col)) & 1) {
        int run = 1;
        while (col + run < 7 && ((rows[r] >> (6 - col - run)) & 1)) run++;
        lcdFill(x + col, y + r, run, 1, c);
        col += run;
      } else col++;
    }
  }
}

static void uiDrawBattery(int y = 5) {
  const int pct = uiBatteryPercent();
  const int x = LCD_W - 49;      // icon origin (top-left of the body)
  const int bodyW = 22, bodyH = 14;     // outer shell size
  const uint16_t outline = uiWhite();
  const uint16_t empty = uiPanel2();
  const uint16_t level = pct < 0 ? uiMuted() :
      (pct <= 15 ? col565(255, 76, 76) :
       (pct <= 35 ? col565(255, 190, 70) : col565(65, 220, 145)));

  // NOTE: lcdFill takes (x, y, w, h), not corner coordinates.
  // Shell: 2px outline around an inner cavity, plus a terminal nub on the right.
  lcdFill(x, y, bodyW, bodyH, outline);
  lcdFill(x + 2, y + 2, bodyW - 4, bodyH - 4, empty);
  lcdFill(x + bodyW, y + 4, 2, bodyH - 8, outline);

  // Inner fill area is 18 x 10 px. Fill width is proportional to the real percentage.
  const int innerX = x + 3, innerY = y + 3, innerW = bodyW - 6, innerH = bodyH - 6;
  if (pct < 0) {
    // Unknown battery sensing: muted dash, not a fake level.
    lcdFill(innerX + 4, innerY + innerH / 2 - 1, innerW - 8, 2, uiMuted());
  } else {
    int fillW = (pct * innerW + 50) / 100;           // rounded
    if (pct > 0 && fillW < 1) fillW = 1;              // keep a sliver visible when not empty
    if (fillW > innerW) fillW = innerW;
    if (fillW > 0) lcdFill(innerX, innerY, fillW, innerH, level);
  }

  char label[8];
  if (pct < 0) snprintf(label, sizeof(label), "--%%");
  else snprintf(label, sizeof(label), "%d%%", pct);
  // Right-aligned percentage next to the icon, drawn in a small clipped region
  // (lcdTextRow would overwrite a full-width row, including the icon).
  const int labelX = x + bodyW + 4, labelW = LCD_W - labelX;
  const int labelH = bodyH;
  const int textPx = textW(label, 1);
  menuTextRowRegion(labelX, y, labelW, labelH, label, 1,
                    pct >= 0 && pct <= 15 ? level : uiWhite(), uiBg(),
                    max(0, labelW - textPx - 1));

  // Lightning bolt to the left of the icon while plugged in / charging.
  if (uiBatteryCharging()) uiDrawBolt(x - 10, y + 3);
}

// ---- ROM list drawing ---------------------------------------------------------
// Repainting the whole 172x320 screen (plus an SD lookup for box art) on every cursor
// move is what made the menus feel sluggish. The list now keeps track of what is already
// on the panel and only repaints what changed:
//   * static chrome (title, accent rule)     -> once per menu entry / rotation
//   * info line + footer                     -> small strips, only when their text changes
//   * list rows                              -> only the old and new selected row, or the
//                                               whole list when it scrolls
//   * box art                                -> deferred until the cursor rests (see romMenu)
static struct { bool valid; int sel, top, rot, count; } menuShown = { false, -1, -1, -1, -1 };
static void menuInvalidate() { menuShown.valid = false; }

static void menuDrawRow(int idx, int row, int sel) {
  const int y = menuListTop() + row * MENU_ROW_H, listW = menuListWidth();
  if (y + MENU_ROW_H > LCD_H - 18) return;
  const bool selected = idx == sel;
  const uint16_t rowBg = selected ? uiBlue() : ((row & 1) ? uiPanel2() : uiBg());
  if (idx < 0 || idx >= romCount) { menuTextRowRegion(0, y, listW, MENU_ROW_H, "", 2, 0, uiBg()); return; }
  char nm[40]; baseName(romNames[idx], nm, sizeof(nm));
  menuFitLabel(nm, sizeof(nm), listW);
  menuTextRowRegion(0, y, listW, MENU_ROW_H, nm, 2, selected ? uiWhite() : uiMuted(), rowBg);
  // Accent bar / separator are folded in by painting just 3 px or 1 px more.
  if (selected) lcdFill(0, y, 3, MENU_ROW_H, uiAccent());
  else lcdFill(0, y + MENU_ROW_H - 1, listW, 1, uiPanel());
}

static void menuDrawBoxArt(int sel) {
  if (romCount > 0 && sel >= 0 && sel < romCount) {
    if (uiWide()) boxArtDraw(romNames[sel], LCD_W - 88, 43, 78, 88);
    else boxArtDraw(romNames[sel], (LCD_W - 72) / 2, 42, 72, 70);
  }
}

// Returns true when it did a full repaint (box art must then be redrawn by the caller).
static bool menuDraw(int sel, int top, const char *status) {
  const bool full = !menuShown.valid || menuShown.rot != (int)lcdRot || menuShown.count != romCount;
  const bool scrolled = full || menuShown.top != top;

  if (full) {
    lcdFill(0, 0, LCD_W, LCD_H, uiBg());
    char title[48];
    snprintf(title, sizeof(title), "%s LIBRARY", sysShort[curSys]);
    lcdTextRow(0, 22, title, 2, uiWhite(), uiPanel());
    lcdFill(0, 22, LCD_W, 2, uiAccent());
    uiDrawBattery();
  }
  if (full || menuShown.sel != sel) {
    char info[48];
    snprintf(info, sizeof(info), "%d GAMES  |  %d/%d", romCount, romCount ? sel + 1 : 0, romCount);
    lcdTextRow(24, 15, info, 1, uiMuted(), uiBg());
  }

  const int rows = menuVisibleRows();
  if (scrolled) {
    for (int i = 0; i < rows; i++) menuDrawRow(top + i, i, sel);
  } else if (menuShown.sel != sel) {
    const int o = menuShown.sel - top, n = sel - top;
    if (o >= 0 && o < rows) menuDrawRow(menuShown.sel, o, sel);
    if (n >= 0 && n < rows) menuDrawRow(sel, n, sel);
  }

  char foot[48];
  if (romListTruncated) snprintf(foot, sizeof(foot), "LOW RAM: LIST INCOMPLETE");
  else if (status && status[0]) snprintf(foot, sizeof(foot), "A PLAY   B BACK   %s", status);
  else snprintf(foot, sizeof(foot), "A PLAY   B BACK   UP/DN MOVE");
  static char lastFoot[48];
  if (full || strcmp(foot, lastFoot) != 0) {
    lcdTextRow(LCD_H - 18, 18, foot, 1, uiMuted(), uiPanel());
    lcdFill(0, LCD_H - 18, LCD_W, 1, uiAccent());
    strlcpy(lastFoot, foot, sizeof(lastFoot));
  }

  menuShown = { true, sel, top, (int)lcdRot, romCount };
  return full;
}

#include "history.h"
#include "favorites.h"

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
  uint16_t prevBleButtons = bleBtn;
  favoriteLoad();
  menuInvalidate();
  bool     artPending = true;                         // box art is drawn once the cursor rests
  uint32_t artDueMs = 0;
  int      artShownSel = -1;

  for (;;) {
    readPad();
    bleTick();
    if (rotateHotkey()) dirty = true;                   // SELECT+RB: rotate (hold RB, then SELECT)
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    const uint16_t bleButtons = bleBtn;
    const bool yPressed = (bleButtons & XB_Y) && !(prevBleButtons & XB_Y);
    prevBleButtons = bleButtons;
    const uint32_t now = millis();

    if (yPressed && sel >= 0 && sel < romCount) {
      char favoritePath[FAVORITE_PATH_LEN];
      snprintf(favoritePath, sizeof(favoritePath), "%s/%s", sysDir[curSys], romNames[sel]);
      if (!favoriteToggle(favoritePath)) {
        // Keep the current list intact if storage is full or the SD write fails.
        favoriteLoad();
      }
      dirty = true;
    }

    // B (or SELECT, unless RB is held for the rotate hotkey): back to main menu
    if ((pressed & 0x02) || ((pressed & 0x04) && !(bleBtn & XB_RB))) { menuSel[curSys] = sel; return -1; }
    if (menuConfirmPressed(pressed, 0)) { menuSel[curSys] = sel; return sel; } // A (or START): play

    uint8_t src = 0; bool fresh = false;
    if (pressed & 0xF0) {
      src = pressed & 0xF0;
      fresh = true;
      nextRep = now + 400;
    } else if ((s & 0xF0) && (int32_t)(now - nextRep) >= 0) {
      src = s & 0xF0;
      nextRep = now + 80;
    } // auto-repeat

    // Only navigate when exactly one direction is active. Simultaneous
    // directions are ambiguous (e.g. noisy input or a diagonal press), so
    // ignore them instead of applying a fixed-priority direction.
    if (src && (src & (src - 1))) src = 0;

    int step = 0;
    if (src == 0x10) step = -1;
    else if (src == 0x20) step = 1;
    else if (src == 0x40) step = -menuVisibleRows();
    else if (src == 0x80) step = menuVisibleRows();
    if (step) {
      int ns = sel + step;
      if (fresh && (step == 1 || step == -1)) {      // wrap around on a fresh press
        if (ns < 0) ns = romCount - 1; else if (ns >= romCount) ns = 0;
      }
      ns = constrain(ns, 0, romCount - 1);
      if (ns != sel) { sel = ns; dirty = true; artPending = true; artDueMs = now + 120; }
    }

    if (sel < top) top = sel;
    if (sel >= top + menuVisibleRows()) top = sel - menuVisibleRows() + 1;

    const int status = bleStatus();
    if (dirty || status != lastStatus) {
      char hint[24];
      char selectedPath[FAVORITE_PATH_LEN];
      selectedPath[0] = 0;
      if (sel >= 0 && sel < romCount)
        snprintf(selectedPath, sizeof(selectedPath), "%s/%s", sysDir[curSys], romNames[sel]);
      snprintf(hint, sizeof(hint), "%s", favoriteIndex(selectedPath) >= 0 ? "Y UNFAVORITE" : "Y FAVORITE");
      if (menuDraw(sel, top, hint)) { artPending = true; artShownSel = -1; artDueMs = now; }
      lastStatus = status; dirty = false;
    }
    // Box art needs SD lookups and possibly a PNG decode: only do it once the cursor has been
    // still for a moment, so scrolling through the list stays instant.
    if (artPending && (int32_t)(millis() - artDueMs) >= 0 && !(padState & 0xF0)) {
      if (artShownSel != sel) { menuDrawBoxArt(sel); artShownSel = sel; }
      artPending = false;
    }
    delay(8);
  }
}

// ---- SD-card battery save manager -------------------------------------------
#define SAVE_MANAGER_MAX 128
#define SAVE_MANAGER_PATH ROM_PATH_LEN
struct SaveManagerEntry { char path[SAVE_MANAGER_PATH]; char label[48]; };
// Allocated from PSRAM at boot (allocUiBuffers): ~42 KB of internal RAM saved.
static SaveManagerEntry *saveManagerEntries = nullptr;
static int saveManagerCount = 0;
static unsigned int saveManagerOverflowCount = 0;

static bool saveManagerHasSuffix(const char *name, const char *suffix) {
  size_t n = strlen(name), z = strlen(suffix);
  return n > z && strcasecmp(name + n - z, suffix) == 0;
}

static void saveManagerScanDir(const char *dir, const char *systemLabel) {
  File folder = SD.open(dir);
  if (!folder || !folder.isDirectory()) { if (folder) folder.close(); return; }
  for (File f = folder.openNextFile(); f; f = folder.openNextFile()) {
    if (!f.isDirectory()) {
      const char *name = f.name();
      const char *base = strrchr(name, '/'); base = base ? base + 1 : name;
      if (saveManagerHasSuffix(base, ".sav")) {
        if (saveManagerCount < SAVE_MANAGER_MAX) {
          SaveManagerEntry &e = saveManagerEntries[saveManagerCount];
          const int pathLen = snprintf(e.path, sizeof(e.path), "%s/%s", dir, base);
          if (pathLen > 0 && (size_t)pathLen < sizeof(e.path)) {
            snprintf(e.label, sizeof(e.label), "%s %s", systemLabel, base);
            saveManagerCount++;
          } else {
            // Never expose a truncated path: it could target a different save.
            e.path[0] = '\0';
            e.label[0] = '\0';
            saveManagerOverflowCount++;
          }
        } else {
          // Keep scanning so overflow is counted and never silently hidden.
          saveManagerOverflowCount++;
        }
      }
    }
    f.close();
  }
  folder.close();
}

// Copy src to a verified temporary file, then atomically promote it to dst.
// Used only for the small save files, never for ROM streaming.
static bool saveManagerCopyVerified(const char *src, const char *dst) {
  char tmp[SAVE_MANAGER_PATH + 8], old[SAVE_MANAGER_PATH + 8];
  if (!src || !dst || !src[0] || !dst[0] || strcmp(src, dst) == 0) return false;
  if (snprintf(tmp, sizeof(tmp), "%s.tmp", dst) >= (int)sizeof(tmp) ||
      snprintf(old, sizeof(old), "%s.old", dst) >= (int)sizeof(old)) return false;

  // Recover an interrupted prior replacement before starting another one.
  // If dst exists it is the committed version; otherwise preserve the old copy.
  if (SD.exists(old)) {
    if (!SD.exists(dst)) {
      if (!SD.rename(old, dst)) return false;
    } else {
      SD.remove(old);
    }
  }

  File in = SD.open(src, FILE_READ);
  if (!in) return false;
  const size_t expected = in.size();
  // A zero-byte save file is valid (for example, an intentionally empty
  // SRAM file). Copy and verify it like any other file instead of rejecting it.
  if (expected > 256 * 1024) { in.close(); return false; }
  SD.remove(tmp);
  File out = SD.open(tmp, FILE_WRITE);
  if (!out) { in.close(); return false; }
  uint8_t buf[512]; size_t copied = 0; bool writeFailed = false;
  while (copied < expected) {
    size_t want = expected - copied; if (want > sizeof(buf)) want = sizeof(buf);
    size_t got = in.read(buf, want);
    if (!got) break;
    size_t wrote = out.write(buf, got);
    if (wrote != got) { writeFailed = true; break; }
    copied += wrote;
  }
  in.close(); out.flush(); out.close();
  // A matching file length is not enough: same-size corruption during an SD
  // write would otherwise be promoted as a successful save. Compare every
  // copied byte against the source before replacing the destination.
  File sourceCheck = SD.open(src, FILE_READ);
  File verify = SD.open(tmp, FILE_READ);
  bool ok = copied == expected && sourceCheck && verify &&
            sourceCheck.size() == expected && verify.size() == expected;
  uint8_t sourceBuf[512], verifyBuf[512];
  size_t checked = 0;
  while (ok && checked < expected) {
    size_t want = expected - checked;
    if (want > sizeof(sourceBuf)) want = sizeof(sourceBuf);
    const size_t sourceRead = sourceCheck.read(sourceBuf, want);
    const size_t verifyRead = verify.read(verifyBuf, want);
    if (sourceRead != want || verifyRead != want ||
        memcmp(sourceBuf, verifyBuf, want) != 0) {
      ok = false;
      break;
    }
    checked += want;
  }
  if (sourceCheck) sourceCheck.close();
  if (verify) verify.close();
  if (!ok) {
    if (writeFailed || copied < expected) {
      sdSaveSetError("SD CARD FULL OR COPY FAILED");
      Serial.printf("SAVE MANAGER: copied %u/%u bytes; SD may be full\n", (unsigned)copied, (unsigned)expected);
    } else {
      sdSaveSetError("SAVE COPY VERIFY FAILED");
    }
    SD.remove(tmp); return false;
  }

  // Never delete dst before the replacement is ready. Move it aside, promote
  // the verified temporary file, and roll back if promotion fails.
  const bool hadDst = SD.exists(dst);
  if (hadDst && !SD.rename(dst, old)) { SD.remove(tmp); return false; }
  if (!SD.rename(tmp, dst)) {
    if (hadDst) SD.rename(old, dst);
    SD.remove(tmp);
    return false;
  }
  if (hadDst) SD.remove(old);
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
  char header[40];
  if (saveManagerOverflowCount) snprintf(header, sizeof(header), "SAVE MANAGER %d+", saveManagerCount);
  else snprintf(header, sizeof(header), "SAVE MANAGER  %02d", saveManagerCount);
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
    if (strlen(label) > (size_t)menuLabelMax()) label[menuLabelMax() - 1] = 0;
    const bool selected = idx == sel;
    lcdTextRow(y, MENU_ROW_H, label, 2, selected ? uiWhite() : uiMuted(), selected ? uiBlue() : ((row & 1) ? uiPanel2() : uiBg()));
    if (selected) lcdFill(0, y, 3, MENU_ROW_H, uiAccent());
    else lcdFill(0, y + MENU_ROW_H - 1, LCD_W, 1, uiPanel());
  }
  const char *footer = (status && status[0]) ? status : (saveManagerOverflowCount ? "LIST FULL: 128 MAX" : "A MANAGE   B BACK");
  lcdTextRow(LCD_H - 18, 18, footer, 1, uiMuted(), uiPanel());
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
    if (pressed & 0x50) { sel = (sel + 3) % 4; dirty = true; }
    if (pressed & 0xA0) { sel = (sel + 1) % 4; dirty = true; }
    if (dirty) {
      lcdFill(0, 0, LCD_W, LCD_H, uiBg());
      lcdTextRow(0, 24, "SAVE OPTIONS", 2, uiWhite(), uiPanel());
      lcdFill(0, 24, LCD_W, 2, uiAccent());
      char label[40]; strlcpy(label, e.label, sizeof(label));
      if (strlen(label) > (size_t)menuLabelMax()) label[menuLabelMax() - 1] = 0;
      lcdTextRow(29, 20, label, 2, uiMuted(), uiBg());
      const char *items[4] = { "RESTORE BACKUP", "CREATE BACKUP", "DELETE SAVE", "BACK" };
      const int baseY = (LCD_H > LCD_W) ? 88 : 48;
      const int rowH = (LCD_H > LCD_W) ? 25 : 21;
      for (int i = 0; i < 4; i++) {
        int y = baseY + i * rowH;
        lcdTextRow(y, rowH, items[i], 2, i == sel ? uiWhite() : uiMuted(), i == sel ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel()));
        if (i == sel) lcdFill(0, y, 3, rowH, uiAccent());
      }
      lcdTextRow(LCD_H - 18, 18, "A SELECT   B BACK", 1, uiMuted(), uiPanel());
      dirty = false;
    }
    if (menuConfirmPressed(pressed, 2)) {
      if (sel == 3) return;
      bool ok = false;
      if (sel == 0) {
        ok = saveManagerRestoreBackup(e.path);
      } else if (sel == 1) {
        if (SD.exists(e.path)) ok = saveManagerCopyVerified(e.path, bak);
      } else {
        // Require an explicit second confirmation before deleting save data.
        int confirm = 1; bool confirmDirty = true; uint8_t confirmPrev = padState;
        uint32_t confirmStart = millis();
        do { readPad(); bleTick(); delay(10); } while (padState && millis() - confirmStart < 1000);
        confirmPrev = padState;
        for (;;) {
          readPad(); bleTick();
          uint8_t cs = padState, cp = cs & ~confirmPrev; confirmPrev = cs;
          if (cp & 0x02) { confirm = 0; break; }
          if (cp & 0x50) { confirm = 0; confirmDirty = true; }
          if (cp & 0xA0) { confirm = 1; confirmDirty = true; }
          if (confirmDirty) {
            lcdFill(0, 0, LCD_W, LCD_H, uiBg());
            lcdTextRow(20, 24, "DELETE SAVE?", 2, uiWhite(), uiPanel());
            lcdTextRow(54, 20, e.label, 1, uiMuted(), uiBg());
            lcdTextRow(92, 24, "NO", 2, confirm == 0 ? uiWhite() : uiMuted(), confirm == 0 ? uiBlue() : uiPanel());
            lcdTextRow(122, 24, "YES, DELETE", 2, confirm == 1 ? uiWhite() : uiMuted(), confirm == 1 ? col565(110,0,0) : uiPanel());
            lcdTextRow(LCD_H - 18, 18, "A CONFIRM   B CANCEL", 1, uiMuted(), uiPanel());
            confirmDirty = false;
          }
          if (menuConfirmPressed(cp, 3)) {
            if (confirm == 1) {
              // Remove every recoverable generation. Battery-save loading can
              // fall back to .bak.prev, so leaving it behind would resurrect a
              // save that the user explicitly deleted. Remove backups first and
              // the primary last, preserving the primary if backup cleanup fails.
              char prev[SAVE_MANAGER_PATH + 16];
              const int prevPathLen = snprintf(prev, sizeof(prev), "%s.bak.prev", e.path);
              const bool prevPathOk = prevPathLen >= 0 && (size_t)prevPathLen < sizeof(prev);
              bool backupsRemoved = prevPathOk;
              if (backupsRemoved && SD.exists(prev) && !SD.remove(prev)) backupsRemoved = false;
              if (backupsRemoved && SD.exists(bak) && !SD.remove(bak)) backupsRemoved = false;
              ok = backupsRemoved && (!SD.exists(e.path) || SD.remove(e.path));
            } else {
              ok = false;
            }
            break;
          }
          delay(16);
        }
        if (!ok) { showMessage("DELETE CANCELLED", "OR DELETE FAILED", col565(80,45,0)); delay(700); }
      }
      if (sel != 2) {
        showMessage(ok ? "SAVE OPERATION OK" : "SAVE OPERATION FAILED",
                    sel == 0 ? "BACKUP RESTORE" : "BACKUP CREATE",
                    ok ? col565(0,80,0) : col565(110,0,0));
        delay(1000);
      }
      dirty = true;
    }
    delay(16);
  }
}

static void saveManagerMenu() {
  if (!saveManagerEntries) { showMessage("OUT OF MEMORY", "SAVE MANAGER OFF", col565(110,0,0)); delay(1000); return; }
  if (!sdMount()) { showMessage("NO SD CARD", "CHECK CARD", col565(110,0,0)); delay(1000); return; }
  sdInitFolders();
  saveManagerCount = 0;
  saveManagerOverflowCount = 0;
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
    if (saveManagerCount && menuConfirmPressed(pressed, 1)) {
      saveManagerActions(sel);
      // Actions can remove the selected save. Rebuild the list and clamp the
      // cursor before drawing or indexing entries again.
      saveManagerCount = 0;
      saveManagerOverflowCount = 0;
      saveManagerScanDir("/saves/gb", "GB");
      saveManagerScanDir("/saves/gbc", "GBC");
      if (saveManagerCount == 0) { sel = 0; top = 0; }
      else if (sel >= saveManagerCount) sel = saveManagerCount - 1;
      if (sel < 0) sel = 0;
      if (top > sel) top = sel;
      if (top < 0) top = 0;
      if (top >= saveManagerCount) top = saveManagerCount ? saveManagerCount - 1 : 0;
      dirty = true;
    }
    int step = (pressed & 0x10) ? -1 : (pressed & 0x20) ? 1 : (pressed & 0x40) ? -menuVisibleRows() : (pressed & 0x80) ? menuVisibleRows() : 0;
    if (step && saveManagerCount) { sel = constrain(sel + step, 0, saveManagerCount-1); dirty = true; }
    if (sel < top) top = sel; if (sel >= top + menuVisibleRows()) top = sel - menuVisibleRows() + 1;
    if (dirty) { saveManagerDraw(sel, top, ""); dirty = false; }
    delay(16);
  }
}

// ---- main menu: choose NES or GAME BOY ---------------------------------------
static int sysMenuVisibleRows() {
  // Keep system names at a readable, fixed size. Additional systems are
  // reached by scrolling instead of compressing every row to fit.
  if (LCD_W == LCD_H) return max(1, (210 - 42 + 2) / 28);   // square: all 6 entries fit
  if (LCD_W > LCD_H) return max(1, (151 - 39) / 22);
  return max(1, (276 - 96) / 37);
}

static void sysMenuDraw(int sel, int top, int status) {
  const bool land = LCD_W > LCD_H;
  static char failTxt[24];
  if (status == 3) snprintf(failTxt, sizeof failTxt, "BT FAIL: %s", bleFail);
  const char *st = (status == 1) ? "PAIRING BLUETOOTH..." : (status == 2) ? "BLUETOOTH CONNECTED" : (status == 3) ? failTxt : "BOOT BUTTON: PAIR GAMEPAD";
  const int itemCount = SYS_COUNT;
  const int visible = sysMenuVisibleRows();
  top = constrain(top, 0, max(0, itemCount - visible));
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  if (LCD_W == LCD_H) {
    // Square 240x240 layout: title 0-26, list 42-208, status 212, footer 224-240.
    const int tw = textW("EMU32", 4);
    lcdTextRow(2, 22, "EMU32", 4, uiWhite(), uiBg(), (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 24, tw, 2, uiAccent());
    lcdTextRow(27, 13, "SELECT SYSTEM", 1, uiWarn(), uiPanel());
    const int rowH = 26, rowStep = 28, listY = 42;
    for (int row = 0; row < visible; ++row) {
      const int idx = top + row;
      if (idx >= itemCount) break;
      const int y = listY + row * rowStep;
      const char *name = sysName[idx];
      const char *sub = sysSub[idx];
      const uint16_t bg = idx == sel ? uiBlue() : ((idx & 1) ? uiPanel2() : uiPanel());
      const uint16_t fg = idx == sel ? uiWhite() : uiMuted();
      lcdTextRow(y, 16, name, 2, fg, bg, (LCD_W - textW(name, 2)) / 2);
      lcdTextRow(y + 16, 10, sub, 1, fg, bg, (LCD_W - textW(sub, 1)) / 2);
      if (idx == sel) lcdFill(0, y, 3, rowH, uiAccent());
    }
    if (itemCount > visible) {
      const int trackY = listY, trackH = visible * rowStep - 2;
      const int thumbH = max(8, trackH * visible / itemCount);
      const int thumbY = trackY + (trackH - thumbH) * top / max(1, itemCount - visible);
      lcdFill(LCD_W - 4, trackY, 2, trackH, uiPanel2());
      lcdFill(LCD_W - 4, thumbY, 2, thumbH, uiAccent());
    }
    lcdTextRow(212, 10, st, 1, status == 3 ? uiWarn() : uiMuted(), uiBg(), (LCD_W - textW(st, 1)) / 2);
    lcdTextRow(224, 16, "A SELECT  UP/DN SCROLL", 1, uiWhite(), uiPanel());
  } else if (!land) {
    const int tw = textW("EMU32", 6);
    lcdTextRow(15, 36, "EMU32", 6, uiWhite(), uiBg(), (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 53, tw, 3, uiAccent());
    lcdTextRow(58, 16, "RETRO GAME STATION", 1, uiMuted(), uiBg(), (LCD_W - textW("RETRO GAME STATION", 1)) / 2);
    lcdTextRow(77, 15, "SELECT SYSTEM", 1, uiWarn(), uiPanel());
    const int rowH = 34, rowStep = 37, listY = 96;
    for (int row = 0; row < visible; ++row) {
      const int idx = top + row;
      if (idx >= itemCount) break;
      const int y = listY + row * rowStep;
      const char *name = sysName[idx];
      const char *sub = sysSub[idx];
      const uint16_t bg = idx == sel ? uiBlue() : ((idx & 1) ? uiPanel2() : uiPanel());
      const uint16_t fg = idx == sel ? uiWhite() : uiMuted();
      lcdTextRow(y, 22, name, 3, fg, bg, (LCD_W - textW(name, 3)) / 2);
      lcdTextRow(y + 22, 12, sub, 1, fg, bg, (LCD_W - textW(sub, 1)) / 2);
      if (idx == sel) lcdFill(0, y, 3, rowH, uiAccent());
    }
    // Scroll indicator shows position without shrinking the system rows.
    if (itemCount > visible) {
      const int trackY = 96, trackH = visible * rowStep - 3;
      const int thumbH = max(12, trackH * visible / itemCount);
      const int thumbY = trackY + (trackH - thumbH) * top / max(1, itemCount - visible);
      lcdFill(LCD_W - 4, trackY, 2, trackH, uiPanel2());
      lcdFill(LCD_W - 4, thumbY, 2, thumbH, uiAccent());
    }
    lcdTextRow(276, 10, st, 1, status == 3 ? uiWarn() : uiMuted(), uiBg(), (LCD_W - textW(st, 1)) / 2);
    lcdTextRow(303, 15, "A SELECT  UP/DN SCROLL", 1, uiWhite(), uiPanel());
  } else {
    const int tw = textW("EMU32", 3);
    lcdTextRow(0, 21, "EMU32", 3, uiWhite(), uiBg(), (LCD_W - tw) / 2);
    lcdFill((LCD_W - tw) / 2, 20, tw, 2, uiAccent());
    lcdTextRow(22, 14, "SELECT SYSTEM", 1, uiWarn(), uiPanel());
    const int rowStep = 22, rowH = 18, listY = 39;
    for (int row = 0; row < visible; ++row) {
      const int idx = top + row;
      if (idx >= itemCount) break;
      const int y = listY + row * rowStep;
      const char *name = sysName[idx];
      const uint16_t bg = idx == sel ? uiBlue() : ((idx & 1) ? uiPanel2() : uiPanel());
      lcdTextRow(y, rowH, name, 2, idx == sel ? uiWhite() : uiMuted(), bg, (LCD_W - textW(name, 2)) / 2);
      if (idx == sel) lcdFill(0, y, 3, rowH, uiAccent());
    }
    if (itemCount > visible) {
      const int trackY = 39, trackH = visible * rowStep - 2;
      const int thumbH = max(8, trackH * visible / itemCount);
      const int thumbY = trackY + (trackH - thumbH) * top / max(1, itemCount - visible);
      lcdFill(LCD_W - 4, trackY, 2, trackH, uiPanel2());
      lcdFill(LCD_W - 4, thumbY, 2, thumbH, uiAccent());
    }
    lcdTextRow(151, 10, st, 1, status == 3 ? uiWarn() : uiMuted(), uiBg(), (LCD_W - textW(st, 1)) / 2);
    lcdTextRow(161, 11, "A OK  UP/DN SCROLL", 1, uiWhite(), uiPanel());
  }
  // Draw the battery badge LAST: the title rows above are full-width and would
  // otherwise paint over it. On the tall layout it sits higher (y=1) so it stays
  // clear of the large EMU32 title that starts at y=18.
  uiDrawBattery(LCD_H > LCD_W ? 1 : 5);
}

// Returns the chosen system id (SYS_NES ... SYS_SMS), or -1.
static int gamesSystemMenu() {
  const int itemCount = SYS_COUNT;
  const int visible = sysMenuVisibleRows();
  int sel = constrain(sysSel, 0, itemCount - 1);
  int top = constrain(sel - visible + 1, 0, max(0, itemCount - visible));
  lcdFill(0, 0, LCD_W, LCD_H, 0);

  uint32_t t0 = millis();
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1500);
  uint8_t prev = padState;
  int lastStatus = -1;
  bool dirty = true;
  uint32_t nextRep = 0;

  for (;;) {
    readPad();
    bleTick();
    if (rotateHotkey()) { top = constrain(sel - visible + 1, 0, max(0, itemCount - visible)); dirty = true; }
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    const uint32_t now = millis();
    if (pressed & 0x02) return -1; // B returns to the top-level launcher.

    if (menuConfirmPressed(pressed, 10)) {
      sysSel = sel; return sel;
    }

    uint8_t src = 0;
    if (pressed & 0x50) { src = 0x10; nextRep = now + 350; }
    else if (pressed & 0xA0) { src = 0x20; nextRep = now + 350; }
    else if ((s & 0x50) && (int32_t)(now - nextRep) >= 0) { src = 0x10; nextRep = now + 90; }
    else if ((s & 0xA0) && (int32_t)(now - nextRep) >= 0) { src = 0x20; nextRep = now + 90; }

    int step = src == 0x10 ? -1 : src == 0x20 ? 1 : 0;
    if (step) {
      const int next = constrain(sel + step, 0, itemCount - 1);
      if (next != sel) { sel = next; dirty = true; }
      if (sel < top) { top = sel; dirty = true; }
      if (sel >= top + visible) { top = sel - visible + 1; dirty = true; }
    }

    const int status = bleStatus();
    if (dirty || status != lastStatus) {
      sysMenuDraw(sel, top, status);
      lastStatus = status;
      dirty = false;
    }
    delay(16);
  }
}

// ---- SD-card file explorer ---------------------------------------------------
// Browse the card, mark items with X (several allowed), and press Y for an action
// sheet: COPY / CUT / PASTE / DELETE / SELECT ALL / CLEAR SELECTION. With nothing
// marked, COPY / CUT / DELETE act on the highlighted item. X and Y need a
// Bluetooth pad; the wired/8-bit pad state has no spare bits for them.
#define FILE_EXPLORER_MAX_ENTRIES 48
#define FILE_EXPLORER_NAME_LEN 64
#define FILE_EXPLORER_PATH_LEN 192
struct FileExplorerEntry { char name[FILE_EXPLORER_NAME_LEN]; bool isDir; uint32_t size; };
static FileExplorerEntry *fileExplorerEntries = nullptr;   // allocated in allocUiBuffers()
static int fileExplorerCount = 0;
static bool fileExplorerTruncated = false;
static char fileExplorerPath[192] = "/";
static char fileExplorerStatus[48] = "";
static bool fileExplorerMarked[FILE_EXPLORER_MAX_ENTRIES];   // X-marked rows in the current folder
static int  fileExplorerMarkedCount = 0;
// Clipboard (filled by COPY / CUT, consumed by PASTE); names live on the heap.
static char  fileClipDir[192] = "";
static char *fileClipNames = nullptr;
static int   fileClipCount = 0;
static bool  fileClipCut = false;

static void fileExplorerScan() {
  fileExplorerCount = 0;
  memset(fileExplorerMarked, 0, sizeof(fileExplorerMarked));   // marks never outlive a scan
  fileExplorerMarkedCount = 0;
  fileExplorerTruncated = false;
  if (strcmp(fileExplorerPath, "/") != 0) {
    strlcpy(fileExplorerEntries[fileExplorerCount].name, "..", sizeof(fileExplorerEntries[0].name));
    fileExplorerEntries[fileExplorerCount].isDir = true;
    fileExplorerEntries[fileExplorerCount].size = 0;
    ++fileExplorerCount;
  }
  File dir = EMU32_SETTINGS_FS.open(fileExplorerPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    // Do not present an SD disconnect or failed directory open as an empty
    // folder. Probe the root to distinguish a lost card from a bad path.
    File root = EMU32_SETTINGS_FS.open("/");
    const bool rootOk = root && root.isDirectory();
    if (root) root.close();
    if (!rootOk) {
      sdMarkIoFault("file explorer directory scan");
      strlcpy(fileExplorerStatus, "SD CARD LOST - REINSERT", sizeof(fileExplorerStatus));
    } else {
      strlcpy(fileExplorerStatus, "CANNOT OPEN FOLDER", sizeof(fileExplorerStatus));
    }
    return;
  }
  fileExplorerStatus[0] = '\0';
  File entry = dir.openNextFile();
  while (entry) {
    const char *raw = entry.name();
    const char *base = raw ? strrchr(raw, '/') : nullptr;
    base = base ? base + 1 : (raw ? raw : "");
    if (base[0] && strcmp(base, ".") && strcmp(base, "..")) {
      if (fileExplorerCount >= FILE_EXPLORER_MAX_ENTRIES) {
        fileExplorerTruncated = true;
        entry.close();
        break;
      }
      // Never store a truncated filename: it would display as a different
      // path and could open the wrong file or fail when selected.
      if (strlen(base) >= sizeof(fileExplorerEntries[0].name)) {
        fileExplorerTruncated = true;
        entry.close();
        entry = dir.openNextFile();
        continue;
      }
      FileExplorerEntry &dst = fileExplorerEntries[fileExplorerCount++];
      strlcpy(dst.name, base, sizeof(dst.name));
      dst.isDir = entry.isDirectory();
      dst.size = dst.isDir ? 0 : (uint32_t)entry.size();
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
  // Keep directories first, then files; sort each group alphabetically.
  // Keep the synthetic ".." entry pinned at the top in subdirectories.
  // At the root there is no parent entry, so sorting must include index zero.
  const int sortStart = strcmp(fileExplorerPath, "/") == 0 ? 0 : 1;
  for (int i = sortStart + 1; i < fileExplorerCount; ++i) {
    FileExplorerEntry key = fileExplorerEntries[i];
    int j = i - 1;
    while (j >= 0 && ((fileExplorerEntries[j].isDir < key.isDir) ||
           (fileExplorerEntries[j].isDir == key.isDir && strcasecmp(fileExplorerEntries[j].name, key.name) > 0))) {
      fileExplorerEntries[j + 1] = fileExplorerEntries[j]; --j;
    }
    fileExplorerEntries[j + 1] = key;
  }
}

static bool fileExplorerJoin(char *out, size_t cap, const char *parent, const char *name) {
  if (!out || !cap || !parent || !name) return false;
  int n = strcmp(parent, "/") == 0
    ? snprintf(out, cap, "/%s", name)
    : snprintf(out, cap, "%s/%s", parent, name);
  return n > 0 && (size_t)n < cap;
}

static void fileExplorerDraw(int selected, int top, const char *message) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  const bool land = LCD_W > LCD_H;
  char title[40];
  const char *path = fileExplorerPath;
  if (strlen(path) > (size_t)(land ? 25 : 32)) {
    snprintf(title, sizeof(title), "...%s", path + strlen(path) - (land ? 22 : 29));
  } else strlcpy(title, path, sizeof(title));
  lcdTextRow(0, 21, "FILE EXPLORER", 2, uiWhite(), uiPanel());
  lcdFill(0, 21, LCD_W, 2, uiAccent());
  lcdTextRow(24, 12, title, 1, uiAccent(), uiBg());
  const int topY = land ? 39 : 48;
  const int rowH = land ? 16 : 19;
  const int visible = max(1, (LCD_H - topY - 19) / rowH);
  for (int row = 0; row < visible; ++row) {
    int idx = top + row;
    int y = topY + row * rowH;
    if (y + rowH > LCD_H - 18) break;
    if (idx >= fileExplorerCount) break;
    FileExplorerEntry &e = fileExplorerEntries[idx];
    const bool marked = fileExplorerMarked[idx];
    char label[72];
    // Two marker columns: '+' = marked with X, '[' = folder.
    snprintf(label, sizeof(label), "%c%c %s", marked ? '+' : ' ', e.isDir ? '[' : ' ', e.name);
    menuFitLabel(label, sizeof(label), LCD_W, 1, 3);
    bool isSel = idx == selected;
    uint16_t bg = isSel ? uiBlue() : (marked ? col565(60, 46, 12) : ((row & 1) ? uiPanel2() : uiBg()));
    uint16_t fg = marked ? uiWarn() : (isSel ? uiWhite() : (e.isDir ? uiAccent() : uiMuted()));
    lcdTextRow(y, rowH, label, 1, fg, bg, 3);
    if (isSel) lcdFill(0, y, 3, rowH, uiAccent());
  }
  if (!fileExplorerCount) {
    const char *empty = "EMPTY FOLDER";
    lcdTextRow(LCD_H/2-8, 17, empty, 1, uiMuted(), uiBg(), (LCD_W-textW(empty,1))/2);
  }
  // Footer priority: action result / error, list-full warning, then the
  // current selection + clipboard summary, then the default key hints.
  char footer[48];
  if (message && message[0]) {
    strlcpy(footer, message, sizeof(footer));
  } else if (fileExplorerTruncated) {
    strlcpy(footer, "LIST FULL  X SEL  Y MENU", sizeof(footer));
  } else if (fileExplorerMarkedCount > 0 || fileClipCount > 0) {
    int n = 0;
    footer[0] = '\0';
    if (fileExplorerMarkedCount > 0)
      n += snprintf(footer + n, sizeof(footer) - n, "%d SEL  ", fileExplorerMarkedCount);
    if (fileClipCount > 0 && n < (int)sizeof(footer))
      n += snprintf(footer + n, sizeof(footer) - n, "%d %s  ", fileClipCount, fileClipCut ? "CUT" : "COPIED");
    if (n < (int)sizeof(footer))
      snprintf(footer + n, sizeof(footer) - n, "Y MENU");
  } else {
    strlcpy(footer, "A OPEN  B BACK  X SEL  Y MENU", sizeof(footer));
  }
  lcdTextRow(LCD_H-18, 18, footer, 1, uiMuted(), uiPanel());
}

static void fileExplorerFileInfo(const char *path, const char *name, uint32_t size) {
  uint8_t prev = padState;
  uint32_t t0 = millis(); do { readPad(); delay(10); } while (padState && millis()-t0 < 800);
  prev = padState;
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
    const char *title = "FILE INFO";
    lcdTextRow(0, 22, title, 2, uiWhite(), uiPanel()); lcdFill(0,22,LCD_W,2,uiAccent());
    char label[48]; strlcpy(label, name ? name : "FILE", sizeof(label));
    menuFitLabel(label, sizeof(label), LCD_W, 1, 2);
    lcdTextRow(31, 18, label, 1, uiAccent(), uiBg(), 2);
    char sizeLine[32];
    if (size < 1024) snprintf(sizeLine, sizeof(sizeLine), "%lu BYTES", (unsigned long)size);
    else if (size < 1024UL*1024UL) snprintf(sizeLine, sizeof(sizeLine), "%lu KB", (unsigned long)((size+512)/1024));
    else snprintf(sizeLine, sizeof(sizeLine), "%lu MB", (unsigned long)(((uint64_t)size+524288ULL)/(1024ULL*1024ULL)));
    lcdTextRow(52, 18, sizeLine, 1, uiWhite(), uiPanel());
    char pathLine[192]; strlcpy(pathLine, path ? path : "", sizeof(pathLine));
    const int maxChars = max(1, (LCD_W-4)/4);
    if ((int)strlen(pathLine) > maxChars) {
      const size_t keep = (size_t)maxChars;
      memmove(pathLine, pathLine + strlen(pathLine) - keep, keep + 1);
      pathLine[0] = '~';
    }
    lcdTextRow(76, 12, pathLine, 1, uiMuted(), uiBg(), 2);
    lcdTextRow(LCD_H-18, 18, "B BACK", 1, uiMuted(), uiPanel());
  for (;;) {
    readPad(); bleTick();
    uint8_t pressed = padState & ~prev; prev = padState;
    if (pressed & 0x02) return;
    delay(16);
  }
}

// ---- selection, clipboard and file operations --------------------------------
// X (mark) and Y (action sheet) exist only on the Bluetooth pad: the 8-bit
// padState has no spare bits, so they are read from the raw bleBtn state.
#define FX_FS EMU32_SETTINGS_FS
#define FX_MAX_DEPTH 5          // deepest folder nesting copy / delete will follow
#define FX_MAX_SUBDIRS 16       // subfolders remembered per folder while copying
#define FX_FS_NAME 128          // longest single name handled during copy / delete

static char fxErr[28] = "";     // reason for the most recent failure (shown in footer)
static void fxFail(const char *why) { strlcpy(fxErr, why, sizeof(fxErr)); }

static inline uint16_t fxRawButtons() { return padLinked() ? (uint16_t)bleBtn : 0; }

static void fxWaitRelease(uint32_t maxMs) {
  const uint32_t t0 = millis();
  do { readPad(); bleTick(); delay(10); } while ((padState || fxRawButtons()) && millis() - t0 < maxMs);
}

static bool fxIsParent(int idx) {
  return idx >= 0 && idx < fileExplorerCount && fileExplorerEntries[idx].isDir &&
         !strcmp(fileExplorerEntries[idx].name, "..");
}

// What COPY / CUT / DELETE act on: every marked item, or (when nothing is
// marked) the highlighted item. The synthetic ".." entry is never a target.
static int fxTargetCount(int sel) {
  if (fileExplorerMarkedCount > 0) return fileExplorerMarkedCount;
  return (sel >= 0 && sel < fileExplorerCount && !fxIsParent(sel)) ? 1 : 0;
}
static bool fxIsTarget(int idx, int sel) {
  if (idx < 0 || idx >= fileExplorerCount || fxIsParent(idx)) return false;
  return fileExplorerMarkedCount > 0 ? fileExplorerMarked[idx] : idx == sel;
}
static void fxRecountMarks() {
  int n = 0;
  for (int i = 0; i < fileExplorerCount; ++i) if (fileExplorerMarked[i]) ++n;
  fileExplorerMarkedCount = n;
}
static void fxToggleMark(int idx) {
  if (idx < 0 || idx >= fileExplorerCount || fxIsParent(idx)) return;
  fileExplorerMarked[idx] = !fileExplorerMarked[idx];
  fxRecountMarks();
}
static void fxSelectAll() {
  for (int i = 0; i < fileExplorerCount; ++i) fileExplorerMarked[i] = !fxIsParent(i);
  fxRecountMarks();
}
static void fxClearMarks() {
  memset(fileExplorerMarked, 0, sizeof(fileExplorerMarked));
  fileExplorerMarkedCount = 0;
}

// Clipboard: the folder the items came from plus their names (heap, sized to fit).
static const char *fileClipName(int i) { return fileClipNames + (size_t)i * FILE_EXPLORER_NAME_LEN; }
static void fileClipClear() {
  if (fileClipNames) free(fileClipNames);
  fileClipNames = nullptr; fileClipCount = 0; fileClipCut = false; fileClipDir[0] = '\0';
}
static bool fxClipStore(bool cut, int sel) {
  const int n = fxTargetCount(sel);
  if (n <= 0) return false;
  char *buf = (char *)malloc((size_t)n * FILE_EXPLORER_NAME_LEN);
  if (!buf) return false;                       // old clipboard stays intact
  int k = 0;
  for (int i = 0; i < fileExplorerCount && k < n; ++i) {
    if (!fxIsTarget(i, sel)) continue;
    strlcpy(buf + (size_t)k * FILE_EXPLORER_NAME_LEN, fileExplorerEntries[i].name, FILE_EXPLORER_NAME_LEN);
    ++k;
  }
  fileClipClear();
  fileClipNames = buf; fileClipCount = k; fileClipCut = cut;
  strlcpy(fileClipDir, fileExplorerPath, sizeof(fileClipDir));
  return k > 0;
}

// ---- path helpers ------------------------------------------------------------
static bool fxIsInside(const char *path, const char *root) {   // path == root or below it (FAT is case-insensitive)
  if (!path || !root) return false;
  const size_t pathLen = strlen(path);
  size_t rootLen = strlen(root);
  // Treat a trailing separator as optional, while preserving "/" itself.
  while (rootLen > 1 && root[rootLen - 1] == '/') --rootLen;
  if (rootLen == 0 || pathLen < rootLen) return false;

  // Compare ASCII path characters without case sensitivity. FAT volumes are
  // case-insensitive, so /ROMS/GAMES and /roms/games refer to the same folder.
  for (size_t i = 0; i < rootLen; ++i) {
    unsigned char a = (unsigned char)path[i];
    unsigned char b = (unsigned char)root[i];
    if (a >= 'A' && a <= 'Z') a = (unsigned char)(a + ('a' - 'A'));
    if (b >= 'A' && b <= 'Z') b = (unsigned char)(b + ('a' - 'A'));
    if (a != b) return false;
  }

  if (pathLen == rootLen || root[rootLen - 1] == '/') return true;
  return path[rootLen] == '/' || path[rootLen] == '\\';
}
static bool fxIsDir(const char *path) {
  File f = FX_FS.open(path);
  const bool d = f && f.isDirectory();
  if (f) f.close();
  return d;
}
static bool fxCardOk() {
  File root = FX_FS.open("/");
  const bool ok = root && root.isDirectory();
  if (root) root.close();
  return ok;
}
static bool fxEntryName(File &e, char *out, size_t cap) {
  const char *raw = e.name();
  const char *base = raw ? strrchr(raw, '/') : nullptr;
  base = base ? base + 1 : (raw ? raw : "");
  if (strlen(base) >= cap) return false;
  strlcpy(out, base, cap);
  return true;
}
// Destination path for `name` inside `dir` that does not exist yet:
// "name.ext", then "name_1.ext", "name_2.ext" ... (kept under the explorer's
// name limit so the pasted item still shows up in the list).
static bool fxUniquePath(char *out, size_t cap, const char *dir, const char *name) {
  if (!fileExplorerJoin(out, cap, dir, name)) return false;
  if (!FX_FS.exists(out)) return true;
  const char *dot = strrchr(name, '.');
  const bool hasExt = dot && dot != name;
  const int extLen = hasExt ? (int)strlen(dot) : 0;
  const int stemFull = hasExt ? (int)(dot - name) : (int)strlen(name);
  for (int i = 1; i < 100; ++i) {
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "_%d", i);
    const int room = (FILE_EXPLORER_NAME_LEN - 1) - extLen - (int)strlen(suffix);
    if (room < 1) return false;
    const int stem = stemFull < room ? stemFull : room;
    char cand[FILE_EXPLORER_NAME_LEN];
    snprintf(cand, sizeof(cand), "%.*s%s%s", stem, name, suffix, hasExt ? dot : "");
    if (!fileExplorerJoin(out, cap, dir, cand)) return false;
    if (!FX_FS.exists(out)) return true;
  }
  return false;
}

// ---- progress screen -----------------------------------------------------------
static void fxBusyLine(const char *name) {
  char label[64];
  strlcpy(label, name ? name : "", sizeof(label));
  menuFitLabel(label, sizeof(label), LCD_W, 1, 4);
  lcdTextRow(LCD_H/2 + 8, 16, label, 1, uiAccent(), uiBg(), 4);
}
static void fxBusyStart(const char *title) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  lcdTextRow(LCD_H/2 - 34, 24, title, 2, uiWhite(), uiPanel());
  lcdTextRow(LCD_H - 18, 18, "PLEASE WAIT - DO NOT REMOVE CARD", 1, uiMuted(), uiPanel());
}
static void fxBusyItem(int n, int total, const char *name) {
  char line[24];
  snprintf(line, sizeof(line), "ITEM %d OF %d", n, total);
  lcdTextRow(LCD_H/2 - 6, 14, line, 1, uiMuted(), uiBg(), 4);
  fxBusyLine(name);
}

// ---- copy / move / delete -------------------------------------------------------
static uint8_t *fxCopyBuf = nullptr;
static size_t   fxCopyBufLen = 0;

static bool fxCopyFile(const char *src, const char *dst) {
  File in = FX_FS.open(src, FILE_READ);
  if (!in || in.isDirectory()) { if (in) in.close(); fxFail("CANNOT READ SOURCE"); return false; }
  const uint32_t total = (uint32_t)in.size();
  File out = FX_FS.open(dst, FILE_WRITE);
  if (!out) { in.close(); fxFail("CANNOT CREATE FILE"); return false; }
  uint32_t done = 0;
  bool ok = true;
  while (done < total) {
    const uint32_t left = total - done;
    const size_t want = left < fxCopyBufLen ? (size_t)left : fxCopyBufLen;
    const size_t got = in.read(fxCopyBuf, want);
    if (got == 0) { fxFail("READ FAILED"); ok = false; break; }
    if (out.write(fxCopyBuf, got) != got) { fxFail("WRITE FAILED - CARD FULL"); ok = false; break; }
    done += (uint32_t)got;
    yield();
  }
  in.close();
  out.close();
  if (ok) {                                       // verify what actually landed on the card
    File chk = FX_FS.open(dst, FILE_READ);
    ok = chk && (uint32_t)chk.size() == total;
    if (chk) chk.close();
    if (!ok) fxFail("VERIFY FAILED");
  }
  if (!ok) { FX_FS.remove(dst); sdMarkIoFault("file explorer copy"); }
  return ok;
}

struct FxLevel {
  char sub[FX_MAX_SUBDIRS][FX_FS_NAME];
  char src[FILE_EXPLORER_PATH_LEN];
  char dst[FILE_EXPLORER_PATH_LEN];
};

// Recursive folder copy. Files are copied while the folder is being listed;
// subfolders are only noted and visited after the listing handle is closed, so
// at most three SD file handles are ever open however deep the tree is.
static bool fxDeleteTree(const char *path, int depth);
static bool fxCopyTree(const char *src, const char *dst, int depth) {
  if (depth > FX_MAX_DEPTH) { fxFail("FOLDER TOO DEEP"); return false; }
  File dir = FX_FS.open(src);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); fxFail("CANNOT READ FOLDER"); return false; }
  if (!FX_FS.mkdir(dst)) { dir.close(); fxFail("CANNOT MAKE FOLDER"); return false; }
  FxLevel *L = (FxLevel *)malloc(sizeof(FxLevel));
  if (!L) {
    dir.close();
    fxFail("OUT OF MEMORY");
    // mkdir(dst) already succeeded; remove the empty destination so a failed
    // copy does not leave a misleading partial folder behind.
    fxDeleteTree(dst, 0);
    return false;
  }
  int nsub = 0;
  bool ok = true;
  char name[FX_FS_NAME];
  File e = dir.openNextFile();
  while (e && ok) {
    const bool nameOk = fxEntryName(e, name, sizeof(name));
    const bool isDir = e.isDirectory();
    e.close();
    if (!nameOk) { fxFail("NAME TOO LONG"); ok = false; break; }
    if (strcmp(name, ".") != 0 && strcmp(name, "..") != 0) {
      if (isDir) {
        if (nsub >= FX_MAX_SUBDIRS) { fxFail("TOO MANY SUBFOLDERS"); ok = false; break; }
        strlcpy(L->sub[nsub++], name, FX_FS_NAME);
      } else if (!fileExplorerJoin(L->src, sizeof(L->src), src, name) ||
                 !fileExplorerJoin(L->dst, sizeof(L->dst), dst, name)) {
        fxFail("PATH TOO LONG"); ok = false; break;
      } else {
        fxBusyLine(name);
        ok = fxCopyFile(L->src, L->dst);
      }
    }
    if (ok) e = dir.openNextFile();
  }
  dir.close();
  for (int i = 0; ok && i < nsub; ++i) {
    if (!fileExplorerJoin(L->src, sizeof(L->src), src, L->sub[i]) ||
        !fileExplorerJoin(L->dst, sizeof(L->dst), dst, L->sub[i])) {
      fxFail("PATH TOO LONG"); ok = false; break;
    }
    fxBusyLine(L->sub[i]);
    ok = fxCopyTree(L->src, L->dst, depth + 1);
  }
  free(L);
  if (!ok) {
    // dst is owned by this operation, so roll back its partial contents.
    fxDeleteTree(dst, 0);
  }
  return ok;
}

// Recursive delete. Each pass re-opens the folder and removes its first entry,
// so no listing handle is held while recursing and nothing is removed while a
// directory iterator is live.
static bool fxDeleteTree(const char *path, int depth) {
  if (!fxIsDir(path)) {
    if (!FX_FS.remove(path)) { fxFail("DELETE FAILED"); return false; }
    return true;
  }
  if (depth > FX_MAX_DEPTH + 2) { fxFail("FOLDER TOO DEEP"); return false; }
  char *buf = (char *)malloc(FX_FS_NAME + FILE_EXPLORER_PATH_LEN);
  if (!buf) { fxFail("OUT OF MEMORY"); return false; }
  char *name = buf;
  char *child = buf + FX_FS_NAME;
  bool ok = true;
  for (;;) {
    File d = FX_FS.open(path);
    if (!d) { fxFail("CANNOT READ FOLDER"); ok = false; break; }
    bool found = false, nameOk = true;
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
      nameOk = fxEntryName(e, name, FX_FS_NAME);
      e.close();
      if (!nameOk || (strcmp(name, ".") != 0 && strcmp(name, "..") != 0)) { found = true; break; }
    }
    d.close();
    if (!found) break;                            // folder is empty
    if (!nameOk) { fxFail("NAME TOO LONG"); ok = false; break; }
    if (!fileExplorerJoin(child, FILE_EXPLORER_PATH_LEN, path, name)) { fxFail("PATH TOO LONG"); ok = false; break; }
    fxBusyLine(name);
    if (!fxDeleteTree(child, depth + 1)) { ok = false; break; }
  }
  free(buf);
  if (ok && !FX_FS.rmdir(path)) { fxFail("CANNOT REMOVE FOLDER"); ok = false; }
  return ok;
}

static void fxResult(char *out, size_t cap, int ok, int failed, const char *okWord) {
  if (failed > 0) {
    if (!fxCardOk()) { sdMarkIoFault("file explorer operation"); strlcpy(out, "SD CARD LOST - REINSERT", cap); }
    else snprintf(out, cap, "%d FAILED: %s", failed, fxErr[0] ? fxErr : "CHECK CARD");
  } else snprintf(out, cap, "%s %d", okWord, ok);
}

// Paste the clipboard into the folder currently shown. Name clashes get a
// "_1", "_2" ... suffix instead of overwriting. Cut items are renamed in place
// (instant, same card); copied items are streamed with read-back verification.
static void fxPaste(char *result, size_t cap) {
  result[0] = '\0';
  if (fileClipCount <= 0) { strlcpy(result, "CLIPBOARD EMPTY", cap); return; }
  fxErr[0] = '\0';
  const bool wasCut = fileClipCut;
  if (!wasCut) {
    fxCopyBufLen = 4096; fxCopyBuf = (uint8_t *)malloc(fxCopyBufLen);
    if (!fxCopyBuf) { fxCopyBufLen = 512; fxCopyBuf = (uint8_t *)malloc(fxCopyBufLen); }
    if (!fxCopyBuf) { fxCopyBufLen = 0; strlcpy(result, "OUT OF MEMORY", cap); return; }
  }
  fxBusyStart(wasCut ? "MOVING" : "COPYING");
  int ok = 0, failed = 0, skipped = 0;
  char src[FILE_EXPLORER_PATH_LEN], dst[FILE_EXPLORER_PATH_LEN];
  for (int i = 0; i < fileClipCount; ++i) {
    const char *name = fileClipName(i);
    fxBusyItem(i + 1, fileClipCount, name);
    if (!fileExplorerJoin(src, sizeof(src), fileClipDir, name)) { fxFail("PATH TOO LONG"); ++failed; continue; }
    if (!FX_FS.exists(src)) { fxFail("SOURCE MISSING"); ++failed; continue; }
    const bool srcIsDir = fxIsDir(src);
    if (srcIsDir && fxIsInside(fileExplorerPath, src)) { fxFail("CANT PASTE INTO ITSELF"); ++failed; continue; }
    if (wasCut && !strcmp(fileClipDir, fileExplorerPath)) { ++skipped; continue; }   // already here
    if (!fxUniquePath(dst, sizeof(dst), fileExplorerPath, name)) { fxFail("PATH TOO LONG"); ++failed; continue; }
    bool done;
    if (wasCut) {
      done = FX_FS.rename(src, dst);
      if (!done) fxFail("MOVE FAILED");
    } else if (srcIsDir) {
      done = fxCopyTree(src, dst, 0);
      if (!done) {                                // do not leave a half-copied folder behind
        char why[sizeof(fxErr)]; strlcpy(why, fxErr, sizeof(why));
        fxDeleteTree(dst, 0);
        fxFail(why);
      }
    } else {
      done = fxCopyFile(src, dst);
    }
    if (done) {
      ++ok;
      if (wasCut) fileClipNames[(size_t)i * FILE_EXPLORER_NAME_LEN] = '\0';   // consumed
    } else ++failed;
  }
  if (fxCopyBuf) { free(fxCopyBuf); fxCopyBuf = nullptr; fxCopyBufLen = 0; }
  if (wasCut) {                                   // drop moved items; keep any that failed / were skipped
    int k = 0;
    for (int i = 0; i < fileClipCount; ++i) {
      if (!fileClipNames[(size_t)i * FILE_EXPLORER_NAME_LEN]) continue;
      if (k != i) memcpy(fileClipNames + (size_t)k * FILE_EXPLORER_NAME_LEN,
                         fileClipNames + (size_t)i * FILE_EXPLORER_NAME_LEN, FILE_EXPLORER_NAME_LEN);
      ++k;
    }
    fileClipCount = k;
    if (k == 0) fileClipClear();
  }
  if (!ok && !failed) strlcpy(result, skipped ? "ALREADY IN THIS FOLDER" : "NOTHING PASTED", cap);
  else fxResult(result, cap, ok, failed, wasCut ? "MOVED" : "PASTED");
}

static void fxDeleteTargets(int sel, char *result, size_t cap) {
  fxErr[0] = '\0';
  fxBusyStart("DELETING");
  const int total = fxTargetCount(sel);
  int n = 0, ok = 0, failed = 0;
  char path[FILE_EXPLORER_PATH_LEN];
  for (int i = 0; i < fileExplorerCount; ++i) {
    if (!fxIsTarget(i, sel)) continue;
    fxBusyItem(++n, total, fileExplorerEntries[i].name);
    if (!fileExplorerJoin(path, sizeof(path), fileExplorerPath, fileExplorerEntries[i].name)) { fxFail("PATH TOO LONG"); ++failed; }
    else if (fxDeleteTree(path, 0)) ++ok;
    else ++failed;
  }
  fxResult(result, cap, ok, failed, "DELETED");
}

// ---- Y action sheet and delete confirmation ---------------------------------------
enum { FX_ACT_NONE = -1, FX_ACT_COPY = 0, FX_ACT_CUT, FX_ACT_PASTE, FX_ACT_DELETE,
       FX_ACT_SELECT_ALL, FX_ACT_CLEAR, FX_ACT_COUNT };

static void fxSheetDraw(const char *title, const char *const *labels, const bool *enabled, int cur) {
  const bool land = LCD_W > LCD_H;
  const int rowH = land ? 16 : 19, titleH = 16;
  const int sheetH = 2 + titleH + FX_ACT_COUNT * rowH;
  const int yTop = LCD_H - 18 - sheetH;
  lcdFill(0, yTop, LCD_W, 2, uiAccent());
  lcdTextRow(yTop + 2, titleH, title, 1, uiWhite(), uiPanel(), 6);
  for (int i = 0; i < FX_ACT_COUNT; ++i) {
    const int y = yTop + 2 + titleH + i * rowH;
    const uint16_t fg = !enabled[i] ? col565(78, 88, 104) : (i == cur ? uiWhite() : uiMuted());
    const uint16_t bg = i == cur ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel());
    lcdTextRow(y, rowH, labels[i], 2, fg, bg, 10);
    if (i == cur) lcdFill(0, y, 3, rowH, uiAccent());
  }
  lcdTextRow(LCD_H - 18, 18, "A OK  B OR Y CLOSE", 1, uiMuted(), uiPanel());
}

// Returns the chosen FX_ACT_* value, or FX_ACT_NONE if closed.
static int fxActionSheet(int sel) {
  fxWaitRelease(800);
  uint8_t prev = padState;
  uint16_t prevRaw = fxRawButtons();
  const int targets = fxTargetCount(sel);
  bool enabled[FX_ACT_COUNT];
  enabled[FX_ACT_COPY] = enabled[FX_ACT_CUT] = enabled[FX_ACT_DELETE] = targets > 0;
  enabled[FX_ACT_PASTE] = fileClipCount > 0;
  enabled[FX_ACT_SELECT_ALL] = fileExplorerCount - (fxIsParent(0) ? 1 : 0) > 0;
  enabled[FX_ACT_CLEAR] = fileExplorerMarkedCount > 0;
  int cur = -1;
  for (int i = 0; i < FX_ACT_COUNT && cur < 0; ++i) if (enabled[i]) cur = i;
  if (cur < 0) { strlcpy(fileExplorerStatus, "EMPTY FOLDER - NOTHING TO DO", sizeof(fileExplorerStatus)); return FX_ACT_NONE; }

  char title[32], pasteLabel[24];
  if (fileExplorerMarkedCount > 0)
    snprintf(title, sizeof(title), "%d MARKED ITEM%s", fileExplorerMarkedCount, fileExplorerMarkedCount == 1 ? "" : "S");
  else strlcpy(title, targets > 0 ? "HIGHLIGHTED ITEM" : "NO ITEM TO ACT ON", sizeof(title));
  if (fileClipCount > 0) snprintf(pasteLabel, sizeof(pasteLabel), "PASTE (%d %s)", fileClipCount, fileClipCut ? "CUT" : "COPIED");
  else strlcpy(pasteLabel, "PASTE", sizeof(pasteLabel));
  const char *labels[FX_ACT_COUNT] = { "COPY", "CUT", pasteLabel, "DELETE", "SELECT ALL", "CLEAR SELECTION" };

  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev; prev = s;
    const uint16_t raw = fxRawButtons(), rawPressed = raw & ~prevRaw; prevRaw = raw;
    if ((pressed & 0x02) || (rawPressed & XB_Y)) return FX_ACT_NONE;
    int dir = 0;
    if (pressed & 0x50) dir = -1;
    else if (pressed & 0xA0) dir = 1;
    if (dir) {
      for (int n = 0; n < FX_ACT_COUNT; ++n) {
        cur = (cur + dir + FX_ACT_COUNT) % FX_ACT_COUNT;
        if (enabled[cur]) break;
      }
      dirty = true;
    }
    if (menuConfirmPressed(pressed, 8)) return cur;
    if (dirty) { fxSheetDraw(title, labels, enabled, cur); dirty = false; }
    delay(16);
  }
}

// Deleting is permanent, so ask first. The cursor starts on NO.
static bool fxConfirmDelete(int sel) {
  fxWaitRelease(800);
  uint8_t prev = padState;
  uint16_t prevRaw = fxRawButtons();
  const int n = fxTargetCount(sel);
  bool anyDir = false;
  const char *firstName = "";
  for (int i = 0; i < fileExplorerCount; ++i) {
    if (!fxIsTarget(i, sel)) continue;
    if (!firstName[0]) firstName = fileExplorerEntries[i].name;
    if (fileExplorerEntries[i].isDir) anyDir = true;
  }
  char what[48];
  if (n == 1) strlcpy(what, firstName, sizeof(what));
  else snprintf(what, sizeof(what), "%d ITEMS", n);
  menuFitLabel(what, sizeof(what), LCD_W, 1, 4);
  int confirm = 0;
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev; prev = s;
    const uint16_t raw = fxRawButtons(), rawPressed = raw & ~prevRaw; prevRaw = raw;
    if ((pressed & 0x02) || (rawPressed & XB_Y)) return false;
    if (pressed & 0x50) { confirm = 0; dirty = true; }
    if (pressed & 0xA0) { confirm = 1; dirty = true; }
    if (dirty) {
      lcdFill(0, 0, LCD_W, LCD_H, uiBg());
      lcdTextRow(20, 24, "DELETE ITEMS", 2, uiWhite(), uiPanel());
      lcdTextRow(50, 14, what, 1, uiMuted(), uiBg(), 4);
      lcdTextRow(65, 12, "THIS CANNOT BE UNDONE", 1, uiWarn(), uiBg(), 4);
      if (anyDir) lcdTextRow(77, 12, "FOLDERS LOSE ALL CONTENTS", 1, uiWarn(), uiBg(), 4);
      lcdTextRow(92, 24, "NO", 2, confirm == 0 ? uiWhite() : uiMuted(), confirm == 0 ? uiBlue() : uiPanel());
      lcdTextRow(122, 24, "YES, DELETE", 2, confirm == 1 ? uiWhite() : uiMuted(), confirm == 1 ? col565(110, 0, 0) : uiPanel());
      lcdTextRow(LCD_H - 18, 18, "A CONFIRM   B CANCEL", 1, uiMuted(), uiPanel());
      dirty = false;
    }
    if (menuConfirmPressed(pressed, 9)) return confirm == 1;
    delay(16);
  }
}

static void fileExplorerMenu() {
  if (!sdMount()) { showMessage("NO SD CARD", "CHECK CARD", col565(110,0,0)); delay(900); return; }
  if (!fileExplorerEntries) { showMessage("OUT OF MEMORY", "FILE EXPLORER OFF", col565(110,0,0)); delay(1000); return; }
  strlcpy(fileExplorerPath, "/", sizeof(fileExplorerPath));
  fileExplorerStatus[0] = '\0';
  fileClipClear();                                   // every visit starts with an empty clipboard
  int sel = 0, top = 0;
  fxWaitRelease(1000);
  uint8_t prev = padState;
  uint16_t prevRaw = fxRawButtons();
  uint32_t nextRep = 0;
  fileExplorerScan();
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev; prev = s;
    const uint16_t raw = fxRawButtons(), rawPressed = raw & ~prevRaw; prevRaw = raw;
    const uint32_t now = millis();
    if (pressed & 0x02) {
      if (strcmp(fileExplorerPath, "/") == 0) { fileClipClear(); return; }
      char *slash = strrchr(fileExplorerPath, '/');
      if (slash && slash == fileExplorerPath) fileExplorerPath[1] = '\0';
      else if (slash) *slash = '\0';
      fileExplorerStatus[0] = '\0';
      sel = top = 0; fileExplorerScan(); nextRep = 0; dirty = true;
    } else if (menuConfirmPressed(pressed, 7) && fileExplorerCount > 0) {
      FileExplorerEntry chosen = fileExplorerEntries[sel];
      if (chosen.isDir) {
        if (!strcmp(chosen.name, "..")) {
          if (strcmp(fileExplorerPath, "/") != 0) {
            char *slash = strrchr(fileExplorerPath, '/');
            if (slash && slash == fileExplorerPath) fileExplorerPath[1] = '\0';
            else if (slash) *slash = '\0';
          }
        } else {
          char next[192];
          if (fileExplorerJoin(next, sizeof(next), fileExplorerPath, chosen.name)) {
            strlcpy(fileExplorerPath, next, sizeof(fileExplorerPath));
            fileExplorerStatus[0] = '\0';
          } else {
            strlcpy(fileExplorerStatus, "PATH TOO LONG", sizeof(fileExplorerStatus));
          }
        }
        sel = top = 0; fileExplorerScan(); nextRep = 0; dirty = true;
      } else {
        char full[192];
        if (fileExplorerJoin(full, sizeof(full), fileExplorerPath, chosen.name)) {
          fileExplorerFileInfo(full, chosen.name, chosen.size);
          // The info screen consumes its own B press; sync edge detection so
          // that press cannot also navigate/back out of the file explorer.
          prev = padState;
          prevRaw = fxRawButtons();
        }
        else { strlcpy(fileExplorerStatus, "PATH TOO LONG", sizeof(fileExplorerStatus)); dirty = true; }
      }
    }

    // X: mark / unmark the highlighted item. Mark as many as you like.
    if ((rawPressed & XB_X) && fileExplorerCount > 0) {
      fileExplorerStatus[0] = '\0';
      fxToggleMark(sel);
      dirty = true;
    }

    // Y: action sheet (copy / cut / paste / delete / select all / clear).
    if (rawPressed & XB_Y) {
      fileExplorerStatus[0] = '\0';
      const int act = fxActionSheet(sel);
      char result[48]; result[0] = '\0';
      bool rescan = false;
      switch (act) {
        case FX_ACT_COPY:
        case FX_ACT_CUT: {
          const bool cut = act == FX_ACT_CUT;
          const int n = fxTargetCount(sel);
          if (fxClipStore(cut, sel)) {
            snprintf(result, sizeof(result), "%d %s", n, cut ? "CUT - PASTE IN TARGET FOLDER" : "COPIED");
            fxClearMarks();
          } else strlcpy(result, "OUT OF MEMORY", sizeof(result));
        } break;
        case FX_ACT_PASTE:
          fxPaste(result, sizeof(result));
          rescan = true;
          break;
        case FX_ACT_DELETE:
          if (fxConfirmDelete(sel)) { fxDeleteTargets(sel, result, sizeof(result)); rescan = true; }
          break;
        case FX_ACT_SELECT_ALL:
          fxSelectAll();
          snprintf(result, sizeof(result), "%d MARKED", fileExplorerMarkedCount);
          break;
        case FX_ACT_CLEAR:
          fxClearMarks();
          break;
        default: break;
      }
      if (rescan) {
        fileExplorerScan();                          // also clears the marks
        sel = constrain(sel, 0, max(0, fileExplorerCount - 1));
      }
      if (result[0] && !fileExplorerStatus[0]) strlcpy(fileExplorerStatus, result, sizeof(fileExplorerStatus));
      fxWaitRelease(400);
      prev = padState; prevRaw = fxRawButtons(); nextRep = 0; dirty = true;
    }

    int step = 0;
    if (pressed & 0x10) { step=-1; nextRep=now+350; }
    else if (pressed & 0x20) { step=1; nextRep=now+350; }
    else if ((s&0x10) && (int32_t)(now-nextRep)>=0) { step=-1; nextRep=now+90; }
    else if ((s&0x20) && (int32_t)(now-nextRep)>=0) { step=1; nextRep=now+90; }
    if (step && fileExplorerCount) {
      const int oldSel = sel;
      sel = constrain(sel+step, 0, fileExplorerCount-1);
      if (sel != oldSel) { dirty = true; fileExplorerStatus[0] = '\0'; }
    }
    const int visible = max(1, (LCD_H-(LCD_W>LCD_H?39:48)-19)/(LCD_W>LCD_H?16:19));
    const int oldTop = top;
    if (top > max(0, fileExplorerCount - visible)) top = max(0, fileExplorerCount - visible);
    if (sel < top) top=sel;
    if (sel >= top+visible) top=sel-visible+1;
    if (top != oldTop) dirty = true;
    if (dirty) { fileExplorerDraw(sel, top, fileExplorerStatus); dirty = false; }
    delay(16);
  }
}

static void toolsMenu() {
  int selected = 0;
  uint32_t t0=millis(); do { readPad(); delay(10); } while (padState && millis()-t0<900);
  uint8_t prev=padState;
  uint32_t nextRep=0;
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    uint8_t s=padState, pressed=s&~prev; prev=s;
    if (pressed & 0x02) return;
    if (menuConfirmPressed(pressed, 5)) {
      if (selected == 0) fileExplorerMenu();
      else saveManagerMenu();
      t0=millis(); do { readPad(); delay(10); } while (padState && millis()-t0<500);
      prev=padState; nextRep=0; dirty=true;
    }
    const uint32_t now=millis();
    int step=0;
    if (pressed & 0x50) { step=-1; nextRep=now+350; }
    else if (pressed & 0xA0) { step=1; nextRep=now+350; }
    else if ((s & 0x50) && (int32_t)(now-nextRep)>=0) { step=-1; nextRep=now+90; }
    else if ((s & 0xA0) && (int32_t)(now-nextRep)>=0) { step=1; nextRep=now+90; }
    if (step) {
      const int oldSelected = selected;
      selected=constrain(selected+step,0,1);
      if (selected != oldSelected) dirty = true;
    }

    if (dirty) {
    lcdFill(0,0,LCD_W,LCD_H,uiBg());
    lcdTextRow(0,22,"TOOLS",2,uiWhite(),uiPanel()); lcdFill(0,22,LCD_W,2,uiAccent());
    const char *items[2]={"FILE EXPLORER","SAVE MANAGER"};
    const int rowStep = LCD_W > LCD_H ? 29 : 43;
    const int rowH = LCD_W > LCD_H ? 25 : 38;
    const int firstY = LCD_H/2-rowStep/2;
    for (int i=0;i<2;++i) {
      int y=firstY+i*rowStep;
      lcdTextRow(y,rowH,items[i],2,i==selected?uiWhite():uiMuted(),i==selected?uiBlue():uiPanel(),(LCD_W-textW(items[i],2))/2);
      if (i==selected) lcdFill(0,y,3,rowH,uiAccent());
    }
    lcdTextRow(LCD_H-18,18,"A OPEN  B BACK",1,uiMuted(),uiPanel());
    dirty = false;
    }
    delay(16);
  }
}

// Top-level launcher. Games opens the existing system chooser; Tools opens utility tools.
static void mainSectionDraw(int selected, const char *message) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  const bool land = LCD_W > LCD_H;
  const int titleY = land ? 2 : 20;
  const int titleScale = land ? 3 : 6;
  const int titleW = textW("EMU32", titleScale);
  lcdTextRow(titleY, land ? 18 : 34, "EMU32", titleScale, uiWhite(), uiBg(), (LCD_W-titleW)/2);
  // Keep the accent rule below the full title glyph height (portrait used to overlap it).
  const int ruleY = land ? 22 : 56;
  lcdFill((LCD_W-titleW)/2, ruleY, titleW, 2, uiAccent());
  const char *items[5] = { "GAMES", "TOOLS", "RECENTS", "FAVORITES", "SETTINGS" };
  const int rowStep = land ? 21 : (LCD_W == LCD_H ? 30 : 35);
  const int rowH = land ? 18 : (LCD_W == LCD_H ? 26 : 31);
  const int listY = land ? 27 : (LCD_W == LCD_H ? 58 : 72);
  for (int i=0; i<5; ++i) {
    const int y = listY + i*rowStep;
    const uint16_t bg = (i == selected) ? uiBlue() : ((i & 1) ? uiPanel2() : uiPanel());
    const uint16_t fg = (i == selected) ? uiWhite() : uiMuted();
    lcdTextRow(y, rowH, items[i], land ? 2 : 3, fg, bg,
               (LCD_W-textW(items[i], land ? 2 : 3))/2);
    if (i == selected) lcdFill(0, y, 3, rowH, uiAccent());
  }
  if (message && message[0]) {
    const int y = land ? LCD_H-27 : LCD_H-39;
    lcdTextRow(y, 12, message, 1, uiMuted(), uiBg(),
               (LCD_W-textW(message,1))/2);
  }
  // Compact device-status strip. Battery percentage is intentionally omitted:
  // these board variants do not currently expose a verified battery ADC reading.
  const char *statusBar = "BATTERY POWER  |  SD  |  GAMEPAD";
  const int statusY = land ? LCD_H-27 : (LCD_W == LCD_H ? LCD_H-30 : 258);
  lcdTextRow(statusY, 12, statusBar, 1, uiMuted(), uiBg(),
             (LCD_W-textW(statusBar,1))/2);
  const char *hint = "A: SELECT  B: BACK";
  lcdTextRow(LCD_H-12, 12, hint, 1, uiWhite(), uiPanel(),
             (LCD_W-textW(hint,1))/2);
}

static int sysMenu() {
  static int mainSel = 0;
  uint32_t t0 = millis();
  do { readPad(); delay(10); } while (padState && millis()-t0 < 1500);
  uint8_t prev = padState;
  uint32_t nextRep = 0;
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    const uint32_t now = millis();
    if (menuConfirmPressed(pressed, 4)) {
      if (mainSel == 0) {
        const int system = gamesSystemMenu();
        if (system >= 0) return system;
        dirty = true;
      } else if (mainSel == 2) {
        const int system = recentMenu();
        if (system >= 0) return system;
        dirty = true;
      } else if (mainSel == 3) {
        const int system = favoritesMenu();
        if (system >= 0) return system;
        dirty = true;
      } else if (mainSel == 4) {
        settingsMenu();
        dirty = true;
      } else if (mainSel == 1) {
        toolsMenu();
        dirty = true;
      }
    }
    uint8_t src = 0;
    if (pressed & 0x50) { src=0x10; nextRep=now+350; }
    else if (pressed & 0xA0) { src=0x20; nextRep=now+350; }
    else if ((s & 0x50) && (int32_t)(now-nextRep)>=0) { src=0x10; nextRep=now+90; }
    else if ((s & 0xA0) && (int32_t)(now-nextRep)>=0) { src=0x20; nextRep=now+90; }
    if (src) {
      const int next = constrain(mainSel + (src==0x10 ? -1 : 1), 0, 4);
      if (next != mainSel) { mainSel=next; dirty=true; }
    }
    if (dirty) { mainSectionDraw(mainSel, ""); dirty=false; }
    delay(16);
  }
}

