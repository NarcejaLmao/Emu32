// Menu rendering and navigation. SD/ROM discovery helpers are in sd_rom.h.
#pragma once

static int textW(const char *s, int scale) { return (int)strlen(s) * 4 * scale - scale; }

// Fit file labels to the available row width and add an ellipsis if needed.
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

// Debounce menu confirm actions in addition to edge-detecting button presses.
// This protects destructive/file actions from noisy GPIO or BLE input snapshots.
static bool menuConfirmPressed(uint8_t pressed, uint8_t context = 0) {
  if (!(pressed & 0x09)) return false; // A or START
  // Debounce independently per interaction context. A confirmation in one
  // screen must not suppress a valid press immediately after entering another.
  static uint32_t lastConfirmMs[8] = {};
  static bool haveConfirmed[8] = {};
  if (context >= 8) context = 0;
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
static int menuListTop() { return LCD_W > LCD_H ? 42 : 119; }
static int menuListWidth() { return LCD_W > LCD_H ? LCD_W - 98 : LCD_W; }
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

static void menuDraw(int sel, int top, const char *status) {
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  char title[48];
  snprintf(title, sizeof(title), "%s LIBRARY", sysShort[curSys]);
  lcdTextRow(0, 22, title, 2, uiWhite(), uiPanel());
  lcdFill(0, 22, LCD_W, 2, uiAccent());
  char info[48];
  snprintf(info, sizeof(info), "%d GAMES  |  %d/%d", romCount, romCount ? sel + 1 : 0, romCount);
  lcdTextRow(24, 15, info, 1, uiMuted(), uiBg());

  const int listTop = menuListTop();
  const int listW = menuListWidth();
  for (int i = 0; i < menuVisibleRows(); i++) {
    const int idx = top + i, y = listTop + i * MENU_ROW_H;
    if (y >= LCD_H - 18) break;
    const bool selected = idx == sel;
    const uint16_t rowBg = selected ? uiBlue() : ((i & 1) ? uiPanel2() : uiBg());
    if (idx >= romCount) {
      menuTextRowRegion(0, y, listW, MENU_ROW_H, "", 2, 0, uiBg());
      continue;
    }
    char nm[40]; baseName(romNames[idx], nm, sizeof(nm));
    menuTextRowRegion(0, y, listW, MENU_ROW_H, nm, 2,
                      selected ? uiWhite() : uiMuted(), rowBg);
    if (selected) lcdFill(0, y, 3, MENU_ROW_H, uiAccent());
    else lcdFill(0, y + MENU_ROW_H - 1, listW, 1, uiPanel());
  }

  // Portrait uses a centered preview above the list. Landscape uses a fixed
  // right-side preview column, keeping the list visible on the left.
  if (romCount > 0 && sel >= 0 && sel < romCount) {
    if (LCD_W > LCD_H) boxArtDraw(romNames[sel], LCD_W - 88, 43, 78, 88);
    else boxArtDraw(romNames[sel], (LCD_W - 72) / 2, 42, 72, 70);
  }

  char foot[48];
  if (romListTruncated) snprintf(foot, sizeof(foot), "LOW RAM: LIST INCOMPLETE");
  else if (status && status[0]) snprintf(foot, sizeof(foot), "A PLAY   B BACK   %s", status);
  else snprintf(foot, sizeof(foot), "A PLAY   B BACK   UP/DN MOVE");
  lcdTextRow(LCD_H - 18, 18, foot, 1, uiMuted(), uiPanel());
  lcdFill(0, LCD_H - 18, LCD_W, 1, uiAccent());
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
      if (ns != sel) { sel = ns; dirty = true; }
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
      menuDraw(sel, top, hint);
      lastStatus = status; dirty = false;
    }
    delay(16);
  }
}

// ---- SD-card battery save manager -------------------------------------------
#define SAVE_MANAGER_MAX 128
#define SAVE_MANAGER_PATH ROM_PATH_LEN
struct SaveManagerEntry { char path[SAVE_MANAGER_PATH]; char label[48]; };
static SaveManagerEntry saveManagerEntries[SAVE_MANAGER_MAX];
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
            e.path[0] = '\\0';
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
  File verify = SD.open(tmp, FILE_READ);
  bool ok = copied == expected && verify && verify.size() == expected;
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
    if (strlen(label) > (size_t)((LCD_W > LCD_H) ? 32 : 19)) label[(LCD_W > LCD_H) ? 31 : 18] = 0;
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
      if (strlen(label) > (size_t)((LCD_W > LCD_H) ? 32 : 19)) label[(LCD_W > LCD_H) ? 31 : 18] = 0;
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
        do { readPad(); delay(10); } while (padState && millis() - confirmStart < 1000);
        confirmPrev = padState;
        for (;;) {
          readPad();
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
              // Remove the primary save first. Keep its backup if this fails.
              ok = SD.remove(e.path);
              if (ok && SD.exists(bak)) SD.remove(bak);
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
  if (!land) {
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
}

// Returns SYS_NES, SYS_GB, SYS_GBC or SYS_A2600.
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

    if (menuConfirmPressed(pressed, 4)) {
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
// Browse directories and inspect file names/sizes without modifying card data.
#define FILE_EXPLORER_MAX_ENTRIES 48
struct FileExplorerEntry { char name[64]; bool isDir; uint32_t size; };
static FileExplorerEntry fileExplorerEntries[FILE_EXPLORER_MAX_ENTRIES];
static int fileExplorerCount = 0;
static bool fileExplorerTruncated = false;
static char fileExplorerPath[192] = "/";

static void fileExplorerScan() {
  fileExplorerCount = 0;
  fileExplorerTruncated = false;
  if (strcmp(fileExplorerPath, "/") != 0) {
    strlcpy(fileExplorerEntries[fileExplorerCount].name, "..", sizeof(fileExplorerEntries[0].name));
    fileExplorerEntries[fileExplorerCount].isDir = true;
    fileExplorerEntries[fileExplorerCount].size = 0;
    ++fileExplorerCount;
  }
  File dir = EMU32_SETTINGS_FS.open(fileExplorerPath);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
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
    char label[72];
    snprintf(label, sizeof(label), "%c %s", e.isDir ? '[' : ' ', e.name);
    menuFitLabel(label, sizeof(label), LCD_W, 1, 3);
    bool isSel = idx == selected;
    uint16_t bg = isSel ? uiBlue() : ((row & 1) ? uiPanel2() : uiBg());
    lcdTextRow(y, rowH, label, 1, isSel ? uiWhite() : (e.isDir ? uiAccent() : uiMuted()), bg, 3);
    if (isSel) lcdFill(0, y, 3, rowH, uiAccent());
  }
  if (!fileExplorerCount) {
    const char *empty = "EMPTY FOLDER";
    lcdTextRow(LCD_H/2-8, 17, empty, 1, uiMuted(), uiBg(), (LCD_W-textW(empty,1))/2);
  }
  char footer[48];
  if (fileExplorerTruncated) strlcpy(footer, "LIST FULL  A OPEN  B BACK", sizeof(footer));
  else if (message && message[0]) strlcpy(footer, message, sizeof(footer));
  else strlcpy(footer, "A OPEN  B BACK  UP/DN", sizeof(footer));
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

static void fileExplorerMenu() {
  if (!sdMount()) { showMessage("NO SD CARD", "CHECK CARD", col565(110,0,0)); delay(900); return; }
  strlcpy(fileExplorerPath, "/", sizeof(fileExplorerPath));
  int sel = 0, top = 0;
  uint32_t t0 = millis(); do { readPad(); delay(10); } while (padState && millis()-t0 < 1000);
  uint8_t prev = padState; uint32_t nextRep = 0;
  fileExplorerScan();
  bool dirty = true;
  for (;;) {
    readPad(); bleTick();
    const uint8_t s = padState, pressed = s & ~prev; prev = s;
    const uint32_t now = millis();
    if (pressed & 0x02) {
      if (strcmp(fileExplorerPath, "/") == 0) return;
      char *slash = strrchr(fileExplorerPath, '/');
      if (slash && slash == fileExplorerPath) fileExplorerPath[1] = '\0';
      else if (slash) *slash = '\0';
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
          if (fileExplorerJoin(next, sizeof(next), fileExplorerPath, chosen.name)) strlcpy(fileExplorerPath, next, sizeof(fileExplorerPath));
        }
        sel = top = 0; fileExplorerScan(); nextRep = 0; dirty = true;
      } else {
        char full[192];
        if (fileExplorerJoin(full, sizeof(full), fileExplorerPath, chosen.name)) fileExplorerFileInfo(full, chosen.name, chosen.size);
      }
    }
    int step = 0;
    if (pressed & 0x10) { step=-1; nextRep=now+350; }
    else if (pressed & 0x20) { step=1; nextRep=now+350; }
    else if ((s&0x10) && (int32_t)(now-nextRep)>=0) { step=-1; nextRep=now+90; }
    else if ((s&0x20) && (int32_t)(now-nextRep)>=0) { step=1; nextRep=now+90; }
    if (step && fileExplorerCount) {
      const int oldSel = sel;
      sel = constrain(sel+step, 0, fileExplorerCount-1);
      if (sel != oldSel) dirty = true;
    }
    const int visible = max(1, (LCD_H-(LCD_W>LCD_H?39:48)-19)/(LCD_W>LCD_H?16:19));
    const int oldTop = top;
    if (sel < top) top=sel;
    if (sel >= top+visible) top=sel-visible+1;
    if (top != oldTop) dirty = true;
    if (dirty) { fileExplorerDraw(sel, top, ""); dirty = false; }
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

