// ============================================================================
//  Emu32 - from-scratch NES + Game Boy + Game Boy Color + Atari 2600 emulators for the ESP32-C6
//  Written without using any third-party emulator code.
//
//  Target board : ESP32-C6-LCD-1.47 (ST7789 172x320 + microSD)
//  Orientation  : starts portrait 172x320 (top toward the USB-C port; see LCD_ROT_DEFAULT below); SELECT+RB cycles all 4 orientations; SELECT+RT saves a screenshot
//  Startup      : Emu32 logo splash screen, then the main menu
//  Main menu    : pick a system - NES, GAME BOY, GAME BOY COLOR or ATARI 2600 (UP/DOWN, then A = select)
//  Game list    : lists the ROMs of that system from the SD card:
//                   NES      /roms/nes   *.nes
//                   GAME BOY /roms/gb    *.gb, *.gbc
//                   GAME BOY COLOR /roms/gbc  *.gbc, *.gb
//                   ATARI 2600 /roms/a2600  *.a26, *.bin
//                 UP/DOWN = move, LEFT/RIGHT = page, A (or START) = play,
//                 B (or SELECT) = back to the main menu
//                 In a game, hold SELECT+START (or A+B+SELECT) 1 s to return to the game list;
//                 SELECT+RB rotates the whole display 90 degrees clockwise per press (portrait, landscape, portrait
//                 flipped, landscape flipped) in menus and in games; SELECT+X (Xbox pad) opens the pause menu:
//                 RESUME / RESTART / FRAMESKIP / FPS COUNTER / ROTATE SCREEN / STATE SLOT / SAVE STATE / LOAD STATE / EXIT GAME
//  NES          : see nes_core.h (mappers 0,1,2,3,4,7,66; keep it in the sketch folder)
//  Game Boy     : see gb_core.h (MBC1/2/3/5, battery saves; keep it in the sketch folder)
//  Game Boy Color: see gbc_core.h (CGB colour, 2x speed, VRAM/WRAM banks, HDMA; MBC1/2/3/5, battery saves;
//                 optional boot ROM /bios/gbc/cgb_boot.bin; keep it in the sketch folder)
//  Atari 2600   : see a2600_core.h (2K/4K/F8/F6/F4/FA/E0/3F; keep it in the sketch folder)
//                 D-pad = joystick, A or B = fire, SELECT = Select, START = Reset
//  Not emulated : audio (NES APU / GB sound registers are stubbed so games still run)
//
//  Arduino IDE: board "ESP32C6 Dev Module", USB CDC On Boot = Enabled.
//  Bluetooth   : needs the "NimBLE-Arduino" library (h2zero, v2.x).
//                Press the BOOT button to start pairing a BLE gamepad (put the pad in
//                pairing mode first). The reason of a failed pairing is shown on screen.
//                The last paired gamepad is reconnected automatically at startup
//                (switch it on while the Emu32 splash is showing).
//                Layout: in the menus a BLE pad uses its own buttons (A = OK, B = back);
//                in a game it is remapped to the emulated system's layout.
// ============================================================================
#pragma GCC optimize ("O3")

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <limits.h>
#include <stdint.h>
#include "src/logo.h"        // boot splash logo

#define NES_DIR       "/roms/nes"
#define GB_DIR        "/roms/gb"
#define GBC_DIR       "/roms/gbc"
#define A2600_DIR     "/roms/a2600"
#define ROM_LIST_INITIAL_CAPACITY 32 // grows as needed, limited by available heap
#define ROM_NAME_LEN  256        // supports full SD/FAT filenames (up to 255 bytes)
#define MENU_ROW_H    15
#define MENU_TOP      20
#define MENU_ROWS     ((LCD_H - MENU_TOP) / MENU_ROW_H)   // 20 in portrait, 10 in landscape
#define SHOW_FPS      1          // default for the in-game menu's FPS COUNTER item (1 = on, 0 = off)

// ---------------------------------------------------------------- pins ------
// ESP32-C6-LCD-1.47 (ST7789 LCD + microSD on a shared SPI bus)
#define PIN_SCLK      7
#define PIN_MOSI      6
#define PIN_MISO      5
#define PIN_LCD_CS    14
#define PIN_LCD_DC    15
#define PIN_LCD_RST   21
#define PIN_LCD_BL    22
#define PIN_SD_CS     4
#define LCD_SPI_HZ    80000000   // lower to 40000000 if the picture glitches
#define LCD_COLOR_TEST 0         // 1 = show labelled colour bars for 4 s at boot (set 0 once colours are right)
#define LCD_INVERT    1          // colour inversion: this IPS panel needs 1; set 0 if colours look negative

// Display geometry. The panel is 172 x 320; the screen can be used in 4 orientations
// (SELECT+RB cycles them). LCD_W / LCD_H / GAME_X / GAME_Y follow the current one:
//   portrait 172 x 320 (rotations 0 and 2), landscape 320 x 172 (rotations 1 and 3).
#define LCD_SHORT     172
#define LCD_LONG      320
#define LCD_XOFF      34         // ST7789 RAM offset on the panel's short axis (same in both directions:
                                 // the 172 px panel is centred in the 240 px RAM)
#define LCD_YOFF      0          // no offset on the long axis
// MADCTL per orientation, each step = 90 deg clockwise. Entry 0 is the original portrait
// (MY|MX); if the picture appears upside down at boot, try LCD_ROT_DEFAULT 2.
static const uint8_t lcdMadctl[4] = { 0xC0, 0xA0, 0x00, 0x60 };
#define LCD_ROT_DEFAULT 0        // 0..3. If the picture is upside down at boot, try 2
static uint8_t lcdRot = LCD_ROT_DEFAULT;
static int     scrW = LCD_SHORT, scrH = LCD_LONG;     // current screen size
static int     gameX = 0, gameY = 0;                  // current top-left of the game picture
#define LCD_W         scrW
#define LCD_H         scrH
#define GAME_X        gameX
#define GAME_Y        gameY

// NES picture area: 4:3 (the original TV aspect ratio), 172x129, pinned to the
// top of the screen (USB-C end). The rest of the screen below it stays black.
#define GAME_W        172
#define GAME_H        129        // 172 * 3 / 4

// Buttons: wire between the GPIO and GND (internal pull-ups are used).
// Set a pin to -1 to disable it. The on-board BOOT button (GPIO9) is not a
// game button: pressing it starts pairing a Bluetooth controller.
//                       A  B  SEL START UP  DOWN LEFT RIGHT
static const int8_t padPins[8] = { 1, 2, 3, -1, 10, 11, 18, 19 };

// Current pad state, bit per button (A B SEL START UP DOWN LEFT RIGHT). Written by
// readPad() (GPIO + Bluetooth), read by the menus and both emulators.
static uint8_t padState;

enum : uint8_t { SYS_NES = 0, SYS_GB = 1, SYS_GBC = 2, SYS_A2600 = 3, SYS_COUNT = 4 };
static uint8_t curSys = SYS_NES;                 // system of the running / selected game
// false in the menus: a Bluetooth pad uses its own native layout (A = OK, B = back).
// true while a game runs: the pad is remapped to the emulated system's layout.
static bool    inGame = false;

#include "src/display.h"

#include "src/controller.h"

#include "src/sd_rom.h"
#define SHOT_FS SD            // screenshot.h writes PNGs through this filesystem
#include "src/screenshot.h"  // must come after sd_rom.h / controller.h and before the cores
#include "src/bios.h"
// PNGdec must be included from the .ino itself: the Arduino IDE only adds a
// library to the build when it sees a plain #include (it ignores __has_include).
// If you do NOT have PNGdec installed, delete the next two lines.
#include <PNGdec.h>
#define EMU32_HAS_PNGDEC 1
#include "src/box_art.h"
#define EMU32_SETTINGS_FS SD
static void settingsMenu();
static void settingsLoad();
static void settingsSave();
#include "src/menus.h"

// ============================================================================
//  Emulators (both use lcdWindow/lcdCfg/outLine/col565/padState/loadErr1-2 above;
//  keep nes_core.h and gb_core.h in the sketch folder)
// ============================================================================
#include "src/nes_core.h"
#include "src/gb_core.h"
#include "src/gbc_core.h"
#include "src/a2600_core.h"
#include "src/core_manager.h"

// ============================================================================
//  Setup / loop
// ============================================================================
static uint32_t nextFrame = 0, frameUs = 16639;
static uint8_t  skipped = 0, frameCount = 0;
// Frameskip (in-game menu): 0 = OFF (every frame drawn), 1 = AUTO (skip up to 3 frames in a
// row when running late), 2 = draw 1 frame in 2, 3 = draw 1 frame in 3.
static uint8_t  fsMode = 1, fsCnt = 0;
static const char *const fsName[4] = { "OFF", "AUTO", "1 IN 2", "1 IN 3" };
static bool showFps = SHOW_FPS;                  // toggled from the in-game menu
#define EMU32_SETTINGS_HAS_AUDIO 0

static uint8_t settingsVolume = 50;
static bool settingsLoaded = false;

static void settingsSave() {
  if (!sdMount()) return;
  if (!EMU32_SETTINGS_FS.exists("/system")) EMU32_SETTINGS_FS.mkdir("/system");
  EMU32_SETTINGS_FS.remove("/system/settings.tmp");
  File f = EMU32_SETTINGS_FS.open("/system/settings.tmp", FILE_WRITE);
  if (!f) return;
  f.printf("volume=%u\nframeskip=%u\nfps=%u\nrotation=%u\n",
           (unsigned)settingsVolume, (unsigned)fsMode, showFps ? 1U : 0U,
           (unsigned)(lcdRot & 3));
  f.flush();
  f.close();
  // Preserve the last valid settings file if installing the new one fails.
  // This filesystem does not reliably replace an existing destination on rename.
  const char *settingsDst = "/system/settings.cfg";
  const char *settingsBak = "/system/settings.bak";
  EMU32_SETTINGS_FS.remove(settingsBak);
  const bool hadSettings = EMU32_SETTINGS_FS.exists(settingsDst);
  if (hadSettings && !EMU32_SETTINGS_FS.rename(settingsDst, settingsBak)) {
    EMU32_SETTINGS_FS.remove("/system/settings.tmp");
    return;
  }
  if (!EMU32_SETTINGS_FS.rename("/system/settings.tmp", settingsDst)) {
    if (hadSettings) EMU32_SETTINGS_FS.rename(settingsBak, settingsDst);
    EMU32_SETTINGS_FS.remove("/system/settings.tmp");
    return;
  }
  if (hadSettings) EMU32_SETTINGS_FS.remove(settingsBak);
}

static void settingsLoad() {
  if (settingsLoaded) return;
  if (!sdMount()) return;
  settingsLoaded = true;
  File f = EMU32_SETTINGS_FS.open("/system/settings.cfg", FILE_READ);
  if (!f) {
    #if EMU32_SETTINGS_HAS_AUDIO
      EmuAudio::setVolume(settingsVolume);
    #endif
    return;
  }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line[0] == '#') continue;
    const int eq = line.indexOf('=');
    if (eq <= 0) continue;
    String key = line.substring(0, eq); key.trim();
    String rawValue = line.substring(eq + 1);
    rawValue.trim();
    // Ignore malformed values instead of String::toInt() silently converting
    // text such as "frameskip=oops" to zero and overwriting a valid setting.
    if (!rawValue.length() || rawValue.length() > 10) continue;
    bool numericValue = true;
    for (size_t i = 0; i < rawValue.length(); ++i) {
      if (rawValue[i] < '0' || rawValue[i] > '9') { numericValue = false; break; }
    }
    if (!numericValue) continue;
    long value = rawValue.toInt();
    if (key == "volume") settingsVolume = (uint8_t)constrain(value, 0, 100);
    else if (key == "frameskip") fsMode = (uint8_t)constrain(value, 0, 3);
    else if (key == "fps") showFps = value != 0;
    else if (key == "rotation") lcdRot = (uint8_t)constrain(value, 0, 3);
  }
  f.close();
  fsCnt = 0;
  lcdSetRotation(lcdRot);
  #if EMU32_SETTINGS_HAS_AUDIO
    EmuAudio::setVolume(settingsVolume);
  #endif
}

static void settingsDraw(int selected, const char *status) {
  const bool land = LCD_W > LCD_H;
  const int scale = land ? 2 : 3;
  lcdTextRow(land ? 2 : 18, 22, "SETTINGS", scale, uiWhite(), uiPanel(),
             (LCD_W - textW("SETTINGS", scale)) / 2);
  lcdFill(0, land ? 25 : 43, LCD_W, 2, uiAccent());
  const char *labels[4] = { "VOLUME", "FRAMESKIP", "FPS COUNTER", "ROTATION" };
  char values[4][20];
  #if EMU32_SETTINGS_HAS_AUDIO
    snprintf(values[0], sizeof(values[0]), "%u%%", (unsigned)settingsVolume);
  #else
    snprintf(values[0], sizeof(values[0]), "N/A");
  #endif
  snprintf(values[1], sizeof(values[1]), "%s", fsName[fsMode]);
  snprintf(values[2], sizeof(values[2]), "%s", showFps ? "ON" : "OFF");
  snprintf(values[3], sizeof(values[3]), "%u DEG", (unsigned)(lcdRot * 90));
  const int top = land ? 30 : 49;
  const int pitch = land ? 27 : 36;
  const int rowH = land ? 23 : 31;
  for (int i=0; i<4; ++i) {
    const int y=top+i*pitch;
    const uint16_t bg = i==selected ? uiBlue() : uiPanel();
    const uint16_t fg = i==selected ? uiWhite() : uiMuted();
    // lcdTextRow paints the entire row background, so calling it once for the
    // label and again for the value erased the label immediately. Compose both
    // strings into one row and render it in a single pass so both stay visible.
    char rowText[64];
    const int labelLen = (int)strlen(labels[i]);
    const int valueLen = (int)strlen(values[i]);
    int valueStart = (LCD_W - textW(values[i], 1) - 7 - 5) / 4;
    if (valueStart < labelLen + 2) valueStart = labelLen + 2;
    if (valueStart > (int)sizeof(rowText) - valueLen - 1)
      valueStart = (int)sizeof(rowText) - valueLen - 1;
    memset(rowText, ' ', sizeof(rowText));
    memcpy(rowText, labels[i], labelLen);
    memcpy(rowText + valueStart, values[i], valueLen);
    rowText[valueStart + valueLen] = '\0';
    lcdTextRow(y, rowH, rowText, 1, fg, bg, 5);
    if (i==selected) lcdFill(0,y,3,rowH,uiAccent());
  }
  const char *hint = status && status[0] ? status : "LEFT/RIGHT CHANGE  B BACK";
  lcdTextRow(LCD_H-(land ? 14 : 18), 12, hint, 1, uiWhite(), uiPanel(),
             max(0,(LCD_W-textW(hint,1))/2));
}

static void settingsChange(int selected, int dir) {
  if (selected == 0) {
    #if EMU32_SETTINGS_HAS_AUDIO
      int v = (int)settingsVolume + dir*5;
      settingsVolume = (uint8_t)constrain(v, 0, 100);
      EmuAudio::setVolume(settingsVolume);
    #endif
  } else if (selected == 1) {
    fsMode = (uint8_t)((fsMode + (dir < 0 ? 3 : 1)) % 4);
    fsCnt = 0; skipped = 0;
  } else if (selected == 2) {
    showFps = !showFps;
  } else if (selected == 3) {
    lcdRot = (uint8_t)((lcdRot + (dir < 0 ? 3 : 1)) & 3);
    lcdSetRotation(lcdRot);
  }
  settingsSave();
}

static void settingsMenu() {
  settingsLoad();
  int sel = 0;
  uint32_t t0 = millis();
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1200);
  uint8_t prev = padState;
  uint32_t nextRep = 0;

  // Draw once on entry. The LCD retains its pixels, so repainting every loop
  // needlessly blanks/repaints the full-width text rows and causes visible flash.
  lcdFill(0, 0, LCD_W, LCD_H, uiBg());
  settingsDraw(sel, "");

  for (;;) {
    readPad();
    bleTick();
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    const uint32_t now = millis();
    if (pressed & 0x02) return;

    bool redraw = false;
    const int oldSel = sel;
    if (pressed & 0x10) sel = (sel + 3) % 4;
    else if (pressed & 0x20) sel = (sel + 1) % 4;
    if (sel != oldSel) redraw = true;

    int dir = 0;
    if (pressed & 0x40) { dir = -1; nextRep = now + 300; }
    else if (pressed & 0x80) { dir = 1; nextRep = now + 300; }
    else if ((s & 0x40) && (int32_t)(now - nextRep) >= 0) { dir = -1; nextRep = now + 100; }
    else if ((s & 0x80) && (int32_t)(now - nextRep) >= 0) { dir = 1; nextRep = now + 100; }

    if (dir) {
      settingsChange(sel, dir);
      redraw = true;
    } else if (menuConfirmPressed(pressed, 6)) {
      settingsChange(sel, 1);
      redraw = true;
    }

    if (redraw) settingsDraw(sel, "");
    delay(10);
  }
}

static uint32_t fpsT0 = 0;                       // start of the current 1 s window (ms)
static uint16_t fpsRun = 0, fpsDrawn = 0;        // frames emulated / frames actually drawn in it

// Small text box with a black background at (x0,y0), drawn over the picture.
// Characters are 4*scale px apart (same font as lcdTextRow).
static void lcdTextBox(int x0, int y0, const char *s, int scale, uint16_t fg, uint16_t bg) {
  const int len = strlen(s), w = len * 4 * scale + scale, h = 5 * scale + 2;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x0, y0, x0 + w - 1, y0 + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int ly = 0; ly < h; ly++) {
    for (int x = 0; x < w; x++) fillBuf[x] = bg;
    if (ly >= 1 && ly < 1 + 5 * scale) {
      const int r = (ly - 1) / scale;
      for (int i = 0; i < len; i++) {
        const uint16_t g = glyph(s[i]);
        if (!g) continue;
        for (int c = 0; c < 3; c++)
          if ((g >> (14 - (r * 3 + c))) & 1)
            for (int k = 0; k < scale; k++) fillBuf[scale + i * 4 * scale + c * scale + k] = fg;
      }
    }
    SPI.writeBytes((uint8_t *)fillBuf, w * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// Top-left corner of the picture: "FPS <emulated> DRAW <drawn>", refreshed once a second.
// FPS ~60 = full speed; DRAW lower than FPS means frames are being skipped.
// The game repaints the picture every drawn frame, so the label is re-stamped after each one.
static char fpsText[24] = "";
static void fpsTick(bool drew) {
  fpsRun++;
  if (drew) fpsDrawn++;
  const uint32_t now = millis();
  if (now - fpsT0 >= 1000) {
    snprintf(fpsText, sizeof(fpsText), "FPS %u DRAW %u", (unsigned)fpsRun, (unsigned)fpsDrawn);
    fpsT0 = now; fpsRun = fpsDrawn = 0;
    static bool lowWarned = false;               // one line only: a full USB-CDC buffer would stall the game
    if (!lowWarned && ESP.getMinFreeHeap() < 20000) { lowWarned = true; Serial.printf("WARN: heap low, min %u\n", (unsigned)ESP.getMinFreeHeap()); }
  }
  if (!showFps) return;
  if (drew && fpsText[0]) lcdTextBox(GAME_X, GAME_Y, fpsText, 1, col565(255, 255, 255), 0);
}

// "SHOT 001 SAVED" / failure label, bottom-left of the picture, re-stamped after each drawn
// frame like the FPS counter. When it expires the picture is repainted to wipe it.
static void shotTick(bool drew) {
  if (!shotMsgUntil) return;
  if ((int32_t)(millis() - shotMsgUntil) >= 0) {
    shotMsgUntil = 0; shotMsg[0] = 0;
    lcdDirty = true;                             // band-based cores (GBC) must resend everything
    return;
  }
  if (!drew) return;
  const int picH = shotPicH(curSys) ? shotPicH(curSys) : GAME_H;
  lcdTextBox(GAME_X, GAME_Y + picH - 8, shotMsg, 1,
             shotMsgOk ? col565(120, 255, 120) : col565(255, 120, 120), 0);
}

// Startup splash: the Emu32 logo (src/logo.h) with the subtitle lines below it.
static void splash() {
  const char *sub1 = "FOR ALL YOUR", *sub2 = "EMULATION NEEDS";
  const int ss = 2;                                           // subtitle text scale
  const int w1 = strlen(sub1) * 4 * ss - ss;
  const int w2 = strlen(sub2) * 4 * ss - ss;
  const int ly = 96;                                          // logo top
  lcdFill(0, 0, LCD_W, LCD_H, 0);
  lcdBlit((LCD_W - LOGO_W) / 2, ly, LOGO_W, LOGO_H, logoImg);
  lcdTextRow(ly + LOGO_H + 24, 5 * ss + 4, sub1, ss, col565(150, 150, 150), 0, (LCD_W - w1) / 2);
  lcdTextRow(ly + LOGO_H + 44, 5 * ss + 4, sub2, ss, col565(150, 150, 150), 0, (LCD_W - w2) / 2);
  Serial.println("Emu32");

  uint32_t t0 = millis();
  if (bleHaveSaved()) {                                       // auto-reconnect last controller
    auto status = [&](const char *m) {
      int w = strlen(m) * 8 - 2;
      lcdTextRow(290, 14, m, 2, col565(150, 150, 150), 0, (LCD_W - w) / 2);
    };
    status("CONNECTING PAD");
    bool ok = bleAutoReconnect();
    status(ok ? "PAD CONNECTED" : "PAD NOT FOUND");
  }
  uint32_t el = millis() - t0;
  delay(el < 1800 ? 1800 - el : 700);
}


// Main menu -> game list -> load the chosen ROM and reset the machine.
// Used at boot and again when the player leaves a game.
//   toMainMenu = true : start at the system chooser (boot)
//   toMainMenu = false: go straight to the game list of the current system
static void shutdownCurrentGame() {
  // Print streaming counters before core shutdown closes its ROM handle.
  sdDiagSummary("game-exit");
  // A single teardown path prevents a second core from inheriting stale file
  // handles, cache pages, or heap allocations after an interrupted launch.
  coreShutdown(curSys);
  Serial.printf("Memory after game cleanup: free=%u, min=%u bytes\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}

static void startGame(bool toMainMenu) {
  shotAbort();                                   // no capture may survive a game change
  shotMsgUntil = 0; shotMsg[0] = 0;
  // If called while a game is active, save battery RAM and release its memory
  // before allocating a ROM list or loading another cartridge.
  if (inGame) shutdownCurrentGame();
  inGame = false;                                // menus use the controller's own layout
  controllerResetInput();                        // discard any held/stale game input before menus
  if (!sdMount()) {
    Serial.printf("ERROR: %s / %s\n", loadErr1, loadErr2);
    showMessage(loadErr1, loadErr2, col565(110, 0, 0));
    delay(1800);
    return;                                      // removable media errors must not hard-lock the device
  }

  // Always re-run folder setup after a successful mount. Startup's first mount
  // may fail transiently; this second call ensures folders are created before
  // the ROM browser tries to open /roms/<system>.
  sdInitFolders();

  bool inList = !toMainMenu;
  for (;;) {
    bool launchRecent = false;
    char selectedPath[192] = "";
    if (!inList) {
      curSys = sysMenu();
      if (curSys < 0) return; // B from system picker exits to the firmware main screen
      if (recentLaunchPath[0]) {
        strlcpy(selectedPath, recentLaunchPath, sizeof(selectedPath));
        recentLaunchPath[0] = 0;
        launchRecent = true;
      }
      inList = true;
    }

    if (!launchRecent && !scanRoms()) {                           // no folder / no ROMs: say why, back to main menu
      Serial.printf("ERROR: %s / %s\n", loadErr1, loadErr2);
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(1800);
      if (strcmp(loadErr1, "NOT ENOUGH RAM") == 0 || strcmp(loadErr1, "SD CARD LOST") == 0) return;
      inList = false;
      continue;
    }

    char path[ROM_NAME_LEN + 16];
    if (launchRecent) {
      strlcpy(path, selectedPath, sizeof(path));
      if (!SD_MMC.exists(path)) {
        showMessage("RECENT GAME MISSING", "CHECK SD CARD", col565(110, 0, 0));
        delay(1800); inList = false; continue;
      }
    } else {
      int sel = romMenu();
      if (sel < 0) {
        freeRomList();
        // B from a game list returns to the system chooser inside Games,
        // rather than unwinding to the top-level EMU32 launcher.
        const int nextSystem = gamesSystemMenu();
        if (nextSystem >= 0) {
          curSys = nextSystem;
          inList = true;
        } else {
          inList = false; // B from the system chooser returns to the main launcher
        }
        continue;
      }
      snprintf(path, sizeof(path), "%s/%s", sysDir[curSys], romNames[sel]);
      freeRomList();                             // give the heap back before loading the ROM
    }
    strlcpy(currentRomPath, path, sizeof(currentRomPath));

    // Validate any supplied BIOS before the emulator starts. GB/GBC BIOS files
    // are optional; a missing file uses the core's post-boot fallback, but a
    // present malformed/unreadable candidate stops launch with a clear message.
    lcdClearTransition(0);
    lcdTextRow(LCD_H / 2 - 10, 20, "CHECKING BIOS", 2, col565(255, 255, 255), 0);
    if (!biosPreLaunchCheck(curSys)) {
      Serial.printf("BIOS CHECK FAILED: %s / %s\n", loadErr1, loadErr2);
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(2200);
      continue;
    }

    lcdClearTransition(0);
    lcdTextRow(LCD_H / 2 - 10, 20, "LOADING ROM", 2, col565(255, 255, 255), 0);
    Serial.printf("Memory before ROM load: free=%u, min=%u bytes\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    sdDiagReset(path);
    const bool ok = coreLoad(curSys, path);
    if (ok) {
      recentRecord(path);
      Serial.printf("Memory after ROM load: free=%u, min=%u bytes\n",
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
      break;
    }

    // load failed: show why and return to the game list
    sdDiagSummary("load-failed");
    Serial.printf("MEM: internal free=%u largest=%u | PSRAM %s, free=%u\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  psramFound() ? "on" : "none", (unsigned)ESP.getFreePsram());
    Serial.printf("ERROR: %s / %s\n", loadErr1, loadErr2);
    showMessage(loadErr1, loadErr2, col565(110, 0, 0));
    delay(2500);
  }

  coreReset(curSys);
  // Every core streams its ROM from the SD card, so the card stays mounted while a game runs.
  frameUs = coreFramePeriodUs(curSys);
  inGame = true;                                 // from here the pad follows the system's layout
  lcdClearTransition(0);
  fpsText[0] = 0;
  skipped = 0;
  nextFrame = micros();
  fpsT0 = millis(); fpsRun = fpsDrawn = 0; fpsText[0] = 0;
}

static void exitToMenu() {
  Serial.println("Back to menu");
  fpsText[0] = 0;
  lcdClearTransition(0);
  // startGame owns teardown so every exit route uses the same cleanup logic.
  startGame(false);                              // game list of the same system
}

// ---- In-game pause menu ------------------------------------------------------
// Opened with SELECT+X (Xbox pad). SELECT+RB (right bumper) rotates the whole display 90 degrees anywhere in a game. SELECT+START (or A+B+SELECT) held ~1 s still quits.
// Portrait: drawn in the black area under the picture, so the frozen game stays visible above it.
// Landscape: there is no room beside the picture, so the menu takes the whole screen.
// UP/DOWN = move, LEFT/RIGHT = change a setting, A (or START) = choose, B/SELECT = resume.
static void restartGame() {
  if (curSys == SYS_NES) mapperInit(); // mapper registers are NES-specific.
  coreReset(curSys);
  frameUs = coreFramePeriodUs(curSys);
}

static bool menuComboDown() {
  const bool sel = padState & 0x04;
  return sel && (bleBtn & XB_X) && !(padState & 0x08);
}

// Blank the pause-menu area: below the picture in portrait, the whole screen in landscape.
static void pauseClear() {
  if (LCD_W > LCD_H) lcdFill(0, 0, LCD_W, LCD_H, 0);
  else               lcdFill(0, GAME_H, LCD_W, LCD_H - GAME_H, 0);
}

#define PAUSE_ITEMS 9
static char pauseStatus[28] = "";
static uint8_t stateSlot = 0;
static void pauseMenuDraw(int sel) {
  static const char *const items[PAUSE_ITEMS] = { "RESUME", "RESTART", "FRAMESKIP", "FPS COUNTER", "ROTATE SCREEN", "STATE SLOT", "SAVE STATE", "LOAD STATE", "EXIT GAME" };
  const uint16_t white = col565(255, 255, 255), grey = col565(150, 150, 150), blue = col565(40, 90, 220);
  const bool land = LCD_W > LCD_H;
  const int y0 = land ? 2 : GAME_H + 8;
  const int first = land ? 17 : 22, pitch = 15, rowH = 14;
  lcdTextRow(y0, 16, "PAUSED", 2, grey, 0, (LCD_W - textW("PAUSED", 2)) / 2);
  for (int i = 0; i < PAUSE_ITEMS; i++) {
    const bool on = (i == sel);
    char line[28];
    if (i == 2)      snprintf(line, sizeof(line), "FRAMESKIP:%s", fsName[fsMode]);
    else if (i == 3) snprintf(line, sizeof(line), "FPS COUNTER:%s", showFps ? "ON" : "OFF");
    else if (i == 5) snprintf(line, sizeof(line), "STATE SLOT:%u", (unsigned)(stateSlot + 1));
    else             snprintf(line, sizeof(line), "%s", items[i]);
    lcdTextRow(y0 + first + i * pitch, rowH, line, 1, on ? white : grey, on ? blue : 0, 6);
  }
  if (pauseStatus[0]) lcdTextRow(LCD_H - (land ? 14 : 18), 14, pauseStatus, 1, col565(120, 220, 120), 0, 4);
  else lcdTextRow(LCD_H - (land ? 14 : 18), 14, "A:OK  SEL:RESUME", 1, col565(90, 90, 90), 0, 8);
}

// Returns true when the player left the game (the list is already shown again).
static bool pauseMenu() {
  uint32_t t0 = millis();                        // wait for buttons to be released
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1500);

  pauseClear();
  lcdDirty = true;                                           // the menu painted over the picture: resend all bands
  int sel = 0;
  uint8_t prev = 0;
  bool done = false, left = false, redraw = false;
  int lastSig = sel | (fsMode << 4) | (showFps << 8) | (stateSlot << 9);
  pauseStatus[0] = 0;
  pauseMenuDraw(sel);

  while (!done) {
    readPad();
    bleTick();
    const uint8_t s = padState, pressed = s & ~prev;
    prev = s;
    int dir = 0;
    if (pressed & 0x10) sel = (sel + PAUSE_ITEMS - 1) % PAUSE_ITEMS;                // up
    if (pressed & 0x20) sel = (sel + 1) % PAUSE_ITEMS;                // down
    if (pressed & 0x40) dir = -1;                           // left
    if (pressed & 0x80) dir = 1;                            // right
    if (pressed & 0x06) done = true;                        // B / SELECT: resume
    if (menuConfirmPressed(pressed, 5) || dir) {                // A / START / LEFT / RIGHT
      if (sel == 0 && !dir) done = true;
      else if (sel == 1 && !dir) { restartGame(); done = true; }
      else if (sel == 2) { fsMode = (fsMode + (dir < 0 ? 3 : 1)) % 4; settingsSave(); }
      else if (sel == 3) { showFps = !showFps; settingsSave(); }
      else if (sel == 4) { lcdSetRotation(lcdRot + (dir < 0 ? 3 : 1)); redraw = true; }   // screen already blanked
      else if (sel == 5) { stateSlot = (stateSlot + (dir < 0 ? 2 : 1)) % 3; redraw = true; }
      else if (sel == 6 && !dir) {
        bool ok = coreSaveState(curSys, stateSlot);
        if (ok) strlcpy(pauseStatus, "STATE SAVED", sizeof(pauseStatus));
        else if (sdSaveLastError[0]) strlcpy(pauseStatus, sdSaveLastError, sizeof(pauseStatus));
        else strlcpy(pauseStatus, "SAVE STATE FAILED", sizeof(pauseStatus));
        redraw = true;
      }
      else if (sel == 7 && !dir) {
        bool ok = coreLoadState(curSys, stateSlot);
        if (ok) strlcpy(pauseStatus, "STATE LOADED", sizeof(pauseStatus));
        else if (sdSaveLastError[0]) strlcpy(pauseStatus, sdSaveLastError, sizeof(pauseStatus));
        else strlcpy(pauseStatus, "NO VALID STATE", sizeof(pauseStatus));
        redraw = true;
      }
      else if (sel == 8 && !dir) { done = true; left = true; }
    }
    const int sig = sel | (fsMode << 4) | (showFps << 8) | (stateSlot << 9) | (pauseStatus[0] ? 0x1000 : 0);
    if (sig != lastSig || redraw) { lastSig = sig; redraw = false; pauseMenuDraw(sel); }
    delay(20);
  }

  if (left) { exitToMenu(); return true; }
  pauseClear();                                              // clear the menu, back to the game
  lcdDirty = true;                                           // menu painted over the picture: resend all bands
  do { readPad(); delay(10); } while (padState && millis() - t0 < 1500);
  fsCnt = 0; skipped = 0;
  fpsT0 = millis(); fpsRun = fpsDrawn = 0; fpsText[0] = 0;
  nextFrame = micros();
  return false;
}

void setup() {
  Serial.begin(115200);
  {
    static const char *const why[] = { "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
                                       "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE", "PWR_GLITCH", "CPU_LOCKUP" };
    const unsigned r = (unsigned)esp_reset_reason();
    Serial.printf("Reset reason: %s (%u)\n", r < 16 ? why[r] : "?", r);   // PANIC / *_WDT / BROWNOUT = a crash
  }
  for (int i = 0; i < 8; i++) if (padPins[i] >= 0) pinMode(padPins[i], INPUT_PULLUP);
  pinMode(PAIR_PIN, INPUT_PULLUP);
  ledInit();

  // Bring up the display before touching SD/BLE or allocating emulator
  // resources. If the card is absent or SD initialization fails, the user
  // still gets a visible screen instead of a black backlight/display.
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  Serial.printf("Memory at boot: internal free=%u, PSRAM %s (%u bytes free)\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                psramFound() ? "ENABLED" : "none (C6 has no PSRAM)", (unsigned)ESP.getFreePsram());
  Serial.println("LCD: initializing");
  lcdInit();
  lcdClearTransition(0);
  lcdTextRow(140, 14, "STARTING EMU32", 2, col565(255, 255, 255), 0);
  Serial.println("LCD: initialized");
#if LCD_COLOR_TEST
  {
    // Each band is labelled with the colour it SHOULD be. Compare with what you see:
    //   red/blue swapped -> toggle 0x08 in lcdMadctl[];  everything negative (white band black) -> LCD_INVERT
    struct Bar { const char *name; uint8_t r, g, b; uint16_t txt; };
    const Bar bars[] = {
      { "RED",   255, 0,   0,   col565(255,255,255) },
      { "GREEN", 0,   255, 0,   col565(0,0,0) },
      { "BLUE",  0,   0,   255, col565(255,255,255) },
      { "WHITE", 255, 255, 255, col565(0,0,0) },
      { "BLACK", 0,   0,   0,   col565(255,255,255) },
    };
    const int bh = LCD_H / 5;
    for (int i = 0; i < 5; i++) {
      const uint16_t c = col565(bars[i].r, bars[i].g, bars[i].b);
      lcdFill(0, i * bh, LCD_W, bh, c);
      lcdTextRow(i * bh + (bh - 20) / 2, 20, bars[i].name, 2, bars[i].txt, c);
    }
    delay(4000);
    lcdClearTransition(0);
    lcdTextRow(140, 14, "STARTING EMU32", 2, col565(255, 255, 255), 0);
  }
#endif

  splash();

  Serial.println("SD: initializing folders");
  sdInitFolders();                              // failure is non-fatal; show menu regardless
  settingsLoad();
  Serial.println("SD: initialization finished");
  coreInitializeBuiltins();
  startGame(true);
}

void loop() {
  // Send 'p' over Serial (115200 baud) to toggle lightweight frame profiling.
  while (Serial.available() > 0) {
    const int cmd = Serial.read();
    if (cmd == 'p' || cmd == 'P') {
      profileFrames = !profileFrames;
      profCoreSum = profCoreMax = profAudioSum = profAudioMax = 0;
      profSamples = 0;
      Serial.printf("PROFILE: %s\n", profileFrames ? "ON (core us)" : "OFF");
    }
  }
  uint32_t now = micros();
  bool draw = true;
  if (fsMode >= 2) {                             // fixed: draw 1 frame in N
    draw = (++fsCnt >= fsMode);
    if (draw) fsCnt = 0;
  } else if (fsMode == 1 && (int32_t)(now - nextFrame) > (int32_t)frameUs && skipped < 3) {
    draw = false; skipped++;                     // auto: running late, skip drawing
  } else skipped = 0;

  readPad();
  // In-game BLE input is event-driven; skip idle reconnect polling on every emulated frame.
  if (!inGame || pairState != PAIR_IDLE) bleTick();

  static bool comboWas = false;
  const bool combo = menuComboDown();
  if (combo && !comboWas) { comboWas = true; shotAbort(); pauseMenu(); return; }
  comboWas = combo;

  rotateHotkey();                                // SELECT+RB: next orientation (the next drawn frame repaints)
  shotHotkey();                                  // SELECT+RT: screenshot request

  // Hold SELECT+START (or A+B+SELECT, for pads without START) for ~1 s to quit.
  static uint8_t quitFrames = 0;
  if (((padState & 0x0C) == 0x0C) || ((padState & 0x07) == 0x07)) {
    if (++quitFrames >= 60) { quitFrames = 0; exitToMenu(); return; }
  } else quitFrames = 0;

  shotArm();                                     // allocate the capture buffer if requested
  if (shotBuf) draw = true;                      // the captured frame must actually be drawn
  uint32_t profElapsed = 0;
  if (profileFrames) { const uint32_t profT = micros(); coreFrame(curSys, draw); profElapsed = micros() - profT; profCoreSum += profElapsed; if (profElapsed > profCoreMax) profCoreMax = profElapsed; if (++profSamples >= 60) { Serial.printf("PROFILE %s: core avg=%lu max=%lu us\n", curSys == SYS_NES ? "NES" : curSys == SYS_GB ? "GB" : curSys == SYS_GBC ? "GBC" : curSys == SYS_A2600 ? "A2600" : "GEN", (unsigned long)(profCoreSum / profSamples), (unsigned long)profCoreMax); profCoreSum = profCoreMax = 0; profSamples = 0; } } else coreFrame(curSys, draw);
  const bool shotTouchedSd = shotService();      // finish / time out the capture (writes the PNG)
  if (curSys == SYS_A2600) frameUs = coreFramePeriodUs(curSys); // PAL/NTSC timing may change
  if (coreRomReadFailed(curSys)) {
    strlcpy(loadErr1, sdIoFault ? "SD CARD LOST" : "ROM READ ERROR", sizeof(loadErr1));
    strlcpy(loadErr2, sdIoFault ? "REINSERT CARD" : "CHECK SD CARD", sizeof(loadErr2));
    showMessage(loadErr1, loadErr2, col565(110, 0, 0));
    delay(1200);
    exitToMenu();
    return;
  }
  fpsTick(draw);
  shotTick(draw);
  if (shotTouchedSd) nextFrame = micros();       // the SD write took a while: resync frame timing

  nextFrame += frameUs;
  int32_t diff = (int32_t)(nextFrame - micros());
  if (diff > 1000) delay(diff / 1000);
  if (diff > 0) { while ((int32_t)(nextFrame - micros()) > 0) { } }
  else if (diff < -100000) nextFrame = micros();     // fell far behind: resync
  if ((++frameCount & 31) == 0) delay(1);            // let the idle task / watchdog run
}
