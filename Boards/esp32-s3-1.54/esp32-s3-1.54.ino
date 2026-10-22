// ============================================================================
//  Emu32 - from-scratch NES + Game Boy + Game Boy Color + Atari 2600 emulators for the ESP32-S3
//  Written without using any third-party emulator code.
//
//  Target board : Waveshare ESP32-S3-LCD-1.54 / ESP32-S3-Touch-LCD-1.54 (JD9853/ST7789-compatible 240x240 LCD + touch + microSD)
//  Orientation  : starts portrait 240x240 (see LCD_ROT_DEFAULT below if it boots upside down); SELECT+RB cycles all 4 orientations; SELECT+RT saves a screenshot
//  Startup      : Emu32 logo splash screen, then the main menu
//  Main menu    : pick a system - NES, GAME BOY, GAME BOY COLOR or ATARI 2600 (UP/DOWN, then A = select)
//  Game list    : lists the ROMs of that system from the SD card:
//                   NES      /roms/nes   *.nes
//                   GAME BOY /roms/gb    *.gb, *.gbc
//                   GAME BOY COLOR /roms/gbc  *.gbc, *.gb
//                   ATARI 2600 /roms/a2600  *.a26, *.bin
//                   GENESIS    /roms/genesis *.md, *.gen, *.bin
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
//  Genesis      : see genesis_core.h (68000 + VDP + lightweight PSG/FM sound; keep it in the sketch folder)
//                 D-pad = joystick, A or B = fire, SELECT = Select, START = Reset
//  Audio        : lightweight register-driven generators for NES, GB, GBC, Atari 2600 and Genesis
//
//  Arduino IDE: board "ESP32S3 Dev Module" (arduino-esp32 core >= 3.0.2), USB CDC On Boot = Enabled,
//               Flash Size 16MB, CPU 240MHz, PSRAM = "OPI PSRAM" (ROM caches spill into it; without it large ROMs can fail with NOT ENOUGH RAM).
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
#include "src/audio.h" // I2S output backend for the board audio codec/amplifier
#include <SPI.h>
#include <FS.h>
#include <SD_MMC.h>        // this board wires the microSD slot to the SDMMC peripheral, not SPI
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <limits.h>
#include <stdint.h>
#include "src/logo.h"        // boot splash logo

#define NES_DIR       "/roms/nes"
#define GB_DIR        "/roms/gb"
#define GBC_DIR       "/roms/gbc"
#define A2600_DIR     "/roms/a2600"
#define GENESIS_DIR   "/roms/genesis"
#define ROM_LIST_INITIAL_CAPACITY 32 // grows as needed, limited by available heap
#define ROM_NAME_LEN  256        // supports full SD/FAT filenames (up to 255 bytes)
#define MENU_ROW_H    15
#define MENU_TOP      20
#define MENU_ROWS     ((LCD_H - MENU_TOP) / MENU_ROW_H)   // 14 in either orientation
#define SHOW_FPS      1          // default for the in-game menu's FPS COUNTER item (1 = on, 0 = off)

// ---------------------------------------------------------------- pins ------
// Waveshare ESP32-S3-LCD-1.54 / ESP32-S3-Touch-LCD-1.54 (ESP32-S3R8, 16MB flash, 8MB PSRAM).
// LCD pins come from Waveshare's factory demo BSP for this board (SCLK 38, MOSI 39, CS 21,
// DC 45, RST 40, BL 46), which also matches the pin text on Waveshare's S3 wiki page.
// NOTE: the 04_gfx_helloworld example on that page uses GPIO1-5, but those are the pins of
// the ESP32-C6 1.54 board (the example was copied); they are NOT the S3 board's LCD pins.
// SD pins are confirmed working on hardware.
// ST7789 240x240 LCD on SPI. The touch variant adds CST816 I2C touch; Emu32 currently
// uses the BLE controller and does not consume touch input.
#define PIN_SCLK      38
#define PIN_MOSI      39
#define PIN_MISO      -1
#define PIN_LCD_CS    21
#define PIN_LCD_DC    45
#define PIN_LCD_RST   40
#define PIN_LCD_BL    46         // backlight, active HIGH
// TF/microSD uses the SDMMC peripheral. 4-bit mode with all six pins matches Waveshare's own
// 07_sd_card_test example for this board. If the mount still times out, try SD_1BIT 1.
#define PIN_SD_CLK    16
#define PIN_SD_CMD    15
#define PIN_SD_D0     17
#define PIN_SD_D1     18
#define PIN_SD_D2     13
#define PIN_SD_D3     14
#define SD_1BIT       0
#define SD_FREQ_KHZ   20000
#define LCD_SPI_HZ    40000000
#define LCD_COLOR_TEST 0         // set to 1 to show labelled colour bars at boot (checks colour order / inversion)
#define LCD_INVERT    1          // this IPS ST7789 panel needs INVON (Waveshare examples use ips=true)
// Other on-board peripherals (do not reuse these GPIOs):
//   I2C (ES8311 / ES7210 / QMI8658 / CST816): SDA 42, SCL 41
//   I2S audio: MCLK 8, BCLK 9, LRCK 10, DIN 11, DOUT 12; speaker amp enable (PA_CTRL) 7
//   USB D-/D+ = 19/20; BOOT button = GPIO0; octal PSRAM = 33-37

// Display geometry: square 240x240 ST7789 panel. Rotation shortcuts remain enabled.
#define LCD_SHORT     240
#define LCD_LONG      240
// The ST7789 has 240x320 RAM but this panel is 240x240, so the flipped orientations
// need a window offset of 80 (same as Adafruit/Arduino_GFX use for 240x240 ST7789).
// MADCTL is RGB order (BGR bit 0x08 clear): 0, MX|MV, MX|MY, MY|MV.
static const uint8_t lcdMadctl[4] = { 0x00, 0x60, 0xC0, 0xA0 };
static const uint8_t lcdXoffRot[4] = { 0, 0, 0, 80 };
static const uint8_t lcdYoffRot[4] = { 0, 0, 80, 0 };
#define LCD_ROT_DEFAULT 0
static uint8_t lcdRot = LCD_ROT_DEFAULT;
static int     scrW = LCD_SHORT, scrH = LCD_LONG;
static int     gameX = 0, gameY = 0;
#define LCD_W         scrW
#define LCD_H         scrH
#define GAME_X        gameX
#define GAME_Y        gameY

// NES picture area: 4:3 TV aspect ratio, 240x180.
#define GAME_W        240
#define GAME_H        180

// Optional wired buttons: wire between a free GPIO and GND (internal pull-ups are used).
// All disabled (-1) by default: play with a BLE gamepad. Do NOT use 38-40/45/46/21 (LCD), 13-18 (microSD), 7-12 (audio),
// 41/42 (I2C), 19/20 (USB), 0 (BOOT) or 33-37 (octal PSRAM). The on-board BOOT button (GPIO0) is not a game button:
// pressing it starts pairing a Bluetooth controller.
//                       A   B   SEL START UP  DOWN LEFT RIGHT
static const int8_t padPins[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

// Current pad state, bit per button (A B SEL START UP DOWN LEFT RIGHT). Written by
// readPad() (GPIO + Bluetooth), read by the menus and both emulators.
static uint8_t padState;

enum : uint8_t { SYS_NES = 0, SYS_GB = 1, SYS_GBC = 2, SYS_A2600 = 3, SYS_GENESIS = 4, SYS_COUNT = 5 };
static uint8_t curSys = SYS_NES;                 // system of the running / selected game
// false in the menus: a Bluetooth pad uses its own native layout (A = OK, B = back).
// true while a game runs: the pad is remapped to the emulated system's layout.
static bool    inGame = false;

#include "src/display.h"

#include "src/controller.h"

#include "src/sd_rom.h"
#define SHOT_FS SD_MMC            // screenshot.h writes PNGs through this filesystem
#include "src/screenshot.h"  // must come after sd_rom.h / controller.h and before the cores
#include "src/bios.h"
// PNGdec must be included from the .ino itself: the Arduino IDE only adds a
// library to the build when it sees a plain #include (it ignores __has_include).
// If you do NOT have PNGdec installed, delete the next two lines.
#include <PNGdec.h>
#define EMU32_HAS_PNGDEC 1
#include "src/box_art.h"
#include "src/menus.h"

// ============================================================================
//  Emulators (both use lcdWindow/lcdCfg/outLine/col565/padState/loadErr1-2 above;
//  keep nes_core.h and gb_core.h in the sketch folder)
// ============================================================================
#include "src/nes_core.h"
#include "src/gb_core.h"
#include "src/gbc_core.h"
#include "src/a2600_core.h"
#include "src/genesis_core.h"
#include "src/core_manager.h"
#include "src/audio_synth.h"

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
  const char *sub1 = "NES GB GBC 2600", *sub2 = "FOR ESP32-S3";
  const int ss = 2;                                           // subtitle text scale
  const int w1 = strlen(sub1) * 4 * ss - ss;
  const int w2 = strlen(sub2) * 4 * ss - ss;
  const int ly = (LCD_H - LOGO_H) / 2 - 20;                   // logo top (centred, a little high)
  lcdFill(0, 0, LCD_W, LCD_H, 0);
  lcdBlit((LCD_W - LOGO_W) / 2, ly, LOGO_W, LOGO_H, logoImg);
  lcdTextRow(ly + LOGO_H + 20, 5 * ss + 4, sub1, ss, col565(150, 150, 150), 0, (LCD_W - w1) / 2);
  lcdTextRow(ly + LOGO_H + 40, 5 * ss + 4, sub2, ss, col565(150, 150, 150), 0, (LCD_W - w2) / 2);
  Serial.println("Emu32");

  uint32_t t0 = millis();
  if (bleHaveSaved()) {                                       // auto-reconnect last controller
    auto status = [&](const char *m) {
      int w = strlen(m) * 8 - 2;
      lcdTextRow(LCD_H - 20, 14, m, 2, col565(150, 150, 150), 0, (LCD_W - w) / 2);
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
    if (!inList) {
      curSys = sysMenu();
      if (curSys < 0) return; // B from system picker exits to the firmware main screen
      inList = true;
    }

    if (!scanRoms()) {                           // no folder / no ROMs: say why, back to main menu
      Serial.printf("ERROR: %s / %s\n", loadErr1, loadErr2);
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(1800);
      if (strcmp(loadErr1, "NOT ENOUGH RAM") == 0 || strcmp(loadErr1, "SD CARD LOST") == 0) return;
      inList = false;
      continue;
    }

    int sel = romMenu();
    if (sel < 0) { freeRomList(); inList = false; continue; }   // SELECT: back to main menu

    char path[ROM_NAME_LEN + 16];
    snprintf(path, sizeof(path), "%s/%s", sysDir[curSys], romNames[sel]);
    strlcpy(currentRomPath, path, sizeof(currentRomPath));
    freeRomList();                               // give the heap back before loading the ROM

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
      Serial.printf("Memory after ROM load: free=%u, min=%u bytes\n",
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
      break;
    }

    // load failed: show why and return to the game list
    sdDiagSummary("load-failed");
    Serial.printf("MEM: internal free=%u largest=%u | PSRAM %s, free=%u\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  psramFound() ? "on" : "OFF (Tools > PSRAM > OPI PSRAM)", (unsigned)ESP.getFreePsram());
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
  if (LCD_W >= LCD_H) lcdFill(0, 0, LCD_W, LCD_H, 0);   // wide and square: the menu uses the whole screen
  else                lcdFill(0, GAME_H, LCD_W, LCD_H - GAME_H, 0);
}

#define PAUSE_ITEMS 9
static char pauseStatus[28] = "";
static uint8_t stateSlot = 0;
static void pauseMenuDraw(int sel) {
  static const char *const items[PAUSE_ITEMS] = { "RESUME", "RESTART", "FRAMESKIP", "FPS COUNTER", "ROTATE SCREEN", "STATE SLOT", "SAVE STATE", "LOAD STATE", "EXIT GAME" };
  const uint16_t white = col565(255, 255, 255), grey = col565(150, 150, 150), blue = col565(40, 90, 220);
  const bool land = LCD_W >= LCD_H;              // wide and square: full-screen pause menu
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
    if (menuConfirmPressed(pressed) || dir) {                // A / START / LEFT / RIGHT
      if (sel == 0 && !dir) done = true;
      else if (sel == 1 && !dir) { restartGame(); done = true; }
      else if (sel == 2) fsMode = (fsMode + (dir < 0 ? 3 : 1)) % 4;
      else if (sel == 3) showFps = !showFps;
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
  delay(250);
  // Initialize audio silently. No startup beep: sound is only produced by games.
  if (!EmuAudio::begin()) {
    Serial.printf("WARN: audio init failed (ES8311=%s, I2S=%s)\n",
                  EmuAudio::isCodecReady() ? "OK" : "NOT FOUND/INIT ERROR",
                  EmuAudio::isReady() ? "OK" : "FAILED");
  } else {
    Serial.println("ES8311 codec + I2S speaker output ready (startup tone disabled)");
  }
  if (EmuAudio::isReady()) Serial.println("Game audio generators ready");
  {
    static const char *const why[] = { "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
                                       "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE", "PWR_GLITCH", "CPU_LOCKUP" };
    const unsigned r = (unsigned)esp_reset_reason();
    Serial.printf("Reset reason: %s (%u)\n", r < 16 ? why[r] : "?", r);   // PANIC / *_WDT / BROWNOUT = a crash
  }
  for (int i = 0; i < 8; i++) if (padPins[i] >= 0) pinMode(padPins[i], INPUT_PULLUP);
  pinMode(PAIR_PIN, INPUT_PULLUP);
  ledInit();

  // Bring up the display before touching SDMMC/BLE or allocating emulator
  // resources. If the card is absent or SDMMC initialization fails, the user
  // still gets a visible screen instead of a black backlight/display.
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  Serial.printf("Memory at boot: internal free=%u, PSRAM %s (%u bytes free)\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                psramFound() ? "ENABLED" : "NOT enabled (Tools > PSRAM > OPI PSRAM)", (unsigned)ESP.getFreePsram());
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
  Serial.println("SD: initialization finished");
  coreInitializeBuiltins();
  startGame(true);
}

void loop() {
  // Optional quiet audio-path test: send "t" in Serial Monitor at 115200 baud.
  // Never play a startup beep automatically.
  if (Serial.available() > 0) {
    const int audioCmd = Serial.read();
    if (audioCmd == 't' || audioCmd == 'T') EmuAudio::testTone();
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
  bleTick();

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
  coreFrame(curSys, draw);
  EmuSound::frame(curSys, frameUs);
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
