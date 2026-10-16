// ============================================================================
//  Emu32 - from-scratch NES + Game Boy + Game Boy Color + Atari 2600 emulators for the ESP32-C6
//  Written without using any third-party emulator code.
//
//  Target board : Waveshare ESP32-C6-LCD-1.47 (ST7789 172x320 + microSD)
//  Orientation  : starts portrait 172x320 (top toward the USB-C port); SELECT+RB cycles all 4 orientations
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
//                 RESUME / RESTART / FRAMESKIP / FPS COUNTER / ROTATE SCREEN / EXIT GAME (back to the game list)
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
#include "src/logo.h"        // boot splash logo

#define NES_DIR       "/roms/nes"
#define GB_DIR        "/roms/gb"
#define GBC_DIR       "/roms/gbc"
#define A2600_DIR     "/roms/a2600"
#define MAX_ROMS      128        // max files listed in the menu
#define ROM_NAME_LEN  80         // longer file names are skipped
#define MENU_ROW_H    15
#define MENU_TOP      20
#define MENU_ROWS     ((LCD_H - MENU_TOP) / MENU_ROW_H)   // 20 in portrait, 10 in landscape
#define SHOW_FPS      1          // default for the in-game menu's FPS COUNTER item (1 = on, 0 = off)

// ---------------------------------------------------------------- pins ------
#define PIN_SCLK      7
#define PIN_MOSI      6
#define PIN_MISO      5
#define PIN_LCD_CS    14
#define PIN_LCD_DC    15
#define PIN_LCD_RST   21
#define PIN_LCD_BL    22
#define PIN_SD_CS     4
#define LCD_SPI_HZ    80000000   // lower to 40000000 if the picture glitches

// Display geometry. The panel is 172 x 320; the screen can be used in 4 orientations
// (SELECT+RB cycles them). LCD_W / LCD_H / GAME_X / GAME_Y follow the current one:
//   portrait 172 x 320 (rotations 0 and 2), landscape 320 x 172 (rotations 1 and 3).
#define LCD_SHORT     172
#define LCD_LONG      320
#define LCD_XOFF      34         // ST7789 RAM offset on the panel's short axis (same in both directions:
                                 // the 172 px panel is centred in the 240 px RAM)
#define LCD_YOFF      0          // no offset on the long axis
// MADCTL per orientation, each step = 90 deg clockwise. Entry 0 is the original portrait
// (MY|MX); if the picture appears upside down at boot, start with lcdRot = 2.
static const uint8_t lcdMadctl[4] = { 0xC0, 0xA0, 0x00, 0x60 };
static uint8_t lcdRot = 0;
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
#include "src/bios.h"
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

// Startup splash: the Emu32 logo (src/logo.h) with the subtitle lines below it.
static void splash() {
  const char *sub1 = "NES GB GBC 2600", *sub2 = "FOR ESP32-C6";
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
  if (curSys == SYS_GB)          gbShutdown();
  else if (curSys == SYS_GBC)    gbcShutdown();
  else if (curSys == SYS_A2600)  a26Shutdown();
  else                           nesShutdown();
  Serial.printf("Memory after game cleanup: free=%u, min=%u bytes\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}

static void startGame(bool toMainMenu) {
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
    if (!inList) { curSys = sysMenu(); inList = true; }

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
    freeRomList();                               // give the heap back before loading the ROM

    lcdClearTransition(0);
    lcdTextRow(LCD_H / 2 - 10, 20, "LOADING ROM", 2, col565(255, 255, 255), 0);
    Serial.printf("Memory before ROM load: free=%u, min=%u bytes\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    sdDiagReset(path);
    const bool ok = (curSys == SYS_NES) ? nesLoad(path)
                  : (curSys == SYS_A2600) ? a26Load(path)
                  : (curSys == SYS_GBC) ? gbcLoad(path) : gbLoad(path);
    if (ok) {
      Serial.printf("Memory after ROM load: free=%u, min=%u bytes\n",
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
      break;
    }

    // load failed: show why and return to the game list
    sdDiagSummary("load-failed");
    Serial.printf("ERROR: %s / %s\n", loadErr1, loadErr2);
    showMessage(loadErr1, loadErr2, col565(110, 0, 0));
    delay(2500);
  }

  if (curSys == SYS_NES) {
    nesReset();                                  // NES streams mapped banks from SD
    frameUs = 16639;                             // 60.1 Hz
  } else if (curSys == SYS_A2600) {
    sdUnmount();                                 // 2600 ROM is fully in RAM now
    a26Reset();
    frameUs = a26FrameUs;                        // 60 Hz NTSC; switches to 50 Hz for PAL games
  } else if (curSys == SYS_GBC) {
    gbcReset();                                  // SD stays mounted: saves + ROM paging
    frameUs = 16742;                             // 70224 dots per frame = 59.73 Hz, also in double speed
  } else {
    gbReset();                                   // SD stays mounted: saves + ROM streaming
    frameUs = 16742;                             // 4194304 Hz / 70224 clocks = 59.73 Hz
  }
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
  if (curSys == SYS_NES)        { mapperInit(); nesReset(); }
  else if (curSys == SYS_A2600) a26Reset();
  else if (curSys == SYS_GBC)   gbcReset();
  else                          gbReset();
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

#define PAUSE_ITEMS 6
static void pauseMenuDraw(int sel) {
  static const char *const items[PAUSE_ITEMS] = { "RESUME", "RESTART", "FRAMESKIP", "FPS COUNTER", "ROTATE SCREEN", "EXIT GAME" };
  const uint16_t white = col565(255, 255, 255), grey = col565(150, 150, 150), blue = col565(40, 90, 220);
  const bool land = LCD_W > LCD_H;
  const int y0 = land ? 2 : GAME_H + 8;
  const int first = land ? 18 : 24, pitch = land ? 20 : 22, rowH = land ? 18 : 20;
  lcdTextRow(y0, 16, "PAUSED", 2, grey, 0, (LCD_W - textW("PAUSED", 2)) / 2);
  for (int i = 0; i < PAUSE_ITEMS; i++) {
    const bool on = (i == sel);
    char line[28];
    if (i == 2)      snprintf(line, sizeof(line), "FRAMESKIP:%s", fsName[fsMode]);
    else if (i == 3) snprintf(line, sizeof(line), "FPS COUNTER:%s", showFps ? "ON" : "OFF");
    else             snprintf(line, sizeof(line), "%s", items[i]);
    lcdTextRow(y0 + first + i * pitch, rowH, line, 2, on ? white : grey, on ? blue : 0, 6);
  }
  lcdTextRow(LCD_H - (land ? 14 : 18), 14, "A:OK  SEL:RESUME", 1, col565(90, 90, 90), 0, 8);
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
  int lastSig = sel | (fsMode << 4) | (showFps << 8);
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
      else if (sel == 5 && !dir) { done = true; left = true; }
    }
    const int sig = sel | (fsMode << 4) | (showFps << 8);
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

  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  sdInitFolders();                              // create ROM/BIOS directories on first boot
  nesInit();
  a26Init();
  lcdInit();
  splash();
  startGame(true);
}

void loop() {
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
  if (combo && !comboWas) { comboWas = true; pauseMenu(); return; }
  comboWas = combo;

  rotateHotkey();                                // SELECT+RB: next orientation (the next drawn frame repaints)

  // Hold SELECT+START (or A+B+SELECT, for pads without START) for ~1 s to quit.
  static uint8_t quitFrames = 0;
  if (((padState & 0x0C) == 0x0C) || ((padState & 0x07) == 0x07)) {
    if (++quitFrames >= 60) { quitFrames = 0; exitToMenu(); return; }
  } else quitFrames = 0;

  if (curSys == SYS_NES) {
    nesFrame(draw);
    if (nesRomReadError) {
      strlcpy(loadErr1, sdIoFault ? "SD CARD LOST" : "ROM READ ERROR", sizeof(loadErr1));
      strlcpy(loadErr2, sdIoFault ? "REINSERT CARD" : "CHECK SD CARD", sizeof(loadErr2));
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(1200);
      exitToMenu();
      return;
    }
  } else if (curSys == SYS_A2600) {
    a26Frame(draw);
    frameUs = a26FrameUs;                        // follows NTSC / PAL detection
  } else if (curSys == SYS_GBC) {
    gbcFrame(draw);
    if (gcRomReadError) {
      strlcpy(loadErr1, sdIoFault ? "SD CARD LOST" : "ROM READ ERROR", sizeof(loadErr1));
      strlcpy(loadErr2, sdIoFault ? "REINSERT CARD" : "CHECK SD CARD", sizeof(loadErr2));
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(1200);
      exitToMenu();
      return;
    }
  } else {
    gbFrame(draw);
    if (gRomReadError) {
      strlcpy(loadErr1, sdIoFault ? "SD CARD LOST" : "ROM READ ERROR", sizeof(loadErr1));
      strlcpy(loadErr2, sdIoFault ? "REINSERT CARD" : "CHECK SD CARD", sizeof(loadErr2));
      showMessage(loadErr1, loadErr2, col565(110, 0, 0));
      delay(1200);
      exitToMenu();
      return;
    }
  }
  fpsTick(draw);

  nextFrame += frameUs;
  int32_t diff = (int32_t)(nextFrame - micros());
  if (diff > 1000) delay(diff / 1000);
  if (diff > 0) { while ((int32_t)(nextFrame - micros()) > 0) { } }
  else if (diff < -100000) nextFrame = micros();     // fell far behind: resync
  if ((++frameCount & 31) == 0) delay(1);            // let the idle task / watchdog run
}
