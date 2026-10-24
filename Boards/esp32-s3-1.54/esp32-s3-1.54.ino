// ============================================================================
//  Emu32 - from-scratch NES + Game Boy + Game Boy Color + Atari 2600 emulators for the ESP32-S3
//  Written without using any third-party emulator code.
//
//  Target board : ESP32-S3-LCD-1.54 / ESP32-S3-Touch-LCD-1.54 (JD9853/ST7789-compatible 240x240 LCD + touch + microSD)
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
// Waveshare ESP32-S3-LCD-1.54 battery power latch:
//   PWR button = GPIO5 (active LOW), battery power hold = GPIO2 (HIGH keeps power on).
#define PIN_PWR_BUTTON 5
#define PIN_PWR_HOLD   2

static void initHardwarePowerLatch() {
  // Must happen early, or battery power may drop when PWR is released at boot.
  pinMode(PIN_PWR_HOLD, OUTPUT);
  digitalWrite(PIN_PWR_HOLD, HIGH);
}

static void powerOffFromButton() {
  Serial.println("PWR: button pressed; shutting down");
  digitalWrite(EmuAudio::PIN_AMP, LOW);
  digitalWrite(PIN_LCD_BL, LOW);
  delay(80);
  // Release the board's battery power latch. A press of the same physical
  // PWR button should re-enable the latch through the board's hardware.
  digitalWrite(PIN_PWR_HOLD, LOW);
  delay(100);
  while (true) delay(1000);
}

static void pollPhysicalPowerButton() {
  // Power button is active-low on GPIO5. This is called from a dedicated
  // FreeRTOS task so it remains responsive even while an emulator core is busy.
  static bool armed = false;
  static bool wasReleased = true;
  static uint32_t pressedAt = 0;
  const bool released = digitalRead(PIN_PWR_BUTTON) == HIGH;
  const uint32_t now = millis();

  // Ignore the press used to start the board; arm after a confirmed release.
  if (!armed) {
    if (released) {
      armed = true;
      wasReleased = true;
    }
    return;
  }

  if (released) {
    wasReleased = true;
    pressedAt = 0;
    return;
  }

  if (wasReleased) {
    wasReleased = false;
    pressedAt = now;
    return;
  }

  // A deliberate hold avoids accidental shutdown from button bounce.
  if (pressedAt != 0 && now - pressedAt >= 700) {
    powerOffFromButton();
  }
}

static void physicalPowerButtonTask(void *parameter) {
  (void)parameter;
  for (;;) {
    pollPhysicalPowerButton();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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
#define EMU32_SETTINGS_FS SD_MMC
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
#include "src/genesis_core.h"
#include "src/core_manager.h"
#include "src/audio_synth.h"

// ============================================================================
//  Setup / loop
// ============================================================================
static uint32_t nextFrame = 0, frameUs = 16639;
static uint8_t  skipped = 0, frameCount = 0;
// Optional frame profiler: send 'p' over Serial at 115200 to toggle one-second
// core/audio timing reports. Disabled by default; no timing calls in the hot path.
static bool profileFrames = false;
static uint32_t profCoreSum = 0, profCoreMax = 0, profAudioSum = 0, profAudioMax = 0;
static uint16_t profSamples = 0;

// Frameskip (in-game menu): 0 = OFF (every frame drawn), 1 = AUTO (skip up to 3 frames in a
// row when running late), 2 = draw 1 frame in 2, 3 = draw 1 frame in 3.
static uint8_t  fsMode = 1, fsCnt = 0;
static const char *const fsName[4] = { "OFF", "AUTO", "1 IN 2", "1 IN 3" };
static bool showFps = SHOW_FPS;                  // toggled from the in-game menu
#define EMU32_SETTINGS_HAS_AUDIO 1

static uint8_t settingsVolume = 50;
static bool settingsLoaded = false;

static void settingsSave() {
  if (!sdMount()) return;
  if (!EMU32_SETTINGS_FS.exists("/system")) EMU32_SETTINGS_FS.mkdir("/system");
  const char *settingsTmp = "/system/settings.tmp";
  const char *settingsDst = "/system/settings.cfg";
  const char *settingsBak = "/system/settings.bak";
  EMU32_SETTINGS_FS.remove(settingsTmp);
  File f = EMU32_SETTINGS_FS.open(settingsTmp, FILE_WRITE);
  if (!f) return;
  const int expected = f.printf("volume=%u\nframeskip=%u\nfps=%u\nrotation=%u\n",
           (unsigned)settingsVolume, (unsigned)fsMode, showFps ? 1U : 0U,
           (unsigned)(lcdRot & 3));
  f.flush();
  f.close();
  // Never promote a partial settings file (for example when the SD card is full).
  File verify = EMU32_SETTINGS_FS.open(settingsTmp, FILE_READ);
  const bool verified = expected > 0 && verify && verify.size() == (size_t)expected;
  if (verify) verify.close();
  if (!verified) {
    EMU32_SETTINGS_FS.remove(settingsTmp);
    Serial.println("SETTINGS: temporary file verification failed; keeping previous settings");
    return;
  }

  // Keep the previous settings until the verified replacement is installed.
  const bool hadSettings = EMU32_SETTINGS_FS.exists(settingsDst);
  if (hadSettings) {
    EMU32_SETTINGS_FS.remove(settingsBak);
    if (!EMU32_SETTINGS_FS.rename(settingsDst, settingsBak)) {
      EMU32_SETTINGS_FS.remove(settingsTmp);
      return;
    }
  }
  if (!EMU32_SETTINGS_FS.rename(settingsTmp, settingsDst)) {
    if (hadSettings) EMU32_SETTINGS_FS.rename(settingsBak, settingsDst);
    EMU32_SETTINGS_FS.remove(settingsTmp);
    return;
  }
  // If the primary was missing, retain any backup until the new primary is safe.
  EMU32_SETTINGS_FS.remove(settingsBak);
}

static void settingsLoad() {
  if (settingsLoaded) return;
  if (!sdMount()) return;
  settingsLoaded = true;
  File f = EMU32_SETTINGS_FS.open("/system/settings.cfg", FILE_READ);
  // A reset between the two rename operations can leave only settings.bak.
  // Restore it before falling back to defaults.
  if (!f && EMU32_SETTINGS_FS.exists("/system/settings.bak")) {
    if (EMU32_SETTINGS_FS.rename("/system/settings.bak", "/system/settings.cfg"))
      f = EMU32_SETTINGS_FS.open("/system/settings.cfg", FILE_READ);
    if (!f) f = EMU32_SETTINGS_FS.open("/system/settings.bak", FILE_READ);
  }
  if (!f) {
    #if EMU32_SETTINGS_HAS_AUDIO
      EmuAudio::setVolume(settingsVolume);
    #endif
    return;
  }
  // Settings files are tiny. Bound both total input and each line so a
  // corrupted file cannot make String allocations grow without limit.
  static const size_t SETTINGS_MAX_BYTES = 512;
  static const size_t SETTINGS_LINE_CAP = 64;
  char line[SETTINGS_LINE_CAP];
  size_t lineLen = 0;
  size_t bytesRead = 0;
  bool lineOverflow = false;
  while (f.available() && bytesRead < SETTINGS_MAX_BYTES) {
    const int ch = f.read();
    if (ch < 0) break;
    ++bytesRead;
    if (ch == '\n') {
      if (!lineOverflow) {
        line[lineLen] = '\0';
        char *start = line;
        while (*start == ' ' || *start == '\t' || *start == '\r') ++start;
        size_t len = strlen(start);
        while (len && (start[len - 1] == ' ' || start[len - 1] == '\t' || start[len - 1] == '\r')) start[--len] = '\0';
        if (len && start[0] != '#') {
          char *eq = strchr(start, '=');
          if (eq && eq != start) {
            *eq++ = '\0';
            char *keyEnd = start + strlen(start);
            while (keyEnd > start && (keyEnd[-1] == ' ' || keyEnd[-1] == '\t')) *--keyEnd = '\0';
            while (*eq == ' ' || *eq == '\t') ++eq;
            char *valueEnd = eq + strlen(eq);
            while (valueEnd > eq && (valueEnd[-1] == ' ' || valueEnd[-1] == '\t' || valueEnd[-1] == '\r')) *--valueEnd = '\0';
            char *end = nullptr;
            const long value = strtol(eq, &end, 10);
            if (eq != end && end && *end == '\0') {
              if (!strcmp(start, "volume")) settingsVolume = (uint8_t)constrain(value, 0, 100);
              else if (!strcmp(start, "frameskip")) fsMode = (uint8_t)constrain(value, 0, 3);
              else if (!strcmp(start, "fps")) showFps = value != 0;
              else if (!strcmp(start, "rotation")) lcdRot = (uint8_t)constrain(value, 0, 3);
            }
          }
        }
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (!lineOverflow) {
      if (lineLen + 1 < sizeof(line)) line[lineLen++] = (char)ch;
      else { lineOverflow = true; lineLen = 0; }
    }
  }
  // Do not parse a partial final line or continue consuming a maliciously
  // large file. The fixed buffer bounds RAM regardless of file contents.
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

#define PAUSE_ITEMS 10
static char pauseStatus[28] = "";
static uint8_t stateSlot = 0;
static void pauseMenuDraw(int sel) {
  static const char *const items[PAUSE_ITEMS] = { "RESUME", "RESTART", "FRAMESKIP", "FPS COUNTER", "VOLUME", "ROTATE SCREEN", "STATE SLOT", "SAVE STATE", "LOAD STATE", "EXIT GAME" };
  const uint16_t white = col565(255, 255, 255), grey = col565(150, 150, 150), blue = col565(40, 90, 220);
  const bool land = LCD_W >= LCD_H;              // wide and square: full-screen pause menu
  const int y0 = land ? 2 : GAME_H + 8;
  const int first = land ? 15 : 18, pitch = 12, rowH = 11;
  lcdTextRow(y0, 16, "PAUSED", 2, grey, 0, (LCD_W - textW("PAUSED", 2)) / 2);
  for (int i = 0; i < PAUSE_ITEMS; i++) {
    const bool on = (i == sel);
    char line[28];
    if (i == 2)      snprintf(line, sizeof(line), "FRAMESKIP:%s", fsName[fsMode]);
    else if (i == 3) snprintf(line, sizeof(line), "FPS COUNTER:%s", showFps ? "ON" : "OFF");
    else if (i == 4) snprintf(line, sizeof(line), "VOLUME:%u%%", (unsigned)EmuAudio::getVolume());
    else if (i == 6) snprintf(line, sizeof(line), "STATE SLOT:%u", (unsigned)(stateSlot + 1));
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
      else if (sel == 4) {
        int volume = (int)EmuAudio::getVolume() + (dir < 0 ? -5 : 5);
        if (dir == 0) volume = volume >= 100 ? 0 : volume + 10;
        EmuAudio::setVolume((uint8_t)constrain(volume, 0, 100));
        settingsVolume = EmuAudio::getVolume();
        settingsSave();
        redraw = true;
      }
      else if (sel == 5) { lcdSetRotation(lcdRot + (dir < 0 ? 3 : 1)); redraw = true; }   // screen already blanked
      else if (sel == 6) { stateSlot = (stateSlot + (dir < 0 ? 2 : 1)) % 3; redraw = true; }
      else if (sel == 7 && !dir) {
        bool ok = coreSaveState(curSys, stateSlot);
        if (ok) strlcpy(pauseStatus, "STATE SAVED", sizeof(pauseStatus));
        else if (sdSaveLastError[0]) strlcpy(pauseStatus, sdSaveLastError, sizeof(pauseStatus));
        else strlcpy(pauseStatus, "SAVE STATE FAILED", sizeof(pauseStatus));
        redraw = true;
      }
      else if (sel == 8 && !dir) {
        bool ok = coreLoadState(curSys, stateSlot);
        if (ok) strlcpy(pauseStatus, "STATE LOADED", sizeof(pauseStatus));
        else if (sdSaveLastError[0]) strlcpy(pauseStatus, sdSaveLastError, sizeof(pauseStatus));
        else strlcpy(pauseStatus, "NO VALID STATE", sizeof(pauseStatus));
        redraw = true;
      }
      else if (sel == 9 && !dir) { done = true; left = true; }
    }
    const int sig = sel | (fsMode << 4) | (showFps << 8) | (stateSlot << 9) | ((int)EmuAudio::getVolume() << 13) | (pauseStatus[0] ? 0x1000 : 0);
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
  initHardwarePowerLatch();
  pinMode(PIN_PWR_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // With USB CDC, a powered cable with no serial monitor reading (a charger, or a PC with the port
  // closed) fills the TX buffer and every Serial.print then blocks for the TX timeout. That stalls
  // the emulator and makes the device crawl whenever USB is plugged in. Never block on USB writes.
  Serial.setTxTimeoutMs(0);
#endif
  delay(250);
  // Some ESP32-S3 Arduino builds enable dynamic CPU-frequency scaling or boot
  // with a reduced clock. The emulator's cycle loops and software audio timing
  // assume the board is running at its full 240 MHz. Restore that clock on every
  // reset, before initializing peripherals or any emulator core.
  setCpuFrequencyMhz(240);
  Serial.printf("CPU clock: %u MHz\n", (unsigned)getCpuFrequencyMhz());
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
  // Monitor PWR independently of the emulator loop, which can be busy in cores.
  const BaseType_t pwrTaskCreated = xTaskCreatePinnedToCore(
      physicalPowerButtonTask, "pwr_button", 2048, nullptr, 3, nullptr, 0);
  Serial.printf("PWR button task: %s\n", pwrTaskCreated == pdPASS ? "started" : "FAILED");
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
  // Physical power button is monitored by physicalPowerButtonTask.
  // Optional quiet audio-path test: send "t" in Serial Monitor at 115200 baud.
  // Never play a startup beep automatically.
  if (Serial.available() > 0) {
    const int audioCmd = Serial.read();
    if (audioCmd == 't' || audioCmd == 'T') EmuAudio::testTone();
    else if (audioCmd == 'p' || audioCmd == 'P') { profileFrames = !profileFrames; profCoreSum = profCoreMax = profAudioSum = profAudioMax = 0; profSamples = 0; Serial.printf("PROFILE: %s\n", profileFrames ? "ON (core/audio us)" : "OFF"); }
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
  uint32_t profT = 0, profElapsed = 0;
  if (profileFrames) profT = micros();
  coreFrame(curSys, draw);
  if (profileFrames) { profElapsed = micros() - profT; profCoreSum += profElapsed; if (profElapsed > profCoreMax) profCoreMax = profElapsed; profT = micros(); }
  EmuSound::frame(curSys, frameUs);
  if (profileFrames) { profElapsed = micros() - profT; profAudioSum += profElapsed; if (profElapsed > profAudioMax) profAudioMax = profElapsed; if (++profSamples >= 60) { Serial.printf("PROFILE %s: core avg=%lu max=%lu us | audio avg=%lu max=%lu us\n", curSys == SYS_NES ? "NES" : curSys == SYS_GB ? "GB" : curSys == SYS_GBC ? "GBC" : curSys == SYS_A2600 ? "A2600" : "GEN", (unsigned long)(profCoreSum / profSamples), (unsigned long)profCoreMax, (unsigned long)(profAudioSum / profSamples), (unsigned long)profAudioMax); profCoreSum = profCoreMax = profAudioSum = profAudioMax = 0; profSamples = 0; } }
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
