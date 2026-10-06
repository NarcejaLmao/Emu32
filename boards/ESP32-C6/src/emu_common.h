// =====================================================================================
//  emu_common.h  -  board pins, screen geometry and helpers shared by every Emu32 core
// =====================================================================================
//  Included by emu32.ino and by nes_core.h / gb_core.h.
//  Needs one thing from emu32.ino: the global `Arduino_GFX *panel` (declared extern below).
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <SD.h>

extern Arduino_GFX *panel;      // the LCD, created in emu32.ino

// ---- Waveshare ESP32-C6-LCD-1.47 pins ----
#define LCD_MOSI 6
#define LCD_SCLK 7
#define LCD_CS   14
#define LCD_DC   15
#define LCD_RST  21
#define LCD_BL   22
#define SD_SCK   7
#define SD_MISO  5
#define SD_MOSI  6
#define SD_CS    4
#define BTN      9    // BOOT button
#define LED_PIN  8    // onboard WS2812 RGB LED

// ---- SD card layout:  /roms/<system>/  for games,  /bios/<system>/  for BIOS / boot ROM files ----
#define ROMS_DIR    "/roms"
#define BIOS_DIR    "/bios"
#define GB_BIOS_DIR "/bios/gb"        // dmg_boot.bin (256 bytes, optional)

#define SW 320
#define SH 172
#define OUT_W 197                      // NES 256x224 scaled to fit 172 px tall keeping the aspect ratio
#define OUT_X0 ((SW - OUT_W) / 2)
#define C(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

// =====================================================================================
//  Shared plumbing used by every emulator core (nes_core.h and gb_core.h)
// =====================================================================================
static uint8_t *romBuf = nullptr;           // the loaded ROM file
static const char *romError = nullptr;      // set by the loaders when a ROM can't be used
static char romErrBuf[48];

// The SD card shares the SPI pins with the LCD, so every SD access (game list scan, ROM load) happens
// BEFORE the LCD is started, and the SD bus is released again before panel->begin().
static bool sdMount() {
  pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 16000000)) { SPI.end(); return false; }
  return true;
}
static void sdUnmount() { SD.end(); SPI.end(); }

// Controller input (filled by the Bluetooth code)
// bit0 A, bit1 B, bit2 Select, bit3 Start, bit4 Up, bit5 Down, bit6 Left, bit7 Right
static volatile uint8_t padBits = 0;

// LCD output: rows are collected and sent in batches
#define BATCH_ROWS 8                 // LCD rows sent per SPI transfer (one window setup instead of eight)
static uint16_t batchBuf[OUT_W * BATCH_ROWS];
static int rowW = OUT_W, rowX0 = OUT_X0;            // width / left edge of the picture being drawn (set once in setup())
static int batchStart = 0, batchN = 0;

static inline void flushRows() {
  if (batchN) { panel->draw16bitRGBBitmap(rowX0, batchStart, batchBuf, rowW, batchN); batchN = 0; }
}
static inline uint16_t *rowSlot(int outRow) {            // where to write the next LCD row
  if (batchN && outRow != batchStart + batchN) flushRows();
  if (!batchN) batchStart = outRow;
  return batchBuf + batchN * rowW;
}
static inline void rowDone() { if (++batchN >= BATCH_ROWS) flushRows(); }

