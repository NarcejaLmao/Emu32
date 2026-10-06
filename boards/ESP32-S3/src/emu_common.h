// =====================================================================================
//  emu_common.h - ESP32-S3-LCD-1.47 board pins, screen geometry and shared helpers
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <SD.h>

extern Arduino_GFX *panel;

// ---- Waveshare ESP32-S3-LCD-1.47 pins ----
// LCD: 172x320 ST7789
#define LCD_MOSI 45
#define LCD_SCLK 40
#define LCD_CS   42
#define LCD_DC   41
#define LCD_RST  39
#define LCD_BL   48

// TF/microSD is wired to the S3's SD signals.
// The firmware uses Arduino SD over SPI, so use CMD/D0/SCLK/CS as MOSI/MISO/CLK/CS.
#define SD_SCK   14
#define SD_MISO  16
#define SD_MOSI  15
#define SD_CS    21

#define BTN      0     // BOOT button
#define LED_PIN  38    // onboard WS2812 RGB LED

#define ROMS_DIR    "/roms"
#define BIOS_DIR    "/bios"
#define GB_BIOS_DIR "/bios/gb"

#define SW 320
#define SH 172
#define OUT_W 197
#define OUT_X0 ((SW - OUT_W) / 2)
#define C(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

// Shared plumbing used by emulator cores.
static uint8_t *romBuf = nullptr;
static const char *romError = nullptr;
static char romErrBuf[48];

static bool sdMount() {
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 16000000)) {
    SPI.end();
    return false;
  }
  return true;
}
static void sdUnmount() { SD.end(); SPI.end(); }

// bit0 A, bit1 B, bit2 Select, bit3 Start, bit4 Up, bit5 Down, bit6 Left, bit7 Right
static volatile uint8_t padBits = 0;

#define BATCH_ROWS 8
static uint16_t batchBuf[OUT_W * BATCH_ROWS];
static int rowW = OUT_W, rowX0 = OUT_X0;
static int batchStart = 0, batchN = 0;

static inline void flushRows() {
  if (batchN) {
    panel->draw16bitRGBBitmap(rowX0, batchStart, batchBuf, rowW, batchN);
    batchN = 0;
  }
}
static inline uint16_t *rowSlot(int outRow) {
  if (batchN && outRow != batchStart + batchN) flushRows();
  if (!batchN) batchStart = outRow;
  return batchBuf + batchN * rowW;
}
static inline void rowDone() {
  if (++batchN >= BATCH_ROWS) flushRows();
}
