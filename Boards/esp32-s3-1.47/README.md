# Emu32 — Waveshare ESP32-S3-LCD-1.47 port

Port of the supplied ESP32-C6 firmware to the Waveshare **ESP32-S3-LCD-1.47**
(ST7789, 172×320, onboard TF/microSD). The `1.47B` revision uses a different
backlight GPIO; this port targets the original non-B ESP32-S3-LCD-1.47 board.

## Arduino IDE

- Board: **Waveshare ESP32-S3-LCD-1.47** (install/update Espressif ESP32 boards package)
- USB CDC On Boot: Enabled
- Flash size: 16 MB (if exposed by the board menu)
- PSRAM: enable OPI PSRAM if exposed by the board menu
- Install **NimBLE-Arduino 2.x**
- Open `esp32-s3-1.47.ino` from this folder and build/upload.

## Hardware changes made

- LCD SPI: SCLK GPIO40, MOSI GPIO45, CS GPIO42, DC GPIO41, reset GPIO39,
  backlight GPIO48.
- microSD: separate SPI bus using SCLK GPIO14, MOSI/CMD GPIO15, MISO/D0 GPIO16,
  CS/D3 GPIO21. This avoids reconfiguring the LCD SPI bus while games render.
- The 8 optional external button inputs are moved away from SD pins:
  A=1, B=2, SELECT=3, UP=10, DOWN=11, LEFT=12, RIGHT=13. START remains
  disabled by default. Inputs are active-low; connect each button between GPIO
  and GND. The onboard BOOT button (GPIO0) starts BLE pairing.

## Important

This port has been adapted at source level but has not been compiled or tested on
physical hardware in this environment. If your board is the **ESP32-S3-LCD-1.47B**
revision, change `PIN_LCD_BL` from GPIO48 to GPIO46. If SD mounting fails, verify
your board revision and Arduino-ESP32 core version; the onboard TF socket is
wired for SDIO but this firmware uses the board's shared card signals in SPI mode.
