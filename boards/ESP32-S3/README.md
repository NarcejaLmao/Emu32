# Emu32 - Waveshare ESP32-S3-LCD-1.47

ESP32-S3 port of the Emu32 firmware originally targeting the Waveshare ESP32-C6-LCD-1.47.

## Board
- Waveshare ESP32-S3-LCD-1.47
- ST7789 LCD, 172x320
- Onboard TF/microSD
- Onboard WS2812 RGB LED
- BOOT button on GPIO0 — controller pairing only

## Arduino board settings
- Board: **ESP32S3 Dev Module**
- USB CDC On Boot: **Enabled**
- Flash/PSRAM: use the board's available/default configuration

## Libraries
- GFX Library for Arduino
- NimBLE-Arduino 2.x

## Storage
FAT32 microSD:
- `/roms/nes/*.nes`
- `/roms/gb/*.gb` or `*.gbc`
- optional `/bios/gb/dmg_boot.bin`

## S3 pin mapping
LCD: MOSI 45, SCLK 40, CS 42, DC 41, RST 39, BL 48  
TF/SD SPI: SCK 14, MOSI 15, MISO 16, CS 21  
RGB LED: 38  
BOOT: 0

The emulator cores (`nes_core.h` and `gb_core.h`) are unchanged from the C6 version.

## BOOT button
The physical BOOT button is reserved exclusively for Bluetooth controller pairing. It does not restart the console, quit games, close menus, or perform any other UI action.
