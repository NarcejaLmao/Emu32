# Emu32 - Waveshare ESP32-C6-LCD-1.47

ESP32-C6 firmware for the Waveshare ESP32-C6-LCD-1.47, running the Emu32 retro gaming firmware.

## Board
- Waveshare ESP32-C6-LCD-1.47
- ESP32-C6 RISC-V processor
- ST7789 LCD, 172x320
- Onboard TF/microSD
- Onboard WS2812 RGB LED
- BOOT button

## Supported Systems
- Nintendo Entertainment System (NES)
- Nintendo Game Boy
- Atari 2600

## Arduino board settings
- Board: **ESP32C6 Dev Module**
- USB CDC On Boot: **Enabled**

## Libraries
- GFX Library for Arduino
- NimBLE-Arduino 2.x

## Storage
FAT32 microSD:
- `/roms/nes/*.nes`
- `/roms/gb/*.gb` or `*.gbc`
- optional `/bios/gb/dmg_boot.bin`

## C6 pin mapping

### LCD
- MOSI: GPIO 6
- SCLK: GPIO 7
- CS: GPIO 14
- DC: GPIO 15
- RST: GPIO 21
- Backlight: GPIO 22

### TF / microSD
- SCK: GPIO 7
- MOSI: GPIO 6
- MISO: GPIO 5
- CS: GPIO 4

### Controls
- BOOT button: GPIO 9
- RGB LED: GPIO 8

## Display
- Controller: ST7789
- Resolution: 172x320
- Landscape mode: 320x172
- NES output is scaled to fit the display while preserving its aspect ratio.

The emulator cores are shared between the supported Emu32 boards; this folder contains the ESP32-C6-specific board configuration.
