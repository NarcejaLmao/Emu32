# Emu32 for the Waveshare ESP32-S3-Touch-LCD-1.47
Port of Emu32 (NES / Game Boy / Game Boy Color / Atari 2600) from the ESP32-C6-LCD-1.47 to the
**Waveshare ESP32-S3-Touch-LCD-1.47** (ESP32-S3R8, 16 MB flash, 1.47" ST7789 172x320 LCD, microSD, WS2812 LED).

## What changed vs. the C6 version
| Area | C6 build | S3 build |
|---|---|---|
| LCD pins | SCLK 7, MOSI 6, CS 14, DC 15, RST 21, BL 22 | SCLK 38, MOSI 39, CS 21, DC 45, RST 40, BL 46 |
| microSD | SPI (CS 4, shared bus) | **SDMMC** via `SD_MMC` - CLK 16, CMD 15, D0 17 (D1 18, D2 13, D3 14 for 4-bit; from the Touch-LCD-1.47 schematic) |
| BOOT / pair button | GPIO 9 | GPIO 0 |
| RGB LED | GPIO 8 | **none** (this board has no RGB LED; GPIO38 is the LCD clock) |
| Wired buttons | GPIO 1,2,3,10,11,18,19 | none by default (BLE pad). Edit `padPins[]` - see comments for pins to avoid |
| Arduino board | ESP32C6 Dev Module | **ESP32S3 Dev Module** |

The emulator cores, menus, save states and BLE code are unchanged. The ST7789-style init sequence (the JD9853 accepts it, as in Waveshare's own Arduino demo) and 34 px column offset are retained. This firmware targets the Touch-LCD-1.47 pinout; the non-touch LCD-1.47 uses a different pin map.

## Arduino IDE settings
- Board: **ESP32S3 Dev Module**, arduino-esp32 core **3.0.2 or newer**
- USB CDC On Boot: **Enabled**
- Flash Size: **16MB**, CPU Frequency: **240MHz**
- PSRAM: **OPI PSRAM** (the ESP32-S3R8 has 8 MB octal PSRAM). Buffers stay in fast internal RAM while it lasts; small ROM page caches may spill into PSRAM instead of failing with "NOT ENOUGH RAM". With PSRAM disabled, large ROMs can run out of internal RAM
- Libraries: **NimBLE-Arduino** (h2zero, v2.x); **PNGdec** optional for box art
- Sketch folder must be named `emu32` (same as `emu32.ino`); keep the `src/` folder inside it

## First-boot troubleshooting
- Picture upside down: `LCD_ROT_DEFAULT` is already 2 (correct for this board); set it to `0` to flip 180 degrees.
- Colours look negative: set `LCD_INVERT` to `1`. Red and blue swapped: toggle the 0x08 (BGR) bit in `lcdMadctl[]`.
- Picture glitches / noise: lower `LCD_SPI_HZ` (default is already 40 MHz; try 20000000).
- "NO SD CARD": make sure the card is FAT32 and fully inserted. The Touch-LCD-1.47 uses CLK GPIO16, CMD GPIO15, D0 GPIO17; the 4-bit pins are D1 GPIO18, D2 GPIO13, D3 GPIO14. The firmware starts at 20 MHz in 1-bit mode for reliability. Once stable, set `SD_1BIT 0` for the faster 4-bit bus.
- "NOT ENOUGH RAM": make sure Tools > PSRAM is **OPI PSRAM**. The serial monitor prints `PSRAM: ... free` and `Memory before ROM load` at boot/launch; send those lines if it still happens.
- Can't flash: hold BOOT, tap RESET, release BOOT.
- Backlight stays dark / blank screen: this firmware is for the **ESP32-S3-Touch-LCD-1.47** (JD9853 LCD + AXS5106L touch). The non-touch ESP32-S3-LCD-1.47 and the 1.47B use a different pin map and are not covered.
- Touch is not used by this firmware (pins reserved: SDA 42, SCL 41, RST 47, INT 48).

# Supported Cores
Nintendo Entertainment System
Nintendo Gameboy
Nintendo Gameboy Color
Atari 2600

# SD Format
SD card must be formatted to FAT32.

/roms/nes/     *.nes
/roms/gb/      *.gb, *.gbc
/roms/gbc/     *.gbc, *.gb
/roms/a2600/   *.a26, *.bin
/bios/gb/     *dmg_boot.bin
/bios/gbc/     *cgb_boot.bin

## BIOS checks before launch

Before starting a Game Boy or Game Boy Color ROM, the firmware checks the matching
`/bios/gb/` or `/bios/gbc/` folder for a supported boot ROM filename and validates
its size (256 bytes for GB, 2304 bytes for GBC). A supplied but invalid/unreadable
BIOS blocks launch and displays an error. BIOS files remain optional: if none is
present, the core launches using its built-in post-boot state. NES and Atari 2600
do not require external BIOS files.

# Bluetooth Pairing
Bluetooth pad: BLE HID only (e.g. Xbox, 8BitDo in BLE mode). Press BOOT (GPIO0) to pair.
The last paired pad is reconnected automatically: at startup, and then every few seconds in the menus, so just switch the pad on (not in pairing mode) whenever you want. A pad that drops during a game reconnects once you are back in the menus. Press BOOT only to pair a *new* pad (this forgets the old one).

# Controls
Menus: UP/DOWN move, A select, B back.

In game: hold SELECT+START for 1 s to quit. SELECT+RB rotates the screen. SELECT+X opens the pause menu (Xbox pad). SELECT+RT (right trigger) saves a screenshot as a 24-bit BMP to /screenshots on the SD card (/screenshots/<ROM name>_NNN.bmp); a short "SHOT NNN SAVED" label appears on screen but is not part of the image.

# Disclaimer
You must supply your own ROMs and BIOS files. Console names and games are trademarks of their owners and creators.


# Box Art

Place PNG cover images on the SD card in a system-specific folder. The filename
must match the ROM filename without its extension (including spaces and punctuation):

- `/imgs/nes/<ROM name>.png`
- `/imgs/gb/<ROM name>.png`
- `/imgs/gbc/<ROM name>.png`
- `/imgs/a2600/<ROM name>.png`

For example, `/roms/gb/Pokemon.gb` uses `/imgs/gb/Pokemon.png`. The browser
preserves the PNG's aspect ratio and scales it into the selected game's preview.
Use non-interlaced PNGs no wider than 320 pixels and no taller than 640 pixels.

PNG decoding uses the **PNGdec** Arduino library by Larry Bank. Install `PNGdec`
from Arduino IDE Library Manager to enable artwork rendering. If it is not installed,
the firmware still builds and displays a `PNGDEC NEEDED` placeholder.

## Save states

The pause menu provides **SAVE STATE**, **LOAD STATE**, and a three-slot
**STATE SLOT** selector for NES, Game Boy, Game Boy Color, and Atari 2600.
State files are stored separately from battery saves in `/states/nes/`,
`/states/gb/`, `/states/gbc/`, and `/states/a2600/`. ROM-path-based names avoid
collisions between games with the same filename in different directories.
NES snapshots preserve CPU/PPU state, RAM, mapper bank mappings, and CHR RAM
when present. Atari 2600 snapshots preserve CPU, RIOT/TIA state, cartridge bank
mapping, cartridge RAM, and framebuffer. State writes use a temporary file and
keep a backup; the state format validates system and payload sizes.

Save-state compatibility across future firmware/core versions is not
guaranteed. Battery saves remain separate and should still be used for normal
long-term game progress.


## SD-card reliability improvements

- Save and state writes are mounted on demand and report card-unavailable errors.
- Short writes, failed verification, and failed rename/promote operations set an SD I/O fault and print serial diagnostics.
- Save-state writes verify the complete temporary file before rotating the previous state to `.bak`; failed promotion attempts restore the backup.
- State loads validate the header, system, core/memory sizes, and exact file length; a valid `.bak` is tried when the primary state is missing or corrupt.
- The pause menu now displays the specific save/state failure reason instead of only a generic failure message.
- Battery-save reads still treat missing save files as a normal first-run condition; invalid-size files are skipped in favor of a backup.

Hardware validation is still required to confirm card-removal behavior on the target board and SD module.

## Better core architecture

- Added `src/core_manager.h` as a single frontend adapter for core initialization, ROM loading, reset/shutdown, frame execution, save-state operations, frame pacing, and ROM-read fault reporting.
- Added a `CoreDescriptor` table to keep each system's identity, default frame period, and SD-streaming behavior in one place.
- Simplified the main loop and pause-menu save/load paths so they no longer duplicate system-specific dispatch logic.
- Kept each emulator's CPU, graphics, cartridge, memory, and timing implementation in its own core header. This is a low-risk architectural boundary for future subsystem extraction without changing emulation behavior in this pass.

The project ZIP was checked for archive integrity. A full Arduino/ESP32 compile and hardware regression test were not available in this environment.

## ROMs always run from the SD card

No core copies a whole ROM into RAM any more. NES, Game Boy, Game Boy Color and Atari 2600 keep the ROM file
open on the card and stream 8 KB / 16 KB / 4 KB / 1 KB pages on demand into a small fixed cache
(NES 4 PRG + 8 CHR banks, GB 4 banks, GBC 16 pages, Atari 16 pages). The card stays mounted during play, and a
read failure shows "ROM READ ERROR / SD CARD LOST" instead of running on corrupt data.
