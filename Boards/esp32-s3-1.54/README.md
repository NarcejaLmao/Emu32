# Emu32 for the ESP32-S3 1.54-inch 240×240 LCD Development Board
Port of Emu32 (NES / Game Boy / Game Boy Color / Atari 2600 / Sega Genesis) for the
**ESP32-S3 1.54-inch LCD Development Board** (square 240×240 display). Confirm the exact board revision and schematic before relying on GPIO assignments; similarly named ESP32-S3 display boards can use different LCD, SD, touch, and audio wiring.

## Board configuration

Pin map (LCD pins from Waveshare's factory demo BSP for the ESP32-S3-LCD-1.54 / Touch-LCD-1.54; SD pins verified on hardware):

| Function | GPIO |
|---|---|
| LCD SCLK / MOSI | 38 / 39 |
| LCD DC / CS / RST | 45 / 21 / 40 |
| LCD backlight (active HIGH) | 46 |
| microSD (SDMMC) CLK / CMD / D0 / D1 / D2 / D3 | 16 / 15 / 17 / 18 / 13 / 14 |
| I2C (codecs, IMU, touch) SDA / SCL | 42 / 41 |
| I2S MCLK / BCLK / LRCK / DIN / DOUT, amp enable | 8 / 9 / 10 / 11 / 12, 7 |
| BOOT button | 0 |
| PWR button / battery power hold | 5 / 2 |

The LCD is an ST7789 IPS panel, so the driver sends INVON (`LCD_INVERT 1`) and uses RGB colour order.
Flipped orientations use an 80-pixel window offset because the panel is 240x240 inside 240x320 controller RAM.

## What changed vs. the C6 version
| Area | C6 build | S3 build |
|---|---|---|
| LCD pins | SCLK 7, MOSI 6, CS 14, DC 15, RST 21, BL 22 | SCLK 38, MOSI 39, CS 21, DC 45, RST 40, BL 46 |
| microSD | SPI (CS 4, shared bus) | **SDMMC** via `SD_MMC` - CLK 16, CMD 15, D0 17 (D1 18, D2 13, D3 14) |
| BOOT / pair button | GPIO 9 | GPIO 0 |
| RGB LED | GPIO 8 | **none** (this board has no RGB LED; the LCD uses GPIO38/39/40/45/46/21) |
| Wired buttons | GPIO 1,2,3,10,11,18,19 | none by default (BLE pad). Edit `padPins[]` - see comments for pins to avoid |
| Arduino board | ESP32C6 Dev Module | **ESP32S3 Dev Module** |

The emulator cores, menus, save states and BLE code are carried over. The sketch and folder are named `esp32-s3-1.54`. Touch and dual microphones are not currently integrated by the Emu32 frontend; board-specific pin assignments must be checked against the exact hardware revision.

## Arduino IDE settings
- Board: **ESP32S3 Dev Module**, arduino-esp32 core **3.0.2 or newer**
- USB CDC On Boot: **Enabled**
- Flash Size: **16MB**, CPU Frequency: **240MHz**
- PSRAM: **OPI PSRAM** (the ESP32-S3R8 has 8 MB octal PSRAM). Buffers stay in fast internal RAM while it lasts; small ROM page caches may spill into PSRAM instead of failing with "NOT ENOUGH RAM". With PSRAM disabled, large ROMs can run out of internal RAM
- Libraries: **NimBLE-Arduino** (h2zero, v2.x); **PNGdec** optional for box art
- For Arduino IDE, open `esp32-s3-1.54/esp32-s3-1.54.ino` from the matching `esp32-s3-1.54` folder and keep the `src/` folder alongside it. If your Arduino IDE version requires the sketch-folder and primary `.ino` basenames to match, this port already follows that convention.

## Screen layout (240x240)
The UI has a dedicated square layout: the system menu fits all six entries on one screen, the game list uses the side-by-side list + box-art layout, the pause menu is full-screen, and the 240x180 game picture is centred vertically. Everything stays inside the 240x240 area in all four orientations.

## First-boot troubleshooting
- Picture orientation: adjust `LCD_ROT_DEFAULT` in the sketch (0–3) if the image is rotated.
- Colours look negative: set `LCD_INVERT` to `0`. Red and blue swapped: set the 0x08 (BGR) bit in `lcdMadctl[]`. Set `LCD_COLOR_TEST` to `1` to show labelled colour bars at boot.
- Picture glitches / noise: lower `LCD_SPI_HZ` (default is already 40 MHz; try 20000000).
- "NO SD CARD": make sure the card is FAT32 and fully inserted. The firmware uses the 4-bit bus at 20 MHz (same as Waveshare's SD example). If the mount times out (error 0x107), reseat the card, try another FAT32 card, or set `SD_1BIT 1`.
- "NOT ENOUGH RAM": make sure Tools > PSRAM is **OPI PSRAM**. The serial monitor prints `PSRAM: ... free` and `Memory before ROM load` at boot/launch; send those lines if it still happens.
- Can't flash: hold BOOT, tap RESET, release BOOT.
- Black screen: the LCD must be on SCLK 38, MOSI 39, DC 45, RST 40, CS 21, BL 46 (see the pin table above). Note: the 04_gfx_helloworld example on Waveshare's S3 wiki page uses GPIO1-5, which are the ESP32-C6 board's pins, not this board's. Open the serial monitor at 115200 baud; you should see `LCD: initialized`. GPIO 41/42 are the codec I2C bus, not the LCD.
- Touch is not yet implemented. Dual-microphone input and any onboard audio codec are also not yet implemented.

# Supported Cores
Nintendo Entertainment System
Nintendo Gameboy
Nintendo Gameboy Color
Atari 2600
Sega Genesis / Mega Drive (basic)

# SD Format
SD card must be formatted to FAT32.

/roms/nes/     *.nes
/roms/gb/      *.gb, *.gbc
/roms/gbc/     *.gbc, *.gb
/roms/a2600/   *.a26, *.bin
/roms/genesis/ *.md, *.gen, *.bin
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

In game: hold SELECT+START for 1 s to quit. SELECT+RB rotates the screen. SELECT+X opens the pause menu (Xbox pad). SELECT+RT (right trigger) saves a screenshot as a PNG to /screenshots on the SD card (/screenshots/<ROM name>_NNN.png); a short "SHOT NNN SAVED" label appears on screen but is not part of the image.

# Disclaimer
You must supply your own ROMs and BIOS files. Console names and games are trademarks of their owners and creators.


# Box Art

Place PNG cover images on the SD card in a system-specific folder. The filename
must match the ROM filename without its extension (including spaces and punctuation):

- `/imgs/nes/<ROM name>.png`
- `/imgs/gb/<ROM name>.png`
- `/imgs/gbc/<ROM name>.png`
- `/imgs/a2600/<ROM name>.png`
- `/imgs/genesis/<ROM name>.png`

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

## Sega Genesis / Mega Drive core (basic)

`src/genesis_core.h` adds a Genesis core (system menu entry **SEGA GENESIS**, ROMs in `/roms/genesis/`,
states in `/states/genesis/`, art in `/imgs/genesis/`). Raw `.md` / `.gen` / `.bin` dumps up to 4 MB;
`.smd` (interleaved) files are not supported. Like the other cores, the ROM is streamed from the SD card
(2-way 2 KB page cache, 128 KB).

**Emulated:** Motorola 68000 (all instructions, approximate timing); VDP Mode 5 with planes A/B, window,
sprites, H32/H40, V28/V30, full/cell/line hscroll, full/2-cell vscroll, DMA (68K->VDP, fill, copy),
H-int/V-int, HV counter; 3-button pad on port 1; Z80 bus-request handshake; save states.

**Not emulated (yet):** Z80 CPU (its 8 KB RAM exists but never runs), full YM2612 FM synthesis, shadow/highlight, interlace, SRAM/battery saves, bank-switch mappers (SSF2),
6-button pad, port 2. Games that need the Z80 to boot or to answer the 68000 may hang.

Output is 320x224 (or 256x224) scaled to the 240x180 picture area (nearest rows, 2-tap horizontal blend).

Controls (Xbox/BLE pad): X = A, A = B, B or Y = C, Menu = Start, View = Select. Menu and quit combos are unchanged.

Memory: about 136 KB for 68K RAM + VRAM + Z80 RAM (internal RAM first, PSRAM fallback), 44 KB frame buffer,
128 KB ROM cache (both preferably PSRAM).

Status: built and checked on a PC with stub Arduino headers using a hand-assembled test ROM (CPU arithmetic and
VDP plane rendering verified). It has NOT been compiled in the Arduino IDE or run on the board, and no commercial
ROMs were available to test, so expect to tune timing and fix compatibility bugs on real hardware.


## ESP32-S3 1.54-inch board port

This folder is the 240×240 port for the Waveshare ESP32-S3-LCD-1.54 and ESP32-S3-Touch-LCD-1.54 family. It retains Emu32's emulator cores, SD-streamed ROMs, save states, screenshots, box art and BLE HID gamepad support. The touch variant's CST816 touchscreen, ES7210 microphones/ES8311 audio codec and QMI8658 IMU are not yet integrated into the Emu32 frontend; they are separate board peripherals, not implied to be supported by this emulator port.

Hardware target: Arduino IDE **ESP32S3 Dev Module**, ESP32 Arduino core 3.x, 16MB flash, OPI PSRAM, USB CDC On Boot enabled; NimBLE-Arduino 2.x and optional PNGdec. Please validate LCD orientation, SD detection and the pin mapping against your exact board revision before relying on it: Waveshare sells both touch and non-touch versions. Serial diagnostics are at 115200 baud.


## Game audio

`src/audio.h` drives the board's I2S output at moderate gain (1/2 software amplitude plus a lower codec level targeting ~50% volume), with no startup beep; optional speaker test via serial `t` command. `src/audio_synth.h` adds lightweight register-driven sound generators for all five systems: NES (two pulse channels, triangle and noise), Game Boy and Game Boy Color (four-channel APU approximation), Atari 2600 (two TIA sound channels), and Genesis (SN76489 PSG plus a lightweight YM2612 tone approximation). Audio is mixed to mono signed 16-bit PCM at 22.05 kHz and sent once per emulated frame.

This is a compact approximation rather than cycle-accurate sound-chip emulation: envelope/sweep details, Game Boy wave RAM playback, full Genesis FM operator algorithms, and per-cycle synchronization are not complete. It should produce game-dependent sound rather than only a startup diagnostic beep. Board audio wiring and codec support still depend on the exact board revision.


### Restart performance fix

The firmware now explicitly restores the ESP32-S3 CPU clock to 240 MHz during every boot and prints the resulting clock to Serial Monitor. This prevents builds configured for dynamic/reduced CPU frequency from leaving emulator timing and audio processing running too slowly after a restart.

## Physical power button

The firmware now holds the Waveshare battery power latch on GPIO2 HIGH at the
very beginning of startup and monitors the physical PWR button on GPIO5. A
normal press-and-release while running mutes the speaker amplifier, blanks the
LCD backlight, and releases the power latch so the board powers off. Pressing
PWR again starts the board through its hardware power circuit. This is intended
for battery operation; USB power can keep the board electrically powered even
after the battery latch is released. Verify these GPIOs against your exact
ESP32-S3-LCD-1.54 revision before flashing.
