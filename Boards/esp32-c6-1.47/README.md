# ESP32-C6 1.47-inch Dev Board
The ESP32-C6 1.47 inch Dev Board is a low power budget micro controller with a built in 1.47 display.

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
Bluetooth pad: BLE HID only (e.g. Xbox, 8BitDo in BLE mode). Press BOOT to pair.
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

The project ZIP was checked for archive integrity. A full Arduino/ESP32-C6 compile and hardware regression test were not available in this environment.

## ROMs always run from the SD card

No core copies a whole ROM into RAM any more. NES, Game Boy, Game Boy Color and Atari 2600 keep the ROM file
open on the card and stream 8 KB / 16 KB / 4 KB / 1 KB pages on demand into a cache
(NES 4 PRG + 8 CHR banks, Atari 16 pages; GB and GBC size theirs from the free RAM, since the C6 has no PSRAM). The card stays mounted during play, and a
read failure shows "ROM READ ERROR / SD CARD LOST" instead of running on corrupt data.
