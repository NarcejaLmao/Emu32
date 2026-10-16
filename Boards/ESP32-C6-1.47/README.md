# Waveshare ESP32-C6 1.47 inch Dev Board
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

# Bluetooth Pairing
Bluetooth pad: BLE HID only (e.g. Xbox, 8BitDo in BLE mode). Press BOOT to pair.

# Controls
Menus: UP/DOWN move, A select, B back.

In game: hold SELECT+START for 1 s to quit. SELECT+RB rotates the screen. SELECT+X opens the pause menu (Xbox pad).

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
