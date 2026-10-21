// ============================================================================
//  Game Boy Color (CGB) emulator
//   - Sharp LR35902 CPU (every opcode, instruction-level timing) with the CGB
//     double-speed mode (KEY1 + STOP), DIV/TIMA timer, all five interrupts,
//     HALT, EI delay
//   - CGB hardware: 2 VRAM banks (tile data + BG attribute maps), 8 WRAM banks,
//     BG/OBJ colour palette RAM (BCPS/BCPD/OCPS/OCPD), per-tile attributes
//     (palette, bank, X/Y flip, BG-over-OBJ priority), OAM attributes (bank,
//     palette), OPRI, general-purpose + HBlank HDMA, OAM DMA
//   - Scanline renderer: BG, window, 8x8/8x16 sprites in full 15-bit colour
//   - Games that are not CGB-aware run in DMG-compatibility mode (BGP/OBP are
//     mapped through palette RAM, like a real Game Boy Color does)
//   - Optional boot ROM: /bios/gbc/cgb_boot.bin (2304 bytes). With it the real
//     colour logo + automatic DMG-game palettes run. Without it the core starts
//     in the post-boot state and DMG-only games use GBC_DMG_PALETTE.
//   - Cartridges: ROM-only, MBC1/1M-style, MBC2, MBC3+RTC, MBC5, HuC1, and basic MMM01 compatibility
//   - Battery saves go to <rom name>.sav next to the ROM on the SD card
//   - ROM is demand-paged in 4 KB pages from the SD card (least recently used
//     page is replaced), so 2-8 MB games work in the little RAM available.
//     A ROM that fits in the cache is loaded once and the card is released.
//   - Renderer: sprite-free lines go tile data -> RGB565 in one pass (spread-LUT decode)
//   - The picture is rendered in 8-line bands straight to the LCD (no full
//     frame buffer), 160x144 scaled to 172x155, pinned to the top in portrait
//   - Not emulated: audio (sound registers are stubbed), serial link, IR port
// ============================================================================
#ifndef GBC_OPT_O3
#define GBC_OPT_O3        1              // 1 = compile this core with -O3 (faster, a bit more code); 0 = sketch default
#endif
#if GBC_OPT_O3
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif
#define GBC_MAX_SLOTS     16             // cached 4 KB ROM pages: the ROM itself always stays on the SD card
#define GBC_MIN_SLOTS     10             // 8 mapped pages + 2 spare
#define GBC_HEAP_RESERVE  (6 * 1024)    // heap kept free when sizing the page cache
#define GBC_DMG_PALETTE   0              // no-boot-ROM palette for DMG games: 0 grey, 1 green, 2 pocket
#define GBC_COLOR_CORRECT 1              // 1 = mix channels like the real LCD, 0 = raw RGB555
#define GBC_DOTS_FRAME    70224          // 154 lines * 456 dots
#define GBC_OUT_W         GAME_W         // 172: scaled picture width (fits outLine[])
#define GBC_OUT_H         155            // 144 * 172 / 160
#define GBC_PAGES_MAX     2048           // 8 MB / 4 KB
#ifndef GBC_FAST
#define GBC_FAST          IRAM_ATTR
#endif
static_assert(GBC_OUT_W <= GAME_W, "outLine[] is only GAME_W pixels wide");

static const uint32_t gbcDmgRGB[3][4] = {
  { 0xFFFFFF, 0xA5A5A5, 0x525252, 0x000000 },
  { 0x9BBC0F, 0x8BAC0F, 0x306230, 0x0F380F },
  { 0xE0F8D0, 0x88C070, 0x346856, 0x081820 },
};

// ---- memory -----------------------------------------------------------------
static uint8_t  gcBootRom[0x900];
static bool     gcBootAvail = false, gcBootActive = false;
static uint8_t *gcBootPage = nullptr;           // 4 KB: boot ROM overlaid on cartridge page 0
static uint8_t *gcMem = nullptr;                // WRAM (32 KB) + VRAM (16 KB)
static uint8_t *gcWram = nullptr, *gcVram = nullptr, *gcVramCur = nullptr;
static uint8_t  gcOam[160], gcHram[127], gcIo[128];
static uint8_t *gcMap[16];                      // read map, 4 KB pages (null = slow path)
static uint8_t *gcWMap[16];                     // write map (VRAM / WRAM only)

// ---- cartridge --------------------------------------------------------------
static File     gcRomFile;
static uint32_t gcRomBanks = 2, gcRomPages = 8;
static uint8_t  gcMbc = 0;                      // 0 ROM, 1 MBC1, 2 MBC2, 3 MBC3, 5 MBC5, 6 HuC1, 7 MMM01-compatible
static bool     gcBattery = false, gcHasRtc = false, gcHdrCgb = false;
static uint8_t *gcRam = nullptr;
static uint32_t gcRamAlloc = 0, gcSavSize = 0;
static bool     gcRomReadError = false;
static uint8_t *gcRamPtr = nullptr;
static bool     gcRamEn = false, gcMode1 = false, gcRamDirty = false;
static uint16_t gcB1 = 1;
static uint8_t  gcB2 = 0, gcRamBank = 0, gcLatch = 0xFF;
static uint32_t gcRamStamp = 0;
static char     gcSavPath[ROM_NAME_LEN + 24];

// ROM page cache (4 KB pages, demand loaded)
static uint8_t *gcPgData[GBC_MAX_SLOTS];
static uint16_t gcPgTag[GBC_MAX_SLOTS];         // ROM page held by the slot (0xFFFF = empty)
static uint32_t gcPgStamp[GBC_MAX_SLOTS];
static uint8_t  gcPgSlot[GBC_PAGES_MAX];        // ROM page -> slot (0xFF = not cached)
static int      gcSlots = 0;
static uint32_t gcStamp = 0;
static uint32_t gcBase0 = 0, gcBase1 = 4;       // first ROM page of the 0000 / 4000 windows

// ---- RTC (MBC3) -------------------------------------------------------------
static uint8_t  gcRtc[5], gcRtcLat[5];
static uint32_t gcRtcLast = 0, gcRtcSub = 0;

// ---- CPU / system state -----------------------------------------------------
static uint8_t  gcR[8];                         // B C D E H L (unused) A
static uint8_t  gcF;
static uint16_t gcPC, gcSP;
static bool     gcIME, gcHalt, gcHaltBug;
static uint8_t  gcEiDelay, gcIF, gcIE;
static uint32_t gcCycles, gcLimit, gcDivZero, gcSerEnd, gcFrameNo;
static int32_t  gcExtra;                        // stall clocks (HDMA, speed switch) paid by the CPU loop
static bool     gcSerOn;
static uint8_t  gcTIMA, gcTMA, gcTAC;
static int32_t  gcTimaCnt, gcTimaPeriod = 1024;
static bool     gcTimerOn;
static uint8_t  gcJoySel, gcPadBtn, gcPadDir, gcJoyPrev;

// ---- CGB state --------------------------------------------------------------
static bool     gcCgb = true;                   // true: CGB mode, false: DMG-compatibility mode
static uint8_t  gcSpeed = 0, gcKey1 = 0;        // 0 normal, 1 double speed
static uint8_t  gcVbk = 0, gcSvbkRaw = 1, gcOpri = 0;
static uint32_t gcWbankOff = 0x1000;            // byte offset of the switchable WRAM bank
static uint8_t  gcBcps = 0, gcOcps = 0;
static uint8_t  gcBgRam[64], gcObjRam[64];
static uint16_t gcBgTab[8][4], gcObjTab[8][4];  // finished RGB565 (byte swapped for the LCD)
static bool     gcHdmaOn = false;
static uint8_t  gcHdmaLeft = 0, gcHdmaReg = 0xFF;
static uint16_t gcHdmaSrc = 0, gcHdmaDst = 0x8000;

// ---- PPU --------------------------------------------------------------------
static uint8_t  gcLCDC, gcSTAT, gcSCY, gcSCX, gcLY, gcLYC, gcBGP, gcOBP0, gcOBP1, gcWY, gcWX, gcMode, gcWinLine;
static int32_t  gcPpuClk, gcPpuLen;
static bool     gcStatLine, gcDraw;
static uint16_t gcBand[8 * 160] __attribute__((aligned(4)));   // 8 finished lines, pushed to the LCD as a band
static uint32_t gcBandHash[20];                 // hash of what each 8-line band last sent to the LCD
static uint8_t  gcColMap[GBC_OUT_W], gcRowMap[GBC_OUT_H], gcRowFirst[145];

// ============================================================================
//  Colour
// ============================================================================
static uint16_t gbcColor(uint16_t c) {
  const int r5 = c & 31, g5 = (c >> 5) & 31, b5 = (c >> 10) & 31;
  const int r8 = (r5 << 3) | (r5 >> 2), g8 = (g5 << 3) | (g5 >> 2), b8 = (b5 << 3) | (b5 >> 2);
#if GBC_COLOR_CORRECT
  // The GBC LCD bleeds channels into each other; this linear mix gives a similar, less garish picture.
  return col565((r8 * 26 + g8 * 4 + b8 * 2) >> 5, (g8 * 24 + b8 * 8) >> 5, (r8 * 6 + g8 * 4 + b8 * 22) >> 5);
#else
  return col565(r8, g8, b8);
#endif
}
static inline uint16_t gbcRaw(const uint8_t *ram, int i) { i &= ~1; return ram[i] | ((uint16_t)ram[i + 1] << 8); }

// DMG-compatibility mode: BGP/OBPn pick one of the four colours of palette 0 (BG, OBJ0) or 1 (OBJ1).
static void gbcDmgBgRebuild() {
  for (int c = 0; c < 4; c++) gcBgTab[0][c] = gbcColor(gbcRaw(gcBgRam, ((gcBGP >> (c * 2)) & 3) * 2));
}
static void gbcDmgObjRebuild(int n) {
  const uint8_t reg = n ? gcOBP1 : gcOBP0;
  for (int c = 0; c < 4; c++) gcObjTab[n][c] = gbcColor(gbcRaw(gcObjRam, n * 8 + ((reg >> (c * 2)) & 3) * 2));
}
static void gbcBgEntry(int idx) {
  const int pal = idx >> 3, col = (idx >> 1) & 3;
  if (gcCgb) gcBgTab[pal][col] = gbcColor(gbcRaw(gcBgRam, idx));
  else if (pal == 0) gbcDmgBgRebuild();
}
static void gbcObjEntry(int idx) {
  const int pal = idx >> 3, col = (idx >> 1) & 3;
  if (gcCgb) gcObjTab[pal][col] = gbcColor(gbcRaw(gcObjRam, idx));
  else if (pal < 2) gbcDmgObjRebuild(pal);
}
static void gbcRebuildAll() {
  if (gcCgb) { for (int i = 0; i < 64; i += 2) { gbcBgEntry(i); gbcObjEntry(i); } }
  else       { gbcDmgBgRebuild(); gbcDmgObjRebuild(0); gbcDmgObjRebuild(1); }
}

// ============================================================================
//  ROM page cache
// ============================================================================
static uint8_t *gbcPage(uint32_t pg) {
  uint8_t s = gcPgSlot[pg];
  if (s != 0xFF) { sdRomDiag.cacheHits++; gcPgStamp[s] = ++gcStamp; return gcPgData[s]; }
  sdRomDiag.cacheMisses++;

  int v = -1; uint32_t best = 0xFFFFFFFF;
  for (int i = 0; i < gcSlots; i++) {
    if (gcPgTag[i] == 0xFFFF) { v = i; break; }
    bool mapped = false;                              // pages in the CPU windows are never evicted
    for (int m = 0; m < 8; m++) if (gcMap[m] == gcPgData[i]) { mapped = true; break; }
    if (mapped) continue;
    if (gcPgStamp[i] <= best) { best = gcPgStamp[i]; v = i; }
  }
  if (v < 0) v = 0;                                   // cannot happen with >= GBC_MIN_SLOTS slots
  if (gcPgTag[v] != 0xFFFF) gcPgSlot[gcPgTag[v]] = 0xFF;

  uint8_t *d = gcPgData[v];
  size_t n = (size_t)sdDiagReadAt(gcRomFile, (uint32_t)pg * 4096UL, d, 4096);
  if (n < 4096) { memset(d + n, 0xFF, 4096 - n); gcRomReadError = true; }
  gcPgTag[v] = pg; gcPgSlot[pg] = v; gcPgStamp[v] = ++gcStamp;
  return d;
}
static inline uint8_t *gbcResident(uint32_t pg) {
  const uint8_t s = gcPgSlot[pg];
  if (s == 0xFF) return nullptr;
  gcPgStamp[s] = ++gcStamp;
  return gcPgData[s];
}

// ============================================================================
//  Cartridge mapping
// ============================================================================
static void gbcBootMap() {                            // boot ROM over 0000-00FF and 0200-08FF
  if (!gcBootPage) return;
  memcpy(gcBootPage, gbcPage(0), 4096);
  memcpy(gcBootPage, gcBootRom, 0x100);
  memcpy(gcBootPage + 0x200, gcBootRom + 0x200, 0x700);
  gcMap[0] = gcBootPage;
}

static void gbcMapRom() {
  uint32_t b0 = 0, b1 = 1;
  switch (gcMbc) {
    case 1: case 6: case 7: b1 = ((uint32_t)gcB2 << 5) | gcB1; if (gcMode1) b0 = (uint32_t)gcB2 << 5; break;
    case 2: case 3: case 5: b1 = gcB1; break;
  }
  b0 %= gcRomBanks; b1 %= gcRomBanks;
  gcBase0 = b0 * 4; gcBase1 = b1 * 4;
  for (int i = 0; i < 4; i++) { gcMap[i] = gbcResident(gcBase0 + i); gcMap[4 + i] = gbcResident(gcBase1 + i); }
  if (gcBootActive) gbcBootMap();
}

static void gbcMapRam() {
  gcRamPtr = nullptr;
  if (gcRamEn && gcRam && gcMbc != 2) {
    uint32_t bank = (gcMbc == 1 || gcMbc == 6 || gcMbc == 7) ? (gcMode1 ? gcB2 : 0) : gcRamBank;
    if (!(gcMbc == 3 && bank >= 8)) gcRamPtr = gcRam + (bank * 0x2000) % gcRamAlloc;
  }
  gcMap[0xA] = gcRamPtr;
  gcMap[0xB] = gcRamPtr ? gcRamPtr + 0x1000 : nullptr;
}

static void gbcRtcTick() {
  uint32_t now = millis(), d = now - gcRtcLast;
  gcRtcLast = now;
  if (gcRtc[4] & 0x40) return;
  gcRtcSub += d;
  while (gcRtcSub >= 1000) {
    gcRtcSub -= 1000;
    if (++gcRtc[0] < 60) continue;
    gcRtc[0] = 0;
    if (++gcRtc[1] < 60) continue;
    gcRtc[1] = 0;
    if (++gcRtc[2] < 24) continue;
    gcRtc[2] = 0;
    uint16_t day = gcRtc[3] | ((gcRtc[4] & 1) << 8);
    if (++day > 511) { day = 0; gcRtc[4] |= 0x80; }
    gcRtc[3] = day & 0xFF; gcRtc[4] = (gcRtc[4] & 0xFE) | (day >> 8);
  }
}

static void gbcMbcWrite(uint16_t a, uint8_t v) {
  switch (gcMbc) {
    case 1: case 6: case 7:
      switch (a >> 13) {
        case 0: gcRamEn = ((v & 0x0F) == 0x0A); gbcMapRam(); break;
        case 1: gcB1 = v & 0x1F; if (!gcB1) gcB1 = 1; gbcMapRom(); break;
        case 2: gcB2 = v & 3; gbcMapRom(); gbcMapRam(); break;
        default: gcMode1 = v & 1; gbcMapRom(); gbcMapRam(); break;
      }
      break;
    case 2:
      if (a < 0x4000) {
        if (a & 0x100) { gcB1 = v & 0x0F; if (!gcB1) gcB1 = 1; gbcMapRom(); }
        else gcRamEn = ((v & 0x0F) == 0x0A);
      }
      break;
    case 3:
      switch (a >> 13) {
        case 0: gcRamEn = ((v & 0x0F) == 0x0A); gbcMapRam(); break;
        case 1: gcB1 = v & 0x7F; if (!gcB1) gcB1 = 1; gbcMapRom(); break;
        case 2: gcRamBank = v; gbcMapRam(); break;
        default:
          if (gcLatch == 0 && v == 1) { gbcRtcTick(); memcpy(gcRtcLat, gcRtc, 5); }
          gcLatch = v;
          break;
      }
      break;
    case 5:
      switch (a >> 12) {
        case 0: case 1: gcRamEn = ((v & 0x0F) == 0x0A); gbcMapRam(); break;
        case 2: gcB1 = (gcB1 & 0x100) | v; gbcMapRom(); break;
        case 3: gcB1 = (gcB1 & 0xFF) | ((uint16_t)(v & 1) << 8); gbcMapRom(); break;
        case 4: case 5: gcRamBank = v & 0x0F; gbcMapRam(); break;
      }
      break;
  }
}

static void gbcRamWrite(uint16_t a, uint8_t v) {
  if (!gcRamEn) return;
  if (gcMbc == 2) { gcRam[a & 0x1FF] = v & 0x0F; gcRamDirty = true; gcRamStamp = gcFrameNo; return; }
  if (gcMbc == 3 && gcRamBank >= 8) {
    if (gcRamBank <= 0x0C) {
      gbcRtcTick();
      static const uint8_t m[5] = { 0x3F, 0x3F, 0x1F, 0xFF, 0xC1 };
      gcRtc[gcRamBank - 8] = v & m[gcRamBank - 8];
      if (gcRamBank == 8) gcRtcSub = 0;
    }
    return;
  }
  if (gcRamPtr) { gcRamPtr[a - 0xA000] = v; gcRamDirty = true; gcRamStamp = gcFrameNo; }
}

static uint8_t gbcRamRead(uint16_t a) {
  if (!gcRamEn) return 0xFF;
  if (gcMbc == 2) return 0xF0 | (gcRam[a & 0x1FF] & 0x0F);
  if (gcMbc == 3 && gcRamBank >= 8 && gcRamBank <= 0x0C) return gcRtcLat[gcRamBank - 8];
  return 0xFF;
}

// ============================================================================
//  I/O registers
// ============================================================================
static void gbcStatUpdate() {
  const uint8_t s = gcSTAT;
  bool line = false;
  if (gcLCDC & 0x80)
    line = ((gcMode == 0) && (s & 0x08)) || ((gcMode == 1) && (s & 0x10)) ||
           ((gcMode == 2) && (s & 0x20)) || ((gcLY == gcLYC) && (s & 0x40));
  if (line && !gcStatLine) gcIF |= 2;
  gcStatLine = line;
}

static inline void gbcSerialCheck() {
  if (gcSerOn && (int32_t)(gcCycles - gcSerEnd) >= 0) { gcSerOn = false; gcIo[1] = 0xFF; gcIo[2] &= 0x7F; gcIF |= 8; }
}

static inline uint8_t gbcRd(uint16_t a);
static inline uint8_t gbcJoyNibble() {
  uint8_t v = 0x0F;
  if (!(gcJoySel & 0x10)) v &= ~gcPadDir;
  if (!(gcJoySel & 0x20)) v &= ~gcPadBtn;
  return v & 0x0F;
}

static inline void gbcMapVram() {
  gcVramCur = gcVram + (gcVbk ? 0x2000 : 0);
  gcMap[8] = gcWMap[8] = gcVramCur;
  gcMap[9] = gcWMap[9] = gcVramCur + 0x1000;
}
static inline void gbcMapWram() {
  uint8_t b = gcSvbkRaw & 7; if (!b) b = 1;
  gcWbankOff = (uint32_t)b * 0x1000;
  gcMap[0xD] = gcWMap[0xD] = gcWram + gcWbankOff;
}

static void gbcHdmaBlock() {                          // copy one 16 byte block, source -> VRAM
  for (int i = 0; i < 16; i++) gcVramCur[(gcHdmaDst + i) & 0x1FFF] = gbcRd(gcHdmaSrc + i);
  gcHdmaSrc += 16;
  gcHdmaDst = 0x8000 | ((gcHdmaDst + 16) & 0x1FFF);
  gcExtra += 32 << gcSpeed;                           // the CPU is held while the block moves
  if (gcHdmaLeft && --gcHdmaLeft == 0) { gcHdmaOn = false; gcHdmaReg = 0xFF; }
}

static void gbcHdmaWrite(uint8_t v) {
  if (gcHdmaOn && !(v & 0x80)) {                      // cancel a running HBlank transfer
    gcHdmaOn = false;
    gcHdmaReg = 0x80 | ((gcHdmaLeft - 1) & 0x7F);
    return;
  }
  gcHdmaSrc = (((uint16_t)gcIo[0x51] << 8) | gcIo[0x52]) & 0xFFF0;
  gcHdmaDst = 0x8000 | ((((uint16_t)gcIo[0x53] << 8) | gcIo[0x54]) & 0x1FF0);
  const uint8_t blocks = (v & 0x7F) + 1;
  if (v & 0x80) { gcHdmaOn = true; gcHdmaLeft = blocks; }
  else { gcHdmaLeft = blocks; gcHdmaOn = true; while (gcHdmaOn) gbcHdmaBlock(); }
}

static void gbcSpeedSwitch() {                        // STOP with KEY1 bit 0 armed
  int32_t rem = (int32_t)(gcLimit - gcCycles);
  gcSpeed ^= 1; gcKey1 = 0;
  rem = gcSpeed ? (rem << 1) : (rem >> 1);
  rem &= ~3;
  gcLimit = gcCycles + rem;                           // keep the same amount of real time left in this frame
  gcDivZero = gcCycles;
  gcExtra += 8192;
}

static void gbcBootEnd() {
  gcBootActive = false;
  if (!gcCgb) { gcVbk = 0; gcSvbkRaw = 1; gbcMapVram(); gbcMapWram(); }
  gbcMapRom();
  gbcRebuildAll();
}

static GBC_FAST uint8_t gbcIoRd(uint8_t r) {
  switch (r) {
    case 0x00: return 0xC0 | gcJoySel | gbcJoyNibble();
    case 0x02: return gcIo[2] | 0x7E;
    case 0x04: return (uint8_t)((gcCycles - gcDivZero) >> 8);
    case 0x05: return gcTIMA;
    case 0x06: return gcTMA;
    case 0x07: return gcTAC | 0xF8;
    case 0x0F: return gcIF | 0xE0;
    case 0x26: return gcIo[0x26] | 0x70;
    case 0x40: return gcLCDC;
    case 0x41: return 0x80 | (gcSTAT & 0x78) | ((gcLCDC & 0x80) ? (((gcLY == gcLYC) ? 4 : 0) | gcMode) : 0);
    case 0x42: return gcSCY;
    case 0x43: return gcSCX;
    case 0x44: return gcLY;
    case 0x45: return gcLYC;
    case 0x47: return gcBGP;
    case 0x48: return gcOBP0;
    case 0x49: return gcOBP1;
    case 0x4A: return gcWY;
    case 0x4B: return gcWX;
    case 0x4D: return gcCgb ? (0x7E | (gcSpeed << 7) | (gcKey1 & 1)) : 0xFF;
    case 0x4F: return gcCgb ? (0xFE | gcVbk) : 0xFF;
    case 0x55: return !gcCgb ? 0xFF : gcHdmaOn ? ((gcHdmaLeft - 1) & 0x7F) : gcHdmaReg;
    case 0x56: return 0x3E | (gcIo[0x56] & 0xC1);
    case 0x68: return gcBcps | 0x40;
    case 0x69: return gcBgRam[gcBcps & 0x3F];
    case 0x6A: return gcOcps | 0x40;
    case 0x6B: return gcObjRam[gcOcps & 0x3F];
    case 0x6C: return 0xFE | gcOpri;
    case 0x70: return gcCgb ? (0xF8 | (gcSvbkRaw & 7)) : 0xFF;
    case 0x75: return gcIo[0x75] | 0x8F;
    case 0x76: case 0x77: return 0x00;                // PCM amplitude readback (audio not emulated)
    default:   return gcIo[r];
  }
}

static GBC_FAST void gbcIoWr(uint8_t r, uint8_t v) {
  switch (r) {
    case 0x00: gcJoySel = v & 0x30; break;
    case 0x02:
      gcIo[2] = v;
      if ((v & 0x81) == 0x81) { gcSerOn = true; gcSerEnd = gcCycles + 4096; }
      break;
    case 0x04: gcDivZero = gcCycles; break;
    case 0x05: gcTIMA = v; break;
    case 0x06: gcTMA = v; break;
    case 0x07: {
      static const int32_t per[4] = { 1024, 16, 64, 256 };
      gcTAC = v & 7; gcTimerOn = v & 4; gcTimaPeriod = per[v & 3];
    } break;
    case 0x0F: gcIF = v & 0x1F; break;
    case 0x40: {
      uint8_t old = gcLCDC; gcLCDC = v;
      if ((old ^ v) & 0x80) {
        gcLY = 0; gcPpuClk = 0; gcWinLine = 0; gcStatLine = false;
        if (v & 0x80) { gcMode = 2; gcPpuLen = 80; gbcStatUpdate(); }
        else { gcMode = 0; gcPpuLen = 0x7FFFFFFF; }
      }
    } break;
    case 0x41: gcSTAT = v & 0x78; gbcStatUpdate(); break;
    case 0x42: gcSCY = v; break;
    case 0x43: gcSCX = v; break;
    case 0x44: break;                                 // LY is read only
    case 0x45: gcLYC = v; gbcStatUpdate(); break;
    case 0x46: {                                      // OAM DMA (done instantly)
      uint16_t src = (uint16_t)v << 8;
      for (int i = 0; i < 160; i++) gcOam[i] = gbcRd(src + i);
      gcIo[0x46] = v;
    } break;
    case 0x47: gcBGP  = v; if (!gcCgb) gbcDmgBgRebuild(); break;
    case 0x48: gcOBP0 = v; if (!gcCgb) gbcDmgObjRebuild(0); break;
    case 0x49: gcOBP1 = v; if (!gcCgb) gbcDmgObjRebuild(1); break;
    case 0x4A: gcWY = v; break;
    case 0x4B: gcWX = v; break;
    case 0x4C:                                        // KEY0: the boot ROM picks CGB / DMG-compat mode
      if (gcBootActive) { gcCgb = !(v & 4); gcOpri = (v & 4) ? 1 : 0; gbcRebuildAll(); }
      break;
    case 0x4D: if (gcCgb) gcKey1 = v & 1; break;
    case 0x4F:
      if (gcCgb) { gcVbk = v & 1; gbcMapVram(); }
      break;
    case 0x50: gcIo[r] = v; if (v && gcBootActive) gbcBootEnd(); break;
    case 0x51: case 0x52: case 0x53: case 0x54: gcIo[r] = v; break;
    case 0x55: if (gcCgb) gbcHdmaWrite(v); break;
    case 0x56: gcIo[r] = v; break;
    case 0x68: gcBcps = v & 0xBF; break;
    case 0x69: {
      const int i = gcBcps & 0x3F;
      gcBgRam[i] = v; gbcBgEntry(i);
      if (gcBcps & 0x80) gcBcps = 0x80 | ((i + 1) & 0x3F);
    } break;
    case 0x6A: gcOcps = v & 0xBF; break;
    case 0x6B: {
      const int i = gcOcps & 0x3F;
      gcObjRam[i] = v; gbcObjEntry(i);
      if (gcOcps & 0x80) gcOcps = 0x80 | ((i + 1) & 0x3F);
    } break;
    case 0x6C: if (gcBootActive) gcOpri = v & 1; break;
    case 0x70:
      if (gcCgb) { gcSvbkRaw = v & 7; gbcMapWram(); }
      break;
    default:   gcIo[r] = v; break;
  }
}

// ============================================================================
//  Bus
// ============================================================================
static GBC_FAST uint8_t gbcRdSlow(uint16_t a) {
  if (a >= 0xFF00) {                                  // HRAM / IO / IE are by far the most common slow accesses
    if (a >= 0xFF80) return (a < 0xFFFF) ? gcHram[a - 0xFF80] : gcIE;
    return gbcIoRd(a & 0x7F);
  }
  if (a < 0x8000) {                                   // ROM page not mapped yet: load it and map it
    const uint32_t pg = ((a & 0x4000) ? gcBase1 : gcBase0) + ((a >> 12) & 3);
    uint8_t *p = gbcPage(pg);
    gcMap[a >> 12] = p;
    return p[a & 0xFFF];
  }
  if (a >= 0xF000) {
    if (a < 0xFE00) return gcWram[gcWbankOff + (a - 0xF000)];     // echo of D000-DDFF
    if (a < 0xFEA0) return gcOam[a - 0xFE00];
    return 0xFF;
  }
  return gbcRamRead(a);                               // A000-BFFF without a direct mapping
}

static GBC_FAST void gbcWrSlow(uint16_t a, uint8_t v) {
  if (a >= 0xFF00) {
    if (a >= 0xFF80) { if (a < 0xFFFF) gcHram[a - 0xFF80] = v; else gcIE = v; }
    else gbcIoWr(a & 0x7F, v);
    return;
  }
  if (a < 0x8000) { gbcMbcWrite(a, v); return; }
  if (a >= 0xA000 && a < 0xC000) { gbcRamWrite(a, v); return; }
  if (a < 0xFE00) { gcWram[gcWbankOff + (a - 0xF000)] = v; return; }
  if (a < 0xFEA0) gcOam[a - 0xFE00] = v;
}

static inline uint8_t gbcRd(uint16_t a) {
  uint8_t *p = gcMap[a >> 12];
  return p ? p[a & 0xFFF] : gbcRdSlow(a);
}
static inline void gbcWr(uint16_t a, uint8_t v) {
  uint8_t *p = gcWMap[a >> 12];
  if (p) p[a & 0xFFF] = v; else gbcWrSlow(a, v);
}

// ============================================================================
//  Timer + PPU
// ============================================================================
static void gbcTimerAdvance() {
  while (gcTimaCnt >= gcTimaPeriod) {
    gcTimaCnt -= gcTimaPeriod;
    if (++gcTIMA == 0) { gcTIMA = gcTMA; gcIF |= 4; }
  }
}

// ---- tile decode -------------------------------------------------------------
// gcSpread[v] spreads the 8 bits of a bitplane byte onto bits 0,2,4..14. Then
// (gcSpread[lo] | gcSpread[hi] << 1) holds a whole 8 pixel tile row, 2 bits per pixel,
// leftmost pixel in bits 15:14. One 512 byte table, no per-pixel shifts/masks while decoding.
static uint16_t gcSpread[256];
static bool gcSpreadReady = false;
static void gbcInitDecodeLut() {
  if (gcSpreadReady) return;
  for (unsigned v = 0; v < 256; ++v) {
    uint16_t s = 0;
    for (unsigned b = 0; b < 8; ++b) s |= (uint16_t)(((v >> b) & 1) << (2 * b));
    gcSpread[v] = s;
  }
  gcSpreadReady = true;
}

// 8 pixel row of a BG / window tile (t = tile number, a = CGB attribute, 0 in DMG mode).
static inline uint32_t gbcTileWord(uint8_t t, uint8_t a, int row, uint8_t lcdc) {
  if (a & 0x40) row = 7 - row;                                       // Y flip
  const uint8_t *base = gcVram + ((a & 8) ? 0x2000 : 0);             // tile bank
  const uint8_t *tp = (lcdc & 0x10) ? base + t * 16 + row * 2
                                    : base + 0x1000 + (int8_t)t * 16 + row * 2;
  return (uint32_t)gcSpread[tp[0]] | ((uint32_t)gcSpread[tp[1]] << 1);
}
// finished RGB565 pixels (used when no sprite is on the line)
static inline void gbcTileColors(uint16_t *o, uint32_t w, const uint16_t *p, bool fx) {
  if (!fx) {
    o[0] = p[(w >> 14) & 3]; o[1] = p[(w >> 12) & 3]; o[2] = p[(w >> 10) & 3]; o[3] = p[(w >> 8) & 3];
    o[4] = p[(w >> 6) & 3];  o[5] = p[(w >> 4) & 3];  o[6] = p[(w >> 2) & 3];  o[7] = p[w & 3];
  } else {
    o[0] = p[w & 3];         o[1] = p[(w >> 2) & 3];  o[2] = p[(w >> 4) & 3];  o[3] = p[(w >> 6) & 3];
    o[4] = p[(w >> 8) & 3];  o[5] = p[(w >> 10) & 3]; o[6] = p[(w >> 12) & 3]; o[7] = p[(w >> 14) & 3];
  }
}
// colour index + flags (bits 2-4 palette, bit 7 BG-over-OBJ) for the sprite mixing path
static inline void gbcTileIdx(uint8_t *o, uint32_t w, uint8_t fl, bool fx) {
  if (!fx) {
    o[0] = ((w >> 14) & 3) | fl; o[1] = ((w >> 12) & 3) | fl; o[2] = ((w >> 10) & 3) | fl; o[3] = ((w >> 8) & 3) | fl;
    o[4] = ((w >> 6) & 3) | fl;  o[5] = ((w >> 4) & 3) | fl;  o[6] = ((w >> 2) & 3) | fl;  o[7] = (w & 3) | fl;
  } else {
    o[0] = (w & 3) | fl;         o[1] = ((w >> 2) & 3) | fl;  o[2] = ((w >> 4) & 3) | fl;  o[3] = ((w >> 6) & 3) | fl;
    o[4] = ((w >> 8) & 3) | fl;  o[5] = ((w >> 10) & 3) | fl; o[6] = ((w >> 12) & 3) | fl; o[7] = ((w >> 14) & 3) | fl;
  }
}

// Render one visible line into the band buffer (finished RGB565).
// Lines without sprites (the common case) go straight from tile data to colours in one pass.
static GBC_FAST void gbcRenderLine(int ly, bool win) {
  uint16_t *out = gcBand + (ly & 7) * 160;
  const uint8_t lcdc = gcLCDC;
  const bool cgb = gcCgb;
  const bool bgOn = cgb || (lcdc & 1);
  const int h = (lcdc & 4) ? 16 : 8;

  uint16_t key[10]; int n = 0;                         // sprites on this line (max 10), key = X<<8 | OAM index
  if (lcdc & 2) {
    for (int i = 0; i < 40 && n < 10; i++) {
      const int y = (int)gcOam[i * 4] - 16;
      if (ly >= y && ly < y + h) key[n++] = ((uint16_t)gcOam[i * 4 + 1] << 8) | i;
    }
  }

  const int ty = (uint8_t)(ly + gcSCY);
  const uint8_t *map = gcVram + ((lcdc & 8) ? 0x1C00 : 0x1800) + ((ty >> 3) << 5);
  const int row = ty & 7, fineX = gcSCX & 7, tx = gcSCX >> 3;
  const uint8_t *wmap = gcVram + ((lcdc & 0x40) ? 0x1C00 : 0x1800) + ((gcWinLine >> 3) << 5);
  const int wrow = gcWinLine & 7, wx = (int)gcWX - 7;

  if (n == 0) {
    // ---- fast path: BG + window straight to RGB565 ---------------------------------------
    if (!bgOn) {                                       // DMG-compat, BG switched off
      const uint16_t off = gbcColor(gbcRaw(gcBgRam, 0));
      for (int x = 0; x < 160; x++) out[x] = off;
      return;
    }
    int dst = 0;
    for (int i = 0; dst < 160; i++) {
      const int mi = (tx + i) & 31;
      const uint8_t a = cgb ? map[mi + 0x2000] : 0;
      const uint32_t w = gbcTileWord(map[mi], a, row, lcdc);
      const uint16_t *pal = gcBgTab[a & 7];
      const bool fx = (a & 0x20) != 0;
      if (i > 0 && dst + 8 <= 160) { gbcTileColors(out + dst, w, pal, fx); dst += 8; }
      else {
        uint16_t t[8]; gbcTileColors(t, w, pal, fx);
        for (int p = (i == 0) ? fineX : 0; p < 8 && dst < 160; p++) out[dst++] = t[p];
      }
    }
    if (win) {
      for (int i = 0, x = wx; x < 160; i++, x += 8) {
        const uint8_t a = cgb ? wmap[i + 0x2000] : 0;
        const uint32_t w = gbcTileWord(wmap[i], a, wrow, lcdc);
        const uint16_t *pal = gcBgTab[a & 7];
        const bool fx = (a & 0x20) != 0;
        if (x >= 0 && x + 8 <= 160) gbcTileColors(out + x, w, pal, fx);
        else {
          uint16_t t[8]; gbcTileColors(t, w, pal, fx);
          for (int p = 0; p < 8; p++) { const int sx = x + p; if (sx >= 0 && sx < 160) out[sx] = t[p]; }
        }
      }
    }
    return;
  }

  // ---- slow path: sprites on this line ---------------------------------------------------
  uint8_t bgl[160];                                    // bits 0-1 colour, 2-4 palette, 7 BG-over-OBJ
  if (bgOn) {
    int dst = 0;
    for (int i = 0; dst < 160; i++) {
      const int mi = (tx + i) & 31;
      const uint8_t a = cgb ? map[mi + 0x2000] : 0;
      const uint32_t w = gbcTileWord(map[mi], a, row, lcdc);
      const uint8_t fl = ((a & 7) << 2) | (a & 0x80);
      const bool fx = (a & 0x20) != 0;
      if (i > 0 && dst + 8 <= 160) { gbcTileIdx(bgl + dst, w, fl, fx); dst += 8; }
      else {
        uint8_t t[8]; gbcTileIdx(t, w, fl, fx);
        for (int p = (i == 0) ? fineX : 0; p < 8 && dst < 160; p++) bgl[dst++] = t[p];
      }
    }
    if (win) {
      for (int i = 0, x = wx; x < 160; i++, x += 8) {
        const uint8_t a = cgb ? wmap[i + 0x2000] : 0;
        const uint32_t w = gbcTileWord(wmap[i], a, wrow, lcdc);
        const uint8_t fl = ((a & 7) << 2) | (a & 0x80);
        const bool fx = (a & 0x20) != 0;
        if (x >= 0 && x + 8 <= 160) gbcTileIdx(bgl + x, w, fl, fx);
        else {
          uint8_t t[8]; gbcTileIdx(t, w, fl, fx);
          for (int p = 0; p < 8; p++) { const int sx = x + p; if (sx >= 0 && sx < 160) bgl[sx] = t[p]; }
        }
      }
    }
  } else {
    memset(bgl, 0, sizeof(bgl));
  }

  uint8_t spr[160];                                    // bits 0-1 colour, 2-4 palette, 5 behind-BG
  if (!cgb || gcOpri) {                                // DMG rule: lowest X wins, then OAM order
    for (int i = 1; i < n; i++) {
      uint16_t k = key[i]; int j = i - 1;
      while (j >= 0 && key[j] > k) { key[j + 1] = key[j]; j--; }
      key[j + 1] = k;
    }
  }                                                    // CGB rule: OAM order only (already in order)
  memset(spr, 0, sizeof(spr));
  for (int k = n - 1; k >= 0; k--) {                   // draw lowest priority first
    const uint8_t *o = gcOam + (key[k] & 0xFF) * 4;
    const int x = (int)o[1] - 8;
    if (x <= -8 || x >= 160) continue;
    const uint8_t attr = o[3];
    int r = ly - ((int)o[0] - 16);
    if (attr & 0x40) r = h - 1 - r;
    uint8_t tile = o[2]; if (h == 16) tile &= 0xFE;
    const uint8_t *tp = gcVram + ((cgb && (attr & 8)) ? 0x2000 : 0) + tile * 16 + r * 2;
    const uint32_t w = (uint32_t)gcSpread[tp[0]] | ((uint32_t)gcSpread[tp[1]] << 1);
    if (!w) continue;                                  // fully transparent row
    const uint8_t pal = cgb ? (attr & 7) : ((attr >> 4) & 1);
    const uint8_t fl = (pal << 2) | ((attr & 0x80) >> 2);
    const bool fx = (attr & 0x20) != 0;
    const int p0 = x < 0 ? -x : 0, p1 = (x + 8 > 160) ? 160 - x : 8;
    for (int p = p0; p < p1; p++) {
      const uint32_t c = fx ? ((w >> (2 * p)) & 3) : ((w >> (14 - 2 * p)) & 3);
      if (c) spr[x + p] = c | fl;
    }
  }

  const uint16_t offCol = bgOn ? 0 : gbcColor(gbcRaw(gcBgRam, 0));   // DMG-compat, BG switched off
  // LCDC bit 0 clear in CGB mode: sprites always win. Otherwise a sprite loses to BG colours 1-3 when
  // the BG tile has the priority attribute or the OAM entry is flagged "behind BG".
  const bool masterOff = cgb && !(lcdc & 1);
  const uint8_t prioMask = masterOff ? 0x00 : 0x80, behindMask = masterOff ? 0x00 : 0x20;
  for (int x = 0; x < 160; x++) {
    const uint8_t s = spr[x], b = bgl[x];
    if (s && ((b & 3) == 0 || (!(b & prioMask) && !(s & behindMask)))) out[x] = gcObjTab[(s >> 2) & 7][s & 3];
    else out[x] = bgOn ? gcBgTab[(b >> 2) & 7][b & 3] : offCol;
  }
}

// Push 8 finished source lines (s0..s0+7) to the LCD, scaled 160x144 -> 172x155.
static GBC_FAST void gbcPushBand(int s0) {
  const int r0 = gcRowFirst[s0], r1 = gcRowFirst[s0 + 8];
  if (r1 <= r0) return;
  // Unchanged band (menus, static backgrounds, paused action): the LCD already shows it, skip the SPI transfer.
  {
    const uint32_t *wp = (const uint32_t *)gcBand;
    uint32_t h = 0x811C9DC5u;
    for (int i = 0; i < 8 * 160 / 2; i += 4) {
      h = (h ^ wp[i]) * 0x9E3779B1u; h = (h ^ wp[i + 1]) * 0x9E3779B1u;
      h = (h ^ wp[i + 2]) * 0x9E3779B1u; h = (h ^ wp[i + 3]) * 0x9E3779B1u;
    }
    h ^= h >> 15;
    const int bi = s0 >> 3;
    if (!lcdDirty && gcBandHash[bi] == h) return;
    gcBandHash[bi] = h;
  }
  SPI.beginTransaction(lcdCfg);
  lcdWindow(GAME_X, GAME_Y + r0, GAME_X + GBC_OUT_W - 1, GAME_Y + r1 - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  int last = -1;
  for (int r = r0; r < r1; r++) {
    const int s = gcRowMap[r];
    if (s != last) {                                   // scaled rows repeat a source line
      const uint16_t *src = gcBand + (s - s0) * 160;
      for (int x = 0; x < GBC_OUT_W; x++) outLine[x] = src[gcColMap[x]];
      last = s;
    }
    SPI.writeBytes((uint8_t *)outLine, GBC_OUT_W * 2);
    shotRow(r, outLine, GBC_OUT_W);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

static GBC_FAST void gbcPpuEvent() {
  gbcSerialCheck();
  while (gcPpuClk >= gcPpuLen) {
    gcPpuClk -= gcPpuLen;
    switch (gcMode) {
      case 2: gcMode = 3; gcPpuLen = 172; break;
      case 3: {
        gcMode = 0; gcPpuLen = 204;
        if (gcLY < 144) {
          const bool win = gcCgb ? ((gcLCDC & 0x20) && gcLY >= gcWY && gcWX < 167)
                                 : ((gcLCDC & 0x21) == 0x21 && gcLY >= gcWY && gcWX < 167);
          if (gcDraw) {
            gbcRenderLine(gcLY, win);
            if ((gcLY & 7) == 7) gbcPushBand(gcLY - 7);
          }
          if (win) gcWinLine++;
          if (gcHdmaOn) gbcHdmaBlock();                // one HBlank DMA block per visible line
        }
      } break;
      case 0:
        gcLY++;
        if (gcLY == 144) { gcMode = 1; gcPpuLen = 456; gcIF |= 1; }
        else             { gcMode = 2; gcPpuLen = 80; }
        break;
      default:
        if (++gcLY > 153) { gcLY = 0; gcMode = 2; gcPpuLen = 80; gcWinLine = 0; }
        break;
    }
    gbcStatUpdate();
  }
}

// c is in CPU clocks. The timer and DIV follow the CPU clock; the PPU always runs at the normal dot rate,
// so in double-speed mode it sees half as many dots per CPU clock.
static inline void gbcTick(int c) {
  gcCycles += c;
  if (gcTimerOn) { gcTimaCnt += c; if (gcTimaCnt >= gcTimaPeriod) gbcTimerAdvance(); }
  gcPpuClk += c >> gcSpeed;
  if (gcPpuClk >= gcPpuLen) gbcPpuEvent();
}

// ============================================================================
//  CPU
// ============================================================================
DRAM_ATTR static const uint8_t gbcCycM[256] = {
  1,3,2,2,1,1,2,1,5,2,2,2,1,1,2,1,  1,3,2,2,1,1,2,1,3,2,2,2,1,1,2,1,
  2,3,2,2,1,1,2,1,2,2,2,2,1,1,2,1,  2,3,2,2,3,3,3,1,2,2,2,2,1,1,2,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  2,3,3,4,3,4,2,4,2,4,3,1,3,6,2,4,  2,3,3,1,3,4,2,4,2,4,3,1,3,1,2,4,
  3,3,2,1,1,4,2,4,4,1,4,1,1,1,2,4,  3,3,2,1,1,4,2,4,3,2,4,1,1,1,2,4
};

static inline void gbcAlu(uint8_t op, uint8_t *R, uint8_t &F, uint8_t v) {
  const uint8_t a = R[7];
  unsigned r, c;
  switch (op) {
    case 0:  // ADD
      r = a + v;
      F = ((r & 0xFF) ? 0 : 0x80) | ((((a & 0xF) + (v & 0xF)) > 0xF) ? 0x20 : 0) | ((r > 0xFF) ? 0x10 : 0);
      R[7] = r; break;
    case 1:  // ADC
      c = (F >> 4) & 1; r = a + v + c;
      F = ((r & 0xFF) ? 0 : 0x80) | ((((a & 0xF) + (v & 0xF) + c) > 0xF) ? 0x20 : 0) | ((r > 0xFF) ? 0x10 : 0);
      R[7] = r; break;
    case 2:  // SUB
      r = a - v;
      F = 0x40 | ((r & 0xFF) ? 0 : 0x80) | (((a & 0xF) < (v & 0xF)) ? 0x20 : 0) | ((a < v) ? 0x10 : 0);
      R[7] = r; break;
    case 3:  // SBC
      c = (F >> 4) & 1; r = a - v - c;
      F = 0x40 | ((r & 0xFF) ? 0 : 0x80) | (((a & 0xF) < ((v & 0xF) + c)) ? 0x20 : 0) | (((unsigned)a < (unsigned)v + c) ? 0x10 : 0);
      R[7] = r; break;
    case 4: R[7] = a & v; F = R[7] ? 0x20 : 0xA0; break;   // AND
    case 5: R[7] = a ^ v; F = R[7] ? 0 : 0x80; break;      // XOR
    case 6: R[7] = a | v; F = R[7] ? 0 : 0x80; break;      // OR
    default:                                                // CP
      F = 0x40 | ((a == v) ? 0x80 : 0) | (((a & 0xF) < (v & 0xF)) ? 0x20 : 0) | ((a < v) ? 0x10 : 0);
      break;
  }
}

static inline bool gbcCond(uint8_t cc, uint8_t F) {
  switch (cc) {
    case 0:  return !(F & 0x80);
    case 1:  return  (F & 0x80);
    case 2:  return !(F & 0x10);
    default: return  (F & 0x10);
  }
}

#undef HLV
#undef RR_GET
#undef RR_SET
#undef PUSH16
#undef POP16
#undef FETCH16
#define HLV            ((uint16_t)((R[4] << 8) | R[5]))
#define RR_GET(p)      ((p) == 3 ? sp : (uint16_t)((R[2 * (p)] << 8) | R[2 * (p) + 1]))
#define RR_SET(p, val) do { uint16_t _v = (val); if ((p) == 3) sp = _v; else { R[2 * (p)] = _v >> 8; R[2 * (p) + 1] = _v; } } while (0)
#define PUSH16(val)    do { uint16_t _v = (val); sp--; gbcWr(sp, _v >> 8); sp--; gbcWr(sp, _v & 0xFF); } while (0)
#define POP16(dst)     do { uint16_t _lo = gbcRd(sp++); uint16_t _hi = gbcRd(sp++); dst = (_hi << 8) | _lo; } while (0)
#define FETCH16(dst)   do { uint16_t _lo = gbcRd(pc++); uint16_t _hi = gbcRd(pc++); dst = (_hi << 8) | _lo; } while (0)

// Run instructions until the global cycle counter reaches gcLimit (CPU clocks).
static GBC_FAST void gbcRun() {
  uint16_t pc = gcPC, sp = gcSP;
  uint8_t  F = gcF, ei = gcEiDelay;
  bool     ime = gcIME, halted = gcHalt, haltBug = gcHaltBug;
  uint8_t *const R = gcR;

  while ((int32_t)(gcLimit - gcCycles) > 0) {
    if (gcExtra) { const int e = gcExtra; gcExtra = 0; gbcTick(e); continue; }   // HDMA / speed-switch stall
    if (ei && --ei == 0) ime = true;

    const uint8_t pend = gcIF & gcIE & 0x1F;
    if (pend) {
      halted = false;
      if (ime) {                                       // service the interrupt
        int bit = __builtin_ctz(pend);
        gcIF &= ~(1 << bit);
        ime = false;
        PUSH16(pc);
        pc = 0x40 + (bit << 3);
        gbcTick(20);
        continue;
      }
    }
    if (halted) {                                      // skip ahead to the next event
      int32_t ps = gcPpuLen - gcPpuClk;
      if (ps > 0x3FFFFFFF) ps = 0x3FFFFFFF;
      int32_t step = ps << gcSpeed;
      if (gcTimerOn) { int32_t t = gcTimaPeriod - gcTimaCnt; if (t < step) step = t; }
      int32_t left = (int32_t)(gcLimit - gcCycles);
      if (step > left) step = left;
      if (step < 4) step = 4;
      gbcTick(step);
      continue;
    }

    uint8_t op = gbcRd(pc++);
    if (haltBug) { pc--; haltBug = false; }
    int cyc = gbcCycM[op] << 2;

    if ((op & 0xC0) == 0x40) {                         // LD r,r' / HALT
      if (op == 0x76) {
        if (!ime && (gcIF & gcIE & 0x1F)) haltBug = true; else halted = true;
      } else {
        const uint8_t d = (op >> 3) & 7, s = op & 7;
        uint8_t v;
        if (s == 6) { v = gbcRd(HLV); cyc = 8; } else v = R[s];
        if (d == 6) { gbcWr(HLV, v); cyc = 8; } else R[d] = v;
      }
    } else if ((op & 0xC0) == 0x80) {                  // ALU A,r
      const uint8_t s = op & 7;
      uint8_t v;
      if (s == 6) { v = gbcRd(HLV); cyc = 8; } else v = R[s];
      gbcAlu((op >> 3) & 7, R, F, v);
    } else switch (op) {
      case 0x00: break;
      case 0x01: R[1] = gbcRd(pc++); R[0] = gbcRd(pc++); break;
      case 0x11: R[3] = gbcRd(pc++); R[2] = gbcRd(pc++); break;
      case 0x21: R[5] = gbcRd(pc++); R[4] = gbcRd(pc++); break;
      case 0x31: FETCH16(sp); break;

      case 0x02: gbcWr((R[0] << 8) | R[1], R[7]); break;
      case 0x12: gbcWr((R[2] << 8) | R[3], R[7]); break;
      case 0x22: { uint16_t hl = HLV; gbcWr(hl, R[7]); hl++; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x32: { uint16_t hl = HLV; gbcWr(hl, R[7]); hl--; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x0A: R[7] = gbcRd((R[0] << 8) | R[1]); break;
      case 0x1A: R[7] = gbcRd((R[2] << 8) | R[3]); break;
      case 0x2A: { uint16_t hl = HLV; R[7] = gbcRd(hl); hl++; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x3A: { uint16_t hl = HLV; R[7] = gbcRd(hl); hl--; R[4] = hl >> 8; R[5] = hl; } break;

      case 0x03: case 0x13: case 0x23: case 0x33: { int p = op >> 4; RR_SET(p, RR_GET(p) + 1); } break;
      case 0x0B: case 0x1B: case 0x2B: case 0x3B: { int p = op >> 4; RR_SET(p, RR_GET(p) - 1); } break;

      case 0x04: case 0x0C: case 0x14: case 0x1C: case 0x24: case 0x2C: case 0x3C: {
        const uint8_t d = op >> 3, v = R[d] + 1; R[d] = v;
        F = (F & 0x10) | (v ? 0 : 0x80) | ((v & 0x0F) ? 0 : 0x20);
      } break;
      case 0x05: case 0x0D: case 0x15: case 0x1D: case 0x25: case 0x2D: case 0x3D: {
        const uint8_t d = op >> 3, v = R[d] - 1; R[d] = v;
        F = (F & 0x10) | 0x40 | (v ? 0 : 0x80) | (((v & 0x0F) == 0x0F) ? 0x20 : 0);
      } break;
      case 0x34: {
        uint16_t hl = HLV; uint8_t v = gbcRd(hl) + 1; gbcWr(hl, v);
        F = (F & 0x10) | (v ? 0 : 0x80) | ((v & 0x0F) ? 0 : 0x20);
      } break;
      case 0x35: {
        uint16_t hl = HLV; uint8_t v = gbcRd(hl) - 1; gbcWr(hl, v);
        F = (F & 0x10) | 0x40 | (v ? 0 : 0x80) | (((v & 0x0F) == 0x0F) ? 0x20 : 0);
      } break;

      case 0x06: case 0x0E: case 0x16: case 0x1E: case 0x26: case 0x2E: case 0x3E:
        R[op >> 3] = gbcRd(pc++); break;
      case 0x36: { uint8_t v = gbcRd(pc++); gbcWr(HLV, v); } break;

      case 0x07: { uint8_t c = R[7] >> 7; R[7] = (R[7] << 1) | c; F = c ? 0x10 : 0; } break;
      case 0x0F: { uint8_t c = R[7] & 1;  R[7] = (R[7] >> 1) | (c << 7); F = c ? 0x10 : 0; } break;
      case 0x17: { uint8_t c = (F >> 4) & 1, n = R[7] >> 7; R[7] = (R[7] << 1) | c; F = n ? 0x10 : 0; } break;
      case 0x1F: { uint8_t c = (F >> 4) & 1, n = R[7] & 1;  R[7] = (R[7] >> 1) | (c << 7); F = n ? 0x10 : 0; } break;

      case 0x08: { uint16_t a; FETCH16(a); gbcWr(a, sp & 0xFF); gbcWr(a + 1, sp >> 8); } break;

      case 0x09: case 0x19: case 0x29: case 0x39: {
        const uint16_t hl = HLV, v = RR_GET(op >> 4);
        const uint32_t r = (uint32_t)hl + v;
        F = (F & 0x80) | ((((hl & 0xFFF) + (v & 0xFFF)) > 0xFFF) ? 0x20 : 0) | ((r > 0xFFFF) ? 0x10 : 0);
        R[4] = r >> 8; R[5] = r;
      } break;

      case 0x10:                                                 // STOP: CGB speed switch when armed
        pc++;
        if (gcCgb && (gcKey1 & 1)) gbcSpeedSwitch();
        break;
      case 0x18: { int8_t e = (int8_t)gbcRd(pc++); pc += e; } break;
      case 0x20: case 0x28: case 0x30: case 0x38: {
        int8_t e = (int8_t)gbcRd(pc++);
        if (gbcCond((op >> 3) & 3, F)) { pc += e; cyc += 4; }
      } break;

      case 0x27: {                                               // DAA
        uint8_t a = R[7];
        if (!(F & 0x40)) {
          if ((F & 0x10) || a > 0x99) { a += 0x60; F |= 0x10; }
          if ((F & 0x20) || (a & 0x0F) > 9) a += 6;
        } else {
          if (F & 0x10) a -= 0x60;
          if (F & 0x20) a -= 6;
        }
        F = (F & 0x50) | (a ? 0 : 0x80);
        R[7] = a;
      } break;
      case 0x2F: R[7] = ~R[7]; F |= 0x60; break;                 // CPL
      case 0x37: F = (F & 0x80) | 0x10; break;                   // SCF
      case 0x3F: F = (F & 0x80) | ((F & 0x10) ^ 0x10); break;    // CCF

      case 0xC0: case 0xC8: case 0xD0: case 0xD8:
        if (gbcCond((op >> 3) & 3, F)) { POP16(pc); cyc += 12; }
        break;
      case 0xC9: POP16(pc); break;
      case 0xD9: POP16(pc); ime = true; ei = 0; break;           // RETI

      case 0xC1: case 0xD1: case 0xE1: { uint16_t v; POP16(v); int p = (op >> 4) & 3; R[2 * p] = v >> 8; R[2 * p + 1] = v; } break;
      case 0xF1: { uint16_t v; POP16(v); R[7] = v >> 8; F = v & 0xF0; } break;
      case 0xC5: case 0xD5: case 0xE5: { int p = (op >> 4) & 3; PUSH16((R[2 * p] << 8) | R[2 * p + 1]); } break;
      case 0xF5: PUSH16((R[7] << 8) | F); break;

      case 0xC2: case 0xCA: case 0xD2: case 0xDA: {
        uint16_t a; FETCH16(a);
        if (gbcCond((op >> 3) & 3, F)) { pc = a; cyc += 4; }
      } break;
      case 0xC3: FETCH16(pc); break;
      case 0xE9: pc = HLV; break;

      case 0xC4: case 0xCC: case 0xD4: case 0xDC: {
        uint16_t a; FETCH16(a);
        if (gbcCond((op >> 3) & 3, F)) { PUSH16(pc); pc = a; cyc += 12; }
      } break;
      case 0xCD: { uint16_t a; FETCH16(a); PUSH16(pc); pc = a; } break;

      case 0xC6: case 0xCE: case 0xD6: case 0xDE: case 0xE6: case 0xEE: case 0xF6: case 0xFE: {
        uint8_t v = gbcRd(pc++);
        gbcAlu((op >> 3) & 7, R, F, v);
      } break;
      case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF:
        PUSH16(pc); pc = op & 0x38; break;

      case 0xE0: { uint8_t n = gbcRd(pc++); gbcWr(0xFF00 | n, R[7]); } break;
      case 0xF0: { uint8_t n = gbcRd(pc++); R[7] = gbcRd(0xFF00 | n); } break;
      case 0xE2: gbcWr(0xFF00 | R[1], R[7]); break;
      case 0xF2: R[7] = gbcRd(0xFF00 | R[1]); break;
      case 0xEA: { uint16_t a; FETCH16(a); gbcWr(a, R[7]); } break;
      case 0xFA: { uint16_t a; FETCH16(a); R[7] = gbcRd(a); } break;

      case 0xE8: case 0xF8: {
        const int8_t e = (int8_t)gbcRd(pc++);
        const uint16_t r = sp + e;
        F = ((((sp & 0xF) + (e & 0xF)) > 0xF) ? 0x20 : 0) | ((((sp & 0xFF) + (e & 0xFF)) > 0xFF) ? 0x10 : 0);
        if (op == 0xE8) sp = r; else { R[4] = r >> 8; R[5] = r; }
      } break;
      case 0xF9: sp = HLV; break;

      case 0xF3: ime = false; ei = 0; break;                     // DI
      case 0xFB: ei = 2; break;                                  // EI

      case 0xCB: {
        const uint8_t cb = gbcRd(pc++);
        const uint8_t r = cb & 7, grp = cb >> 6, y = (cb >> 3) & 7;
        uint8_t v = (r == 6) ? gbcRd(HLV) : R[r];
        cyc = (r == 6) ? ((grp == 1) ? 12 : 16) : 8;
        if (grp == 0) {
          uint8_t c;
          switch (y) {
            case 0: c = v >> 7; v = (v << 1) | c; break;                   // RLC
            case 1: c = v & 1;  v = (v >> 1) | (c << 7); break;            // RRC
            case 2: c = v >> 7; v = (v << 1) | ((F >> 4) & 1); break;      // RL
            case 3: c = v & 1;  v = (v >> 1) | ((F & 0x10) << 3); break;   // RR
            case 4: c = v >> 7; v <<= 1; break;                            // SLA
            case 5: c = v & 1;  v = (v >> 1) | (v & 0x80); break;          // SRA
            case 6: c = 0;      v = (v << 4) | (v >> 4); break;            // SWAP
            default: c = v & 1; v >>= 1; break;                            // SRL
          }
          F = (v ? 0 : 0x80) | (c ? 0x10 : 0);
        } else if (grp == 1) {                                             // BIT
          F = (F & 0x10) | 0x20 | ((v & (1 << y)) ? 0 : 0x80);
        } else if (grp == 2) v &= ~(1 << y);                               // RES
        else v |= (1 << y);                                                // SET
        if (grp != 1) { if (r == 6) gbcWr(HLV, v); else R[r] = v; }
      } break;

      default: break;                                          // illegal opcodes: NOP
    }
    gbcTick(cyc);
  }
  gcPC = pc; gcSP = sp; gcF = F;
  gcIME = ime; gcHalt = halted; gcHaltBug = haltBug; gcEiDelay = ei;
}

// ============================================================================
//  Frame, input, saves
// ============================================================================
static void gbcJoypadUpdate() {
  const uint8_t s = padState;                       // A B SEL START UP DOWN LEFT RIGHT
  gcPadBtn = s & 0x0F;
  gcPadDir = ((s >> 7) & 1) | ((s >> 5) & 2) | ((s >> 2) & 0x0C);
  const uint8_t now = gbcJoyNibble();
  if ((gcJoyPrev & ~now) & 0x0F) gcIF |= 0x10;      // a selected button went down
  gcJoyPrev = now;
}

static void gbcSaveRam() {
  if (!gcBattery || !gcRam || !gcRamDirty || !gcSavPath[0]) return;
  // Keep dirty state set on any failure so the next autosave/exit retries.
  if (sdAtomicSaveWrite(gcSavPath, gcRam, gcSavSize)) gcRamDirty = false;
}

static void gbcFrame(bool draw) {
  gbcInitDecodeLut();
  // MBC3 RTC registers are clocked independently of game reads/writes. Tick
  // once per emulated frame so games that poll the RTC without re-latching do
  // not see a frozen clock; gbcRtcTick() uses millis() and accounts for stalls.
  if (gcHasRtc) gbcRtcTick();
  gcDraw = draw;
  gbcJoypadUpdate();
  gcLimit += (uint32_t)GBC_DOTS_FRAME << gcSpeed;   // a frame is always 70224 dots, i.e. twice the CPU clocks at 2x
  gbcRun();
  if (draw) lcdDirty = false;                       // every band was compared against the LCD contents this frame
  if (!(gcLCDC & 0x80)) {                           // LCD off: white screen, keep the counter small
    gcPpuClk = 0;
    if (draw) { lcdFill(GAME_X, GAME_Y, GBC_OUT_W, GBC_OUT_H, 0xFFFF); shotFillAll(0xFFFF); lcdDirty = true; }
  }
  gbcSerialCheck();
  gcFrameNo++;
  if (gcRamDirty && (gcFrameNo - gcRamStamp) > 120) gbcSaveRam();   // ~2 s after the last write
}

// ============================================================================
//  Load / reset / shutdown
// ============================================================================
static void gbcFreeAll() {
  if (gcRomFile) gcRomFile.close();
  for (int i = 0; i < GBC_MAX_SLOTS; i++) { free(gcPgData[i]); gcPgData[i] = nullptr; gcPgTag[i] = 0xFFFF; gcPgStamp[i] = 0; }
  memset(gcPgSlot, 0xFF, sizeof(gcPgSlot));
  gcSlots = 0; gcStamp = 0;
  free(gcRam); gcRam = nullptr; gcRamPtr = nullptr;
  free(gcMem); gcMem = gcWram = gcVram = gcVramCur = nullptr;
  free(gcBootPage); gcBootPage = nullptr;
  for (int i = 0; i < 16; i++) gcMap[i] = gcWMap[i] = nullptr;
  gcRomReadError = false;
  gcRamDirty = false;
  gcSavPath[0] = 0;
}

// Everything that does not depend on the game is built here, before the first frame:
// tile-decode table, scaling maps, band hashes.
static void gbcBuildTables() {
  gbcInitDecodeLut();
  for (int x = 0; x < GBC_OUT_W; x++) gcColMap[x] = (x * 160) / GBC_OUT_W;
  for (int y = 0; y < GBC_OUT_H; y++) gcRowMap[y] = (y * 144) / GBC_OUT_H;
  for (int s = 0; s <= 144; s++) { int r = 0; while (r < GBC_OUT_H && gcRowMap[r] < s) r++; gcRowFirst[s] = r; }
  memset(gcBandHash, 0, sizeof(gcBandHash));
  lcdDirty = true;
}

static bool gbcFail(const char *l1, const char *l2) {
  strlcpy(loadErr1, l1, sizeof(loadErr1));
  strlcpy(loadErr2, l2, sizeof(loadErr2));
  gbcFreeAll();
  return false;
}

static bool gbcLoad(const char *path) {
  gbcFreeAll();
  gbcBuildTables();
  gcBootAvail = false; gcBootActive = false;
  // Optional 2304 byte CGB boot ROM supplied by the user.
  static const char *const biosNames[] = { "cgb_boot.bin", "cgb_bios.bin", "gbc_bios.bin", "gbc_boot.bin", "boot.bin" };
  char biosPath[96];
  gcBootAvail = biosLoadFirst("gbc", biosNames, sizeof(biosNames) / sizeof(biosNames[0]),
                              gcBootRom, sizeof(gcBootRom), biosPath, sizeof(biosPath));
  gcRomReadError = false;
  gcRomFile = SD_MMC.open(path, FILE_READ);
  if (!gcRomFile || gcRomFile.isDirectory()) return gbcFail("ROM NOT FOUND", "CHECK SD CARD");
  const size_t sz = gcRomFile.size();
  if (sz < 0x8000 || sz > 512UL * 16384 || (sz % 16384) != 0) return gbcFail("BAD ROM SIZE", "NOT A GB ROM");

  uint8_t h[0x50];                                   // header 0x100-0x14F
  if (sdDiagReadAt(gcRomFile, 0x100, h, sizeof(h)) != (int)sizeof(h)) return gbcFail("ROM TRUNCATED", "HEADER");
  const uint8_t cgb = h[0x43], type = h[0x47], ramCode = h[0x49];
  gcHdrCgb = (cgb & 0x80) != 0;                      // 0x80 = CGB enhanced, 0xC0 = CGB only

  switch (type) {
    case 0x00: case 0x08: case 0x09:                                   gcMbc = 0; break;
    case 0x01: case 0x02: case 0x03:                                   gcMbc = 1; break;
    case 0x0B: case 0x0C: case 0x0D:                                   gcMbc = 7; break; // MMM01: basic MBC1-compatible banking
    case 0x05: case 0x06:                                              gcMbc = 2; break;
    case 0x0F: case 0x10: case 0x11: case 0x12: case 0x13:             gcMbc = 3; break;
    case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E:  gcMbc = 5; break;
    case 0xFF:                                                        gcMbc = 6; break; // HuC1: MBC1-style ROM/RAM banking
    default: { char b[24]; snprintf(b, sizeof(b), "MBC TYPE %02X", type); return gbcFail(b, "NOT SUPPORTED"); }
  }
  gcBattery = (type == 0x03 || type == 0x06 || type == 0x09 || type == 0x0D || type == 0x0F || type == 0x10 ||
               type == 0x13 || type == 0x1B || type == 0x1E || type == 0xFF);
  gcHasRtc  = (type == 0x0F || type == 0x10);

  static const uint32_t ramTab[6] = { 0, 2048, 8192, 32768, 131072, 65536 };
  uint32_t ramSize = ramCode < 6 ? ramTab[ramCode] : 0;
  gcRamAlloc = (gcMbc == 2) ? 512 : (ramSize == 0 ? 0 : (ramSize < 8192 ? 8192 : ramSize));
  gcSavSize = gcRamAlloc;
  gcRomBanks = sz / 16384;
  gcRomPages = sz / 4096;

  // Reserve the minimum ROM page cache FIRST, before cart RAM / WRAM fragment the heap; top it up afterwards.
  const int want = gcRomPages < (uint32_t)GBC_MAX_SLOTS ? (int)gcRomPages : GBC_MAX_SLOTS;
  const int minSlots = gcRomPages < (uint32_t)GBC_MIN_SLOTS ? (int)gcRomPages : GBC_MIN_SLOTS;
  gcSlots = 0;
  while (gcSlots < minSlots) {
    uint8_t *p = (uint8_t *)emuAlloc(4096, 0);
    if (!p) break;
    gcPgData[gcSlots++] = p;
  }
  if (gcSlots < minSlots) { Serial.printf("ROM CACHE: only %d slots; internal free=%u largest=%u psram=%d\n", gcSlots, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), (int)psramFound()); return gbcFail("NOT ENOUGH RAM", "ROM CACHE"); }
  gcMem = (uint8_t *)emuAlloc(0x8000 + 0x4000, 0, true);     // 8 WRAM banks + 2 VRAM banks
  if (!gcMem) return gbcFail("NOT ENOUGH RAM", "GBC MEMORY");
  gcWram = gcMem; gcVram = gcMem + 0x8000; gcVramCur = gcVram;
  if (gcRamAlloc) {
    gcRam = (uint8_t *)emuAlloc(gcRamAlloc, 0, true);
    if (!gcRam) { char b[24]; snprintf(b, sizeof(b), "CART RAM %uK", (unsigned)(gcRamAlloc / 1024)); return gbcFail("NOT ENOUGH RAM", b); }
  }
  if (gcBootAvail) {
    gcBootPage = (uint8_t *)emuAlloc(4096, 0, true);
    if (!gcBootPage) gcBootAvail = false;            // boot ROM is optional: carry on without it
  }
  while (gcSlots < want) {
    uint8_t *p = (uint8_t *)emuAlloc(4096, GBC_HEAP_RESERVE);
    if (!p) break;
    gcPgData[gcSlots++] = p;
  }
  for (int i = 0; i < GBC_MAX_SLOTS; i++) { gcPgTag[i] = 0xFFFF; gcPgStamp[i] = 0; }
  memset(gcPgSlot, 0xFF, sizeof(gcPgSlot));
  gcStamp = 0;

  // The ROM always stays on the SD card: the file remains open and pages are streamed into the small cache.
  const bool all = false;
  gbcPage(0);                                         // first page of bank 0 (header + reset vectors)
  // Pre-load the start of the ROM with sequential reads (fast, no seeks) so the first moments of the game
  // do not stall on the SD card. Two slots stay free for the CPU windows.
  for (int pg = 1; pg < (int)gcRomPages && pg < gcSlots - 2 && !gcRomReadError; pg++) gbcPage(pg);
  if (gcRomReadError) return gbcFail("ROM READ ERROR", "CHECK SD CARD");

  // battery RAM
  gcSavPath[0] = 0;
  if (gcBattery && gcRam) {
    if (sdBuildSavePath(path, "gbc", gcSavPath, sizeof(gcSavPath))) {
      if (!sdAtomicSaveLoad(gcSavPath, gcRam, gcSavSize)) {
        char legacyPath[ROM_NAME_LEN + 32];
        bool imported = false;
        // Migrate the former /saves/<system>/<basename>.sav layout only when
        // the ROM is in the system root, where the old name is unambiguous.
        if (sdBuildLegacyManagedSavePath(path, "gbc", legacyPath, sizeof(legacyPath)) &&
            strcmp(legacyPath, gcSavPath) != 0 &&
            sdAtomicSaveLoad(legacyPath, gcRam, gcSavSize)) {
          imported = true;
          Serial.printf("GBC: migrated managed save; next write goes to %s\n", gcSavPath);
        }
        // Older releases also stored saves beside the ROM. This path includes
        // the full ROM path, so it remains safe for same-named games elsewhere.
        if (!imported && sdBuildLegacySavePath(path, legacyPath, sizeof(legacyPath)) &&
            sdAtomicSaveLoad(legacyPath, gcRam, gcSavSize)) {
          Serial.printf("GBC: imported beside-ROM save; next write goes to %s\n", gcSavPath);
        }
      }
    } else {
      gcSavPath[0] = 0;
      Serial.println("GBC: could not construct save path");
    }
  }

  Serial.printf("GBC: %uK ROM, mapper %u, RAM %uK%s%s, %s, %s, cache %d/%u pages%s\n",
                (unsigned)(sz / 1024), gcMbc, (unsigned)(gcRamAlloc / 1024), gcBattery ? " +battery" : "",
                gcHasRtc ? " +rtc" : "", gcHdrCgb ? (cgb == 0xC0 ? "CGB only" : "CGB enhanced") : "DMG game",
                gcBootAvail ? "boot ROM" : "no boot ROM", gcSlots, (unsigned)gcRomPages, " (streaming from SD)");
  Serial.printf("Free heap after load: %u bytes\n", (unsigned)ESP.getFreeHeap());
  return true;
}

static void gbcReset() {
  memset(gcWram, 0, 0x8000); memset(gcVram, 0, 0x4000);
  memset(gcOam, 0, sizeof(gcOam)); memset(gcHram, 0, sizeof(gcHram));
  memset(gcIo, 0xFF, sizeof(gcIo));
  gcSpeed = 0; gcKey1 = 0; gcVbk = 0; gcSvbkRaw = 1; gcExtra = 0;
  gcHdmaOn = false; gcHdmaLeft = 0; gcHdmaReg = 0xFF; gcHdmaSrc = 0; gcHdmaDst = 0x8000;
  gcBcps = gcOcps = 0;
  for (int i = 0; i < 64; i += 2) { gcBgRam[i] = gcObjRam[i] = 0xFF; gcBgRam[i + 1] = gcObjRam[i + 1] = 0x7F; }   // white

  gcBootActive = gcBootAvail;
  gcLCDC = 0; gcSTAT = 0; gcSCY = gcSCX = gcLY = gcLYC = gcWY = gcWX = 0; gcWinLine = 0;
  gcBGP = gcOBP0 = gcOBP1 = 0;

  if (gcBootActive) {
    // Real boot ROM: start from power-on. The boot ROM itself runs in CGB mode until it writes KEY0.
    gcCgb = true; gcOpri = 0;
    memset(gcR, 0, sizeof(gcR)); gcF = 0; gcSP = 0; gcPC = 0x0000;
    gcIME = gcHalt = gcHaltBug = false; gcEiDelay = 0; gcIF = 0; gcIE = 0;
    gcMode = 0; gcPpuClk = 0; gcPpuLen = 0x7FFFFFFF; gcStatLine = false;
  } else {
    static const uint8_t snd[][2] = {                // sound registers after the boot ROM
      {0x10,0x80},{0x11,0xBF},{0x12,0xF3},{0x13,0xFF},{0x14,0xBF},{0x16,0x3F},{0x17,0x00},{0x18,0xFF},
      {0x19,0xBF},{0x1A,0x7F},{0x1B,0xFF},{0x1C,0x9F},{0x1D,0xFF},{0x1E,0xBF},{0x20,0xFF},{0x21,0x00},
      {0x22,0x00},{0x23,0xBF},{0x24,0x77},{0x25,0xF3},{0x26,0xF1} };
    for (unsigned i = 0; i < sizeof(snd) / 2; i++) gcIo[snd[i][0]] = snd[i][1];
    gcIo[1] = 0; gcIo[2] = 0x7F; gcIo[0x46] = 0x00;

    gcCgb = gcHdrCgb;                                // what the boot ROM would have decided
    gcOpri = gcCgb ? 0 : 1;
    gcR[0] = 0x00; gcR[1] = 0x00; gcR[2] = gcCgb ? 0xFF : 0x00; gcR[3] = gcCgb ? 0x56 : 0x08;
    gcR[4] = 0x00; gcR[5] = gcCgb ? 0x0D : 0x7C; gcR[6] = 0; gcR[7] = 0x11;
    gcF = 0x80; gcSP = 0xFFFE; gcPC = 0x0100;
    gcIME = gcHalt = gcHaltBug = false; gcEiDelay = 0; gcIF = 0x01; gcIE = 0;

    gcLCDC = 0x91; gcBGP = 0xFC; gcOBP0 = gcOBP1 = 0xFF;
    gcMode = 2; gcPpuClk = 0; gcPpuLen = 80; gcStatLine = false;

    if (!gcCgb) {                                    // DMG game: BG, OBJ0 and OBJ1 all use the chosen scheme
      for (int pal = 0; pal < 3; pal++) {
        for (int c = 0; c < 4; c++) {
          const uint32_t rgb = gbcDmgRGB[GBC_DMG_PALETTE][c];
          const uint16_t c15 = (((rgb >> 16) & 0xFF) >> 3) | ((((rgb >> 8) & 0xFF) >> 3) << 5) | (((rgb & 0xFF) >> 3) << 10);
          if (pal == 0) { gcBgRam[c * 2] = c15 & 0xFF; gcBgRam[c * 2 + 1] = c15 >> 8; }
          else { gcObjRam[(pal - 1) * 8 + c * 2] = c15 & 0xFF; gcObjRam[(pal - 1) * 8 + c * 2 + 1] = c15 >> 8; }
        }
      }
    }
  }

  gcCycles = gcLimit = gcDivZero = 0; gcFrameNo = 0; gcSerOn = false;
  gcTIMA = gcTMA = gcTAC = 0; gcTimaCnt = 0; gcTimaPeriod = 1024; gcTimerOn = false;
  gcJoySel = 0x30; gcPadBtn = gcPadDir = 0; gcJoyPrev = 0x0F;
  gcDraw = true;

  gcB1 = 1; gcB2 = 0; gcMode1 = false; gcRamBank = 0; gcLatch = 0xFF;
  // ROM-only cartridges with built-in RAM (types 08/09) have no RAM-enable
  // register. Keep their RAM mapped from power-on; MBC cartridges still start
  // disabled and must receive the usual 0x0A enable write.
  gcRamEn = (gcMbc == 0 && gcRamAlloc != 0);
  gcRamDirty = false; gcRamStamp = 0;
  memset(gcRtc, 0, sizeof(gcRtc)); memset(gcRtcLat, 0, sizeof(gcRtcLat)); gcRtcLast = millis(); gcRtcSub = 0;

  for (int i = 0; i < 16; i++) gcMap[i] = gcWMap[i] = nullptr;
  gcMap[0xC] = gcWMap[0xC] = gcWram;                 // WRAM bank 0
  gcMap[0xE] = gcWMap[0xE] = gcWram;                 // echo RAM
  gbcMapVram();
  gbcMapWram();
  gbcMapRom();
  gbcMapRam();
  gbcRebuildAll();

  memset(gcBandHash, 0, sizeof(gcBandHash));
  lcdDirty = true;                                   // repaint every band on the first frame
}

// Versioned save-state payload; derived page-map/cache pointers are rebuilt on load.
struct GbcStateCore {
  uint32_t ramAlloc,savSize,cycles,limit,divZero,serEnd,frameNo,rtcLast,rtcSub,ramStamp,stamp,base0,base1,wbankOff;
  uint16_t b1,pc,sp,hdmaSrc,hdmaDst;
  uint8_t mbc,b2,ramBank,latch,rtc[5],rtcLat[5],r[8],f,eiDelay,iff,ie,tima,tma,tac,joySel,padBtn,padDir,joyPrev;
  uint8_t speed,key1,vbk,svbkRaw,opri,bcps,ocps,hdmaLeft,hdmaReg;
  uint8_t lcdc,stat,scy,scx,ly,lyc,bgp,obp0,obp1,wy,wx,mode,winLine;
  int32_t timaCnt,timaPeriod,ppuClk,ppuLen,extra;
  bool battery,hasRtc,hdrCgb,ramEn,mode1,ime,halt,haltBug,serOn,timerOn,cgb,hdmaOn,statLine,draw;
  uint8_t oam[160],hram[127],io[128],bgRam[64],objRam[64];
};
static bool gbcStateSave(uint8_t slot = 0) {
  if(!gcMem||!currentRomPath[0]||!sdMount()) return false;
  char path[192]; if(!sdBuildStatePath(currentRomPath,"gbc",path,sizeof(path),slot)) return false;
  GbcStateCore c{};
  c.ramAlloc=gcRamAlloc;c.savSize=gcSavSize;c.cycles=gcCycles;c.limit=gcLimit;c.divZero=gcDivZero;c.serEnd=gcSerEnd;c.frameNo=gcFrameNo;c.rtcLast=gcRtcLast;c.rtcSub=gcRtcSub;c.ramStamp=gcRamStamp;c.stamp=gcStamp;c.base0=gcBase0;c.base1=gcBase1;c.wbankOff=gcWbankOff;
  c.b1=gcB1;c.pc=gcPC;c.sp=gcSP;c.hdmaSrc=gcHdmaSrc;c.hdmaDst=gcHdmaDst;c.mbc=gcMbc;c.b2=gcB2;c.ramBank=gcRamBank;c.latch=gcLatch;memcpy(c.rtc,gcRtc,5);memcpy(c.rtcLat,gcRtcLat,5);memcpy(c.r,gcR,8);c.f=gcF;c.eiDelay=gcEiDelay;c.iff=gcIF;c.ie=gcIE;c.tima=gcTIMA;c.tma=gcTMA;c.tac=gcTAC;c.joySel=gcJoySel;c.padBtn=gcPadBtn;c.padDir=gcPadDir;c.joyPrev=gcJoyPrev;
  c.speed=gcSpeed;c.key1=gcKey1;c.vbk=gcVbk;c.svbkRaw=gcSvbkRaw;c.opri=gcOpri;c.bcps=gcBcps;c.ocps=gcOcps;c.hdmaLeft=gcHdmaLeft;c.hdmaReg=gcHdmaReg;
  c.lcdc=gcLCDC;c.stat=gcSTAT;c.scy=gcSCY;c.scx=gcSCX;c.ly=gcLY;c.lyc=gcLYC;c.bgp=gcBGP;c.obp0=gcOBP0;c.obp1=gcOBP1;c.wy=gcWY;c.wx=gcWX;c.mode=gcMode;c.winLine=gcWinLine;
  c.timaCnt=gcTimaCnt;c.timaPeriod=gcTimaPeriod;c.ppuClk=gcPpuClk;c.ppuLen=gcPpuLen;c.extra=gcExtra;c.battery=gcBattery;c.hasRtc=gcHasRtc;c.hdrCgb=gcHdrCgb;c.ramEn=gcRamEn;c.mode1=gcMode1;c.ime=gcIME;c.halt=gcHalt;c.haltBug=gcHaltBug;c.serOn=gcSerOn;c.timerOn=gcTimerOn;c.cgb=gcCgb;c.hdmaOn=gcHdmaOn;c.statLine=gcStatLine;c.draw=true;
  memcpy(c.oam,gcOam,sizeof(gcOam));memcpy(c.hram,gcHram,sizeof(gcHram));memcpy(c.io,gcIo,sizeof(gcIo));memcpy(c.bgRam,gcBgRam,sizeof(gcBgRam));memcpy(c.objRam,gcObjRam,sizeof(gcObjRam));
  bool ok=sdStateWrite(path,SYS_GBC,&c,sizeof(c),gcMem,0xC000,gcRam,gcRamAlloc);Serial.printf("STATE GBC: %s (%s)\n",ok?"saved":"save failed",path);return ok;
}
static bool gbcStateLoad(uint8_t slot = 0) {
  if(!gcMem||!currentRomPath[0]||!sdMount()) return false;
  char path[192];if(!sdBuildStatePath(currentRomPath,"gbc",path,sizeof(path),slot))return false;GbcStateCore c{};
  if(!sdStateRead(path,SYS_GBC,&c,sizeof(c),gcMem,0xC000,gcRam,gcRamAlloc)||c.ramAlloc!=gcRamAlloc||c.mbc!=gcMbc)return false;
  gcSavSize=c.savSize;gcCycles=c.cycles;gcLimit=c.limit;gcDivZero=c.divZero;gcSerEnd=c.serEnd;gcFrameNo=c.frameNo;gcRtcLast=millis();gcRtcSub=c.rtcSub;gcRamStamp=c.ramStamp;gcStamp=c.stamp;gcBase0=c.base0;gcBase1=c.base1;gcWbankOff=c.wbankOff;
  gcB1=c.b1;gcPC=c.pc;gcSP=c.sp;gcHdmaSrc=c.hdmaSrc;gcHdmaDst=c.hdmaDst;gcB2=c.b2;gcRamBank=c.ramBank;gcLatch=c.latch;memcpy(gcRtc,c.rtc,5);memcpy(gcRtcLat,c.rtcLat,5);memcpy(gcR,c.r,8);gcF=c.f;gcEiDelay=c.eiDelay;gcIF=c.iff;gcIE=c.ie;gcTIMA=c.tima;gcTMA=c.tma;gcTAC=c.tac;gcJoySel=c.joySel;gcPadBtn=c.padBtn;gcPadDir=c.padDir;gcJoyPrev=c.joyPrev;
  gcSpeed=c.speed;gcKey1=c.key1;gcVbk=c.vbk;gcSvbkRaw=c.svbkRaw;gcOpri=c.opri;gcBcps=c.bcps;gcOcps=c.ocps;gcHdmaLeft=c.hdmaLeft;gcHdmaReg=c.hdmaReg;gcLCDC=c.lcdc;gcSTAT=c.stat;gcSCY=c.scy;gcSCX=c.scx;gcLY=c.ly;gcLYC=c.lyc;gcBGP=c.bgp;gcOBP0=c.obp0;gcOBP1=c.obp1;gcWY=c.wy;gcWX=c.wx;gcMode=c.mode;gcWinLine=c.winLine;
  gcTimaCnt=c.timaCnt;gcTimaPeriod=c.timaPeriod;gcPpuClk=c.ppuClk;gcPpuLen=c.ppuLen;gcExtra=c.extra;gcRamEn=c.ramEn;gcMode1=c.mode1;gcIME=c.ime;gcHalt=c.halt;gcHaltBug=c.haltBug;gcSerOn=c.serOn;gcTimerOn=c.timerOn;gcCgb=c.cgb;gcHdmaOn=c.hdmaOn;gcStatLine=c.statLine;gcDraw=true;
  memcpy(gcOam,c.oam,sizeof(gcOam));memcpy(gcHram,c.hram,sizeof(gcHram));memcpy(gcIo,c.io,sizeof(gcIo));memcpy(gcBgRam,c.bgRam,sizeof(gcBgRam));memcpy(gcObjRam,c.objRam,sizeof(gcObjRam));
  for(int i=0;i<16;i++)gcMap[i]=gcWMap[i]=nullptr;gcMap[0xC]=gcWMap[0xC]=gcWram;gcMap[0xE]=gcWMap[0xE]=gcWram;gbcMapVram();gbcMapWram();gbcMapRom();gbcMapRam();gbcRebuildAll();memset(gcBandHash,0,sizeof(gcBandHash));if(gcRam)gcRamDirty=true;lcdDirty=true;return true;
}

static void gbcShutdown() {
  gbcSaveRam();
  gbcFreeAll();
}

#if GBC_OPT_O3
#pragma GCC pop_options
#endif
