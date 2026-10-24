// ============================================================================
//  Game Boy (DMG) emulator
//   - Sharp LR35902 CPU (every opcode, instruction-level timing), DIV/TIMA
//     timer, VBlank / STAT / timer / serial / joypad interrupts, HALT, EI delay
//   - Scanline renderer: background, window, 8x8 and 8x16 sprites, OAM DMA
//   - Cartridges: ROM only, MBC1, MBC2, MBC3 (with a simple RTC), MBC5
//   - Battery saves go to <rom name>.sav next to the ROM on the SD card
//   - ROM banks live in a RAM cache. A ROM that does not fit stays on the SD
//     card and 16 KB banks are loaded on demand (least recently used first),
//     so big games work too (a bank switch that misses costs a few ms).
//   - Output is 160x144 scaled to 172x155 and pinned to the top of the portrait
//     172x320 panel (same place as the NES picture)
//   - Not emulated: audio, Game Boy Color (CGB-only games are refused)
// ============================================================================
#define GB_MAX_SLOTS     32              // cache more 16 KB ROM banks when free heap permits (SD remains the backing store)
#define GB_HEAP_RESERVE  (56 * 1024)     // heap kept free when sizing the bank cache
#define GB_PALETTE       0               // 0 = classic green, 1 = grey, 2 = pocket
#define GB_FRAME_CYCLES  70224           // 154 lines * 456 clocks
#define GB_OUT_W         GAME_W          // 172: scaled picture width (fits outLine[])
#define GB_OUT_H         155             // 144 * 172 / 160
static_assert(GB_OUT_W <= GAME_W, "outLine[] is only GAME_W pixels wide");

static const uint32_t gbPalRGB[3][4] = {
  { 0x9BBC0F, 0x8BAC0F, 0x306230, 0x0F380F },
  { 0xFFFFFF, 0xAAAAAA, 0x555555, 0x000000 },
  { 0xE0F8D0, 0x88C070, 0x346856, 0x081820 },
};

// ---- memory -----------------------------------------------------------------
static uint8_t gGbBootRom[256];
static bool gGbBootAvailable = false, gGbBootActive = false;
static uint8_t *gMem = nullptr;                 // WRAM + VRAM + frame buffer in one block
static uint8_t *gWram = nullptr, *gVram = nullptr, *gFb = nullptr;
static uint8_t  gOam[160], gHram[127], gIo[128];
static uint8_t *gMap[16];                       // read map, 4 KB pages (null = slow path)
static uint8_t *gWMap[16];                      // write map (VRAM / WRAM only)

// ---- cartridge --------------------------------------------------------------
static File     gRomFile;
static uint32_t gRomBanks = 2;
static uint8_t  gMbc = 0;                       // 0 none, 1, 2, 3, 5
static bool     gMbc1M = false;                 // MBC1 multicart wiring (4-bit low bank register)
static bool     gBattery = false, gHasRtc = false;
static uint8_t *gRam = nullptr;
static uint32_t gRamAlloc = 0, gSavSize = 0;
static bool gRomReadError = false;
static uint8_t *gRamPtr = nullptr;              // currently mapped 8 KB RAM window
static bool     gRamEn = false, gMode1 = false, gRamDirty = false;
static uint16_t gB1 = 1;                        // ROM bank register (MBC1: 5 bit)
static uint8_t  gB2 = 0, gRamBank = 0, gLatch = 0xFF;
static uint32_t gRamStamp = 0;
static char     gSavPath[ROM_NAME_LEN + 24];

static uint8_t *gSlotData[GB_MAX_SLOTS];        // ROM bank cache
static uint16_t gSlotBank[GB_MAX_SLOTS];
static uint32_t gSlotStamp[GB_MAX_SLOTS];
static int      gSlots = 0;
static uint8_t  gBankSlot[512];                 // bank -> slot (0xFF = not cached)
static uint32_t gStamp = 0;

// ---- RTC (MBC3) -------------------------------------------------------------
static uint8_t  gRtc[5], gRtcLat[5];
static uint32_t gRtcLast = 0, gRtcSub = 0;

// ---- CPU / system state -----------------------------------------------------
static uint8_t  gR[8];                          // B C D E H L (unused) A
static uint8_t  gF;
static uint16_t gPC, gSP;
static bool     gIME, gHalt, gHaltBug;
static uint8_t  gEiDelay, gIF, gIE;
static uint32_t gbCycles, gbLimit, gDivZero, gSerEnd, gbFrameNo;
static bool     gSerOn;
static uint8_t  gTIMA, gTMA, gTAC;
static int32_t  gTimaCnt, gTimaPeriod = 1024;
static bool     gTimerOn;
static uint8_t  gJoySel, gPadBtn, gPadDir, gJoyPrev;

// ---- PPU --------------------------------------------------------------------
static uint8_t  gLCDC, gSTAT, gSCY, gSCX, gLY, gLYC, gBGP, gOBP0, gOBP1, gWY, gWX, gMode, gWinLine;
static int32_t  gPpuClk, gPpuLen;
static bool     gStatLine, gbDraw;
static uint8_t  gPalBg[4], gPalObj[2][4];
static uint16_t gbPal565[4];
static uint8_t  gbColMap[GB_OUT_W], gbRowMap[GB_OUT_H];

// Detect MBC1M multicarts from valid Nintendo-logo/header-checksum pairs
// at the common 16-bank sub-ROM boundaries. This applies to the original
// monochrome Game Boy core; the GBC core is intentionally untouched.
static bool gbDetectMbc1M() {
  static const uint8_t logo[48] = {0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D, 0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99, 0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E};
  if (gMbc != 1 || gRomBanks < 32) return false;
  const uint32_t candidates[] = { 0x10, 0x20, 0x30 };
  for (uint32_t bank : candidates) {
    if (bank >= gRomBanks) continue;
    uint8_t h[0x50];
    if (sdDiagReadAt(gRomFile, bank * 16384UL + 0x100, h, sizeof(h)) != (int)sizeof(h)) continue;
    if (memcmp(h + 4, logo, sizeof(logo)) != 0) continue;
    uint8_t sum = 0;
    for (int i = 0x34; i <= 0x4C; ++i) sum = (uint8_t)(sum - h[i] - 1);
    if (sum == h[0x4D]) return true;
  }
  return false;
}

// ============================================================================
//  ROM bank cache
// ============================================================================
static bool gbLoadBankInto(int slot, uint32_t bank) {
  if (slot < 0 || slot >= gSlots || bank >= gRomBanks || !gSlotData[slot] || !gRomFile) return false;
  uint8_t *d = gSlotData[slot];
  const uint32_t offset = bank * 16384UL;
  size_t n = (size_t)sdDiagReadAt(gRomFile, offset, d, 16384);
  if (n < 16384) {
    // A short read indicates a damaged file or SD error; don't silently run
    // corrupted instructions from a buffer padded with 0xFF.
    memset(d + n, 0xFF, 16384 - n);
    return false;
  }
  return true;
}

// Pointer to a bank's data, loading it from the SD card if needed. 'pin' is a
// bank that must not be evicted (the one mapped in the other window).
static uint8_t *gbGetBank(uint32_t bank, uint32_t pin) {
  int s = gBankSlot[bank];
  if (s != 0xFF) { sdRomDiag.cacheHits++; gSlotStamp[s] = ++gStamp; return gSlotData[s]; }
  sdRomDiag.cacheMisses++;

  int v = -1; uint32_t best = 0xFFFFFFFF;
  for (int i = 0; i < gSlots; i++) {
    if (gSlotBank[i] == 0xFFFF) { v = i; break; }
    if (gSlotBank[i] == 0 || gSlotBank[i] == pin) continue;
    if (gSlotStamp[i] <= best) { best = gSlotStamp[i]; v = i; }
  }
  if (v < 0) v = gSlots - 1;                       // cannot happen with >= 3 slots
  if (gSlotBank[v] != 0xFFFF) gBankSlot[gSlotBank[v]] = 0xFF;
  if (!gbLoadBankInto(v, bank)) {
    memset(gSlotData[v], 0xFF, 16384);
    gRomReadError = true;
  }
  gSlotBank[v] = bank; gBankSlot[bank] = v; gSlotStamp[v] = ++gStamp;
  return gSlotData[v];
}

// ============================================================================
//  Cartridge mapping
// ============================================================================
static void gbMapRom() {
  uint32_t b0 = 0, b1 = 1;
  switch (gMbc) {
    case 1:
      // MBC1M multicarts wire the low bank register to four bits and the
      // upper register to bits 4-5, rather than bits 5-6 on ordinary MBC1.
      b1 = ((uint32_t)gB2 << (gMbc1M ? 4 : 5)) | (gMbc1M ? (gB1 & 0x0F) : gB1);
      if (gMode1) b0 = (uint32_t)gB2 << (gMbc1M ? 4 : 5);
      break;
    case 2: case 3: case 5: b1 = gB1; break;
  }
  b0 %= gRomBanks; b1 %= gRomBanks;
  uint8_t *p1 = gbGetBank(b1, b0);
  uint8_t *p0 = gbGetBank(b0, b1);
  for (int i = 0; i < 4; i++) { gMap[i] = p0 + i * 0x1000; gMap[4 + i] = p1 + i * 0x1000; }
}

static void gbMapRam() {
  gRamPtr = nullptr;
  if (gRamEn && gRam && gMbc != 2) {
    uint32_t bank = (gMbc == 1) ? (gMode1 ? gB2 : 0) : gRamBank;
    if (!(gMbc == 3 && bank >= 8)) gRamPtr = gRam + (bank * 0x2000) % gRamAlloc;
  }
  gMap[0xA] = gRamPtr;
  gMap[0xB] = gRamPtr ? gRamPtr + 0x1000 : nullptr;
}

static void gbRtcTick() {
  uint32_t now = millis(), d = now - gRtcLast;
  gRtcLast = now;
  if (gRtc[4] & 0x40) return;                       // halted
  gRtcSub += d;
  while (gRtcSub >= 1000) {
    gRtcSub -= 1000;
    if (++gRtc[0] < 60) continue;
    gRtc[0] = 0;
    if (++gRtc[1] < 60) continue;
    gRtc[1] = 0;
    if (++gRtc[2] < 24) continue;
    gRtc[2] = 0;
    uint16_t day = gRtc[3] | ((gRtc[4] & 1) << 8);
    if (++day > 511) { day = 0; gRtc[4] |= 0x80; }
    gRtc[3] = day & 0xFF; gRtc[4] = (gRtc[4] & 0xFE) | (day >> 8);
  }
}

static void gbMbcWrite(uint16_t a, uint8_t v) {
  switch (gMbc) {
    case 1:
      switch (a >> 13) {
        case 0: gRamEn = ((v & 0x0F) == 0x0A); gbMapRam(); break;
        case 1: gB1 = v & 0x1F; if (!gB1) gB1 = 1; gbMapRom(); break;
        case 2: gB2 = v & 3; gbMapRom(); gbMapRam(); break;
        default: gMode1 = v & 1; gbMapRom(); gbMapRam(); break;
      }
      break;
    case 2:
      if (a < 0x4000) {
        if (a & 0x100) { gB1 = v & 0x0F; if (!gB1) gB1 = 1; gbMapRom(); }
        else gRamEn = ((v & 0x0F) == 0x0A);
      }
      break;
    case 3:
      switch (a >> 13) {
        case 0: gRamEn = ((v & 0x0F) == 0x0A); gbMapRam(); break;
        case 1: gB1 = v & 0x7F; if (!gB1) gB1 = 1; gbMapRom(); break;
        case 2: gRamBank = v; gbMapRam(); break;
        default:
          if (gLatch == 0 && v == 1) { gbRtcTick(); memcpy(gRtcLat, gRtc, 5); }
          gLatch = v;
          break;
      }
      break;
    case 5:
      switch (a >> 12) {
        case 0: case 1: gRamEn = ((v & 0x0F) == 0x0A); gbMapRam(); break;
        case 2: gB1 = (gB1 & 0x100) | v; gbMapRom(); break;
        case 3: gB1 = (gB1 & 0xFF) | ((uint16_t)(v & 1) << 8); gbMapRom(); break;
        case 4: case 5: gRamBank = v & 0x0F; gbMapRam(); break;
      }
      break;
  }
}

static void gbRamWrite(uint16_t a, uint8_t v) {
  if (!gRamEn) return;
  if (gMbc == 2) { gRam[a & 0x1FF] = v & 0x0F; gRamDirty = true; gRamStamp = gbFrameNo; return; }
  if (gMbc == 3 && gRamBank >= 8) {
    if (gRamBank <= 0x0C) {
      gbRtcTick();
      static const uint8_t m[5] = { 0x3F, 0x3F, 0x1F, 0xFF, 0xC1 };
      gRtc[gRamBank - 8] = v & m[gRamBank - 8];
      if (gRamBank == 8) gRtcSub = 0;
    }
    return;
  }
  if (gRamPtr) { gRamPtr[a - 0xA000] = v; gRamDirty = true; gRamStamp = gbFrameNo; }
}

static uint8_t gbRamRead(uint16_t a) {
  if (!gRamEn) return 0xFF;
  if (gMbc == 2) return 0xF0 | (gRam[a & 0x1FF] & 0x0F);
  if (gMbc == 3 && gRamBank >= 8 && gRamBank <= 0x0C) return gRtcLat[gRamBank - 8];
  return 0xFF;
}

// ============================================================================
//  I/O registers
// ============================================================================
static inline void gbPalSet(uint8_t *d, uint8_t v) { for (int i = 0; i < 4; i++) d[i] = (v >> (i * 2)) & 3; }

static void gbStatUpdate() {
  const uint8_t s = gSTAT;
  bool line = false;
  if (gLCDC & 0x80)
    line = ((gMode == 0) && (s & 0x08)) || ((gMode == 1) && (s & 0x10)) ||
           ((gMode == 2) && (s & 0x20)) || ((gLY == gLYC) && (s & 0x40));
  if (line && !gStatLine) gIF |= 2;
  gStatLine = line;
}

static inline void gbSerialCheck() {
  if (gSerOn && (int32_t)(gbCycles - gSerEnd) >= 0) { gSerOn = false; gIo[1] = 0xFF; gIo[2] &= 0x7F; gIF |= 8; }
}

static inline uint8_t gbRd(uint16_t a);
static inline uint8_t gbJoyNibble() {
  uint8_t v = 0x0F;
  if (!(gJoySel & 0x10)) v &= ~gPadDir;
  if (!(gJoySel & 0x20)) v &= ~gPadBtn;
  return v & 0x0F;
}

static IRAM_ATTR uint8_t gbIoRd(uint8_t r) {
  switch (r) {
    case 0x00: return 0xC0 | gJoySel | gbJoyNibble();
    case 0x02: return gIo[2] | 0x7E;
    case 0x04: return (uint8_t)((gbCycles - gDivZero) >> 8);
    case 0x05: return gTIMA;
    case 0x06: return gTMA;
    case 0x07: return gTAC | 0xF8;
    case 0x0F: return gIF | 0xE0;
    case 0x26: return gIo[0x26] | 0x70;
    case 0x40: return gLCDC;
    case 0x41: return 0x80 | (gSTAT & 0x78) | ((gLCDC & 0x80) ? (((gLY == gLYC) ? 4 : 0) | gMode) : 0);
    case 0x42: return gSCY;
    case 0x43: return gSCX;
    case 0x44: return gLY;
    case 0x45: return gLYC;
    case 0x47: return gBGP;
    case 0x48: return gOBP0;
    case 0x49: return gOBP1;
    case 0x4A: return gWY;
    case 0x4B: return gWX;
    default:   return gIo[r];
  }
}

static IRAM_ATTR void gbIoWr(uint8_t r, uint8_t v) {
  switch (r) {
    case 0x50: if (v && gGbBootActive) { gGbBootActive = false; } gIo[r] = v; break;
    case 0x00: gJoySel = v & 0x30; break;
    case 0x02:
      gIo[2] = v;
      if ((v & 0x81) == 0x81) { gSerOn = true; gSerEnd = gbCycles + 4096; }
      break;
    case 0x04: gDivZero = gbCycles; break;
    case 0x05: gTIMA = v; break;
    case 0x06: gTMA = v; break;
    case 0x07: {
      static const int32_t per[4] = { 1024, 16, 64, 256 };
      gTAC = v & 7; gTimerOn = v & 4; gTimaPeriod = per[v & 3];
    } break;
    case 0x0F: gIF = v & 0x1F; break;
    case 0x40: {
      uint8_t old = gLCDC; gLCDC = v;
      if ((old ^ v) & 0x80) {
        gLY = 0; gPpuClk = 0; gWinLine = 0; gStatLine = false;
        if (v & 0x80) { gMode = 2; gPpuLen = 80; gbStatUpdate(); }
        else { gMode = 0; gPpuLen = 0x7FFFFFFF; memset(gFb, 0, 160 * 144); }
      }
    } break;
    case 0x41: gSTAT = v & 0x78; gbStatUpdate(); break;
    case 0x42: gSCY = v; break;
    case 0x43: gSCX = v; break;
    case 0x44: break;                                // LY is read only
    case 0x45: gLYC = v; gbStatUpdate(); break;
    case 0x46: {                                     // OAM DMA (done instantly)
      uint16_t src = (uint16_t)v << 8;
      for (int i = 0; i < 160; i++) gOam[i] = gbRd(src + i);
      gIo[0x46] = v;
    } break;
    case 0x47: gBGP  = v; gbPalSet(gPalBg, v); break;
    case 0x48: gOBP0 = v; gbPalSet(gPalObj[0], v); break;
    case 0x49: gOBP1 = v; gbPalSet(gPalObj[1], v); break;
    case 0x4A: gWY = v; break;
    case 0x4B: gWX = v; break;
    default:   gIo[r] = v; break;
  }
}

// ============================================================================
//  Bus
// ============================================================================
static IRAM_ATTR uint8_t gbRdSlow(uint16_t a) {
  if (a >= 0xF000) {
    if (a < 0xFE00) return gWram[a - 0xE000];        // echo of D000-DDFF
    if (a < 0xFEA0) return gOam[a - 0xFE00];
    if (a < 0xFF00) return 0xFF;
    if (a < 0xFF80) return gbIoRd(a & 0x7F);
    if (a < 0xFFFF) return gHram[a - 0xFF80];
    return gIE;
  }
  return gbRamRead(a);                               // A000-BFFF without a direct mapping
}

static IRAM_ATTR void gbWrSlow(uint16_t a, uint8_t v) {
  if (a < 0x8000) { gbMbcWrite(a, v); return; }
  if (a >= 0xA000 && a < 0xC000) { gbRamWrite(a, v); return; }
  if (a < 0xFE00) { gWram[a - 0xE000] = v; return; }
  if (a < 0xFEA0) { gOam[a - 0xFE00] = v; return; }
  if (a < 0xFF00) return;
  if (a < 0xFF80) { gbIoWr(a & 0x7F, v); return; }
  if (a < 0xFFFF) { gHram[a - 0xFF80] = v; return; }
  gIE = v;
}

static inline uint8_t gbRd(uint16_t a) {
  // DMG boot ROM overlays cartridge addresses 0000-00FF until FF50 is written.
  if (gGbBootActive && a < 0x0100) return gGbBootRom[a];
  uint8_t *p = gMap[a >> 12];
  return p ? p[a & 0xFFF] : gbRdSlow(a);
}
static inline void gbWr(uint16_t a, uint8_t v) {
  uint8_t *p = gWMap[a >> 12];
  if (p) p[a & 0xFFF] = v; else gbWrSlow(a, v);
}

// ============================================================================
//  Timer + PPU
// ============================================================================
static void gbTimerAdvance() {
  while (gTimaCnt >= gTimaPeriod) {
    gTimaCnt -= gTimaPeriod;
    if (++gTIMA == 0) { gTIMA = gTMA; gIF |= 4; }
  }
}

static inline void gbDecode8(uint8_t *o, uint8_t lo, uint8_t hi) {
  o[0] = ((lo >> 7) & 1) | ((hi >> 6) & 2);
  o[1] = ((lo >> 6) & 1) | ((hi >> 5) & 2);
  o[2] = ((lo >> 5) & 1) | ((hi >> 4) & 2);
  o[3] = ((lo >> 4) & 1) | ((hi >> 3) & 2);
  o[4] = ((lo >> 3) & 1) | ((hi >> 2) & 2);
  o[5] = ((lo >> 2) & 1) | ((hi >> 1) & 2);
  o[6] = ((lo >> 1) & 1) | (hi & 2);
  o[7] = (lo & 1) | ((hi << 1) & 2);
}

// Render one visible line into the shade frame buffer (values 0..3).
static IRAM_ATTR void gbRenderLine(int ly, bool win) {
  uint8_t *out = gFb + ly * 160;
  const uint8_t lcdc = gLCDC;
  uint8_t bgl[160];
  static const uint8_t zeroPal[4] = { 0, 0, 0, 0 };
  const uint8_t *bgPal = zeroPal;

  if (lcdc & 1) {
    bgPal = gPalBg;
    uint8_t tmp[176];
    const uint8_t y = (uint8_t)(ly + gSCY);
    const uint8_t *map = gVram + ((lcdc & 8) ? 0x1C00 : 0x1800) + ((y >> 3) << 5);
    const int fy = (y & 7) << 1, tx = gSCX >> 3;
    for (int i = 0; i < 21; i++) {
      uint8_t t = map[(tx + i) & 31];
      const uint8_t *tp = (lcdc & 0x10) ? gVram + t * 16 + fy : gVram + 0x1000 + (int8_t)t * 16 + fy;
      gbDecode8(tmp + i * 8, tp[0], tp[1]);
    }
    memcpy(bgl, tmp + (gSCX & 7), 160);

    if (win) {
      const int wx = (int)gWX - 7;
      const uint8_t *wmap = gVram + ((lcdc & 0x40) ? 0x1C00 : 0x1800) + ((gWinLine >> 3) << 5);
      const int wfy = (gWinLine & 7) << 1;
      for (int i = 0, x = wx; x < 160; i++, x += 8) {
        uint8_t t = wmap[i];
        const uint8_t *tp = (lcdc & 0x10) ? gVram + t * 16 + wfy : gVram + 0x1000 + (int8_t)t * 16 + wfy;
        uint8_t px[8];
        gbDecode8(px, tp[0], tp[1]);
        for (int p = 0; p < 8; p++) { int sx = x + p; if (sx >= 0 && sx < 160) bgl[sx] = px[p]; }
      }
    }
  } else {
    memset(bgl, 0, sizeof(bgl));
  }

  bool anySpr = false;
  uint8_t spr[160];
  if (lcdc & 2) {
    const int h = (lcdc & 4) ? 16 : 8;
    uint16_t key[10]; int n = 0;
    for (int i = 0; i < 40 && n < 10; i++) {
      int y = (int)gOam[i * 4] - 16;
      if (ly >= y && ly < y + h) key[n++] = ((uint16_t)gOam[i * 4 + 1] << 8) | i;
    }
    if (n) {
      for (int i = 1; i < n; i++) {                       // sort: lowest X first, then OAM order
        uint16_t k = key[i]; int j = i - 1;
        while (j >= 0 && key[j] > k) { key[j + 1] = key[j]; j--; }
        key[j + 1] = k;
      }
      memset(spr, 0, sizeof(spr));
      for (int k = n - 1; k >= 0; k--) {                  // draw lowest priority first
        const uint8_t *o = gOam + (key[k] & 0xFF) * 4;
        const int x = (int)o[1] - 8;
        if (x <= -8 || x >= 160) continue;
        const uint8_t attr = o[3];
        int row = ly - ((int)o[0] - 16);
        if (attr & 0x40) row = h - 1 - row;
        uint8_t tile = o[2]; if (h == 16) tile &= 0xFE;
        const uint8_t *tp = gVram + tile * 16 + row * 2;
        const uint8_t lo = tp[0], hi = tp[1];
        const uint8_t flags = ((attr & 0x10) ? 4 : 0) | ((attr & 0x80) ? 8 : 0);
        for (int p = 0; p < 8; p++) {
          int px = x + p;
          if ((unsigned)px >= 160) continue;
          int b = (attr & 0x20) ? p : 7 - p;
          uint8_t c = ((lo >> b) & 1) | (((hi >> b) & 1) << 1);
          if (c) spr[px] = c | flags;
        }
      }
      anySpr = true;
    }
  }

  if (!anySpr) {
    for (int x = 0; x < 160; x++) out[x] = bgPal[bgl[x]];
  } else {
    for (int x = 0; x < 160; x++) {
      uint8_t s = spr[x], b = bgl[x];
      out[x] = (s && (!(s & 8) || b == 0)) ? gPalObj[(s >> 2) & 1][s & 3] : bgPal[b];
    }
  }
}

static IRAM_ATTR void gbPpuEvent() {
  gbSerialCheck();
  while (gPpuClk >= gPpuLen) {
    gPpuClk -= gPpuLen;
    switch (gMode) {
      case 2: gMode = 3; gPpuLen = 172; break;
      case 3: {
        gMode = 0; gPpuLen = 204;
        if (gLY < 144) {
          bool win = (gLCDC & 0x21) == 0x21 && gLY >= gWY && gWX < 167;
          if (gbDraw) gbRenderLine(gLY, win);
          if (win) gWinLine++;
        }
      } break;
      case 0:
        gLY++;
        if (gLY == 144) { gMode = 1; gPpuLen = 456; gIF |= 1; }
        else            { gMode = 2; gPpuLen = 80; }
        break;
      default:
        if (++gLY > 153) { gLY = 0; gMode = 2; gPpuLen = 80; gWinLine = 0; }
        break;
    }
    gbStatUpdate();
  }
}

static inline void gbTick(int c) {
  gbCycles += c;
  if (gTimerOn) { gTimaCnt += c; if (gTimaCnt >= gTimaPeriod) gbTimerAdvance(); }
  gPpuClk += c;
  if (gPpuClk >= gPpuLen) gbPpuEvent();
}

// ============================================================================
//  CPU
// ============================================================================
// base cycles in machine cycles (x4 = clocks). Rows 4-B are filled in by the
// LD r,r / ALU decoder; conditional jumps add their extra cycles when taken.
DRAM_ATTR static const uint8_t gbCycM[256] = {
  1,3,2,2,1,1,2,1,5,2,2,2,1,1,2,1,  1,3,2,2,1,1,2,1,3,2,2,2,1,1,2,1,
  2,3,2,2,1,1,2,1,2,2,2,2,1,1,2,1,  2,3,2,2,3,3,3,1,2,2,2,2,1,1,2,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
  2,3,3,4,3,4,2,4,2,4,3,1,3,6,2,4,  2,3,3,1,3,4,2,4,2,4,3,1,3,1,2,4,
  3,3,2,1,1,4,2,4,4,1,4,1,1,1,2,4,  3,3,2,1,1,4,2,4,3,2,4,1,1,1,2,4
};

static inline void gbAlu(uint8_t op, uint8_t *R, uint8_t &F, uint8_t v) {
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

static inline bool gbCond(uint8_t cc, uint8_t F) {
  switch (cc) {
    case 0:  return !(F & 0x80);
    case 1:  return  (F & 0x80);
    case 2:  return !(F & 0x10);
    default: return  (F & 0x10);
  }
}

#define HLV            ((uint16_t)((R[4] << 8) | R[5]))
#define RR_GET(p)      ((p) == 3 ? sp : (uint16_t)((R[2 * (p)] << 8) | R[2 * (p) + 1]))
#define RR_SET(p, val) do { uint16_t _v = (val); if ((p) == 3) sp = _v; else { R[2 * (p)] = _v >> 8; R[2 * (p) + 1] = _v; } } while (0)
#define PUSH16(val)    do { uint16_t _v = (val); sp--; gbWr(sp, _v >> 8); sp--; gbWr(sp, _v & 0xFF); } while (0)
#define POP16(dst)     do { uint16_t _lo = gbRd(sp++); uint16_t _hi = gbRd(sp++); dst = (_hi << 8) | _lo; } while (0)
#define FETCH16(dst)   do { uint16_t _lo = gbRd(pc++); uint16_t _hi = gbRd(pc++); dst = (_hi << 8) | _lo; } while (0)

// Run instructions until the global cycle counter reaches 'limit'.
static IRAM_ATTR void gbRun(uint32_t limit) {
  uint16_t pc = gPC, sp = gSP;
  uint8_t  F = gF, ei = gEiDelay;
  bool     ime = gIME, halted = gHalt, haltBug = gHaltBug;
  uint8_t *const R = gR;

  while ((int32_t)(limit - gbCycles) > 0) {
    if (ei && --ei == 0) ime = true;

    const uint8_t pend = gIF & gIE & 0x1F;
    if (pend) {
      halted = false;
      if (ime) {                                       // service the interrupt
        int bit = __builtin_ctz(pend);
        gIF &= ~(1 << bit);
        ime = false;
        PUSH16(pc);
        pc = 0x40 + (bit << 3);
        gbTick(20);
        continue;
      }
    }
    if (halted) {                                      // skip ahead to the next event
      int32_t step = gPpuLen - gPpuClk;
      if (gTimerOn) { int32_t t = gTimaPeriod - gTimaCnt; if (t < step) step = t; }
      int32_t left = (int32_t)(limit - gbCycles);
      if (step > left) step = left;
      if (step < 1) step = 1;
      gbTick(step);
      continue;
    }

    uint8_t op = gbRd(pc++);
    if (haltBug) { pc--; haltBug = false; }
    int cyc = gbCycM[op] << 2;

    if ((op & 0xC0) == 0x40) {                         // LD r,r' / HALT
      if (op == 0x76) {
        if (!ime && (gIF & gIE & 0x1F)) haltBug = true; else halted = true;
      } else {
        const uint8_t d = (op >> 3) & 7, s = op & 7;
        uint8_t v;
        if (s == 6) { v = gbRd(HLV); cyc = 8; } else v = R[s];
        if (d == 6) { gbWr(HLV, v); cyc = 8; } else R[d] = v;
      }
    } else if ((op & 0xC0) == 0x80) {                  // ALU A,r
      const uint8_t s = op & 7;
      uint8_t v;
      if (s == 6) { v = gbRd(HLV); cyc = 8; } else v = R[s];
      gbAlu((op >> 3) & 7, R, F, v);
    } else switch (op) {
      case 0x00: break;
      case 0x01: R[1] = gbRd(pc++); R[0] = gbRd(pc++); break;
      case 0x11: R[3] = gbRd(pc++); R[2] = gbRd(pc++); break;
      case 0x21: R[5] = gbRd(pc++); R[4] = gbRd(pc++); break;
      case 0x31: FETCH16(sp); break;

      case 0x02: gbWr((R[0] << 8) | R[1], R[7]); break;
      case 0x12: gbWr((R[2] << 8) | R[3], R[7]); break;
      case 0x22: { uint16_t hl = HLV; gbWr(hl, R[7]); hl++; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x32: { uint16_t hl = HLV; gbWr(hl, R[7]); hl--; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x0A: R[7] = gbRd((R[0] << 8) | R[1]); break;
      case 0x1A: R[7] = gbRd((R[2] << 8) | R[3]); break;
      case 0x2A: { uint16_t hl = HLV; R[7] = gbRd(hl); hl++; R[4] = hl >> 8; R[5] = hl; } break;
      case 0x3A: { uint16_t hl = HLV; R[7] = gbRd(hl); hl--; R[4] = hl >> 8; R[5] = hl; } break;

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
        uint16_t hl = HLV; uint8_t v = gbRd(hl) + 1; gbWr(hl, v);
        F = (F & 0x10) | (v ? 0 : 0x80) | ((v & 0x0F) ? 0 : 0x20);
      } break;
      case 0x35: {
        uint16_t hl = HLV; uint8_t v = gbRd(hl) - 1; gbWr(hl, v);
        F = (F & 0x10) | 0x40 | (v ? 0 : 0x80) | (((v & 0x0F) == 0x0F) ? 0x20 : 0);
      } break;

      case 0x06: case 0x0E: case 0x16: case 0x1E: case 0x26: case 0x2E: case 0x3E:
        R[op >> 3] = gbRd(pc++); break;
      case 0x36: { uint8_t v = gbRd(pc++); gbWr(HLV, v); } break;

      case 0x07: { uint8_t c = R[7] >> 7; R[7] = (R[7] << 1) | c; F = c ? 0x10 : 0; } break;
      case 0x0F: { uint8_t c = R[7] & 1;  R[7] = (R[7] >> 1) | (c << 7); F = c ? 0x10 : 0; } break;
      case 0x17: { uint8_t c = (F >> 4) & 1, n = R[7] >> 7; R[7] = (R[7] << 1) | c; F = n ? 0x10 : 0; } break;
      case 0x1F: { uint8_t c = (F >> 4) & 1, n = R[7] & 1;  R[7] = (R[7] >> 1) | (c << 7); F = n ? 0x10 : 0; } break;

      case 0x08: { uint16_t a; FETCH16(a); gbWr(a, sp & 0xFF); gbWr(a + 1, sp >> 8); } break;

      case 0x09: case 0x19: case 0x29: case 0x39: {
        const uint16_t hl = HLV, v = RR_GET(op >> 4);
        const uint32_t r = (uint32_t)hl + v;
        F = (F & 0x80) | ((((hl & 0xFFF) + (v & 0xFFF)) > 0xFFF) ? 0x20 : 0) | ((r > 0xFFFF) ? 0x10 : 0);
        R[4] = r >> 8; R[5] = r;
      } break;

      case 0x10: pc++; break;                                    // STOP (treated as NOP)
      case 0x18: { int8_t e = (int8_t)gbRd(pc++); pc += e; } break;
      case 0x20: case 0x28: case 0x30: case 0x38: {
        int8_t e = (int8_t)gbRd(pc++);
        if (gbCond((op >> 3) & 3, F)) { pc += e; cyc += 4; }
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
        if (gbCond((op >> 3) & 3, F)) { POP16(pc); cyc += 12; }
        break;
      case 0xC9: POP16(pc); break;
      case 0xD9: POP16(pc); ime = true; ei = 0; break;           // RETI

      case 0xC1: case 0xD1: case 0xE1: { uint16_t v; POP16(v); int p = (op >> 4) & 3; R[2 * p] = v >> 8; R[2 * p + 1] = v; } break;
      case 0xF1: { uint16_t v; POP16(v); R[7] = v >> 8; F = v & 0xF0; } break;
      case 0xC5: case 0xD5: case 0xE5: { int p = (op >> 4) & 3; PUSH16((R[2 * p] << 8) | R[2 * p + 1]); } break;
      case 0xF5: PUSH16((R[7] << 8) | F); break;

      case 0xC2: case 0xCA: case 0xD2: case 0xDA: {
        uint16_t a; FETCH16(a);
        if (gbCond((op >> 3) & 3, F)) { pc = a; cyc += 4; }
      } break;
      case 0xC3: FETCH16(pc); break;
      case 0xE9: pc = HLV; break;

      case 0xC4: case 0xCC: case 0xD4: case 0xDC: {
        uint16_t a; FETCH16(a);
        if (gbCond((op >> 3) & 3, F)) { PUSH16(pc); pc = a; cyc += 12; }
      } break;
      case 0xCD: { uint16_t a; FETCH16(a); PUSH16(pc); pc = a; } break;

      case 0xC6: case 0xCE: case 0xD6: case 0xDE: case 0xE6: case 0xEE: case 0xF6: case 0xFE: {
        uint8_t v = gbRd(pc++);
        gbAlu((op >> 3) & 7, R, F, v);
      } break;
      case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF:
        PUSH16(pc); pc = op & 0x38; break;

      case 0xE0: { uint8_t n = gbRd(pc++); gbWr(0xFF00 | n, R[7]); } break;
      case 0xF0: { uint8_t n = gbRd(pc++); R[7] = gbRd(0xFF00 | n); } break;
      case 0xE2: gbWr(0xFF00 | R[1], R[7]); break;
      case 0xF2: R[7] = gbRd(0xFF00 | R[1]); break;
      case 0xEA: { uint16_t a; FETCH16(a); gbWr(a, R[7]); } break;
      case 0xFA: { uint16_t a; FETCH16(a); R[7] = gbRd(a); } break;

      case 0xE8: case 0xF8: {
        const int8_t e = (int8_t)gbRd(pc++);
        const uint16_t r = sp + e;
        F = ((((sp & 0xF) + (e & 0xF)) > 0xF) ? 0x20 : 0) | ((((sp & 0xFF) + (e & 0xFF)) > 0xFF) ? 0x10 : 0);
        if (op == 0xE8) sp = r; else { R[4] = r >> 8; R[5] = r; }
      } break;
      case 0xF9: sp = HLV; break;

      case 0xF3: ime = false; ei = 0; break;                     // DI
      case 0xFB: ei = 2; break;                                  // EI

      case 0xCB: {
        const uint8_t cb = gbRd(pc++);
        const uint8_t r = cb & 7, grp = cb >> 6, y = (cb >> 3) & 7;
        uint8_t v = (r == 6) ? gbRd(HLV) : R[r];
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
        if (grp != 1) { if (r == 6) gbWr(HLV, v); else R[r] = v; }
      } break;

      default: break;                                          // illegal opcodes: NOP
    }
    gbTick(cyc);
  }
  gPC = pc; gSP = sp; gF = F;
  gIME = ime; gHalt = halted; gHaltBug = haltBug; gEiDelay = ei;
}

// ============================================================================
//  Frame, input, LCD output, saves
// ============================================================================
static void gbJoypadUpdate() {
  const uint8_t s = padState;                       // A B SEL START UP DOWN LEFT RIGHT
  gPadBtn = s & 0x0F;
  gPadDir = ((s >> 7) & 1) | ((s >> 5) & 2) | ((s >> 2) & 0x0C);
  const uint8_t now = gbJoyNibble();
  if ((gJoyPrev & ~now) & 0x0F) gIF |= 0x10;        // a selected button went down
  gJoyPrev = now;
}

static void gbSaveRam() {
  if (!gBattery || !gRam || !gRamDirty || !gSavPath[0]) return;
  // Keep dirty state set on any failure so the next autosave/exit retries.
  if (sdAtomicSaveWrite(gSavPath, gRam, gSavSize)) gRamDirty = false;
}

static IRAM_ATTR void gbPush() {
  SPI.beginTransaction(lcdCfg);
  lcdWindow(GAME_X, GAME_Y, GAME_X + GB_OUT_W - 1, GAME_Y + GB_OUT_H - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  int last = -1;
  for (int r = 0; r < GB_OUT_H; r++) {
    const int s = gbRowMap[r];
    if (s != last) {                                // scaled rows repeat a source line
      const uint8_t *src = gFb + s * 160;
      for (int x = 0; x < GB_OUT_W; x++) outLine[x] = gbPal565[src[gbColMap[x]]];
      last = s;
    }
    SPI.writeBytes((uint8_t *)outLine, GB_OUT_W * 2);
    shotRow(r, outLine, GB_OUT_W);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

static void gbFrame(bool draw) {
  gbDraw = draw;
  gbJoypadUpdate();
  gbLimit += GB_FRAME_CYCLES;
  gbRun(gbLimit);
  if (!(gLCDC & 0x80)) gPpuClk = 0;                 // LCD off: keep the counter small
  gbSerialCheck();
  gbFrameNo++;
  if (draw) gbPush();
  if (gRamDirty && (gbFrameNo - gRamStamp) > 120) gbSaveRam();   // ~2 s after the last write
}

// ============================================================================
//  Load / reset / shutdown
// ============================================================================
static void gbFreeAll() {
  if (gRomFile) gRomFile.close();
  for (int i = 0; i < GB_MAX_SLOTS; i++) { free(gSlotData[i]); gSlotData[i] = nullptr; }
  gSlots = 0;
  free(gRam); gRam = nullptr; gRamPtr = nullptr;
  free(gMem); gMem = gWram = gVram = gFb = nullptr;
  for (int i = 0; i < 16; i++) gMap[i] = gWMap[i] = nullptr;
  gRomReadError = false;
  gRamDirty = false;
  gSavPath[0] = 0;
}

static bool gbFail(const char *l1, const char *l2) {
  strlcpy(loadErr1, l1, sizeof(loadErr1));
  strlcpy(loadErr2, l2, sizeof(loadErr2));
  gbFreeAll();
  return false;
}

static bool gbLoad(const char *path) {
  gbFreeAll();
  gGbBootAvailable = false;
  gGbBootActive = false;
  // Original monochrome Game Boy boot ROM is exactly 256 bytes. The BIOS is
  // optional; without it, the core starts in its existing post-boot state.
  static const char *const biosNames[] = { "dmg_boot.bin", "boot.bin", "gb.bin" };
  char biosPath[96];
  gGbBootAvailable = biosLoadFirst("gb", biosNames, sizeof(biosNames) / sizeof(biosNames[0]),
                                   gGbBootRom, sizeof(gGbBootRom), biosPath, sizeof(biosPath));
  gRomReadError = false;
  gRomFile = SD.open(path, FILE_READ);
  if (!gRomFile || gRomFile.isDirectory()) return gbFail("ROM NOT FOUND", "CHECK SD CARD");
  const size_t sz = gRomFile.size();
  if (sz < 0x8000 || sz > 512UL * 16384 || (sz % 16384) != 0)
    return gbFail("BAD ROM SIZE", "NOT A GB ROM");

  uint8_t h[0x50];                                   // header 0x100-0x14F
  if (sdDiagReadAt(gRomFile, 0x100, h, sizeof(h)) != (int)sizeof(h)) return gbFail("ROM TRUNCATED", "HEADER");
  const uint8_t cgb = h[0x43], type = h[0x47], ramCode = h[0x49];
  if (cgb == 0xC0) return gbFail("GB COLOR ONLY", "NOT SUPPORTED");

  switch (type) {
    case 0x00: case 0x08: case 0x09:                                   gMbc = 0; break;
    case 0x01: case 0x02: case 0x03:                                   gMbc = 1; break;
    case 0x05: case 0x06:                                              gMbc = 2; break;
    case 0x0F: case 0x10: case 0x11: case 0x12: case 0x13:             gMbc = 3; break;
    case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E:  gMbc = 5; break;
    default: { char b[24]; snprintf(b, sizeof(b), "MBC TYPE %02X", type); return gbFail(b, "NOT SUPPORTED"); }
  }
  gBattery = (type == 0x03 || type == 0x06 || type == 0x09 || type == 0x0F || type == 0x10 ||
              type == 0x13 || type == 0x1B || type == 0x1E);
  gHasRtc  = (type == 0x0F || type == 0x10);

  static const uint32_t ramTab[6] = { 0, 2048, 8192, 32768, 131072, 65536 };
  uint32_t ramSize = ramCode < 6 ? ramTab[ramCode] : 0;
  gRamAlloc = (gMbc == 2) ? 512 : (ramSize == 0 ? 0 : (ramSize < 8192 ? 8192 : ramSize));
  gSavSize = gRamAlloc;
  gRomBanks = sz / 16384;
  gMbc1M = gbDetectMbc1M();
  if (gMbc1M) Serial.println("GB: detected MBC1M multicart banking");

  // Reserve the minimum ROM cache FIRST (3 banks: bank 0 + two windows), before the big cart-RAM block can
  // fragment the heap; then allocate cart RAM; then top the cache up with whatever is left.
  const int want = gRomBanks < GB_MAX_SLOTS ? (int)gRomBanks : GB_MAX_SLOTS;
  const int minSlots = gRomBanks < 3 ? (int)gRomBanks : 3;
  gSlots = 0;
  while (gSlots < minSlots) {
    uint8_t *p = (uint8_t *)emuAlloc(16384, 0);
    if (!p) break;
    gSlotData[gSlots++] = p;
  }
  if (gSlots < minSlots) { Serial.printf("ROM CACHE: only %d slots; internal free=%u largest=%u psram=%d\n", gSlots, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), (int)psramFound()); return gbFail("NOT ENOUGH RAM", "ROM CACHE"); }
  gMem = (uint8_t *)emuAlloc(0x4000 + 160 * 144, 0, true);
  if (!gMem) return gbFail("NOT ENOUGH RAM", "GB MEMORY");
  gWram = gMem; gVram = gMem + 0x2000; gFb = gMem + 0x4000;
  if (gRamAlloc) {
    gRam = (uint8_t *)emuAlloc(gRamAlloc, 0, true);
    if (!gRam) { char b[24]; snprintf(b, sizeof(b), "CART RAM %uK", (unsigned)(gRamAlloc / 1024)); return gbFail("NOT ENOUGH RAM", b); }
  }
  while (gSlots < want) {
    uint8_t *p = (uint8_t *)emuAlloc(16384, GB_HEAP_RESERVE);
    if (!p) break;
    gSlotData[gSlots++] = p;
  }
  for (int i = 0; i < GB_MAX_SLOTS; i++) { gSlotBank[i] = 0xFFFF; gSlotStamp[i] = 0; }
  memset(gBankSlot, 0xFF, sizeof(gBankSlot));
  gStamp = 0;

  // The ROM always stays on the SD card: the file remains open and banks are streamed into the small cache.
  const bool all = false;
  gRomFile.seek(0);
  gbGetBank(0, 0xFFFF);                               // bank 0 stays resident forever
  if (gRomReadError) return gbFail("ROM READ ERROR", "CHECK SD CARD");

  // battery RAM
  gSavPath[0] = 0;
  if (gBattery && gRam) {
    if (sdBuildSavePath(path, "gb", gSavPath, sizeof(gSavPath))) {
      if (!sdAtomicSaveLoad(gSavPath, gRam, gSavSize)) {
        char legacyPath[ROM_NAME_LEN + 32];
        bool imported = false;
        // Migrate the former /saves/<system>/<basename>.sav layout only when
        // the ROM is in the system root, where the old name is unambiguous.
        if (sdBuildLegacyManagedSavePath(path, "gb", legacyPath, sizeof(legacyPath)) &&
            strcmp(legacyPath, gSavPath) != 0 &&
            sdAtomicSaveLoad(legacyPath, gRam, gSavSize)) {
          imported = true;
          Serial.printf("GB: migrated managed save; next write goes to %s\n", gSavPath);
        }
        // Older releases also stored saves beside the ROM. This path includes
        // the full ROM path, so it remains safe for same-named games elsewhere.
        if (!imported && sdBuildLegacySavePath(path, legacyPath, sizeof(legacyPath)) &&
            sdAtomicSaveLoad(legacyPath, gRam, gSavSize)) {
          Serial.printf("GB: imported beside-ROM save; next write goes to %s\n", gSavPath);
        }
      }
    } else {
      gSavPath[0] = 0;
      Serial.println("GB: could not construct save path");
    }
  }

  Serial.printf("GB: %uK ROM, mapper %u, RAM %uK%s%s, cache %d/%u banks%s\n",
                (unsigned)(sz / 1024), gMbc, (unsigned)(gRamAlloc / 1024), gBattery ? " +battery" : "",
                gHasRtc ? " +rtc" : "", gSlots, (unsigned)gRomBanks, " (streaming from SD)");
  Serial.printf("Free heap after load: %u bytes\n", (unsigned)ESP.getFreeHeap());
  return true;
}

static void gbReset() {
  memset(gWram, 0, 0x2000); memset(gVram, 0, 0x2000); memset(gFb, 0, 160 * 144);
  memset(gOam, 0, sizeof(gOam)); memset(gHram, 0, sizeof(gHram));
  memset(gIo, 0xFF, sizeof(gIo));
  if (!gGbBootAvailable) {
    static const uint8_t snd[][2] = {                // sound registers after the boot ROM
      {0x10,0x80},{0x11,0xBF},{0x12,0xF3},{0x13,0xFF},{0x14,0xBF},{0x16,0x3F},{0x17,0x00},{0x18,0xFF},
      {0x19,0xBF},{0x1A,0x7F},{0x1B,0xFF},{0x1C,0x9F},{0x1D,0xFF},{0x1E,0xBF},{0x20,0xFF},{0x21,0x00},
      {0x22,0x00},{0x23,0xBF},{0x24,0x77},{0x25,0xF3},{0x26,0xF1} };
    for (unsigned i = 0; i < sizeof(snd) / 2; i++) gIo[snd[i][0]] = snd[i][1];
    gIo[1] = 0; gIo[2] = 0x7E; gIo[0x46] = 0xFF;
  }

  // CPU state: either power-on state for the real boot ROM, or the established
  // post-boot state used when no BIOS was supplied.
  if (gGbBootAvailable) {
    gGbBootActive = true;
    memset(gR, 0, sizeof(gR)); gF = 0; gSP = 0; gPC = 0x0000;
    gIME = gHalt = gHaltBug = false; gEiDelay = 0; gIF = 0; gIE = 0;
    gLCDC = 0; gSTAT = 0; gSCY = gSCX = gLY = gLYC = gWY = gWX = 0; gWinLine = 0;
    gBGP = gOBP0 = gOBP1 = 0; gbPalSet(gPalBg, gBGP); gbPalSet(gPalObj[0], gOBP0); gbPalSet(gPalObj[1], gOBP1);
  } else {
    gGbBootActive = false;
  gR[0] = 0x00; gR[1] = 0x13; gR[2] = 0x00; gR[3] = 0xD8; gR[4] = 0x01; gR[5] = 0x4D; gR[6] = 0; gR[7] = 0x01;
  gF = 0xB0; gSP = 0xFFFE; gPC = 0x0100;
  gIME = gHalt = gHaltBug = false; gEiDelay = 0; gIF = 0x01; gIE = 0;
  }

  gbCycles = gbLimit = gDivZero = 0; gbFrameNo = 0; gSerOn = false;
  gTIMA = gTMA = gTAC = 0; gTimaCnt = 0; gTimaPeriod = 1024; gTimerOn = false;
  gJoySel = 0x30; gPadBtn = gPadDir = 0; gJoyPrev = 0x0F;

  // Only synthesize the post-boot LCD/palette register state when skipping
  // the boot ROM. A real boot ROM expects LCDC and the palettes to start at 0
  // and will initialize them itself; overwriting them here breaks boot-up.
  if (!gGbBootAvailable) {
    gLCDC = 0x91;
    gBGP = 0xFC;
    gOBP0 = gOBP1 = 0xFF;
    gbPalSet(gPalBg, gBGP);
    gbPalSet(gPalObj[0], gOBP0);
    gbPalSet(gPalObj[1], gOBP1);
  }
  gSTAT = 0;
  gSCY = gSCX = gLY = gLYC = gWY = gWX = 0;
  gWinLine = 0;
  gMode = gGbBootAvailable ? 0 : 2;
  gPpuClk = 0;
  gPpuLen = gGbBootAvailable ? 0x7FFFFFFF : 80;
  gStatLine = false;
  gbDraw = true;

  gB1 = 1; gB2 = 0; gMode1 = false; gRamEn = false; gRamBank = 0; gLatch = 0xFF;
  gRamDirty = false; gRamStamp = 0;
  memset(gRtc, 0, sizeof(gRtc)); memset(gRtcLat, 0, sizeof(gRtcLat)); gRtcLast = millis(); gRtcSub = 0;

  for (int i = 0; i < 16; i++) gMap[i] = gWMap[i] = nullptr;
  gMap[8] = gWMap[8] = gVram;          gMap[9] = gWMap[9] = gVram + 0x1000;
  gMap[0xC] = gWMap[0xC] = gWram;      gMap[0xD] = gWMap[0xD] = gWram + 0x1000;
  gMap[0xE] = gWMap[0xE] = gWram;      // echo RAM
  gbMapRom();
  gbMapRam();

  for (int x = 0; x < GB_OUT_W; x++) gbColMap[x] = (x * 160) / GB_OUT_W;
  for (int y = 0; y < GB_OUT_H; y++) gbRowMap[y] = (y * 144) / GB_OUT_H;
  for (int i = 0; i < 4; i++) {
    uint32_t c = gbPalRGB[GB_PALETTE][i];
    gbPal565[i] = col565((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
  }
}

// Versioned save-state payload. ROM cache pointers are deliberately excluded;
// they are rebuilt from the saved mapper registers after restoring memory.
struct GbStateCore {
  uint32_t ramAlloc, savSize, cycles, limit, divZero, serEnd, frameNo, rtcLast, rtcSub, ramStamp, stamp;
  uint16_t b1, pc, sp;
  uint8_t mbc, b2, ramBank, latch, rtc[5], rtcLat[5], r[8], f, eiDelay, iff, ie;
  uint8_t tima, tma, tac, joySel, padBtn, padDir, joyPrev;
  uint8_t lcdc, stat, scy, scx, ly, lyc, bgp, obp0, obp1, wy, wx, mode, winLine;
  int32_t timaCnt, timaPeriod, ppuClk, ppuLen;
  bool battery, hasRtc, ramEn, mode1, ime, halt, haltBug, serOn, timerOn, statLine, draw;
  uint8_t oam[160], hram[127], io[128], palBg[4], palObj[2][4];
};
static bool gbStateSave(uint8_t slot = 0) {
  if (!gMem || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if (!sdBuildStatePath(currentRomPath, "gb", path, sizeof(path), slot)) return false;
  GbStateCore c{};
  c.ramAlloc=gRamAlloc; c.savSize=gSavSize; c.cycles=gbCycles; c.limit=gbLimit; c.divZero=gDivZero; c.serEnd=gSerEnd; c.frameNo=gbFrameNo;
  c.rtcLast=gRtcLast; c.rtcSub=gRtcSub; c.ramStamp=gRamStamp; c.stamp=gStamp; c.b1=gB1; c.pc=gPC; c.sp=gSP;
  c.mbc=gMbc; c.b2=gB2; c.ramBank=gRamBank; c.latch=gLatch; memcpy(c.rtc,gRtc,5); memcpy(c.rtcLat,gRtcLat,5); memcpy(c.r,gR,8); c.f=gF; c.eiDelay=gEiDelay; c.iff=gIF; c.ie=gIE;
  c.tima=gTIMA; c.tma=gTMA; c.tac=gTAC; c.joySel=gJoySel; c.padBtn=gPadBtn; c.padDir=gPadDir; c.joyPrev=gJoyPrev;
  c.lcdc=gLCDC; c.stat=gSTAT; c.scy=gSCY; c.scx=gSCX; c.ly=gLY; c.lyc=gLYC; c.bgp=gBGP; c.obp0=gOBP0; c.obp1=gOBP1; c.wy=gWY; c.wx=gWX; c.mode=gMode; c.winLine=gWinLine;
  c.timaCnt=gTimaCnt; c.timaPeriod=gTimaPeriod; c.ppuClk=gPpuClk; c.ppuLen=gPpuLen;
  c.battery=gBattery; c.hasRtc=gHasRtc; c.ramEn=gRamEn; c.mode1=gMode1; c.ime=gIME; c.halt=gHalt; c.haltBug=gHaltBug; c.serOn=gSerOn; c.timerOn=gTimerOn; c.statLine=gStatLine; c.draw=gbDraw;
  memcpy(c.oam,gOam,sizeof(gOam)); memcpy(c.hram,gHram,sizeof(gHram)); memcpy(c.io,gIo,sizeof(gIo)); memcpy(c.palBg,gPalBg,sizeof(gPalBg)); memcpy(c.palObj,gPalObj,sizeof(gPalObj));
  bool ok=sdStateWrite(path,SYS_GB,&c,sizeof(c),gMem,0x4000+160*144,gRam,gRamAlloc);
  Serial.printf("STATE GB: %s (%s)\n",ok?"saved":"save failed",path); return ok;
}
static bool gbStateLoad(uint8_t slot = 0) {
  if (!gMem || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if (!sdBuildStatePath(currentRomPath,"gb",path,sizeof(path),slot)) return false;
  GbStateCore c{};
  if (!sdStateRead(path,SYS_GB,&c,sizeof(c),gMem,0x4000+160*144,gRam,gRamAlloc) || c.ramAlloc!=gRamAlloc || c.mbc!=gMbc) return false;
  gSavSize=c.savSize; gbCycles=c.cycles; gbLimit=c.limit; gDivZero=c.divZero; gSerEnd=c.serEnd; gbFrameNo=c.frameNo; gRtcLast=millis(); gRtcSub=c.rtcSub; gRamStamp=c.ramStamp; gStamp=c.stamp;
  gB1=c.b1; gPC=c.pc; gSP=c.sp; gB2=c.b2; gRamBank=c.ramBank; gLatch=c.latch; memcpy(gRtc,c.rtc,5); memcpy(gRtcLat,c.rtcLat,5); memcpy(gR,c.r,8); gF=c.f; gEiDelay=c.eiDelay; gIF=c.iff; gIE=c.ie;
  gTIMA=c.tima; gTMA=c.tma; gTAC=c.tac; gJoySel=c.joySel; gPadBtn=c.padBtn; gPadDir=c.padDir; gJoyPrev=c.joyPrev;
  gLCDC=c.lcdc; gSTAT=c.stat; gSCY=c.scy; gSCX=c.scx; gLY=c.ly; gLYC=c.lyc; gBGP=c.bgp; gOBP0=c.obp0; gOBP1=c.obp1; gWY=c.wy; gWX=c.wx; gMode=c.mode; gWinLine=c.winLine;
  gTimaCnt=c.timaCnt; gTimaPeriod=c.timaPeriod; gPpuClk=c.ppuClk; gPpuLen=c.ppuLen; gRamEn=c.ramEn; gMode1=c.mode1; gIME=c.ime; gHalt=c.halt; gHaltBug=c.haltBug; gSerOn=c.serOn; gTimerOn=c.timerOn; gStatLine=c.statLine; gbDraw=true;
  memcpy(gOam,c.oam,sizeof(gOam)); memcpy(gHram,c.hram,sizeof(gHram)); memcpy(gIo,c.io,sizeof(gIo)); memcpy(gPalBg,c.palBg,sizeof(gPalBg)); memcpy(gPalObj,c.palObj,sizeof(gPalObj));
  gbMapRom(); gbMapRam(); if (gRam) gRamDirty=true; lcdDirty=true; return true;
}

static void gbShutdown() {
  gbSaveRam();
  gbFreeAll();
}
