// ============================================================================
//  NES emulator (2A03 / 2C02), from scratch
//   - MOS 6502 CPU (official opcodes + the common unofficial ones), dot-accurate
//     enough PPU with scanline rendering, sprites, sprite-0 hit, OAM DMA
//   - Mappers: 0 (NROM), 1 (MMC1), 2 (UxROM), 3 (CNROM), 4 (MMC3),
//              7 (AxROM), 66 (GxROM)
//   - PRG/CHR ROM banks are streamed from SD through small fixed RAM caches
//   - Output is 256x224 squeezed to 172x129 (4:3) and pinned to the top of the
//     portrait 172x320 panel (same place as the Game Boy picture)
//   - Not emulated: audio (APU registers are stubbed so games still run)
//
//  Needs from the main sketch (defined above the #include):
//    lcdWindow / lcdCfg / outLine / col565 / padState / loadErr1-2,
//    GAME_W / GAME_H / GAME_X / GAME_Y, LCD_* and PIN_LCD_* macros
//  Entry points:  nesInit()  nesLoad(path)  nesReset()  nesFrame(draw)  nesShutdown()
// ============================================================================
// ============================================================================
//  Cartridge / memory
// ============================================================================
static uint8_t  ram[0x800];
static uint8_t  prgram[0x2000];
// 4 KB nametable RAM is required for cartridges with four-screen mirroring.
static uint8_t  vram[0x1000];
static uint8_t  oam[256];
static uint8_t  palRam[32];

// ROM images stay on SD. Only the currently mapped PRG/CHR banks occupy RAM.
#define NES_PRG_CACHE_BANKS 4
#define NES_CHR_CACHE_BANKS 8
static File     nesRomFile;
static uint32_t nesPrgOffset = 16, nesChrOffset = 16;
// Allocated only while an NES game is loaded (40 KB that GB/GBC/2600 can use instead).
static uint8_t (*nesPrgCache)[0x2000] = nullptr;   // [NES_PRG_CACHE_BANKS][0x2000]
static uint8_t (*nesChrCache)[0x400]  = nullptr;   // [NES_CHR_CACHE_BANKS][0x400]
// Track the bank currently stored in each mapper slot. Mapper register writes
// often re-select the same bank; avoiding redundant SD seeks/reads removes a
// major source of frame-time spikes on streaming cartridges.
static uint32_t nesPrgCachedBank[NES_PRG_CACHE_BANKS];
static uint32_t nesChrCachedBank[NES_CHR_CACHE_BANKS];
#define NES_CACHE_INVALID 0xFFFFFFFFUL
static uint8_t *chr = nullptr;                 // only allocated for CHR RAM
static bool     chrRam = false;
static bool     nesRomReadError = false;
static uint32_t prg8n = 2, chr1n = 8;
static uint8_t  mapper = 0;
static bool     fourScreen = false;

static uint8_t *prgBank[4];
static uint8_t *chrBank[8];
static uint8_t *ntBase[4];

static bool irqLine = false;

static uint8_t  colMap[GAME_W];    // output column -> NES pixel column
static uint16_t nesPal565[64];   // byte-swapped RGB565, ready for the wire
static uint16_t palCol[32];

// Pre-expanded 2bpp bitplane lookup: removes repeated shifts/masks from the
// hottest background/sprite pixel loops. Initialized once by nesInit().
static uint8_t nesBitLut[256][8];

static const uint32_t nesRGB[64] = {
  0x666666,0x002A88,0x1412A7,0x3B00A4,0x5C007E,0x6E0040,0x6C0600,0x561D00,
  0x333500,0x0B4800,0x005200,0x004F08,0x00404D,0x000000,0x000000,0x000000,
  0xADADAD,0x155FD9,0x4240FF,0x7527FE,0xA01ACC,0xB71E7B,0xB53120,0x994E00,
  0x6B6D00,0x388700,0x0C9300,0x008F32,0x007C8D,0x000000,0x000000,0x000000,
  0xFFFEFF,0x64B0FF,0x9290FF,0xC676FF,0xF36AFF,0xFE6ECC,0xFE8170,0xEA9E22,
  0xBCBE00,0x88D800,0x5CE430,0x45E082,0x48CDDE,0x4F4F4F,0x000000,0x000000,
  0xFFFEFF,0xC0DFFF,0xD3D2FF,0xE8C8FF,0xFBC2FF,0xFEC4EA,0xFECCC5,0xF7D8A5,
  0xE4E594,0xCFEF96,0xBDF4AB,0xB3F3CC,0xB5EBF2,0xB8B8B8,0x000000,0x000000
};

// ============================================================================
//  Mappers
// ============================================================================
static void setMirror(uint8_t m) {   // 0 horiz, 1 vert, 2 single-lo, 3 single-hi
  // Four-screen cartridges have four physically distinct 1 KB nametables;
  // mapper mirroring writes must not override this hardwired arrangement.
  if (fourScreen) {
    for (int i = 0; i < 4; ++i) ntBase[i] = vram + i * 0x400;
    return;
  }
  uint8_t *a = vram, *b = vram + 0x400;
  switch (m) {
    case 0: ntBase[0] = a; ntBase[1] = a; ntBase[2] = b; ntBase[3] = b; break;
    case 1: ntBase[0] = a; ntBase[1] = b; ntBase[2] = a; ntBase[3] = b; break;
    case 2: ntBase[0] = a; ntBase[1] = a; ntBase[2] = a; ntBase[3] = a; break;
    default: ntBase[0] = b; ntBase[1] = b; ntBase[2] = b; ntBase[3] = b; break;
  }
}
static bool nesReadAt(uint32_t offset, uint8_t *dst, size_t len) {
  if (sdDiagReadAt(nesRomFile, offset, dst, len) != (int)len) {
    nesRomReadError = true;
    memset(dst, 0xFF, len);
    return false;
  }
  return true;
}
static void setPrg8(int slot, uint32_t b) {
  b %= prg8n;
  prgBank[slot] = nesPrgCache[slot];
  if (nesPrgCachedBank[slot] == b) { sdRomDiag.cacheHits++; return; }
  sdRomDiag.cacheMisses++;
  if (nesReadAt(nesPrgOffset + b * 0x2000UL, nesPrgCache[slot], 0x2000))
    nesPrgCachedBank[slot] = b;
  else
    nesPrgCachedBank[slot] = NES_CACHE_INVALID;
}
static void setPrg16(int slot, uint32_t b) { setPrg8(slot, b * 2); setPrg8(slot + 1, b * 2 + 1); }
static void setPrg32(uint32_t b)           { for (int i = 0; i < 4; i++) setPrg8(i, b * 4 + i); }
static void setChr1(int slot, uint32_t b) {
  b %= chr1n;
  if (chrRam) chrBank[slot] = chr + b * 0x400;
  else {
    chrBank[slot] = nesChrCache[slot];
    if (nesChrCachedBank[slot] == b) { sdRomDiag.cacheHits++; return; }
    sdRomDiag.cacheMisses++;
    if (nesReadAt(nesChrOffset + b * 0x400UL, nesChrCache[slot], 0x400))
      nesChrCachedBank[slot] = b;
    else
      nesChrCachedBank[slot] = NES_CACHE_INVALID;
  }
}
static void setChr4(int slot, uint32_t b)  { for (int i = 0; i < 4; i++) setChr1(slot + i, b * 4 + i); }
static void setChr8(uint32_t b)            { for (int i = 0; i < 8; i++) setChr1(i, b * 8 + i); }

// --- MMC1 ---
static uint8_t m1sr = 0x10, m1[4] = { 0x0C, 0, 0, 0 };
static void mmc1Update() {
  uint8_t c = m1[0];
  switch (c & 3) { case 0: setMirror(2); break; case 1: setMirror(3); break;
                   case 2: setMirror(1); break; default: setMirror(0); break; }
  uint8_t pm = (c >> 2) & 3, p = m1[3] & 0x0F;
  if (pm < 2)       setPrg32(p >> 1);
  else if (pm == 2) { setPrg16(0, 0); setPrg16(2, p); }
  else              { setPrg16(0, p); setPrg16(2, prg8n / 2 - 1); }
  if (c & 0x10) { setChr4(0, m1[1]); setChr4(4, m1[2]); }
  else          setChr8(m1[1] >> 1);
}

// --- MMC3 ---
static uint8_t m3sel = 0, m3r[8] = { 0, 2, 4, 5, 6, 7, 0, 1 }, m3latch = 0, m3counter = 0;
static bool    m3reload = false, m3irqEn = false;
static void mmc3Update() {
  if (!(m3sel & 0x80)) {
    setChr1(0, m3r[0] & ~1); setChr1(1, m3r[0] | 1);
    setChr1(2, m3r[1] & ~1); setChr1(3, m3r[1] | 1);
    for (int i = 0; i < 4; i++) setChr1(4 + i, m3r[2 + i]);
  } else {
    for (int i = 0; i < 4; i++) setChr1(i, m3r[2 + i]);
    setChr1(4, m3r[0] & ~1); setChr1(5, m3r[0] | 1);
    setChr1(6, m3r[1] & ~1); setChr1(7, m3r[1] | 1);
  }
  uint32_t last = prg8n - 1, sl = prg8n - 2;
  if (!(m3sel & 0x40)) { setPrg8(0, m3r[6]); setPrg8(1, m3r[7]); setPrg8(2, sl); setPrg8(3, last); }
  else                 { setPrg8(0, sl); setPrg8(1, m3r[7]); setPrg8(2, m3r[6]); setPrg8(3, last); }
}
static void mmc3Scan() {
  if (m3counter == 0 || m3reload) { m3counter = m3latch; m3reload = false; }
  else m3counter--;
  if (m3counter == 0 && m3irqEn) irqLine = true;
}

static void mapperInit() {
  switch (mapper) {
    case 0: case 3: for (int i = 0; i < 4; i++) setPrg8(i, i); setChr8(0); break;
    case 1: m1sr = 0x10; m1[0] = 0x0C; m1[1] = m1[2] = m1[3] = 0; mmc1Update(); break;
    case 2: setPrg16(0, 0); setPrg16(2, prg8n / 2 - 1); setChr8(0); break;
    case 4: m3sel = 0; m3latch = 0; m3counter = 0; m3reload = false; m3irqEn = false;
            { uint8_t d[8] = { 0, 2, 4, 5, 6, 7, 0, 1 }; memcpy(m3r, d, 8); }
            mmc3Update(); break;
    case 7: setPrg32(0); setChr8(0); setMirror(2); break;
    case 66: setPrg32(0); setChr8(0); break;
  }
}

static void mapperWrite(uint16_t a, uint8_t v) {
  switch (mapper) {
    case 1:
      if (v & 0x80) { m1sr = 0x10; m1[0] |= 0x0C; mmc1Update(); }
      else {
        bool done = m1sr & 1;
        m1sr = (m1sr >> 1) | ((v & 1) << 4);
        if (done) { m1[(a >> 13) & 3] = m1sr; m1sr = 0x10; mmc1Update(); }
      }
      break;
    case 2: setPrg16(0, v); break;
    case 3: setChr8(v); break;
    case 4:
      switch (a & 0xE001) {
        case 0x8000: m3sel = v; mmc3Update(); break;
        case 0x8001: m3r[m3sel & 7] = v; mmc3Update(); break;
        case 0xA000: if (!fourScreen) setMirror((v & 1) ? 0 : 1); break;
        case 0xC000: m3latch = v; break;
        case 0xC001: m3counter = 0; m3reload = true; break;
        case 0xE000: m3irqEn = false; irqLine = false; break;
        case 0xE001: m3irqEn = true; break;
      }
      break;
    case 7:  setPrg32(v & 7); setMirror((v & 0x10) ? 3 : 2); break;
    case 66: setPrg32((v >> 4) & 3); setChr8(v & 3); break;
  }
}

// ============================================================================
//  PPU
// ============================================================================
static uint8_t  ppuCtrl, ppuMask, oamAddr, ioBus, readBuf, fineX;
static bool     vblFlag, spr0Flag, ovfFlag, wLatch, nmiPending;
static uint16_t vAddr, tAddr;

static uint8_t  bgBuf[272];
static uint8_t  sprLine[256];
static uint8_t  emitLine[240];

static void updatePalCols() {
  uint8_t m = (ppuMask & 1) ? 0x30 : 0x3F;
  for (int i = 0; i < 32; i++) palCol[i] = nesPal565[palRam[i] & m];
}

static inline uint8_t ppuRd(uint16_t a) {
  a &= 0x3FFF;
  if (a < 0x2000) return chrBank[a >> 10][a & 0x3FF];
  if (a < 0x3F00) return ntBase[(a >> 10) & 3][a & 0x3FF];
  uint8_t i = a & 0x1F; if ((i & 3) == 0) i &= 0x0F;
  return palRam[i];
}
static inline void ppuWr(uint16_t a, uint8_t v) {
  a &= 0x3FFF;
  if (a < 0x2000) { if (chrRam) chrBank[a >> 10][a & 0x3FF] = v; }
  else if (a < 0x3F00) ntBase[(a >> 10) & 3][a & 0x3FF] = v;
  else {
    uint8_t i = a & 0x1F; if ((i & 3) == 0) i &= 0x0F;
    palRam[i] = v & 0x3F;
    palCol[i] = nesPal565[v & ((ppuMask & 1) ? 0x30 : 0x3F)];
  }
}

static uint8_t ppuReadReg(uint8_t r) {
  switch (r) {
    case 2: {
      uint8_t v = (vblFlag ? 0x80 : 0) | (spr0Flag ? 0x40 : 0) | (ovfFlag ? 0x20 : 0) | (ioBus & 0x1F);
      vblFlag = false; wLatch = false;
      return v;
    }
    case 4: return oam[oamAddr];
    case 7: {
      uint16_t a = vAddr & 0x3FFF; uint8_t r2;
      if (a >= 0x3F00) { r2 = ppuRd(a); readBuf = ntBase[(a >> 10) & 3][a & 0x3FF]; }
      else { r2 = readBuf; readBuf = ppuRd(a); }
      vAddr = (vAddr + ((ppuCtrl & 4) ? 32 : 1)) & 0x7FFF;
      return r2;
    }
  }
  return ioBus;
}

static void ppuWriteReg(uint8_t r, uint8_t v) {
  ioBus = v;
  switch (r) {
    case 0:
      if ((v & 0x80) && !(ppuCtrl & 0x80) && vblFlag) nmiPending = true;
      ppuCtrl = v;
      tAddr = (tAddr & 0xF3FF) | ((v & 3) << 10);
      break;
    case 1: {
      // The palette lookup only depends on the greyscale bit. Games commonly
      // rewrite PPUMASK during rendering; avoid rebuilding all 32 RGB565
      // entries when only the rendering-enable/left-edge bits change.
      const bool grayChanged = ((ppuMask ^ v) & 0x01) != 0;
      ppuMask = v;
      if (grayChanged) updatePalCols();
    } break;
    case 3: oamAddr = v; break;
    case 4: oam[oamAddr++] = v; break;
    case 5:
      if (!wLatch) { tAddr = (tAddr & 0xFFE0) | (v >> 3); fineX = v & 7; wLatch = true; }
      else { tAddr = (tAddr & 0x8C1F) | ((v & 7) << 12) | ((v & 0xF8) << 2); wLatch = false; }
      break;
    case 6:
      if (!wLatch) { tAddr = (tAddr & 0x00FF) | ((v & 0x3F) << 8); wLatch = true; }
      else { tAddr = (tAddr & 0xFF00) | v; vAddr = tAddr; wLatch = false; }
      break;
    case 7:
      ppuWr(vAddr, v);
      vAddr = (vAddr + ((ppuCtrl & 4) ? 32 : 1)) & 0x7FFF;
      break;
  }
}

// Render one visible scanline (and push it to the LCD if it is one we show).
static IRAM_ATTR void renderLine(int line, bool draw) {
  const bool bgOn = ppuMask & 0x08, sprOn = ppuMask & 0x10;
  const bool doOut = draw && emitLine[line];

  if (!bgOn && !sprOn) {
    if (doOut) {
      uint16_t c = palCol[0];
      for (int x = 0; x < GAME_W; x++) outLine[x] = c;
      SPI.writeBytes((uint8_t *)outLine, GAME_W * 2);
      shotRowNext(outLine, GAME_W);
    }
    return;
  }

  if (bgOn) {
    uint16_t vv = vAddr;
    const uint16_t base = (ppuCtrl & 0x10) ? 0x1000 : 0;
    const uint8_t fy = (vv >> 12) & 7;
    uint8_t *o = bgBuf;
    for (int tile = 0; tile < 33; tile++) {
      const uint8_t *nt = ntBase[(vv >> 10) & 3];
      uint8_t ti = nt[vv & 0x3FF];
      uint8_t at = nt[0x3C0 + ((vv >> 4) & 0x38) + ((vv >> 2) & 7)];
      uint8_t pb = ((at >> (((vv >> 4) & 4) | (vv & 2))) & 3) << 2;
      uint16_t pa = base + ti * 16 + fy;
      const uint8_t *cp = chrBank[pa >> 10] + (pa & 0x3FF);
      uint8_t lo = cp[0], hi = cp[8];
      for (int px = 0; px < 8; px++) {
        uint8_t c = nesBitLut[lo][px] | (nesBitLut[hi][px] << 1);
        o[px] = c ? (pb | c) : 0;
      }
      o += 8;
      if ((vv & 31) == 31) { vv &= ~31; vv ^= 0x400; } else vv++;
    }
  } else {
    memset(bgBuf, 0, sizeof(bgBuf));
  }
  uint8_t *bg = bgBuf + (bgOn ? fineX : 0);
  if (!(ppuMask & 2)) memset(bg, 0, 8);

  if (sprOn) {
    memset(sprLine, 0, 256);
    const int h = (ppuCtrl & 0x20) ? 16 : 8;
    uint8_t idx[8]; int n = 0;
    for (int i = 0; i < 64; i++) {
      int row = line - oam[i * 4] - 1;
      if (row >= 0 && row < h) {
        if (n < 8) idx[n++] = i; else { ovfFlag = true; break; }
      }
    }
    for (int k = n - 1; k >= 0; k--) {
      int i = idx[k];
      const uint8_t *s = &oam[i * 4];
      int row = line - s[0] - 1;
      uint8_t attr = s[2];
      if (attr & 0x80) row = h - 1 - row;
      uint16_t pa;
      if (h == 8) pa = ((ppuCtrl & 8) ? 0x1000 : 0) + s[1] * 16 + row;
      else        pa = ((s[1] & 1) ? 0x1000 : 0) + (s[1] & 0xFE) * 16 + (row & 7) + ((row & 8) ? 16 : 0);
      const uint8_t *cp = chrBank[pa >> 10] + (pa & 0x3FF);
      uint8_t lo = cp[0], hi = cp[8];
      uint8_t pb = ((attr & 3) << 2) | ((attr & 0x20) ? 0x10 : 0);
      for (int px = 0; px < 8; px++) {
        int sx = s[3] + px;
        if (sx > 255) break;
        uint8_t bitX = (attr & 0x40) ? (uint8_t)(7 - px) : (uint8_t)px;
        uint8_t c = nesBitLut[lo][bitX] | (nesBitLut[hi][bitX] << 1);
        if (!c) continue;
        if (sx < 8 && !(ppuMask & 4)) continue;
        sprLine[sx] = c | pb;
        if (i == 0 && !spr0Flag && (bg[sx] & 3) && sx != 255) spr0Flag = true;
      }
    }
  }

  if (doOut) {
    for (int x = 0; x < GAME_W; x++) {
      const int sx = colMap[x];
      uint8_t b = bg[sx], s = sprOn ? sprLine[sx] : 0, pi;
      if (s && (!(s & 0x10) || !(b & 3))) pi = 0x10 | (s & 0x0F);
      else pi = (b & 3) ? (b & 0x0F) : 0;
      outLine[x] = palCol[pi];
    }
    SPI.writeBytes((uint8_t *)outLine, GAME_W * 2);
    shotRowNext(outLine, GAME_W);
  }

  // end-of-line scroll register updates (only while rendering)
  if ((vAddr & 0x7000) != 0x7000) vAddr += 0x1000;
  else {
    vAddr &= ~0x7000;
    uint16_t y = (vAddr & 0x3E0) >> 5;
    if (y == 29)      { y = 0; vAddr ^= 0x800; }
    else if (y == 31) y = 0;
    else              y++;
    vAddr = (vAddr & ~0x3E0) | (y << 5);
  }
  vAddr = (vAddr & ~0x041F) | (tAddr & 0x041F);
}

// ============================================================================
//  Bus
// ============================================================================
static uint8_t padShift;                 // padState itself lives in the main sketch
static bool    padStrobe;
static int     dmaStall;

static uint8_t rd(uint16_t a);

static uint8_t ioRead(uint16_t a) {
  if (a == 0x4016) {
    uint8_t r = (padShift & 1) | 0x40;
    if (!padStrobe) padShift = (padShift >> 1) | 0x80;
    return r;
  }
  if (a == 0x4017) return 0x40;
  return 0;
}
static void ioWrite(uint16_t a, uint8_t v) {
  if (a == 0x4014) {
    uint16_t base = v << 8;
    if (v < 0x20) { for (int i = 0; i < 256; i++) oam[(uint8_t)(oamAddr + i)] = ram[(base + i) & 0x7FF]; }
    else          { for (int i = 0; i < 256; i++) oam[(uint8_t)(oamAddr + i)] = rd(base + i); }
    dmaStall += 513;
  } else if (a == 0x4016) {
    padStrobe = v & 1;
    if (padStrobe) padShift = padState;
  }
  // APU registers ($4000-$4013, $4015, $4017) are ignored: no audio.
}

static inline uint8_t rd(uint16_t a) {
  if (a >= 0x8000) return prgBank[(a >> 13) & 3][a & 0x1FFF];
  if (a < 0x2000)  return ram[a & 0x7FF];
  if (a < 0x4000)  return ppuReadReg(a & 7);
  if (a >= 0x6000) return prgram[a - 0x6000];
  return ioRead(a);
}
static inline void wr(uint16_t a, uint8_t v) {
  if (a < 0x2000)       ram[a & 0x7FF] = v;
  else if (a < 0x4000)  ppuWriteReg(a & 7, v);
  else if (a >= 0x8000) mapperWrite(a, v);
  else if (a >= 0x6000) prgram[a - 0x6000] = v;
  else                  ioWrite(a, v);
}
static inline uint16_t rd16(uint16_t a) { return rd(a) | (rd(a + 1) << 8); }

// ============================================================================
//  6502 CPU
// ============================================================================
static uint16_t pc;
static uint8_t  ra, rx, ry, sp, ps;
static int32_t  cpuDots;
static int      xcyc;

DRAM_ATTR static const uint8_t cycTab[256] = {
  7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4, 2,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,
  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4, 2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6, 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7
};

static inline void push(uint8_t v) { ram[0x100 + sp] = v; sp--; }
static inline uint8_t pull()       { sp++; return ram[0x100 + sp]; }
static inline void nz(uint8_t v)   { ps = (ps & 0x7D) | (v & 0x80) | (v ? 0 : 2); }

static inline uint8_t doAsl(uint8_t v) { ps = (ps & ~1) | (v >> 7); v <<= 1; nz(v); return v; }
static inline uint8_t doLsr(uint8_t v) { ps = (ps & ~1) | (v & 1); v >>= 1; nz(v); return v; }
static inline uint8_t doRol(uint8_t v) { uint8_t c = ps & 1; ps = (ps & ~1) | (v >> 7); v = (v << 1) | c; nz(v); return v; }
static inline uint8_t doRor(uint8_t v) { uint8_t c = ps & 1; ps = (ps & ~1) | (v & 1); v = (v >> 1) | (c << 7); nz(v); return v; }
static inline void doAdc(uint8_t m) {
  uint16_t s = ra + m + (ps & 1);
  uint8_t r = s;
  uint8_t ov = (~(ra ^ m) & (ra ^ r) & 0x80) ? 0x40 : 0;
  ps = (ps & ~0x41) | (s >> 8) | ov;
  ra = r; nz(ra);
}
static inline void doCmp(uint8_t r, uint8_t m) { uint8_t t = r - m; ps = (ps & ~1) | (r >= m ? 1 : 0); nz(t); }

static inline int doInt(uint16_t vec) {
  push(pc >> 8); push(pc & 0xFF);
  push((ps & ~0x10) | 0x20);
  ps |= 4;
  pc = rd16(vec);
  return 7;
}

// effective address for the "group 1" addressing modes (bbb field)
static inline uint16_t am01(uint8_t bbb, bool write) {
  uint16_t b, ea; uint8_t z;
  switch (bbb) {
    case 0: z = rd(pc++) + rx; return rd(z) | (rd((uint8_t)(z + 1)) << 8);
    case 1: return rd(pc++);
    case 2: return pc++;
    case 3: ea = rd16(pc); pc += 2; return ea;
    case 4: z = rd(pc++); b = rd(z) | (rd((uint8_t)(z + 1)) << 8); ea = b + ry;
            if (!write && ((b ^ ea) & 0xFF00)) xcyc++;
            return ea;
    case 5: return (uint8_t)(rd(pc++) + rx);
    case 6: b = rd16(pc); pc += 2; ea = b + ry;
            if (!write && ((b ^ ea) & 0xFF00)) xcyc++;
            return ea;
    default: b = rd16(pc); pc += 2; ea = b + rx;
            if (!write && ((b ^ ea) & 0xFF00)) xcyc++;
            return ea;
  }
}

static IRAM_ATTR int cpuStep() {
  if (nmiPending) { nmiPending = false; return doInt(0xFFFA); }
  if (irqLine && !(ps & 4)) return doInt(0xFFFE);

  uint8_t op = rd(pc++);
  const uint8_t cc = op & 3, bbb = (op >> 2) & 7, aaa = op >> 5;
  uint16_t ea; uint8_t m;
  xcyc = 0;

  if (cc == 1) {
    ea = am01(bbb, aaa == 4);
    if (aaa == 4) { if (bbb != 2) wr(ea, ra); }
    else {
      m = rd(ea);
      switch (aaa) {
        case 0: ra |= m; nz(ra); break;
        case 1: ra &= m; nz(ra); break;
        case 2: ra ^= m; nz(ra); break;
        case 3: doAdc(m); break;
        case 5: ra = m; nz(ra); break;
        case 6: doCmp(ra, m); break;
        case 7: doAdc(m ^ 0xFF); break;
      }
    }
  }
  else if (cc == 2) {
    if (bbb == 2) {
      switch (aaa) {
        case 0: ra = doAsl(ra); break;
        case 1: ra = doRol(ra); break;
        case 2: ra = doLsr(ra); break;
        case 3: ra = doRor(ra); break;
        case 4: ra = rx; nz(ra); break;
        case 5: rx = ra; nz(rx); break;
        case 6: rx--; nz(rx); break;
        default: break;
      }
    } else if (bbb == 6) {
      if (aaa == 4) sp = rx;
      else if (aaa == 5) { rx = sp; nz(rx); }
    } else if (bbb == 4) {
      // KIL opcodes: treat as NOP
    } else if (bbb == 0) {
      if (aaa == 5) { rx = rd(pc++); nz(rx); } else pc++;
    } else {
      const bool useY = (aaa == 4 || aaa == 5);
      switch (bbb) {
        case 1: ea = rd(pc++); break;
        case 3: ea = rd16(pc); pc += 2; break;
        case 5: ea = (uint8_t)(rd(pc++) + (useY ? ry : rx)); break;
        default: {
          uint16_t b = rd16(pc); pc += 2;
          ea = b + (useY ? ry : rx);
          if (aaa == 5 && ((b ^ ea) & 0xFF00)) xcyc++;
        } break;
      }
      if (aaa == 4) { if (bbb != 7) wr(ea, rx); }
      else if (aaa == 5) { rx = rd(ea); nz(rx); }
      else {
        m = rd(ea);
        switch (aaa) {
          case 0: m = doAsl(m); break;
          case 1: m = doRol(m); break;
          case 2: m = doLsr(m); break;
          case 3: m = doRor(m); break;
          case 6: m--; nz(m); break;
          default: m++; nz(m); break;
        }
        wr(ea, m);
      }
    }
  }
  else if (cc == 0) {
    switch (bbb) {
      case 4: {   // branches
        static const uint8_t masks[4] = { 0x80, 0x40, 0x01, 0x02 };
        bool cond = ((ps & masks[aaa >> 1]) != 0) == (aaa & 1);
        int8_t off = (int8_t)rd(pc++);
        if (cond) {
          uint16_t np = pc + off;
          xcyc += 1 + (((pc ^ np) & 0xFF00) ? 1 : 0);
          pc = np;
        }
      } break;
      case 2:
        switch (aaa) {
          case 0: push(ps | 0x30); break;                       // PHP
          case 1: ps = (pull() & 0xEF) | 0x20; break;           // PLP
          case 2: push(ra); break;                              // PHA
          case 3: ra = pull(); nz(ra); break;                   // PLA
          case 4: ry--; nz(ry); break;                          // DEY
          case 5: ry = ra; nz(ry); break;                       // TAY
          case 6: ry++; nz(ry); break;                          // INY
          default: rx++; nz(rx); break;                         // INX
        }
        break;
      case 6:
        switch (aaa) {
          case 0: ps &= ~1; break;      // CLC
          case 1: ps |= 1; break;       // SEC
          case 2: ps &= ~4; break;      // CLI
          case 3: ps |= 4; break;       // SEI
          case 4: ra = ry; nz(ra); break; // TYA
          case 5: ps &= ~0x40; break;   // CLV
          case 6: ps &= ~8; break;      // CLD
          default: ps |= 8; break;      // SED
        }
        break;
      case 0:
        switch (aaa) {
          case 0:   // BRK
            pc++; push(pc >> 8); push(pc & 0xFF); push(ps | 0x30); ps |= 4; pc = rd16(0xFFFE);
            break;
          case 1: { // JSR
            uint16_t t = rd16(pc); uint16_t r = pc + 1;
            push(r >> 8); push(r & 0xFF); pc = t;
          } break;
          case 2: { // RTI
            ps = (pull() & 0xEF) | 0x20;
            uint8_t lo = pull(), hi = pull(); pc = lo | (hi << 8);
          } break;
          case 3: { // RTS
            uint8_t lo = pull(), hi = pull(); pc = (lo | (hi << 8)) + 1;
          } break;
          case 4: pc++; break;                                  // NOP #imm
          case 5: ry = rd(pc++); nz(ry); break;                 // LDY #
          case 6: doCmp(ry, rd(pc++)); break;                   // CPY #
          default: doCmp(rx, rd(pc++)); break;                  // CPX #
        }
        break;
      default: {  // bbb 1,3,5,7
        switch (bbb) {
          case 1: ea = rd(pc++); break;
          case 3: ea = rd16(pc); pc += 2; break;
          case 5: ea = (uint8_t)(rd(pc++) + rx); break;
          default: {
            uint16_t b = rd16(pc); pc += 2; ea = b + rx;
            if (aaa != 4 && ((b ^ ea) & 0xFF00)) xcyc++;
          } break;
        }
        switch (aaa) {
          case 1:  // BIT (zp / abs only)
            if (bbb == 1 || bbb == 3) {
              m = rd(ea);
              ps = (ps & 0x3D) | (m & 0xC0) | ((ra & m) ? 0 : 2);
            }
            break;
          case 2: if (bbb == 3) pc = ea; break;                 // JMP abs
          case 3:                                               // JMP (ind)
            if (bbb == 3) {
              uint8_t lo = rd(ea), hi = rd((ea & 0xFF00) | ((ea + 1) & 0xFF));
              pc = lo | (hi << 8);
            }
            break;
          case 4: if (bbb != 7) wr(ea, ry); break;              // STY
          case 5: ry = rd(ea); nz(ry); break;                   // LDY
          case 6: if (bbb == 1 || bbb == 3) doCmp(ry, rd(ea)); break;  // CPY
          case 7: if (bbb == 1 || bbb == 3) doCmp(rx, rd(ea)); break;  // CPX
          default: break;                                       // unofficial NOPs
        }
      } break;
    }
  }
  else {  // cc == 3 : a handful of unofficial opcodes that some games use
    if (bbb == 2) {
      if (op == 0xEB) doAdc(rd(pc++) ^ 0xFF); else pc++;
    } else {
      if ((aaa == 4 || aaa == 5) && bbb == 5) ea = (uint8_t)(rd(pc++) + ry);
      else if (aaa == 5 && bbb == 7) {
        uint16_t b = rd16(pc); pc += 2; ea = b + ry;
        if ((b ^ ea) & 0xFF00) xcyc++;
      } else ea = am01(bbb, aaa != 5);
      switch (aaa) {
        case 4: if (bbb == 0 || bbb == 1 || bbb == 3 || bbb == 5) wr(ea, ra & rx); break;  // SAX
        case 5: if (bbb != 6) { ra = rx = rd(ea); nz(ra); } break;                           // LAX
        default:
          m = rd(ea);
          switch (aaa) {
            case 0: m = doAsl(m); ra |= m; nz(ra); break;   // SLO
            case 1: m = doRol(m); ra &= m; nz(ra); break;   // RLA
            case 2: m = doLsr(m); ra ^= m; nz(ra); break;   // SRE
            case 3: m = doRor(m); doAdc(m); break;          // RRA
            case 6: m--; doCmp(ra, m); break;               // DCP
            default: m++; doAdc(m ^ 0xFF); break;           // ISB
          }
          wr(ea, m);
          break;
      }
    }
  }

  int c = cycTab[op] + xcyc + dmaStall;
  dmaStall = 0;
  return c;
}

static inline void runCpu(int dots) {
  cpuDots += dots;
  while (cpuDots > 0) cpuDots -= cpuStep() * 3;
}

// ============================================================================
//  Frame
// ============================================================================
static void runFrame(bool draw) {
  for (int line = 0; line < 240; line++) {
    renderLine(line, draw);
    if (mapper == 4 && (ppuMask & 0x18)) mmc3Scan();
    runCpu(341);
  }
  runCpu(341);                                   // post-render line
  vblFlag = true;                                // line 241: vblank starts
  if (ppuCtrl & 0x80) nmiPending = true;
  for (int l = 241; l < 261; l++) runCpu(341);
  vblFlag = false; spr0Flag = false; ovfFlag = false;   // pre-render line
  if (ppuMask & 0x18) {
    vAddr = (vAddr & ~0x7BFF) | (tAddr & 0x7BFF);
    if (mapper == 4) mmc3Scan();
  }
  runCpu(341);
}

static void lcdFrameBegin() {
  shotSeqBegin();
  SPI.beginTransaction(lcdCfg);
  lcdWindow(GAME_X, GAME_Y, GAME_X + GAME_W - 1, GAME_Y + GAME_H - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
}
static void lcdFrameEnd() {
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// ============================================================================
//  Entry points used by the main sketch
// ============================================================================
// One-time setup: NES palette -> RGB565 and the 256 -> GAME_W column map.
static void nesInit() {
  for (int value = 0; value < 256; ++value) {
    for (int px = 0; px < 8; ++px)
      nesBitLut[value][px] = (uint8_t)((value >> (7 - px)) & 1);
  }
  for (int i = 0; i < 64; i++)
    nesPal565[i] = col565((nesRGB[i] >> 16) & 0xFF, (nesRGB[i] >> 8) & 0xFF, nesRGB[i] & 0xFF);
  for (int x = 0; x < GAME_W; x++) colMap[x] = (x * 256 + 128) / GAME_W;
}

// Release the cartridge (call when leaving a game).
static void nesShutdown() {
  if (nesRomFile) nesRomFile.close();
  for (int i = 0; i < NES_PRG_CACHE_BANKS; ++i) nesPrgCachedBank[i] = NES_CACHE_INVALID;
  for (int i = 0; i < NES_CHR_CACHE_BANKS; ++i) nesChrCachedBank[i] = NES_CACHE_INVALID;
  free(chr); chr = nullptr;
  free(nesPrgCache); nesPrgCache = nullptr;
  free(nesChrCache); nesChrCache = nullptr;
  nesRomReadError = false;
}

// Record an error, release anything half-loaded and return false so the
// caller can go back to the menu instead of hanging.
static bool nesFail(File &f, const char *l1, const char *l2) {
  strlcpy(loadErr1, l1, sizeof(loadErr1));
  strlcpy(loadErr2, l2, sizeof(loadErr2));
  if (f) f.close();
  nesShutdown();
  return false;
}

static bool nesLoad(const char *path) {
  nesShutdown();
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) return nesFail(f, "ROM NOT FOUND", "CHECK SD CARD");
  nesPrgCache = (uint8_t (*)[0x2000])emuAlloc((size_t)NES_PRG_CACHE_BANKS * 0x2000, 0, true);
  nesChrCache = (uint8_t (*)[0x400])emuAlloc((size_t)NES_CHR_CACHE_BANKS * 0x400, 0, true);
  if (!nesPrgCache || !nesChrCache) return nesFail(f, "NOT ENOUGH RAM", "NES CACHE");

  const uint32_t fileSize = f.size();
  uint8_t h[16];
  if (fileSize < sizeof(h) || sdDiagReadAt(f, 0, h, sizeof(h)) != (int)sizeof(h) ||
      memcmp(h, "NES\x1A", 4) != 0) return nesFail(f, "NOT A NES ROM", "BAD HEADER");

  // Accept legacy iNES and NES 2.0 ROM-size fields.
  const bool nes20 = (h[7] & 0x0C) == 0x08;
  const uint8_t f6 = h[6], f7 = h[7];
  mapper = (f6 >> 4) | (f7 & 0xF0);
  if (nes20) mapper |= (uint16_t)(h[8] & 0x0F) << 8;
  fourScreen = f6 & 8;

  uint64_t prgSize64, chrSize64;
  if (nes20) {
    auto decodeRomSize = [](uint8_t lsb, uint8_t msNibble, uint32_t unit) -> uint64_t {
      if (msNibble != 0x0F)
        return (uint64_t)(((uint16_t)msNibble << 8) | lsb) * unit;
      const uint8_t exponent = lsb >> 2;
      const uint8_t multiplier = (lsb & 3) * 2 + 1;
      // NES 2.0 exponent/multiplier encoding can overflow even when the
      // exponent itself is below 63 (for example, 2^62 * 3).
      if (exponent >= 64) return UINT64_MAX;
      const uint64_t base = 1ULL << exponent;
      if (base > UINT64_MAX / multiplier) return UINT64_MAX;
      return base * multiplier;
    };
    prgSize64 = decodeRomSize(h[4], h[9] & 0x0F, 16384);
    chrSize64 = decodeRomSize(h[5], h[9] >> 4, 8192);
  } else {
    prgSize64 = (uint64_t)h[4] * 16384ULL;
    chrSize64 = (uint64_t)h[5] * 8192ULL;
  }
  if (prgSize64 == 0) return nesFail(f, "NOT A NES ROM", "NO PRG DATA");
  if (prgSize64 > UINT32_MAX || chrSize64 > UINT32_MAX)
    return nesFail(f, "ROM TOO LARGE", "SIZE NOT SUPPORTED");

  switch (mapper) {
    case 0: case 1: case 2: case 3: case 4: case 7: case 66: break;
    default: { char b[24]; snprintf(b, sizeof(b), "MAPPER %u", mapper); return nesFail(f, b, "NOT SUPPORTED"); }
  }
  const uint32_t trainerSize = (f6 & 4) ? 512U : 0U;
  const uint32_t prgSize = (uint32_t)prgSize64;
  const uint32_t chrSize = (uint32_t)chrSize64;
  if ((uint64_t)16 + trainerSize + prgSize + chrSize > fileSize)
    return nesFail(f, "ROM TRUNCATED", "SIZE HEADER MISMATCH");

  nesPrgOffset = 16 + trainerSize;
  nesChrOffset = nesPrgOffset + prgSize;
  prg8n = prgSize / 0x2000UL;
  chr1n = chrSize ? chrSize / 0x400UL : 8;
  chrRam = (chrSize == 0);
  if (chrRam) {
    chr = (uint8_t *)emuAlloc(8192, 0, true);
    if (!chr) return nesFail(f, "NOT ENOUGH RAM", "CHR RAM");
  }
  // Re-open as the persistent streaming handle. Do not keep a full PRG/CHR copy.
  f.close();
  nesRomFile = SD_MMC.open(path, FILE_READ);
  if (!nesRomFile || nesRomFile.isDirectory()) return nesFail(nesRomFile, "ROM OPEN ERROR", "CHECK SD CARD");
  nesRomReadError = false;

  Serial.printf("ROM: %s\n", path);
  Serial.printf("ROM: PRG %uK, CHR %s %uK, mapper %u, mirroring %s (streaming)\n",
                (unsigned)(prgSize / 1024), chrRam ? "RAM" : "ROM", (unsigned)(chr1n),
                mapper, fourScreen ? "4-screen" : ((f6 & 1) ? "vertical" : "horizontal"));
  Serial.printf("Free heap after load: %u bytes\n", (unsigned)ESP.getFreeHeap());

  setMirror((f6 & 1) ? 1 : 0);
  mapperInit();
  if (nesRomReadError) return nesFail(nesRomFile, "ROM READ ERROR", "CHECK SD CARD");
  return true;
}

// Reset the NES to its power-on state (the ROM is already loaded).
static void nesReset() {
  // which NES scanlines are shown: lines 8..231 are squeezed into GAME_H rows
  memset(emitLine, 0, sizeof(emitLine));
  for (int r = 0; r < GAME_H; r++) {
    int s = 8 + (r * 224 + GAME_H / 2) / GAME_H;
    if (s > 231) s = 231;
    emitLine[s] = 1;
  }

  // clean machine state (matters when coming back from a previous game)
  memset(ram, 0, sizeof(ram));
  memset(prgram, 0, sizeof(prgram));
  memset(vram, 0, sizeof(vram));
  memset(oam, 0, sizeof(oam));
  memset(palRam, 0, sizeof(palRam));
  irqLine = false; dmaStall = 0; padStrobe = false; padShift = 0;
  ioBus = readBuf = 0;

  // power-on state
  ppuCtrl = ppuMask = oamAddr = fineX = 0;
  vblFlag = spr0Flag = ovfFlag = wLatch = nmiPending = false;
  vAddr = tAddr = 0;
  updatePalCols();

  sp = 0xFD; ps = 0x24; ra = rx = ry = 0;
  pc = rd16(0xFFFC);
  cpuDots = 0;
}

// Portable-by-layout snapshot of mutable NES machine state. ROM caches and pointers
// are excluded; bank windows are reconstructed from saved cache-bank indices.
struct NesStateCore {
  uint32_t prgBanks[4], chrBanks[8];
  uint8_t mapper, mirror, m1sr, m1[4], m3sel, m3r[8], m3latch, m3counter;
  bool m3reload, m3irqEn, irqLine, chrRam, fourScreen;
  uint8_t ram[0x800], prgram[0x2000], vram[0x1000], oam[256], palRam[32];
  uint8_t ppuCtrl, ppuMask, oamAddr, ioBus, readBuf, fineX;
  bool vblFlag, spr0Flag, ovfFlag, wLatch, nmiPending;
  uint16_t vAddr, tAddr;
  uint8_t bgBuf[272], sprLine[256], emitLine[240];
  uint16_t pc; uint8_t ra,rx,ry,sp,ps; int32_t cpuDots; int xcyc;
  uint8_t padShift; bool padStrobe; int dmaStall;
};
static bool nesStateSave(uint8_t slot = 0) {
  if (!nesRomFile || !currentRomPath[0] || !sdMount()) return false;
  char path[ROM_PATH_LEN + 16]; if (!sdBuildStatePath(currentRomPath,"nes",path,sizeof(path),slot)) return false;
  NesStateCore c{};
  for(int i=0;i<4;i++) c.prgBanks[i]=nesPrgCachedBank[i];
  for(int i=0;i<8;i++) c.chrBanks[i]=chrRam ? i : nesChrCachedBank[i];
  c.mapper=mapper;
  if(ntBase[0]==vram && ntBase[1]==vram && ntBase[2]==vram && ntBase[3]==vram) c.mirror=2;
  else if(ntBase[0]==vram+0x400 && ntBase[1]==vram+0x400 && ntBase[2]==vram+0x400 && ntBase[3]==vram+0x400) c.mirror=3;
  else if(ntBase[0]==vram && ntBase[1]==vram+0x400 && ntBase[2]==vram && ntBase[3]==vram+0x400) c.mirror=1;
  else c.mirror=0;
  c.m1sr=m1sr; memcpy(c.m1,m1,4); c.m3sel=m3sel; memcpy(c.m3r,m3r,8); c.m3latch=m3latch; c.m3counter=m3counter;
  c.m3reload=m3reload; c.m3irqEn=m3irqEn; c.irqLine=irqLine; c.chrRam=chrRam; c.fourScreen=fourScreen;
  memcpy(c.ram,ram,sizeof(ram)); memcpy(c.prgram,prgram,sizeof(prgram)); memcpy(c.vram,vram,sizeof(vram)); memcpy(c.oam,oam,sizeof(oam)); memcpy(c.palRam,palRam,sizeof(palRam));
  c.ppuCtrl=ppuCtrl;c.ppuMask=ppuMask;c.oamAddr=oamAddr;c.ioBus=ioBus;c.readBuf=readBuf;c.fineX=fineX;
  c.vblFlag=vblFlag;c.spr0Flag=spr0Flag;c.ovfFlag=ovfFlag;c.wLatch=wLatch;c.nmiPending=nmiPending;c.vAddr=vAddr;c.tAddr=tAddr;
  memcpy(c.bgBuf,bgBuf,sizeof(bgBuf));memcpy(c.sprLine,sprLine,sizeof(sprLine));memcpy(c.emitLine,emitLine,sizeof(emitLine));
  c.pc=pc;c.ra=ra;c.rx=rx;c.ry=ry;c.sp=sp;c.ps=ps;c.cpuDots=cpuDots;c.xcyc=xcyc;c.padShift=padShift;c.padStrobe=padStrobe;c.dmaStall=dmaStall;
  bool ok=sdStateWrite(path,SYS_NES,&c,sizeof(c),chrRam?chr:nullptr,chrRam?8192:0,nullptr,0);
  Serial.printf("STATE NES: %s (%s)\n",ok?"saved":"save failed",path); return ok;
}
static bool nesStateLoad(uint8_t slot = 0) {
  if (!nesRomFile || !currentRomPath[0] || !sdMount()) return false;
  char path[ROM_PATH_LEN + 16]; if (!sdBuildStatePath(currentRomPath,"nes",path,sizeof(path),slot)) return false;
  NesStateCore c{};
  if(!sdStateRead(path,SYS_NES,&c,sizeof(c),chrRam?chr:nullptr,chrRam?8192:0,nullptr,0) || c.mapper!=mapper || c.chrRam!=chrRam) return false;
  memcpy(ram,c.ram,sizeof(ram));memcpy(prgram,c.prgram,sizeof(prgram));memcpy(vram,c.vram,sizeof(vram));memcpy(oam,c.oam,sizeof(oam));memcpy(palRam,c.palRam,sizeof(palRam));
  m1sr=c.m1sr;memcpy(m1,c.m1,4);m3sel=c.m3sel;memcpy(m3r,c.m3r,8);m3latch=c.m3latch;m3counter=c.m3counter;m3reload=c.m3reload;m3irqEn=c.m3irqEn;irqLine=c.irqLine;fourScreen=c.fourScreen;
  // Force cache reloads so mapper windows point at the saved banks, not the
  // banks left by the game immediately before loading this state.
  for(int i=0;i<4;i++) nesPrgCachedBank[i]=NES_CACHE_INVALID;
  for(int i=0;i<8;i++) nesChrCachedBank[i]=NES_CACHE_INVALID;
  for(int i=0;i<4;i++) setPrg8(i,c.prgBanks[i]);
  if(chrRam){ for(int i=0;i<8;i++) chrBank[i]=chr+i*0x400; }
  else for(int i=0;i<8;i++) setChr1(i,c.chrBanks[i]);
  setMirror(c.mirror);
  ppuCtrl=c.ppuCtrl;ppuMask=c.ppuMask;oamAddr=c.oamAddr;ioBus=c.ioBus;readBuf=c.readBuf;fineX=c.fineX;
  vblFlag=c.vblFlag;spr0Flag=c.spr0Flag;ovfFlag=c.ovfFlag;wLatch=c.wLatch;nmiPending=c.nmiPending;vAddr=c.vAddr;tAddr=c.tAddr;
  memcpy(bgBuf,c.bgBuf,sizeof(bgBuf));memcpy(sprLine,c.sprLine,sizeof(sprLine));memcpy(emitLine,c.emitLine,sizeof(emitLine));
  pc=c.pc;ra=c.ra;rx=c.rx;ry=c.ry;sp=c.sp;ps=c.ps;cpuDots=c.cpuDots;xcyc=c.xcyc;padShift=c.padShift;padStrobe=c.padStrobe;dmaStall=c.dmaStall;
  updatePalCols(); lcdDirty=true; return !nesRomReadError;
}

// Run one NES frame; the LCD transaction is only opened when the frame is drawn.
static void nesFrame(bool draw) {
  if (draw) lcdFrameBegin();
  runFrame(draw);
  if (draw) lcdFrameEnd();
}
