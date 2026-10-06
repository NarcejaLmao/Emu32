// =====================================================================================
//  nes_core.h  -  NES emulation core for Emu32
// =====================================================================================
//  Part of the Emu32 sketch: keep it in the "src" folder next to emu32.ino.  It is #included by emu32.ino
//  and pulls in emu_common.h (same folder), which provides:
//    C(r,g,b), SW, SH, OUT_W, OUT_X0, rowSlot() / rowDone() / flushRows() (LCD row batching),
//    padBits (controller), sdMount() / sdUnmount(), romBuf, romError, romErrBuf.
//  Contents: iNES loader, mappers 0/1/2/3/4/7/66, PPU, 6502 CPU, one-frame runner.
//  Public API:  loadRom(path)  nesInit()  nesReset()  emuFrame(draw)   and the FRAME_US constant.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <SD.h>
#include "emu_common.h"

#define NES_TOP 8                      // first NES scanline shown (the top/bottom 8 lines are overscan)
#define NES_LINES 224
#define FRAME_US 16639                 // 60.1 Hz

// =====================================================================================
//  Cartridge (iNES) + mappers
// =====================================================================================
enum { MIR_H, MIR_V, MIR_1LO, MIR_1HI };

static uint8_t *prgRom = nullptr, *chrRom = nullptr;
static uint32_t prgSize = 0, chrSize = 0;
static bool chrIsRam = false;
static uint8_t chrRam[8192];
static uint8_t prgRam[8192];
static int mapperId = 0;
static int headerMirror = MIR_H;

static uint8_t *prgBank[4];   // 8 KB pieces at $8000, $A000, $C000, $E000
static uint8_t *chrBank[8];   // 1 KB pieces at $0000-$1FFF of the PPU
static uint8_t vram[2048];    // two nametables
static uint8_t *ntPtr[4];

static inline void setMirror(int m) {
  static const uint8_t map[4][4] = {{0, 0, 1, 1}, {0, 1, 0, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}};
  for (int i = 0; i < 4; i++) ntPtr[i] = vram + map[m][i] * 1024;
}
static inline void setPrg8(int slot, uint32_t b) { uint32_t n = prgSize / 8192; prgBank[slot] = prgRom + (b % n) * 8192UL; }
static inline void setPrg16(int slot16, uint32_t b) { setPrg8(slot16 * 2, b * 2); setPrg8(slot16 * 2 + 1, b * 2 + 1); }
static inline void setPrg32(uint32_t b) { for (int i = 0; i < 4; i++) setPrg8(i, b * 4 + i); }
static inline void setChr1(int slot, uint32_t b) {
  uint8_t *base = chrIsRam ? chrRam : chrRom;
  uint32_t n = (chrIsRam ? 8192 : chrSize) / 1024;
  chrBank[slot] = base + (b % n) * 1024UL;
}
static inline void setChr8(uint32_t b) { for (int i = 0; i < 8; i++) setChr1(i, b * 8 + i); }
static inline void setChr4(int slot4, uint32_t b) { for (int i = 0; i < 4; i++) setChr1(slot4 * 4 + i, b * 4 + i); }

// MMC1
static uint8_t mm1shift = 0x10, mm1cnt = 0, mm1ctrl = 0x0C, mm1chr0 = 0, mm1chr1 = 0, mm1prg = 0;
static void mmc1Update() {
  static const uint8_t mm[4] = {MIR_1LO, MIR_1HI, MIR_V, MIR_H};
  setMirror(mm[mm1ctrl & 3]);
  int pm = (mm1ctrl >> 2) & 3;
  if (pm <= 1)      setPrg32((mm1prg & 0x0F) >> 1);
  else if (pm == 2) { setPrg16(0, 0); setPrg16(1, mm1prg & 0x0F); }
  else              { setPrg16(0, mm1prg & 0x0F); setPrg16(1, prgSize / 16384 - 1); }
  if (mm1ctrl & 0x10) { setChr4(0, mm1chr0); setChr4(1, mm1chr1); }
  else                setChr8(mm1chr0 >> 1);
}
// MMC3
static uint8_t mm3sel = 0, mm3reg[8] = {0, 2, 4, 5, 6, 7, 0, 1}, mm3latch = 0, mm3counter = 0;
static bool mm3reload = false, mm3irqEn = false;
static volatile bool irqLine = false;
static void mmc3Update() {
  uint32_t n8 = prgSize / 8192;
  uint32_t r6 = mm3reg[6] & 0x3F, r7 = mm3reg[7] & 0x3F;
  if (!(mm3sel & 0x40)) { setPrg8(0, r6); setPrg8(1, r7); setPrg8(2, n8 - 2); setPrg8(3, n8 - 1); }
  else                  { setPrg8(0, n8 - 2); setPrg8(1, r7); setPrg8(2, r6); setPrg8(3, n8 - 1); }
  int a = (mm3sel & 0x80) ? 4 : 0, b = (mm3sel & 0x80) ? 0 : 4;   // where the 2 KB banks / 1 KB banks go
  setChr1(a + 0, mm3reg[0] & ~1); setChr1(a + 1, mm3reg[0] | 1);
  setChr1(a + 2, mm3reg[1] & ~1); setChr1(a + 3, mm3reg[1] | 1);
  setChr1(b + 0, mm3reg[2]); setChr1(b + 1, mm3reg[3]); setChr1(b + 2, mm3reg[4]); setChr1(b + 3, mm3reg[5]);
}
static void mmc3Clock() {
  if (mm3counter == 0 || mm3reload) { mm3counter = mm3latch; mm3reload = false; }
  else mm3counter--;
  if (mm3counter == 0 && mm3irqEn) irqLine = true;
}

static void mapperWrite(uint16_t a, uint8_t v) {
  switch (mapperId) {
    case 1:
      if (v & 0x80) { mm1shift = 0x10; mm1cnt = 0; mm1ctrl |= 0x0C; mmc1Update(); return; }
      mm1shift = (mm1shift >> 1) | ((v & 1) << 4);
      if (++mm1cnt == 5) {
        switch ((a >> 13) & 3) { case 0: mm1ctrl = mm1shift; break; case 1: mm1chr0 = mm1shift; break;
                                 case 2: mm1chr1 = mm1shift; break; default: mm1prg = mm1shift; break; }
        mm1shift = 0x10; mm1cnt = 0; mmc1Update();
      }
      break;
    case 2: setPrg16(0, v & 0x0F); break;
    case 3: setChr8(v & 3); break;
    case 4:
      switch (a & 0xE001) {
        case 0x8000: mm3sel = v; mmc3Update(); break;
        case 0x8001: mm3reg[mm3sel & 7] = v; mmc3Update(); break;
        case 0xA000: setMirror((v & 1) ? MIR_H : MIR_V); break;
        case 0xC000: mm3latch = v; break;
        case 0xC001: mm3reload = true; mm3counter = 0; break;
        case 0xE000: mm3irqEn = false; irqLine = false; break;
        case 0xE001: mm3irqEn = true; break;
      }
      break;
    case 7: setPrg32(v & 7); setMirror((v & 0x10) ? MIR_1HI : MIR_1LO); break;
    case 66: setPrg32((v >> 4) & 3); setChr8(v & 3); break;
  }
}

static void cartInit() {
  setMirror(headerMirror);
  setChr8(0);
  switch (mapperId) {
    case 1: mmc1Update(); break;
    case 2: setPrg16(0, 0); setPrg16(1, prgSize / 16384 - 1); break;
    case 4: mmc3Update(); break;
    case 7: setPrg32(0); setMirror(MIR_1LO); break;
    default: setPrg32(0); break;
  }
}


// Reads the ROM at `path` from the SD card into RAM and parses the iNES header.
static bool loadRom(const char *path) {
  if (!sdMount()) { romError = "SD card not found"; return false; }
  File f = SD.open(path, FILE_READ);
  if (!f) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t sz = f.size();
  if (sz < 16 + 8192) { romError = "ROM file is too small"; f.close(); sdUnmount(); return false; }
  romBuf = (uint8_t *)malloc(sz);
  if (!romBuf) {
    snprintf(romErrBuf, sizeof(romErrBuf), "ROM too big (%u KB)", (unsigned)(sz / 1024));
    romError = romErrBuf; f.close(); sdUnmount(); return false;
  }
  size_t got = 0;
  while (got < sz) {
    int n = f.read(romBuf + got, min((size_t)4096, sz - got));
    if (n <= 0) break;
    got += n;
  }
  f.close(); sdUnmount();
  if (got != sz) { romError = "Could not read ROM"; return false; }

  const uint8_t *h = romBuf;
  if (h[0] != 'N' || h[1] != 'E' || h[2] != 'S' || h[3] != 0x1A) { romError = "Not an iNES ROM file"; return false; }
  uint32_t prg16 = h[4], chr8 = h[5];
  uint8_t f6 = h[6], f7 = h[7];
  mapperId = (f6 >> 4) | (f7 & 0xF0);
  uint32_t off = 16 + ((f6 & 4) ? 512 : 0);
  prgSize = prg16 * 16384UL; chrSize = chr8 * 8192UL;
  if (prgSize == 0 || off + prgSize + chrSize > sz) { romError = "ROM file is truncated"; return false; }
  prgRom = romBuf + off; chrRom = prgRom + prgSize;
  chrIsRam = (chrSize == 0);
  headerMirror = (f6 & 1) ? MIR_V : MIR_H;
  if (!(mapperId == 0 || mapperId == 1 || mapperId == 2 || mapperId == 3 || mapperId == 4 || mapperId == 7 || mapperId == 66)) {
    snprintf(romErrBuf, sizeof(romErrBuf), "Mapper %d not supported", mapperId);
    romError = romErrBuf; return false;
  }
  Serial.printf("ROM: mapper %d, PRG %u KB, CHR %u KB%s\n", mapperId, (unsigned)(prgSize / 1024), (unsigned)(chrSize / 1024), chrIsRam ? " (RAM)" : "");
  return true;
}

// =====================================================================================
//  Controller input (filled by the Bluetooth code)
// =====================================================================================
static uint8_t ctrlShift = 0;
static bool ctrlStrobe = false;

// =====================================================================================
//  PPU
// =====================================================================================
static uint8_t ppuCtrl = 0, ppuMask = 0, ppuStatus = 0, oamAddr = 0, fineX = 0, wLatch = 0, readBuf = 0;
static uint16_t ppuV = 0, ppuT = 0;
static uint8_t oam[256];
static uint8_t pal[32];
static uint16_t palRgb[32];
static uint16_t nesRgb[64];
static bool nmiPending = false;

static const uint8_t NES_PAL[64][3] = {
  {84,84,84},{0,30,116},{8,16,144},{48,0,136},{68,0,100},{92,0,48},{84,4,0},{60,24,0},{32,42,0},{8,58,0},{0,64,0},{0,60,0},{0,50,60},{0,0,0},{0,0,0},{0,0,0},
  {152,150,152},{8,76,196},{48,50,236},{92,30,228},{136,20,176},{160,20,100},{152,34,32},{120,60,0},{84,90,0},{40,114,0},{8,124,0},{0,118,40},{0,102,120},{0,0,0},{0,0,0},{0,0,0},
  {236,238,236},{76,154,236},{120,124,236},{176,98,236},{228,84,236},{236,88,180},{236,106,100},{212,136,32},{160,170,0},{116,196,0},{76,208,32},{56,204,108},{56,180,204},{60,60,60},{0,0,0},{0,0,0},
  {236,238,236},{168,204,236},{188,188,236},{212,178,236},{236,174,236},{236,174,212},{236,180,176},{228,196,144},{204,210,120},{180,222,120},{168,226,144},{152,226,180},{160,214,228},{160,162,160},{0,0,0},{0,0,0}
};

static inline int palIndex(uint16_t a) {
  int i = a & 0x1F;
  if (i >= 0x10 && (i & 3) == 0) i -= 0x10;
  return i;
}

static inline uint8_t ppuRead(uint16_t a) {
  a &= 0x3FFF;
  if (a < 0x2000) return chrBank[a >> 10][a & 0x3FF];
  if (a < 0x3F00) return ntPtr[(a >> 10) & 3][a & 0x3FF];
  return pal[palIndex(a)];
}

static inline void ppuWrite(uint16_t a, uint8_t v) {
  a &= 0x3FFF;
  if (a < 0x2000) { if (chrIsRam) chrBank[a >> 10][a & 0x3FF] = v; }
  else if (a < 0x3F00) ntPtr[(a >> 10) & 3][a & 0x3FF] = v;
  else { int i = palIndex(a); pal[i] = v & 0x3F; palRgb[i] = nesRgb[v & 0x3F]; }
}

static uint8_t ppuRegRead(int r) {
  switch (r) {
    case 2: { uint8_t s = (ppuStatus & 0xE0) | (readBuf & 0x1F); ppuStatus &= ~0x80; wLatch = 0; return s; }
    case 4: return oam[oamAddr];
    case 7: {
      uint16_t a = ppuV & 0x3FFF; uint8_t r2;
      if (a >= 0x3F00) { r2 = pal[palIndex(a)]; readBuf = ppuRead(a - 0x1000); }
      else { r2 = readBuf; readBuf = ppuRead(a); }
      ppuV += (ppuCtrl & 4) ? 32 : 1;
      return r2;
    }
  }
  return 0;
}

static void ppuRegWrite(int r, uint8_t v) {
  switch (r) {
    case 0: {
      bool was = ppuCtrl & 0x80;
      ppuCtrl = v;
      ppuT = (ppuT & 0xF3FF) | ((v & 3) << 10);
      if (!was && (v & 0x80) && (ppuStatus & 0x80)) nmiPending = true;
      break;
    }
    case 1: ppuMask = v; break;
    case 3: oamAddr = v; break;
    case 4: oam[oamAddr++] = v; break;
    case 5:
      if (!wLatch) { fineX = v & 7; ppuT = (ppuT & ~0x001F) | (v >> 3); wLatch = 1; }
      else { ppuT = (ppuT & ~0x73E0) | ((v & 7) << 12) | ((v >> 3) << 5); wLatch = 0; }
      break;
    case 6:
      if (!wLatch) { ppuT = (ppuT & 0x00FF) | ((v & 0x3F) << 8); wLatch = 1; }
      else { ppuT = (ppuT & 0xFF00) | v; ppuV = ppuT; wLatch = 0; }
      break;
    case 7:
      ppuWrite(ppuV, v);
      ppuV += (ppuCtrl & 4) ? 32 : 1;
      break;
  }
}

static uint8_t bgLine[272];
static uint8_t spLine[256], spPri[256], spZero[256];
static uint8_t xmap[OUT_W];
static uint16_t spread16[256];       // bit i of a byte -> bit 2i (decodes 8 pixels of a tile row at once)
static int lastOutRow = -1;


// Sprite 0 hit test for one line (the flag only needs the overlap, not the picture)
static void sprite0Hit(bool bgOn, bool bgLeft, bool spLeft, int fx) {
  if (!bgOn) return;
  for (int x = 0; x < 255; x++) {
    if (!spLine[x] || !spZero[x]) continue;
    if (x < 8 && !(bgLeft && spLeft)) continue;
    if (bgLine[x + fx]) { ppuStatus |= 0x40; break; }
  }
}

// Renders NES scanline `sl`. If `want`, the line is also scaled and sent to the LCD at row `outRow`.
// Even when nothing is drawn, this runs for lines where sprite 0 may hit (games rely on that flag).
static void renderLine(int sl, bool want, int outRow) {
  bool bgOn = ppuMask & 0x08, spOn = ppuMask & 0x10;
  int fx = fineX;

  if (bgOn) {
    uint16_t va = ppuV;
    int fy = (va >> 12) & 7;
    uint16_t bgBase = (ppuCtrl & 0x10) ? 0x1000 : 0;
    uint8_t *out = bgLine;
    for (int tile = 0; tile < 33; tile++) {
      uint8_t *nt = ntPtr[(va >> 10) & 3];
      uint8_t ti = nt[va & 0x3FF];
      uint8_t at = nt[0x3C0 | ((va >> 4) & 0x38) | ((va >> 2) & 7)];
      uint8_t palb = ((at >> (((va >> 4) & 4) | (va & 2))) & 3) << 2;
      uint16_t a = bgBase + ti * 16 + fy;
      const uint8_t *p = chrBank[a >> 10] + (a & 0x3FF);
      uint8_t lo = p[0], hi = p[8];
      if (!(lo | hi)) { memset(out, 0, 8); }               // empty tile row (very common)
      else {
        uint16_t q = spread16[lo] | (spread16[hi] << 1);
#define BGPX(i, sh) { uint8_t px = (q >> (sh)) & 3; out[i] = px ? (palb | px) : 0; }
        BGPX(0, 14) BGPX(1, 12) BGPX(2, 10) BGPX(3, 8) BGPX(4, 6) BGPX(5, 4) BGPX(6, 2) BGPX(7, 0)
#undef BGPX
      }
      out += 8;
      if ((va & 0x1F) == 31) { va &= ~0x1F; va ^= 0x0400; } else va++;
    }
  }

  // sprites on this line
  int h = (ppuCtrl & 0x20) ? 16 : 8;
  int cnt = 0; uint8_t idx[8]; bool hasZero = false;
  if (spOn) {
    for (int i = 0; i < 64; i++) {
      int row = sl - (oam[i * 4] + 1);
      if (row < 0 || row >= h) continue;
      if (cnt < 8) { idx[cnt++] = i; if (i == 0) hasZero = true; } else { ppuStatus |= 0x20; break; }
    }
    if (cnt) {
      memset(spLine, 0, 256);
      for (int k = cnt - 1; k >= 0; k--) {
        int i = idx[k];
        int row = sl - (oam[i * 4] + 1);
        uint8_t tile = oam[i * 4 + 1], attr = oam[i * 4 + 2];
        int sx = oam[i * 4 + 3];
        if (attr & 0x80) row = h - 1 - row;
        uint16_t base; int tIdx;
        if (h == 8) { base = (ppuCtrl & 8) ? 0x1000 : 0; tIdx = tile; }
        else { base = (tile & 1) ? 0x1000 : 0; tIdx = tile & 0xFE; if (row >= 8) { tIdx++; row -= 8; } }
        uint16_t a = base + tIdx * 16 + row;
        const uint8_t *p = chrBank[a >> 10] + (a & 0x3FF);
        uint8_t lo = p[0], hi = p[8];
        if (!(lo | hi)) continue;                                 // fully transparent row
        for (int b = 0; b < 8; b++) {
          int bit = (attr & 0x40) ? b : 7 - b;
          uint8_t px = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
          int x = sx + b;
          if (x > 255) break;
          if (!px) continue;
          spLine[x] = 0x10 | ((attr & 3) << 2) | px;
          spPri[x] = (attr >> 5) & 1;
          spZero[x] = (i == 0);
        }
      }
    }
  }

  bool bgLeft = ppuMask & 2, spLeft = ppuMask & 4;
  if (cnt && hasZero && !(ppuStatus & 0x40)) sprite0Hit(bgOn, bgLeft, spLeft, fx);
  if (want) {
    uint16_t *dst = rowSlot(outRow);
    for (int ox = 0; ox < OUT_W; ox++) {          // only the 197 pixels that are shown get composed
      int x = xmap[ox];
      uint8_t c = (bgOn && (x >= 8 || bgLeft)) ? bgLine[x + fx] : 0;
      if (cnt) {
        uint8_t sp = spLine[x];
        if (sp && (x >= 8 || spLeft)) c = (spPri[x] && c) ? c : sp;
      }
      dst[ox] = palRgb[c];
    }
    rowDone();
  }
}

static void ppuEndLine() {                            // what the PPU does at the end of every rendered line
  if (!(ppuMask & 0x18)) return;
  if ((ppuV & 0x7000) != 0x7000) ppuV += 0x1000;
  else {
    ppuV &= ~0x7000;
    int y = (ppuV & 0x03E0) >> 5;
    if (y == 29) { y = 0; ppuV ^= 0x0800; }
    else if (y == 31) y = 0;
    else y++;
    ppuV = (ppuV & ~0x03E0) | (y << 5);
  }
  ppuV = (ppuV & ~0x041F) | (ppuT & 0x041F);         // copy horizontal scroll bits
}

// =====================================================================================
//  CPU (6502 without decimal mode) + bus
// =====================================================================================
#define FC 0x01
#define FZ 0x02
#define FI 0x04
#define FD 0x08
#define FB 0x10
#define FU 0x20
#define FV 0x40
#define FN 0x80

static uint8_t ram[2048];
static uint8_t cA, cX, cY, cSP, cP;
static uint16_t cPC;
static int dmaCycles = 0;

static uint8_t rdIO(uint16_t a) {
  if (a < 0x4000) return ppuRegRead(a & 7);
  if (a == 0x4016) {
    uint8_t bit = ctrlShift & 1;
    if (ctrlStrobe) ctrlShift = padBits; else ctrlShift = (ctrlShift >> 1) | 0x80;
    return 0x40 | bit;
  }
  if (a == 0x4017) return 0x40;
  if (a >= 0x6000 && a < 0x8000) return prgRam[a - 0x6000];
  return 0;
}
static inline uint8_t rd(uint16_t a) {
  if (a < 0x2000) return ram[a & 0x7FF];
  if (a >= 0x8000) return prgBank[(a >> 13) & 3][a & 0x1FFF];
  return rdIO(a);
}
static inline uint16_t rd16(uint16_t a) { return rd(a) | (rd(a + 1) << 8); }

static void wr(uint16_t a, uint8_t v) {
  if (a < 0x2000) { ram[a & 0x7FF] = v; return; }
  if (a < 0x4000) { ppuRegWrite(a & 7, v); return; }
  if (a < 0x4020) {
    if (a == 0x4014) {                                    // sprite DMA
      uint16_t base = v << 8;
      for (int i = 0; i < 256; i++) oam[(uint8_t)(oamAddr + i)] = rd(base + i);
      dmaCycles += 513;
    } else if (a == 0x4016) {
      ctrlStrobe = v & 1;
      if (ctrlStrobe) ctrlShift = padBits;
    }
    return;
  }
  if (a >= 0x6000 && a < 0x8000) { prgRam[a - 0x6000] = v; return; }
  if (a >= 0x8000) mapperWrite(a, v);
}

static inline void setNZ(uint8_t v) { cP = (cP & ~(FN | FZ)) | (v & 0x80) | (v ? 0 : FZ); }
static inline void push8(uint8_t v) { ram[0x100 + cSP--] = v; }
static inline uint8_t pull8() { return ram[0x100 + (uint8_t)(++cSP)]; }
static inline void push16(uint16_t v) { push8(v >> 8); push8(v & 0xFF); }
static inline uint16_t pull16() { uint8_t lo = pull8(); return lo | (pull8() << 8); }

static inline void doADC(uint8_t m) {
  uint16_t r = cA + m + (cP & FC);
  cP &= ~(FC | FV);
  if (r > 0xFF) cP |= FC;
  if (~(cA ^ m) & (cA ^ r) & 0x80) cP |= FV;
  cA = (uint8_t)r; setNZ(cA);
}
static inline void doCMP(uint8_t reg, uint8_t m) { cP = (cP & ~FC) | (reg >= m ? FC : 0); setNZ((uint8_t)(reg - m)); }
static inline uint8_t opASL(uint8_t v) { cP = (cP & ~FC) | (v >> 7); v <<= 1; setNZ(v); return v; }
static inline uint8_t opLSR(uint8_t v) { cP = (cP & ~FC) | (v & 1); v >>= 1; setNZ(v); return v; }
static inline uint8_t opROL(uint8_t v) { uint8_t c = cP & FC; cP = (cP & ~FC) | (v >> 7); v = (v << 1) | c; setNZ(v); return v; }
static inline uint8_t opROR(uint8_t v) { uint8_t c = cP & FC; cP = (cP & ~FC) | (v & 1); v = (v >> 1) | (c << 7); setNZ(v); return v; }

static inline int doBranch(bool cond) {
  int8_t off = (int8_t)rd(cPC++);
  int c = 2;
  if (cond) { uint16_t np = cPC + off; c += 1 + (((np ^ cPC) & 0xFF00) ? 1 : 0); cPC = np; }
  return c;
}
static int doInterrupt(uint16_t vec, bool brk) {
  push16(cPC);
  push8((cP & ~FB) | FU | (brk ? FB : 0));
  cP |= FI;
  cPC = rd16(vec);
  return 7;
}

enum { M_IMM, M_ZP, M_ZPX, M_ZPY, M_ABS, M_ABX, M_ABY, M_IZX, M_IZY };
static const uint8_t modeBase[9] = {2, 3, 4, 4, 4, 4, 4, 6, 5};

static inline uint16_t getAddr(int m, bool &cross) {
  cross = false;
  switch (m) {
    case M_IMM: return cPC++;
    case M_ZP:  return rd(cPC++);
    case M_ZPX: return (uint8_t)(rd(cPC++) + cX);
    case M_ZPY: return (uint8_t)(rd(cPC++) + cY);
    case M_ABS: { uint16_t a = rd16(cPC); cPC += 2; return a; }
    case M_ABX: { uint16_t b = rd16(cPC); cPC += 2; uint16_t a = b + cX; cross = (b ^ a) & 0xFF00; return a; }
    case M_ABY: { uint16_t b = rd16(cPC); cPC += 2; uint16_t a = b + cY; cross = (b ^ a) & 0xFF00; return a; }
    case M_IZX: { uint8_t p = rd(cPC++) + cX; return rd(p) | (rd((uint8_t)(p + 1)) << 8); }
    default:    { uint8_t p = rd(cPC++); uint16_t b = rd(p) | (rd((uint8_t)(p + 1)) << 8); uint16_t a = b + cY; cross = (b ^ a) & 0xFF00; return a; }
  }
}
static inline int extra1(int m) { return (m == M_ABX || m == M_ABY || m == M_IZY) ? 1 : 0; }   // store / RMW page-cross penalty

static int cpuStep() {
  if (nmiPending) { nmiPending = false; return doInterrupt(0xFFFA, false); }
  if (irqLine && !(cP & FI)) return doInterrupt(0xFFFE, false);

  uint8_t op = rd(cPC++);
  switch (op) {                                         // single-byte / special opcodes first
    case 0x00: cPC++; return doInterrupt(0xFFFE, true);
    case 0x20: { uint16_t a = rd16(cPC); push16(cPC + 1); cPC = a; return 6; }
    case 0x40: cP = (pull8() & ~FB) | FU; cPC = pull16(); return 6;
    case 0x60: cPC = pull16() + 1; return 6;
    case 0x08: push8(cP | FB | FU); return 3;
    case 0x28: cP = (pull8() & ~FB) | FU; return 4;
    case 0x48: push8(cA); return 3;
    case 0x68: cA = pull8(); setNZ(cA); return 4;
    case 0x88: cY--; setNZ(cY); return 2;
    case 0xA8: cY = cA; setNZ(cY); return 2;
    case 0xC8: cY++; setNZ(cY); return 2;
    case 0xE8: cX++; setNZ(cX); return 2;
    case 0x18: cP &= ~FC; return 2;
    case 0x38: cP |= FC; return 2;
    case 0x58: cP &= ~FI; return 2;
    case 0x78: cP |= FI; return 2;
    case 0xB8: cP &= ~FV; return 2;
    case 0xD8: cP &= ~FD; return 2;
    case 0xF8: cP |= FD; return 2;
    case 0x98: cA = cY; setNZ(cA); return 2;
    case 0x8A: cA = cX; setNZ(cA); return 2;
    case 0x9A: cSP = cX; return 2;
    case 0xAA: cX = cA; setNZ(cX); return 2;
    case 0xBA: cX = cSP; setNZ(cX); return 2;
    case 0xCA: cX--; setNZ(cX); return 2;
    case 0xEA: case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: return 2;
    case 0x10: return doBranch(!(cP & FN));
    case 0x30: return doBranch(cP & FN);
    case 0x50: return doBranch(!(cP & FV));
    case 0x70: return doBranch(cP & FV);
    case 0x90: return doBranch(!(cP & FC));
    case 0xB0: return doBranch(cP & FC);
    case 0xD0: return doBranch(!(cP & FZ));
    case 0xF0: return doBranch(cP & FZ);
    case 0x4C: cPC = rd16(cPC); return 3;
    case 0x6C: { uint16_t p = rd16(cPC); cPC = rd(p) | (rd((p & 0xFF00) | ((p + 1) & 0xFF)) << 8); return 5; }
  }

  int cc = op & 3, bbb = (op >> 2) & 7, aaa = op >> 5;
  bool cross; uint16_t addr; int m;

  if (cc == 1) {                                        // ORA AND EOR ADC STA LDA CMP SBC
    static const uint8_t mt[8] = {M_IZX, M_ZP, M_IMM, M_ABS, M_IZY, M_ZPX, M_ABY, M_ABX};
    m = mt[bbb];
    if (aaa == 4) {
      if (m == M_IMM) { cPC++; return 2; }
      addr = getAddr(m, cross); wr(addr, cA);
      return modeBase[m] + extra1(m);
    }
    addr = getAddr(m, cross);
    uint8_t v = rd(addr);
    switch (aaa) {
      case 0: cA |= v; setNZ(cA); break;
      case 1: cA &= v; setNZ(cA); break;
      case 2: cA ^= v; setNZ(cA); break;
      case 3: doADC(v); break;
      case 5: cA = v; setNZ(cA); break;
      case 6: doCMP(cA, v); break;
      case 7: doADC(~v); break;
    }
    return modeBase[m] + (cross ? 1 : 0);
  }

  if (cc == 2) {                                        // shifts, INC/DEC, LDX/STX
    if (bbb == 4 || bbb == 6) return 2;                 // unofficial / jam: treat as NOP
    if (bbb == 2) {
      switch (aaa) { case 0: cA = opASL(cA); break; case 1: cA = opROL(cA); break;
                     case 2: cA = opLSR(cA); break; case 3: cA = opROR(cA); break; }
      return 2;
    }
    if (bbb == 0) {
      if (aaa == 5) { cX = rd(cPC++); setNZ(cX); } else cPC++;
      return 2;
    }
    m = (bbb == 1) ? M_ZP : (bbb == 3) ? M_ABS : (bbb == 5) ? ((aaa == 4 || aaa == 5) ? M_ZPY : M_ZPX)
                                                          : ((aaa == 5) ? M_ABY : M_ABX);
    addr = getAddr(m, cross);
    if (aaa == 4) { if (bbb == 7) return 5; wr(addr, cX); return modeBase[m] + extra1(m); }
    if (aaa == 5) { cX = rd(addr); setNZ(cX); return modeBase[m] + (cross ? 1 : 0); }
    uint8_t v = rd(addr), r = 0;
    switch (aaa) {
      case 0: r = opASL(v); break;
      case 1: r = opROL(v); break;
      case 2: r = opLSR(v); break;
      case 3: r = opROR(v); break;
      case 6: r = v - 1; setNZ(r); break;
      default: r = v + 1; setNZ(r); break;
    }
    wr(addr, r);
    return modeBase[m] + 2 + extra1(m);
  }

  if (cc == 0) {                                        // BIT STY LDY CPY CPX (+ unofficial NOPs)
    if (bbb == 2 || bbb == 4 || bbb == 6) return 2;
    m = (bbb == 0) ? M_IMM : (bbb == 1) ? M_ZP : (bbb == 3) ? M_ABS : (bbb == 5) ? M_ZPX : M_ABX;
    addr = getAddr(m, cross);
    switch (aaa) {
      case 1:
        if (m == M_ZP || m == M_ABS) {
          uint8_t v = rd(addr);
          cP = (cP & ~(FN | FV | FZ)) | (v & 0xC0) | ((cA & v) ? 0 : FZ);
          return modeBase[m];
        }
        break;
      case 4: if (m != M_IMM && m != M_ABX) { wr(addr, cY); return modeBase[m]; } break;
      case 5: cY = rd(addr); setNZ(cY); return modeBase[m] + (cross ? 1 : 0);
      case 6: if (m == M_IMM || m == M_ZP || m == M_ABS) { doCMP(cY, rd(addr)); return modeBase[m]; } break;
      case 7: if (m == M_IMM || m == M_ZP || m == M_ABS) { doCMP(cX, rd(addr)); return modeBase[m]; } break;
    }
    return modeBase[m] + (cross ? 1 : 0);               // NOP with operand
  }

  // cc == 3: unofficial combined opcodes (SLO RLA SRE RRA SAX LAX DCP ISC)
  if (bbb == 2) { cPC++; return 2; }
  switch (bbb) {
    case 0: m = M_IZX; break;
    case 1: m = M_ZP; break;
    case 3: m = M_ABS; break;
    case 4: m = M_IZY; break;
    case 5: m = (aaa == 4 || aaa == 5) ? M_ZPY : M_ZPX; break;
    case 6: m = M_ABY; break;
    default: m = (aaa == 5) ? M_ABY : M_ABX; break;
  }
  addr = getAddr(m, cross);
  if (aaa == 4) {
    if (m == M_IZX || m == M_ZP || m == M_ABS || m == M_ZPY) { wr(addr, cA & cX); return modeBase[m]; }
    return 5;
  }
  uint8_t v = rd(addr), r;
  if (aaa == 5) { cA = cX = v; setNZ(v); return modeBase[m] + (cross ? 1 : 0); }
  switch (aaa) {
    case 0: r = opASL(v); wr(addr, r); cA |= r; setNZ(cA); break;
    case 1: r = opROL(v); wr(addr, r); cA &= r; setNZ(cA); break;
    case 2: r = opLSR(v); wr(addr, r); cA ^= r; setNZ(cA); break;
    case 3: r = opROR(v); wr(addr, r); doADC(r); break;
    case 6: r = v - 1; wr(addr, r); doCMP(cA, r); break;
    default: r = v + 1; wr(addr, r); doADC(~r); break;
  }
  return modeBase[m] + 2 + extra1(m);
}

static void nesReset() {
  memset(ram, 0, sizeof(ram));
  memset(vram, 0, sizeof(vram));
  memset(oam, 0xFF, sizeof(oam));
  for (int i = 0; i < 32; i++) { pal[i] = 0; palRgb[i] = nesRgb[0]; }
  ppuCtrl = ppuMask = ppuStatus = oamAddr = fineX = wLatch = readBuf = 0;
  ppuV = ppuT = 0; nmiPending = false; irqLine = false;
  cartInit();
  cA = cX = cY = 0; cSP = 0xFD; cP = FI | FU;
  cPC = rd16(0xFFFC);
}

// =====================================================================================
//  One emulated frame (262 scanlines)
// =====================================================================================
static int32_t cpuBudget = 0;   // CPU time still owed, in PPU dots

static void emuFrame(bool draw) {
  lastOutRow = -1;
  flushRows();
  for (int sl = 0; sl < 262; sl++) {
    if (sl == 241) { ppuStatus |= 0x80; if (ppuCtrl & 0x80) nmiPending = true; }
    else if (sl == 261) ppuStatus &= 0x1F;               // end of vblank: clear vblank, sprite 0, overflow

    if (sl < 240) {
      bool want = false; int outRow = 0;
      if (sl >= NES_TOP && sl < NES_TOP + NES_LINES) {
        int o = ((sl - NES_TOP) * SH) / NES_LINES;
        if (o != lastOutRow) { lastOutRow = o; if (draw) { want = true; outRow = o; } }
      }
      bool rendering = (ppuMask & 0x18) != 0;
      bool s0 = false;
      if (!want && rendering && !(ppuStatus & 0x40) && (ppuMask & 0x18) == 0x18) {
        int row = sl - (oam[0] + 1);
        s0 = (row >= 0 && row < ((ppuCtrl & 0x20) ? 16 : 8));
      }
      if (rendering && (want || s0)) renderLine(sl, want, outRow);
      else if (want) {                                    // rendering off: plain backdrop color
        uint16_t bd = palRgb[0];
        uint16_t *dst = rowSlot(outRow);
        for (int ox = 0; ox < OUT_W; ox++) dst[ox] = bd;
        rowDone();
      }
    }

    cpuBudget += 341;                                     // 341 PPU dots per scanline = 113.67 CPU cycles
    while (cpuBudget > 0) {
      int c = cpuStep();
      if (dmaCycles) { c += dmaCycles; dmaCycles = 0; }
      cpuBudget -= c * 3;
    }

    if (ppuMask & 0x18) {
      if (sl < 240) { ppuEndLine(); if (mapperId == 4) mmc3Clock(); }
      else if (sl == 261) {
        ppuV = (ppuV & ~0x041F) | (ppuT & 0x041F);
        ppuV = (ppuV & ~0x7BE0) | (ppuT & 0x7BE0);         // copy vertical scroll bits
        if (mapperId == 4) mmc3Clock();
      }
    }
  }
  flushRows();                                            // send the last rows of the picture
}

// Builds the lookup tables. Call once before nesReset().
static void nesInit() {
  for (int i = 0; i < 64; i++) nesRgb[i] = C(NES_PAL[i][0], NES_PAL[i][1], NES_PAL[i][2]);
  for (int i = 0; i < OUT_W; i++) xmap[i] = (i * 256) / OUT_W;
  for (int i = 0; i < 256; i++) { uint16_t v = 0; for (int b = 0; b < 8; b++) if (i & (1 << b)) v |= 1 << (2 * b); spread16[i] = v; }
}

// the CPU flag names above are only needed inside this file
#undef FC
#undef FZ
#undef FI
#undef FD
#undef FB
#undef FU
#undef FV
#undef FN
