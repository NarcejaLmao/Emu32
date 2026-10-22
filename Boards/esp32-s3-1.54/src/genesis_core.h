// ============================================================================
// Emu32 Sega Genesis / Mega Drive core (basic).
//
//  * Motorola 68000 interpreter (full instruction set, approximate bus timing,
//    no address-error / privilege-violation traps)
//  * VDP in Mode 5: planes A/B, window, sprites, H32/H40, V28/V30, hscroll modes
//    (full/cell/line), vscroll modes (full/2-cell), VRAM/CRAM/VSRAM, DMA
//    (68K->VDP, fill, copy), H-int and V-int
//  * 3-button pad on port 1, version/IO registers, Z80 bus-request handshake
//  * ROM stays on the SD card (2-way, 2 KB page cache), up to 4 MB, raw .bin/.md/.gen
//
//  NOT emulated: Z80 CPU (its RAM is present, but never executed), YM2612, PSG,
//  shadow/highlight, interlace, SRAM/bank-switch mappers, 6-button pad, port 2,
//  .smd interleaved ROMs. PSG and lightweight YM2612 audio registers feed the frontend sound generator.
//
//  Needs (from the sketch): lcdWindow / lcdCfg / padState / padBtnC / loadErr1-2,
//  sdDiagReadAt, sdMount, sdBuildStatePath, sdStateWrite/Read, currentRomPath,
//  shotRow, GAME_X/Y/W/H.
//  Public: genLoad(path) genReset() genFrame(draw) genShutdown() genRomReadFailed()
//          gen::stateSave(slot) gen::stateLoad(slot)
// ============================================================================
#ifndef EMU32_GENESIS_CORE_H
#define EMU32_GENESIS_CORE_H
#include <esp_heap_caps.h>

namespace gen {

static const uint32_t RAM_SZ = 65536, VRAM_SZ = 65536, ZRAM_SZ = 8192;
static const uint32_t BLOB_SZ = RAM_SZ + VRAM_SZ + ZRAM_SZ;
static const int CYC_LINE = 488, LINES_TOTAL = 262;

static uint8_t  *blob = nullptr;                 // 68K RAM + VRAM + Z80 RAM (contiguous for save states)
static uint8_t  *ram = nullptr, *vram = nullptr, *zram = nullptr;
static uint16_t *fb = nullptr;                   // GAME_W x GAME_H, byte-swapped RGB565 (wire order)
static bool      romOk = false, romReadError = false;

// ------------------------------------------------------------- ROM cache ----
static File      romFile;
static uint32_t  romSize = 0;
static const int PG_SH = 11, PG_SZ = 2048, NSETS = 32;
static uint8_t  *pgData = nullptr;               // NSETS * 2 ways * 2 KB
static int16_t   pgTag[NSETS * 2];
static uint8_t   pgVict[NSETS];
static const uint8_t *fetchBase = nullptr;
static int32_t   fetchPage = -1;

static const uint8_t *romPage(uint32_t pg) {
  const uint32_t set = pg & (NSETS - 1);
  for (int w = 0; w < 2; w++)
    if (pgTag[set * 2 + w] == (int16_t)pg) { pgVict[set] = w ^ 1; return pgData + (((set * 2 + w)) << PG_SH); }
  const int w = pgVict[set]; pgVict[set] = w ^ 1;
  uint8_t *d = pgData + (((set * 2 + w)) << PG_SH);
  const uint32_t off = pg << PG_SH;
  size_t want = 0;
  if (off < romSize) want = (romSize - off) < (uint32_t)PG_SZ ? (romSize - off) : PG_SZ;
  if (want && sdDiagReadAt(romFile, off, d, want) != (int)want) { romReadError = true; memset(d, 0, PG_SZ); }
  else if (want < (size_t)PG_SZ) memset(d + want, 0, PG_SZ - want);
  pgTag[set * 2 + w] = (int16_t)pg;
  fetchPage = -1;                                // the fetch pointer may have been evicted
  return d;
}

// ------------------------------------------------------------------ state ----
static int32_t  cyc = 0, cycTarget = 0, lineBase = 0;
static uint32_t d[8], a[8], pc, spAlt;
static uint32_t fX, fN, fZ, fV, fC;
static uint8_t  ipm, fS, fT;
static bool     stopped;

// VDP
static uint8_t  reg[32];
static uint16_t cram[64], vsram[64];
static uint16_t pal565[64];                      // host order RGB565 for each CRAM entry
static uint16_t vAddr; static uint8_t vCode; static bool vPending, dmaFill;
static bool     vintPend, hintPend, vBlank, hBlank, vintFlag;
static int      curLine, hcnt;
static int      vdpW = 320, vdpH = 224;

// IO / Z80
static uint8_t  ioData1, ioCtrl1, ioData2, ioCtrl2, ioData3, ioCtrl3;
static uint16_t psgReg[8];
static uint8_t psgLatch = 0, ymAddr[2], ymReg[2][256];
static bool     zBusReq, zReset;

static int16_t  lineOut[256];                    // source line -> output row (or -1)
static uint16_t xmap[GAME_W];
static int      mapW = 0, mapH = 0;
static uint8_t  lnA[336], lnB[336], lnS[336], lnC[336];

// =================================================================== VDP ======
static inline void setPal(int i, uint16_t v) {
  cram[i] = v & 0x0EEE;
  const uint8_t r = (v >> 1) & 7, g = (v >> 5) & 7, b = (v >> 9) & 7;
  pal565[i] = ((r << 2 | r >> 1) << 11) | ((g << 3 | g) << 5) | (b << 2 | b >> 1);
}

static uint16_t rd16(uint32_t ad);

static void vdpPut(uint16_t w) {
  switch (vCode & 0x0F) {
    case 1: {
      const uint16_t ad = vAddr;
      vram[ad & 0xFFFE] = (ad & 1) ? (w & 0xFF) : (w >> 8);
      vram[ad | 1]      = (ad & 1) ? (w >> 8)   : (w & 0xFF);
      break;
    }
    case 3: setPal((vAddr >> 1) & 63, w); break;
    case 5: vsram[(vAddr >> 1) & 63] = w & 0x7FF; break;
  }
  vAddr += reg[15];
}

static uint16_t vdpGet() {
  uint16_t v = 0;
  switch (vCode & 0x0F) {
    case 0: v = (vram[vAddr & 0xFFFE] << 8) | vram[vAddr | 1]; break;
    case 8: v = cram[(vAddr >> 1) & 63]; break;
    case 4: v = vsram[(vAddr >> 1) & 63]; break;
  }
  vAddr += reg[15];
  return v;
}

static void dmaStart() {
  uint32_t len = reg[19] | (reg[20] << 8); if (!len) len = 0x10000;
  const int mode = reg[23] >> 6;
  if (!(mode & 2)) {                             // 68K -> VDP
    uint32_t src = ((uint32_t)(reg[23] & 0x7F) << 17) | ((uint32_t)reg[22] << 9) | ((uint32_t)reg[21] << 1);
    for (uint32_t i = 0; i < len; i++) {
      vdpPut(rd16(src));
      src = (src & 0xFE0000) | ((src + 2) & 0x1FFFF);
    }
    reg[19] = reg[20] = 0;
  } else if (mode == 2) {
    dmaFill = true;                              // runs on the next data-port write
  } else {                                       // VRAM copy
    uint16_t src = reg[21] | (reg[22] << 8);
    for (uint32_t i = 0; i < len; i++) { vram[vAddr] = vram[src]; src++; vAddr += reg[15]; }
    reg[19] = reg[20] = 0;
  }
}

static void vdpCtrlWrite(uint16_t w) {
  if (vPending) {
    vPending = false;
    vCode = (vCode & 3) | ((w >> 2) & 0x3C);
    vAddr = (vAddr & 0x3FFF) | ((w & 3) << 14);
    if ((vCode & 0x20) && (reg[1] & 0x10)) dmaStart();
  } else if ((w & 0xC000) == 0x8000) {
    const int r = (w >> 8) & 0x1F;
    if (r < 24) reg[r] = w & 0xFF;
  } else {
    vCode = (vCode & 0x3C) | (w >> 14);
    vAddr = (vAddr & 0xC000) | (w & 0x3FFF);
    vPending = true;
  }
}

static void vdpDataWrite(uint16_t w) {
  vPending = false;
  if (dmaFill) {
    dmaFill = false;
    uint32_t len = reg[19] | (reg[20] << 8); if (!len) len = 0x10000;
    vdpPut(w);
    const uint8_t f = w >> 8;
    for (uint32_t i = 0; i < len; i++) { vram[vAddr ^ 1] = f; vAddr += reg[15]; }
    reg[19] = reg[20] = 0;
    return;
  }
  vdpPut(w);
}

static uint16_t vdpStatus() {
  vPending = false;
  uint16_t s = 0x3600;                           // FIFO empty
  if (vintFlag) s |= 0x80;
  if (vBlank)   s |= 0x08;
  if (hBlank)   s |= 0x04;
  return s;
}

static uint16_t vdpHV() {
  int v = curLine; if (v > 0xEA) v -= 6;
  int h = ((cyc - lineBase) * 160 / CYC_LINE) & 0xFF;
  return (uint16_t)((v << 8) | h);
}

// ============================================================ IO / bus ========
static uint8_t padRead(uint8_t data, uint8_t ctrl) {
  const bool th = (data & ctrl & 0x40) || !(ctrl & 0x40);
  uint8_t bits;
  if (th) {
    bits = 0x3F;
    if (padState & 0x10) bits &= ~0x01;
    if (padState & 0x20) bits &= ~0x02;
    if (padState & 0x40) bits &= ~0x04;
    if (padState & 0x80) bits &= ~0x08;
    if (padState & 0x02) bits &= ~0x10;          // B
    if (padBtnC)         bits &= ~0x20;          // C
    bits |= 0x40;
  } else {
    bits = 0x33;                                 // L/R lines low in the TH=0 phase
    if (padState & 0x10) bits &= ~0x01;
    if (padState & 0x20) bits &= ~0x02;
    if (padState & 0x01) bits &= ~0x10;          // A
    if (padState & 0x08) bits &= ~0x20;          // START
  }
  return ((data & ctrl) | (bits & ~ctrl)) & 0x7F;
}

static uint8_t ioRead(uint32_t ad) {             // odd register addresses
  switch (ad & 0x1F) {
    case 0x01: return 0xA0;                      // overseas, NTSC, no expansion unit
    case 0x03: return padRead(ioData1, ioCtrl1);
    case 0x05: return 0x7F;                      // port 2: nothing connected
    case 0x07: return ioData3;
    case 0x09: return ioCtrl1;
    case 0x0B: return ioCtrl2;
    case 0x0D: return ioCtrl3;
  }
  return 0;
}
static void ioWrite(uint32_t ad, uint8_t v) {
  switch (ad & 0x1F) {
    case 0x03: ioData1 = v; break;
    case 0x05: ioData2 = v; break;
    case 0x07: ioData3 = v; break;
    case 0x09: ioCtrl1 = v; break;
    case 0x0B: ioCtrl2 = v; break;
    case 0x0D: ioCtrl3 = v; break;
  }
}

static uint16_t rd16(uint32_t ad) {
  ad &= 0xFFFFFF; cyc += 4;
  if (ad < 0x400000) { const uint8_t *p = romPage(ad >> PG_SH) + (ad & (PG_SZ - 1)); return (p[0] << 8) | p[1]; }
  if (ad >= 0xE00000) { ad &= 0xFFFE; return (ram[ad] << 8) | ram[ad + 1]; }
  if ((ad & 0xE00000) == 0xC00000) {
    const int p = (ad & 0x1F) >> 1;              // 0-1 data, 2-3 control, 4-7 HV counter, 8+ PSG
    if (p < 2) return vdpGet();
    if (p < 4) return vdpStatus();
    return p < 8 ? vdpHV() : 0;
  }
  if (ad >= 0xA00000 && ad < 0xA10000) {
    if (ad < 0xA04000) { const uint8_t b = zram[ad & 0x1FFF]; return (b << 8) | b; }
    return 0;                                    // YM2612 status: never busy
  }
  if (ad >= 0xA10000 && ad < 0xA10020) return ioRead(ad | 1);
  if (ad == 0xA11100) return (zBusReq && !zReset) ? 0x0000 : 0x0100;
  return 0;
}
static uint8_t rd8(uint32_t ad) {
  ad &= 0xFFFFFF;
  if (ad < 0x400000) { cyc += 4; return romPage(ad >> PG_SH)[ad & (PG_SZ - 1)]; }
  if (ad >= 0xE00000) { cyc += 4; return ram[ad & 0xFFFF]; }
  if (ad >= 0xA00000 && ad < 0xA04000) { cyc += 4; return zram[ad & 0x1FFF]; }
  if (ad >= 0xA10000 && ad < 0xA10020) { cyc += 4; return (ad & 1) ? ioRead(ad) : 0; }
  if (ad == 0xA11100) { cyc += 4; return (zBusReq && !zReset) ? 0 : 1; }
  const uint16_t w = rd16(ad & ~1u);             // rd16 charges the cycles
  return (ad & 1) ? (w & 0xFF) : (w >> 8);
}

static void wr16(uint32_t ad, uint16_t v) {
  ad &= 0xFFFFFF; cyc += 4;
  if (ad >= 0xE00000) { ad &= 0xFFFE; ram[ad] = v >> 8; ram[ad + 1] = v & 0xFF; return; }
  if ((ad & 0xE00000) == 0xC00000) {
    const int p = (ad & 0x1F) >> 1;
    if (p < 2) vdpDataWrite(v); else if (p < 4) vdpCtrlWrite(v);   // PSG / others ignored
    return;
  }
  if (ad >= 0xA00000 && ad < 0xA10000) {
    if (ad < 0xA02000) zram[ad & 0x1FFF] = v >> 8;
    else if (ad >= 0xA04000 && ad < 0xA04004) { const int port = (ad >> 1) & 1; ymAddr[port] = (uint8_t)(v >> 8); }
    return;
  }
  if (ad >= 0xA10000 && ad < 0xA10020) { ioWrite(ad | 1, v & 0xFF); return; }
  if (ad == 0xA11100) { zBusReq = (v >> 8) & 1; return; }
  if (ad == 0xA11200) { zReset = !((v >> 8) & 1); return; }
}
static void wr8(uint32_t ad, uint8_t v) {
  ad &= 0xFFFFFF;
  if (ad >= 0xE00000) { cyc += 4; ram[ad & 0xFFFF] = v; return; }
  if (ad >= 0xA00000 && ad < 0xA02000) { cyc += 4; zram[ad & 0x1FFF] = v; return; }
  if (ad >= 0xA04000 && ad < 0xA04004) { cyc += 4; const int port = (ad >> 1) & 1; if (ad & 1) ymReg[port][ymAddr[port]] = v; else ymAddr[port] = v; return; }
  if (ad >= 0xA10000 && ad < 0xA10020) { cyc += 4; if (ad & 1) ioWrite(ad, v); return; }
  if (ad == 0xA11100) { cyc += 4; zBusReq = v & 1; return; }
  if (ad == 0xA11200) { cyc += 4; zReset = !(v & 1); return; }
  if ((ad & 0xE00000) == 0xC00000) { wr16(ad & ~1u, (v << 8) | v); return; }   // byte to VDP: duplicated
  cyc += 4;
}
static inline uint32_t rd32(uint32_t ad) { const uint32_t h = rd16(ad); return (h << 16) | rd16(ad + 2); }
static inline void wr32(uint32_t ad, uint32_t v) { wr16(ad, v >> 16); wr16(ad + 2, v & 0xFFFF); }

// ================================================================ 68000 =======
static inline uint16_t fetch16() {
  const uint32_t ad = pc & 0xFFFFFF; pc += 2; cyc += 4;
  if (ad < 0x400000) {
    const uint32_t pg = ad >> PG_SH;
    if ((int32_t)pg != fetchPage) { fetchBase = romPage(pg); fetchPage = pg; }
    const uint8_t *p = fetchBase + (ad & (PG_SZ - 1));
    return (p[0] << 8) | p[1];
  }
  cyc -= 4;
  return rd16(ad);
}
static inline uint32_t fetch32() { const uint32_t h = fetch16(); return (h << 16) | fetch16(); }

static inline uint32_t szMask(int sz) { return sz == 1 ? 0xFF : sz == 2 ? 0xFFFF : 0xFFFFFFFFu; }
static inline uint32_t szMsb(int sz)  { return sz == 1 ? 0x80 : sz == 2 ? 0x8000 : 0x80000000u; }
static inline uint32_t rdS(uint32_t ad, int sz) { return sz == 1 ? rd8(ad) : sz == 2 ? rd16(ad) : rd32(ad); }
static inline void wrS(uint32_t ad, int sz, uint32_t v) { if (sz == 1) wr8(ad, v); else if (sz == 2) wr16(ad, v); else wr32(ad, v); }

static inline uint16_t getSR() {
  return (fT << 15) | (fS << 13) | (ipm << 8) | (fX << 4) | (fN << 3) | (fZ << 2) | (fV << 1) | fC;
}
static inline void setCCR(uint16_t v) { fX = (v >> 4) & 1; fN = (v >> 3) & 1; fZ = (v >> 2) & 1; fV = (v >> 1) & 1; fC = v & 1; }
static inline void setS(uint8_t s) { if (s != fS) { const uint32_t t = a[7]; a[7] = spAlt; spAlt = t; fS = s; } }
static inline void setSR(uint16_t v) { setS((v >> 13) & 1); fT = (v >> 15) & 1; ipm = (v >> 8) & 7; setCCR(v); }

static inline void push16(uint16_t v) { a[7] -= 2; wr16(a[7], v); }
static inline void push32(uint32_t v) { a[7] -= 4; wr32(a[7], v); }
static inline uint16_t pop16() { const uint16_t v = rd16(a[7]); a[7] += 2; return v; }
static inline uint32_t pop32() { const uint32_t v = rd32(a[7]); a[7] += 4; return v; }

static void excep(int vec, uint32_t retPc) {
  const uint16_t sr = getSR();
  setS(1); fT = 0;
  push32(retPc); push16(sr);
  pc = rd32(vec * 4);
  cyc += 20;
}

static inline bool cc(int c) {
  switch (c) {
    case 0: return true;   case 1: return false;
    case 2: return !fC && !fZ;  case 3: return fC || fZ;
    case 4: return !fC;    case 5: return fC;
    case 6: return !fZ;    case 7: return fZ;
    case 8: return !fV;    case 9: return fV;
    case 10: return !fN;   case 11: return fN;
    case 12: return fN == fV;   case 13: return fN != fV;
    case 14: return !fZ && (fN == fV);
    default: return fZ || (fN != fV);
  }
}

static inline void setNZ(uint32_t r, int sz) { fN = (r & szMsb(sz)) ? 1 : 0; fZ = (r & szMask(sz)) == 0; }

static uint32_t aluAdd(uint32_t s, uint32_t dd, int sz, uint32_t xin, bool ext) {
  const uint32_t m = szMask(sz); s &= m; dd &= m;
  const uint64_t r = (uint64_t)s + dd + xin;
  const uint32_t res = (uint32_t)r & m;
  fC = fX = r > m;
  fV = (((s ^ res) & (dd ^ res)) & szMsb(sz)) ? 1 : 0;
  fN = (res & szMsb(sz)) ? 1 : 0;
  if (ext) { if (res) fZ = 0; } else fZ = res == 0;
  return res;
}
static uint32_t aluSub(uint32_t s, uint32_t dd, int sz, uint32_t xin, bool ext, bool setX) {
  const uint32_t m = szMask(sz); s &= m; dd &= m;
  const uint32_t res = (dd - s - xin) & m;
  fC = ((uint64_t)s + xin) > dd;
  if (setX) fX = fC;
  fV = (((s ^ dd) & (res ^ dd)) & szMsb(sz)) ? 1 : 0;
  fN = (res & szMsb(sz)) ? 1 : 0;
  if (ext) { if (res) fZ = 0; } else fZ = res == 0;
  return res;
}

// ---- effective addresses -----------------------------------------------------
static inline uint32_t indexExt(uint32_t base) {
  const uint16_t e = fetch16();
  uint32_t idx = (e & 0x8000) ? a[(e >> 12) & 7] : d[(e >> 12) & 7];
  if (!(e & 0x800)) idx = (int32_t)(int16_t)idx;
  return base + (int32_t)(int8_t)e + idx;
}
static uint32_t eaAddr(int mode, int r, int sz) {
  switch (mode) {
    case 2: return a[r];
    case 3: { const uint32_t v = a[r]; a[r] += (sz == 1 && r == 7) ? 2 : sz; return v; }
    case 4: a[r] -= (sz == 1 && r == 7) ? 2 : sz; cyc += 2; return a[r];
    case 5: { const int16_t dp = (int16_t)fetch16(); return a[r] + dp; }
    case 6: cyc += 2; return indexExt(a[r]);
    default:
      switch (r) {
        case 0: return (uint32_t)(int32_t)(int16_t)fetch16();
        case 1: return fetch32();
        case 2: { const uint32_t b = pc; const int16_t dp = (int16_t)fetch16(); return b + dp; }
        default: { cyc += 2; return indexExt(pc); }
      }
  }
}
struct Ea { uint8_t mode, reg; uint32_t ad; };
static void eaPrep(Ea &e, int mode, int r, int sz) {
  e.mode = mode; e.reg = r;
  if (mode < 2) return;
  if (mode == 7 && r == 4) { e.ad = (sz == 4) ? fetch32() : (fetch16() & szMask(sz)); return; }
  e.ad = eaAddr(mode, r, sz);
}
static inline uint32_t eaGet(const Ea &e, int sz) {
  if (e.mode == 0) return d[e.reg] & szMask(sz);
  if (e.mode == 1) return a[e.reg] & szMask(sz);
  if (e.mode == 7 && e.reg == 4) return e.ad;
  return rdS(e.ad, sz);
}
static inline void eaPut(const Ea &e, int sz, uint32_t v) {
  if (e.mode == 0) {
    if (sz == 4) d[e.reg] = v;
    else if (sz == 2) d[e.reg] = (d[e.reg] & 0xFFFF0000u) | (v & 0xFFFF);
    else d[e.reg] = (d[e.reg] & 0xFFFFFF00u) | (v & 0xFF);
  } else if (e.mode == 1) a[e.reg] = v;
  else wrS(e.ad, sz, v);
}
static inline int sizeBits(int b) { return b == 0 ? 1 : b == 1 ? 2 : 4; }

// ---- shifts / rotates --------------------------------------------------------
static uint32_t shiftOp(int type, bool left, uint32_t v, uint32_t cnt, int sz) {
  const uint32_t m = szMask(sz), msb = szMsb(sz);
  v &= m; fV = 0;
  if (cnt == 0) { fC = (type == 2) ? fX : 0; setNZ(v, sz); return v; }
  cyc += 2 * cnt;
  for (uint32_t i = 0; i < cnt; i++) {
    uint32_t c;
    switch (type) {
      case 0:
        if (left) { c = (v & msb) != 0; const uint32_t nv = (v << 1) & m; if ((nv ^ v) & msb) fV = 1; v = nv; }
        else { c = v & 1; v = (v >> 1) | (v & msb); }
        fX = c; break;
      case 1:
        if (left) { c = (v & msb) != 0; v = (v << 1) & m; } else { c = v & 1; v >>= 1; }
        fX = c; break;
      case 2:
        if (left) { c = (v & msb) != 0; v = ((v << 1) | fX) & m; } else { c = v & 1; v = (v >> 1) | (fX ? msb : 0); }
        fX = c; break;
      default:
        if (left) { c = (v & msb) != 0; v = ((v << 1) | c) & m; } else { c = v & 1; v = (v >> 1) | (c ? msb : 0); }
        break;
    }
    fC = c;
  }
  setNZ(v, sz);
  return v;
}

static void bitOp(int type, int mode, int r, uint32_t bitn) {
  if (mode == 0) {
    const uint32_t m = 1u << (bitn & 31);
    fZ = !(d[r] & m);
    if (type == 1) d[r] ^= m; else if (type == 2) d[r] &= ~m; else if (type == 3) d[r] |= m;
    cyc += 2;
  } else {
    Ea e; eaPrep(e, mode, r, 1);
    const uint32_t m = 1u << (bitn & 7);
    uint32_t v = eaGet(e, 1);
    fZ = !(v & m);
    if (type) { if (type == 1) v ^= m; else if (type == 2) v &= ~m; else v |= m; eaPut(e, 1, v); }
  }
}

static void logicGroup(uint16_t op, int kind) {  // kind 0 = OR, 1 = AND
  const int rg = (op >> 9) & 7, opm = (op >> 6) & 7, mode = (op >> 3) & 7, r = op & 7;
  const int sz = sizeBits(opm & 3);
  Ea e; eaPrep(e, mode, r, sz);
  const uint32_t src = eaGet(e, sz);
  if (opm < 3) {
    const uint32_t dd = d[rg] & szMask(sz);
    const uint32_t res = kind ? (dd & src) : (dd | src);
    setNZ(res, sz); fV = fC = 0;
    eaPut(Ea{0, (uint8_t)rg, 0}, sz, res);
    if (sz == 4) cyc += 4;
  } else {
    const uint32_t dd = d[rg] & szMask(sz);
    const uint32_t res = kind ? (dd & src) : (dd | src);
    setNZ(res, sz); fV = fC = 0;
    eaPut(e, sz, res);
  }
}

static uint8_t bcdAdd(uint8_t s, uint8_t dd) {
  uint32_t lo = (s & 15) + (dd & 15) + fX, hi = (s >> 4) + (dd >> 4);
  if (lo > 9) { lo -= 10; hi++; }
  fC = 0; if (hi > 9) { hi -= 10; fC = 1; }
  const uint8_t res = (hi << 4) | lo; fX = fC; if (res) fZ = 0; return res;
}
static uint8_t bcdSub(uint8_t s, uint8_t dd) {
  int lo = (dd & 15) - (s & 15) - (int)fX, hi = (dd >> 4) - (s >> 4);
  if (lo < 0) { lo += 10; hi--; }
  fC = 0; if (hi < 0) { hi += 10; fC = 1; }
  const uint8_t res = ((hi & 15) << 4) | lo; fX = fC; if (res) fZ = 0; return res;
}

static inline uint32_t regVal(int i) { return i < 8 ? d[i] : a[i - 8]; }
static inline void setReg(int i, uint32_t v) { if (i < 8) d[i] = v; else a[i - 8] = v; }

static void step() {
  const uint16_t op = fetch16();
  const int mode = (op >> 3) & 7, r = op & 7, rg = (op >> 9) & 7;
  switch (op >> 12) {
  // ------------------------------------------------------------------ 0 ----
  case 0x0: {
    if (op & 0x100) {
      if (mode == 1) {                           // MOVEP
        const int16_t dp = (int16_t)fetch16(); uint32_t ad = a[r] + dp; const int opm = (op >> 6) & 7;
        if (opm == 4) { d[rg] = (d[rg] & 0xFFFF0000u) | (rd8(ad) << 8) | rd8(ad + 2); }
        else if (opm == 5) { uint32_t v = rd8(ad) << 24; v |= rd8(ad + 2) << 16; v |= rd8(ad + 4) << 8; v |= rd8(ad + 6); d[rg] = v; }
        else if (opm == 6) { wr8(ad, d[rg] >> 8); wr8(ad + 2, d[rg]); }
        else { wr8(ad, d[rg] >> 24); wr8(ad + 2, d[rg] >> 16); wr8(ad + 4, d[rg] >> 8); wr8(ad + 6, d[rg]); }
        break;
      }
      bitOp((op >> 6) & 3, mode, r, d[rg]);
      break;
    }
    const int kind = rg;
    if (kind == 4) { const uint32_t bn = fetch16() & 0xFF; bitOp((op >> 6) & 3, mode, r, bn); break; }
    if (op == 0x003C || op == 0x023C || op == 0x0A3C) {           // ORI/ANDI/EORI to CCR
      const uint16_t im = fetch16() & 0xFF, ccr = getSR() & 0xFF;
      setCCR(kind == 0 ? (ccr | im) : kind == 1 ? (ccr & im) : (ccr ^ im)); break;
    }
    if (op == 0x007C || op == 0x027C || op == 0x0A7C) {           // ... to SR
      const uint16_t im = fetch16(), sr = getSR();
      setSR(kind == 0 ? (sr | im) : kind == 1 ? (sr & im) : (sr ^ im)); break;
    }
    const int sz = sizeBits((op >> 6) & 3);
    const uint32_t imm = (sz == 4) ? fetch32() : (fetch16() & szMask(sz));
    Ea e; eaPrep(e, mode, r, sz);
    const uint32_t v = eaGet(e, sz);
    uint32_t res;
    switch (kind) {
      case 0: res = v | imm; setNZ(res, sz); fV = fC = 0; break;
      case 1: res = v & imm; setNZ(res, sz); fV = fC = 0; break;
      case 2: res = aluSub(imm, v, sz, 0, false, true); break;
      case 3: res = aluAdd(imm, v, sz, 0, false); break;
      case 5: res = v ^ imm; setNZ(res, sz); fV = fC = 0; break;
      default: aluSub(imm, v, sz, 0, false, false); if (sz == 4 && mode == 0) cyc += 2; return;   // CMPI
    }
    eaPut(e, sz, res);
    if (sz == 4 && mode == 0) cyc += 4;
    break;
  }
  // ------------------------------------------------------------- 1,2,3 ----
  case 0x1: case 0x2: case 0x3: {
    const int sz = (op >> 12) == 1 ? 1 : (op >> 12) == 3 ? 2 : 4;
    Ea s; eaPrep(s, mode, r, sz);
    uint32_t v = eaGet(s, sz);
    const int dm = (op >> 6) & 7;
    if (dm == 1) { a[rg] = (sz == 2) ? (uint32_t)(int32_t)(int16_t)v : v; break; }    // MOVEA
    Ea t; eaPrep(t, dm, rg, sz);
    setNZ(v, sz); fV = fC = 0;
    eaPut(t, sz, v);
    break;
  }
  // ------------------------------------------------------------------ 4 ----
  case 0x4: {
    if (op & 0x100) {
      if ((op & 0x1C0) == 0x1C0) {               // LEA
        a[rg] = eaAddr(mode, r, 4);
      } else if ((op & 0x1C0) == 0x180) {        // CHK
        Ea e; eaPrep(e, mode, r, 2);
        const int16_t b = (int16_t)eaGet(e, 2), v = (int16_t)d[rg];
        if (v < 0) { fN = 1; excep(6, pc); } else if (v > b) { fN = 0; excep(6, pc); }
      } else excep(4, pc - 2);
      break;
    }
    switch ((op >> 6) & 0x3F) {
      case 0x00: case 0x01: case 0x02: {         // NEGX
        const int sz = sizeBits((op >> 6) & 3); Ea e; eaPrep(e, mode, r, sz);
        eaPut(e, sz, aluSub(eaGet(e, sz), 0, sz, fX, true, true)); break; }
      case 0x03: { Ea e; eaPrep(e, mode, r, 2); eaPut(e, 2, getSR()); break; }       // MOVE from SR
      case 0x08: case 0x09: case 0x0A: {         // CLR
        const int sz = sizeBits((op >> 6) & 3); Ea e; eaPrep(e, mode, r, sz);
        eaPut(e, sz, 0); fN = 0; fZ = 1; fV = fC = 0; break; }
      case 0x10: case 0x11: case 0x12: {         // NEG
        const int sz = sizeBits((op >> 6) & 3); Ea e; eaPrep(e, mode, r, sz);
        eaPut(e, sz, aluSub(eaGet(e, sz), 0, sz, 0, false, true)); break; }
      case 0x13: { Ea e; eaPrep(e, mode, r, 2); setCCR(eaGet(e, 2)); break; }        // MOVE to CCR
      case 0x18: case 0x19: case 0x1A: {         // NOT
        const int sz = sizeBits((op >> 6) & 3); Ea e; eaPrep(e, mode, r, sz);
        const uint32_t res = ~eaGet(e, sz) & szMask(sz); setNZ(res, sz); fV = fC = 0; eaPut(e, sz, res); break; }
      case 0x1B: { Ea e; eaPrep(e, mode, r, 2); setSR(eaGet(e, 2)); break; }         // MOVE to SR
      case 0x20: {                               // NBCD
        Ea e; eaPrep(e, mode, r, 1); eaPut(e, 1, bcdSub(eaGet(e, 1), 0)); break; }
      case 0x21:
        if (mode == 0) { d[r] = (d[r] << 16) | (d[r] >> 16); setNZ(d[r], 4); fV = fC = 0; }   // SWAP
        else push32(eaAddr(mode, r, 4));                                                   // PEA
        break;
      case 0x22: case 0x23: case 0x32: case 0x33: {
        if (mode == 0 && !(op & 0x400)) {        // EXT
          if (op & 0x40) { d[r] = (int32_t)(int16_t)d[r]; setNZ(d[r], 4); }
          else { d[r] = (d[r] & 0xFFFF0000u) | (uint16_t)(int16_t)(int8_t)d[r]; setNZ(d[r], 2); }
          fV = fC = 0; break;
        }
        const uint16_t mask = fetch16(); const int sz = (op & 0x40) ? 4 : 2;
        if (op & 0x400) {                        // memory -> registers
          uint32_t ad = (mode == 3) ? a[r] : eaAddr(mode, r, sz);
          for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
            const uint32_t v = sz == 2 ? (uint32_t)(int32_t)(int16_t)rd16(ad) : rd32(ad);
            setReg(i, v); ad += sz;
          }
          if (mode == 3) a[r] = ad;
        } else if (mode == 4) {                  // registers -> memory, predecrement (reversed mask)
          uint32_t ad = a[r];
          for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
            ad -= sz; const uint32_t v = regVal(15 - i);
            if (sz == 2) wr16(ad, v); else wr32(ad, v);
          }
          a[r] = ad;
        } else {
          uint32_t ad = eaAddr(mode, r, sz);
          for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
            const uint32_t v = regVal(i);
            if (sz == 2) wr16(ad, v); else wr32(ad, v);
            ad += sz;
          }
        }
        break;
      }
      case 0x28: case 0x29: case 0x2A: {         // TST
        const int sz = sizeBits((op >> 6) & 3); Ea e; eaPrep(e, mode, r, sz);
        setNZ(eaGet(e, sz), sz); fV = fC = 0; break; }
      case 0x2B:
        if (op == 0x4AFC) { excep(4, pc - 2); break; }
        { Ea e; eaPrep(e, mode, r, 1); const uint32_t v = eaGet(e, 1); setNZ(v, 1); fV = fC = 0; eaPut(e, 1, v | 0x80); }
        break;                                   // TAS
      case 0x39:
        if (op < 0x4E50) { excep(32 + (op & 15), pc); }                               // TRAP
        else if (op < 0x4E58) { const int16_t dp = (int16_t)fetch16(); push32(a[r]); a[r] = a[7]; a[7] += dp; }   // LINK
        else if (op < 0x4E60) { a[7] = a[r]; a[r] = pop32(); }                        // UNLK
        else if (op < 0x4E70) { if (op & 8) a[r] = spAlt; else spAlt = a[r]; }        // MOVE USP (valid in supervisor)
        else switch (op) {
          case 0x4E70: break;                                                          // RESET
          case 0x4E71: break;                                                          // NOP
          case 0x4E72: setSR(fetch16()); stopped = true; break;                        // STOP
          case 0x4E73: { const uint16_t sr = pop16(); const uint32_t np = pop32(); setSR(sr); pc = np; break; }   // RTE
          case 0x4E75: pc = pop32(); break;                                            // RTS
          case 0x4E76: if (fV) excep(7, pc); break;                                    // TRAPV
          case 0x4E77: { setCCR(pop16()); pc = pop32(); break; }                       // RTR
          default: excep(4, pc - 2);
        }
        break;
      case 0x3A: { const uint32_t t = eaAddr(mode, r, 4); push32(pc); pc = t; break; }   // JSR
      case 0x3B: pc = eaAddr(mode, r, 4); break;                                        // JMP
      default: excep(4, pc - 2);
    }
    break;
  }
  // ------------------------------------------------------------------ 5 ----
  case 0x5: {
    if (((op >> 6) & 3) == 3) {
      const int c = (op >> 8) & 15;
      if (mode == 1) {                           // DBcc
        const uint32_t base = pc; const int16_t dp = (int16_t)fetch16();
        if (!cc(c)) {
          const uint16_t cnt = (uint16_t)d[r] - 1;
          d[r] = (d[r] & 0xFFFF0000u) | cnt;
          if (cnt != 0xFFFF) { pc = base + dp; cyc += 2; } else cyc += 4;
        } else cyc += 2;
      } else { Ea e; eaPrep(e, mode, r, 1); eaPut(e, 1, cc(c) ? 0xFF : 0); }   // Scc
    } else {
      uint32_t q = rg; if (!q) q = 8;
      const int sz = sizeBits((op >> 6) & 3);
      if (mode == 1) { if (op & 0x100) a[r] -= q; else a[r] += q; cyc += 4; }
      else {
        Ea e; eaPrep(e, mode, r, sz);
        const uint32_t v = eaGet(e, sz);
        eaPut(e, sz, (op & 0x100) ? aluSub(q, v, sz, 0, false, true) : aluAdd(q, v, sz, 0, false));
        if (sz == 4 && mode == 0) cyc += 4;
      }
    }
    break;
  }
  // ------------------------------------------------------------------ 6 ----
  case 0x6: {
    const int c = (op >> 8) & 15; const uint32_t base = pc;
    int32_t dp = (int8_t)op; bool w = false;
    if ((op & 0xFF) == 0) { dp = (int16_t)fetch16(); w = true; }
    if (c == 1) { push32(pc); pc = base + dp; cyc += 4; }                 // BSR
    else if (cc(c)) { pc = base + dp; cyc += 4; }
    else cyc += w ? 4 : 2;
    break;
  }
  case 0x7: d[rg] = (int32_t)(int8_t)op; setNZ(d[rg], 4); fV = fC = 0; break;
  // ------------------------------------------------------------------ 8 ----
  case 0x8: {
    const int opm = (op >> 6) & 7;
    if (opm == 3 || opm == 7) {                  // DIVU / DIVS
      Ea e; eaPrep(e, mode, r, 2);
      const uint32_t sv = eaGet(e, 2);
      if (sv == 0) { excep(5, pc); break; }
      cyc += 130;
      if (opm == 3) {
        const uint32_t dv = d[rg], q = dv / sv, rem = dv % sv;
        if (q > 0xFFFF) { fV = 1; fN = 1; fC = 0; }
        else { d[rg] = (rem << 16) | q; setNZ(q, 2); fV = fC = 0; }
      } else {
        const int32_t dv = (int32_t)d[rg]; const int32_t dsr = (int16_t)sv;
        const int64_t q = (int64_t)dv / dsr; const int32_t rem = (int32_t)((int64_t)dv % dsr);
        if (q > 32767 || q < -32768) { fV = 1; fN = 1; fC = 0; }
        else { d[rg] = ((uint32_t)(rem & 0xFFFF) << 16) | (uint16_t)q; setNZ((uint16_t)q, 2); fV = fC = 0; }
      }
    } else if (opm == 4 && mode < 2) {           // SBCD
      if (mode == 0) { const uint8_t res = bcdSub(d[r], d[rg]); d[rg] = (d[rg] & 0xFFFFFF00u) | res; }
      else { a[r] -= (r == 7) ? 2 : 1; const uint8_t s = rd8(a[r]); a[rg] -= (rg == 7) ? 2 : 1; wr8(a[rg], bcdSub(s, rd8(a[rg]))); }
    } else logicGroup(op, 0);
    break;
  }
  // ------------------------------------------------------------- 9 / D ----
  case 0x9: case 0xD: {
    const bool add = (op >> 12) == 0xD;
    const int opm = (op >> 6) & 7;
    if (opm == 3 || opm == 7) {                  // ADDA / SUBA
      const int sz = opm == 3 ? 2 : 4; Ea e; eaPrep(e, mode, r, sz);
      uint32_t s = eaGet(e, sz); if (sz == 2) s = (int32_t)(int16_t)s;
      a[rg] = add ? a[rg] + s : a[rg] - s; cyc += 4;
    } else if (opm >= 4 && mode < 2) {           // ADDX / SUBX
      const int sz = sizeBits(opm & 3);
      if (mode == 0) {
        const uint32_t res = add ? aluAdd(d[r], d[rg], sz, fX, true) : aluSub(d[r], d[rg], sz, fX, true, true);
        eaPut(Ea{0, (uint8_t)rg, 0}, sz, res);
      } else {
        a[r] -= (sz == 1 && r == 7) ? 2 : sz; const uint32_t s = rdS(a[r], sz);
        a[rg] -= (sz == 1 && rg == 7) ? 2 : sz; const uint32_t dd = rdS(a[rg], sz);
        wrS(a[rg], sz, add ? aluAdd(s, dd, sz, fX, true) : aluSub(s, dd, sz, fX, true, true));
      }
    } else {
      const int sz = sizeBits(opm & 3); Ea e; eaPrep(e, mode, r, sz);
      const uint32_t s = eaGet(e, sz);
      if (opm < 3) {
        const uint32_t dd = d[rg];
        const uint32_t res = add ? aluAdd(s, dd, sz, 0, false) : aluSub(s, dd, sz, 0, false, true);
        eaPut(Ea{0, (uint8_t)rg, 0}, sz, res);
        if (sz == 4) cyc += 4;
      } else {
        const uint32_t res = add ? aluAdd(d[rg], s, sz, 0, false) : aluSub(d[rg], s, sz, 0, false, true);
        eaPut(e, sz, res);
      }
    }
    break;
  }
  // ------------------------------------------------------------------ B ----
  case 0xB: {
    const int opm = (op >> 6) & 7;
    if (opm == 3 || opm == 7) {                  // CMPA
      const int sz = opm == 3 ? 2 : 4; Ea e; eaPrep(e, mode, r, sz);
      uint32_t s = eaGet(e, sz); if (sz == 2) s = (int32_t)(int16_t)s;
      aluSub(s, a[rg], 4, 0, false, false); cyc += 2;
    } else if (opm < 3) {                        // CMP
      const int sz = sizeBits(opm); Ea e; eaPrep(e, mode, r, sz);
      aluSub(eaGet(e, sz), d[rg], sz, 0, false, false);
    } else if (mode == 1) {                      // CMPM
      const int sz = sizeBits(opm & 3);
      const uint32_t s = rdS(a[r], sz); a[r] += (sz == 1 && r == 7) ? 2 : sz;
      const uint32_t dd = rdS(a[rg], sz); a[rg] += (sz == 1 && rg == 7) ? 2 : sz;
      aluSub(s, dd, sz, 0, false, false);
    } else {                                     // EOR
      const int sz = sizeBits(opm & 3); Ea e; eaPrep(e, mode, r, sz);
      const uint32_t res = (eaGet(e, sz) ^ d[rg]) & szMask(sz);
      setNZ(res, sz); fV = fC = 0; eaPut(e, sz, res);
    }
    break;
  }
  // ------------------------------------------------------------------ C ----
  case 0xC: {
    const int opm = (op >> 6) & 7; const uint16_t m = op & 0x1F8;
    if (opm == 3 || opm == 7) {                  // MULU / MULS
      Ea e; eaPrep(e, mode, r, 2);
      const uint32_t s = eaGet(e, 2); uint32_t res;
      if (opm == 3) res = (uint32_t)(uint16_t)d[rg] * s;
      else res = (uint32_t)((int32_t)(int16_t)d[rg] * (int32_t)(int16_t)s);
      d[rg] = res; setNZ(res, 4); fV = fC = 0; cyc += 66;
    } else if (opm == 4 && mode < 2) {           // ABCD
      if (mode == 0) { const uint8_t res = bcdAdd(d[r], d[rg]); d[rg] = (d[rg] & 0xFFFFFF00u) | res; }
      else { a[r] -= (r == 7) ? 2 : 1; const uint8_t s = rd8(a[r]); a[rg] -= (rg == 7) ? 2 : 1; wr8(a[rg], bcdAdd(s, rd8(a[rg]))); }
    } else if (m == 0x140) { const uint32_t t = d[rg]; d[rg] = d[r]; d[r] = t; cyc += 2; }
    else if (m == 0x148) { const uint32_t t = a[rg]; a[rg] = a[r]; a[r] = t; cyc += 2; }
    else if (m == 0x188) { const uint32_t t = d[rg]; d[rg] = a[r]; a[r] = t; cyc += 2; }
    else logicGroup(op, 1);
    break;
  }
  // ------------------------------------------------------------------ E ----
  case 0xE: {
    if (((op >> 6) & 3) == 3) {                  // memory shift/rotate, one bit, word
      Ea e; eaPrep(e, mode, r, 2);
      eaPut(e, 2, shiftOp((op >> 9) & 3, (op & 0x100) != 0, eaGet(e, 2), 1, 2));
    } else {
      const int sz = sizeBits((op >> 6) & 3);
      const uint32_t cnt = (op & 0x20) ? (d[rg] & 63) : (rg ? rg : 8);
      const uint32_t res = shiftOp((op >> 3) & 3, (op & 0x100) != 0, d[r], cnt, sz);
      eaPut(Ea{0, (uint8_t)r, 0}, sz, res);
      if (sz == 4) cyc += 2;
    }
    break;
  }
  default: excep(op >> 12 == 0xA ? 10 : 11, pc - 2); break;      // line-A / line-F
  }
}

static inline void checkInt() {
  const int lvl = (vintPend && (reg[1] & 0x20)) ? 6 : (hintPend && (reg[0] & 0x10)) ? 4 : 0;
  if (lvl > ipm) {
    stopped = false;
    if (lvl == 6) vintPend = false; else hintPend = false;
    excep(24 + lvl, pc);
    ipm = lvl; cyc += 24;
  }
}
static void runCpu(int32_t target) {
  while (cyc < target) {
    if (vintPend || hintPend) checkInt();
    if (stopped) { cyc = target; break; }
    step();
  }
}

// =============================================================== rendering ====
static inline uint16_t vr16(uint32_t ad) { ad &= 0xFFFE; return (vram[ad] << 8) | vram[ad + 1]; }
static const uint8_t sizeTab[4] = { 32, 64, 32, 128 };

static inline void tileRow(uint8_t *dst, uint16_t e, int row, int fx, int n) {
  if (e & 0x1000) row = 7 - row;
  const uint32_t ta = ((uint32_t)(e & 0x7FF) << 5) + (row << 2);
  const uint8_t pal = ((e >> 13) & 3) << 4, pri = (e & 0x8000) ? 0x40 : 0;
  for (int i = 0; i < n; i++) {
    int px = fx + i; if (e & 0x800) px = 7 - px;
    const uint8_t b = vram[(ta + (px >> 1)) & 0xFFFF];
    const uint8_t nb = (px & 1) ? (b & 15) : (b >> 4);
    dst[i] = nb ? (pal | nb | pri) : 0;
  }
}

static void renderPlane(uint8_t *dst, int plane, int y) {
  const int W = vdpW;
  const uint32_t base = plane ? ((reg[4] & 7) << 13) : ((reg[2] & 0x38) << 10);
  const int wc = sizeTab[reg[16] & 3], hc = sizeTab[(reg[16] >> 4) & 3];
  const uint32_t hb = (reg[13] & 0x3F) << 10; const int hm = reg[11] & 3;
  const uint32_t ha = hm == 2 ? hb + ((y & ~7) << 2) : hm == 3 ? hb + (y << 2) : hb;
  const int hs = vr16(ha + plane * 2) & 0x3FF;
  const bool vcol = reg[11] & 4;
  const int xmask = wc * 8 - 1, ymask = hc * 8 - 1;
  int xsrc = (0 - hs) & xmask;
  for (int x = 0; x < W;) {
    const int tile = xsrc >> 3, fx = xsrc & 7;
    int n = 8 - fx; if (x + n > W) n = W - x;
    int vs;
    if (vcol) { vs = vsram[(x >> 4) * 2 + plane]; const int lim = 16 - (x & 15); if (n > lim) n = lim; }
    else vs = vsram[plane];
    const int ys = (y + vs) & ymask;
    const uint16_t e = vr16(base + ((((ys >> 3) * wc) + tile) << 1));
    tileRow(dst + x, e, ys & 7, fx, n);
    x += n; xsrc = (xsrc + n) & xmask;
  }
}

static void renderWindow(int y) {
  const bool h40 = vdpW == 320;
  const int stride = h40 ? 64 : 32;
  const uint32_t wb = (reg[3] & (h40 ? 0x3C : 0x3E)) << 10;
  const int vlim = reg[18] & 31, hlim = (reg[17] & 31) * 2;
  const bool vin = (reg[18] & 0x80) ? ((y >> 3) >= vlim) : ((y >> 3) < vlim);
  const bool hright = reg[17] & 0x80;
  if (!vin && !hright && hlim == 0) return;
  for (int cx = 0; cx < vdpW / 8; cx++) {
    const bool hin = hright ? (cx >= hlim) : (cx < hlim);
    if (!(vin || hin)) continue;
    const uint16_t e = vr16(wb + ((((y >> 3) * stride) + cx) << 1));
    tileRow(lnA + cx * 8, e, y & 7, 0, 8);
  }
}

static void renderSprites(int y) {
  const int W = vdpW; const bool h40 = W == 320;
  memset(lnS, 0, W);
  const int maxSpr = h40 ? 80 : 64, maxLine = h40 ? 20 : 16;
  const uint32_t sat = (reg[5] & (h40 ? 0x7E : 0x7F)) << 9;
  int idx = 0, cnt = 0;
  for (int n = 0; n < maxSpr; n++) {
    const uint32_t ad = sat + idx * 8;
    const int sy = (vr16(ad) & 0x3FF) - 128;
    const uint16_t w1 = vr16(ad + 2);
    const int hs = ((w1 >> 10) & 3) + 1, vs = ((w1 >> 8) & 3) + 1;
    int row = y - sy;
    if (row >= 0 && row < vs * 8) {
      const uint16_t w2 = vr16(ad + 4);
      const int sx = (vr16(ad + 6) & 0x1FF) - 128;
      const bool hf = w2 & 0x800, vf = w2 & 0x1000;
      const uint8_t pal = ((w2 >> 13) & 3) << 4, pri = (w2 & 0x8000) ? 0x40 : 0;
      if (vf) row = vs * 8 - 1 - row;
      for (int c = 0; c < hs; c++) {
        const int cc2 = hf ? hs - 1 - c : c;
        const uint32_t ta = (((uint32_t)((w2 & 0x7FF) + cc2 * vs + (row >> 3))) << 5) + ((row & 7) << 2);
        for (int px = 0; px < 8; px++) {
          const int xx = sx + c * 8 + px;
          if (xx < 0 || xx >= W || (lnS[xx] & 15)) continue;
          const int fxp = hf ? 7 - px : px;
          const uint8_t b = vram[(ta + (fxp >> 1)) & 0xFFFF];
          const uint8_t nb = (fxp & 1) ? (b & 15) : (b >> 4);
          if (nb) lnS[xx] = pal | nb | pri;
        }
      }
      if (++cnt >= maxLine) break;
    }
    idx = vr16(ad + 2) & 0x7F;
    if (idx == 0) break;
  }
}

static inline uint16_t avg565(uint16_t x, uint16_t y) {
  return ((x & 0xF7DE) >> 1) + ((y & 0xF7DE) >> 1) + (x & y & 0x0821);
}

static void rebuildMaps() {
  mapW = vdpW; mapH = vdpH;
  for (int i = 0; i < 256; i++) lineOut[i] = -1;
  for (int r = 0; r < GAME_H; r++) {
    int y = (r * mapH + mapH / 2) / GAME_H; if (y >= mapH) y = mapH - 1;
    if (lineOut[y] < 0) lineOut[y] = r;
  }
  for (int x = 0; x < GAME_W; x++) xmap[x] = (x * mapW) / GAME_W;
}

static void renderLine(int y, int outRow) {
  const int W = vdpW;
  const uint8_t bd = reg[7] & 0x3F;
  if (!(reg[1] & 0x40)) memset(lnC, bd, W);
  else {
    renderPlane(lnB, 1, y);
    renderPlane(lnA, 0, y);
    renderWindow(y);
    renderSprites(y);
    for (int x = 0; x < W; x++) {
      const uint8_t b = lnB[x], p = lnA[x], s = lnS[x];
      uint8_t o = bd;
      if ((b & 15) && !(b & 64)) o = b & 63;
      if ((p & 15) && !(p & 64)) o = p & 63;
      if ((s & 15) && !(s & 64)) o = s & 63;
      if ((b & 15) &&  (b & 64)) o = b & 63;
      if ((p & 15) &&  (p & 64)) o = p & 63;
      if ((s & 15) &&  (s & 64)) o = s & 63;
      lnC[x] = o;
    }
  }
  lnC[W] = lnC[W - 1];
  uint16_t *dst = fb + outRow * GAME_W;
  for (int x = 0; x < GAME_W; x++) {
    const int sx = xmap[x];
    const uint16_t c = avg565(pal565[lnC[sx]], pal565[lnC[sx + 1]]);
    dst[x] = (c >> 8) | (c << 8);
  }
}

static IRAM_ATTR void blit() {
  SPI.beginTransaction(lcdCfg);
  lcdWindow(GAME_X, GAME_Y, GAME_X + GAME_W - 1, GAME_Y + GAME_H - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int r = 0; r < GAME_H; r++) {
    const uint16_t *row = fb + r * GAME_W;
    SPI.writeBytes((uint8_t *)row, GAME_W * 2);
    shotRow(r, row, GAME_W);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// ================================================================ lifecycle ===
static void freeAll() {
  if (romFile) romFile.close();
  if (blob)   { heap_caps_free(blob);   blob = nullptr; }
  if (pgData) { heap_caps_free(pgData); pgData = nullptr; }
  if (fb)     { heap_caps_free(fb);     fb = nullptr; }
  ram = vram = zram = nullptr;
  romOk = false; romReadError = false; romSize = 0; fetchPage = -1;
}

static void *gAlloc(size_t n, bool internalFirst) {
  void *p = nullptr;
  if (internalFirst) p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!p && psramFound()) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return p;
}

static void reset() {
  memset(psgReg, 0, sizeof(psgReg)); memset(ymReg, 0, sizeof(ymReg)); memset(ymAddr, 0, sizeof(ymAddr)); psgLatch = 0;
  memset(blob, 0, BLOB_SZ);
  memset(reg, 0, sizeof(reg)); memset(vsram, 0, sizeof(vsram));
  for (int i = 0; i < 64; i++) setPal(i, 0);
  reg[15] = 2;
  vAddr = 0; vCode = 0; vPending = dmaFill = false;
  vintPend = hintPend = vBlank = hBlank = vintFlag = false;
  curLine = 0; hcnt = 0; vdpW = 320; vdpH = 224; mapW = 0;
  ioData1 = ioData2 = ioData3 = 0; ioCtrl1 = ioCtrl2 = ioCtrl3 = 0;
  zBusReq = false; zReset = true;
  memset(d, 0, sizeof(d)); memset(a, 0, sizeof(a));
  fX = fN = fZ = fV = fC = 0; fT = 0; fS = 1; ipm = 7; stopped = false; spAlt = 0;
  cyc = cycTarget = lineBase = 0; fetchPage = -1;
  memset(pgTag, 0xFF, sizeof(pgTag)); memset(pgVict, 0, sizeof(pgVict));
  a[7] = rd32(0); pc = rd32(4);
  cyc = 0;
  if (fb) memset(fb, 0, GAME_W * GAME_H * 2);
}

static bool install(uint32_t sz) {
  romSize = sz;
  blob = (uint8_t *)gAlloc(BLOB_SZ, true);
  pgData = (uint8_t *)gAlloc(NSETS * 2 * PG_SZ, false);
  fb = (uint16_t *)gAlloc(GAME_W * GAME_H * 2, false);
  if (!blob || !pgData || !fb) {
    freeAll();
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "GENESIS", sizeof(loadErr2));
    return false;
  }
  ram = blob; vram = blob + RAM_SZ; zram = blob + RAM_SZ + VRAM_SZ;
  memset(pgTag, 0xFF, sizeof(pgTag)); memset(pgVict, 0, sizeof(pgVict));
  const uint32_t sp = rd32(0), start = rd32(4);
  if (romReadError || start >= romSize || (sp & 1) || (start & 1)) {
    freeAll();
    strlcpy(loadErr1, "BAD ROM", sizeof(loadErr1)); strlcpy(loadErr2, "NOT RAW GENESIS", sizeof(loadErr2));
    return false;
  }
  romOk = true;
  reset();
  return true;
}

static void frame(bool draw) {
  if (!romOk || !fb) return;
  vdpW = (reg[12] & 1) ? 320 : 256;
  vdpH = (reg[1] & 8) ? 240 : 224;
  if (mapW != vdpW || mapH != vdpH) rebuildMaps();
  for (int line = 0; line < LINES_TOTAL; line++) {
    curLine = line; lineBase = cycTarget; hBlank = false;
    if (line == 0) { vBlank = false; vintFlag = false; hcnt = reg[10]; }
    if (line < vdpH) {
      if (hcnt == 0) { hcnt = reg[10]; hintPend = true; } else hcnt--;
      const int r = lineOut[line];
      if (draw && r >= 0) renderLine(line, r);
    } else {
      if (line == vdpH) { vBlank = true; vintFlag = true; vintPend = true; hintPend = false; }
      hcnt = reg[10];
    }
    cycTarget = lineBase + 360;  runCpu(cycTarget);
    hBlank = true;
    cycTarget = lineBase + CYC_LINE; runCpu(cycTarget);
  }
  cyc -= cycTarget; cycTarget = 0;
  if (draw) blit();
}

// --------------------------------------------------------------- save state ---
struct GenState {
  uint32_t romSize;
  uint32_t d[8], a[8], pc, spAlt, fX, fN, fZ, fV, fC;
  uint8_t ipm, fS, fT, stopped;
  uint8_t reg[32]; uint16_t cram[64], vsram[64];
  uint16_t vAddr; uint8_t vCode, vPending, dmaFill, vintPend, hintPend, vBlank, vintFlag;
  int32_t hcnt;
  uint8_t ioData1, ioCtrl1, ioData2, ioCtrl2, ioData3, ioCtrl3, zBusReq, zReset;
};

static bool stateSave(uint8_t slot) {
  if (!romOk || !blob || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if (!sdBuildStatePath(currentRomPath, "genesis", path, sizeof(path), slot)) return false;
  GenState c{}; c.romSize = romSize;
  memcpy(c.d, d, sizeof(d)); memcpy(c.a, a, sizeof(a));
  c.pc = pc; c.spAlt = spAlt; c.fX = fX; c.fN = fN; c.fZ = fZ; c.fV = fV; c.fC = fC;
  c.ipm = ipm; c.fS = fS; c.fT = fT; c.stopped = stopped;
  memcpy(c.reg, reg, sizeof(reg)); memcpy(c.cram, cram, sizeof(cram)); memcpy(c.vsram, vsram, sizeof(vsram));
  c.vAddr = vAddr; c.vCode = vCode; c.vPending = vPending; c.dmaFill = dmaFill;
  c.vintPend = vintPend; c.hintPend = hintPend; c.vBlank = vBlank; c.vintFlag = vintFlag; c.hcnt = hcnt;
  c.ioData1 = ioData1; c.ioCtrl1 = ioCtrl1; c.ioData2 = ioData2; c.ioCtrl2 = ioCtrl2;
  c.ioData3 = ioData3; c.ioCtrl3 = ioCtrl3; c.zBusReq = zBusReq; c.zReset = zReset;
  const bool ok = sdStateWrite(path, SYS_GENESIS, &c, sizeof(c), blob, BLOB_SZ, nullptr, 0);
  Serial.printf("STATE GENESIS: %s (%s)\n", ok ? "saved" : "save failed", path);
  return ok;
}

static bool stateLoad(uint8_t slot) {
  if (!romOk || !blob || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if (!sdBuildStatePath(currentRomPath, "genesis", path, sizeof(path), slot)) return false;
  GenState c{};
  uint8_t *tmp = (uint8_t *)gAlloc(BLOB_SZ, false);       // validate fully before touching live state
  if (!tmp) return false;
  if (!sdStateRead(path, SYS_GENESIS, &c, sizeof(c), tmp, BLOB_SZ, nullptr, 0) || c.romSize != romSize) { heap_caps_free(tmp); return false; }
  memcpy(blob, tmp, BLOB_SZ); heap_caps_free(tmp);
  memcpy(d, c.d, sizeof(d)); memcpy(a, c.a, sizeof(a));
  pc = c.pc; spAlt = c.spAlt; fX = c.fX; fN = c.fN; fZ = c.fZ; fV = c.fV; fC = c.fC;
  ipm = c.ipm; fS = c.fS; fT = c.fT; stopped = c.stopped;
  memcpy(reg, c.reg, sizeof(reg)); memcpy(vsram, c.vsram, sizeof(vsram));
  for (int i = 0; i < 64; i++) setPal(i, c.cram[i]);
  vAddr = c.vAddr; vCode = c.vCode; vPending = c.vPending; dmaFill = c.dmaFill;
  vintPend = c.vintPend; hintPend = c.hintPend; vBlank = c.vBlank; vintFlag = c.vintFlag; hcnt = c.hcnt;
  ioData1 = c.ioData1; ioCtrl1 = c.ioCtrl1; ioData2 = c.ioData2; ioCtrl2 = c.ioCtrl2;
  ioData3 = c.ioData3; ioCtrl3 = c.ioCtrl3; zBusReq = c.zBusReq; zReset = c.zReset;
  cyc = cycTarget = lineBase = 0; fetchPage = -1; mapW = 0;
  return true;
}

}  // namespace gen

// ---- public wrappers used by core_manager.h ----------------------------------
static void genShutdown() { gen::freeAll(); }
static bool genRomReadFailed() { return gen::romReadError; }
static void genReset() { if (gen::romOk) gen::reset(); }
static void genFrame(bool draw) { gen::frame(draw); }

static bool genLoad(const char *path) {
  gen::freeAll();
  gen::romFile = SD_MMC.open(path, FILE_READ);
  if (!gen::romFile || gen::romFile.isDirectory()) {
    gen::freeAll();
    strlcpy(loadErr1, "ROM NOT FOUND", sizeof(loadErr1)); strlcpy(loadErr2, "CHECK SD CARD", sizeof(loadErr2));
    return false;
  }
  const size_t sz = gen::romFile.size();
  if (sz < 0x200 || sz > 0x400000) {
    gen::freeAll();
    strlcpy(loadErr1, "BAD ROM SIZE", sizeof(loadErr1)); strlcpy(loadErr2, "NEED 512B - 4MB", sizeof(loadErr2));
    return false;
  }
  Serial.printf("ROM: %s (%u bytes)\n", path, (unsigned)sz);
  return gen::install((uint32_t)sz);              // the file stays open: the ROM is read from the card as it runs
}

#endif // EMU32_GENESIS_CORE_H
