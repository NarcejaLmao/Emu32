// ============================================================================
// Emu32 Sega Master System core (basic).
//
//  * Zilog Z80 interpreter: full documented instruction set plus the common
//    undocumented behaviour (IXH/IXL, SLL, flag bits 3/5, DDCB register copy),
//    IM 0/1/2, NMI (the console PAUSE button = START), EI delay, T-state timing
//  * SMS VDP in Mode 4 (192/224/240 lines): background with per-line H scroll,
//    column V scroll, scroll locks, priority tiles, 8x8 / 8x16 sprites,
//    zoomed sprites, 8-sprite line limit with overflow + collision flags
//    (latched on every active line, also on skipped frames / undrawn lines),
//    left-column mask, CRAM, line (H) interrupt and frame (V) interrupt
//  * Sega mapper ($FFFC-$FFFF, 32 KB cart RAM) and the Codemasters mapper
//    (auto-detected from the ROM header checksum), up to 4 MB
//  * Joypad 1 (D-pad + buttons 1/2), TH/TR I/O control port, V/H counters
//  * SN76489 PSG -> AudioEv::psgWr() when the sketch has audio (S3 1.54)
//  * ROM is loaded fully into PSRAM (SMS carts are <= 1 MB in practice)
//
//  NOT emulated: TMS9918 legacy modes (SG-1000 / Mode 2 titles), Game Gear,
//  FM sound unit, light gun / paddle / port 2 pad, PAL timing (60 Hz NTSC),
//  BIOS (the cartridge boots directly), battery SRAM files (save states
//  include cart RAM instead).
//
//  Controls: pad A = button 1, pad B = button 2, START = PAUSE.
//
//  Needs (from the sketch): lcdStreamBegin/Write/End, padState, loadErr1-2,
//  sdDiagReadAt, sdMount, sdBuildStatePath, sdStateWrite/Read,
//  currentRomPath, shotRow, GAME_X/Y/W/H, SYS_SMS.
//  Public: smsLoad(path) smsReset() smsFrame(draw) smsShutdown()
//          smsRomReadFailed() sms::stateSave(slot) sms::stateLoad(slot)
// ============================================================================
#ifndef EMU32_SMS_CORE_H
#define EMU32_SMS_CORE_H
#include <esp_heap_caps.h>

#if defined(__has_include)
#  if __has_include("audio_events.h")
#    define EMU32_SMS_AUDIO 1
#  endif
#endif
#ifndef EMU32_SMS_AUDIO
#  define EMU32_SMS_AUDIO 0
#endif

namespace sms {

static const int CYC_LINE = 228, LINES_TOTAL = 262;
static const uint32_t RAM_SZ = 8192, VRAM_SZ = 16384, CRAMX_SZ = 32768;
static const uint32_t BLOB_SZ = RAM_SZ + VRAM_SZ + CRAMX_SZ;

static uint8_t  *blob = nullptr, *ram = nullptr, *vram = nullptr, *cartRam = nullptr;
static uint8_t  *rom = nullptr;
static uint32_t  romSize = 0, romBanks = 1;
static uint16_t *fb = nullptr;                    // GAME_W x GAME_H, byte-swapped RGB565 (wire order)
static bool      romOk = false, cm = false;      // cm = Codemasters mapper
static bool      romReadError = false;

// ------------------------------------------------------------------ mapper --
static uint8_t  mapReg[4];                        // Sega: $FFFC..$FFFF ; Codemasters: [1..3] = bank of slot 0..2
static int32_t  offLo, off0, off1, off2;          // ROM index = off + address
static bool     ramSlot2;
static uint8_t  ramPage;

static inline uint32_t bankOf(uint8_t v) { return (uint32_t)v % romBanks; }
static void applyMap() {
  off0 = (int32_t)(bankOf(mapReg[1]) * 0x4000u);
  off1 = (int32_t)(bankOf(mapReg[2]) * 0x4000u) - 0x4000;
  off2 = (int32_t)(bankOf(mapReg[3]) * 0x4000u) - 0x8000;
  if (cm) { offLo = off0; ramSlot2 = false; ramPage = 0; }
  else    { offLo = 0; ramSlot2 = (mapReg[0] & 8) != 0; ramPage = (mapReg[0] >> 2) & 1; }
}

// ------------------------------------------------------------------- Z80 ----
union Pair { uint16_t w; struct { uint8_t l, h; } b; };
static Pair AF, BC, DE, HL, AF2, BC2, DE2, HL2, IX, IY;
static uint16_t SP, PC;
static uint8_t  Ireg, Rreg, IM;
static bool     IFF1, IFF2, halted, eiDelay, nmiPend, startPrev;
static int32_t  cyc = 0, lineBase = 0;

#define Z_A (AF.b.h)
#define Z_F (AF.b.l)
#define Z_B (BC.b.h)
#define Z_C (BC.b.l)
#define Z_D (DE.b.h)
#define Z_E (DE.b.l)
#define Z_H (HL.b.h)
#define Z_L (HL.b.l)

enum : uint8_t { ZF_S = 0x80, ZF_Z = 0x40, ZF_Y = 0x20, ZF_H = 0x10, ZF_X = 0x08, ZF_P = 0x04, ZF_N = 0x02, ZF_C = 0x01 };
static uint8_t parT[256];

// ------------------------------------------------------------------- VDP ----
static uint8_t  reg[16];
static uint8_t  cramRaw[32];
static uint16_t pal[32];                          // host order RGB565
static uint16_t vAddr;
static uint8_t  vCode, readBuf, ctrlLatch, ctrlByte, status;
static bool     hintPend;
static int      lcnt, curLine, vdpH = 192;
static uint8_t  ioCtl, memCtl;
static uint32_t expTab[256];
static int16_t  lineOut[256];
static uint16_t xmap[GAME_W];
static int      mapH = 0;
static uint8_t  lnBg[256], lnSp[256], lnOut[257];

static inline void setPal(int i, uint8_t v) {
  cramRaw[i] = v & 0x3F;
  const uint8_t r = (v & 3) * 85, g = ((v >> 2) & 3) * 85, b = ((v >> 4) & 3) * 85;
  pal[i] = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static inline bool irqLine() {
  return ((status & 0x80) && (reg[1] & 0x20)) || (hintPend && (reg[0] & 0x10));
}

static void psgOut(uint8_t v) {
#if EMU32_SMS_AUDIO
  AudioEv::psgWr(v);
#else
  (void)v;
#endif
}

static void vdpCtrlWrite(uint8_t v) {
  if (!ctrlLatch) { ctrlLatch = 1; ctrlByte = v; vAddr = (vAddr & 0x3F00) | v; return; }
  ctrlLatch = 0;
  vCode = v >> 6;
  vAddr = (uint16_t)(ctrlByte | ((v & 0x3F) << 8));
  if (vCode == 0) { readBuf = vram[vAddr]; vAddr = (vAddr + 1) & 0x3FFF; }
  else if (vCode == 2) { const uint8_t r = v & 0x0F; if (r < 11) reg[r] = ctrlByte; }
}
static void vdpDataWrite(uint8_t v) {
  ctrlLatch = 0;
  if (vCode == 3) setPal(vAddr & 31, v); else vram[vAddr] = v;
  readBuf = v;
  vAddr = (vAddr + 1) & 0x3FFF;
}
static uint8_t vdpDataRead() {
  ctrlLatch = 0;
  const uint8_t r = readBuf;
  readBuf = vram[vAddr];
  vAddr = (vAddr + 1) & 0x3FFF;
  return r;
}
static uint8_t vdpStatusRead() {
  ctrlLatch = 0;
  const uint8_t r = status;
  status &= 0x1F; hintPend = false;
  return r;
}

static uint8_t vCounter() {
  int v = curLine;
  if (vdpH == 192) { if (v > 0xDA) v -= 6; }
  else if (vdpH == 224) { if (v > 0xEA) v -= 6; }
  return (uint8_t)v;
}

static uint8_t joy1() {
  uint8_t v = 0xFF;
  if (padState & 0x10) v &= ~0x01;
  if (padState & 0x20) v &= ~0x02;
  if (padState & 0x40) v &= ~0x04;
  if (padState & 0x80) v &= ~0x08;
  if (padState & 0x01) v &= ~0x10;
  if (padState & 0x02) v &= ~0x20;
  if (!(ioCtl & 0x01)) { if (ioCtl & 0x10) v |= 0x20; else v &= ~0x20; }   // TR as output
  return v;
}
static uint8_t joy2() {
  uint8_t v = 0xFF;
  v = (uint8_t)((v & ~0x40) | ((ioCtl & 0x02) ? 0x40 : ((ioCtl & 0x20) ? 0x40 : 0)));   // P1 TH
  v = (uint8_t)((v & ~0x80) | ((ioCtl & 0x08) ? 0x80 : ((ioCtl & 0x80) ? 0x80 : 0)));   // P2 TH
  return v;
}

static uint8_t portIn(uint8_t p) {
  switch (p & 0xC1) {
    case 0x40: return vCounter();
    case 0x41: return (uint8_t)(((cyc - lineBase) * 170) / CYC_LINE);
    case 0x80: return vdpDataRead();
    case 0x81: return vdpStatusRead();
    case 0xC0: return joy1();
    case 0xC1: return joy2();
    default:   return 0xFF;
  }
}
static void portOut(uint8_t p, uint8_t v) {
  switch (p & 0xC1) {
    case 0x00: memCtl = v; break;
    case 0x01: ioCtl = v; break;
    case 0x40: case 0x41: psgOut(v); break;
    case 0x80: vdpDataWrite(v); break;
    case 0x81: vdpCtrlWrite(v); break;
    default: break;
  }
}

// -------------------------------------------------------------- memory bus --
static inline uint8_t rd(uint16_t a) {
  if (a < 0xC000) {
    if (a < 0x400)  return rom[offLo + a];
    if (a < 0x4000) return rom[off0 + a];
    if (a < 0x8000) return rom[off1 + a];
    if (ramSlot2)   return cartRam[ramPage * 0x4000 + (a & 0x3FFF)];
    return rom[off2 + a];
  }
  return ram[a & 0x1FFF];
}
static inline void wr(uint16_t a, uint8_t v) {
  if (a >= 0xC000) {
    ram[a & 0x1FFF] = v;
    if (!cm && a >= 0xFFFC) { mapReg[a - 0xFFFC] = v; applyMap(); }
    return;
  }
  if (cm) {
    if (a == 0x0000) { mapReg[1] = v; applyMap(); }
    else if (a == 0x4000) { mapReg[2] = v; applyMap(); }
    else if (a == 0x8000) { mapReg[3] = v; applyMap(); }
    return;
  }
  if (a >= 0x8000 && ramSlot2) cartRam[ramPage * 0x4000 + (a & 0x3FFF)] = v;
}
static inline uint16_t rd16(uint16_t a) { const uint16_t l = rd(a); return l | ((uint16_t)rd((uint16_t)(a + 1)) << 8); }
static inline void wr16(uint16_t a, uint16_t v) { wr(a, (uint8_t)v); wr((uint16_t)(a + 1), (uint8_t)(v >> 8)); }
static inline uint8_t  fetch8()  { return rd(PC++); }
static inline uint16_t fetch16() { const uint16_t l = rd(PC++); const uint16_t h = rd(PC++); return l | (h << 8); }
static inline uint8_t  fetchOp() { Rreg = (Rreg & 0x80) | ((Rreg + 1) & 0x7F); return rd(PC++); }
static inline void push(uint16_t v) { SP -= 2; wr16(SP, v); }
static inline uint16_t pop() { const uint16_t v = rd16(SP); SP += 2; return v; }

// ------------------------------------------------------------------- ALU ----
static inline uint8_t szp(uint8_t r) { return (r & 0xA8) | (r ? 0 : ZF_Z) | parT[r]; }

static void aluOp(int op, uint8_t v) {
  const uint8_t a = Z_A;
  const unsigned c = Z_F & ZF_C;
  unsigned r;
  switch (op) {
    case 0: case 1:
      r = a + v + (op ? c : 0);
      Z_F = (r & 0xA8) | ((r & 0xFF) ? 0 : ZF_Z) | ((a ^ v ^ r) & ZF_H) | ((~(a ^ v) & (a ^ r) & 0x80) ? ZF_P : 0) | ((r >> 8) & 1);
      Z_A = (uint8_t)r; break;
    case 2: case 3: case 7:
      r = a - v - (op == 3 ? c : 0);
      Z_F = (r & 0x80) | ((r & 0xFF) ? 0 : ZF_Z) | ((a ^ v ^ r) & ZF_H) | (((a ^ v) & (a ^ r) & 0x80) ? ZF_P : 0) | ZF_N | ((r >> 8) & 1);
      if (op == 7) Z_F |= (v & 0x28); else { Z_F |= (r & 0x28); Z_A = (uint8_t)r; }
      break;
    case 4: Z_A = a & v; Z_F = szp(Z_A) | ZF_H; break;
    case 5: Z_A = a ^ v; Z_F = szp(Z_A); break;
    default: Z_A = a | v; Z_F = szp(Z_A); break;
  }
}
static inline uint8_t inc8(uint8_t v) { const uint8_t r = v + 1; Z_F = (Z_F & ZF_C) | (r & 0xA8) | (r ? 0 : ZF_Z) | (((r & 0xF) == 0) ? ZF_H : 0) | (r == 0x80 ? ZF_P : 0); return r; }
static inline uint8_t dec8(uint8_t v) { const uint8_t r = v - 1; Z_F = (Z_F & ZF_C) | (r & 0xA8) | (r ? 0 : ZF_Z) | (((r & 0xF) == 0xF) ? ZF_H : 0) | (r == 0x7F ? ZF_P : 0) | ZF_N; return r; }

static uint16_t add16(uint16_t a, uint16_t b) {
  const uint32_t r = (uint32_t)a + b;
  Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | ((r >> 8) & 0x28) | (((a ^ b ^ r) >> 8) & ZF_H) | ((r >> 16) & 1);
  return (uint16_t)r;
}
static void adc16(uint16_t b) {
  const uint16_t a = HL.w; const uint32_t r = (uint32_t)a + b + (Z_F & ZF_C);
  Z_F = ((r >> 8) & 0xA8) | ((r & 0xFFFF) ? 0 : ZF_Z) | (((a ^ b ^ r) >> 8) & ZF_H) | ((~(a ^ b) & (a ^ r) & 0x8000) ? ZF_P : 0) | ((r >> 16) & 1);
  HL.w = (uint16_t)r;
}
static void sbc16(uint16_t b) {
  const uint16_t a = HL.w; const uint32_t r = (uint32_t)a - b - (Z_F & ZF_C);
  Z_F = ((r >> 8) & 0xA8) | ((r & 0xFFFF) ? 0 : ZF_Z) | (((a ^ b ^ r) >> 8) & ZF_H) | (((a ^ b) & (a ^ r) & 0x8000) ? ZF_P : 0) | ZF_N | ((r >> 16) & 1);
  HL.w = (uint16_t)r;
}

static uint8_t rotOp(int y, uint8_t v) {
  uint8_t c, r;
  switch (y) {
    case 0: c = v >> 7; r = (v << 1) | c; break;
    case 1: c = v & 1;  r = (v >> 1) | (c << 7); break;
    case 2: c = v >> 7; r = (v << 1) | (Z_F & ZF_C); break;
    case 3: c = v & 1;  r = (v >> 1) | ((Z_F & ZF_C) << 7); break;
    case 4: c = v >> 7; r = v << 1; break;
    case 5: c = v & 1;  r = (v >> 1) | (v & 0x80); break;
    case 6: c = v >> 7; r = (v << 1) | 1; break;
    default: c = v & 1; r = v >> 1; break;
  }
  Z_F = szp(r) | c;
  return r;
}

static void daa() {
  const uint8_t a = Z_A; uint8_t corr = 0; bool c = Z_F & ZF_C;
  if ((Z_F & ZF_H) || (a & 0xF) > 9) corr |= 0x06;
  if (c || a > 0x99) { corr |= 0x60; c = true; }
  uint8_t r; bool h;
  if (Z_F & ZF_N) { r = a - corr; h = (Z_F & ZF_H) && (a & 0xF) < 6; }
  else          { r = a + corr; h = (a & 0xF) > 9; }
  Z_F = (Z_F & ZF_N) | szp(r) | (h ? ZF_H : 0) | (c ? ZF_C : 0);
  Z_A = r;
}

static inline bool cond(int c) {
  switch (c) {
    case 0: return !(Z_F & ZF_Z); case 1: return (Z_F & ZF_Z) != 0;
    case 2: return !(Z_F & ZF_C); case 3: return (Z_F & ZF_C) != 0;
    case 4: return !(Z_F & ZF_P); case 5: return (Z_F & ZF_P) != 0;
    case 6: return !(Z_F & ZF_S); default: return (Z_F & ZF_S) != 0;
  }
}

static uint8_t *const r8base[8] = { &BC.b.h, &BC.b.l, &DE.b.h, &DE.b.l, &HL.b.h, &HL.b.l, nullptr, &AF.b.h };
static uint8_t *const r8ix[8]   = { &BC.b.h, &BC.b.l, &DE.b.h, &DE.b.l, &IX.b.h, &IX.b.l, nullptr, &AF.b.h };
static uint8_t *const r8iy[8]   = { &BC.b.h, &BC.b.l, &DE.b.h, &DE.b.l, &IY.b.h, &IY.b.l, nullptr, &AF.b.h };

static int execCB(uint8_t cb, uint16_t addr, bool indexed) {
  const int x = cb >> 6, y = (cb >> 3) & 7, z = cb & 7;
  const bool mem = indexed || z == 6;
  const uint8_t v = mem ? rd(addr) : *r8base[z];
  uint8_t res = v;
  switch (x) {
    case 0: res = rotOp(y, v); break;
    case 1: { const uint8_t m = v & (1 << y); Z_F = (Z_F & ZF_C) | ZF_H | (m ? (m & 0x80) : (ZF_Z | ZF_P)) | (v & 0x28); return indexed ? 16 : (mem ? 12 : 8); }
    case 2: res = v & ~(1 << y); break;
    default: res = v | (1 << y); break;
  }
  if (mem) { wr(addr, res); if (indexed && z != 6) *r8base[z] = res; }
  else *r8base[z] = res;
  return indexed ? 19 : (mem ? 15 : 8);
}

static int execED(uint8_t op) {
  const int x = op >> 6, y = (op >> 3) & 7, z = op & 7, p = y >> 1, q = y & 1;
  uint16_t *rp[4] = { &BC.w, &DE.w, &HL.w, &SP };
  if (x == 1) {
    switch (z) {
      case 0: { const uint8_t v = portIn(Z_C); if (y != 6) *r8base[y] = v; Z_F = (Z_F & ZF_C) | szp(v); return 12; }
      case 1: portOut(Z_C, y == 6 ? 0 : *r8base[y]); return 12;
      case 2: if (q) adc16(*rp[p]); else sbc16(*rp[p]); return 15;
      case 3: { const uint16_t nn = fetch16(); if (q) *rp[p] = rd16(nn); else wr16(nn, *rp[p]); return 20; }
      case 4: { const uint8_t a = Z_A; Z_A = 0; aluOp(2, a); return 8; }
      case 5: PC = pop(); IFF1 = IFF2; return 14;
      case 6: IM = (y == 2 || y == 6) ? 1 : (y == 3 || y == 7) ? 2 : 0; return 8;
      default:
        switch (y) {
          case 0: Ireg = Z_A; return 9;
          case 1: Rreg = Z_A; return 9;
          case 2: Z_A = Ireg; Z_F = (Z_F & ZF_C) | (szp(Z_A) & ~ZF_P) | (IFF2 ? ZF_P : 0); return 9;
          case 3: Z_A = Rreg; Z_F = (Z_F & ZF_C) | (szp(Z_A) & ~ZF_P) | (IFF2 ? ZF_P : 0); return 9;
          case 4: { const uint8_t m = rd(HL.w); wr(HL.w, (m >> 4) | (Z_A << 4)); Z_A = (Z_A & 0xF0) | (m & 0x0F); Z_F = (Z_F & ZF_C) | szp(Z_A); return 18; }
          case 5: { const uint8_t m = rd(HL.w); wr(HL.w, (m << 4) | (Z_A & 0x0F)); Z_A = (Z_A & 0xF0) | (m >> 4); Z_F = (Z_F & ZF_C) | szp(Z_A); return 18; }
          default: return 8;
        }
    }
  }
  if (x == 2 && z < 4 && y >= 4) {
    const bool dec = (y & 1) != 0, rep = y >= 6;
    const int16_t d = dec ? -1 : 1;
    bool again = false;
    switch (z) {
      case 0: {                                             // LDI/LDD/LDIR/LDDR
        const uint8_t v = rd(HL.w); wr(DE.w, v);
        HL.w += d; DE.w += d; BC.w--;
        const uint8_t n = v + Z_A;
        Z_F = (Z_F & (ZF_S | ZF_Z | ZF_C)) | ((n & 2) ? ZF_Y : 0) | (n & ZF_X) | (BC.w ? ZF_P : 0);
        again = rep && BC.w;
      } break;
      case 1: {                                             // CPI/CPD/CPIR/CPDR
        const uint8_t v = rd(HL.w); const uint8_t r = Z_A - v;
        const uint8_t h = (Z_A ^ v ^ r) & ZF_H;
        HL.w += d; BC.w--;
        const uint8_t n = r - (h ? 1 : 0);
        Z_F = (Z_F & ZF_C) | ZF_N | (r & 0x80) | (r ? 0 : ZF_Z) | h | ((n & 2) ? ZF_Y : 0) | (n & ZF_X) | (BC.w ? ZF_P : 0);
        again = rep && BC.w && r;
      } break;
      case 2: {                                             // INI/IND/INIR/INDR
        const uint8_t v = portIn(Z_C); wr(HL.w, v); HL.w += d; Z_B--;
        Z_F = (Z_B & 0xA8) | (Z_B ? 0 : ZF_Z) | ZF_N | (Z_F & ZF_C);
        again = rep && Z_B;
      } break;
      default: {                                            // OUTI/OUTD/OTIR/OTDR
        const uint8_t v = rd(HL.w); Z_B--; portOut(Z_C, v); HL.w += d;
        Z_F = (Z_B & 0xA8) | (Z_B ? 0 : ZF_Z) | ZF_N | (Z_F & ZF_C);
        again = rep && Z_B;
      } break;
    }
    if (again) { PC -= 2; return 21; }
    return 16;
  }
  return 8;                                                  // undefined ED opcode = NOP
}

static int step() {
  if (halted) return 4;
  uint8_t op = fetchOp();
  Pair *xr = &HL; uint8_t *const *rt = r8base; int t = 0; bool pf = false;
  while (op == 0xDD || op == 0xFD) {
    if (op == 0xDD) { xr = &IX; rt = r8ix; } else { xr = &IY; rt = r8iy; }
    pf = true; t += 4; op = fetchOp();
  }
  const int x = op >> 6, y = (op >> 3) & 7, z = op & 7, p = y >> 1, q = y & 1;
  uint16_t *rp16[4] = { &BC.w, &DE.w, &xr->w, &SP };
  uint16_t *rp2[4]  = { &BC.w, &DE.w, &xr->w, &AF.w };
  auto ea = [&]() -> uint16_t { if (!pf) return HL.w; t += 8; return (uint16_t)(xr->w + (int8_t)fetch8()); };

  switch (x) {
    case 0:
      switch (z) {
        case 0:
          switch (y) {
            case 0: return t + 4;
            case 1: { const Pair tmp = AF; AF = AF2; AF2 = tmp; return t + 4; }
            case 2: { const int8_t d = (int8_t)fetch8(); if (--Z_B) { PC += d; return t + 13; } return t + 8; }
            case 3: { const int8_t d = (int8_t)fetch8(); PC += d; return t + 12; }
            default: { const int8_t d = (int8_t)fetch8(); if (cond(y - 4)) { PC += d; return t + 12; } return t + 7; }
          }
        case 1:
          if (!q) { *rp16[p] = fetch16(); return t + 10; }
          xr->w = add16(xr->w, *rp16[p]); return t + 11;
        case 2:
          switch (y) {
            case 0: wr(BC.w, Z_A); return t + 7;
            case 1: Z_A = rd(BC.w); return t + 7;
            case 2: wr(DE.w, Z_A); return t + 7;
            case 3: Z_A = rd(DE.w); return t + 7;
            case 4: wr16(fetch16(), xr->w); return t + 16;
            case 5: xr->w = rd16(fetch16()); return t + 16;
            case 6: wr(fetch16(), Z_A); return t + 13;
            default: Z_A = rd(fetch16()); return t + 13;
          }
        case 3: if (q) (*rp16[p])--; else (*rp16[p])++; return t + 6;
        case 4:
          if (y == 6) { const uint16_t a = ea(); wr(a, inc8(rd(a))); return t + 11; }
          *rt[y] = inc8(*rt[y]); return t + 4;
        case 5:
          if (y == 6) { const uint16_t a = ea(); wr(a, dec8(rd(a))); return t + 11; }
          *rt[y] = dec8(*rt[y]); return t + 4;
        case 6:
          if (y == 6) { const uint16_t a = ea(); wr(a, fetch8()); return t + (pf ? 7 : 10); }
          *rt[y] = fetch8(); return t + 7;
        default:
          switch (y) {
            case 0: { const uint8_t c = Z_A >> 7; Z_A = (Z_A << 1) | c; Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | c; return t + 4; }
            case 1: { const uint8_t c = Z_A & 1; Z_A = (Z_A >> 1) | (c << 7); Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | c; return t + 4; }
            case 2: { const uint8_t c = Z_A >> 7; Z_A = (Z_A << 1) | (Z_F & ZF_C); Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | c; return t + 4; }
            case 3: { const uint8_t c = Z_A & 1; Z_A = (Z_A >> 1) | ((Z_F & ZF_C) << 7); Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | c; return t + 4; }
            case 4: daa(); return t + 4;
            case 5: Z_A = ~Z_A; Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P | ZF_C)) | (Z_A & 0x28) | ZF_H | ZF_N; return t + 4;
            case 6: Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | ZF_C; return t + 4;
            default: Z_F = (Z_F & (ZF_S | ZF_Z | ZF_P)) | (Z_A & 0x28) | ((Z_F & ZF_C) ? ZF_H : ZF_C); return t + 4;
          }
      }
    case 1:
      if (op == 0x76) { halted = true; return t + 4; }
      if (y == 6) { const uint16_t a = ea(); wr(a, *r8base[z]); return t + 7; }
      if (z == 6) { const uint16_t a = ea(); *r8base[y] = rd(a); return t + 7; }
      *rt[y] = *rt[z]; return t + 4;
    case 2:
      if (z == 6) { const uint16_t a = ea(); aluOp(y, rd(a)); return t + 7; }
      aluOp(y, *rt[z]); return t + 4;
    default:
      switch (z) {
        case 0: if (cond(y)) { PC = pop(); return t + 11; } return t + 5;
        case 1:
          if (!q) { *rp2[p] = pop(); return t + 10; }
          switch (p) {
            case 0: PC = pop(); return t + 10;
            case 1: { Pair tmp; tmp = BC; BC = BC2; BC2 = tmp; tmp = DE; DE = DE2; DE2 = tmp; tmp = HL; HL = HL2; HL2 = tmp; return t + 4; }
            case 2: PC = xr->w; return t + 4;
            default: SP = xr->w; return t + 6;
          }
        case 2: { const uint16_t nn = fetch16(); if (cond(y)) PC = nn; return t + 10; }
        case 3:
          switch (y) {
            case 0: PC = fetch16(); return t + 10;
            case 1:
              if (pf) { const uint16_t a = (uint16_t)(xr->w + (int8_t)fetch8()); const uint8_t cb = fetch8(); return t + execCB(cb, a, true); }
              return execCB(fetchOp(), HL.w, false);
            case 2: { const uint8_t n = fetch8(); portOut(n, Z_A); return t + 11; }
            case 3: { const uint8_t n = fetch8(); Z_A = portIn(n); return t + 11; }
            case 4: { const uint16_t v = rd16(SP); wr16(SP, xr->w); xr->w = v; return t + 19; }
            case 5: { const uint16_t v = DE.w; DE.w = HL.w; HL.w = v; return t + 4; }
            case 6: IFF1 = IFF2 = false; return t + 4;
            default: IFF1 = IFF2 = true; eiDelay = true; return t + 4;
          }
        case 4: { const uint16_t nn = fetch16(); if (cond(y)) { push(PC); PC = nn; return t + 17; } return t + 10; }
        case 5:
          if (!q) { push(*rp2[p]); return t + 11; }
          if (p == 0) { const uint16_t nn = fetch16(); push(PC); PC = nn; return t + 17; }
          return execED(fetchOp());                            // p == 2 (ED); DD / FD were consumed above
        case 6: aluOp(y, fetch8()); return t + 7;
        default: push(PC); PC = y * 8; return t + 11;
      }
  }
  return t + 4;
}

static void runCpu(int32_t target) {
  while (cyc < target) {
    const bool canInt = !eiDelay; eiDelay = false;
    if (nmiPend) { nmiPend = false; halted = false; IFF2 = IFF1; IFF1 = false; push(PC); PC = 0x66; cyc += 11; continue; }
    if (canInt && IFF1 && irqLine()) {
      halted = false; IFF1 = IFF2 = false; push(PC);
      if (IM == 2) { PC = rd16((uint16_t)((Ireg << 8) | 0xFF)); cyc += 19; } else { PC = 0x38; cyc += 13; }
      continue;
    }
    cyc += step();
  }
}

// ----------------------------------------------------------------- render ---
static void rebuildMaps() {
  mapH = vdpH;
  for (int i = 0; i < 256; i++) lineOut[i] = -1;
  for (int r = 0; r < GAME_H; r++) {
    int y = (r * mapH + mapH / 2) / GAME_H; if (y >= mapH) y = mapH - 1;
    if (lineOut[y] < 0) lineOut[y] = r;
  }
  for (int x = 0; x < GAME_W; x++) xmap[x] = (uint16_t)((x * 256) / GAME_W);
}

static void renderBg(int y) {
  const uint16_t nameBase = (vdpH == 192) ? ((reg[2] & 0x0E) << 10) : (((reg[2] & 0x0C) << 10) | 0x0700);
  const int hs = ((reg[0] & 0x40) && y < 16) ? 0 : reg[8];
  const int wrapH = (vdpH == 192) ? 224 : 256;
  for (int n = 0; n < 32; n++) {
    const int sx = (n * 8 + hs) & 255;
    int vs = reg[9];
    if ((reg[0] & 0x80) && (sx >> 3) >= 24) vs = 0;
    const int by = (y + vs) % wrapH;
    const uint8_t *e = vram + ((nameBase + (((by >> 3) << 5) + n) * 2) & 0x3FFF);
    const uint16_t ent = e[0] | (e[1] << 8);
    int fy = by & 7; if (ent & 0x400) fy = 7 - fy;
    const uint8_t *tp = vram + (((ent & 0x1FF) * 32 + fy * 4) & 0x3FFF);
    const uint32_t px = expTab[tp[0]] | (expTab[tp[1]] << 1) | (expTab[tp[2]] << 2) | (expTab[tp[3]] << 3);
    const uint8_t base = (ent & 0x800) ? 16 : 0, prio = (ent & 0x1000) ? 0x40 : 0;
    for (int i = 0; i < 8; i++) {
      const int s = (ent & 0x200) ? 7 - i : i;
      const uint8_t c = (px >> (4 * s)) & 15;
      lnBg[(sx + i) & 255] = base + c + (c ? prio : 0);
    }
  }
}

// paint = false: latch only the status flags (overflow $40 / collision $20) for a line
// that is not being drawn, so games polling them behave the same with frameskip or
// when the 192-line picture is scaled down to GAME_H rows.
static bool renderSprites(int y, bool paint = true) {
  const uint8_t *sat = vram + ((reg[5] & 0x7E) << 7);
  const bool tall = (reg[1] & 2) != 0; const int zoom = reg[1] & 1;
  const int h = (tall ? 16 : 8) << zoom;
  int hits[8]; int n = 0;
  for (int i = 0; i < 64; i++) {
    const int ry = sat[i];
    if (vdpH == 192 && ry == 0xD0) break;
    const int top = (ry > 0xF0) ? ry - 255 : ry + 1;
    const int dy = y - top;
    if (dy < 0 || dy >= h) continue;
    if (n == 8) { status |= 0x40; break; }
    hits[n++] = i;
  }
  if (!n || (!paint && n < 2)) return false;      // a collision needs two sprites on the line
  memset(lnSp, 0, 256);
  const int shift = (reg[0] & 8) ? 8 : 0;
  const uint16_t pbase = (reg[6] & 4) ? 0x2000 : 0;
  for (int k = 0; k < n; k++) {
    const int i = hits[k];
    const int ry = sat[i];
    const int top = (ry > 0xF0) ? ry - 255 : ry + 1;
    const int x0 = (int)sat[0x80 + i * 2] - shift;
    int tile = sat[0x81 + i * 2];
    int row = (y - top) >> zoom;
    if (tall) { tile &= 0xFE; if (row >= 8) { tile |= 1; row -= 8; } }
    const uint8_t *tp = vram + ((pbase + tile * 32 + row * 4) & 0x3FFF);
    const uint32_t px = expTab[tp[0]] | (expTab[tp[1]] << 1) | (expTab[tp[2]] << 2) | (expTab[tp[3]] << 3);
    for (int j = 0; j < 8; j++) {
      const uint8_t c = (px >> (4 * j)) & 15;
      if (!c) continue;
      for (int zz = 0; zz <= zoom; zz++) {
        const int sx = x0 + (j << zoom) + zz;
        if ((unsigned)sx > 255) continue;
        if (lnSp[sx]) status |= 0x20; else lnSp[sx] = 16 + c;
      }
    }
  }
  return true;
}

static inline uint16_t avg565(uint16_t x, uint16_t y) {
  return ((x & 0xF7DE) >> 1) + ((y & 0xF7DE) >> 1) + (x & y & 0x0821);
}

static void renderLine(int y, int outRow) {
  const uint8_t bd = 16 + (reg[7] & 15);
  if (!(reg[1] & 0x40)) memset(lnOut, bd, 256);
  else {
    renderBg(y);
    const bool spr = renderSprites(y);
    if (spr) { for (int x = 0; x < 256; x++) { const uint8_t b = lnBg[x], s = lnSp[x]; lnOut[x] = (s && !(b & 0x40)) ? s : (b & 0x1F); } }
    else     { for (int x = 0; x < 256; x++) lnOut[x] = lnBg[x] & 0x1F; }
    if (reg[0] & 0x20) memset(lnOut, bd, 8);
  }
  lnOut[256] = lnOut[255];
  uint16_t *dst = fb + outRow * GAME_W;
  for (int x = 0; x < GAME_W; x++) {
    const int sx = xmap[x];
    const uint16_t c = avg565(pal[lnOut[sx]], pal[lnOut[sx + 1]]);
    dst[x] = (c >> 8) | (c << 8);
  }
}

static IRAM_ATTR void blit() {
  lcdStreamBegin(GAME_X, GAME_Y, GAME_X + GAME_W - 1, GAME_Y + GAME_H - 1);
  for (int r = 0; r < GAME_H; r++) {
    const uint16_t *row = fb + r * GAME_W;
    lcdStreamWrite((uint8_t *)row, GAME_W * 2);
    shotRow(r, row, GAME_W);
  }
  lcdStreamEnd();
}

// ================================================================ lifecycle ===
static void initTables() {
  for (int i = 0; i < 256; i++) {
    int b = 0; for (int k = 0; k < 8; k++) b += (i >> k) & 1;
    parT[i] = (b & 1) ? 0 : ZF_P;
    uint32_t e = 0; for (int k = 0; k < 8; k++) e |= (uint32_t)((i >> (7 - k)) & 1) << (4 * k);
    expTab[i] = e;
  }
}

static void freeAll() {
  if (blob)   { heap_caps_free(blob);   blob = nullptr; }
  if (rom)    { heap_caps_free(rom);    rom = nullptr; }
  if (fb)     { heap_caps_free(fb);     fb = nullptr; }
  ram = vram = cartRam = nullptr;
  romOk = false; romReadError = false; romSize = 0; romBanks = 1; cm = false;
}

static void *sAlloc(size_t n, bool internalFirst) {
  void *p = nullptr;
  const size_t keep = bleInited ? 0 : BLE_INTERNAL_NEED;   // keep room for BOOT-button BLE pairing
  if (internalFirst && heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > keep + n) p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!p && psramFound()) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p && !internalFirst && heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > keep + n) p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return p;
}

static void reset() {
  memset(blob, 0, BLOB_SZ);
  memset(reg, 0, sizeof(reg)); reg[2] = 0xFF; reg[5] = 0xFF; reg[6] = 0xFF; reg[10] = 0xFF;
  for (int i = 0; i < 32; i++) setPal(i, 0);
  vAddr = 0; vCode = 0; readBuf = 0; ctrlLatch = 0; ctrlByte = 0; status = 0; hintPend = false;
  lcnt = 0xFF; curLine = 0; vdpH = 192; mapH = 0; ioCtl = 0xFF; memCtl = 0;
  mapReg[0] = 0; mapReg[1] = 0; mapReg[2] = 1; mapReg[3] = 2;
  applyMap();
  AF.w = 0xFFFF; BC.w = DE.w = HL.w = 0; AF2.w = BC2.w = DE2.w = HL2.w = 0;
  IX.w = IY.w = 0xFFFF; SP = 0xDFF0; PC = 0; Ireg = Rreg = 0; IM = 1;
  IFF1 = IFF2 = false; halted = false; eiDelay = false; nmiPend = false;
  startPrev = (padState & 0x08) != 0;                     // START still held from the menu must not fire a PAUSE NMI
  cyc = 0; lineBase = 0;
  if (fb) memset(fb, 0, GAME_W * GAME_H * 2);
}

// Takes ownership of an already filled, padded ROM buffer ('size' = real bytes).
static bool install(uint32_t size) {
  initTables();
  romSize = size;
  romBanks = (size + 0x3FFF) / 0x4000; if (!romBanks) romBanks = 1;
  blob = (uint8_t *)sAlloc(BLOB_SZ, true);
  fb = (uint16_t *)sAlloc(GAME_W * GAME_H * 2, false);
  if (!blob || !fb) {
    freeAll();
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "MASTER SYSTEM", sizeof(loadErr2));
    return false;
  }
  ram = blob; vram = blob + RAM_SZ; cartRam = blob + RAM_SZ + VRAM_SZ;
  cm = false;
  if (size >= 0x8000) {                                   // Codemasters header: checksum word + its complement
    const uint16_t ck = rom[0x7FE6] | (rom[0x7FE7] << 8), cc = rom[0x7FE8] | (rom[0x7FE9] << 8);
    cm = ck && (uint16_t)(ck + cc) == 0;
  }
  romOk = true;
  reset();
  return true;
}

static void frame(bool draw) {
  if (!romOk || !fb) return;
  vdpH = 192;
  if (reg[0] & 0x04) {                                    // Mode 4 only
    if ((reg[0] & 0x02) && (reg[1] & 0x10)) vdpH = 224;      // M2 + M1
    else if ((reg[0] & 0x02) && (reg[1] & 0x08)) vdpH = 240;  // M2 + M3
  }
  if (mapH != vdpH) rebuildMaps();
  const bool st = (padState & 0x08) != 0;                 // START = PAUSE (NMI on the press)
  if (st && !startPrev) nmiPend = true;
  startPrev = st;
  for (int line = 0; line < LINES_TOTAL; line++) {
    curLine = line; lineBase = (int32_t)line * CYC_LINE;
    if (line <= vdpH) { if (--lcnt < 0) { lcnt = reg[10]; hintPend = true; } }
    else lcnt = reg[10];
    if (line == vdpH + 1) status |= 0x80;
    if (line < vdpH) {
      const int r = draw ? lineOut[line] : -1;
      if (r >= 0) renderLine(line, r);
      else if (reg[1] & 0x40) renderSprites(line, false);   // flags only, nothing painted
    }
    runCpu(lineBase + CYC_LINE);
  }
  cyc -= LINES_TOTAL * CYC_LINE;
  if (draw) blit();
}

// -------------------------------------------------------------- save state ---
struct SmsState {
  uint32_t romSize;
  uint16_t af, bc, de, hl, af2, bc2, de2, hl2, ix, iy, sp, pc;
  uint8_t  i, r, im, iff1, iff2, halted, eiDelay, nmiPend, startPrev, cm;
  uint8_t  reg[16], cramRaw[32], mapReg[4];
  uint16_t vAddr;
  uint8_t  vCode, readBuf, ctrlLatch, ctrlByte, status, hintPend, ioCtl, memCtl;
  int16_t  lcnt;
};

static bool stateSave(uint8_t slot) {
  if (!romOk || !blob || !currentRomPath[0] || !sdMount()) return false;
  char path[ROM_PATH_LEN + 16]; if (!sdBuildStatePath(currentRomPath, "sms", path, sizeof(path), slot)) return false;
  SmsState c{}; c.romSize = romSize;
  c.af = AF.w; c.bc = BC.w; c.de = DE.w; c.hl = HL.w; c.af2 = AF2.w; c.bc2 = BC2.w; c.de2 = DE2.w; c.hl2 = HL2.w;
  c.ix = IX.w; c.iy = IY.w; c.sp = SP; c.pc = PC;
  c.i = Ireg; c.r = Rreg; c.im = IM; c.iff1 = IFF1; c.iff2 = IFF2; c.halted = halted; c.eiDelay = eiDelay;
  c.nmiPend = nmiPend; c.startPrev = startPrev; c.cm = cm;
  memcpy(c.reg, reg, sizeof(reg)); memcpy(c.cramRaw, cramRaw, sizeof(cramRaw)); memcpy(c.mapReg, mapReg, sizeof(mapReg));
  c.vAddr = vAddr; c.vCode = vCode; c.readBuf = readBuf; c.ctrlLatch = ctrlLatch; c.ctrlByte = ctrlByte;
  c.status = status; c.hintPend = hintPend; c.ioCtl = ioCtl; c.memCtl = memCtl; c.lcnt = (int16_t)lcnt;
  const bool ok = sdStateWrite(path, SYS_SMS, &c, sizeof(c), blob, BLOB_SZ, nullptr, 0);
  Serial.printf("STATE SMS: %s (%s)\n", ok ? "saved" : "save failed", path);
  return ok;
}

static bool stateLoad(uint8_t slot) {
  if (!romOk || !blob || !currentRomPath[0] || !sdMount()) return false;
  char path[ROM_PATH_LEN + 16]; if (!sdBuildStatePath(currentRomPath, "sms", path, sizeof(path), slot)) return false;
  SmsState c{};
  uint8_t *tmp = (uint8_t *)sAlloc(BLOB_SZ, false);       // validate fully before touching live state
  if (!tmp) return false;
  if (!sdStateRead(path, SYS_SMS, &c, sizeof(c), tmp, BLOB_SZ, nullptr, 0) || c.romSize != romSize || (bool)c.cm != cm) { heap_caps_free(tmp); return false; }
  memcpy(blob, tmp, BLOB_SZ); heap_caps_free(tmp);
  AF.w = c.af; BC.w = c.bc; DE.w = c.de; HL.w = c.hl; AF2.w = c.af2; BC2.w = c.bc2; DE2.w = c.de2; HL2.w = c.hl2;
  IX.w = c.ix; IY.w = c.iy; SP = c.sp; PC = c.pc;
  Ireg = c.i; Rreg = c.r; IM = c.im; IFF1 = c.iff1; IFF2 = c.iff2; halted = c.halted; eiDelay = c.eiDelay;
  nmiPend = c.nmiPend; startPrev = c.startPrev;
  memcpy(reg, c.reg, sizeof(reg)); memcpy(mapReg, c.mapReg, sizeof(mapReg));
  for (int i = 0; i < 32; i++) setPal(i, c.cramRaw[i]);
  vAddr = c.vAddr; vCode = c.vCode; readBuf = c.readBuf; ctrlLatch = c.ctrlLatch; ctrlByte = c.ctrlByte;
  status = c.status; hintPend = c.hintPend; ioCtl = c.ioCtl; memCtl = c.memCtl; lcnt = c.lcnt;
  applyMap();
  cyc = 0; lineBase = 0; mapH = 0;
  return true;
}

}  // namespace sms

#undef Z_A
#undef Z_F
#undef Z_B
#undef Z_C
#undef Z_D
#undef Z_E
#undef Z_H
#undef Z_L

// ---- public wrappers used by core_manager.h ----------------------------------
static void smsShutdown() { sms::freeAll(); }
static bool smsRomReadFailed() { return sms::romReadError; }
static void smsReset() { if (sms::romOk) sms::reset(); }
static void smsFrame(bool draw) { sms::frame(draw); }

static bool smsLoad(const char *path) {
  sms::freeAll();
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    strlcpy(loadErr1, "ROM NOT FOUND", sizeof(loadErr1)); strlcpy(loadErr2, "CHECK SD CARD", sizeof(loadErr2));
    return false;
  }
  const size_t fileSz = f.size();
  const uint32_t skip = ((fileSz & 0x3FFF) == 0x200) ? 0x200 : 0;   // strip a 512-byte copier header
  const size_t sz = fileSz - skip;
  if (sz < 0x2000 || sz > 0x400000) {
    f.close();
    strlcpy(loadErr1, "BAD ROM SIZE", sizeof(loadErr1)); strlcpy(loadErr2, "NEED 8KB-4MB", sizeof(loadErr2));
    return false;
  }
  const size_t padded = (sz + 0x3FFF) & ~(size_t)0x3FFF;
  sms::rom = (uint8_t *)sms::sAlloc(padded, false);
  if (!sms::rom) {
    f.close(); sms::freeAll();
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "MASTER SYSTEM", sizeof(loadErr2));
    return false;
  }
  memset(sms::rom, 0, padded);
  Serial.printf("ROM: %s (%u bytes)\n", path, (unsigned)sz);
  for (size_t off = 0; off < sz; ) {
    const size_t n = (sz - off) < 8192 ? (sz - off) : 8192;
    if (sdDiagReadAt(f, (uint32_t)(skip + off), sms::rom + off, n) != (int)n) {
      f.close(); sms::freeAll();
      strlcpy(loadErr1, "ROM READ ERROR", sizeof(loadErr1)); strlcpy(loadErr2, "CHECK SD CARD", sizeof(loadErr2));
      return false;
    }
    off += n;
  }
  f.close();                                            // the whole ROM now lives in RAM
  return sms::install((uint32_t)sz);
}

#endif // EMU32_SMS_CORE_H
