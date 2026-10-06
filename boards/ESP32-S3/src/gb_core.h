// =====================================================================================
//  gb_core.h  -  Game Boy (DMG) emulation core for Emu32
// =====================================================================================
//  Part of the Emu32 sketch: keep it in the "src" folder next to emu32.ino.  It is #included by emu32.ino
//  and pulls in emu_common.h (same folder), which provides:
//    C(r,g,b), SH, OUT_W, rowSlot() / rowDone() / flushRows() (LCD row batching),
//    padBits (controller), sdMount() / sdUnmount() (the GB ROM has its own buffer, not romBuf),
//    romError, romErrBuf.
//  Contents: LR35902 CPU, timer, PPU, MBC1 / MBC3 / MBC5 cartridges, .gb loader.
//  Public API:  gbLoadRom(path)  gbInit()  gbReset()  gbFrame(draw)   and the GB_FRAME_US constant.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <SD.h>
#include "emu_common.h"

// =====================================================================================
//  GAME BOY (DMG) emulator: CPU + timer + PPU, cartridges MBC1 / MBC3 / MBC5.
//  No sound, no save files. Picture is scaled to 197x172, same bars/controls as the NES side.
// =====================================================================================
#define GB_FRAME_US 16743              // 59.7 Hz

static uint8_t *gbRom = nullptr, *gbRomBank1 = nullptr, *gbEram = nullptr, *gbEramPtr = nullptr;
static uint32_t gbRomBanks = 2, gbEramSize = 0;
static int gbMbc = 0;                  // 0 = none, 1 = MBC1, 3 = MBC3, 5 = MBC5
static uint16_t gbRomReg = 1;
static uint8_t gbHiReg = 0, gbRamReg = 0, gbMode = 0;
static bool gbEramOn = false;
static uint8_t *gbBoot = nullptr;      // 256-byte DMG boot ROM from /bios/gb (optional); nullptr = skip it and start at 0x0100
static bool gbBootOn = false;          // boot ROM currently mapped over 0x0000-0x00FF (until the game is started via FF50)

static uint8_t gbVram[8192], gbWram[8192], gbOam[160], gbHram[128], gbIO[128], gbIE = 0;
static uint8_t gR[8];                  // B C D E H L (index 6 unused) A
static uint8_t gF;                     // flags: Z=0x80 N=0x40 H=0x20 C=0x10
static uint16_t gbPC, gbSP, gbDiv, gbTimAcc;
static bool gbIME, gbHalt, gbEiPending;
static int gbLineCyc = 0, gbWinLine = 0;

static uint8_t gbXmap[OUT_W];
static uint8_t gbBgIdx[160];
static uint16_t gbPx[160], gbRowBuf[OUT_W];
static const uint16_t gbShade[4] = { C(224, 248, 208), C(136, 192, 112), C(52, 104, 86), C(8, 24, 32) };

// ---- cartridge banking ----
static void gbMbcUpdate() {
  uint32_t b = (gbMbc == 1) ? ((gbRomReg & 0x1F) | ((uint32_t)gbHiReg << 5)) : gbRomReg;
  gbRomBank1 = gbRom + (b % gbRomBanks) * 16384UL;
  if (gbEramSize) {
    uint32_t rb = (gbMbc == 1) ? (gbMode ? gbHiReg : 0) : (gbRamReg & 0x0F);
    gbEramPtr = gbEram + (rb % (gbEramSize / 8192)) * 8192UL;
  }
}

static void gbMbcWrite(uint16_t a, uint8_t v) {
  switch (a >> 13) {
    case 0: gbEramOn = ((v & 0x0F) == 0x0A); return;
    case 1:
      if (gbMbc == 1)      { gbRomReg = v & 0x1F; if (!gbRomReg) gbRomReg = 1; }
      else if (gbMbc == 3) { gbRomReg = v & 0x7F; if (!gbRomReg) gbRomReg = 1; }
      else if (a < 0x3000) gbRomReg = (gbRomReg & 0x100) | v;               // MBC5 low 8 bits
      else                 gbRomReg = (gbRomReg & 0xFF) | ((v & 1) << 8);   // MBC5 bit 9
      break;
    case 2: if (gbMbc == 1) gbHiReg = v & 3; else gbRamReg = v; break;
    default: gbMode = v & 1; break;
  }
  gbMbcUpdate();
}

// ---- memory bus ----
static uint8_t gbIoRd(uint8_t i) {
  switch (i) {
    case 0x00: {                                       // joypad (padBits: A B Sel Start Up Down Left Right)
      uint8_t sel = gbIO[0], p = padBits, r = 0x0F;
      if (!(sel & 0x10)) r &= ~(((p >> 7) & 1) | (((p >> 6) & 1) << 1) | (((p >> 4) & 1) << 2) | (((p >> 5) & 1) << 3));
      if (!(sel & 0x20)) r &= ~(p & 0x0F);
      return 0xC0 | sel | r;
    }
    case 0x04: return gbDiv >> 8;
    case 0x0F: return gbIO[0x0F] | 0xE0;
    case 0x41: {
      uint8_t mode = 0;
      if (gbIO[0x40] & 0x80) mode = (gbIO[0x44] >= 144) ? 1 : (gbLineCyc < 80 ? 2 : (gbLineCyc < 252 ? 3 : 0));
      return 0x80 | (gbIO[0x41] & 0x78) | (gbIO[0x44] == gbIO[0x45] ? 4 : 0) | mode;
    }
  }
  return gbIO[i];
}

static inline uint8_t gbRd(uint16_t a) {
  switch (a >> 12) {
    case 0: if (gbBootOn && a < 0x100) return gbBoot[a]; return gbRom[a];
    case 1: case 2: case 3: return gbRom[a];
    case 4: case 5: case 6: case 7: return gbRomBank1[a & 0x3FFF];
    case 8: case 9: return gbVram[a & 0x1FFF];
    case 10: case 11: return (gbEramOn && gbEramSize) ? gbEramPtr[a & 0x1FFF] : 0xFF;
    case 12: case 13: case 14: return gbWram[a & 0x1FFF];
    default:
      if (a < 0xFE00) return gbWram[a & 0x1FFF];
      if (a < 0xFEA0) return gbOam[a - 0xFE00];
      if (a < 0xFF00) return 0xFF;
      if (a < 0xFF80) return gbIoRd(a & 0x7F);
      if (a == 0xFFFF) return gbIE;
      return gbHram[a & 0x7F];
  }
}

static void gbIoWr(uint8_t i, uint8_t v) {
  switch (i) {
    case 0x00: gbIO[0] = v & 0x30; break;
    case 0x04: gbDiv = 0; break;
    case 0x41: gbIO[0x41] = v & 0x78; break;
    case 0x44: break;
    case 0x50: if (v) gbBootOn = false; gbIO[0x50] = v; break;      // any non-zero write unmaps the boot ROM
    case 0x46: { uint16_t s = v << 8; for (int k = 0; k < 160; k++) gbOam[k] = gbRd(s + k); gbIO[0x46] = v; break; }
    default: gbIO[i] = v; break;
  }
}

static void gbWr(uint16_t a, uint8_t v) {
  switch (a >> 12) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: if (gbMbc) gbMbcWrite(a, v); break;
    case 8: case 9: gbVram[a & 0x1FFF] = v; break;
    case 10: case 11: if (gbEramOn && gbEramSize) gbEramPtr[a & 0x1FFF] = v; break;
    case 12: case 13: case 14: gbWram[a & 0x1FFF] = v; break;
    default:
      if (a < 0xFE00) gbWram[a & 0x1FFF] = v;
      else if (a < 0xFEA0) gbOam[a - 0xFE00] = v;
      else if (a < 0xFF00) { }
      else if (a < 0xFF80) gbIoWr(a & 0x7F, v);
      else if (a == 0xFFFF) gbIE = v;
      else gbHram[a & 0x7F] = v;
  }
}

// ---- CPU ----
static inline uint8_t gbFetch8() { return gbRd(gbPC++); }
static inline uint16_t gbFetch16() { uint8_t l = gbRd(gbPC++); uint8_t h = gbRd(gbPC++); return (h << 8) | l; }
static inline uint16_t gbHL() { return (gR[4] << 8) | gR[5]; }
static inline void gbSetHL(uint16_t v) { gR[4] = v >> 8; gR[5] = v; }
static inline uint16_t gbGetRR(int i) { return i == 3 ? gbSP : (uint16_t)((gR[i * 2] << 8) | gR[i * 2 + 1]); }
static inline void gbSetRR(int i, uint16_t v) { if (i == 3) gbSP = v; else { gR[i * 2] = v >> 8; gR[i * 2 + 1] = v; } }
static inline uint8_t gbGetR(int i) { return i == 6 ? gbRd(gbHL()) : gR[i]; }
static inline void gbSetR(int i, uint8_t v) { if (i == 6) gbWr(gbHL(), v); else gR[i] = v; }
static inline void gbPush(uint16_t v) { gbWr(--gbSP, v >> 8); gbWr(--gbSP, v & 0xFF); }
static inline uint16_t gbPop() { uint8_t l = gbRd(gbSP++); uint8_t h = gbRd(gbSP++); return (h << 8) | l; }
static inline bool gbCond(int cc) {
  switch (cc) { case 0: return !(gF & 0x80); case 1: return gF & 0x80; case 2: return !(gF & 0x10); default: return gF & 0x10; }
}

static void gbAlu(int op, uint8_t v) {
  uint8_t a = gR[7]; int r, c = (gF >> 4) & 1;
  switch (op) {
    case 0: case 1:                                    // ADD / ADC
      if (op == 0) c = 0;
      r = a + v + c;
      gF = ((r & 0xFF) ? 0 : 0x80) | (((a & 0xF) + (v & 0xF) + c) > 0xF ? 0x20 : 0) | (r > 0xFF ? 0x10 : 0);
      gR[7] = r; break;
    case 2: case 3: case 7:                            // SUB / SBC / CP
      if (op != 3) c = 0;
      r = a - v - c;
      gF = 0x40 | ((r & 0xFF) ? 0 : 0x80) | (((a & 0xF) < ((v & 0xF) + c)) ? 0x20 : 0) | (r < 0 ? 0x10 : 0);
      if (op != 7) gR[7] = r;
      break;
    case 4: a &= v; gF = (a ? 0 : 0x80) | 0x20; gR[7] = a; break;
    case 5: a ^= v; gF = a ? 0 : 0x80; gR[7] = a; break;
    default: a |= v; gF = a ? 0 : 0x80; gR[7] = a; break;
  }
}

static int gbCB() {
  uint8_t op = gbFetch8();
  int r = op & 7, bit = (op >> 3) & 7;
  uint8_t v = gbGetR(r);
  switch (op >> 6) {
    case 0: {
      uint8_t c, res;
      switch (bit) {
        case 0: c = v >> 7; res = (v << 1) | c; break;                       // RLC
        case 1: c = v & 1;  res = (v >> 1) | (c << 7); break;                // RRC
        case 2: c = v >> 7; res = (v << 1) | ((gF >> 4) & 1); break;         // RL
        case 3: c = v & 1;  res = (v >> 1) | (((gF >> 4) & 1) << 7); break;  // RR
        case 4: c = v >> 7; res = v << 1; break;                             // SLA
        case 5: c = v & 1;  res = (v >> 1) | (v & 0x80); break;              // SRA
        case 6: c = 0;      res = (v << 4) | (v >> 4); break;                // SWAP
        default: c = v & 1; res = v >> 1; break;                             // SRL
      }
      gF = (res ? 0 : 0x80) | (c ? 0x10 : 0);
      gbSetR(r, res);
      return r == 6 ? 4 : 2;
    }
    case 1: gF = (gF & 0x10) | 0x20 | ((v & (1 << bit)) ? 0 : 0x80); return r == 6 ? 3 : 2;   // BIT
    case 2: gbSetR(r, v & ~(1 << bit)); return r == 6 ? 4 : 2;                                // RES
    default: gbSetR(r, v | (1 << bit)); return r == 6 ? 4 : 2;                                // SET
  }
}

// Runs one instruction (or interrupt entry). Returns machine cycles (1 M-cycle = 4 clocks).
static int gbExec() {
  uint8_t ip = gbIO[0x0F] & gbIE & 0x1F;
  if (ip) {
    gbHalt = false;
    if (gbIME) {
      for (int i = 0; i < 5; i++) if (ip & (1 << i)) {
        gbIME = false; gbIO[0x0F] &= ~(1 << i); gbPush(gbPC); gbPC = 0x40 + i * 8; return 5;
      }
    }
  }
  if (gbHalt) return 1;

  uint8_t op = gbFetch8();
  switch (op) {
    case 0x00: return 1;
    case 0x10: gbPC++; return 1;                                            // STOP
    case 0x76: gbHalt = true; return 1;
    case 0xCB: return gbCB();
    case 0x08: { uint16_t a = gbFetch16(); gbWr(a, gbSP & 0xFF); gbWr(a + 1, gbSP >> 8); return 5; }
    case 0x18: { int8_t e = (int8_t)gbFetch8(); gbPC += e; return 3; }
    case 0x20: case 0x28: case 0x30: case 0x38: {
      int8_t e = (int8_t)gbFetch8();
      if (gbCond((op >> 3) & 3)) { gbPC += e; return 3; }
      return 2;
    }
    case 0x07: { uint8_t a = gR[7], c = a >> 7; gR[7] = (a << 1) | c; gF = c ? 0x10 : 0; return 1; }
    case 0x0F: { uint8_t a = gR[7], c = a & 1; gR[7] = (a >> 1) | (c << 7); gF = c ? 0x10 : 0; return 1; }
    case 0x17: { uint8_t a = gR[7], c = a >> 7; gR[7] = (a << 1) | ((gF >> 4) & 1); gF = c ? 0x10 : 0; return 1; }
    case 0x1F: { uint8_t a = gR[7], c = a & 1; gR[7] = (a >> 1) | (((gF >> 4) & 1) << 7); gF = c ? 0x10 : 0; return 1; }
    case 0x27: {                                                            // DAA
      int a = gR[7];
      if (!(gF & 0x40)) {
        if ((gF & 0x10) || a > 0x99) { a += 0x60; gF |= 0x10; }
        if ((gF & 0x20) || (a & 0xF) > 9) a += 6;
      } else {
        if (gF & 0x10) a -= 0x60;
        if (gF & 0x20) a -= 6;
      }
      a &= 0xFF; gF = (gF & 0x50) | (a ? 0 : 0x80); gR[7] = a; return 1;
    }
    case 0x2F: gR[7] = ~gR[7]; gF |= 0x60; return 1;
    case 0x37: gF = (gF & 0x80) | 0x10; return 1;
    case 0x3F: gF = (gF & 0x80) | ((gF & 0x10) ^ 0x10); return 1;
    case 0xC3: gbPC = gbFetch16(); return 4;
    case 0xC9: gbPC = gbPop(); return 4;
    case 0xD9: gbPC = gbPop(); gbIME = true; return 4;
    case 0xCD: { uint16_t a = gbFetch16(); gbPush(gbPC); gbPC = a; return 6; }
    case 0xE9: gbPC = gbHL(); return 1;
    case 0xE0: gbWr(0xFF00 + gbFetch8(), gR[7]); return 3;
    case 0xF0: gR[7] = gbRd(0xFF00 + gbFetch8()); return 3;
    case 0xE2: gbWr(0xFF00 + gR[1], gR[7]); return 2;
    case 0xF2: gR[7] = gbRd(0xFF00 + gR[1]); return 2;
    case 0xEA: { uint16_t a = gbFetch16(); gbWr(a, gR[7]); return 4; }
    case 0xFA: { uint16_t a = gbFetch16(); gR[7] = gbRd(a); return 4; }
    case 0xE8: case 0xF8: {                                                 // ADD SP,e / LD HL,SP+e
      int8_t e = (int8_t)gbFetch8();
      uint16_t r = gbSP + e;
      gF = (((gbSP & 0xF) + (e & 0xF)) > 0xF ? 0x20 : 0) | (((gbSP & 0xFF) + (e & 0xFF)) > 0xFF ? 0x10 : 0);
      if (op == 0xE8) { gbSP = r; return 4; }
      gbSetHL(r); return 3;
    }
    case 0xF9: gbSP = gbHL(); return 2;
    case 0xF3: gbIME = false; return 1;
    case 0xFB: gbEiPending = true; return 1;
  }

  if (op < 0x40) {
    int r = (op >> 3) & 7;
    switch (op & 0x0F) {
      case 0x01: gbSetRR(op >> 4, gbFetch16()); return 3;
      case 0x09: {
        uint16_t hl = gbHL(), v = gbGetRR(op >> 4); uint32_t s = hl + v;
        gF = (gF & 0x80) | (((hl & 0xFFF) + (v & 0xFFF)) > 0xFFF ? 0x20 : 0) | (s > 0xFFFF ? 0x10 : 0);
        gbSetHL(s); return 2;
      }
      case 0x03: gbSetRR(op >> 4, gbGetRR(op >> 4) + 1); return 2;
      case 0x0B: gbSetRR(op >> 4, gbGetRR(op >> 4) - 1); return 2;
      case 0x04: case 0x0C: {
        uint8_t v = gbGetR(r) + 1; gbSetR(r, v);
        gF = (gF & 0x10) | (v ? 0 : 0x80) | ((v & 0xF) == 0 ? 0x20 : 0);
        return r == 6 ? 3 : 1;
      }
      case 0x05: case 0x0D: {
        uint8_t v = gbGetR(r) - 1; gbSetR(r, v);
        gF = (gF & 0x10) | 0x40 | (v ? 0 : 0x80) | ((v & 0xF) == 0xF ? 0x20 : 0);
        return r == 6 ? 3 : 1;
      }
      case 0x06: case 0x0E: gbSetR(r, gbFetch8()); return r == 6 ? 3 : 2;
      case 0x02: case 0x0A: {                                               // LD (BC/DE/HL+/HL-),A and back
        uint16_t a;
        switch (op >> 4) {
          case 0: a = gbGetRR(0); break;
          case 1: a = gbGetRR(1); break;
          case 2: a = gbHL(); gbSetHL(a + 1); break;
          default: a = gbHL(); gbSetHL(a - 1); break;
        }
        if (op & 8) gR[7] = gbRd(a); else gbWr(a, gR[7]);
        return 2;
      }
    }
    return 1;
  }
  if (op < 0x80) { int d = (op >> 3) & 7, s = op & 7; gbSetR(d, gbGetR(s)); return (d == 6 || s == 6) ? 2 : 1; }
  if (op < 0xC0) { gbAlu((op >> 3) & 7, gbGetR(op & 7)); return (op & 7) == 6 ? 2 : 1; }

  switch (op & 7) {
    case 0: if (op < 0xE0) { if (gbCond((op >> 3) & 3)) { gbPC = gbPop(); return 5; } return 2; } break;
    case 1: {
      uint16_t v = gbPop();
      switch ((op >> 4) & 3) {
        case 0: gbSetRR(0, v); break;
        case 1: gbSetRR(1, v); break;
        case 2: gbSetRR(2, v); break;
        default: gR[7] = v >> 8; gF = v & 0xF0; break;
      }
      return 3;
    }
    case 2: if (op < 0xE0) { uint16_t a = gbFetch16(); if (gbCond((op >> 3) & 3)) { gbPC = a; return 4; } return 3; } break;
    case 4: if (op < 0xE0) { uint16_t a = gbFetch16(); if (gbCond((op >> 3) & 3)) { gbPush(gbPC); gbPC = a; return 6; } return 3; } break;
    case 5: {
      uint16_t v;
      switch ((op >> 4) & 3) {
        case 0: v = gbGetRR(0); break;
        case 1: v = gbGetRR(1); break;
        case 2: v = gbHL(); break;
        default: v = (gR[7] << 8) | gF; break;
      }
      gbPush(v); return 4;
    }
    case 6: gbAlu((op >> 3) & 7, gbFetch8()); return 2;
    case 7: gbPush(gbPC); gbPC = op & 0x38; return 4;
  }
  return 1;
}

static int gbStep() {                                  // EI takes effect after the NEXT instruction
  bool ei = gbEiPending; gbEiPending = false;
  int c = gbExec();
  if (ei) gbIME = true;
  return c;
}

static inline void gbTick(int cyc) {                   // DIV + TIMA
  gbDiv += cyc;
  uint8_t tac = gbIO[0x07];
  if (tac & 4) {
    static const uint16_t per[4] = {1024, 16, 64, 256};
    gbTimAcc += cyc;
    uint16_t p = per[tac & 3];
    while (gbTimAcc >= p) {
      gbTimAcc -= p;
      if (++gbIO[0x05] == 0) { gbIO[0x05] = gbIO[0x06]; gbIO[0x0F] |= 4; }
    }
  }
}

// ---- PPU ----
static inline void gbTileRow(uint16_t mapOff, int tx, int fy, bool uns, uint8_t &lo, uint8_t &hi) {
  uint8_t t = gbVram[mapOff + tx];
  int a = (uns ? t * 16 : 0x1000 + (int8_t)t * 16) + fy * 2;
  lo = gbVram[a]; hi = gbVram[a + 1];
}

static void gbRenderLine(int ly) {
  uint8_t lcdc = gbIO[0x40], bgp = gbIO[0x47];
  bool uns = lcdc & 0x10;
  uint16_t bpal[4];
  for (int i = 0; i < 4; i++) bpal[i] = gbShade[(bgp >> (i * 2)) & 3];

  if (lcdc & 1) {
    int y = (ly + gbIO[0x42]) & 255, sx = gbIO[0x43];
    uint16_t map = ((lcdc & 8) ? 0x1C00 : 0x1800) + (y >> 3) * 32;
    uint8_t lo = 0, hi = 0;
    for (int x = 0; x < 160; x++) {
      int bx = (x + sx) & 255;
      if (x == 0 || (bx & 7) == 0) gbTileRow(map, bx >> 3, y & 7, uns, lo, hi);
      int bit = 7 - (bx & 7), ci = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
      gbBgIdx[x] = ci; gbPx[x] = bpal[ci];
    }
    int wy = gbIO[0x4A], wx = gbIO[0x4B] - 7;
    if ((lcdc & 0x20) && ly >= wy && wx < 160) {                        // window
      uint16_t wmap = ((lcdc & 0x40) ? 0x1C00 : 0x1800) + (gbWinLine >> 3) * 32;
      int x0 = wx < 0 ? 0 : wx;
      for (int x = x0; x < 160; x++) {
        int wxp = x - wx;
        if (x == x0 || (wxp & 7) == 0) gbTileRow(wmap, wxp >> 3, gbWinLine & 7, uns, lo, hi);
        int bit = 7 - (wxp & 7), ci = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
        gbBgIdx[x] = ci; gbPx[x] = bpal[ci];
      }
      gbWinLine++;
    }
  } else {
    for (int x = 0; x < 160; x++) { gbBgIdx[x] = 0; gbPx[x] = bpal[0]; }
  }

  if (lcdc & 2) {                                                       // sprites (max 10 per line)
    int h = (lcdc & 4) ? 16 : 8, n = 0;
    uint16_t key[10];
    for (int i = 0; i < 40 && n < 10; i++) {
      int sy = gbOam[i * 4] - 16;
      if (ly >= sy && ly < sy + h) key[n++] = gbOam[i * 4 + 1] * 64 + i;
    }
    for (int a = 1; a < n; a++) {                                       // draw high X first, low X last (wins)
      uint16_t kk = key[a]; int b = a - 1;
      while (b >= 0 && key[b] < kk) { key[b + 1] = key[b]; b--; }
      key[b + 1] = kk;
    }
    for (int k = 0; k < n; k++) {
      int i = key[k] & 63;
      int sy = gbOam[i * 4] - 16, sx = gbOam[i * 4 + 1] - 8;
      uint8_t tile = gbOam[i * 4 + 2], at = gbOam[i * 4 + 3];
      int row = ly - sy;
      if (at & 0x40) row = h - 1 - row;
      if (h == 16) tile &= 0xFE;
      int a = tile * 16 + row * 2;
      uint8_t lo = gbVram[a], hi = gbVram[a + 1], obp = gbIO[(at & 0x10) ? 0x49 : 0x48];
      for (int b = 0; b < 8; b++) {
        int x = sx + b;
        if (x < 0 || x >= 160) continue;
        int bit = (at & 0x20) ? b : 7 - b, ci = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
        if (!ci || ((at & 0x80) && gbBgIdx[x])) continue;
        gbPx[x] = gbShade[(obp >> (ci * 2)) & 3];
      }
    }
  }

  for (int ox = 0; ox < OUT_W; ox++) gbRowBuf[ox] = gbPx[gbXmap[ox]];   // 160 -> 197 wide
  int o0 = (ly * SH) / 144, o1 = ((ly + 1) * SH) / 144;                 // 144 -> 172 tall
  for (int r = o0; r < o1; r++) { memcpy(rowSlot(r), gbRowBuf, OUT_W * 2); rowDone(); }
}

static void gbRun(int target) {
  while (gbLineCyc < target) {
    int c = gbStep() * 4;
    gbLineCyc += c;
    gbTick(c);
  }
}

static void gbFrame(bool draw) {
  flushRows();
  gbWinLine = 0;
  for (int ly = 0; ly < 154; ly++) {
    bool on = gbIO[0x40] & 0x80;
    gbIO[0x44] = on ? ly : 0;
    if (on) {
      uint8_t st = gbIO[0x41];
      if (ly == gbIO[0x45] && (st & 0x40)) gbIO[0x0F] |= 2;            // LY == LYC
      if (ly == 144) { gbIO[0x0F] |= 1; if (st & 0x10) gbIO[0x0F] |= 2; }   // VBlank
      else if (ly < 144 && (st & 0x20)) gbIO[0x0F] |= 2;               // OAM scan
    }
    gbRun(252);                                                        // end of drawing: render the line
    if (on && ly < 144) {
      if (draw) gbRenderLine(ly);
      if (gbIO[0x41] & 0x08) gbIO[0x0F] |= 2;                          // HBlank
    }
    gbRun(456);
    gbLineCyc -= 456;
  }
  flushRows();
}

static void gbReset() {
  memset(gbVram, 0, sizeof(gbVram)); memset(gbWram, 0, sizeof(gbWram));
  memset(gbOam, 0, sizeof(gbOam));   memset(gbHram, 0, sizeof(gbHram)); memset(gbIO, 0, sizeof(gbIO));
  gbIE = 0;
  if (gbBoot) {                                         // real power-on: everything zero, run the boot ROM from 0x0000
    memset(gR, 0, sizeof(gR)); gF = 0;
    gbSP = 0x0000; gbPC = 0x0000; gbDiv = 0; gbBootOn = true;
  } else {                                              // no BIOS: jump straight to the state the boot ROM would leave behind
    gbIO[0x40] = 0x91; gbIO[0x47] = 0xFC; gbIO[0x48] = 0xFF; gbIO[0x49] = 0xFF; gbIO[0x26] = 0xF1; gbIO[0x0F] = 0x01;
    gR[0] = 0x00; gR[1] = 0x13; gR[2] = 0x00; gR[3] = 0xD8; gR[4] = 0x01; gR[5] = 0x4D; gR[7] = 0x01; gF = 0xB0;
    gbSP = 0xFFFE; gbPC = 0x0100; gbDiv = 0xABCC; gbBootOn = false;
  }
  gbIME = gbHalt = gbEiPending = false;
  gbTimAcc = 0; gbLineCyc = 0;
  gbRomReg = 1; gbHiReg = 0; gbRamReg = 0; gbMode = 0; gbEramOn = false;
  gbMbcUpdate();
}

// Looks for the DMG boot ROM in /bios/gb (SD must be mounted). Missing or wrong-size files are ignored: the game then starts directly.
static void gbLoadBios() {
  static const char *names[] = { "dmg_boot.bin", "dmg_rom.bin", "gb_bios.bin", "bios.bin" };
  for (const char *n : names) {
    char path[48]; snprintf(path, sizeof(path), "%s/%s", GB_BIOS_DIR, n);
    if (!SD.exists(path)) continue;
    File b = SD.open(path, FILE_READ);
    if (!b) continue;
    if (b.size() != 256) { Serial.printf("GB BIOS %s ignored (must be 256 bytes)\n", path); b.close(); continue; }
    gbBoot = (uint8_t *)malloc(256);
    if (gbBoot && b.read(gbBoot, 256) == 256) { Serial.printf("GB BIOS loaded: %s\n", path); b.close(); return; }
    free(gbBoot); gbBoot = nullptr; b.close();
  }
  Serial.println("No GB BIOS in /bios/gb - skipping boot ROM");
}

// Reads a .gb file from the SD card into RAM (before the LCD is started, same as the NES loader).
static bool gbLoadRom(const char *path) {
  if (!sdMount()) { romError = "SD card not found"; return false; }
  File f = SD.open(path, FILE_READ);
  if (!f) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t sz = f.size();
  if (sz < 0x150) { romError = "ROM file is too small"; f.close(); sdUnmount(); return false; }
  uint32_t alloc = ((sz + 16383) / 16384) * 16384UL;
  if (alloc < 32768) alloc = 32768;
  gbRom = (uint8_t *)malloc(alloc);
  if (!gbRom) {
    snprintf(romErrBuf, sizeof(romErrBuf), "ROM too big (%u KB)", (unsigned)(sz / 1024));
    romError = romErrBuf; f.close(); sdUnmount(); return false;
  }
  memset(gbRom + sz, 0xFF, alloc - sz);
  size_t got = 0;
  while (got < sz) {
    int n = f.read(gbRom + got, min((size_t)4096, sz - got));
    if (n <= 0) break;
    got += n;
  }
  f.close();
  gbLoadBios();                                          // optional boot ROM, read while the SD bus is still ours
  sdUnmount();
  if (got != sz) { romError = "Could not read ROM"; return false; }

  uint8_t t = gbRom[0x147], rc = gbRom[0x149];
  if (gbRom[0x143] == 0xC0) { romError = "Color-only game, no DMG mode"; return false; }
  if (t == 0 || t == 8 || t == 9) gbMbc = 0;
  else if (t >= 1 && t <= 3) gbMbc = 1;
  else if (t >= 0x0F && t <= 0x13) gbMbc = 3;
  else if (t >= 0x19 && t <= 0x1E) gbMbc = 5;
  else { snprintf(romErrBuf, sizeof(romErrBuf), "Cart type %02X not supported", t); romError = romErrBuf; return false; }
  gbRomBanks = alloc / 16384;

  static const uint32_t ramSz[6] = {0, 2048, 8192, 32768, 131072, 65536};
  gbEramSize = rc < 6 ? ramSz[rc] : 0;
  if (gbEramSize > 32768) gbEramSize = 32768;
  if (gbEramSize && gbEramSize < 8192) gbEramSize = 8192;
  if (gbEramSize) {
    gbEram = (uint8_t *)calloc(gbEramSize, 1);
    if (!gbEram) { romError = "Not enough RAM"; return false; }
  }
  Serial.printf("GB ROM: %u KB, cart type %02X, RAM %u KB\n", (unsigned)(sz / 1024), t, (unsigned)(gbEramSize / 1024));
  return true;
}


// Builds the lookup tables. Call once before gbReset().
static void gbInit() {
  for (int i = 0; i < OUT_W; i++) gbXmap[i] = (i * 160) / OUT_W;
}
