// =====================================================================================
//  sms_core.h  -  Sega Master System emulation core for Emu32
// =====================================================================================
//  Part of the Emu32 sketch: keep it in the "src" folder next to emu32.ino.  It is #included by emu32.ino
//  and pulls in emu_common.h (same folder), which provides:
//    C(r,g,b), SH, A26_OUT_W, rowSlot() / rowDone() / flushRows() (LCD row batching),
//    padBits (controller), sdMount() / sdUnmount(), romError, romErrBuf.
//  Contents: Z80 CPU, VDP (mode 4 only: background, sprites, scrolling, line + frame interrupts),
//            Sega mapper (and the Codemasters variant), I/O ports, 1 controller, .sms loader.
//  No sound, no save files, no BIOS needed.  Picture is 256x192 scaled to 229x172 (same width as the Atari 2600).
//  Controls:  D-pad, A = button 1, B = button 2, Start = PAUSE button (non-maskable interrupt).
//  ROMs run straight from the SD card (same idea as genesis_core.h): the file is never loaded into RAM. The core keeps a cache of
//  16 KB ROM banks (= the mapper's bank size) and reads a bank from the card the first time the game maps it in. The cache is
//  whatever RAM is left over (SMS_MIN_FREE_HEAP is kept free for Bluetooth), so a small ROM ends up fully resident after a short
//  warm-up. Because the card and the LCD share the SPI pins, the ROM is opened before the LCD starts and the card is mounted again
//  by smsStreamResume() once the LCD is up.
//  Public API:  smsStreamOpen(path)  smsStreamResume()  smsInit()  smsReset()  smsFrame(draw)   and the SMS_FRAME_US constant.
//    smsAfterSd   optional hook (set by the glue) called after every SD access to give the shared SPI bus back to the LCD.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <SD.h>
#include "emu_common.h"

#define SMS_FRAME_US   16688             // 59.92 Hz (NTSC)
#define SMS_LINES      262
#define SMS_LINE_CYC   228               // Z80 clock cycles per scanline (3.579545 MHz)
#define SMS_RENDER_AT  160               // cycle of the line at which it is drawn (after a line interrupt handler has had time to run)
#define SMS_ACTIVE     192
#ifndef SMS_MIN_FREE_HEAP
#define SMS_MIN_FREE_HEAP 30000          // heap that must stay free after the ROM is loaded
#endif
#define SMS_OUT_W      A26_OUT_W         // 256 -> 229 columns
#define SMS_OUT_X0     A26_OUT_X0

// ---------------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------------
static uint8_t  *smsCart = nullptr;                         // optional cartridge RAM (the ROM itself is streamed from the SD card)
static uint32_t smsRomBanks = 2, smsCartSize = 0;           // cartridge RAM: 0, 16 or 32 KB
static uint8_t  smsRam[8192];
static uint8_t  smsVram[16384];
static uint8_t  smsCram[32];
static uint16_t smsPal[32];                                 // CRAM as RGB565
static uint8_t  smsReg[16];                                 // VDP registers

static bool     smsCodem = false;                           // Codemasters mapper (writes to 0000 / 4000 / 8000 select banks)
static uint8_t  smsBank[3] = { 0, 1, 2 }, smsMapCtl = 0;
static const uint8_t *smsB0, *smsB1, *smsB2;                // bank pointers for 0000-3FFF, 4000-7FFF, 8000-BFFF
static bool     smsCartOn = false;                          // cartridge RAM mapped over slot 2
static bool     smsFixed1K = true;                          // first 1 KB of slot 0 always shows bank 0 (Sega mapper)

static bool     smsLatch = false;                           // VDP control port: first byte received
static uint16_t smsAddr = 0;
static uint8_t  smsCode = 0, smsFirst = 0, smsBuf = 0;
static uint8_t  smsStatus = 0;                              // bit7 frame interrupt, bit6 sprite overflow, bit5 sprite collision
static bool     smsHint = false;                            // line interrupt pending
static int      smsCnt = 0xFF;                              // line counter
static int      smsLine = 0;
static bool     smsNmi = false, smsPrevStart = false;

static uint8_t  smsBgLine[256], smsSprLine[256];
static bool     smsSprDirty = false;
static uint8_t  smsXmap[SMS_OUT_W];

// ---------------------------------------------------------------------------------
//  ROM access: 16 KB bank cache filled from the SD card
// ---------------------------------------------------------------------------------
#ifndef SMS_SD_HZ
#define SMS_SD_HZ 16000000                       // SD clock while playing (the LCD shares the bus and sets its own clock per transfer)
#endif
#ifndef SMS_CACHE_MAX
#define SMS_CACHE_MAX 32                         // at most 32 x 16 KB = 512 KB of cache (a 512 KB ROM can be fully resident)
#endif
#define SMS_CACHE_MIN  5                         // pinned bank 0 + three mapped banks + one free slot: fewer than this and the cache can't work
#define SMS_BANK_SIZE  16384UL

static File     smsFile;                         // the open ROM file (valid between smsStreamResume() and power-off)
static char     smsRomPath[96];
static uint32_t smsRomSize = 0, smsFileSkip = 0; // ROM bytes (without the optional 512-byte copier header) / bytes to skip in the file
static uint8_t *smsSlotBuf[SMS_CACHE_MAX];
static uint32_t smsSlotTag[SMS_CACHE_MAX];       // ROM bank held by each slot (0xFFFFFFFF = empty)
static uint32_t smsSlotUse[SMS_CACHE_MAX];       // last-use stamp for LRU eviction
static int      smsNSlots = 0;
static uint32_t smsUseClock = 0;
static uint32_t smsMapTag[3] = { 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu };   // ROM bank currently mapped in each slot (skips needless remaps)
static const uint8_t *smsPin0 = nullptr;         // slot 0 is pinned to bank 0: the first 1 KB of the address space always shows it
static void   (*smsAfterSd)() = nullptr;         // set by the glue: restores the LCD's SPI settings after an SD access
static uint32_t smsReadErrors = 0;               // failed SD reads (card removed / bus trouble): those bytes read as 0xFF

// Fills one cache slot with ROM bank `bank` (0xFF past the end of the ROM).
static void smsBankFill(uint8_t *dst, uint32_t bank) {
  uint32_t off = bank * (uint32_t)SMS_BANK_SIZE, n = 0;
  if (off < smsRomSize) {
    n = smsRomSize - off; if (n > (uint32_t)SMS_BANK_SIZE) n = (uint32_t)SMS_BANK_SIZE;
    bool ok = smsFile && smsFile.seek(smsFileSkip + off);
    uint32_t got = 0;
    while (ok && got < n) {
      int r = smsFile.read(dst + got, n - got);
      if (r <= 0) { ok = false; break; }
      got += (uint32_t)r;
    }
    if (!ok) { smsReadErrors++; n = got; }
  }
  if (smsAfterSd) smsAfterSd();                                    // SD access done: give the bus back to the LCD
  if (n < (uint32_t)SMS_BANK_SIZE) memset(dst + n, 0xFF, SMS_BANK_SIZE - n);
}

// Returns a pointer to ROM bank `bank` (already reduced modulo the bank count), loading it from the card on a miss.
// A slot that is currently mapped (smsB0/B1/B2) or pinned (slot 0) is never evicted, so the pointers stay valid.
static const uint8_t *smsGetBank(uint32_t bank) {
  int idx = -1;
  for (int i = 0; i < smsNSlots; i++) if (smsSlotTag[i] == bank) { idx = i; break; }
  if (idx < 0) {
    uint32_t best = 0;
    for (int i = 1; i < smsNSlots; i++) {
      const uint8_t *b = smsSlotBuf[i];
      if (b == smsB0 || b == smsB1 || b == smsB2) continue;
      if (idx < 0 || smsSlotUse[i] < best) { idx = i; best = smsSlotUse[i]; }
    }
    if (idx < 0) idx = 1;                                           // cannot happen with >= SMS_CACHE_MIN slots; stay safe
    smsBankFill(smsSlotBuf[idx], bank);
    smsSlotTag[idx] = bank;
  }
  smsSlotUse[idx] = ++smsUseClock;
  return smsSlotBuf[idx];
}

// ---------------------------------------------------------------------------------
//  Cartridge mapper + memory
// ---------------------------------------------------------------------------------
static void smsMapUpdate() {
  uint32_t n = smsRomBanks;
  uint32_t b0 = smsBank[0] % n, b1 = smsBank[1] % n, b2 = smsBank[2] % n;
  if (smsMapTag[0] != b0) { smsB0 = smsGetBank(b0); smsMapTag[0] = b0; }
  if (smsMapTag[1] != b1) { smsB1 = smsGetBank(b1); smsMapTag[1] = b1; }
  if (smsCartOn && smsCart) { smsB2 = smsCart + ((((smsMapCtl >> 2) & 1) * 16384UL) % smsCartSize); smsMapTag[2] = 0xFFFFFFFFu; }
  else if (smsMapTag[2] != b2) { smsB2 = smsGetBank(b2); smsMapTag[2] = b2; }
}

static inline uint8_t smsRd(uint16_t a) {
  if (a < 0x4000) return (a < 0x400 && smsFixed1K) ? smsPin0[a] : smsB0[a];
  if (a < 0x8000) return smsB1[a - 0x4000];
  if (a < 0xC000) return smsB2[a - 0x8000];
  return smsRam[a & 0x1FFF];
}

static void smsWr(uint16_t a, uint8_t v) {
  if (a >= 0xC000) {
    smsRam[a & 0x1FFF] = v;
    if (a >= 0xFFFC && !smsCodem) {                        // Sega mapper registers (mirrored at the top of RAM)
      switch (a & 3) {
        case 0: smsMapCtl = v; smsCartOn = (v & 8) != 0; break;
        case 1: smsBank[0] = v; break;
        case 2: smsBank[1] = v; break;
        default: smsBank[2] = v; break;
      }
      smsMapUpdate();
    }
    return;
  }
  if (smsCodem) {
    if (a == 0x0000)      { smsBank[0] = v; smsMapUpdate(); }
    else if (a == 0x4000) { smsBank[1] = v; smsMapUpdate(); }
    else if (a == 0x8000) { smsBank[2] = v; smsMapUpdate(); }
    return;
  }
  if (a >= 0x8000 && smsCartOn && smsCart) smsCart[((((smsMapCtl >> 2) & 1) * 16384UL) % smsCartSize) + (a - 0x8000)] = v;
}

// ---------------------------------------------------------------------------------
//  VDP ports
// ---------------------------------------------------------------------------------
static inline void smsCramWrite(int i, uint8_t v) {
  i &= 31; smsCram[i] = v & 0x3F;
  smsPal[i] = C((v & 3) * 85, ((v >> 2) & 3) * 85, ((v >> 4) & 3) * 85);
}

static void smsVdpCtrlWr(uint8_t v) {
  if (!smsLatch) { smsFirst = v; smsLatch = true; smsAddr = (uint16_t)((smsAddr & 0x3F00) | v); return; }
  smsLatch = false;
  smsCode = v >> 6;
  smsAddr = (uint16_t)(smsFirst | ((v & 0x3F) << 8));
  if (smsCode == 0) { smsBuf = smsVram[smsAddr]; smsAddr = (smsAddr + 1) & 0x3FFF; }
  else if (smsCode == 2) smsReg[v & 0x0F] = smsFirst;
}

static void smsVdpDataWr(uint8_t v) {
  smsLatch = false;
  if (smsCode == 3) smsCramWrite(smsAddr, v);
  else smsVram[smsAddr & 0x3FFF] = v;
  smsBuf = v;
  smsAddr = (smsAddr + 1) & 0x3FFF;
}

static uint8_t smsVdpDataRd() {
  smsLatch = false;
  uint8_t r = smsBuf;
  smsBuf = smsVram[smsAddr & 0x3FFF];
  smsAddr = (smsAddr + 1) & 0x3FFF;
  return r;
}

static uint8_t smsVdpStatusRd() {
  smsLatch = false;
  uint8_t r = smsStatus | 0x1F;
  smsStatus = 0;                                           // reading clears the flags and drops the interrupt lines
  smsHint = false;
  return r;
}

static inline bool smsIrqLine() {
  return ((smsStatus & 0x80) && (smsReg[1] & 0x20)) || (smsHint && (smsReg[0] & 0x10));
}

// ---------------------------------------------------------------------------------
//  I/O ports (A0-A5 / A6-A7 decoding like the real console: only bits 0, 6 and 7 matter)
// ---------------------------------------------------------------------------------
static int szLineCyc = 0;

static uint8_t smsPadRead(int port) {
  if (port) return 0xFF;                                   // 0xDD: second pad / reset button: nothing pressed
  uint8_t p = padBits, v = 0xFF;                           // 0xDC: bit0 Up, 1 Down, 2 Left, 3 Right, 4 Button 1, 5 Button 2 (active low)
  if (p & 0x10) v &= ~0x01;
  if (p & 0x20) v &= ~0x02;
  if (p & 0x40) v &= ~0x04;
  if (p & 0x80) v &= ~0x08;
  if (p & 0x01) v &= ~0x10;
  if (p & 0x02) v &= ~0x20;
  return v;
}

static uint8_t smsIn(uint16_t port) {
  switch (port & 0xC1) {
    case 0x40: { int l = smsLine; return (uint8_t)(l <= 0xDA ? l : l - 6); }          // V counter (NTSC 192-line)
    case 0x41: { int h = (szLineCyc * 171) / SMS_LINE_CYC; return (uint8_t)(h > 0xA8 ? 0xA8 : h); }   // H counter (approximate)
    case 0x80: return smsVdpDataRd();
    case 0x81: return smsVdpStatusRd();
    case 0xC0: return smsPadRead(0);
    case 0xC1: return smsPadRead(1);
  }
  return 0xFF;
}

static void smsOut(uint16_t port, uint8_t v) {
  switch (port & 0xC1) {
    case 0x80: smsVdpDataWr(v); break;
    case 0x81: smsVdpCtrlWr(v); break;
    default: break;                                        // 0x3E memory control, 0x3F I/O control, 0x7E/7F PSG: ignored (no sound)
  }
}

// =====================================================================================
//  Zilog Z80
// =====================================================================================
#pragma GCC push_options
#pragma GCC optimize("O2")

static uint8_t  szR[8], szR2[8], szF, szF2;               // B C D E H L (6 unused) A   and the shadow set
static uint16_t szIXY[2], szSP, szPC;                      // IX, IY
static uint8_t  szI, szRr, szIM;
static bool     szIFF1, szIFF2, szHalt, szEiDelay;

static inline uint16_t szFetch16();
static inline uint8_t  szFetch8() { return smsRd(szPC++); }
static inline uint8_t  szFetchOp() { szRr = (uint8_t)((szRr & 0x80) | ((szRr + 1) & 0x7F)); return smsRd(szPC++); }
static inline uint16_t szFetch16() { uint8_t lo = smsRd(szPC++); uint8_t hi = smsRd(szPC++); return (uint16_t)(lo | (hi << 8)); }
static inline uint16_t szRd16(uint16_t a) { return (uint16_t)(smsRd(a) | (smsRd((uint16_t)(a + 1)) << 8)); }
static inline void     szWr16(uint16_t a, uint16_t v) { smsWr(a, (uint8_t)v); smsWr((uint16_t)(a + 1), (uint8_t)(v >> 8)); }
static inline void     szPush16(uint16_t v) { szSP -= 2; szWr16(szSP, v); }
static inline uint16_t szPop16() { uint16_t v = szRd16(szSP); szSP += 2; return v; }

static inline uint8_t szSZ53(uint8_t r) { return (uint8_t)((r & 0xA8) | (r ? 0 : 0x40)); }
static inline uint8_t szPar(uint8_t r) { return __builtin_parity(r) ? 0 : 0x04; }

static inline uint16_t szHL() { return (uint16_t)((szR[4] << 8) | szR[5]); }
static inline void szSetHL(uint16_t v) { szR[4] = v >> 8; szR[5] = (uint8_t)v; }
static inline uint16_t szBC() { return (uint16_t)((szR[0] << 8) | szR[1]); }
static inline void szSetBC(uint16_t v) { szR[0] = v >> 8; szR[1] = (uint8_t)v; }
static inline uint16_t szDE() { return (uint16_t)((szR[2] << 8) | szR[3]); }
static inline void szSetDE(uint16_t v) { szR[2] = v >> 8; szR[3] = (uint8_t)v; }

// 16-bit register pairs: p = 0 BC, 1 DE, 2 HL (IX / IY with a prefix), 3 SP
static inline uint16_t szGetRP(int p, int idx) {
  switch (p) {
    case 0: return szBC();
    case 1: return szDE();
    case 2: return idx ? szIXY[idx - 1] : szHL();
    default: return szSP;
  }
}
static inline void szSetRP(int p, int idx, uint16_t v) {
  switch (p) {
    case 0: szSetBC(v); break;
    case 1: szSetDE(v); break;
    case 2: if (idx) szIXY[idx - 1] = v; else szSetHL(v); break;
    default: szSP = v; break;
  }
}

// 8-bit register access with IXH / IXL substitution (r = 0..5, 7)
static inline uint8_t szRdR(int r, int idx) {
  if (idx) { if (r == 4) return (uint8_t)(szIXY[idx - 1] >> 8); if (r == 5) return (uint8_t)szIXY[idx - 1]; }
  return szR[r];
}
static inline void szWrR(int r, int idx, uint8_t v) {
  if (idx && (r == 4 || r == 5)) {
    uint16_t &x = szIXY[idx - 1];
    if (r == 4) x = (uint16_t)((x & 0x00FF) | (v << 8)); else x = (uint16_t)((x & 0xFF00) | v);
    return;
  }
  szR[r] = v;
}

// address of the (HL) / (IX+d) operand; fetches the displacement when indexed
static inline uint16_t szEA(int idx) {
  if (!idx) return szHL();
  int8_t d = (int8_t)szFetch8();
  return (uint16_t)(szIXY[idx - 1] + d);
}

static inline bool szCond(int c) {
  switch (c) {
    case 0: return !(szF & 0x40);
    case 1: return  (szF & 0x40);
    case 2: return !(szF & 0x01);
    case 3: return  (szF & 0x01);
    case 4: return !(szF & 0x04);
    case 5: return  (szF & 0x04);
    case 6: return !(szF & 0x80);
    default: return (szF & 0x80);
  }
}

static void szAlu(int op, uint8_t v) {
  uint8_t a = szR[7]; int r; uint8_t f;
  switch (op) {
    case 0: case 1: {                                                  // ADD / ADC
      int c = (op == 1) ? (szF & 1) : 0; r = a + v + c; uint8_t res = (uint8_t)r;
      f = szSZ53(res) | ((a ^ v ^ res) & 0x10) | ((~(a ^ v) & (a ^ res) & 0x80) ? 4 : 0) | ((r >> 8) & 1);
      szR[7] = res; break;
    }
    case 2: case 3: case 7: {                                          // SUB / SBC / CP
      int c = (op == 3) ? (szF & 1) : 0; r = a - v - c; uint8_t res = (uint8_t)r;
      f = szSZ53(res) | ((a ^ v ^ res) & 0x10) | (((a ^ v) & (a ^ res) & 0x80) ? 4 : 0) | 2 | ((r >> 8) & 1);
      if (op == 7) f = (uint8_t)((f & ~0x28) | (v & 0x28)); else szR[7] = res;
      break;
    }
    case 4: r = a & v; f = szSZ53((uint8_t)r) | 0x10 | szPar((uint8_t)r); szR[7] = (uint8_t)r; break;   // AND
    case 5: r = a ^ v; f = szSZ53((uint8_t)r) | szPar((uint8_t)r);        szR[7] = (uint8_t)r; break;   // XOR
    default: r = a | v; f = szSZ53((uint8_t)r) | szPar((uint8_t)r);       szR[7] = (uint8_t)r; break;   // OR
  }
  szF = f;
}

static inline uint8_t szInc8(uint8_t v) {
  uint8_t r = v + 1;
  szF = (szF & 1) | szSZ53(r) | ((v & 0xF) == 0xF ? 0x10 : 0) | (v == 0x7F ? 0x04 : 0);
  return r;
}
static inline uint8_t szDec8(uint8_t v) {
  uint8_t r = v - 1;
  szF = (szF & 1) | szSZ53(r) | ((v & 0xF) == 0 ? 0x10 : 0) | (v == 0x80 ? 0x04 : 0) | 2;
  return r;
}

static uint8_t szShift(int y, uint8_t v) {                                // CB rotate / shift group
  uint8_t c, r;
  switch (y) {
    case 0: c = v >> 7; r = (uint8_t)((v << 1) | c); break;               // RLC
    case 1: c = v & 1;  r = (uint8_t)((v >> 1) | (c << 7)); break;        // RRC
    case 2: c = v >> 7; r = (uint8_t)((v << 1) | (szF & 1)); break;       // RL
    case 3: c = v & 1;  r = (uint8_t)((v >> 1) | ((szF & 1) << 7)); break;// RR
    case 4: c = v >> 7; r = (uint8_t)(v << 1); break;                     // SLA
    case 5: c = v & 1;  r = (uint8_t)((v >> 1) | (v & 0x80)); break;      // SRA
    case 6: c = v >> 7; r = (uint8_t)((v << 1) | 1); break;               // SLL (undocumented)
    default: c = v & 1; r = v >> 1; break;                                // SRL
  }
  szF = szSZ53(r) | szPar(r) | c;
  return r;
}

static inline uint16_t szAdd16(uint16_t a, uint16_t b) {
  uint32_t r = (uint32_t)a + b;
  szF = (szF & 0xC4) | (uint8_t)(((a ^ b ^ r) >> 8) & 0x10) | (uint8_t)((r >> 16) & 1) | (uint8_t)((r >> 8) & 0x28);
  return (uint16_t)r;
}
static inline uint16_t szAdc16(uint16_t a, uint16_t b) {
  uint32_t r = (uint32_t)a + b + (szF & 1); uint16_t res = (uint16_t)r;
  szF = (uint8_t)((res >> 8) & 0xA8) | (res ? 0 : 0x40) | (uint8_t)(((a ^ b ^ res) >> 8) & 0x10)
      | ((~(a ^ b) & (a ^ res) & 0x8000) ? 4 : 0) | (uint8_t)((r >> 16) & 1);
  return res;
}
static inline uint16_t szSbc16(uint16_t a, uint16_t b) {
  int32_t r = (int32_t)a - b - (szF & 1); uint16_t res = (uint16_t)r;
  szF = (uint8_t)((res >> 8) & 0xA8) | (res ? 0 : 0x40) | (uint8_t)(((a ^ b ^ res) >> 8) & 0x10)
      | (((a ^ b) & (a ^ res) & 0x8000) ? 4 : 0) | 2 | ((r & 0x10000) ? 1 : 0);
  return res;
}

static void szDaa() {
  uint8_t a = szR[7], adj = 0, c = szF & 1;
  if ((szF & 0x10) || (a & 0xF) > 9) adj |= 0x06;
  if (c || a > 0x99) { adj |= 0x60; c = 1; }
  uint8_t r = (szF & 2) ? (uint8_t)(a - adj) : (uint8_t)(a + adj);
  szF = szSZ53(r) | szPar(r) | (szF & 2) | c | ((a ^ r) & 0x10);
  szR[7] = r;
}

static int szCB(int idx) {
  uint16_t ea = 0; uint8_t op;
  if (idx) { int8_t d = (int8_t)szFetch8(); ea = (uint16_t)(szIXY[idx - 1] + d); op = szFetch8(); }
  else op = szFetchOp();
  int x = op >> 6, y = (op >> 3) & 7, z = op & 7;
  bool mem = idx || z == 6;
  if (mem && !idx) ea = szHL();
  uint8_t v = mem ? smsRd(ea) : szR[z];
  uint8_t res = v;
  switch (x) {
    case 0: res = szShift(y, v); break;
    case 1: {                                                          // BIT
      uint8_t t = v & (1 << y);
      szF = (szF & 1) | 0x10 | (t ? 0 : 0x44) | (t & 0x80) | ((mem ? (ea >> 8) : v) & 0x28);
      return idx ? 16 : (z == 6 ? 12 : 8);
    }
    case 2: res = v & ~(1 << y); break;                                // RES
    default: res = v | (1 << y); break;                                // SET
  }
  if (mem) smsWr(ea, res);
  if (z != 6) szR[z] = res;                                            // (with an index prefix the result is copied to the register too)
  return idx ? 19 : (z == 6 ? 15 : 8);
}

static int szED() {
  uint8_t op = szFetchOp();
  int x = op >> 6, y = (op >> 3) & 7, z = op & 7, p = y >> 1, q = y & 1;
  if (x == 1) {
    switch (z) {
      case 0: { uint8_t v = smsIn(szBC()); if (y != 6) szR[y] = v; szF = (szF & 1) | szSZ53(v) | szPar(v); return 12; }
      case 1: smsOut(szBC(), y == 6 ? 0 : szR[y]); return 12;
      case 2: { uint16_t hl = szHL(), rr = szGetRP(p, 0); szSetHL(q ? szAdc16(hl, rr) : szSbc16(hl, rr)); return 15; }
      case 3: {
        uint16_t a = szFetch16();
        if (q == 0) szWr16(a, szGetRP(p, 0)); else szSetRP(p, 0, szRd16(a));
        return 20;
      }
      case 4: { uint8_t a = szR[7]; szR[7] = 0; szAlu(2, a); return 8; }                              // NEG: A = 0 - A
      case 5: szIFF1 = szIFF2; szPC = szPop16(); return 14;                                          // RETN / RETI
      case 6: { static const uint8_t im[4] = { 0, 0, 1, 2 }; szIM = im[y & 3]; return 8; }
      default:
        switch (y) {
          case 0: szI = szR[7]; return 9;
          case 1: szRr = szR[7]; return 9;
          case 2: szR[7] = szI;  szF = (szF & 1) | szSZ53(szR[7]) | (szIFF2 ? 4 : 0); return 9;
          case 3: szR[7] = szRr; szF = (szF & 1) | szSZ53(szR[7]) | (szIFF2 ? 4 : 0); return 9;
          case 4: { uint8_t m = smsRd(szHL()), a = szR[7];                                   // RRD
                    smsWr(szHL(), (uint8_t)((a << 4) | (m >> 4)));
                    szR[7] = (uint8_t)((a & 0xF0) | (m & 0x0F)); szF = (szF & 1) | szSZ53(szR[7]) | szPar(szR[7]); return 18; }
          case 5: { uint8_t m = smsRd(szHL()), a = szR[7];                                   // RLD
                    smsWr(szHL(), (uint8_t)((m << 4) | (a & 0x0F)));
                    szR[7] = (uint8_t)((a & 0xF0) | (m >> 4)); szF = (szF & 1) | szSZ53(szR[7]) | szPar(szR[7]); return 18; }
          default: return 8;
        }
    }
  }
  if (x == 2 && z <= 3 && y >= 4) {                                    // block transfer / compare / I-O
    bool rep = (y & 2) != 0; int d = (y & 1) ? -1 : 1;
    switch (z) {
      case 0: {                                                        // LDI / LDD / LDIR / LDDR
        uint8_t v = smsRd(szHL()); smsWr(szDE(), v);
        szSetHL((uint16_t)(szHL() + d)); szSetDE((uint16_t)(szDE() + d)); szSetBC((uint16_t)(szBC() - 1));
        uint8_t n = (uint8_t)(v + szR[7]);
        szF = (szF & 0xC1) | (szBC() ? 4 : 0) | (n & 8) | ((n & 2) << 4);
        if (rep && szBC()) { szPC -= 2; return 21; }
        return 16;
      }
      case 1: {                                                        // CPI / CPD / CPIR / CPDR
        uint8_t v = smsRd(szHL()), a = szR[7], r = (uint8_t)(a - v); uint8_t hf = (a ^ v ^ r) & 0x10;
        szSetHL((uint16_t)(szHL() + d)); szSetBC((uint16_t)(szBC() - 1));
        uint8_t n = (uint8_t)(r - (hf ? 1 : 0));
        szF = (szF & 1) | 2 | (r & 0x80) | (r ? 0 : 0x40) | hf | (szBC() ? 4 : 0) | ((n & 2) << 4) | (n & 8);
        if (rep && szBC() && r) { szPC -= 2; return 21; }
        return 16;
      }
      case 2: {                                                        // INI / IND / INIR / INDR
        uint8_t v = smsIn(szBC()); smsWr(szHL(), v);
        szSetHL((uint16_t)(szHL() + d)); szR[0]--;
        szF = (szR[0] ? 0 : 0x40) | 2 | (szR[0] & 0xA8);
        if (rep && szR[0]) { szPC -= 2; return 21; }
        return 16;
      }
      default: {                                                       // OUTI / OUTD / OTIR / OTDR
        uint8_t v = smsRd(szHL()); szR[0]--; smsOut(szBC(), v);
        szSetHL((uint16_t)(szHL() + d));
        szF = (szR[0] ? 0 : 0x40) | 2 | (szR[0] & 0xA8);
        if (rep && szR[0]) { szPC -= 2; return 21; }
        return 16;
      }
    }
  }
  return 8;                                                            // undefined ED opcodes are NOPs
}

static int szMain(uint8_t op, int idx) {
  int x = op >> 6, y = (op >> 3) & 7, z = op & 7, p = y >> 1, q = y & 1;
  switch (x) {
    case 0:
      switch (z) {
        case 0:
          if (y == 0) return 4;
          if (y == 1) { uint8_t t = szR[7]; szR[7] = szR2[7]; szR2[7] = t; t = szF; szF = szF2; szF2 = t; return 4; }   // EX AF,AF'
          if (y == 2) { int8_t d = (int8_t)szFetch8(); if (--szR[0]) { szPC += d; return 13; } return 8; }          // DJNZ
          if (y == 3) { int8_t d = (int8_t)szFetch8(); szPC += d; return 12; }                                      // JR
          { int8_t d = (int8_t)szFetch8(); if (szCond(y - 4)) { szPC += d; return 12; } return 7; }                 // JR cc
        case 1:
          if (q == 0) { szSetRP(p, idx, szFetch16()); return 10; }                                                  // LD rp,nn
          { uint16_t r = szAdd16(szGetRP(2, idx), szGetRP(p, idx)); szSetRP(2, idx, r); return 11; }                // ADD HL,rp
        case 2:
          switch (y) {
            case 0: smsWr(szBC(), szR[7]); return 7;
            case 1: szR[7] = smsRd(szBC()); return 7;
            case 2: smsWr(szDE(), szR[7]); return 7;
            case 3: szR[7] = smsRd(szDE()); return 7;
            case 4: { uint16_t a = szFetch16(); szWr16(a, szGetRP(2, idx)); return 16; }
            case 5: { uint16_t a = szFetch16(); szSetRP(2, idx, szRd16(a)); return 16; }
            case 6: { uint16_t a = szFetch16(); smsWr(a, szR[7]); return 13; }
            default: { uint16_t a = szFetch16(); szR[7] = smsRd(a); return 13; }
          }
        case 3: szSetRP(p, idx, (uint16_t)(szGetRP(p, idx) + (q ? -1 : 1))); return 6;
        case 4:
          if (y == 6) { uint16_t ea = szEA(idx); smsWr(ea, szInc8(smsRd(ea))); return 11 + (idx ? 8 : 0); }
          szWrR(y, idx, szInc8(szRdR(y, idx))); return 4;
        case 5:
          if (y == 6) { uint16_t ea = szEA(idx); smsWr(ea, szDec8(smsRd(ea))); return 11 + (idx ? 8 : 0); }
          szWrR(y, idx, szDec8(szRdR(y, idx))); return 4;
        case 6:
          if (y == 6) { uint16_t ea = szEA(idx); smsWr(ea, szFetch8()); return 10 + (idx ? 5 : 0); }
          szWrR(y, idx, szFetch8()); return 7;
        default: {
          uint8_t a = szR[7], c;
          switch (y) {
            case 0: c = a >> 7; a = (uint8_t)((a << 1) | c); szR[7] = a; szF = (szF & 0xC4) | c | (a & 0x28); break;                 // RLCA
            case 1: c = a & 1;  a = (uint8_t)((a >> 1) | (c << 7)); szR[7] = a; szF = (szF & 0xC4) | c | (a & 0x28); break;         // RRCA
            case 2: c = a >> 7; a = (uint8_t)((a << 1) | (szF & 1)); szR[7] = a; szF = (szF & 0xC4) | c | (a & 0x28); break;       // RLA
            case 3: c = a & 1;  a = (uint8_t)((a >> 1) | ((szF & 1) << 7)); szR[7] = a; szF = (szF & 0xC4) | c | (a & 0x28); break;// RRA
            case 4: szDaa(); break;
            case 5: a = ~a; szR[7] = a; szF = (szF & 0xC5) | 0x12 | (a & 0x28); break;                                              // CPL
            case 6: szF = (szF & 0xC4) | 1 | (a & 0x28); break;                                                                      // SCF
            default: c = szF & 1; szF = (szF & 0xC4) | (c << 4) | (c ? 0 : 1) | (a & 0x28); break;                                  // CCF
          }
          return 4;
        }
      }
    case 1:
      if (op == 0x76) { szHalt = true; return 4; }                                                                       // HALT
      if (y == 6) { uint16_t ea = szEA(idx); smsWr(ea, szR[z]); return 7 + (idx ? 8 : 0); }                             // LD (HL),r  (plain registers)
      if (z == 6) { uint16_t ea = szEA(idx); szR[y] = smsRd(ea); return 7 + (idx ? 8 : 0); }                            // LD r,(HL)
      szWrR(y, idx, szRdR(z, idx)); return 4;
    case 2:
      if (z == 6) { uint16_t ea = szEA(idx); szAlu(y, smsRd(ea)); return 7 + (idx ? 8 : 0); }
      szAlu(y, szRdR(z, idx)); return 4;
    default:
      switch (z) {
        case 0: if (szCond(y)) { szPC = szPop16(); return 11; } return 5;                                               // RET cc
        case 1:
          if (q == 0) {                                                                                                // POP
            uint16_t v = szPop16();
            if (p == 3) { szR[7] = v >> 8; szF = (uint8_t)v; } else szSetRP(p, idx, v);
            return 10;
          }
          switch (p) {
            case 0: szPC = szPop16(); return 10;                                                                       // RET
            case 1: for (int i = 0; i < 6; i++) { uint8_t t = szR[i]; szR[i] = szR2[i]; szR2[i] = t; } return 4;      // EXX
            case 2: szPC = szGetRP(2, idx); return 4;                                                                  // JP (HL)
            default: szSP = szGetRP(2, idx); return 6;                                                                 // LD SP,HL
          }
        case 2: { uint16_t a = szFetch16(); if (szCond(y)) szPC = a; return 10; }                                      // JP cc,nn
        case 3:
          switch (y) {
            case 0: szPC = szFetch16(); return 10;                                                                     // JP nn
            case 2: { uint8_t n = szFetch8(); smsOut((uint16_t)((szR[7] << 8) | n), szR[7]); return 11; }              // OUT (n),A
            case 3: { uint8_t n = szFetch8(); szR[7] = smsIn((uint16_t)((szR[7] << 8) | n)); return 11; }              // IN A,(n)
            case 4: { uint16_t v = szRd16(szSP); szWr16(szSP, szGetRP(2, idx)); szSetRP(2, idx, v); return 19; }       // EX (SP),HL
            case 5: { uint16_t t = szDE(); szSetDE(szHL()); szSetHL(t); return 4; }                                    // EX DE,HL
            case 6: szIFF1 = szIFF2 = false; return 4;                                                                 // DI
            default: szIFF1 = szIFF2 = true; szEiDelay = true; return 4;                                               // EI
          }
        case 4: { uint16_t a = szFetch16(); if (szCond(y)) { szPush16(szPC); szPC = a; return 17; } return 10; }       // CALL cc,nn
        case 5:
          if (q == 0) {                                                                                                // PUSH
            uint16_t v = (p == 3) ? (uint16_t)((szR[7] << 8) | szF) : szGetRP(p, idx);
            szPush16(v); return 11;
          }
          { uint16_t a = szFetch16(); szPush16(szPC); szPC = a; return 17; }                                           // CALL nn
        case 6: szAlu(y, szFetch8()); return 7;
        default: szPush16(szPC); szPC = (uint16_t)(y * 8); return 11;                                                  // RST
      }
  }
  return 4;
}

// Runs one instruction (or takes a pending interrupt). Returns the cycles used.
static int szStep() {
  if (smsNmi) { smsNmi = false; szHalt = false; szIFF2 = szIFF1; szIFF1 = false; szPush16(szPC); szPC = 0x66; return 11; }
  if (szIFF1 && !szEiDelay && smsIrqLine()) {
    szHalt = false; szIFF1 = szIFF2 = false;
    szPush16(szPC);
    if (szIM == 2) { szPC = szRd16((uint16_t)((szI << 8) | 0xFF)); return 19; }
    szPC = 0x38; return 13;
  }
  szEiDelay = false;
  if (szHalt) { szRr = (uint8_t)((szRr & 0x80) | ((szRr + 1) & 0x7F)); return 4; }

  int cyc = 0, idx = 0; uint8_t op;
  for (;;) {
    op = szFetchOp();
    if (op == 0xDD) { idx = 1; cyc += 4; }
    else if (op == 0xFD) { idx = 2; cyc += 4; }
    else break;
  }
  if (op == 0xCB) return cyc + szCB(idx);
  if (op == 0xED) return cyc + szED();
  return cyc + szMain(op, idx);
}

#pragma GCC pop_options

// =====================================================================================
//  VDP: scanline renderer (mode 4)
// =====================================================================================
static void smsRenderLine(int y, int r0) {
  uint8_t reg0 = smsReg[0], reg1 = smsReg[1];
  uint8_t bd = (uint8_t)(0x10 | (smsReg[7] & 15));                       // backdrop = sprite palette colour (reg 7)
  if (smsSprDirty) { memset(smsSprLine, 0, 256); smsSprDirty = false; }

  if (!(reg1 & 0x40)) memset(smsBgLine, bd, 256);                        // display off: backdrop only
  else {
    // ---- background ----
    uint32_t nt = (uint32_t)(smsReg[2] & 0x0E) << 10;
    int hs = ((reg0 & 0x40) && y < 16) ? 0 : smsReg[8];                  // top two rows can be locked against horizontal scrolling
    int x = 0;
    while (x < 256) {
      int sx = (x - hs) & 255, fine = sx & 7, n = 8 - fine;
      if (x + n > 256) n = 256 - x;
      if (x < 192 && x + n > 192) n = 192 - x;                           // right 8 columns can be locked against vertical scrolling
      int vs = ((reg0 & 0x80) && x >= 192) ? 0 : smsReg[9];
      int py = (y + vs) % 224, fy = py & 7;
      uint32_t ea = (nt + (uint32_t)(((py >> 3) * 32 + (sx >> 3)) * 2)) & 0x3FFF;
      uint8_t lo = smsVram[ea], hi = smsVram[ea + 1];
      int tile = lo | ((hi & 1) << 8);
      bool hflip = (hi & 2) != 0;
      if (hi & 4) fy = 7 - fy;
      uint8_t attr = (uint8_t)(((hi & 8) ? 0x10 : 0) | ((hi & 0x10) ? 0x20 : 0));
      const uint8_t *t = smsVram + tile * 32 + fy * 4;
      uint8_t b0 = t[0], b1 = t[1], b2 = t[2], b3 = t[3];
      for (int i = 0; i < n; i++) {
        int bit = hflip ? (fine + i) : 7 - (fine + i);
        uint8_t c = (uint8_t)(((b0 >> bit) & 1) | (((b1 >> bit) & 1) << 1) | (((b2 >> bit) & 1) << 2) | (((b3 >> bit) & 1) << 3));
        smsBgLine[x + i] = c | attr;
      }
      x += n;
    }

    // ---- sprites (8 per line, first in the table wins) ----
    bool big = (reg1 & 2) != 0, zoom = (reg1 & 1) != 0;
    int h = (big ? 16 : 8) * (zoom ? 2 : 1);
    uint32_t sat = (uint32_t)(smsReg[5] & 0x7E) << 7;
    int xoff = (reg0 & 8) ? 8 : 0;
    int tbase = (smsReg[6] & 4) ? 256 : 0;
    uint8_t idxs[8]; int cnt = 0;
    for (int i = 0; i < 64; i++) {
      uint8_t yv = smsVram[(sat + i) & 0x3FFF];
      if (yv == 0xD0) break;
      int row = (uint8_t)(y - yv - 1);
      if (row < h) {
        if (cnt == 8) { smsStatus |= 0x40; break; }
        idxs[cnt++] = (uint8_t)i;
      }
    }
    for (int k = 0; k < cnt; k++) {
      int i = idxs[k];
      uint8_t yv = smsVram[(sat + i) & 0x3FFF];
      int sxp = (int)smsVram[(sat + 0x80 + i * 2) & 0x3FFF] - xoff;
      int tile = smsVram[(sat + 0x81 + i * 2) & 0x3FFF];
      int row = (uint8_t)(y - yv - 1);
      if (zoom) row >>= 1;
      if (big) { tile &= 0xFE; if (row >= 8) { tile++; row -= 8; } }
      const uint8_t *t = smsVram + ((tbase + tile) & 0x1FF) * 32 + row * 4;
      uint8_t b0 = t[0], b1 = t[1], b2 = t[2], b3 = t[3];
      for (int px = 0; px < 8; px++) {
        int bit = 7 - px;
        uint8_t c = (uint8_t)(((b0 >> bit) & 1) | (((b1 >> bit) & 1) << 1) | (((b2 >> bit) & 1) << 2) | (((b3 >> bit) & 1) << 3));
        if (!c) continue;
        for (int zz = 0; zz < (zoom ? 2 : 1); zz++) {
          int xx = sxp + px * (zoom ? 2 : 1) + zz;
          if (xx < 0 || xx > 255) continue;
          if (smsSprLine[xx]) { smsStatus |= 0x20; continue; }
          smsSprLine[xx] = c; smsSprDirty = true;
        }
      }
    }
    if (reg0 & 0x20) { memset(smsBgLine, bd, 8); memset(smsSprLine, 0, 8); }   // leftmost 8 pixels blanked
  }

  uint16_t *dst = rowSlot(r0);
  for (int ox = 0; ox < SMS_OUT_W; ox++) {
    int xx = smsXmap[ox]; uint8_t b = smsBgLine[xx], s = smsSprLine[xx];
    int ci = (s && !((b & 0x20) && (b & 15))) ? 16 + s : (b & 0x1F);
    dst[ox] = smsPal[ci];
  }
  rowDone();
}

// =====================================================================================
//  Frame loop
// =====================================================================================
static void smsRun(int target) {
  while (szLineCyc < target) szLineCyc += szStep();
}

static void smsFrame(bool draw) {
  flushRows();
  bool st = (padBits & 0x08) != 0;                                       // Start = PAUSE button = NMI on the rising edge
  if (st && !smsPrevStart) smsNmi = true;
  smsPrevStart = st;
  for (int ln = 0; ln < SMS_LINES; ln++) {
    smsLine = ln;
    if (ln <= SMS_ACTIVE) { if (--smsCnt < 0) { smsCnt = smsReg[10]; smsHint = true; } }   // line interrupt counter
    else smsCnt = smsReg[10];
    if (ln == SMS_ACTIVE) smsStatus |= 0x80;                             // frame interrupt
    smsRun(SMS_RENDER_AT);
    if (draw && ln < SMS_ACTIVE) {
      int o0 = (ln * SH) / SMS_ACTIVE, o1 = ((ln + 1) * SH) / SMS_ACTIVE; // 192 -> 172 lines: some lines are skipped
      if (o1 > o0) smsRenderLine(ln, o0);
    }
    smsRun(SMS_LINE_CYC);
    szLineCyc -= SMS_LINE_CYC;
  }
  flushRows();
}

// =====================================================================================
//  Setup / loader
// =====================================================================================
static void smsInit() {
  for (int i = 0; i < SMS_OUT_W; i++) smsXmap[i] = (uint8_t)((i * 256) / SMS_OUT_W);
}

static void smsReset() {
  memset(smsRam, 0, sizeof(smsRam)); memset(smsVram, 0, sizeof(smsVram));
  memset(smsReg, 0, sizeof(smsReg)); smsReg[10] = 0xFF;
  for (int i = 0; i < 32; i++) smsCramWrite(i, 0);
  smsLatch = false; smsAddr = 0; smsCode = 0; smsBuf = 0; smsStatus = 0; smsHint = false; smsCnt = 0xFF; smsLine = 0;
  smsNmi = false; smsPrevStart = false; smsSprDirty = true;
  smsBank[0] = 0; smsBank[1] = 1; smsBank[2] = 2; smsMapCtl = 0; smsCartOn = false;
  smsFixed1K = !smsCodem;
  if (smsCart) memset(smsCart, 0, smsCartSize);
  smsB0 = smsB1 = smsB2 = nullptr;
  smsMapTag[0] = smsMapTag[1] = smsMapTag[2] = 0xFFFFFFFFu;     // force the banks to be mapped in again (needs the ROM file open: call smsStreamResume() first)
  smsMapUpdate();
  memset(szR, 0, sizeof(szR)); memset(szR2, 0, sizeof(szR2));
  szR[7] = 0xFF; szF = 0xFF; szF2 = 0xFF;
  szIXY[0] = szIXY[1] = 0xFFFF; szSP = 0xDFF0; szPC = 0;
  szI = 0; szRr = 0; szIM = 0; szIFF1 = szIFF2 = false; szHalt = false; szEiDelay = false; szLineCyc = 0;
}

// ---------------------------------------------------------------------------------
//  SD streaming: open (before the LCD starts) and resume (after it has started)
// ---------------------------------------------------------------------------------
static void smsStreamFail(const char *msg) {
  romError = msg;
  if (smsFile) smsFile.close();
  sdUnmount();
  for (int i = 0; i < smsNSlots; i++) { free(smsSlotBuf[i]); smsSlotBuf[i] = nullptr; }
  smsNSlots = 0;
  free(smsCart); smsCart = nullptr; smsCartSize = 0;
}

// Opens the .sms file, sizes the bank cache from the free heap, loads bank 0, detects the mapper, then releases the card.
static bool smsStreamOpen(const char *path) {
  if (!sdMount()) { romError = "SD card not found"; return false; }
  smsFile = SD.open(path, FILE_READ);
  if (!smsFile) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t fsz = smsFile.size(), skip = ((fsz & 0x3FFF) == 512) ? 512 : 0;   // 512-byte copier header
  size_t sz = fsz - skip;
  if (sz < 0x2000) { smsStreamFail("ROM file is too small"); return false; }
  smsFileSkip = (uint32_t)skip; smsRomSize = (uint32_t)sz;
  strncpy(smsRomPath, path, sizeof(smsRomPath) - 1); smsRomPath[sizeof(smsRomPath) - 1] = 0;
  uint32_t banks = (uint32_t)((sz + 16383) / 16384);
  if (banks < 2) banks = 2;
  smsRomBanks = banks;

  uint32_t want = banks > SMS_CACHE_MAX ? SMS_CACHE_MAX : banks;      // banks the whole ROM would need (capped)
  uint32_t must = want < SMS_CACHE_MIN ? want : SMS_CACHE_MIN;
  smsNSlots = 0;
  while ((uint32_t)smsNSlots < must) {                                // the slots the cache cannot work without
    uint8_t *b = (uint8_t *)malloc(SMS_BANK_SIZE);
    if (!b) break;
    smsSlotBuf[smsNSlots] = b; smsSlotTag[smsNSlots] = 0xFFFFFFFFu; smsSlotUse[smsNSlots] = 0; smsNSlots++;
  }
  if ((uint32_t)smsNSlots < must) { smsStreamFail("Not enough RAM for ROM cache"); return false; }
  smsCart = nullptr; smsCartSize = 0;                                 // optional 16 KB of cartridge RAM (before the cache grows, so battery games get it)
  if (ESP.getFreeHeap() > 16384 + SMS_MIN_FREE_HEAP && (smsCart = (uint8_t *)malloc(16384))) smsCartSize = 16384;
  while ((uint32_t)smsNSlots < want && ESP.getFreeHeap() > SMS_MIN_FREE_HEAP + SMS_BANK_SIZE) {   // the rest of the free RAM = extra cache
    uint8_t *b = (uint8_t *)malloc(SMS_BANK_SIZE);
    if (!b) break;
    smsSlotBuf[smsNSlots] = b; smsSlotTag[smsNSlots] = 0xFFFFFFFFu; smsSlotUse[smsNSlots] = 0; smsNSlots++;
  }
  smsUseClock = 0; smsReadErrors = 0;
  smsB0 = smsB1 = smsB2 = nullptr;
  smsMapTag[0] = smsMapTag[1] = smsMapTag[2] = 0xFFFFFFFFu;

  smsAfterSd = nullptr;                                               // (not needed yet: the LCD isn't running)
  smsBankFill(smsSlotBuf[0], 0);                                      // slot 0 = bank 0, pinned for good
  smsSlotTag[0] = 0; smsSlotUse[0] = 0; smsPin0 = smsSlotBuf[0];

  // Codemasters carts carry a checksum pair at 0x7FE6 / 0x7FE8 that adds up to 0x10000
  smsCodem = false;
  if (sz >= 0x7FEA) {
    uint8_t h[4];
    if (smsFile.seek(skip + 0x7FE6) && smsFile.read(h, 4) == 4) {
      uint16_t c1 = (uint16_t)(h[0] | (h[1] << 8)), c2 = (uint16_t)(h[2] | (h[3] << 8));
      if (c1 && (uint16_t)(c1 + c2) == 0) smsCodem = true;
    }
  }
  smsFile.close(); sdUnmount();                                       // the LCD needs the SPI pins next
  Serial.printf("SMS ROM (SD stream): %u KB (%u banks)%s, cache %d x 16 KB%s\n", (unsigned)(sz / 1024), (unsigned)smsRomBanks,
                smsCodem ? ", Codemasters mapper" : "", smsNSlots, smsCart ? ", cart RAM" : "");
  return true;
}

// Call after the LCD has been started: mounts the card again on the shared SPI bus and re-opens the ROM.
// (Deliberately not sdMount(): on failure that would call SPI.end() and kill the LCD.)
static bool smsStreamResume() {
  pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, SMS_SD_HZ)) { romError = "SD card lost"; return false; }
  smsFile = SD.open(smsRomPath, FILE_READ);
  if (smsAfterSd) smsAfterSd();                                       // mounting also changed the bus settings
  if (!smsFile) { romError = "ROM not found on SD"; return false; }
  return true;
}
