// =====================================================================================
//  genesis_core.h  -  Sega Genesis / Mega Drive emulation core for Emu32  (CORE ONLY)
// =====================================================================================
//  Keep it next to emu32.ino (same folder as gb_core.h).  Pulls in emu_common.h, which provides:
//    C(r,g,b), SW, SH, padBits (controller), sdMount() / sdUnmount(), romError, romErrBuf, and the extern `panel`.
//
//  Contents: Motorola 68000 (full instruction set, supervisor/user, all exceptions, interrupts), VDP (planes A/B,
//  window, sprites, H/V scroll modes, DMA: 68k->VRAM/CRAM/VSRAM, fill, copy, H and V interrupts), 3-button pads,
//  Z80 bus-request handshake, .bin / .md / .gen / .smd loader.
//
//  ROMs of ANY size run straight from the SD card (GEN_STREAM_FROM_SD, default 1): the file is never loaded into RAM.
//  Instead the core keeps a cache of 4 KB ROM pages and reads a page from the card the first time the game touches it
//  (cache = all the RAM that is left over, so a small ROM ends up fully resident after a short warm-up and runs at RAM speed).
//  Because the card and the LCD share the SPI pins, the ROM is opened before the LCD starts (header / region) and the card
//  is mounted again by genStreamResume() once the LCD is up.  GEN_STREAM_FROM_SD 0 = the old behaviour (whole ROM in RAM).
//
//  NOT emulated: sound (YM2612 / PSG), the Z80 CPU itself (its 8 KB RAM and the bus-request registers exist, so
//  games boot and run, just silently), shadow/highlight, interlace, SRAM saves, mappers (SSF2 banking), 6-button pads.
//
//  Public API (same shape as the other cores):
//      genStreamOpen(path)         open the ROM for streaming (call before the LCD is started) - allocates the ~136 KB of
//                                  work memory, then the page cache; the card is released again afterwards
//      genStreamResume()           call once the LCD is running: mounts the card again so pages can be fetched while playing
//      genLoadRom(path)            old way: read the whole ROM into RAM (call before the LCD is started)
//      genSetRom(ptr, size)        alternative to genLoadRom: use a ROM that already sits in memory-mapped flash
//      genInit()                   build the lookup tables (once, before genReset)
//      genReset()                  power-on reset
//      genFrame(draw)              run one video frame; draw=false skips rendering (frame skipping)
//      GEN_FRAME_US                frame period in microseconds (59.9 Hz)
//      GEN_OUT_W / GEN_OUT_X0      width / left edge of the picture on the LCD: set rowW = GEN_OUT_W and
//                                  rowX0 = GEN_OUT_X0 in the glue so the side status bars are drawn correctly
//
//  Controller mapping (padBits -> Genesis):  A -> C,  B -> B,  Select -> A,  Start -> Start,  D-pad -> D-pad.
//
//  MEMORY: the Genesis needs 64 KB work RAM + 64 KB VRAM + 8 KB Z80 RAM (allocated only while this core runs); the rest of
//  the heap (minus GEN_MIN_FREE_HEAP, kept for run-time needs once Bluetooth has been started) becomes the ROM page cache.  genSetRom() with a
//  flash-mapped pointer still works too: when `genRom` is set the cache is bypassed.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <SD.h>
#include "emu_common.h"

#define GEN_FRAME_US  16688                      // 59.92 Hz
#define GEN_OUT_W     246                        // 320x224 scaled to 172 px tall keeping 4:3-ish aspect
#define GEN_OUT_X0    ((SW - GEN_OUT_W) / 2)
#define GEN_LINES     262                        // NTSC: 262 lines of 488 68k-cycles
#define GEN_LINE_CYC  488
#define GEN_HBLANK_AT 360                        // cycle of the line where H-blank starts (line is drawn + HINT raised here)

// ---------------------------------------------------------------------------------
//  Memory / state
// ---------------------------------------------------------------------------------
static const uint8_t *genRom = nullptr;          // the cartridge (RAM copy or flash-mapped pointer)
static uint32_t genRomSize = 0;
static uint8_t *genRomHeap = nullptr;            // set when genLoadRom() allocated the ROM
static uint8_t *genRam = nullptr, *genVram = nullptr, *genZram = nullptr;   // 64 KB, 64 KB, 8 KB (allocated on load)
static uint8_t genVersion = 0xA0;                // I/O version register: overseas NTSC

static uint16_t genCram[64], genPal[64], genVsram[40];     // colour RAM (9-bit), its RGB565 cache, vertical scroll RAM
static uint8_t  genReg[32];                      // VDP registers
static uint16_t genVdpAddr = 0, genVdpFirst = 0;
static uint8_t  genVdpCode = 0;
static bool     genVdpPending = false;           // first half of a 2-word control command received
static bool     genFillPending = false;          // DMA fill armed, waiting for the data word
static uint8_t  genSticky = 0;                   // status bits 5 (sprite collision) / 6 (sprite overflow), cleared when read
static bool     genVintPending = false, genHintPending = false, genVblank = false, genVintFlag = false;
static int      genHcnt = 0, genLineNo = 0, genLineCyc = 0;

static uint8_t  genIoData[3] = { 0x7F, 0x7F, 0x7F }, genIoCtrl[3] = { 0, 0, 0 };
static bool     genZreq = false;                 // Z80 bus requested (granted immediately - there is no Z80 CPU)

static inline uint16_t genRd16(uint32_t a);
static uint8_t  genBusRd8(uint32_t a);
static uint16_t genBusRd16(uint32_t a);
static void     genBusWr8(uint32_t a, uint8_t v);
static void     genBusWr16(uint32_t a, uint16_t v);

// ---------------------------------------------------------------------------------
//  VDP: memory ports, DMA, status
// ---------------------------------------------------------------------------------
static inline void genCramWrite(int idx, uint16_t w) {
  idx &= 0x3F;
  genCram[idx] = w & 0x0EEE;
  uint8_t r = (w >> 1) & 7, g = (w >> 5) & 7, b = (w >> 9) & 7;
  genPal[idx] = C((r << 5) | (r << 2) | (r >> 1), (g << 5) | (g << 2) | (g >> 1), (b << 5) | (b << 2) | (b >> 1));
}

// One word to the memory selected by the current command code, then auto-increment (data port and DMA both use this).
static void genVdpPut(uint16_t w) {
  switch (genVdpCode & 0x0F) {
    case 1: {                                                      // VRAM
      uint16_t a = genVdpAddr;
      if (a & 1) w = (uint16_t)((w >> 8) | (w << 8));              // odd address: bytes are swapped
      a &= 0xFFFE;
      genVram[a] = w >> 8; genVram[a + 1] = (uint8_t)w;
      break;
    }
    case 3: genCramWrite(genVdpAddr >> 1, w); break;               // CRAM
    case 5: { int i = (genVdpAddr >> 1) & 0x7F; if (i < 40) genVsram[i] = w & 0x7FF; break; }   // VSRAM
  }
  genVdpAddr += genReg[15];
}

static uint16_t genVdpDataRd() {
  genVdpPending = false;
  uint16_t v = 0;
  switch (genVdpCode & 0x0F) {
    case 0: { uint16_t a = genVdpAddr & 0xFFFE; v = (uint16_t)((genVram[a] << 8) | genVram[a + 1]); break; }
    case 8: v = genCram[(genVdpAddr >> 1) & 0x3F]; break;
    case 4: { int i = (genVdpAddr >> 1) & 0x7F; v = i < 40 ? genVsram[i] : 0; break; }
  }
  genVdpAddr += genReg[15];
  return v;
}

static uint32_t genDmaLen() { uint32_t n = genReg[19] | ((uint32_t)genReg[20] << 8); return n ? n : 0x10000; }

static void genDmaFill(uint16_t w) {
  uint32_t n = genDmaLen();
  genVdpPut(w);                                                    // first word goes in normally
  uint8_t hi = w >> 8;
  for (uint32_t i = 0; i < n; i++) { genVram[genVdpAddr] = hi; genVdpAddr += genReg[15]; }     // VRAM is kept in logical byte order
  genReg[19] = genReg[20] = 0;
}

static void genDma() {
  uint8_t mode = genReg[23] & 0xC0;
  if (mode == 0x80) { genFillPending = true; return; }             // VRAM fill: waits for the data word
  uint32_t n = genDmaLen();
  if (mode == 0xC0) {                                              // VRAM -> VRAM copy
    uint16_t src = genReg[21] | (genReg[22] << 8);
    for (uint32_t i = 0; i < n; i++) { genVram[genVdpAddr] = genVram[src]; src++; genVdpAddr += genReg[15]; }
    genReg[21] = src & 0xFF; genReg[22] = src >> 8; genReg[19] = genReg[20] = 0;
    return;
  }
  uint32_t src = (((uint32_t)genReg[23] & 0x7F) << 17) | ((uint32_t)genReg[22] << 9) | ((uint32_t)genReg[21] << 1);
  for (uint32_t i = 0; i < n; i++) {                               // 68k bus -> VDP
    genVdpPut(genRd16(src));
    src = (src & 0xFE0000) | ((src + 2) & 0x1FFFE);                // the source wraps inside its 128 KB block
  }
  genReg[21] = (src >> 1) & 0xFF; genReg[22] = (src >> 9) & 0xFF; genReg[19] = genReg[20] = 0;
}

static void genVdpCtrlWr(uint16_t w) {
  if (genVdpPending) {                                             // second word of a command
    genVdpPending = false;
    genVdpAddr = (genVdpFirst & 0x3FFF) | ((w & 3) << 14);
    genVdpCode = ((genVdpFirst >> 14) & 3) | ((w >> 2) & 0x3C);
    if ((genVdpCode & 0x20) && (genReg[1] & 0x10)) genDma();
    return;
  }
  if ((w & 0xC000) == 0x8000) {                                    // register write: 100R RRRR dddd dddd
    int r = (w >> 8) & 0x1F;
    if (r < 24) genReg[r] = w & 0xFF;
    return;
  }
  genVdpFirst = w;                                                 // first word of a command
  genVdpCode = (genVdpCode & 0x3C) | (w >> 14);
  genVdpAddr = (genVdpAddr & 0xC000) | (w & 0x3FFF);
  genVdpPending = true;
}

static void genVdpDataWr(uint16_t w) {
  genVdpPending = false;
  if (genFillPending) { genFillPending = false; genDmaFill(w); return; }
  genVdpPut(w);
}

static uint16_t genVdpStatus() {
  genVdpPending = false;
  uint16_t s = 0x3600 | genSticky;                                 // FIFO empty; collision / overflow bits
  if (genVintFlag) s |= 0x80;
  if (genVblank) s |= 0x08;
  if (genLineCyc >= GEN_HBLANK_AT) s |= 0x04;
  genSticky = 0;
  return s;
}

static uint16_t genHvCounter() {
  int v = genLineNo; if (v > 0xEA) v -= 6;
  int h = (genLineCyc * 256) / GEN_LINE_CYC; if (h > 255) h = 255;
  return (uint16_t)((v << 8) | h);
}

// ---------------------------------------------------------------------------------
//  I/O: 3-button pad on port 1, nothing on port 2
// ---------------------------------------------------------------------------------
static uint8_t genPadRead(int port) {
  uint8_t ctrl = genIoCtrl[port], data = genIoData[port], v;
  if (port == 0) {
    uint8_t p = padBits;
    bool th = (ctrl & 0x40) ? (data & 0x40) != 0 : true;           // an input line is pulled high
    bool U = p & 0x10, D = p & 0x20, L = p & 0x40, R = p & 0x80;
    bool A = p & 0x04, B = p & 0x02, Cb = p & 0x01, St = p & 0x08;  // pad Select -> A, pad B -> B, pad A -> C
    if (th) v = 0x40 | (U ? 0 : 1) | (D ? 0 : 2) | (L ? 0 : 4) | (R ? 0 : 8) | (B ? 0 : 0x10) | (Cb ? 0 : 0x20);
    else    v =        (U ? 0 : 1) | (D ? 0 : 2)                              | (A ? 0 : 0x10) | (St ? 0 : 0x20);
  } else v = 0x7F;                                                 // no controller: all lines high
  return ((data & ctrl) | (v & ~ctrl)) & 0x7F;
}

static inline int genIrqLevel() {
  if (!(genVintPending | genHintPending)) return 0;                // fast exit: this runs before every 68k instruction
  if (genVintPending && (genReg[1] & 0x20)) return 6;
  if (genHintPending && (genReg[0] & 0x10)) return 4;
  return 0;
}
static inline void genIrqAck(int lvl) { if (lvl == 6) genVintPending = false; else if (lvl == 4) genHintPending = false; }

// ---------------------------------------------------------------------------------
//  ROM access: a direct pointer (RAM copy / flash-mapped) or a page cache filled from the SD card
// ---------------------------------------------------------------------------------
#ifndef GEN_STREAM_FROM_SD
#define GEN_STREAM_FROM_SD 1
#endif
#ifndef GEN_MIN_FREE_HEAP
#define GEN_MIN_FREE_HEAP 40000                  // heap that must stay free once the Bluetooth stack is up (the cache stops growing there)
#endif
#ifndef GEN_SD_HZ
#define GEN_SD_HZ 16000000                       // SD clock while playing (the LCD shares the bus and sets its own clock per transfer)
#endif
#ifndef GEN_CACHE_MAX
#define GEN_CACHE_MAX 64                         // at most 64 x 4 KB = 256 KB of cache
#endif
#define GEN_CACHE_MIN 6                          // fewer pages than this and the game would thrash: refuse to start
#define GEN_PG_BITS   12                         // 4 KB pages (8 SD sectors per read)
#define GEN_PG_SIZE   (1u << GEN_PG_BITS)
#define GEN_PG_MASK   (GEN_PG_SIZE - 1)

static File     genFile;                         // the open ROM file (only valid between genStreamResume() and power-off)
static char     genRomPath[96];
static bool     genSmd = false;                  // Super Magic Drive dump (512-byte header, interleaved 16 KB blocks): de-interleaved per page
static uint32_t genFileSkip = 0;                 // 512 for .smd, else 0
static uint8_t *genSmdTmp = nullptr;             // 4 KB scratch for de-interleaving
static uint8_t *genPageBuf[GEN_CACHE_MAX];
static uint16_t genPageTag[GEN_CACHE_MAX];       // ROM page held by each cache slot (0xFFFF = empty)
static uint32_t genPageUse[GEN_CACHE_MAX];       // last-use stamp, for LRU eviction
static int      genNPages = 0;
static uint32_t genUseClock = 0;
static int      genFrontIdx[2] = { -1, -1 };     // the two most recently used slots: checked first, never evicted
static uint32_t genFrontPg[2] = { 0xFFFFFFFFu, 0xFFFFFFFFu };
static const uint8_t *genFrontBase[2] = { nullptr, nullptr };
static void   (*genAfterSd)() = nullptr;         // set by the glue: restores the LCD's SPI settings (the SD card shares the bus and changes them)
static uint32_t genReadErrors = 0;               // SD reads that failed (card removed / bus trouble): those bytes read as 0

static bool genFileRead(uint32_t pos, uint8_t *dst, uint32_t n) {
  if (!genFile || !genFile.seek(pos)) return false;
  uint32_t got = 0;
  while (got < n) {
    int r = genFile.read(dst + got, n - got);
    if (r <= 0) return false;
    got += (uint32_t)r;
  }
  return true;
}

// Fills one cache page with ROM bytes [pg*4096 .. +4095] (zero past the end of the ROM).
static void genPageFill(uint8_t *dst, uint32_t pg) {
  uint32_t off = pg << GEN_PG_BITS;
  bool ok = true;
  uint32_t n = off < genRomSize ? ((genRomSize - off) < GEN_PG_SIZE ? (genRomSize - off) : GEN_PG_SIZE) : 0;
  if (genSmd && n == GEN_PG_SIZE && (off & ~0x3FFFu) + 16384 <= genRomSize && genSmdTmp) {
    uint32_t base = genFileSkip + (off & ~0x3FFFu), i0 = (off & 0x3FFF) >> 1;     // 2048 byte pairs per page
    ok = genFileRead(base + i0, genSmdTmp, 2048) && genFileRead(base + 8192 + i0, genSmdTmp + 2048, 2048);
    if (ok) for (int i = 0; i < 2048; i++) { dst[2 * i + 1] = genSmdTmp[i]; dst[2 * i] = genSmdTmp[2048 + i]; }
  } else if (n) {
    ok = genFileRead(genFileSkip + off, dst, n);
  }
  if (genAfterSd) genAfterSd();                                    // SD access done: give the bus back to the LCD in the state it expects
  if (!ok) { genReadErrors++; n = 0; }
  if (n < GEN_PG_SIZE) memset(dst + n, 0, GEN_PG_SIZE - n);
}

static const uint8_t *genRomPtrSlow(uint32_t a) {
  uint32_t pg = a >> GEN_PG_BITS;
  int idx = -1;
  if (pg == genFrontPg[1]) idx = genFrontIdx[1];
  else for (int i = 0; i < genNPages; i++) if (genPageTag[i] == pg) { idx = i; break; }
  if (idx < 0) {                                                   // miss: evict the least recently used slot that isn't a front entry
    uint32_t best = 0xFFFFFFFFu;
    for (int i = 0; i < genNPages; i++) {
      if (i == genFrontIdx[0] || i == genFrontIdx[1]) continue;
      if (genPageUse[i] < best) { best = genPageUse[i]; idx = i; }
    }
    genPageFill(genPageBuf[idx], pg);
    genPageTag[idx] = (uint16_t)pg;
  }
  genPageUse[idx] = ++genUseClock;
  if (genFrontIdx[0] != idx) {                                     // promote to the front; the old front becomes the second entry
    genFrontIdx[1] = genFrontIdx[0]; genFrontPg[1] = genFrontPg[0]; genFrontBase[1] = genFrontBase[0];
    genFrontIdx[0] = idx; genFrontPg[0] = pg; genFrontBase[0] = genPageBuf[idx];
  }
  return genPageBuf[idx] + (a & GEN_PG_MASK);
}

static inline const uint8_t *genRomPtr(uint32_t a) {               // pointer to ROM byte `a` (a < genRomSize); valid until the next cache miss
  if ((a >> GEN_PG_BITS) == genFrontPg[0]) return genFrontBase[0] + (a & GEN_PG_MASK);
  return genRomPtrSlow(a);
}
static inline uint8_t genRomRd8(uint32_t a) { return genRom ? genRom[a] : *genRomPtr(a); }
static inline uint16_t genRomRd16(uint32_t a) {
  if (genRom) return (uint16_t)((genRom[a] << 8) | genRom[a + 1]);
  if ((a & GEN_PG_MASK) == GEN_PG_MASK) return (uint16_t)((genRomRd8(a) << 8) | genRomRd8(a + 1));   // odd address on a page edge
  const uint8_t *p = genRomPtr(a);
  return (uint16_t)((p[0] << 8) | p[1]);
}

// ---------------------------------------------------------------------------------
//  68000 bus (24-bit address, big-endian)
// ---------------------------------------------------------------------------------
static inline uint8_t genRd8(uint32_t a) {
  a &= 0xFFFFFF;
  if (a >= 0xE00000) return genRam[a & 0xFFFF];
  if (a < 0x400000) return a < genRomSize ? genRomRd8(a) : 0;
  return genBusRd8(a);
}
static inline uint16_t genRd16(uint32_t a) {
  a &= 0xFFFFFF;
  if (a >= 0xE00000) { a &= 0xFFFF; return (uint16_t)((genRam[a] << 8) | genRam[(a + 1) & 0xFFFF]); }
  if (a < 0x400000) return (a + 1 < genRomSize) ? genRomRd16(a) : 0;
  return genBusRd16(a);
}
static inline uint32_t genRd32(uint32_t a) { return ((uint32_t)genRd16(a) << 16) | genRd16(a + 2); }
static inline void genWr8(uint32_t a, uint8_t v) {
  a &= 0xFFFFFF;
  if (a >= 0xE00000) genRam[a & 0xFFFF] = v; else if (a >= 0x400000) genBusWr8(a, v);
}
static inline void genWr16(uint32_t a, uint16_t v) {
  a &= 0xFFFFFF;
  if (a >= 0xE00000) { a &= 0xFFFF; genRam[a] = v >> 8; genRam[(a + 1) & 0xFFFF] = (uint8_t)v; }
  else if (a >= 0x400000) genBusWr16(a, v);
}
static inline void genWr32(uint32_t a, uint32_t v) { genWr16(a, v >> 16); genWr16(a + 2, (uint16_t)v); }

static inline bool genIsVdp(uint32_t a) { return (a & 0xE700E0) == 0xC00000; }

static uint8_t genIoRd8(uint32_t a) {                              // 0xA10000-0xA1001F, odd addresses
  switch ((a >> 1) & 0xF) {
    case 0: return genVersion;
    case 1: return genPadRead(0);
    case 2: return genPadRead(1);
    case 3: return 0x7F;
    case 4: return genIoCtrl[0];
    case 5: return genIoCtrl[1];
    case 6: return genIoCtrl[2];
  }
  return 0;
}

static uint8_t genBusRd8(uint32_t a) {
  if (a >= 0xA00000 && a < 0xA10000) {                             // Z80 space
    if (a < 0xA04000) return genZram[a & 0x1FFF];
    return 0;                                                      // YM2612 status: never busy
  }
  if (a >= 0xA10000 && a < 0xA10020) return (a & 1) ? genIoRd8(a) : 0;
  if ((a & 0xFFFFFE) == 0xA11100) return (a & 1) ? 0 : (genZreq ? 0 : 1);   // bit0 = 0 once the 68k owns the Z80 bus
  if (genIsVdp(a)) {
    uint32_t r = a & 0x1F; uint16_t v;
    if (r < 4) v = genVdpDataRd(); else if (r < 8) v = genVdpStatus(); else if (r < 16) v = genHvCounter(); else return 0;
    return (a & 1) ? (uint8_t)v : (uint8_t)(v >> 8);
  }
  return 0;
}

static uint16_t genBusRd16(uint32_t a) {
  if (a >= 0xA00000 && a < 0xA10000) { uint8_t b = genBusRd8(a); return (uint16_t)((b << 8) | b); }
  if (a >= 0xA10000 && a < 0xA10020) return genIoRd8(a | 1);
  if ((a & 0xFFFFFE) == 0xA11100) return genZreq ? 0 : 0x0100;
  if (genIsVdp(a)) {
    uint32_t r = a & 0x1F;
    if (r < 4) return genVdpDataRd();
    if (r < 8) return genVdpStatus();
    if (r < 16) return genHvCounter();
  }
  return 0;
}

static void genBusWr8(uint32_t a, uint8_t v) {
  if (a >= 0xA00000 && a < 0xA10000) { if (a < 0xA04000) genZram[a & 0x1FFF] = v; return; }
  if (a >= 0xA10000 && a < 0xA10020) {
    if (!(a & 1)) return;
    switch ((a >> 1) & 0xF) {
      case 1: genIoData[0] = v; break;  case 2: genIoData[1] = v; break;  case 3: genIoData[2] = v; break;
      case 4: genIoCtrl[0] = v; break;  case 5: genIoCtrl[1] = v; break;  case 6: genIoCtrl[2] = v; break;
    }
    return;
  }
  if ((a & 0xFFFFFE) == 0xA11100) { if (!(a & 1)) genZreq = v & 1; return; }
  if (genIsVdp(a)) {
    uint32_t r = a & 0x1F; uint16_t w = (uint16_t)((v << 8) | v);
    if (r < 4) genVdpDataWr(w); else if (r < 8) genVdpCtrlWr(w);   // 0x11 = PSG: ignored
  }
}

static void genBusWr16(uint32_t a, uint16_t v) {
  if (a >= 0xA00000 && a < 0xA10000) { if (a < 0xA04000) genZram[a & 0x1FFF] = v >> 8; return; }
  if (a >= 0xA10000 && a < 0xA10020) { genBusWr8(a | 1, (uint8_t)v); return; }
  if ((a & 0xFFFFFE) == 0xA11100) { genZreq = (v >> 8) & 1; return; }
  if (genIsVdp(a)) {
    uint32_t r = a & 0x1F;
    if (r < 4) genVdpDataWr(v); else if (r < 8) genVdpCtrlWr(v);
  }
}

// =====================================================================================
//  Motorola 68000
// =====================================================================================
#pragma GCC push_options
#pragma GCC optimize("O2")

static uint32_t g68D[8], g68A[8], g68Alt, g68PC;                   // A[7] is the active stack pointer, g68Alt the inactive one
static uint8_t  g68X, g68N, g68Z, g68V, g68C, g68S, g68T, g68Mask; // flags, supervisor, trace, interrupt mask
static bool     g68Stopped;
static int      g68Cyc;

struct G68Op { uint8_t kind; uint8_t reg; uint32_t addr; };        // kind: 0 Dn, 1 An, 2 memory at addr, 3 immediate (value in addr)

static inline uint32_t g68M(int sz)  { return sz == 1 ? 0xFFu : sz == 2 ? 0xFFFFu : 0xFFFFFFFFu; }
static inline uint32_t g68Ms(int sz) { return sz == 1 ? 0x80u : sz == 2 ? 0x8000u : 0x80000000u; }

static inline uint16_t g68Fetch16() { uint16_t v = genRd16(g68PC); g68PC += 2; return v; }
static inline uint32_t g68Fetch32() { uint32_t v = genRd32(g68PC); g68PC += 4; return v; }

static uint16_t g68GetSR() {
  return (uint16_t)((g68T << 15) | (g68S << 13) | (g68Mask << 8) | (g68X << 4) | (g68N << 3) | (g68Z << 2) | (g68V << 1) | g68C);
}
static void g68SetSR(uint16_t v) {
  uint8_t ns = (v >> 13) & 1;
  if (ns != g68S) { uint32_t t = g68A[7]; g68A[7] = g68Alt; g68Alt = t; g68S = ns; }
  g68T = (v >> 15) & 1; g68Mask = (v >> 8) & 7;
  g68X = (v >> 4) & 1; g68N = (v >> 3) & 1; g68Z = (v >> 2) & 1; g68V = (v >> 1) & 1; g68C = v & 1;
}

static inline void g68Push16(uint16_t v) { g68A[7] -= 2; genWr16(g68A[7], v); }
static inline void g68Push32(uint32_t v) { g68A[7] -= 4; genWr32(g68A[7], v); }
static inline uint16_t g68Pop16() { uint16_t v = genRd16(g68A[7]); g68A[7] += 2; return v; }
static inline uint32_t g68Pop32() { uint32_t v = genRd32(g68A[7]); g68A[7] += 4; return v; }

static void g68Exception(int vec, uint32_t pc) {
  uint16_t sr = g68GetSR();
  g68SetSR((uint16_t)((sr | 0x2000) & ~0x8000));                   // supervisor mode, trace off
  g68Push32(pc); g68Push16(sr);
  g68PC = genRd32(vec * 4);
  g68Cyc += 34;
}

static inline uint32_t g68RdMem(uint32_t a, int sz) { return sz == 1 ? genRd8(a) : sz == 2 ? genRd16(a) : genRd32(a); }
static inline void g68WrMem(uint32_t a, int sz, uint32_t v) { if (sz == 1) genWr8(a, (uint8_t)v); else if (sz == 2) genWr16(a, (uint16_t)v); else genWr32(a, v); }

static uint32_t g68Index(uint32_t base) {                          // brief extension word: d8(An/PC,Xn)
  uint16_t e = g68Fetch16();
  uint32_t r = (e & 0x8000) ? g68A[(e >> 12) & 7] : g68D[(e >> 12) & 7];
  if (!(e & 0x800)) r = (uint32_t)(int32_t)(int16_t)r;
  return base + (int8_t)e + r;
}

// Effective-address decode (consumes extension words, does the pre-decrement / post-increment).
static void g68Ea(G68Op &o, int mode, int reg, int sz) {
  int inc = (sz == 1 && reg == 7) ? 2 : sz;
  bool lng = sz == 4;
  o.reg = reg;
  switch (mode) {
    case 0: o.kind = 0; return;
    case 1: o.kind = 1; return;
    case 2: o.kind = 2; o.addr = g68A[reg]; g68Cyc += lng ? 8 : 4; return;
    case 3: o.kind = 2; o.addr = g68A[reg]; g68A[reg] += inc; g68Cyc += lng ? 8 : 4; return;
    case 4: g68A[reg] -= inc; o.kind = 2; o.addr = g68A[reg]; g68Cyc += lng ? 10 : 6; return;
    case 5: o.kind = 2; o.addr = g68A[reg] + (int16_t)g68Fetch16(); g68Cyc += lng ? 12 : 8; return;
    case 6: o.kind = 2; o.addr = g68Index(g68A[reg]); g68Cyc += lng ? 14 : 10; return;
    default:
      switch (reg) {
        case 0: o.kind = 2; o.addr = (uint32_t)(int32_t)(int16_t)g68Fetch16(); g68Cyc += lng ? 12 : 8; return;
        case 1: o.kind = 2; o.addr = g68Fetch32(); g68Cyc += lng ? 16 : 12; return;
        case 2: { uint32_t b = g68PC; o.kind = 2; o.addr = b + (int16_t)g68Fetch16(); g68Cyc += lng ? 12 : 8; return; }
        case 3: { uint32_t b = g68PC; o.kind = 2; o.addr = g68Index(b); g68Cyc += lng ? 14 : 10; return; }
        default: o.kind = 3; o.addr = (sz == 4) ? g68Fetch32() : (g68Fetch16() & g68M(sz)); g68Cyc += lng ? 8 : 4; return;
      }
  }
}

static inline uint32_t g68Rd(const G68Op &o, int sz) {
  switch (o.kind) {
    case 0: return g68D[o.reg] & g68M(sz);
    case 1: return g68A[o.reg] & g68M(sz);
    case 2: return g68RdMem(o.addr, sz);
    default: return o.addr;
  }
}
static inline void g68Wr(const G68Op &o, int sz, uint32_t v) {
  switch (o.kind) {
    case 0: { uint32_t m = g68M(sz); g68D[o.reg] = (g68D[o.reg] & ~m) | (v & m); break; }
    case 1: g68A[o.reg] = v; break;
    case 2: g68WrMem(o.addr, sz, v); break;
  }
}

static inline void g68NZ(uint32_t r, int sz) { g68N = (r & g68Ms(sz)) != 0; g68Z = (r & g68M(sz)) == 0; g68V = 0; g68C = 0; }

static uint32_t g68Add(uint32_t s, uint32_t d, int sz, int x, bool xop) {
  uint32_t m = g68M(sz), ms = g68Ms(sz);
  s &= m; d &= m;
  uint64_t r = (uint64_t)s + d + x;
  uint32_t res = (uint32_t)r & m;
  g68C = g68X = (uint8_t)((r >> (sz * 8)) & 1);
  g68V = ((s ^ res) & (d ^ res) & ms) != 0;
  g68N = (res & ms) != 0;
  if (xop) { if (res) g68Z = 0; } else g68Z = res == 0;
  return res;
}
// d - s - x ; setX=false for CMP
static uint32_t g68Sub(uint32_t s, uint32_t d, int sz, int x, bool xop, bool setX) {
  uint32_t m = g68M(sz), ms = g68Ms(sz);
  s &= m; d &= m;
  uint32_t res = (uint32_t)((uint64_t)d - s - x) & m;
  g68C = ((uint64_t)s + x) > d;
  if (setX) g68X = g68C;
  g68V = ((s ^ d) & (d ^ res) & ms) != 0;
  g68N = (res & ms) != 0;
  if (xop) { if (res) g68Z = 0; } else g68Z = res == 0;
  return res;
}

static bool g68Cond(int cc) {
  switch (cc) {
    case 0: return true;               case 1: return false;
    case 2: return !g68C && !g68Z;     case 3: return g68C || g68Z;
    case 4: return !g68C;              case 5: return g68C;
    case 6: return !g68Z;              case 7: return g68Z;
    case 8: return !g68V;              case 9: return g68V;
    case 10: return !g68N;             case 11: return g68N;
    case 12: return g68N == g68V;      case 13: return g68N != g68V;
    case 14: return !g68Z && g68N == g68V;
    default: return g68Z || g68N != g68V;
  }
}

static uint32_t g68Shift(int type, bool left, uint32_t v, int cnt, int sz) {
  uint32_t m = g68M(sz), ms = g68Ms(sz);
  v &= m;
  uint8_t c = 0, vf = 0;
  if (cnt == 0 && type == 2) c = g68X;
  for (int i = 0; i < cnt; i++) {
    switch (type) {
      case 0:
        if (left) { c = (v & ms) != 0; uint32_t n = (v << 1) & m; if ((n ^ v) & ms) vf = 1; v = n; }
        else      { c = v & 1; v = (v >> 1) | (v & ms); }
        g68X = c; break;
      case 1:
        if (left) { c = (v & ms) != 0; v = (v << 1) & m; } else { c = v & 1; v >>= 1; }
        g68X = c; break;
      case 2:
        if (left) { c = (v & ms) != 0; v = ((v << 1) | g68X) & m; } else { c = v & 1; v = (v >> 1) | (g68X ? ms : 0); }
        g68X = c; break;
      default:
        if (left) { c = (v & ms) != 0; v = ((v << 1) | c) & m; } else { c = v & 1; v = (v >> 1) | (c ? ms : 0); }
        break;
    }
  }
  g68C = c; g68V = vf; g68N = (v & ms) != 0; g68Z = v == 0;
  return v;
}

static void g68BitOp(uint16_t op, uint32_t bn, int mode, int reg) {
  int type = (op >> 6) & 3;                                        // 0 BTST, 1 BCHG, 2 BCLR, 3 BSET
  if (mode == 0) {
    uint32_t bit = 1u << (bn & 31), v = g68D[reg];
    g68Z = !(v & bit);
    if (type == 1) v ^= bit; else if (type == 2) v &= ~bit; else if (type == 3) v |= bit;
    if (type) g68D[reg] = v;
    g68Cyc += type == 0 ? 2 : 4;
  } else {
    G68Op o; g68Ea(o, mode, reg, 1);
    uint32_t bit = 1u << (bn & 7), v = g68Rd(o, 1);
    g68Z = !(v & bit);
    if (type == 1) v ^= bit; else if (type == 2) v &= ~bit; else if (type == 3) v |= bit;
    if (type) g68Wr(o, 1, v);
  }
}

static void g68Movem(uint16_t op, bool toReg, int sz) {
  int mode = (op >> 3) & 7, reg = op & 7;
  uint16_t mask = g68Fetch16();
  G68Op o; uint32_t addr;
  if (toReg) {
    if (mode == 3) addr = g68A[reg]; else { g68Ea(o, mode, reg, sz); addr = o.addr; }
    for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
      uint32_t v = sz == 2 ? (uint32_t)(int32_t)(int16_t)genRd16(addr) : genRd32(addr);
      if (i < 8) g68D[i] = v; else g68A[i - 8] = v;
      addr += sz; g68Cyc += sz == 2 ? 4 : 8;
    }
    if (mode == 3) g68A[reg] = addr;
  } else if (mode == 4) {
    addr = g68A[reg];
    for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
      int r = 15 - i; uint32_t v = r < 8 ? g68D[r] : g68A[r - 8];
      addr -= sz; if (sz == 2) genWr16(addr, (uint16_t)v); else genWr32(addr, v);
      g68Cyc += sz == 2 ? 4 : 8;
    }
    g68A[reg] = addr;
  } else {
    g68Ea(o, mode, reg, sz); addr = o.addr;
    for (int i = 0; i < 16; i++) if (mask & (1 << i)) {
      uint32_t v = i < 8 ? g68D[i] : g68A[i - 8];
      if (sz == 2) genWr16(addr, (uint16_t)v); else genWr32(addr, v);
      addr += sz; g68Cyc += sz == 2 ? 4 : 8;
    }
  }
}

static void g68Div(uint16_t op, bool sgn) {
  G68Op o; g68Ea(o, (op >> 3) & 7, op & 7, 2);
  uint32_t dv = g68Rd(o, 2); int dn = (op >> 9) & 7;
  if (dv == 0) { g68Exception(5, g68PC); return; }
  if (!sgn) {
    uint32_t n = g68D[dn], q = n / dv, r = n % dv;
    if (q > 0xFFFF) { g68V = 1; g68C = 0; g68N = 1; }
    else { g68D[dn] = (r << 16) | q; g68N = (q & 0x8000) != 0; g68Z = q == 0; g68V = 0; g68C = 0; }
    g68Cyc += 136;
  } else {
    int32_t n = (int32_t)g68D[dn], d = (int16_t)dv;
    if (n == (int32_t)0x80000000 && d == -1) { g68V = 1; g68C = 0; g68N = 1; g68Cyc += 150; return; }
    int32_t q = n / d, r = n % d;
    if (q < -32768 || q > 32767) { g68V = 1; g68C = 0; g68N = 1; }
    else { g68D[dn] = ((uint32_t)(uint16_t)r << 16) | (uint16_t)q; g68N = q < 0; g68Z = q == 0; g68V = 0; g68C = 0; }
    g68Cyc += 150;
  }
}

static uint8_t g68Abcd(uint8_t s, uint8_t d) {
  uint32_t lo = (d & 0xF) + (s & 0xF) + g68X; if (lo > 9) lo += 6;
  uint32_t res = (d & 0xF0) + (s & 0xF0) + lo;
  g68C = g68X = res > 0x99; if (g68C) res += 0x60;
  res &= 0xFF; if (res) g68Z = 0;
  return (uint8_t)res;
}
static uint8_t g68Sbcd(uint8_t s, uint8_t d) {
  int lo = (d & 0xF) - (s & 0xF) - g68X; if (lo < 0) lo -= 6;
  int res = (d & 0xF0) - (s & 0xF0) + lo;
  g68C = g68X = res < 0; if (g68C) res -= 0x60;
  res &= 0xFF; if (res) g68Z = 0;
  return (uint8_t)res;
}

static void g68Group4(uint16_t op, uint32_t opAddr) {
  int mode = (op >> 3) & 7, reg = op & 7;
  G68Op o;
  if (op & 0x100) {
    int an = (op >> 9) & 7;
    if ((op & 0x1C0) == 0x1C0) { g68Ea(o, mode, reg, 4); g68A[an] = o.addr; g68Cyc += 0; return; }          // LEA
    if ((op & 0x1C0) == 0x180) {                                                                            // CHK
      g68Ea(o, mode, reg, 2);
      int16_t bound = (int16_t)g68Rd(o, 2), v = (int16_t)g68D[an];
      if (v < 0) { g68N = 1; g68Exception(6, g68PC); } else if (v > bound) { g68N = 0; g68Exception(6, g68PC); }
      return;
    }
    g68Exception(4, opAddr); return;
  }
  switch ((op >> 9) & 7) {
    case 0: case 1: case 2: case 3: case 5: {
      int s = (op >> 6) & 3;
      int grp = (op >> 9) & 7;
      if (s == 3) {
        if (grp == 0) { g68Ea(o, mode, reg, 2); g68Wr(o, 2, g68GetSR()); return; }                        // MOVE from SR
        if (grp == 2) { g68Ea(o, mode, reg, 2); uint16_t v = (uint16_t)g68Rd(o, 2); g68SetSR((g68GetSR() & 0xFF00) | (v & 0x1F)); return; }   // MOVE to CCR
        if (grp == 3) {                                                                                     // MOVE to SR
          if (!g68S) { g68Exception(8, opAddr); return; }
          g68Ea(o, mode, reg, 2); g68SetSR((uint16_t)g68Rd(o, 2)); return;
        }
        if (grp == 5) {
          if (op == 0x4AFC) { g68Exception(4, opAddr); return; }                                            // ILLEGAL
          g68Ea(o, mode, reg, 1); uint32_t v = g68Rd(o, 1); g68NZ(v, 1); g68Wr(o, 1, v | 0x80); return;    // TAS
        }
        g68Exception(4, opAddr); return;
      }
      int sz = 1 << s;
      g68Ea(o, mode, reg, sz);
      uint32_t v = g68Rd(o, sz);
      switch (grp) {
        case 0: v = g68Sub(v, 0, sz, g68X, true, true); g68Wr(o, sz, v); break;      // NEGX
        case 1: g68N = 0; g68Z = 1; g68V = 0; g68C = 0; g68Wr(o, sz, 0); break;      // CLR
        case 2: v = g68Sub(v, 0, sz, 0, false, true); g68Wr(o, sz, v); break;        // NEG
        case 3: v = ~v & g68M(sz); g68NZ(v, sz); g68Wr(o, sz, v); break;             // NOT
        default: g68NZ(v, sz); break;                                                 // TST
      }
      return;
    }
    case 4:
      switch ((op >> 6) & 3) {
        case 0: {                                                                     // NBCD
          g68Ea(o, mode, reg, 1);
          uint8_t r = g68Sbcd((uint8_t)g68Rd(o, 1), 0); g68Wr(o, 1, r); return;
        }
        case 1:
          if (mode == 0) { uint32_t v = (g68D[reg] << 16) | (g68D[reg] >> 16); g68D[reg] = v; g68NZ(v, 4); return; }   // SWAP
          g68Ea(o, mode, reg, 4); g68Push32(o.addr); return;                                                           // PEA
        case 2:
          if (mode == 0) { uint32_t v = (g68D[reg] & 0xFFFF0000) | (uint16_t)(int16_t)(int8_t)g68D[reg]; g68D[reg] = v; g68NZ(v, 2); return; }   // EXT.W
          g68Movem(op, false, 2); return;
        default:
          if (mode == 0) { uint32_t v = (uint32_t)(int32_t)(int16_t)g68D[reg]; g68D[reg] = v; g68NZ(v, 4); return; }  // EXT.L
          g68Movem(op, false, 4); return;
      }
    case 6: g68Movem(op, true, (op & 0x40) ? 4 : 2); return;                          // MOVEM mem -> regs
    default: break;                                                                   // 0x4Exx
  }
  switch ((op >> 6) & 7) {
    case 1:
      switch ((op >> 4) & 3) {
        case 0: g68Exception(32 + (op & 15), g68PC); return;                          // TRAP
        case 1:
          if (!(op & 8)) { int16_t d = (int16_t)g68Fetch16(); g68Push32(g68A[reg]); g68A[reg] = g68A[7]; g68A[7] += d; }   // LINK
          else { uint32_t t = g68A[reg]; g68A[7] = t; g68A[reg] = g68Pop32(); }                                           // UNLK
          return;
        case 2:
          if (!g68S) { g68Exception(8, opAddr); return; }
          if (op & 8) g68A[reg] = g68Alt; else g68Alt = g68A[reg];                    // MOVE USP
          return;
        default:
          switch (op & 15) {
            case 0: if (!g68S) g68Exception(8, opAddr); return;                       // RESET
            case 1: return;                                                           // NOP
            case 2: if (!g68S) { g68Exception(8, opAddr); return; } g68SetSR(g68Fetch16()); g68Stopped = true; return;   // STOP
            case 3: if (!g68S) { g68Exception(8, opAddr); return; } { uint16_t sr = g68Pop16(); g68PC = g68Pop32(); g68SetSR(sr); } g68Cyc += 16; return;  // RTE
            case 5: g68PC = g68Pop32(); g68Cyc += 12; return;                         // RTS
            case 6: if (g68V) g68Exception(7, g68PC); return;                         // TRAPV
            case 7: { uint16_t c = g68Pop16(); g68SetSR((g68GetSR() & 0xFF00) | (c & 0x1F)); g68PC = g68Pop32(); g68Cyc += 16; return; }   // RTR
          }
          g68Exception(4, opAddr); return;
      }
    case 2: g68Ea(o, mode, reg, 4); g68Push32(g68PC); g68PC = o.addr; g68Cyc += 8; return;   // JSR
    case 3: g68Ea(o, mode, reg, 4); g68PC = o.addr; g68Cyc += 4; return;                      // JMP
  }
  g68Exception(4, opAddr);
}

// Runs one instruction (or takes a pending interrupt). Returns the cycles used.
static int g68Step() {
  int lvl = genIrqLevel();
  if (lvl > g68Mask) {
    genIrqAck(lvl); g68Stopped = false;
    uint16_t sr = g68GetSR();
    g68SetSR((uint16_t)((sr | 0x2000) & ~0x8000));
    g68Push32(g68PC); g68Push16(sr);
    g68Mask = lvl;
    g68PC = genRd32((24 + lvl) * 4);
    return 44;
  }
  if (g68Stopped) return 4;

  g68Cyc = 4;
  uint32_t opAddr = g68PC;
  uint16_t op = g68Fetch16();
  int mode = (op >> 3) & 7, reg = op & 7;
  G68Op o, d;

  switch (op >> 12) {
    case 0x0:
      if (op & 0x100) {
        if (mode == 1) {                                                                           // MOVEP
          int dn = (op >> 9) & 7; uint32_t a = g68A[reg] + (int16_t)g68Fetch16();
          switch ((op >> 6) & 7) {
            case 4: g68D[dn] = (g68D[dn] & 0xFFFF0000) | (genRd8(a) << 8) | genRd8(a + 2); break;
            case 5: g68D[dn] = ((uint32_t)genRd8(a) << 24) | (genRd8(a + 2) << 16) | (genRd8(a + 4) << 8) | genRd8(a + 6); break;
            case 6: genWr8(a, g68D[dn] >> 8); genWr8(a + 2, (uint8_t)g68D[dn]); break;
            case 7: genWr8(a, g68D[dn] >> 24); genWr8(a + 2, g68D[dn] >> 16); genWr8(a + 4, g68D[dn] >> 8); genWr8(a + 6, (uint8_t)g68D[dn]); break;
            default: g68Exception(4, opAddr); break;
          }
          g68Cyc += 12;
        } else g68BitOp(op, g68D[(op >> 9) & 7], mode, reg);
      } else {
        int t = (op >> 9) & 7;
        if (t == 4) { uint32_t bn = g68Fetch16() & 0xFF; g68BitOp(op, bn, mode, reg); }
        else if (t == 7 || ((op >> 6) & 3) == 3) g68Exception(4, opAddr);
        else {
          int sz = 1 << ((op >> 6) & 3);
          uint32_t imm = sz == 4 ? g68Fetch32() : (g68Fetch16() & g68M(sz));
          if (mode == 7 && reg == 4 && (t == 0 || t == 1 || t == 5) && sz <= 2) {                  // ORI/ANDI/EORI to CCR / SR
            uint16_t sr = g68GetSR();
            if (sz == 1) {
              uint16_t c = sr & 0xFF; c = t == 0 ? (c | imm) : t == 1 ? (c & imm) : (c ^ imm);
              g68SetSR((sr & 0xFF00) | (c & 0x1F));
            } else if (!g68S) g68Exception(8, opAddr);
            else g68SetSR(t == 0 ? (sr | imm) : t == 1 ? (sr & imm) : (sr ^ imm));
            g68Cyc += 16;
          } else {
            g68Ea(o, mode, reg, sz);
            uint32_t v = g68Rd(o, sz), r;
            switch (t) {
              case 0: r = v | imm; g68NZ(r, sz); g68Wr(o, sz, r); break;
              case 1: r = v & imm; g68NZ(r, sz); g68Wr(o, sz, r); break;
              case 2: r = g68Sub(imm, v, sz, 0, false, true); g68Wr(o, sz, r); break;
              case 3: r = g68Add(imm, v, sz, 0, false); g68Wr(o, sz, r); break;
              case 5: r = v ^ imm; g68NZ(r, sz); g68Wr(o, sz, r); break;
              default: g68Sub(imm, v, sz, 0, false, false); break;                               // CMPI
            }
          }
        }
      }
      break;

    case 0x1: case 0x2: case 0x3: {                                                                // MOVE / MOVEA
      int sz = (op >> 12) == 1 ? 1 : (op >> 12) == 3 ? 2 : 4;
      g68Ea(o, mode, reg, sz);
      uint32_t v = g68Rd(o, sz);
      int dm = (op >> 6) & 7, dr = (op >> 9) & 7;
      if (dm == 1) { g68A[dr] = sz == 2 ? (uint32_t)(int32_t)(int16_t)v : v; }
      else { g68NZ(v, sz); g68Ea(d, dm, dr, sz); g68Wr(d, sz, v); }
      break;
    }

    case 0x4: g68Group4(op, opAddr); break;

    case 0x5: {
      int s = (op >> 6) & 3;
      if (s == 3) {
        int cc = (op >> 8) & 15;
        if (mode == 1) {                                                                           // DBcc
          uint32_t base = g68PC; int16_t disp = (int16_t)g68Fetch16();
          if (!g68Cond(cc)) {
            int16_t c = (int16_t)g68D[reg] - 1;
            g68D[reg] = (g68D[reg] & 0xFFFF0000) | (uint16_t)c;
            if (c != -1) { g68PC = base + disp; g68Cyc = 10; } else g68Cyc = 14;
          } else g68Cyc = 12;
        } else { g68Ea(o, mode, reg, 1); g68Wr(o, 1, g68Cond(cc) ? 0xFF : 0); }                    // Scc
      } else {                                                                                     // ADDQ / SUBQ
        int sz = 1 << s, data = (op >> 9) & 7; if (!data) data = 8;
        if (mode == 1) { if (op & 0x100) g68A[reg] -= data; else g68A[reg] += data; g68Cyc += 4; }
        else {
          g68Ea(o, mode, reg, sz);
          uint32_t v = g68Rd(o, sz);
          v = (op & 0x100) ? g68Sub(data, v, sz, 0, false, true) : g68Add(data, v, sz, 0, false);
          g68Wr(o, sz, v);
        }
      }
      break;
    }

    case 0x6: {                                                                                    // Bcc / BRA / BSR
      uint32_t base = g68PC; int cc = (op >> 8) & 15;
      int32_t disp = (int8_t)op;
      bool word = (op & 0xFF) == 0;
      if (word) disp = (int16_t)g68Fetch16();
      if (cc == 1) { g68Push32(g68PC); g68PC = base + disp; g68Cyc = 18; }
      else if (g68Cond(cc)) { g68PC = base + disp; g68Cyc = 10; }
      else g68Cyc = word ? 12 : 8;
      break;
    }

    case 0x7:
      if (op & 0x100) { g68Exception(4, opAddr); break; }
      g68D[(op >> 9) & 7] = (uint32_t)(int32_t)(int8_t)op; g68NZ(g68D[(op >> 9) & 7], 4);
      break;

    case 0x8: case 0xC: {                                                                          // OR/DIV/SBCD   AND/MUL/ABCD/EXG
      int dn = (op >> 9) & 7; bool isC = (op >> 12) == 0xC;
      if ((op & 0x1C0) == 0xC0) {
        if (!isC) g68Div(op, false);
        else { g68Ea(o, mode, reg, 2); uint32_t r = (g68D[dn] & 0xFFFF) * (g68Rd(o, 2) & 0xFFFF); g68D[dn] = r; g68NZ(r, 4); g68Cyc += 34; }
      } else if ((op & 0x1C0) == 0x1C0) {
        if (!isC) g68Div(op, true);
        else { g68Ea(o, mode, reg, 2); int32_t r = (int32_t)(int16_t)g68D[dn] * (int32_t)(int16_t)g68Rd(o, 2); g68D[dn] = (uint32_t)r; g68NZ((uint32_t)r, 4); g68Cyc += 34; }
      } else if ((op & 0x1F0) == 0x100) {                                                          // ABCD / SBCD
        if (mode == 0) { uint8_t r = isC ? g68Abcd((uint8_t)g68D[reg], (uint8_t)g68D[dn]) : g68Sbcd((uint8_t)g68D[reg], (uint8_t)g68D[dn]); g68D[dn] = (g68D[dn] & ~0xFFu) | r; }
        else {
          g68A[reg] -= (reg == 7) ? 2 : 1; uint8_t s = genRd8(g68A[reg]);
          g68A[dn] -= (dn == 7) ? 2 : 1; uint8_t dd = genRd8(g68A[dn]);
          genWr8(g68A[dn], isC ? g68Abcd(s, dd) : g68Sbcd(s, dd));
        }
        g68Cyc += 2;
      } else if (isC && (op & 0x1F8) == 0x140) { uint32_t t = g68D[dn]; g68D[dn] = g68D[reg]; g68D[reg] = t; }        // EXG Dx,Dy
      else if (isC && (op & 0x1F8) == 0x148) { uint32_t t = g68A[dn]; g68A[dn] = g68A[reg]; g68A[reg] = t; }        // EXG Ax,Ay
      else if (isC && (op & 0x1F8) == 0x188) { uint32_t t = g68D[dn]; g68D[dn] = g68A[reg]; g68A[reg] = t; }        // EXG Dx,Ay
      else {
        int sz = 1 << ((op >> 6) & 3);
        g68Ea(o, mode, reg, sz);
        uint32_t s = g68Rd(o, sz), r;
        if (op & 0x100) { r = isC ? (s & g68D[dn]) : (s | g68D[dn]); g68NZ(r, sz); g68Wr(o, sz, r); }
        else { r = isC ? (s & g68D[dn]) : (s | g68D[dn]); g68NZ(r, sz); uint32_t m = g68M(sz); g68D[dn] = (g68D[dn] & ~m) | (r & m); }
      }
      break;
    }

    case 0x9: case 0xD: {                                                                          // SUB / ADD
      bool add = (op >> 12) == 0xD; int dn = (op >> 9) & 7;
      if ((op & 0xC0) == 0xC0) {                                                                   // SUBA / ADDA
        int sz = (op & 0x100) ? 4 : 2;
        g68Ea(o, mode, reg, sz);
        uint32_t s = g68Rd(o, sz); if (sz == 2) s = (uint32_t)(int32_t)(int16_t)s;
        if (add) g68A[dn] += s; else g68A[dn] -= s;
        g68Cyc += 4;
      } else if ((op & 0x130) == 0x100) {                                                          // ADDX / SUBX
        int sz = 1 << ((op >> 6) & 3);
        if (mode == 0) {
          uint32_t r = add ? g68Add(g68D[reg], g68D[dn], sz, g68X, true) : g68Sub(g68D[reg], g68D[dn], sz, g68X, true, true);
          uint32_t m = g68M(sz); g68D[dn] = (g68D[dn] & ~m) | r;
        } else {
          int inc1 = (sz == 1 && reg == 7) ? 2 : sz, inc2 = (sz == 1 && dn == 7) ? 2 : sz;
          g68A[reg] -= inc1; uint32_t s = g68RdMem(g68A[reg], sz);
          g68A[dn] -= inc2; uint32_t dd = g68RdMem(g68A[dn], sz);
          uint32_t r = add ? g68Add(s, dd, sz, g68X, true) : g68Sub(s, dd, sz, g68X, true, true);
          g68WrMem(g68A[dn], sz, r);
        }
      } else {
        int sz = 1 << ((op >> 6) & 3);
        g68Ea(o, mode, reg, sz);
        uint32_t s = g68Rd(o, sz);                                                                 // the <ea> operand (read once)
        if (op & 0x100) {                                                                          // Dn op <ea> -> <ea>
          uint32_t r = add ? g68Add(g68D[dn], s, sz, 0, false) : g68Sub(g68D[dn], s, sz, 0, false, true);
          g68Wr(o, sz, r);
        } else {                                                                                   // <ea> op Dn -> Dn
          uint32_t r = add ? g68Add(s, g68D[dn], sz, 0, false) : g68Sub(s, g68D[dn], sz, 0, false, true);
          uint32_t m = g68M(sz); g68D[dn] = (g68D[dn] & ~m) | r;
        }
      }
      break;
    }

    case 0xB: {                                                                                    // CMP / CMPA / CMPM / EOR
      int dn = (op >> 9) & 7;
      if ((op & 0xC0) == 0xC0) {                                                                   // CMPA
        int sz = (op & 0x100) ? 4 : 2;
        g68Ea(o, mode, reg, sz);
        uint32_t s = g68Rd(o, sz); if (sz == 2) s = (uint32_t)(int32_t)(int16_t)s;
        g68Sub(s, g68A[dn], 4, 0, false, false);
      } else {
        int sz = 1 << ((op >> 6) & 3);
        if (op & 0x100) {
          if (mode == 1) {                                                                         // CMPM (Ay)+,(Ax)+
            int i1 = (sz == 1 && reg == 7) ? 2 : sz, i2 = (sz == 1 && dn == 7) ? 2 : sz;
            uint32_t s = g68RdMem(g68A[reg], sz); g68A[reg] += i1;
            uint32_t dd = g68RdMem(g68A[dn], sz); g68A[dn] += i2;
            g68Sub(s, dd, sz, 0, false, false);
          } else {                                                                                 // EOR Dn,<ea>
            g68Ea(o, mode, reg, sz);
            uint32_t r = g68Rd(o, sz) ^ g68D[dn]; g68NZ(r, sz); g68Wr(o, sz, r);
          }
        } else { g68Ea(o, mode, reg, sz); g68Sub(g68Rd(o, sz), g68D[dn], sz, 0, false, false); }   // CMP
      }
      break;
    }

    case 0xE: {                                                                                    // shifts and rotates
      if ((op & 0xC0) == 0xC0) {                                                                   // memory form: word, by 1
        g68Ea(o, mode, reg, 2);
        uint32_t r = g68Shift((op >> 9) & 3, (op & 0x100) != 0, g68Rd(o, 2), 1, 2);
        g68Wr(o, 2, r);
      } else {
        int sz = 1 << ((op >> 6) & 3);
        int cnt = (op & 0x20) ? (int)(g68D[(op >> 9) & 7] & 63) : (((op >> 9) & 7) ? ((op >> 9) & 7) : 8);
        uint32_t r = g68Shift((op >> 3) & 3, (op & 0x100) != 0, g68D[reg], cnt, sz);
        uint32_t m = g68M(sz); g68D[reg] = (g68D[reg] & ~m) | r;
        g68Cyc = (sz == 4 ? 8 : 6) + 2 * cnt;
      }
      break;
    }

    case 0xA: g68Exception(10, opAddr); break;                                                     // line A
    default:  g68Exception(11, opAddr); break;                                                     // line F
  }
  return g68Cyc;
}

#pragma GCC pop_options

static void g68Reset() {
  memset(g68D, 0, sizeof(g68D)); memset(g68A, 0, sizeof(g68A));
  g68Alt = 0; g68X = g68N = g68Z = g68V = g68C = 0; g68T = 0; g68S = 1; g68Mask = 7; g68Stopped = false;
  g68A[7] = genRd32(0); g68PC = genRd32(4);
}

// =====================================================================================
//  VDP: scanline renderer (planes A / B / window / sprites -> palette index line -> RGB565 rows)
// =====================================================================================
static uint8_t  genBufA[320], genBufB[320], genBufS[320], genLineIdx[320];
static bool     genSprDirty = false;                             // genBufS holds sprite pixels from the previous line and must be cleared
static uint16_t genXmap40[GEN_OUT_W], genXmap32[GEN_OUT_W];
static uint16_t genRowBuf[GEN_OUT_W];
static uint16_t genBatch[GEN_OUT_W * 8];                         // own row batcher (this picture is wider than the shared one)
static int genBatchStart = 0, genBatchN = 0;

static inline void genFlushRows() {
  if (genBatchN) { panel->draw16bitRGBBitmap(GEN_OUT_X0, genBatchStart, genBatch, GEN_OUT_W, genBatchN); genBatchN = 0; }
}
static inline uint16_t *genRowSlot(int outRow) {
  if (genBatchN && outRow != genBatchStart + genBatchN) genFlushRows();
  if (!genBatchN) genBatchStart = outRow;
  return genBatch + genBatchN * GEN_OUT_W;
}
static inline void genRowDone() { if (++genBatchN >= 8) genFlushRows(); }

static inline uint32_t genTileRow(uint32_t tile, int ty) {
  const uint8_t *p = genVram + ((tile & 0x7FF) << 5) + (ty << 2);
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// n pixels of one tile row starting at pixel `fine`; value = priority<<7 | palette<<4 | colour (0 = transparent)
static inline uint8_t genPx(uint32_t nib, uint8_t attr) { return nib ? (uint8_t)(attr | nib) : 0; }
static inline void genPutRun(uint8_t *dst, uint32_t row, int fine, int n, bool hflip, uint8_t attr) {
  if (!row) { for (int i = 0; i < n; i++) dst[i] = 0; return; }       // empty tile row (very common): all transparent
  if (n == 8) {                                                        // whole tile row (fine is 0): unrolled
    if (!hflip) {
      dst[0] = genPx(row >> 28, attr);        dst[1] = genPx((row >> 24) & 15, attr);
      dst[2] = genPx((row >> 20) & 15, attr); dst[3] = genPx((row >> 16) & 15, attr);
      dst[4] = genPx((row >> 12) & 15, attr); dst[5] = genPx((row >> 8) & 15, attr);
      dst[6] = genPx((row >> 4) & 15, attr);  dst[7] = genPx(row & 15, attr);
    } else {
      dst[0] = genPx(row & 15, attr);         dst[1] = genPx((row >> 4) & 15, attr);
      dst[2] = genPx((row >> 8) & 15, attr);  dst[3] = genPx((row >> 12) & 15, attr);
      dst[4] = genPx((row >> 16) & 15, attr); dst[5] = genPx((row >> 20) & 15, attr);
      dst[6] = genPx((row >> 24) & 15, attr); dst[7] = genPx(row >> 28, attr);
    }
    return;
  }
  for (int i = 0; i < n; i++) {
    int px = fine + i;
    dst[i] = genPx(hflip ? (row >> (4 * px)) & 15 : (row >> (28 - 4 * px)) & 15, attr);
  }
}

static inline uint16_t genVr16(uint32_t a) { a &= 0xFFFE; return (uint16_t)((genVram[a] << 8) | genVram[a + 1]); }

static void genDrawPlane(uint8_t *buf, int x0, int x1, int y, bool isB) {
  static const uint8_t dims[4] = { 32, 64, 32, 128 };
  uint32_t nameBase = isB ? ((uint32_t)(genReg[4] & 7) << 13) : ((uint32_t)(genReg[2] & 0x38) << 10);
  int wc = dims[genReg[16] & 3], hc = dims[(genReg[16] >> 4) & 3];
  int pmask = wc * 8 - 1, ymask = hc * 8 - 1;
  int hmode = genReg[11] & 3;
  uint32_t hoff = hmode == 3 ? (uint32_t)y * 4 : hmode == 2 ? (uint32_t)(y & ~7) * 4 : 0;
  int hs = genVr16(((uint32_t)(genReg[13] & 0x3F) << 10) + hoff + (isB ? 2 : 0)) & 0x3FF;
  bool vcol = (genReg[11] & 4) != 0;
  int x = x0;
  while (x < x1) {
    int px = (x - hs) & pmask;
    int fine = px & 7;
    int end = x + 8 - fine; if (end > x1) end = x1;
    if (vcol) { int nb = (x | 15) + 1; if (end > nb) end = nb; }
    int vs = vcol ? genVsram[(x >> 4) * 2 + (isB ? 1 : 0)] : genVsram[isB ? 1 : 0];
    int py = (y + (vs & 0x3FF)) & ymask;
    uint16_t e = genVr16(nameBase + (((uint32_t)(py >> 3) * wc + ((px >> 3) & (wc - 1))) << 1));
    int ty = py & 7; if (e & 0x1000) ty = 7 - ty;
    uint8_t attr = (uint8_t)(((e >> 8) & 0x80) | (((e >> 13) & 3) << 4));
    genPutRun(buf + x, genTileRow(e & 0x7FF, ty), fine, end - x, (e & 0x800) != 0, attr);
    x = end;
  }
}

static void genDrawWindow(uint8_t *buf, int x0, int x1, int y, bool h40) {
  uint32_t base = (uint32_t)(genReg[3] & (h40 ? 0x3C : 0x3E)) << 10;
  int wc = h40 ? 64 : 32, cy = y >> 3;
  for (int x = x0; x < x1; x += 8) {
    uint16_t e = genVr16(base + (((uint32_t)cy * wc + (x >> 3)) << 1));
    int ty = y & 7; if (e & 0x1000) ty = 7 - ty;
    uint8_t attr = (uint8_t)(((e >> 8) & 0x80) | (((e >> 13) & 3) << 4));
    int n = x1 - x < 8 ? x1 - x : 8;
    genPutRun(buf + x, genTileRow(e & 0x7FF, ty), 0, n, (e & 0x800) != 0, attr);
  }
}

static void genDrawSprites(int y, int W) {
  bool h40 = W == 320;
  int maxSpr = h40 ? 20 : 16, maxTotal = h40 ? 80 : 64;
  uint32_t sat = (uint32_t)(genReg[5] & (h40 ? 0x7E : 0x7F)) << 9;
  const uint8_t *sp[20];
  int n = 0, idx = 0, guard = 0;
  do {
    const uint8_t *e = genVram + ((sat + idx * 8) & 0xFFF8);
    int sy = (((e[0] << 8) | e[1]) & 0x3FF) - 128;
    int vs = (e[2] & 3) + 1;
    if (y >= sy && y < sy + vs * 8) {
      if (n >= maxSpr) { genSticky |= 0x40; break; }
      sp[n++] = e;
    }
    idx = e[3] & 0x7F;
  } while (idx != 0 && idx < maxTotal && ++guard < maxTotal);
  if (!n) return;
  genSprDirty = true;
  for (int k = 0; k < n; k++) {
    const uint8_t *e = sp[k];
    int sy = (((e[0] << 8) | e[1]) & 0x3FF) - 128;
    int hs = ((e[2] >> 2) & 3) + 1, vs = (e[2] & 3) + 1;
    uint16_t attr = (uint16_t)((e[4] << 8) | e[5]);
    int sx = (((e[6] << 8) | e[7]) & 0x1FF) - 128;
    bool hf = (attr & 0x800) != 0, vf = (attr & 0x1000) != 0;
    uint8_t base = (uint8_t)(((attr >> 8) & 0x80) | (((attr >> 13) & 3) << 4));
    int py = y - sy; if (vf) py = vs * 8 - 1 - py;
    uint32_t tile = attr & 0x7FF;
    for (int col = 0; col < hs; col++) {
      int c = hf ? hs - 1 - col : col;
      uint32_t row = genTileRow(tile + c * vs + (py >> 3), py & 7);
      int xs = sx + col * 8;
      for (int i = 0; i < 8; i++) {
        int x = xs + i;
        if (x < 0 || x >= W) continue;
        uint32_t nib = hf ? (row >> (4 * i)) & 15 : (row >> (28 - 4 * i)) & 15;
        if (!nib) continue;
        if (genBufS[x] & 15) { genSticky |= 0x20; continue; }       // first sprite in the list wins
        genBufS[x] = (uint8_t)(base | nib);
      }
    }
  }
}

static void genRenderLine(int y, int r0) {
  bool h40 = (genReg[12] & 0x81) == 0x81;
  int W = h40 ? 320 : 256;
  uint8_t bg = genReg[7] & 0x3F;

  if (!(genReg[1] & 0x40)) memset(genLineIdx, bg, W);               // display off: backdrop only
  else {
    if (genSprDirty) { memset(genBufS, 0, 320); genSprDirty = false; }   // planes A / B overwrite every pixel themselves; only sprites need clearing, and only after a line that had some
    genDrawPlane(genBufB, 0, W, y, true);
    // plane A / window split
    uint8_t r17 = genReg[17], r18 = genReg[18];
    int wy = (r18 & 0x1F) * 8;
    bool vWin = (r18 & 0x80) ? (y >= wy) : (y < wy);
    int hp = (r17 & 0x1F) * 16; if (hp > W) hp = W;
    if (vWin)                 genDrawWindow(genBufA, 0, W, y, h40);
    else if (r17 & 0x80)    { if (hp > 0) genDrawPlane(genBufA, 0, hp, y, false); if (hp < W) genDrawWindow(genBufA, hp, W, y, h40); }
    else                    { if (hp > 0) genDrawWindow(genBufA, 0, hp, y, h40);  if (hp < W) genDrawPlane(genBufA, hp, W, y, false); }
    genDrawSprites(y, W);
    for (int x = 0; x < W; x++) {                                    // priority: S hi > A hi > B hi > S lo > A lo > B lo > backdrop
      uint8_t b = genBufB[x], a = genBufA[x], s = genBufS[x], o = bg;
      if ((b & 15) && !(b & 0x80)) o = b & 0x3F;
      if ((a & 15) && !(a & 0x80)) o = a & 0x3F;
      if ((s & 15) && !(s & 0x80)) o = s & 0x3F;
      if ((b & 15) &&  (b & 0x80)) o = b & 0x3F;
      if ((a & 15) &&  (a & 0x80)) o = a & 0x3F;
      if ((s & 15) &&  (s & 0x80)) o = s & 0x3F;
      genLineIdx[x] = o;
    }
  }
  const uint16_t *xm = h40 ? genXmap40 : genXmap32;
  uint16_t *dst = genRowSlot(r0);                                    // write straight into the LCD batch (no temp row + memcpy)
  for (int ox = 0; ox < GEN_OUT_W; ox++) dst[ox] = genPal[genLineIdx[xm[ox]]];
  genRowDone();
}

// =====================================================================================
//  Frame loop
// =====================================================================================
static void genRun(int target) {
  while (genLineCyc < target) {
    if (g68Stopped && genIrqLevel() <= g68Mask) { genLineCyc = target; break; }   // STOP: nothing to do until an interrupt
    genLineCyc += g68Step();
  }
}

static void genFrame(bool draw) {
  genFlushRows();
  for (int ln = 0; ln < GEN_LINES; ln++) {
    genLineNo = ln;
    if (ln == 0) { genVblank = false; genVintFlag = false; }
    if (ln == 224) { genVblank = true; genVintFlag = true; genVintPending = true; }
    if (ln < 224) {
      genRun(GEN_HBLANK_AT);
      if (draw) {
        int o0 = (ln * SH) / 224, o1 = ((ln + 1) * SH) / 224;        // 224 -> 172 lines: some lines are skipped (and not even rendered)
        if (o1 > o0) genRenderLine(ln, o0);
      }
      if (--genHcnt < 0) { genHcnt = genReg[10]; genHintPending = true; }
    } else genHcnt = genReg[10];
    genRun(GEN_LINE_CYC);
    genLineCyc -= GEN_LINE_CYC;
  }
  genFlushRows();
}

// =====================================================================================
//  Setup / loader
// =====================================================================================
static void genInit() {
  for (int i = 0; i < GEN_OUT_W; i++) { genXmap40[i] = (i * 320) / GEN_OUT_W; genXmap32[i] = (i * 256) / GEN_OUT_W; }
}

static void genReset() {
  memset(genRam, 0, 65536); memset(genVram, 0, 65536); memset(genZram, 0, 8192);
  memset(genReg, 0, sizeof(genReg)); memset(genVsram, 0, sizeof(genVsram));
  for (int i = 0; i < 64; i++) genCramWrite(i, 0);
  genVdpAddr = genVdpFirst = 0; genVdpCode = 0; genVdpPending = genFillPending = false; genSticky = 0;
  genVintPending = genHintPending = genVblank = genVintFlag = false;
  genHcnt = 0; genLineNo = 0; genLineCyc = 0;
  for (int i = 0; i < 3; i++) { genIoData[i] = 0x7F; genIoCtrl[i] = 0; }
  genZreq = false; genBatchN = 0;
  g68Reset();
}

static bool genAllocMem() {
  if (genRam) return true;
  genRam = (uint8_t *)malloc(65536); genVram = (uint8_t *)malloc(65536); genZram = (uint8_t *)malloc(8192);
  if (genRam && genVram && genZram) return true;
  free(genRam); free(genVram); free(genZram); genRam = genVram = genZram = nullptr;
  return false;
}

// Region from the header (0x1F0..0x1FF): J-only carts get the domestic version register so their region check passes.
static void genDetectRegion() {
  bool j = false, other = false;
  for (uint32_t a = 0x1F0; a < 0x200 && a < genRomSize; a++) {
    char c = (char)genRomRd8(a);
    if (c == 'J' || c == '1') j = true;
    else if (c == 'U' || c == 'E' || c == '4' || c == '8' || c == 'W' || c == 'A') other = true;
  }
  genVersion = (j && !other) ? 0x20 : 0xA0;
}

// Use a ROM that is already readable memory (for example a flash-mapped partition).
static bool genSetRom(const uint8_t *p, uint32_t size) {
  if (size < 0x200) { romError = "ROM file is too small"; return false; }
  if (!genAllocMem()) { romError = "Not enough RAM"; return false; }
  genRom = p; genRomSize = size; genDetectRegion();
  return true;
}

// Reads a .bin / .md / .gen / .smd file from the SD card into RAM (before the LCD is started).
static bool genLoadRom(const char *path) {
  if (!genAllocMem()) { romError = "Not enough RAM"; return false; }     // work memory first: whatever is left is for the ROM
  if (!sdMount()) { romError = "SD card not found"; return false; }
  File f = SD.open(path, FILE_READ);
  if (!f) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t fsz = f.size();
  if (fsz < 0x200) { romError = "ROM file is too small"; f.close(); sdUnmount(); return false; }
  bool smd = (fsz & 0x3FFF) == 512;                                      // Super Magic Drive dump: 512-byte header + interleaved blocks
  size_t skip = smd ? 512 : 0, sz = fsz - skip;
  uint8_t *buf = (uint8_t *)malloc(sz + 4);
  if (!buf) {
    snprintf(romErrBuf, sizeof(romErrBuf), "ROM too big (%u KB)", (unsigned)(sz / 1024));
    romError = romErrBuf; f.close(); sdUnmount(); return false;
  }
  if (skip) f.seek(skip);
  size_t got = 0;
  while (got < sz) {
    int n = f.read(buf + got, min((size_t)4096, sz - got));
    if (n <= 0) break;
    got += n;
  }
  f.close(); sdUnmount();
  if (got != sz) { free(buf); romError = "Could not read ROM"; return false; }
  memset(buf + sz, 0, 4);
  if (smd) {                                                             // de-interleave: first 8 KB of a block = odd bytes, second = even bytes
    uint8_t *tmp = (uint8_t *)malloc(16384);
    if (!tmp) { free(buf); romError = "Not enough RAM"; return false; }
    for (size_t b = 0; b + 16384 <= sz; b += 16384) {
      memcpy(tmp, buf + b, 16384);
      for (int i = 0; i < 8192; i++) { buf[b + 2 * i + 1] = tmp[i]; buf[b + 2 * i] = tmp[8192 + i]; }
    }
    free(tmp);
  }
  genRomHeap = buf; genRom = buf; genRomSize = sz;
  genDetectRegion();
  Serial.printf("Genesis ROM: %u KB%s, version reg %02X\n", (unsigned)(sz / 1024), smd ? " (SMD)" : "", genVersion);
  return true;
}

// ---------------------------------------------------------------------------------
//  SD streaming: open (before the LCD starts) and resume (after it has started)
// ---------------------------------------------------------------------------------
static void genStreamFail(const char *msg) {
  romError = msg;
  if (genFile) genFile.close();
  sdUnmount();
  for (int i = 0; i < genNPages; i++) { free(genPageBuf[i]); genPageBuf[i] = nullptr; }
  genNPages = 0;
  free(genSmdTmp); genSmdTmp = nullptr;
}

// Opens the ROM, checks it, sizes the page cache from the free heap, reads the header page, then releases the card.
static bool genStreamOpen(const char *path) {
  if (!genAllocMem()) { romError = "Not enough RAM"; return false; }
  if (!sdMount()) { romError = "SD card not found"; return false; }
  genFile = SD.open(path, FILE_READ);
  if (!genFile) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t fsz = genFile.size();
  if (fsz < 0x200) { genStreamFail("ROM file is too small"); return false; }
  genSmd = (fsz & 0x3FFF) == 512;
  genFileSkip = genSmd ? 512 : 0;
  genRomSize = (uint32_t)(fsz - genFileSkip);
  if (genRomSize > 0x400000) genRomSize = 0x400000;                // the cartridge window is 4 MB
  strncpy(genRomPath, path, sizeof(genRomPath) - 1); genRomPath[sizeof(genRomPath) - 1] = 0;
  genRom = nullptr;                                                // streaming mode: genRomRd8/16 use the cache
  if (genSmd) { genSmdTmp = (uint8_t *)malloc(GEN_PG_SIZE); if (!genSmdTmp) { genStreamFail("Not enough RAM"); return false; } }

  uint32_t want = (genRomSize + GEN_PG_SIZE - 1) >> GEN_PG_BITS;   // pages the whole ROM would need
  if (want > GEN_CACHE_MAX) want = GEN_CACHE_MAX;
  genNPages = 0;
  while ((uint32_t)genNPages < want && ESP.getFreeHeap() > GEN_MIN_FREE_HEAP + GEN_PG_SIZE) {
    uint8_t *pg = (uint8_t *)malloc(GEN_PG_SIZE);
    if (!pg) break;
    genPageBuf[genNPages] = pg; genPageTag[genNPages] = 0xFFFF; genPageUse[genNPages] = 0; genNPages++;
  }
  if ((uint32_t)genNPages < (want < GEN_CACHE_MIN ? want : GEN_CACHE_MIN)) { genStreamFail("Not enough RAM for ROM cache"); return false; }
  genUseClock = 0; genReadErrors = 0;
  genFrontIdx[0] = genFrontIdx[1] = -1; genFrontPg[0] = genFrontPg[1] = 0xFFFFFFFFu;

  genDetectRegion();                                               // reads the header through the cache (page 0 stays resident)
  genFile.close(); sdUnmount();                                    // the LCD needs the SPI pins next
  Serial.printf("Genesis ROM (SD stream): %u KB%s, cache %d x 4 KB, version reg %02X\n",
                (unsigned)(genRomSize / 1024), genSmd ? " (SMD)" : "", genNPages, genVersion);
  return true;
}

// Call after the LCD has been started: mounts the card again on the shared SPI bus and re-opens the ROM.
// (Deliberately not sdMount(): on failure that would call SPI.end() and kill the LCD.)
static bool genStreamResume() {
  if (genRom) return true;                                         // RAM copy / flash-mapped ROM: nothing to do
  pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, GEN_SD_HZ)) { romError = "SD card lost"; return false; }
  genFile = SD.open(genRomPath, FILE_READ);
  if (genAfterSd) genAfterSd();                                    // mounting also changed the bus settings
  if (!genFile) { romError = "ROM not found on SD"; return false; }
  return true;
}
