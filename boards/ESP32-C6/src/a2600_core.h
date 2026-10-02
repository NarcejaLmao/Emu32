// =====================================================================================
//  a2600_core.h  -  Atari 2600 (VCS) emulation core for Emu32            [integrated in emu32.ino]
// =====================================================================================
//  Header-only and hardware independent (no Arduino calls), so it sits in the
//  "src" folder next to emu32.ino (#included from there), or can be compiled on a PC for testing.  Everything is `static`
//  and prefixed a26 / A26_, so it cannot collide with the NES / Game Boy code in emu32.ino.
//
//  What is emulated
//    - 6507 CPU: all official opcodes, NMOS decimal mode (games use BCD for scores) and the
//      common illegal opcodes (NOPs, LAX, SAX, SLO, RLA, SRE, RRA, DCP, ISC, ANC, ALR, SBX).
//      Cycle exact: every bus access is one tick, including dummy cycles and page-cross penalties.
//    - TIA: playfield, 2 players, 2 missiles, ball, NUSIZ copies, REFP, VDELP/VDELBL, RESMP,
//      HMOVE/HMCLR (+ the 8 pixel HMOVE blank), all 15 collision flags, priority / score mode,
//      WSYNC, VSYNC/VBLANK, joystick triggers with latch.  Pixels are rendered lazily: the
//      line is drawn up to the current colour clock before every TIA write/read, so mid-line
//      register changes (sprite kernels, "racing the beam") work.
//    - RIOT (6532): 128 bytes RAM, interval timer (1/8/64/1024 + underflow), SWCHA/SWCHB.
//    - Cartridges: 2K, 4K, F8 (8K), FA (12K), F6 (16K), F4 (32K) [+Superchip RAM], E0 (8K), 3F.
//      The mapper is auto-detected from the ROM size / contents.
//    - NOT emulated: TIA audio (the registers are stored in a26Aud[] only), paddles / keypads,
//      PAL palette (PAL timing is detected, colours stay NTSC), FE / E7 / F0 / DPC / Supercharger.
//
//  How emu32.ino uses it (see the ATARI 2600 glue section there)
//      a26RowFn = myRowCallback;                    // void myRowCallback(int y, const uint16_t *rgb565)
//      if (!a26Load(romBuf, romSize)) show(a26Error);   // romBuf must stay allocated while running
//      a26Reset();
//      loop:  a26Pad0 = ...;  a26Pad1 = ...;  a26Console = ...;
//             a26Frame(true);                       // runs one video frame, calls a26RowFn per row
//             wait until a26FrameUs() has passed
//
//  The callback receives A26_H (192) rows of A26_W (160) RGB565 pixels, y = 0 is the top row.
//  The 2600's pixels are wide: a 160x192 picture is shown at 4:3 as roughly 229x172 on the LCD,
//  so scale x by ~1.43 (e.g. a 229 entry x-map like the NES xmap[]) and y by ~0.9.
// =====================================================================================
#pragma once
#include <stdint.h>
#include <string.h>

// ------------------------------------------------------------------ public configuration / API
#define A26_W           160       // visible pixels per row
#define A26_H           192       // rows handed to the row callback
#define A26_MAX_LINES   320       // safety: a frame is forced to end after this many scanlines
#define A26_TOP_MIN     30        // auto picture-top detection is clamped to [A26_TOP_MIN, A26_TOP_MAX]
#define A26_TOP_MAX     100
#define A26_TOP_DEFAULT 37        // used when the game never turns VBLANK off

// joystick bits for a26Pad0 / a26Pad1 (1 = pressed)
#define A26_UP     0x01
#define A26_DOWN   0x02
#define A26_LEFT   0x04
#define A26_RIGHT  0x08
#define A26_FIRE   0x10
// console switch bits for a26Console (1 = pressed / on)
#define A26_RESET   0x01          // game reset button
#define A26_SELECT  0x02          // game select button
#define A26_BW      0x08          // TV type switch: 1 = black & white (palette is not changed)
#define A26_DIFF0A  0x40          // left difficulty  switch in position A
#define A26_DIFF1A  0x80          // right difficulty switch in position A

// A26_FAST: put the hot functions in internal RAM (IRAM) on the ESP32 so they never wait for the flash cache.
// Costs roughly 20-30 KB of RAM. Define A26_NO_IRAM before including this file to turn it off.
#if defined(IRAM_ATTR) && !defined(A26_NO_IRAM)
  #define A26_FAST IRAM_ATTR
#else
  #define A26_FAST
#endif

typedef void (*A26RowFn)(int y, const uint16_t *rgb565);

static A26RowFn a26RowFn = nullptr;               // called once per visible row (only when a26Frame(true))
static volatile uint8_t a26Pad0 = 0, a26Pad1 = 0; // player 0 / 1 joystick
static volatile uint8_t a26Console = 0;           // console switches
static const char *a26Error = nullptr;            // set when a26Load() fails
static uint16_t a26Rgb[128];                      // palette (RGB565), index = TIA colour >> 1
static bool a26Pal = false;                       // true once a ~50 Hz (>287 lines) frame was seen
static uint32_t a26FrameCount = 0;
static int a26FrameLines = 262;                   // scanlines in the last frame
static uint8_t a26Aud[6];                         // AUDC0 AUDC1 AUDF0 AUDF1 AUDV0 AUDV1 (no sound generated)

static inline uint32_t a26FrameUs() { return a26Pal ? 20056 : 16683; }   // time one frame should take

// ------------------------------------------------------------------ NTSC palette (RGB888, 16 hues x 8 luminances)
static const uint32_t A26_NTSC[128] = {
  0x000000, 0x4a4a4a, 0x6f6f6f, 0x8e8e8e, 0xaaaaaa, 0xc0c0c0, 0xd6d6d6, 0xececec,
  0x484800, 0x69690f, 0x86861d, 0xa2a22a, 0xbbbb35, 0xd2d240, 0xe8e84a, 0xfcfc54,
  0x7c2c00, 0x904811, 0xa26221, 0xb47a30, 0xc3903d, 0xd2a44a, 0xdfb755, 0xecc860,
  0x901c00, 0xa33915, 0xb55328, 0xc66c3a, 0xd5824a, 0xe39759, 0xf0aa67, 0xfcbc74,
  0x940000, 0xa71a1a, 0xb83232, 0xc84848, 0xd65c5c, 0xe46f6f, 0xf08080, 0xfc9090,
  0x840064, 0x97197a, 0xa8308f, 0xb846a2, 0xc659b3, 0xd46cc3, 0xe07cd2, 0xec8ce0,
  0x500084, 0x68199a, 0x7d30ad, 0x9246c0, 0xa459d0, 0xb56ce0, 0xc57cee, 0xd48cfc,
  0x140090, 0x331aa3, 0x4e32b5, 0x6848c6, 0x7f5cd5, 0x956fe3, 0xa980f0, 0xbc90fc,
  0x000094, 0x181aa7, 0x2d32b8, 0x4248c8, 0x545cd6, 0x656fe4, 0x7580f0, 0x8490fc,
  0x001c88, 0x183b9d, 0x2d57b0, 0x4272c2, 0x548ad2, 0x65a0e1, 0x75b5ef, 0x84c8fc,
  0x003064, 0x185080, 0x2d6d98, 0x4288b0, 0x54a0c5, 0x65b7d9, 0x75cceb, 0x84e0fc,
  0x004030, 0x18624e, 0x2d8169, 0x429e82, 0x54b899, 0x65d1ae, 0x75e7c2, 0x84fcd4,
  0x004400, 0x1a661a, 0x328432, 0x48a048, 0x5cba5c, 0x6fd26f, 0x80e880, 0x90fc90,
  0x143c00, 0x355f18, 0x527e2d, 0x6e9c42, 0x87b754, 0x9ed065, 0xb4e775, 0xc8fc84,
  0x303800, 0x505916, 0x6d762b, 0x88923e, 0xa0ab4f, 0xb7c25f, 0xccd86e, 0xe0ec7c,
  0x482c00, 0x694d14, 0x866a26, 0xa28638, 0xbb9f47, 0xd2b656, 0xe8cc63, 0xfce070
};

// ------------------------------------------------------------------ internal state: time
static uint32_t a26Cyc = 0;          // CPU cycles since reset (RIOT timer reference)
static int a26LineCyc = 0;           // CPU cycle inside the current scanline, 0..75 (1 CPU cycle = 3 colour clocks)
static int a26Line = 0;              // scanline since the last VSYNC
static int a26Top = A26_TOP_DEFAULT; // first scanline handed to the row callback
static int a26FirstNB = -1;          // first scanline of this frame with VBLANK off
static bool a26FrameDone = false, a26Draw = true;

static A26_FAST void a26EndLine();   // forward declarations (CPU ticks drive the TIA)
static A26_FAST void a26FrameEnd(bool vsync);

static inline void a26Tick() {       // one CPU cycle elapsed
  a26Cyc++;
  if (++a26LineCyc >= 76) a26EndLine();
}

// ------------------------------------------------------------------ cartridge
enum { A26_M_2K, A26_M_4K, A26_M_F8, A26_M_F6, A26_M_F4, A26_M_FA, A26_M_E0, A26_M_3F };
static const uint8_t *a26Rom = nullptr;
static uint32_t a26RomSize = 0;
static uint8_t a26Map = A26_M_4K;
static bool a26SC = false;           // Superchip RAM present (F8/F6/F4)
static bool a26HasX = false;         // any cartridge RAM at all (Superchip or FA)
static uint8_t a26Xram[256];         // Superchip (128 B) or FA (256 B) cartridge RAM
static const uint8_t *a26Seg[4];     // four 1 KB windows at $1000, $1400, $1800, $1C00

static void a26Set4K(uint32_t bank) {
  uint32_t n = a26RomSize / 4096; if (!n) n = 1;
  const uint8_t *b = a26Rom + (bank % n) * 4096UL;
  for (int i = 0; i < 4; i++) a26Seg[i] = b + i * 1024;
}
static void a26Set3F(uint32_t bank) {              // lower 2 KB switchable, upper 2 KB fixed to the last bank
  uint32_t n = a26RomSize / 2048;
  const uint8_t *lo = a26Rom + (bank % n) * 2048UL, *hi = a26Rom + a26RomSize - 2048;
  a26Seg[0] = lo; a26Seg[1] = lo + 1024; a26Seg[2] = hi; a26Seg[3] = hi + 1024;
}
static void a26CartInit() {
  memset(a26Xram, 0, sizeof(a26Xram));
  a26HasX = a26SC || a26Map == A26_M_FA;
  switch (a26Map) {
    case A26_M_2K: a26Seg[0] = a26Seg[2] = a26Rom; a26Seg[1] = a26Seg[3] = a26Rom + 1024; break;
    case A26_M_4K: a26Set4K(0); break;
    case A26_M_F8: case A26_M_F6: case A26_M_F4: a26Set4K(a26RomSize / 4096 - 1); break;
    case A26_M_FA: a26Set4K(2); break;
    case A26_M_E0: for (int i = 0; i < 4; i++) a26Seg[i] = a26Rom + (4 + i) * 1024UL; break;
    case A26_M_3F: a26Set3F(0); break;
  }
}
static inline void a26Hot(uint16_t a) {            // bank-switch hot spots (any read or write triggers them)
  switch (a26Map) {
    case A26_M_F8: if (a >= 0xFF8 && a <= 0xFF9) a26Set4K(a - 0xFF8); break;
    case A26_M_FA: if (a >= 0xFF8 && a <= 0xFFA) a26Set4K(a - 0xFF8); break;
    case A26_M_F6: if (a >= 0xFF6 && a <= 0xFF9) a26Set4K(a - 0xFF6); break;
    case A26_M_F4: if (a >= 0xFF4 && a <= 0xFFB) a26Set4K(a - 0xFF4); break;
    case A26_M_E0:
      if (a >= 0xFE0 && a <= 0xFF7) {
        int seg = (a - 0xFE0) >> 3;                // 0..2: which 1 KB window, low 3 bits: which 1 KB slice
        a26Seg[seg] = a26Rom + (uint32_t)(a & 7) * 1024UL;
      }
      break;
    default: break;
  }
}
static inline uint8_t a26CartRead(uint16_t a) {
  a &= 0xFFF;
  if (a >= 0xFE0) a26Hot(a);                                            // every bank-switch hot spot lives in $FE0-$FFF
  if (a26HasX) {
    if (a26SC && a >= 0x80 && a < 0x100) return a26Xram[a - 0x80];      // Superchip read port
    if (a26Map == A26_M_FA && a >= 0x100 && a < 0x200) return a26Xram[a - 0x100];
  }
  return a26Seg[a >> 10][a & 0x3FF];
}
static inline void a26CartWrite(uint16_t a, uint8_t v) {
  a &= 0xFFF;
  if (a >= 0xFE0) a26Hot(a);
  if (a26HasX) {
    if (a26SC && a < 0x80) a26Xram[a] = v;                              // Superchip write port
    else if (a26Map == A26_M_FA && a < 0x100) a26Xram[a] = v;
  }
}

static uint32_t a26CountSig(const uint8_t *sig, int n) {
  uint32_t cnt = 0;
  for (uint32_t i = 0; i + n <= a26RomSize; i++) if (!memcmp(a26Rom + i, sig, n)) cnt++;
  return cnt;
}
static bool a26DetectSC(uint32_t banks) {          // Superchip carts repeat 128 bytes at the start of every 4K bank
  for (uint32_t b = 0; b < banks; b++) {
    const uint8_t *p = a26Rom + b * 4096UL;
    if (memcmp(p, p + 128, 128)) return false;
  }
  return true;
}

// ------------------------------------------------------------------ RIOT (6532): RAM, timer, ports
static uint8_t a26Ram[128];
static uint8_t a26SwchaOut = 0, a26SwchaDdr = 0, a26SwchbDdr = 0, a26SwchbOut = 0;
static uint8_t a26TmV = 0xFF, a26TmSh = 10;        // timer start value / log2(interval)
static uint32_t a26TmStart = 0;                    // CPU cycle the timer was (re)started

static uint8_t a26SwchaIn() {
  uint8_t p0 = a26Pad0, p1 = a26Pad1, s = 0xFF;
  if (p0 & A26_UP)    s &= ~0x10;
  if (p0 & A26_DOWN)  s &= ~0x20;
  if (p0 & A26_LEFT)  s &= ~0x40;
  if (p0 & A26_RIGHT) s &= ~0x80;
  if (p1 & A26_UP)    s &= ~0x01;
  if (p1 & A26_DOWN)  s &= ~0x02;
  if (p1 & A26_LEFT)  s &= ~0x04;
  if (p1 & A26_RIGHT) s &= ~0x08;
  return s;
}
static uint8_t a26SwchbIn() {
  uint8_t c = a26Console, s = 0x34;                // bits 2,4,5 are unused (read as 1)
  if (!(c & A26_RESET)) s |= 0x01;
  if (!(c & A26_SELECT)) s |= 0x02;
  if (!(c & A26_BW)) s |= 0x08;                    // 1 = colour
  return s | (c & 0xC0);
}
static A26_FAST uint8_t a26RiotRead(uint16_t a) {
  if (a & 4) {
    uint32_t d = a26Cyc - a26TmStart, lim = (uint32_t)(a26TmV + 1) << a26TmSh;
    if (a & 1) return (d >= lim) ? 0x80 : 0x00;    // TIMINT: underflow flag
    if (d < lim) return (uint8_t)(a26TmV - (d >> a26TmSh));
    uint8_t v = (uint8_t)(0xFF - (d - lim));       // after underflow it counts down every cycle...
    a26TmV = v; a26TmStart = a26Cyc;               // ...until read: the flag clears and the prescaler is back
    return v;
  }
  switch (a & 3) {
    case 0: return (a26SwchaIn() & ~a26SwchaDdr) | (a26SwchaOut & a26SwchaDdr);
    case 1: return a26SwchaDdr;
    case 2: return (a26SwchbIn() & ~a26SwchbDdr) | (a26SwchbOut & a26SwchbDdr);
    default: return a26SwchbDdr;
  }
}
static A26_FAST void a26RiotWrite(uint16_t a, uint8_t v) {
  if (a & 4) {
    if (a & 0x10) {                                // TIM1T / TIM8T / TIM64T / T1024T
      static const uint8_t sh[4] = {0, 3, 6, 10};
      a26TmV = v; a26TmSh = sh[a & 3]; a26TmStart = a26Cyc;
    }                                              // else: edge detect control (unused)
    return;
  }
  switch (a & 3) {
    case 0: a26SwchaOut = v; break;
    case 1: a26SwchaDdr = v; break;
    case 2: a26SwchbOut = v; break;
    default: a26SwchbDdr = v; break;
  }
}

// ------------------------------------------------------------------ TIA
static uint8_t a26Vsync = 0, a26Vblank = 0, a26Nusiz[2], a26Refp[2], a26ColuP[2], a26ColuPF, a26ColuBK, a26CtrlPF;
static uint8_t a26Pf[3];
static uint8_t a26Grp[2], a26GrpOld[2], a26Enam[2], a26Enabl, a26EnablOld, a26Hm[5], a26Vdel[3], a26Resmp[2];
static int16_t a26PosP[2], a26PosM[2], a26PosB;
static uint64_t a26Cx;                      // collision latches, register r in bits 8r..8r+7
static uint8_t a26Latch[2];                 // INPT4/5 latches
static bool a26Wsync = false, a26HmoveBlank = false;
static int a26RendX = 0;                    // colour clocks of the current line already rendered
static uint16_t a26Row[A26_W];

// values derived from the registers (see a26Derive)
static uint8_t a26EP[2], a26EM[2], a26EB, a26MW[2], a26BW;
static uint32_t a26PfL, a26PfR;
static bool a26Any;                         // any movable object visible?

static const uint8_t A26_NC[8]     = {1, 2, 2, 3, 2, 1, 3, 1};                        // copies per NUSIZ mode
static const uint8_t A26_OFF[8][3] = {{0,0,0},{0,16,0},{0,32,0},{0,16,32},{0,64,0},{0,0,0},{0,32,64},{0,0,0}};
static const uint8_t A26_PSH[8]    = {0, 0, 0, 0, 0, 1, 0, 2};                         // player size shift (x1, x2, x4)

// collision tables: object mask bits P0=1 M0=2 P1=4 M1=8 PF=16 BL=32
static uint64_t a26CollTab[64];
static void a26BuildCollTab() {
  static const uint8_t pairs[15][4] = {          // {objA, objB, register, bit}
    {2, 4, 0, 7}, {2, 1, 0, 6},  {8, 1, 1, 7},  {8, 4, 1, 6},
    {1, 16, 2, 7}, {1, 32, 2, 6}, {4, 16, 3, 7}, {4, 32, 3, 6},
    {2, 16, 4, 7}, {2, 32, 4, 6}, {8, 16, 5, 7}, {8, 32, 5, 6},
    {32, 16, 6, 7}, {1, 4, 7, 7}, {2, 8, 7, 6}};
  for (int m = 0; m < 64; m++) {
    uint64_t v = 0;
    for (int i = 0; i < 15; i++)
      if ((m & pairs[i][0]) && (m & pairs[i][1])) v |= (uint64_t)1 << (pairs[i][2] * 8 + pairs[i][3]);
    a26CollTab[m] = v;
  }
}

static inline uint8_t a26Rev8(uint8_t b) {
  b = (b >> 4) | (b << 4);
  b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
  return ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
}

static A26_FAST void a26Derive() {
  for (int i = 0; i < 2; i++) {
    a26EP[i] = a26Vdel[i] ? a26GrpOld[i] : a26Grp[i];
    int psh = A26_PSH[a26Nusiz[i] & 7];
    a26MW[i] = 1 << ((a26Nusiz[i] >> 4) & 3);
    if (a26Resmp[i]) a26PosM[i] = (a26PosP[i] + ((8 << psh) >> 1)) % 160;   // locked to the middle of the player
    a26EM[i] = (a26Enam[i] & 2) && !a26Resmp[i];
  }
  a26EB = ((a26Vdel[2] ? a26EnablOld : a26Enabl) & 2) != 0;
  a26BW = 1 << ((a26CtrlPF >> 4) & 3);
  uint32_t l = ((a26Pf[0] >> 4) & 0xF) | ((uint32_t)a26Rev8(a26Pf[1]) << 4) | ((uint32_t)a26Pf[2] << 12);
  uint32_t r = l;
  if (a26CtrlPF & 1)                                                          // reflected right half
    r = a26Rev8(a26Pf[2]) | ((uint32_t)a26Pf[1] << 8) | ((uint32_t)(a26Rev8((a26Pf[0] >> 4) & 0xF) >> 4) << 16);
  a26PfL = l; a26PfR = r;
  a26Any = a26EP[0] || a26EP[1] || a26EM[0] || a26EM[1] || a26EB;
}

// ---- pixel renderer ---------------------------------------------------------------------------
// Instead of asking "which objects cover pixel x?" for every pixel, each object paints its own pixels into
// a26M[] (one mask byte per pixel: P0=1 M0=2 P1=4 M1=8 PF=16 BL=32), then one pass turns the masks into
// collisions and colours.  Same result as the old per-pixel code, a lot less work per pixel.
static uint8_t a26M[A26_W];

static inline void a26FillRun(int s, int w, int x0, int x1, uint8_t bit) {   // object at s (0..159), w pixels, wraps at 160
  int e = s + w;
  int a = s > x0 ? s : x0, b = e < x1 ? e : x1;
  for (int x = a; x < b; x++) a26M[x] |= bit;
  if (e > A26_W) {
    b = e - A26_W; if (b > x1) b = x1;
    for (int x = x0; x < b; x++) a26M[x] |= bit;
  }
}
static inline void a26SpriteSeg(int x0, int x1, int from, int to, int j, int sh, uint8_t g, bool rf, uint8_t bit) {
  if (from < x0) { j += x0 - from; from = x0; }                       // pixel x shows sprite column j
  if (to > x1) to = x1;
  for (int x = from; x < to; x++, j++) {
    int b = j >> sh;
    if ((g >> (rf ? b : 7 - b)) & 1) a26M[x] |= bit;
  }
}

static A26_FAST void a26Pixels(int x0, int x1) {
  if (!a26Draw && !a26Any && !(a26PfL | a26PfR)) return;             // frame skip + nothing that could collide
  const bool vb = (a26Vblank & 2) != 0;
  const uint16_t black = a26Rgb[0], bkc = a26Rgb[a26ColuBK];
  const int hb = a26HmoveBlank ? 8 : 0;                              // pixels left of hb are forced black
  if (!a26Any && !(a26PfL | a26PfR)) {                               // fast path: background only
    for (int x = x0; x < x1; x++) a26Row[x] = (vb || x < hb) ? black : bkc;
    return;
  }
  memset(a26M + x0, 0, x1 - x0);

  if (a26PfL | a26PfR) {                                             // playfield: 4 pixel wide blocks
    for (int b = x0 >> 2, be = (x1 - 1) >> 2; b <= be; b++) {
      if (!((b < 20) ? (a26PfL >> b) & 1 : (a26PfR >> (b - 20)) & 1)) continue;
      int a = b * 4, e = a + 4; if (a < x0) a = x0; if (e > x1) e = x1;
      for (int x = a; x < e; x++) a26M[x] |= 16;
    }
  }
  if (a26Any) {
    for (int i = 0; i < 2; i++) {
      const int mode = a26Nusiz[i] & 7, nc = A26_NC[mode];
      if (a26EP[i]) {                                                // player i
        const int sh = A26_PSH[mode], w = 8 << sh;
        const uint8_t g = a26EP[i], bit = i ? 4 : 1;
        const bool rf = a26Refp[i] != 0;
        for (int c = 0; c < nc; c++) {
          int s = a26PosP[i] + A26_OFF[mode][c]; if (s >= A26_W) s -= A26_W;
          a26SpriteSeg(x0, x1, s, s + w < A26_W ? s + w : A26_W, 0, sh, g, rf, bit);
          if (s + w > A26_W) a26SpriteSeg(x0, x1, 0, s + w - A26_W, A26_W - s, sh, g, rf, bit);
        }
      }
      if (a26EM[i]) {                                                // missile i
        for (int c = 0; c < nc; c++) {
          int s = a26PosM[i] + A26_OFF[mode][c]; if (s >= A26_W) s -= A26_W;
          a26FillRun(s, a26MW[i], x0, x1, i ? 8 : 2);
        }
      }
    }
    if (a26EB) a26FillRun(a26PosB, a26BW, x0, x1, 32);
  }

  const bool prio = (a26CtrlPF & 4) != 0, score = (a26CtrlPF & 2) != 0, draw = a26Draw;
  uint64_t cx = a26Cx;
  unsigned lastKey = 0xFFFF; uint16_t lastC = 0;
  for (int x = x0; x < x1; x++) {
    const unsigned m = a26M[x];
    if (m & (m - 1)) cx |= a26CollTab[m];                            // two or more objects on this pixel
    if (!draw) continue;                                             // skipped frame: collisions only
    if (vb || x < hb) { a26Row[x] = black; continue; }
    const bool left = x < 80;
    const unsigned key = m | (left ? 0 : 64);
    if (key != lastKey) {                                            // colour only changes when the mask changes
      lastKey = key;
      uint8_t ci;
      const bool p0 = m & 3, p1 = m & 12, pf = m & 16, bl = m & 32;
      const uint8_t pfc = score ? a26ColuP[left ? 0 : 1] : a26ColuPF;
      if (prio) ci = pf ? pfc : bl ? a26ColuPF : p0 ? a26ColuP[0] : p1 ? a26ColuP[1] : a26ColuBK;
      else      ci = p0 ? a26ColuP[0] : p1 ? a26ColuP[1] : pf ? pfc : bl ? a26ColuPF : a26ColuBK;
      lastC = a26Rgb[ci];
    }
    a26Row[x] = lastC;
  }
  a26Cx = cx;
}

static inline void a26Sync(int clk) {               // render colour clocks [a26RendX, clk)
  if (clk > 228) clk = 228;
  if (clk <= a26RendX) return;
  int x0 = a26RendX - 68, x1 = clk - 68;
  a26RendX = clk;
  if (x1 <= 0) return;                              // still in horizontal blank
  if (x0 < 0) x0 = 0;
  a26Pixels(x0, x1);
}

static A26_FAST void a26EndLine() {
  a26Sync(228);
  if (!(a26Vblank & 2) && a26FirstNB < 0) a26FirstNB = a26Line;
  int y = a26Line - a26Top;
  if (a26Draw && a26RowFn && y >= 0 && y < A26_H) a26RowFn(y, a26Row);
  a26Line++; a26LineCyc = 0; a26RendX = 0; a26HmoveBlank = false;
  if (a26Line >= A26_MAX_LINES) a26FrameEnd(false);   // game never sends VSYNC: force a frame boundary
}
static A26_FAST void a26FrameEnd(bool vsync) {
  if (vsync) { a26FrameLines = a26Line; a26Pal = a26Line > 287; }   // only trust the length of VSYNC-delimited frames
  if (a26FirstNB >= 0) {                            // follow the game's own picture start (applies from the next frame)
    int t = a26FirstNB < A26_TOP_MIN ? A26_TOP_DEFAULT : a26FirstNB;
    a26Top = t > A26_TOP_MAX ? A26_TOP_MAX : t;
  }
  a26FirstNB = -1; a26Line = 0; a26FrameDone = true; a26FrameCount++;
}

static inline int a26RespPos(int clk, int off, int inHblank) {   // where an object lands when its RESxx is strobed
  int h = clk - 68;
  return h < 0 ? inHblank : (h + off) % 160;
}

static A26_FAST void a26TiaWrite(uint8_t r, uint8_t v) {
  const int clk = a26LineCyc * 3;
  r &= 0x3F;
  // Everything before this write must be drawn with the old state.  Registers that cannot change the picture
  // (WSYNC, RSYNC, audio, HMxx, HMCLR) skip that work.
  if (!(r == 0x02 || r == 0x03 || (r >= 0x15 && r <= 0x1A) || (r >= 0x20 && r <= 0x24) || r == 0x2B)) a26Sync(clk);
  switch (r) {
    case 0x00:                                      // VSYNC: the falling edge of bit 1 ends the frame
      if ((a26Vsync & 2) && !(v & 2) && a26Line >= 50) a26FrameEnd(true);
      a26Vsync = v; return;
    case 0x01: a26Vblank = v; if (!(v & 0x40)) a26Latch[0] = a26Latch[1] = 0x80; return;
    case 0x02: if (a26LineCyc) a26Wsync = true; return;          // WSYNC: CPU halts until the line ends
    case 0x03: return;                                           // RSYNC
    case 0x04: case 0x05: a26Nusiz[r - 4] = v & 0x37; break;
    case 0x06: case 0x07: a26ColuP[r - 6] = (v >> 1) & 0x7F; return;
    case 0x08: a26ColuPF = (v >> 1) & 0x7F; return;
    case 0x09: a26ColuBK = (v >> 1) & 0x7F; return;
    case 0x0A: a26CtrlPF = v; break;
    case 0x0B: case 0x0C: a26Refp[r - 0x0B] = v & 8; return;
    case 0x0D: case 0x0E: case 0x0F: a26Pf[r - 0x0D] = v; break;
    case 0x10: case 0x11: a26PosP[r - 0x10] = a26RespPos(clk, 5, 3); break;
    case 0x12: case 0x13: a26PosM[r - 0x12] = a26RespPos(clk, 4, 2); break;
    case 0x14: a26PosB = a26RespPos(clk, 4, 2); break;
    case 0x15: case 0x16: case 0x17: case 0x18: case 0x19: case 0x1A: a26Aud[r - 0x15] = v; return;
    case 0x1B: a26Grp[0] = v; a26GrpOld[1] = a26Grp[1]; break;                          // GRP0 also latches GRP1's old copy
    case 0x1C: a26Grp[1] = v; a26GrpOld[0] = a26Grp[0]; a26EnablOld = a26Enabl; break;  // GRP1 latches GRP0 and ENABL
    case 0x1D: case 0x1E: a26Enam[r - 0x1D] = v; break;
    case 0x1F: a26Enabl = v; break;
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: a26Hm[r - 0x20] = v; return;
    case 0x25: case 0x26: case 0x27: a26Vdel[r - 0x25] = v & 1; break;
    case 0x28: case 0x29: a26Resmp[r - 0x28] = v & 2; break;
    case 0x2A: {                                    // HMOVE: shift every object by its HM value (+n = n pixels left)
      int16_t *pos[5] = {&a26PosP[0], &a26PosP[1], &a26PosM[0], &a26PosM[1], &a26PosB};
      for (int i = 0; i < 5; i++) {
        int n = a26Hm[i] >> 4; if (n >= 8) n -= 16;
        int p = (*pos[i] - n) % 160; if (p < 0) p += 160;
        *pos[i] = p;
      }
      if (clk < 68) a26HmoveBlank = true;           // strobed during HBLANK: the first 8 pixels stay black
      break;
    }
    case 0x2B: memset(a26Hm, 0, sizeof(a26Hm)); return;                                 // HMCLR
    case 0x2C: a26Cx = 0; return;                                                       // CXCLR
    default: return;
  }
  a26Derive();
}

static A26_FAST uint8_t a26TiaRead(uint16_t a) {
  uint8_t r = a & 0x0F;
  if (r < 8) { a26Sync(a26LineCyc * 3); return (uint8_t)(a26Cx >> (r * 8)); }          // collision flags
  if (r < 12) return 0x80;                                                              // paddle inputs (not connected)
  if (r < 14) {                                                                         // INPT4/5: joystick triggers
    int i = r - 12;
    uint8_t live = ((i ? a26Pad1 : a26Pad0) & A26_FIRE) ? 0x00 : 0x80;
    if (a26Vblank & 0x40) { a26Latch[i] &= live; return a26Latch[i]; }                 // latched mode
    return live;
  }
  return 0;
}

// ------------------------------------------------------------------ bus
static inline uint8_t a26Read(uint16_t a) {
  a &= 0x1FFF;                                      // the 6507 only has 13 address lines
  if (a & 0x1000) return a26CartRead(a);
  if (!(a & 0x80)) return a26TiaRead(a);
  if (!(a & 0x200)) return a26Ram[a & 0x7F];
  return a26RiotRead(a);
}
static inline void a26Write(uint16_t a, uint8_t v) {
  a &= 0x1FFF;
  if (a & 0x1000) { a26CartWrite(a, v); return; }
  if (!(a & 0x80)) {
    a26TiaWrite(a & 0x3F, v);
    if (a26Map == A26_M_3F && (a & 0x7F) <= 0x3F) a26Set3F(v);   // Tigervision: any write to $00-$3F selects the bank
    return;
  }
  if (!(a & 0x200)) { a26Ram[a & 0x7F] = v; return; }
  a26RiotWrite(a, v);
}
static inline uint8_t a26Rd(uint16_t a) { a26Tick(); return a26Read(a); }        // one bus cycle each
static inline void a26Wr(uint16_t a, uint8_t v) { a26Tick(); a26Write(a, v); }
static inline void a26Idle() { a26Tick(); }

// ------------------------------------------------------------------ 6507 CPU
#define A26_FC 0x01
#define A26_FZ 0x02
#define A26_FI 0x04
#define A26_FD 0x08
#define A26_FB 0x10
#define A26_FU 0x20
#define A26_FV 0x40
#define A26_FN 0x80

static uint8_t a26A, a26X, a26Y, a26S, a26P;
static uint16_t a26PC;

static inline void a26NZ(uint8_t v) { a26P = (a26P & ~(A26_FN | A26_FZ)) | (v & 0x80) | (v ? 0 : A26_FZ); }
static inline void a26Push(uint8_t v) { a26Wr(0x100 + a26S, v); a26S--; }
static inline uint8_t a26Pull() { a26S++; return a26Rd(0x100 + a26S); }

static A26_FAST void a26Adc(uint8_t m) {
  int c = a26P & A26_FC;
  if (a26P & A26_FD) {                              // NMOS decimal mode
    int al = (a26A & 0x0F) + (m & 0x0F) + c;
    if (al >= 0x0A) al = ((al + 6) & 0x0F) + 0x10;
    int t = (a26A & 0xF0) + (m & 0xF0) + al;
    a26P &= ~(A26_FC | A26_FV | A26_FN | A26_FZ);
    if (!(uint8_t)(a26A + m + c)) a26P |= A26_FZ;   // Z comes from the binary result
    if (t & 0x80) a26P |= A26_FN;
    if (~(a26A ^ m) & (a26A ^ t) & 0x80) a26P |= A26_FV;
    if (t >= 0xA0) t += 0x60;
    if (t >= 0x100) a26P |= A26_FC;
    a26A = (uint8_t)t;
  } else {
    int r = a26A + m + c;
    a26P &= ~(A26_FC | A26_FV);
    if (r > 0xFF) a26P |= A26_FC;
    if (~(a26A ^ m) & (a26A ^ r) & 0x80) a26P |= A26_FV;
    a26A = (uint8_t)r; a26NZ(a26A);
  }
}
static A26_FAST void a26Sbc(uint8_t m) {
  int c = a26P & A26_FC;
  if (a26P & A26_FD) {                              // NMOS decimal mode: all flags come from the binary subtraction
    int al = (a26A & 0x0F) - (m & 0x0F) + c - 1;
    if (al < 0) al = ((al - 6) & 0x0F) - 0x10;
    int t = (a26A & 0xF0) - (m & 0xF0) + al;
    if (t < 0) t -= 0x60;
    uint8_t nm = (uint8_t)~m;
    int r = a26A + nm + c;
    a26P &= ~(A26_FC | A26_FV);
    if (r > 0xFF) a26P |= A26_FC;
    if (~(a26A ^ nm) & (a26A ^ r) & 0x80) a26P |= A26_FV;
    a26NZ((uint8_t)r);
    a26A = (uint8_t)t;
  } else a26Adc((uint8_t)~m);
}
static inline void a26Cmp(uint8_t reg, uint8_t m) { a26P = (a26P & ~A26_FC) | (reg >= m ? A26_FC : 0); a26NZ((uint8_t)(reg - m)); }
static inline uint8_t a26Asl(uint8_t v) { a26P = (a26P & ~A26_FC) | (v >> 7); v <<= 1; a26NZ(v); return v; }
static inline uint8_t a26Lsr(uint8_t v) { a26P = (a26P & ~A26_FC) | (v & 1); v >>= 1; a26NZ(v); return v; }
static inline uint8_t a26Rol(uint8_t v) { uint8_t c = a26P & A26_FC; a26P = (a26P & ~A26_FC) | (v >> 7); v = (v << 1) | c; a26NZ(v); return v; }
static inline uint8_t a26Ror(uint8_t v) { uint8_t c = a26P & A26_FC; a26P = (a26P & ~A26_FC) | (v & 1); v = (v >> 1) | (c << 7); a26NZ(v); return v; }
static inline uint8_t a26Shift(int k, uint8_t v) { return k == 0 ? a26Asl(v) : k == 1 ? a26Rol(v) : k == 2 ? a26Lsr(v) : a26Ror(v); }

static A26_FAST void a26Branch(bool cond) {
  int8_t off = (int8_t)a26Rd(a26PC++);
  if (cond) {
    a26Idle();
    uint16_t np = a26PC + off;
    if ((np ^ a26PC) & 0xFF00) a26Idle();           // page crossed
    a26PC = np;
  }
}

enum { A26_IMM, A26_ZP, A26_ZPX, A26_ZPY, A26_ABS, A26_ABX, A26_ABY, A26_IZX, A26_IZY };

// Effective address; `wr` = the access is a store / read-modify-write (always pays the indexing cycle).
static A26_FAST uint16_t a26EA(int mode, bool wr) {
  uint16_t a, b; uint8_t z;
  switch (mode) {
    case A26_IMM: return a26PC++;
    case A26_ZP:  return a26Rd(a26PC++);
    case A26_ZPX: z = a26Rd(a26PC++); a26Idle(); return (uint8_t)(z + a26X);
    case A26_ZPY: z = a26Rd(a26PC++); a26Idle(); return (uint8_t)(z + a26Y);
    case A26_ABS: a = a26Rd(a26PC++); a |= a26Rd(a26PC++) << 8; return a;
    case A26_ABX: case A26_ABY:
      b = a26Rd(a26PC++); b |= a26Rd(a26PC++) << 8;
      a = b + (mode == A26_ABX ? a26X : a26Y);
      if (wr || ((a ^ b) & 0xFF00)) a26Idle();
      return a;
    case A26_IZX: {
      z = a26Rd(a26PC++); a26Idle(); z += a26X;
      a = a26Rd(z); a |= a26Rd((uint8_t)(z + 1)) << 8; return a;
    }
    default: {                                      // A26_IZY
      z = a26Rd(a26PC++);
      b = a26Rd(z); b |= a26Rd((uint8_t)(z + 1)) << 8;
      a = b + a26Y;
      if (wr || ((a ^ b) & 0xFF00)) a26Idle();
      return a;
    }
  }
}

static A26_FAST void a26Alu(int k, uint8_t m) {              // the eight "group 1" operations (LDA = 5, STA handled by caller)
  switch (k) {
    case 0: a26A |= m; a26NZ(a26A); break;
    case 1: a26A &= m; a26NZ(a26A); break;
    case 2: a26A ^= m; a26NZ(a26A); break;
    case 3: a26Adc(m); break;
    case 5: a26A = m; a26NZ(a26A); break;
    case 6: a26Cmp(a26A, m); break;
    case 7: a26Sbc(m); break;
  }
}

static A26_FAST void a26Step() {
  static const uint8_t g1[8] = {A26_IZX, A26_ZP, A26_IMM, A26_ABS, A26_IZY, A26_ZPX, A26_ABY, A26_ABX};
  const uint8_t op = a26Rd(a26PC++);
  const int cc = op & 3, bbb = (op >> 2) & 7, aaa = op >> 5;
  uint16_t ea; uint8_t v;

  if (cc == 1) {                                    // ORA AND EOR ADC STA LDA CMP SBC
    int m = g1[bbb];
    if (aaa == 4) {
      if (m == A26_IMM) a26Rd(a26PC++);             // 0x89: NOP #imm
      else { ea = a26EA(m, true); a26Wr(ea, a26A); }
    } else { ea = a26EA(m, false); a26Alu(aaa, a26Rd(ea)); }
  }
  else if (cc == 2) {                               // shifts, INC/DEC, LDX/STX, transfers
    if (bbb == 2) {                                 // accumulator / implied
      a26Idle();
      switch (aaa) {
        case 0: case 1: case 2: case 3: a26A = a26Shift(aaa, a26A); break;
        case 4: a26A = a26X; a26NZ(a26A); break;    // TXA
        case 5: a26X = a26A; a26NZ(a26X); break;    // TAX
        case 6: a26X--; a26NZ(a26X); break;         // DEX
        default: break;                             // NOP
      }
    } else if (bbb == 4 || bbb == 6) {              // KIL / TXS / TSX / unofficial NOPs
      a26Idle();
      if (bbb == 6 && aaa == 4) a26S = a26X;        // TXS
      else if (bbb == 6 && aaa == 5) { a26X = a26S; a26NZ(a26X); }   // TSX
    } else if (bbb == 0) {                          // #imm : LDX, else NOP
      v = a26Rd(a26PC++);
      if (aaa == 5) { a26X = v; a26NZ(v); }
    } else {
      int m = bbb == 1 ? A26_ZP : bbb == 3 ? A26_ABS : bbb == 5 ? ((aaa == 4 || aaa == 5) ? A26_ZPY : A26_ZPX)
                                                                 : (aaa == 5 ? A26_ABY : A26_ABX);
      if (aaa == 4) {
        if (bbb == 7) { ea = a26EA(m, true); a26Rd(ea); }          // 0x9E (SHX): not emulated, acts as NOP
        else { ea = a26EA(m, true); a26Wr(ea, a26X); }             // STX
      } else if (aaa == 5) { ea = a26EA(m, false); a26X = a26Rd(ea); a26NZ(a26X); }   // LDX
      else {                                                        // ASL ROL LSR ROR DEC INC (read-modify-write)
        ea = a26EA(m, true);
        v = a26Rd(ea);
        a26Idle();                                                  // the dummy write cycle
        if (aaa < 4) v = a26Shift(aaa, v);
        else { v += (aaa == 6) ? -1 : 1; a26NZ(v); }
        a26Wr(ea, v);
      }
    }
  }
  else if (cc == 0) {
    if (bbb == 4) {                                 // conditional branches
      bool flag = aaa < 2 ? (a26P & A26_FN) : aaa < 4 ? (a26P & A26_FV) : aaa < 6 ? (a26P & A26_FC) : (a26P & A26_FZ);
      a26Branch((aaa & 1) ? flag : !flag);
    } else if (bbb == 2) {                          // PHP PLP PHA PLA DEY TAY INY INX
      a26Idle();
      switch (aaa) {
        case 0: a26Push(a26P | A26_FB | A26_FU); break;
        case 1: a26Idle(); a26P = (a26Pull() & ~A26_FB) | A26_FU; break;
        case 2: a26Push(a26A); break;
        case 3: a26Idle(); a26A = a26Pull(); a26NZ(a26A); break;
        case 4: a26Y--; a26NZ(a26Y); break;
        case 5: a26Y = a26A; a26NZ(a26Y); break;
        case 6: a26Y++; a26NZ(a26Y); break;
        default: a26X++; a26NZ(a26X); break;
      }
    } else if (bbb == 6) {                          // CLC SEC CLI SEI TYA CLV CLD SED
      a26Idle();
      switch (aaa) {
        case 0: a26P &= ~A26_FC; break;
        case 1: a26P |= A26_FC; break;
        case 2: a26P &= ~A26_FI; break;
        case 3: a26P |= A26_FI; break;
        case 4: a26A = a26Y; a26NZ(a26A); break;
        case 5: a26P &= ~A26_FV; break;
        case 6: a26P &= ~A26_FD; break;
        default: a26P |= A26_FD; break;
      }
    } else if (bbb == 0) {
      switch (aaa) {
        case 0: {                                   // BRK
          a26Rd(a26PC++);
          a26Push(a26PC >> 8); a26Push(a26PC & 0xFF); a26Push(a26P | A26_FB | A26_FU);
          a26P |= A26_FI;
          uint16_t lo = a26Rd(0xFFFE); a26PC = lo | (a26Rd(0xFFFF) << 8);
          break;
        }
        case 1: {                                   // JSR
          uint16_t lo = a26Rd(a26PC++);
          a26Idle();
          a26Push(a26PC >> 8); a26Push(a26PC & 0xFF);
          a26PC = lo | (a26Rd(a26PC) << 8);
          break;
        }
        case 2: {                                   // RTI
          a26Idle(); a26Idle();
          a26P = (a26Pull() & ~A26_FB) | A26_FU;
          uint16_t lo = a26Pull(); a26PC = lo | (a26Pull() << 8);
          break;
        }
        case 3: {                                   // RTS
          a26Idle(); a26Idle();
          uint16_t lo = a26Pull(); a26PC = lo | (a26Pull() << 8);
          a26Idle(); a26PC++;
          break;
        }
        case 4: a26Rd(a26PC++); break;              // 0x80: NOP #imm
        case 5: v = a26Rd(a26PC++); a26Y = v; a26NZ(v); break;   // LDY #
        case 6: a26Cmp(a26Y, a26Rd(a26PC++)); break;             // CPY #
        default: a26Cmp(a26X, a26Rd(a26PC++)); break;            // CPX #
      }
    } else {                                        // bbb 1,3,5,7: zp, abs, zp,X, abs,X
      if (bbb == 3 && (aaa == 2 || aaa == 3)) {     // JMP abs / JMP (ind)
        uint16_t p = a26Rd(a26PC++); p |= a26Rd(a26PC++) << 8;
        if (aaa == 2) a26PC = p;
        else {
          uint16_t lo = a26Rd(p);
          a26PC = lo | (a26Rd((p & 0xFF00) | ((p + 1) & 0xFF)) << 8);   // 6502 page-wrap bug
        }
      } else {
        int m = bbb == 1 ? A26_ZP : bbb == 3 ? A26_ABS : bbb == 5 ? A26_ZPX : A26_ABX;
        bool store = (aaa == 4 && bbb != 7);
        ea = a26EA(m, store);
        if (store) a26Wr(ea, a26Y);                                  // STY
        else {
          v = a26Rd(ea);
          if (aaa == 5) { a26Y = v; a26NZ(v); }                      // LDY
          else if (aaa == 1 && (bbb == 1 || bbb == 3)) {             // BIT
            a26P = (a26P & ~(A26_FN | A26_FV | A26_FZ)) | (v & 0xC0) | ((a26A & v) ? 0 : A26_FZ);
          }
          else if (aaa == 6 && (bbb == 1 || bbb == 3)) a26Cmp(a26Y, v);   // CPY
          else if (aaa == 7 && (bbb == 1 || bbb == 3)) a26Cmp(a26X, v);   // CPX
          // everything else is an unofficial NOP with an operand
        }
      }
    }
  }
  else {                                            // cc == 3: unofficial opcodes
    if (bbb == 2) {                                 // immediate forms
      v = a26Rd(a26PC++);
      switch (op) {
        case 0x0B: case 0x2B: a26A &= v; a26NZ(a26A); a26P = (a26P & ~A26_FC) | (a26A >> 7); break;   // ANC
        case 0x4B: a26A &= v; a26A = a26Lsr(a26A); break;                                            // ALR
        case 0xCB: { uint8_t t = a26A & a26X; a26P = (a26P & ~A26_FC) | (t >= v ? A26_FC : 0); a26X = t - v; a26NZ(a26X); break; }  // SBX
        case 0xEB: a26Sbc(v); break;                                                                 // SBC #
        default: break;                                                                              // XAA / ARR / LAX # : NOP
      }
    } else {
      int m = bbb == 0 ? A26_IZX : bbb == 1 ? A26_ZP : bbb == 3 ? A26_ABS : bbb == 4 ? A26_IZY
            : bbb == 5 ? ((aaa == 4 || aaa == 5) ? A26_ZPY : A26_ZPX)
            : bbb == 6 ? A26_ABY : (aaa == 5 ? A26_ABY : A26_ABX);
      if (aaa == 4) {                               // SAX (the other 0x9x opcodes are not emulated)
        ea = a26EA(m, true);
        if (m == A26_IZX || m == A26_ZP || m == A26_ABS || m == A26_ZPY) a26Wr(ea, a26A & a26X); else a26Rd(ea);
      } else if (aaa == 5) {                        // LAX
        ea = a26EA(m, false); v = a26Rd(ea); a26A = a26X = v; a26NZ(v);
      } else {                                      // SLO RLA SRE RRA DCP ISC
        ea = a26EA(m, true);
        v = a26Rd(ea);
        a26Idle();
        switch (aaa) {
          case 0: v = a26Asl(v); a26A |= v; a26NZ(a26A); break;
          case 1: v = a26Rol(v); a26A &= v; a26NZ(a26A); break;
          case 2: v = a26Lsr(v); a26A ^= v; a26NZ(a26A); break;
          case 3: v = a26Ror(v); a26Adc(v); break;
          case 6: v--; a26Cmp(a26A, v); break;
          default: v++; a26Sbc(v); break;
        }
        a26Wr(ea, v);
      }
    }
  }

  if (a26Wsync) {                                   // WSYNC: the CPU sits idle until the end of the scanline
    a26Wsync = false;
    if (a26LineCyc) {                               // (the instruction itself may already have ended the line)
      a26Cyc += 76 - a26LineCyc;                    // same as ticking until the line ends, in one go
      a26EndLine();
    }
  }
}

// ------------------------------------------------------------------ top level
static void a26Init() {                             // palette + collision table (cheap, safe to call repeatedly)
  for (int i = 0; i < 128; i++) {
    uint32_t c = A26_NTSC[i];
    a26Rgb[i] = (uint16_t)((((c >> 16) & 0xF8) << 8) | (((c >> 8) & 0xFC) << 3) | ((c & 0xFF) >> 3));
  }
  a26BuildCollTab();
}

// `rom` must stay valid (it is not copied).  Returns false and sets a26Error if the cart is not supported.
static bool a26Load(const uint8_t *rom, uint32_t size) {
  a26Error = nullptr;
  a26Rom = rom; a26RomSize = size; a26SC = false;
  a26Init();
  if (!rom || size < 2048) { a26Error = "ROM file is too small"; return false; }
  static const uint8_t e0sig[6][3] = {{0x8D,0xE0,0x1F},{0x8D,0xE0,0x5F},{0x8D,0xE9,0xFF},{0xAD,0xE9,0xFF},{0xAD,0xED,0xFF},{0xAD,0xF3,0xBF}};
  static const uint8_t sig3f[2] = {0x85, 0x3F};
  if (size == 2048) a26Map = A26_M_2K;
  else if (size == 4096) a26Map = A26_M_4K;
  else if (size == 12288) a26Map = A26_M_FA;
  else if (size % 2048 == 0 && size <= 524288) {
    bool e0 = false;
    if (size == 8192) for (int i = 0; i < 6; i++) if (a26CountSig(e0sig[i], 3)) e0 = true;
    if (e0) a26Map = A26_M_E0;
    else if (a26CountSig(sig3f, 2) >= 2) a26Map = A26_M_3F;
    else if (size == 8192) a26Map = A26_M_F8;
    else if (size == 16384) a26Map = A26_M_F6;
    else if (size == 32768) a26Map = A26_M_F4;
    else { a26Error = "Cart type not supported"; return false; }
    if (a26Map == A26_M_F8 || a26Map == A26_M_F6 || a26Map == A26_M_F4) a26SC = a26DetectSC(size / 4096);
  } else { a26Error = "ROM size not supported"; return false; }
  return true;
}

static void a26Reset() {
  a26Init();
  memset(a26Ram, 0, sizeof(a26Ram));
  a26CartInit();
  // TIA
  a26Vsync = a26Vblank = 0; a26ColuPF = a26ColuBK = 0; a26CtrlPF = 0;
  memset(a26Nusiz, 0, sizeof(a26Nusiz)); memset(a26Refp, 0, sizeof(a26Refp)); memset(a26ColuP, 0, sizeof(a26ColuP));
  memset(a26Pf, 0, sizeof(a26Pf)); memset(a26Grp, 0, sizeof(a26Grp)); memset(a26GrpOld, 0, sizeof(a26GrpOld));
  memset(a26Enam, 0, sizeof(a26Enam)); memset(a26Hm, 0, sizeof(a26Hm)); memset(a26Vdel, 0, sizeof(a26Vdel));
  memset(a26Resmp, 0, sizeof(a26Resmp)); memset(a26Aud, 0, sizeof(a26Aud));
  a26Enabl = a26EnablOld = 0; a26PosP[0] = a26PosP[1] = a26PosM[0] = a26PosM[1] = a26PosB = 0;
  a26Cx = 0; a26Latch[0] = a26Latch[1] = 0x80; a26Wsync = false; a26HmoveBlank = false; a26RendX = 0;
  for (int i = 0; i < A26_W; i++) a26Row[i] = a26Rgb[0];
  a26Derive();
  // RIOT
  a26SwchaOut = a26SwchaDdr = a26SwchbOut = a26SwchbDdr = 0;
  a26TmV = 0xFF; a26TmSh = 10; a26TmStart = 0;
  // timing
  a26Cyc = 0; a26LineCyc = 0; a26Line = 0; a26Top = A26_TOP_DEFAULT; a26FirstNB = -1; a26FrameDone = false;
  a26Pal = false; a26FrameCount = 0; a26FrameLines = 262;
  // CPU
  a26A = a26X = a26Y = 0; a26S = 0xFD; a26P = A26_FI | A26_FU;
  a26PC = a26Read(0xFFFC) | (a26Read(0xFFFD) << 8);
}

// Runs the machine until the game finishes one video frame (VSYNC released).  With draw = false the
// pictures is not sent to a26RowFn (frame skip) but the TIA still runs, so collisions keep working.
static A26_FAST void a26Frame(bool draw) {
  a26Draw = draw;
  a26FrameDone = false;
  while (!a26FrameDone) a26Step();
}
