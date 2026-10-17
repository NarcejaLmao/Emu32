// ============================================================================
//  Atari 2600 (VCS) emulator core   -   a2600_core.h
//   - MOS 6507 CPU (6502 core with decimal mode, 13-bit address bus)
//   - The CPU and the TIA run in lock-step: a TIA register write lands on the
//     colour clock of its own write cycle, so mid-scanline kernels (HMOVE,
//     per-line PF / colour rewrites, "racing the beam" code) work
//   - TIA video: playfield (repeat / mirror, score mode, priority), 2 players
//     (every NUSIZ copy mode and size, REFP, VDELP), 2 missiles (RESMP), ball
//     (VDELBL), HMOVE + comb blanking, all 15 collision latches, NTSC palette
//   - RIOT: 128 bytes RAM, interval timer (1T / 8T / 64T / 1024T), joystick
//     and console switches
//   - Cartridges (auto-detected): 2K, 4K, F8, F6, F4 (+ SuperChip RAM),
//     FA (CBS RAM Plus), E0 (Parker Bros), 3F (Tigervision)
//   - Output: the 160 x 192 picture is scaled to 172 x GAME_H (bilinear) and
//     pinned to the top of the portrait 172x320 panel, like the NES / GB
//   - Not emulated: audio, paddles / keypads, DPC / FE / E7 cartridges, PAL
//     colours (PAL games run, with NTSC colours, at 50 Hz pacing)
//
//  ROM folder on the SD card:  A2600_DIR  = /roms/a2600   (*.a26, *.bin)
//  Controls: D-pad = joystick, A or B = fire, SELECT = Select, START = Reset
//
//  Needs from the main sketch (defined above the #include):
//    lcdWindow / lcdCfg / outLine / padState / loadErr1-2, SD + File,
//    GAME_W / GAME_H / GAME_X / GAME_Y, PIN_LCD_CS / PIN_LCD_DC
//  Entry points:
//    a26Init()  a26Load(path)  a26Reset()  a26Frame(draw)  a26Shutdown()
//    a26IsRomName(name)        a26FrameUs  (frame period in microseconds;
//                                           copy it to the sketch's frameUs)
// ============================================================================
#ifndef A2600_DIR
#define A2600_DIR "/roms/a2600"
#endif
#define A26_LINES        192             // picture lines kept per frame
#define A26_OUT_H        GAME_H          // output rows (172 x GAME_H)
#define A26_MAX_LINES    330             // force a new frame when no VSYNC shows up
#define A26_P0_DIFF_A    0               // left  difficulty switch: 1 = A (pro), 0 = B
#define A26_P1_DIFF_A    0               // right difficulty switch: 1 = A (pro), 0 = B
#define A26_BW           0               // 1 = black & white switch position

static uint32_t a26FrameUs = 16688;      // 262 lines * 76 cycles / 1.193182 MHz

namespace a26 {

// ============================================================================
//  NTSC palette (index = COLUxx >> 1: 16 hues x 8 luminances)
// ============================================================================
DRAM_ATTR static const uint32_t ntscRGB[128] = {
  0x000000,0x4A4A4A,0x6F6F6F,0x8E8E8E,0xAAAAAA,0xC0C0C0,0xD6D6D6,0xECECEC,
  0x484800,0x69690F,0x86861D,0xA2A22A,0xBBBB35,0xD2D240,0xE8E84A,0xFCFC54,
  0x7C2C00,0x904811,0xA26221,0xB47A30,0xC3903D,0xD2A44A,0xDFB755,0xECC860,
  0x901C00,0xA33915,0xB55328,0xC66C3A,0xD5824A,0xE39759,0xF0AA67,0xFCBC74,
  0x940000,0xA71A1A,0xB83232,0xC84848,0xD65C5C,0xE46F6F,0xF08080,0xFC9090,
  0x840064,0x97197A,0xA8308F,0xB846A2,0xC659B3,0xD46CC3,0xE07CD2,0xEC8CE0,
  0x500084,0x68199A,0x7D30AD,0x9246C0,0xA459D0,0xB56CE0,0xC57CEE,0xD48CFC,
  0x140090,0x331AA3,0x4E32B5,0x6848C6,0x7F5CD5,0x956FE3,0xA980F0,0xBC90FC,
  0x000094,0x181AA7,0x2D32B8,0x4248C8,0x545CD6,0x656FE4,0x7580F0,0x8490FC,
  0x001C88,0x183B9D,0x2D57B0,0x4272C2,0x548AD2,0x65A0E1,0x75B5EF,0x84C8FC,
  0x003064,0x185080,0x2D6D98,0x4288B0,0x54A0C5,0x65B7D9,0x75CCEB,0x84E0FC,
  0x004030,0x18624E,0x2D8169,0x429E82,0x54B899,0x65D1AE,0x75E7C2,0x84FCD4,
  0x004400,0x1A661A,0x328432,0x48A048,0x5CBA5C,0x6FD26F,0x80E880,0x90FC90,
  0x143C00,0x355F18,0x527E2D,0x6E9C42,0x87B754,0x9ED065,0xB4E775,0xC8FC84,
  0x303800,0x505916,0x6D762B,0x88923E,0xA0AB4F,0xB7C25F,0xCCD86E,0xE0EC7C,
  0x482C00,0x694D14,0x866A26,0xA28638,0xBB9F47,0xD2B656,0xE8CC63,0xFCE070
};
static uint16_t pal[128];                // native (not byte-swapped) RGB565

// ============================================================================
//  Cartridge
// ============================================================================
enum : uint8_t { M_4K, M_F8, M_F6, M_F4, M_FA, M_E0, M_3F };
static uint8_t *rom = nullptr;
static uint32_t romSize = 0;
static uint8_t  mapper = M_4K;
static uint8_t  ramKind = 0;             // 0 none, 1 SuperChip (128 B), 2 CBS RAM Plus (256 B)
static bool     hotMapper = false;       // bank switches on reads/writes near $1FF8
static int      nBanks2k = 2;
static uint8_t *cmap[4];                 // four 1 KB windows of $1000-$1FFF
static uint8_t  xram[256];

static void map4k(int b) { uint8_t *p = rom + (uint32_t)b * 4096; for (int i = 0; i < 4; i++) cmap[i] = p + i * 1024; }
static void map1k(int slot, int b) { cmap[slot] = rom + (uint32_t)b * 1024; }
static void map2k(int slot, int b) {
  uint8_t *p = rom + (uint32_t)b * 2048;
  cmap[slot * 2] = p; cmap[slot * 2 + 1] = p + 1024;
}

static void cartHot(uint16_t o) {        // o = address & 0xFFF, o >= 0xFE0
  switch (mapper) {
    case M_F8: if (o >= 0xFF8 && o <= 0xFF9) map4k(o - 0xFF8); break;
    case M_F6: if (o >= 0xFF6 && o <= 0xFF9) map4k(o - 0xFF6); break;
    case M_F4: if (o >= 0xFF4 && o <= 0xFFB) map4k(o - 0xFF4); break;
    case M_FA: if (o >= 0xFF8 && o <= 0xFFA) map4k(o - 0xFF8); break;
    case M_E0: if (o <= 0xFF7) map1k((o - 0xFE0) >> 3, o & 7); break;
    default: break;
  }
}

static inline uint8_t cartRd(uint16_t a) {
  const uint16_t o = a & 0xFFF;
  if (hotMapper && o >= 0xFE0) cartHot(o);
  if (ramKind) {
    if (ramKind == 1) { if (o >= 0x80 && o < 0x100) return xram[o & 0x7F]; }
    else if (o >= 0x100 && o < 0x200) return xram[o & 0xFF];
  }
  return cmap[o >> 10][o & 0x3FF];
}
static inline void cartWr(uint16_t a, uint8_t v) {
  const uint16_t o = a & 0xFFF;
  if (hotMapper && o >= 0xFE0) cartHot(o);
  if (ramKind == 1) { if (o < 0x80) xram[o] = v; }
  else if (ramKind == 2) { if (o < 0x100) xram[o] = v; }
}

// ============================================================================
//  Machine state
// ============================================================================
static uint8_t  ram[128];                // RIOT RAM  ($80-$FF, stack lives here too)

// ---- CPU -------------------------------------------------------------------
static uint16_t pc;
static uint8_t  ra, rx, ry, sp, ps;
static uint8_t  curOp;
static int      xcyc;                    // extra cycles of the current instruction
static int      advCyc;                  // cycles of this instruction already given to TIA/RIOT

// ---- RIOT ------------------------------------------------------------------
static int32_t  timerTicks = 0;          // counts down one per CPU cycle; INTIM = ticks >> shift
static uint8_t  timerShift = 0;
static uint8_t  swcha = 0xFF, swchb = 0x3F;
static bool     fireDown[2];
static uint8_t  inptLatch[2] = { 0x80, 0x80 };

// ---- TIA -------------------------------------------------------------------
static int      tiaX = 0;                // colour clock in the scanline: 0..227 (0-67 = HBLANK)
static int      line = 0;                // scanline since VSYNC
static bool     vsyncOn = false, wsync = false, frameDone = false;
static uint8_t  vblank = 0;
static uint8_t  cols[4];                 // palette index of BK, PF, P0, P1
static uint8_t  ctrlpf, pf0, pf1, pf2, nusiz[2];
static uint8_t  grpN[2], grpO[2], refp[2], vdelp[2], vdelbl;
static bool     enam[2], enabN, enabO, resmp[2];
static int      px[2], mx[2], bx;        // object positions 0..159 (left edge)
static int8_t   hm[5];                   // HMP0 HMP1 HMM0 HMM1 HMBL as signed -8..7
static uint16_t cx = 0;                  // 15 collision flags (see collTab)
static bool     hmoveBlank = false;
static bool     pfDirty = true, mapDirty = true;

static uint8_t  pfQ[40];                 // playfield, one entry per 4-pixel quad (0 / 32)
static uint8_t  lineObj[168];            // per pixel: 1 P0, 2 P1, 4 M0, 8 M1, 16 BL, 32 PF
DRAM_ATTR static uint16_t collTab[64];
DRAM_ATTR static uint8_t  srcTab[3][64]; // 0 BK 1 PF 2 P0 3 P1 4 PF-score
static const uint8_t *curSrc = srcTab[0];

// ---- frame buffer / output -------------------------------------------------
static uint8_t *fb = nullptr;            // A26_LINES x 160 palette indices
static uint8_t  scratch[160];
static uint8_t *dst = scratch;
static bool     drawFrame = true;
static int      y0 = 40;                 // first scanline of the picture
static int      firstVis = 9999, pendY0 = 0, pendCnt = 0;
static int      lastLines = 0;           // length of the last frame (debug / pacing)
static bool     y0Init = false;
static uint8_t  rowA[A26_OUT_H], rowB[A26_OUT_H], rowW[A26_OUT_H];
static uint8_t  colSrc[GAME_W];

// ============================================================================
//  6502 / 6507 CPU
// ============================================================================
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

static void tiaRun(int clocks);
static uint8_t tiaRd(uint16_t a);
static void    tiaWr(uint16_t a, uint8_t v);

// Give the TIA and the RIOT timer `n` CPU cycles.
static inline void advance(int n) {
  tiaRun(n * 3);
  timerTicks -= n;
  if (timerTicks < -65536) timerTicks = (timerTicks & 0xFF) - 256;
}
// Bring the hardware up to cycle `target` of the current instruction.
static inline void syncTo(int target) {
  if (target > advCyc) { advance(target - advCyc); advCyc = target; }
}
// A hardware access happens in the last cycle of the instruction (plus any
// page-cross cycle): run the hardware up to the start of that cycle.
static inline void hwSync() { syncTo(cycTab[curOp] + xcyc - 1); }

static uint8_t riotRd(uint16_t a) {
  hwSync();
  if (a & 4) {
    if (a & 1) return (timerTicks < 0) ? 0x80 : 0x00;               // TIMINT
    return (timerTicks >= 0) ? (uint8_t)(timerTicks >> timerShift) : (uint8_t)(timerTicks & 0xFF);   // INTIM
  }
  switch (a & 3) {
    case 0: return swcha;
    case 2: return swchb;
    default: return 0;                                              // DDRs read as 0
  }
}
static void riotWr(uint16_t a, uint8_t v) {
  hwSync();
  if (a & 0x10) {                                                   // TIM1T / TIM8T / TIM64T / T1024T
    static const uint8_t sh[4] = { 0, 3, 6, 10 };
    timerShift = sh[a & 3];
    timerTicks = ((int32_t)v << timerShift) + 1;                    // +1: set at the start of the write cycle
  }
}

static IRAM_ATTR uint8_t rd(uint16_t a) {
  a &= 0x1FFF;
  if (a & 0x1000) return cartRd(a);
  if (a & 0x80) return (a & 0x200) ? riotRd(a) : ram[a & 0x7F];
  return tiaRd(a);
}
static IRAM_ATTR void wr(uint16_t a, uint8_t v) {
  a &= 0x1FFF;
  if (a & 0x1000) { cartWr(a, v); return; }
  if (a & 0x80) { if (a & 0x200) riotWr(a, v); else ram[a & 0x7F] = v; return; }
  if (mapper == M_3F && (a & 0x10FF) == 0x3F) {                     // Tigervision bank register
    map2k(0, v % nBanks2k);
  }
  tiaWr(a, v);
}
static inline uint16_t rd16(uint16_t a) { return rd(a) | (rd(a + 1) << 8); }

static inline void push(uint8_t v) { ram[sp & 0x7F] = v; sp--; }
static inline uint8_t pull()       { sp++; return ram[sp & 0x7F]; }
static inline void nz(uint8_t v)   { ps = (ps & 0x7D) | (v & 0x80) | (v ? 0 : 2); }

static inline uint8_t doAsl(uint8_t v) { ps = (ps & ~1) | (v >> 7); v <<= 1; nz(v); return v; }
static inline uint8_t doLsr(uint8_t v) { ps = (ps & ~1) | (v & 1); v >>= 1; nz(v); return v; }
static inline uint8_t doRol(uint8_t v) { uint8_t c = ps & 1; ps = (ps & ~1) | (v >> 7); v = (v << 1) | c; nz(v); return v; }
static inline uint8_t doRor(uint8_t v) { uint8_t c = ps & 1; ps = (ps & ~1) | (v & 1); v = (v >> 1) | (c << 7); nz(v); return v; }

static void doAdc(uint8_t m) {
  if (ps & 8) {                                                     // decimal mode (NMOS 6502 behaviour)
    const uint32_t c = ps & 1;
    uint32_t t = (ra & 0xF) + (m & 0xF) + c;
    if (t > 9) t += 6;
    if (t <= 0x0F) t = (t & 0xF) + (ra & 0xF0) + (m & 0xF0);
    else           t = (t & 0xF) + (ra & 0xF0) + (m & 0xF0) + 0x10;
    const bool z = ((ra + m + c) & 0xFF) == 0;
    const bool n = t & 0x80;
    const bool v = ((ra ^ t) & 0x80) && !((ra ^ m) & 0x80);
    if ((t & 0x1F0) > 0x90) t += 0x60;
    const bool cy = (t & 0xFF0) > 0xF0;
    ra = (uint8_t)t;
    ps = (ps & ~0xC3) | (n ? 0x80 : 0) | (v ? 0x40 : 0) | (z ? 2 : 0) | (cy ? 1 : 0);
    return;
  }
  uint16_t s = ra + m + (ps & 1);
  uint8_t r = s;
  uint8_t ov = (~(ra ^ m) & (ra ^ r) & 0x80) ? 0x40 : 0;
  ps = (ps & ~0x41) | (s >> 8) | ov;
  ra = r; nz(ra);
}
static void doSbc(uint8_t m) {
  if (ps & 8) {
    const uint32_t borrow = (ps & 1) ? 0 : 1;
    const uint32_t t = (uint32_t)ra - m - borrow;
    uint32_t lo = (uint32_t)(ra & 0xF) - (m & 0xF) - borrow, ta;
    if (lo & 0x10) ta = ((lo - 6) & 0xF) | (((ra & 0xF0) - (m & 0xF0) - 0x10) & 0x1FF);
    else           ta = (lo & 0xF) | (((ra & 0xF0) - (m & 0xF0)) & 0x1FF);
    if (ta & 0x100) ta -= 0x60;
    const bool v = ((ra ^ t) & 0x80) && ((ra ^ m) & 0x80);
    ps = (ps & ~0xC3) | ((t & 0x80) ? 0x80 : 0) | (v ? 0x40 : 0) | ((t & 0xFF) ? 0 : 2) | (t < 0x100 ? 1 : 0);
    ra = (uint8_t)ta;
    return;
  }
  doAdc(m ^ 0xFF);   // binary mode: A + ~M + C
}
static inline void doCmp(uint8_t r, uint8_t m) { uint8_t t = r - m; ps = (ps & ~1) | (r >= m ? 1 : 0); nz(t); }

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
  uint8_t op = rd(pc++);
  curOp = op;
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
        case 7: doSbc(m); break;
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
      if (op == 0xEB) doSbc(rd(pc++)); else pc++;
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
            default: m++; doSbc(m); break;                  // ISB
          }
          wr(ea, m);
          break;
      }
    }
  }
  return cycTab[op] + xcyc;
}

// ============================================================================
//  TIA
// ============================================================================
static void rebuildPF() {
  uint32_t left = 0;
  for (int i = 0; i < 4; i++) if (pf0 & (0x10 << i)) left |= 1u << i;
  for (int i = 0; i < 8; i++) if (pf1 & (0x80 >> i)) left |= 1u << (4 + i);
  for (int i = 0; i < 8; i++) if (pf2 & (1 << i))    left |= 1u << (12 + i);
  const bool refl = ctrlpf & 1;
  for (int i = 0; i < 20; i++) pfQ[i] = (left >> i & 1) ? 32 : 0;
  for (int i = 0; i < 20; i++) pfQ[20 + i] = refl ? pfQ[19 - i] : pfQ[i];
  pfDirty = false;
}

static const uint8_t copyOff[8][3] = {
  { 0, 255, 255 }, { 0, 16, 255 }, { 0, 32, 255 }, { 0, 16, 32 },
  { 0, 64, 255 }, { 0, 255, 255 }, { 0, 32, 64 }, { 0, 255, 255 }
};

static inline void put(int x, uint8_t bit, int xmin) {
  while (x >= 160) x -= 160;
  if (x >= xmin) lineObj[x] |= bit;
}
static void stampRun(int pos, int w, uint8_t bit, int xmin) {
  for (int i = 0; i < w; i++) put(pos + i, bit, xmin);
}

// Rebuild the per-pixel object map from pixel `xmin` to the end of the line.
static void buildMap(int xmin) {
  if (pfDirty) rebuildPF();
  for (int x = xmin; x < 160; x++) lineObj[x] = pfQ[x >> 2];

  for (int n = 0; n < 2; n++) {                                     // players
    const uint8_t g = vdelp[n] ? grpO[n] : grpN[n];
    if (!g) continue;
    const int nus = nusiz[n] & 7, sc = (nus == 5) ? 2 : (nus == 7 ? 4 : 1);
    const bool rf = refp[n] & 8;
    for (int c = 0; c < 3 && copyOff[nus][c] != 255; c++) {
      const int base = px[n] + copyOff[nus][c];
      for (int b = 0; b < 8; b++)
        if (g & (rf ? (1 << b) : (0x80 >> b))) stampRun(base + b * sc, sc, 1 << n, xmin);
    }
  }
  for (int n = 0; n < 2; n++) {                                     // missiles
    if (!enam[n] || resmp[n]) continue;
    const int nus = nusiz[n] & 7, w = 1 << ((nusiz[n] >> 4) & 3);
    for (int c = 0; c < 3 && copyOff[nus][c] != 255; c++)
      stampRun(mx[n] + copyOff[nus][c], w, 4 << n, xmin);
  }
  if (vdelbl ? enabO : enabN) stampRun(bx, 1 << ((ctrlpf >> 4) & 3), 16, xmin);   // ball
  mapDirty = false;
}

static void setDst() {
  const int row = line - y0;
  dst = (fb && drawFrame && row >= 0 && row < A26_LINES) ? fb + row * 160 : scratch;
  if (dst != scratch) memset(dst, 0, 160);
}

static void frameEnd() {
  if (firstVis < 9999) {
    if (!y0Init) { y0 = firstVis; y0Init = true; pendY0 = firstVis; pendCnt = 0; }
    else if (firstVis == pendY0) { if (++pendCnt >= 3) y0 = pendY0; }
    else { pendY0 = firstVis; pendCnt = 1; }
  }
  firstVis = 9999;
  lastLines = line;
  if (line > 100) a26FrameUs = (line >= 290) ? 20055 : 16688;       // PAL (312 lines) = 50 Hz, NTSC = 60 Hz
  line = 0;
  frameDone = true;
  setDst();
}

static IRAM_ATTR void renderSpan(int x0, int x1) {
  if (vblank & 2) return;                                           // blanked: row stays black
  if (line >= 6 && line < firstVis) firstVis = line;
  if (hmoveBlank && x0 < 8) { x0 = 8; if (x0 >= x1) return; }
  if (mapDirty) buildMap(x0);
  if (!drawFrame || dst == scratch) {                               // skipped frame / off-screen line: collisions only
    uint16_t f = cx;
    for (int x = x0; x < x1; x++) f |= collTab[lineObj[x]];
    cx = f;
    return;
  }
  uint16_t f = cx;
  const uint8_t *st = curSrc;
  const uint8_t bk = cols[0];
  uint8_t *d = dst;
  for (int x = x0; x < x1; x++) {
    const uint8_t o = lineObj[x];
    if (!o) { d[x] = bk; continue; }
    f |= collTab[o];
    uint8_t s = st[o];
    if (s == 4) s = (x < 80) ? 2 : 3;                               // score mode: PF takes the player colours
    d[x] = cols[s];
  }
  cx = f;
}

static IRAM_ATTR void tiaRun(int clocks) {
  while (clocks > 0) {
    int n = 228 - tiaX;
    if (n > clocks) n = clocks;
    const int a = tiaX, b = tiaX + n;
    if (b > 68) renderSpan((a > 68 ? a : 68) - 68, b - 68);
    tiaX = b; clocks -= n;
    if (tiaX >= 228) {
      tiaX = 0; hmoveBlank = false; mapDirty = true;
      if (++line >= A26_MAX_LINES) frameEnd();
      setDst();
    }
  }
}

static inline int posPlayer(int c) { int p = (c < 68) ? 3 : c - 63; return p >= 160 ? p - 160 : p; }
static inline int posMissile(int c) { int p = (c < 68) ? 2 : c - 64; return p >= 160 ? p - 160 : p; }
static inline int wrap160(int p) { while (p < 0) p += 160; while (p >= 160) p -= 160; return p; }

static void selectSrc() { curSrc = srcTab[(ctrlpf & 4) ? 2 : ((ctrlpf & 2) ? 1 : 0)]; }

static uint8_t cxRead(int r) {
  const uint16_t f = cx;
  switch (r) {
    case 6:  return ((f >> 12) & 1) << 7;
    case 7:  return (((f >> 13) & 1) << 7) | (((f >> 14) & 1) << 6);
    default: return (((f >> (2 * r)) & 1) << 7) | (((f >> (2 * r + 1)) & 1) << 6);
  }
}

static uint8_t tiaRd(uint16_t a) {
  hwSync();
  const int r = a & 0x0F;
  if (r < 8) return cxRead(r);
  if (r == 0x0C || r == 0x0D) {                                     // INPT4 / INPT5: fire buttons
    const int i = r - 0x0C;
    const uint8_t v = fireDown[i] ? 0x00 : 0x80;
    if (vblank & 0x40) { inptLatch[i] &= v; return inptLatch[i]; }
    return v;
  }
  if (r < 0x0C) return 0x80;                                        // INPT0-3 (paddles): not connected
  return 0;
}

static void tiaWr(uint16_t a, uint8_t v) {
  hwSync();
  const int c = tiaX;                                               // colour clock at the start of the write cycle
  switch (a & 0x3F) {
    case 0x00:                                                      // VSYNC
      if ((v & 2) && !vsyncOn && line >= 10) frameEnd();
      vsyncOn = v & 2;
      break;
    case 0x01:                                                      // VBLANK
      vblank = v;
      if (!(v & 0x40)) inptLatch[0] = inptLatch[1] = 0x80;
      break;
    case 0x02: wsync = true; break;                                 // WSYNC
    case 0x04: nusiz[0] = v; mapDirty = true; break;
    case 0x05: nusiz[1] = v; mapDirty = true; break;
    case 0x06: cols[2] = (v >> 1) & 0x7F; break;                    // COLUP0
    case 0x07: cols[3] = (v >> 1) & 0x7F; break;                    // COLUP1
    case 0x08: cols[1] = (v >> 1) & 0x7F; break;                    // COLUPF
    case 0x09: cols[0] = (v >> 1) & 0x7F; break;                    // COLUBK
    case 0x0A: ctrlpf = v; pfDirty = mapDirty = true; selectSrc(); break;
    case 0x0B: refp[0] = v; mapDirty = true; break;
    case 0x0C: refp[1] = v; mapDirty = true; break;
    case 0x0D: pf0 = v; pfDirty = mapDirty = true; break;
    case 0x0E: pf1 = v; pfDirty = mapDirty = true; break;
    case 0x0F: pf2 = v; pfDirty = mapDirty = true; break;
    case 0x10: px[0] = posPlayer(c);  mapDirty = true; break;       // RESP0
    case 0x11: px[1] = posPlayer(c);  mapDirty = true; break;       // RESP1
    case 0x12: mx[0] = posMissile(c); mapDirty = true; break;       // RESM0
    case 0x13: mx[1] = posMissile(c); mapDirty = true; break;       // RESM1
    case 0x14: bx    = posMissile(c); mapDirty = true; break;       // RESBL
    case 0x1B: grpN[0] = v; grpO[1] = grpN[1]; mapDirty = true; break;                 // GRP0
    case 0x1C: grpN[1] = v; grpO[0] = grpN[0]; enabO = enabN; mapDirty = true; break;  // GRP1
    case 0x1D: enam[0] = v & 2; mapDirty = true; break;
    case 0x1E: enam[1] = v & 2; mapDirty = true; break;
    case 0x1F: enabN = v & 2; mapDirty = true; break;
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: {       // HMP0 HMP1 HMM0 HMM1 HMBL
      int n = v >> 4; if (n >= 8) n -= 16;
      hm[(a & 0x3F) - 0x20] = (int8_t)n;
    } break;
    case 0x25: vdelp[0] = v & 1; mapDirty = true; break;
    case 0x26: vdelp[1] = v & 1; mapDirty = true; break;
    case 0x27: vdelbl = v & 1; mapDirty = true; break;
    case 0x28: case 0x29: {                                         // RESMP0 / RESMP1
      const int n = (a & 0x3F) - 0x28;
      const bool lock = v & 2;
      if (resmp[n] && !lock) {                                      // released: missile stays at the player's middle
        const int nus = nusiz[n] & 7, off = (nus == 5) ? 6 : (nus == 7 ? 10 : 3);
        mx[n] = wrap160(px[n] + off);
      }
      resmp[n] = lock; mapDirty = true;
    } break;
    case 0x2A:                                                      // HMOVE
      px[0] = wrap160(px[0] - hm[0]); px[1] = wrap160(px[1] - hm[1]);
      mx[0] = wrap160(mx[0] - hm[2]); mx[1] = wrap160(mx[1] - hm[3]);
      bx    = wrap160(bx    - hm[4]);
      if (c < 68) hmoveBlank = true;                                // the "HMOVE comb": first 8 pixels blanked
      mapDirty = true;
      break;
    case 0x2B: hm[0] = hm[1] = hm[2] = hm[3] = hm[4] = 0; break;    // HMCLR
    case 0x2C: cx = 0; break;                                       // CXCLR
    default: break;                                                 // RSYNC, audio, unused
  }
}

// ============================================================================
//  Tables, input, output
// ============================================================================
static void buildTables() {
  for (int i = 0; i < 128; i++) {
    const uint32_t c = ntscRGB[i];
    const uint8_t r = c >> 16, g = c >> 8, b = c;
    pal[i] = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }
  for (int o = 0; o < 64; o++) {
    const bool p0 = o & 1, p1 = o & 2, m0 = o & 4, m1 = o & 8, bl = o & 16, pf = o & 32;
    const uint16_t f =
        ((m0 && p1) << 0)  | ((m0 && p0) << 1)  | ((m1 && p0) << 2)  | ((m1 && p1) << 3) |
        ((p0 && pf) << 4)  | ((p0 && bl) << 5)  | ((p1 && pf) << 6)  | ((p1 && bl) << 7) |
        ((m0 && pf) << 8)  | ((m0 && bl) << 9)  | ((m1 && pf) << 10) | ((m1 && bl) << 11) |
        ((bl && pf) << 12) | ((p0 && p1) << 13) | ((m0 && m1) << 14);
    collTab[o] = f;

    for (int mode = 0; mode < 3; mode++) {                          // 0 normal, 1 score, 2 PF priority
      const uint8_t pfSrc = (mode == 1) ? 4 : 1;
      uint8_t s = 0;
      if (mode == 2) {
        if (bl) s = 1; else if (pf) s = 1;
        else if (p0 || m0) s = 2; else if (p1 || m1) s = 3;
      } else {
        if (p0 || m0) s = 2; else if (p1 || m1) s = 3;
        else if (bl) s = 1; else if (pf) s = pfSrc;
      }
      srcTab[mode][o] = s;
    }
  }
  for (int r = 0; r < A26_OUT_H; r++) {                             // vertical bilinear: 192 -> A26_OUT_H
    int p = (((2 * r + 1) * A26_LINES * 128) / A26_OUT_H) - 128;    // position in 1/256 source lines
    if (p < 0) p = 0;
    int i0 = p >> 8, w = (((p & 255) * 32) + 128) >> 8;
    if (i0 >= A26_LINES - 1) { i0 = A26_LINES - 1; w = 0; }
    rowA[r] = i0; rowB[r] = (i0 + 1 < A26_LINES) ? i0 + 1 : i0; rowW[r] = w;
  }
  for (int x = 0; x < GAME_W; x++) colSrc[x] = ((2 * x + 1) * 160) / (2 * GAME_W);   // 172 -> 160 columns
}

static void readPad() {
  const uint8_t s = padState;                                       // A B SEL START UP DOWN LEFT RIGHT
  uint8_t a = 0xFF;
  if (s & 0x80) a &= ~0x80;                                         // right
  if (s & 0x40) a &= ~0x40;                                         // left
  if (s & 0x20) a &= ~0x20;                                         // down
  if (s & 0x10) a &= ~0x10;                                         // up
  swcha = a;
  uint8_t b = 0x34;
  if (!(s & 0x08)) b |= 0x01;                                       // Reset  (START)
  if (!(s & 0x04)) b |= 0x02;                                       // Select (SELECT)
  if (!A26_BW) b |= 0x08;
  if (A26_P0_DIFF_A) b |= 0x40;
  if (A26_P1_DIFF_A) b |= 0x80;
  swchb = b;
  fireDown[0] = s & 3;
  fireDown[1] = false;
  if (vblank & 0x40) for (int i = 0; i < 2; i++) inptLatch[i] &= fireDown[i] ? 0x00 : 0x80;
}

// 5-bit alpha blend of two RGB565 colours (alpha 0 -> b, 32 -> f)
static inline uint16_t blend565(uint16_t f, uint16_t b, uint32_t alpha) {
  uint32_t bg = (b | ((uint32_t)b << 16)) & 0x07E0F81F;
  uint32_t fg = (f | ((uint32_t)f << 16)) & 0x07E0F81F;
  uint32_t r = ((((fg - bg) * alpha) >> 5) + bg) & 0x07E0F81F;
  return (uint16_t)(r | (r >> 16));
}

static IRAM_ATTR void blit() {
  SPI.beginTransaction(lcdCfg);
  lcdWindow(GAME_X, GAME_Y, GAME_X + GAME_W - 1, GAME_Y + A26_OUT_H - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int r = 0; r < A26_OUT_H; r++) {
    const uint8_t *pa = fb + rowA[r] * 160, *pb = fb + rowB[r] * 160;
    const uint32_t w = rowW[r];
    for (int x = 0; x < GAME_W; x++) {
      const int sx = colSrc[x];
      uint16_t c = pal[pa[sx]];
      if (w && pa[sx] != pb[sx]) c = blend565(pal[pb[sx]], c, w);
      outLine[x] = (c >> 8) | (c << 8);                             // byte-swapped for the wire
    }
    SPI.writeBytes((uint8_t *)outLine, GAME_W * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// ============================================================================
//  Install / reset / run
// ============================================================================
static void freeAll() {
  free(rom); rom = nullptr; romSize = 0;
  free(fb);  fb = nullptr;
  // Bank-window pointers refer into the ROM allocation; invalidate them before
  // another cartridge is loaded so no failed/partial load can leave stale refs.
  for (int i = 0; i < 4; ++i) cmap[i] = nullptr;
  ramKind = 0; hotMapper = false; nBanks2k = 2;
}

static void reset() {
  memset(ram, 0, sizeof(ram));
  memset(xram, 0, sizeof(xram));
  ra = rx = ry = 0; sp = 0xFD; ps = 0x34;
  timerTicks = 0; timerShift = 0;
  tiaX = 0; line = 0; vsyncOn = false; wsync = false; frameDone = false;
  vblank = 0; ctrlpf = pf0 = pf1 = pf2 = 0;
  memset(cols, 0, sizeof(cols));
  nusiz[0] = nusiz[1] = 0; grpN[0] = grpN[1] = grpO[0] = grpO[1] = 0;
  refp[0] = refp[1] = vdelp[0] = vdelp[1] = vdelbl = 0;
  enam[0] = enam[1] = enabN = enabO = resmp[0] = resmp[1] = false;
  px[0] = px[1] = mx[0] = mx[1] = bx = 0;
  memset(hm, 0, sizeof(hm));
  cx = 0; hmoveBlank = false; pfDirty = mapDirty = true;
  inptLatch[0] = inptLatch[1] = 0x80;
  firstVis = 9999; pendY0 = 0; pendCnt = 0; y0Init = false; y0 = 40;
  selectSrc();
  if (fb) memset(fb, 0, A26_LINES * 160);

  // power-on bank layout
  switch (mapper) {
    case M_4K: map4k(0); break;
    case M_F8: case M_F6: case M_F4: case M_FA: map4k((int)(romSize / 4096) - 1); break;
    case M_E0: map1k(0, 4); map1k(1, 5); map1k(2, 6); map1k(3, 7); break;
    case M_3F: map2k(0, 0); map2k(1, nBanks2k - 1); break;
  }
  pc = rd16(0xFFFC);
  setDst();
}

// Take ownership of `buf` (malloc'd, `sz` bytes). Returns false (and sets loadErr1/2) on a bad ROM.
static bool install(uint8_t *buf, uint32_t sz) {
  freeAll();
  if (sz == 0) { free(buf); return false; }
  if (sz <= 2048) {                                                 // 1K / 2K ROM: mirror it through the 4K window
    uint32_t pw = 1024; while (pw < sz) pw <<= 1;
    uint8_t *t = (uint8_t *)malloc(4096);
    if (!t) { free(buf); strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "ROM", sizeof(loadErr2)); return false; }
    memset(t, 0xFF, 4096);
    memcpy(t, buf, sz);
    for (uint32_t o = pw; o < 4096; o += pw) memcpy(t + o, t, pw);
    free(buf); buf = t; sz = 4096;
  }
  if (sz % 4096 != 0 || sz > 65536) {
    free(buf);
    strlcpy(loadErr1, "BAD ROM SIZE", sizeof(loadErr1)); strlcpy(loadErr2, "NOT A 2600 ROM", sizeof(loadErr2));
    return false;
  }
  rom = buf; romSize = sz; nBanks2k = sz / 2048;

  // --- detect the cartridge type ---
  int n3f = 0, nE0 = 0;
  for (uint32_t i = 0; i + 2 < sz; i++) {
    if (rom[i] == 0x85 && rom[i + 1] == 0x3F) n3f++;                                  // STA $3F
    if ((rom[i] == 0xAD || rom[i] == 0x8D || rom[i] == 0x2C || rom[i] == 0x0C) &&     // LDA / STA / BIT / NOP abs
        rom[i + 1] >= 0xE0 && rom[i + 1] <= 0xF7 && (rom[i + 2] & 0x1F) == 0x1F) nE0++;
  }
  ramKind = 0; hotMapper = true;
  switch (sz) {
    case 4096:  mapper = M_4K; hotMapper = false; break;
    case 8192:  mapper = (n3f >= 2) ? M_3F : (nE0 >= 3 ? M_E0 : M_F8); break;
    case 12288: mapper = M_FA; ramKind = 2; break;
    case 16384: mapper = (n3f >= 2) ? M_3F : M_F6; break;
    case 32768: mapper = (n3f >= 2) ? M_3F : M_F4; break;
    default:    mapper = M_3F; break;                                                  // 20K..64K: Tigervision
  }
  if (mapper == M_3F) hotMapper = false;
  if (mapper == M_F8 || mapper == M_F6 || mapper == M_F4) {                            // SuperChip: unused first 256 bytes of every bank
    bool sc = true;
    const uint8_t v0 = rom[0];
    for (uint32_t b = 0; b < sz && sc; b += 4096)
      for (int i = 0; i < 256; i++) if (rom[b + i] != v0) { sc = false; break; }
    if (sc) ramKind = 1;
  }
  fb = (uint8_t *)calloc(1, A26_LINES * 160);
  if (!fb) { freeAll(); strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "FRAME BUFFER", sizeof(loadErr2)); return false; }
  static const char *const names[] = { "4K", "F8", "F6", "F4", "FA", "E0", "3F" };
  Serial.printf("2600: %uK ROM, mapper %s%s\n", (unsigned)(sz / 1024), names[mapper], ramKind ? " + RAM" : "");
  return true;
}

// Run the machine until the TIA sees the next VSYNC (one TV frame).
static IRAM_ATTR void runFrame() {
  frameDone = false;
  while (!frameDone) {
    advCyc = 0;
    const int c = cpuStep();
    syncTo(c);
    if (wsync) {                                                    // CPU halted until the end of the scanline
      wsync = false;
      if (tiaX != 0) advance((228 - tiaX) / 3);
    }
  }
}

}  // namespace a26

// ============================================================================
//  Entry points used by the main sketch
// ============================================================================
static void a26Init() { a26::buildTables(); }

static bool a26IsRomName(const char *n) {
  const size_t l = strlen(n);
  if (l < 5 || n[0] == '.') return false;                           // also skips ._mac files
  return strcasecmp(n + l - 4, ".a26") == 0 || strcasecmp(n + l - 4, ".bin") == 0;
}

static void a26Shutdown() { a26::freeAll(); }

static void a26Reset() {
  if (a26::rom) a26::reset();
  a26FrameUs = 16688;
}

static bool a26Load(const char *path) {
  a26::freeAll();
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    strlcpy(loadErr1, "ROM NOT FOUND", sizeof(loadErr1)); strlcpy(loadErr2, "CHECK SD CARD", sizeof(loadErr2));
    return false;
  }
  const size_t sz = f.size();
  if (sz == 0 || sz > 65536) {
    f.close();
    strlcpy(loadErr1, "BAD ROM SIZE", sizeof(loadErr1)); strlcpy(loadErr2, "NOT A 2600 ROM", sizeof(loadErr2));
    return false;
  }
  uint8_t *buf = (uint8_t *)malloc(sz);
  if (!buf) {
    f.close();
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1)); strlcpy(loadErr2, "ROM", sizeof(loadErr2));
    return false;
  }
  if (f.read(buf, sz) != (int)sz) {
    f.close(); free(buf);
    strlcpy(loadErr1, "ROM TRUNCATED", sizeof(loadErr1)); strlcpy(loadErr2, "READ ERROR", sizeof(loadErr2));
    return false;
  }
  f.close();
  Serial.printf("ROM: %s\n", path);
  return a26::install(buf, sz);
}

// One TV frame (VSYNC to VSYNC). `draw` = false skips the screen update (frame skip).
static void a26Frame(bool draw) {
  if (!a26::rom || !a26::fb) return;
  a26::drawFrame = draw;
  a26::readPad();
  a26::setDst();
  a26::runFrame();
  if (draw) a26::blit();
}

// Atari 2600 state snapshot. Cartridge ROM is immutable and stays resident;
// the bank-window offsets and SuperChip/CBS RAM are restored separately.
namespace a26 {
struct A26StateCore {
  uint32_t romSize; uint8_t mapper,ramKind; bool hotMapper; int nBanks2k;
  uint16_t bankOffsets[4]; uint8_t xram[256], ram[128];
  uint16_t pc; uint8_t ra,rx,ry,sp,ps,curOp; int xcyc,advCyc;
  int32_t timerTicks; uint8_t timerShift,swcha,swchb; bool fireDown[2]; uint8_t inptLatch[2];
  int tiaX,line; bool vsyncOn,wsync,frameDone; uint8_t vblank,cols[4],ctrlpf,pf0,pf1,pf2,nusiz[2];
  uint8_t grpN[2],grpO[2],refp[2],vdelp[2],vdelbl; bool enam[2],enabN,enabO,resmp[2];
  int px[2],mx[2],bx; int8_t hm[5]; uint16_t cx; bool hmoveBlank,pfDirty,mapDirty;
  uint8_t pfQ[40],lineObj[168],scratch[160]; bool drawFrame; int y0,firstVis,pendY0,pendCnt,lastLines; bool y0Init;
  uint8_t rowA[A26_OUT_H],rowB[A26_OUT_H],rowW[A26_OUT_H],colSrc[GAME_W];
};
static bool stateSave(uint8_t slot) {
  if(!rom || !fb || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if(!sdBuildStatePath(currentRomPath,"a2600",path,sizeof(path),slot)) return false;
  A26StateCore c{}; c.romSize=romSize;c.mapper=mapper;c.ramKind=ramKind;c.hotMapper=hotMapper;c.nBanks2k=nBanks2k;
  for(int i=0;i<4;i++) c.bankOffsets[i]=(uint16_t)(cmap[i]-rom);
  memcpy(c.xram,xram,sizeof(xram));memcpy(c.ram,ram,sizeof(ram));
  c.pc=pc;c.ra=ra;c.rx=rx;c.ry=ry;c.sp=sp;c.ps=ps;c.curOp=curOp;c.xcyc=xcyc;c.advCyc=advCyc;
  c.timerTicks=timerTicks;c.timerShift=timerShift;c.swcha=swcha;c.swchb=swchb;memcpy(c.fireDown,fireDown,sizeof(fireDown));memcpy(c.inptLatch,inptLatch,sizeof(inptLatch));
  c.tiaX=tiaX;c.line=line;c.vsyncOn=vsyncOn;c.wsync=wsync;c.frameDone=frameDone;c.vblank=vblank;memcpy(c.cols,cols,sizeof(cols));c.ctrlpf=ctrlpf;c.pf0=pf0;c.pf1=pf1;c.pf2=pf2;memcpy(c.nusiz,nusiz,sizeof(nusiz));
  memcpy(c.grpN,grpN,sizeof(grpN));memcpy(c.grpO,grpO,sizeof(grpO));memcpy(c.refp,refp,sizeof(refp));memcpy(c.vdelp,vdelp,sizeof(vdelp));c.vdelbl=vdelbl;
  memcpy(c.enam,enam,sizeof(enam));c.enabN=enabN;c.enabO=enabO;memcpy(c.resmp,resmp,sizeof(resmp));memcpy(c.px,px,sizeof(px));memcpy(c.mx,mx,sizeof(mx));c.bx=bx;memcpy(c.hm,hm,sizeof(hm));c.cx=cx;c.hmoveBlank=hmoveBlank;c.pfDirty=pfDirty;c.mapDirty=mapDirty;
  memcpy(c.pfQ,pfQ,sizeof(pfQ));memcpy(c.lineObj,lineObj,sizeof(lineObj));memcpy(c.scratch,scratch,sizeof(scratch));c.drawFrame=drawFrame;c.y0=y0;c.firstVis=firstVis;c.pendY0=pendY0;c.pendCnt=pendCnt;c.lastLines=lastLines;c.y0Init=y0Init;
  memcpy(c.rowA,rowA,sizeof(rowA));memcpy(c.rowB,rowB,sizeof(rowB));memcpy(c.rowW,rowW,sizeof(rowW));memcpy(c.colSrc,colSrc,sizeof(colSrc));
  bool ok=sdStateWrite(path,SYS_A2600,&c,sizeof(c),fb,A26_LINES*160,nullptr,0);
  Serial.printf("STATE A2600: %s (%s)\n",ok?"saved":"save failed",path); return ok;
}
static bool stateLoad(uint8_t slot) {
  if(!rom || !fb || !currentRomPath[0] || !sdMount()) return false;
  char path[192]; if(!sdBuildStatePath(currentRomPath,"a2600",path,sizeof(path),slot)) return false;
  A26StateCore c{};
  if(!sdStateRead(path,SYS_A2600,&c,sizeof(c),fb,A26_LINES*160,nullptr,0) || c.romSize!=romSize || c.mapper!=mapper || c.ramKind!=ramKind) return false;
  memcpy(xram,c.xram,sizeof(xram));memcpy(ram,c.ram,sizeof(ram));
  for(int i=0;i<4;i++){ if(c.bankOffsets[i]>=romSize) return false; cmap[i]=rom+c.bankOffsets[i]; }
  pc=c.pc;ra=c.ra;rx=c.rx;ry=c.ry;sp=c.sp;ps=c.ps;curOp=c.curOp;xcyc=c.xcyc;advCyc=c.advCyc;
  timerTicks=c.timerTicks;timerShift=c.timerShift;swcha=c.swcha;swchb=c.swchb;memcpy(fireDown,c.fireDown,sizeof(fireDown));memcpy(inptLatch,c.inptLatch,sizeof(inptLatch));
  tiaX=c.tiaX;line=c.line;vsyncOn=c.vsyncOn;wsync=c.wsync;frameDone=c.frameDone;vblank=c.vblank;memcpy(cols,c.cols,sizeof(cols));ctrlpf=c.ctrlpf;pf0=c.pf0;pf1=c.pf1;pf2=c.pf2;memcpy(nusiz,c.nusiz,sizeof(nusiz));
  memcpy(grpN,c.grpN,sizeof(grpN));memcpy(grpO,c.grpO,sizeof(grpO));memcpy(refp,c.refp,sizeof(refp));memcpy(vdelp,c.vdelp,sizeof(vdelp));vdelbl=c.vdelbl;
  memcpy(enam,c.enam,sizeof(enam));enabN=c.enabN;enabO=c.enabO;memcpy(resmp,c.resmp,sizeof(resmp));memcpy(px,c.px,sizeof(px));memcpy(mx,c.mx,sizeof(mx));bx=c.bx;memcpy(hm,c.hm,sizeof(hm));cx=c.cx;hmoveBlank=c.hmoveBlank;pfDirty=c.pfDirty;mapDirty=c.mapDirty;
  memcpy(pfQ,c.pfQ,sizeof(pfQ));memcpy(lineObj,c.lineObj,sizeof(lineObj));memcpy(scratch,c.scratch,sizeof(scratch));drawFrame=true;y0=c.y0;firstVis=c.firstVis;pendY0=c.pendY0;pendCnt=c.pendCnt;lastLines=c.lastLines;y0Init=c.y0Init;
  memcpy(rowA,c.rowA,sizeof(rowA));memcpy(rowB,c.rowB,sizeof(rowB));memcpy(rowW,c.rowW,sizeof(rowW));memcpy(colSrc,c.colSrc,sizeof(colSrc));selectSrc();setDst();lcdDirty=true;return true;
}
}
