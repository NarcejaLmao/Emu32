#pragma once
// Lightweight register-driven sound generators for the five built-in systems.
// This is deliberately a compact approximation (not cycle-accurate chip emulation),
// intended to make games audible without the RAM/CPU cost of full sound-chip cores.
#include "audio.h"
#include <math.h>

namespace EmuSound {
  static uint32_t phase[8] = {};
  static uint16_t noiseLfsr[4] = {1, 0xACE1, 0x1234, 0xBEEF};
  static uint32_t frameRemainder = 0;
  static int16_t mix[512];
  static uint32_t ymPhase[6] = {};

  static inline int32_t square(uint32_t &p, float hz, int amp) {
    if (hz <= 0.0f) return 0;
    uint32_t step = (uint32_t)((double)hz * 4294967296.0 / EMU32_AUDIO_SAMPLE_RATE);
    p += step;
    return (p & 0x80000000UL) ? amp : -amp;
  }
  static inline int32_t triangle(uint32_t &p, float hz, int amp) {
    if (hz <= 0.0f) return 0;
    p += (uint32_t)((double)hz * 4294967296.0 / EMU32_AUDIO_SAMPLE_RATE);
    int32_t x = (int32_t)(p >> 16) & 0xFFFF;
    int32_t t = x < 32768 ? x : 65535 - x;
    return ((t * 2 * amp) / 32768) - amp;
  }
  static inline float gbHz(uint16_t period) { return period >= 2048 ? 0.0f : 131072.0f / (2048 - period); }
  static inline int gbSample(const uint8_t *r, uint32_t *ph, uint16_t *lfsr) {
    if (!(r[0x26] & 0x80)) return 0;
    int out = 0;
    // NR11/12/13/14: square 1, envelope volume and frequency
    for (int ch=0; ch<2; ++ch) {
      int base = ch ? 0x16 : 0x11;
      int freq = r[base+2] | ((r[base+3] & 7) << 8);
      int vol = (r[base+1] >> 4) & 15;
      if ((r[base+3] & 0x80) && vol) out += square(ph[ch], gbHz(freq), vol * 100);
    }
    // Wave channel: 32-sample wave RAM is not yet modeled, use a low-amplitude triangle.
    { int f = r[0x1D] | ((r[0x1E] & 7) << 8); int level = (r[0x1C] >> 5) & 3;
      if ((r[0x1E] & 0x80) && level) out += triangle(ph[2], gbHz(f) / 2.0f, (4-level)*120); }
    // Noise channel: LFSR noise controlled by NR43.
    if ((r[0x23] & 0x80) && (r[0x21] >> 4)) {
      uint32_t div = (r[0x22] & 7) ? ((r[0x22] & 7) * 16u) : 8u;
      div <<= ((r[0x22] >> 4) & 15);
      if (div < 8) div = 8;
      if ((ph[3]++ % (div / 8 + 1)) == 0) { uint16_t bit = (*lfsr ^ (*lfsr >> 1)) & 1; *lfsr = (*lfsr >> 1) | (bit << 14); if (r[0x22] & 8) *lfsr = (*lfsr & ~(1u<<6)) | (bit<<6); }
      out += ((*lfsr & 1) ? 1 : -1) * ((r[0x21] >> 4) * 100);
    }
    int left = (r[0x25] >> 4) & 7, right = r[0x25] & 7;
    out = out * (left + right + 1) / 8;
    return out;
  }
  static inline int nesSample() {
    int out = 0;
    // NES pulse 1/2 ($4000-7, $4004-7)
    for (int ch=0; ch<2; ++ch) {
      int b = ch * 4; uint8_t ctl = nesApu[b], lo = nesApu[b+2], hi = nesApu[b+3];
      int period = lo | ((hi & 7) << 8), vol = ctl & 15;
      if ((nesApu[0x15] & (1 << ch)) && !(hi & 0x80) && period > 7 && vol)
        out += square(phase[ch], 1789773.0f / (16.0f * (period + 1)), vol * 100);
    }
    { int p = nesApu[0x0A] | ((nesApu[0x0B] & 7) << 8); int v = nesApu[0x08] & 0x7F;
      if ((nesApu[0x15] & 4) && p > 1) out += triangle(phase[2], 1789773.0f / (32.0f * (p + 1)), v ? 450 : 0); }
    if (nesApu[0x15] & 8) { int v=nesApu[0x0C]&15; uint32_t rate=1u << ((nesApu[0x0E]>>4)&15); if ((phase[3]++ % (rate*3+1))==0) { uint16_t bit=(noiseLfsr[0]^(noiseLfsr[0]>>1))&1; noiseLfsr[0]=(noiseLfsr[0]>>1)|(bit<<14); } out += (noiseLfsr[0]&1 ? 1:-1)*v*90; }
    return out / 2;
  }
  static inline int a2600Sample() {
    int out=0;
    for (int ch=0; ch<2; ++ch) {
      int audc=audioReg[ch*2], audf=audioReg[ch*2+1]&31, vol=audioReg[4+ch]&15;
      if (!vol) continue;
      uint32_t rate=(audf+1)*((audc&3)==0 ? 1u : 2u);
      if ((phase[4+ch]++ % (rate+1))==0) { uint16_t bit=(noiseLfsr[ch+1]^(noiseLfsr[ch+1]>>1))&1; noiseLfsr[ch+1]=(noiseLfsr[ch+1]>>1)|(bit<<14); }
      int wave;
      switch (audc & 0x0F) { case 0: case 11: wave=(phase[4+ch]&8)?1:-1; break; case 4: case 5: case 12: case 13: wave=(phase[4+ch]&4)?1:-1; break; default: wave=(noiseLfsr[ch+1]&1)?1:-1; }
      out += wave*vol*180;
    }
    return out;
  }
  static inline int genesisSample() {
    int out=0;
    // SN76489: three square channels + noise. The 68k VDP-port writes are tracked in psgReg.
    for (int ch=0; ch<3; ++ch) { int period=gen::psgReg[ch*2]&0x3FF; int vol=gen::psgReg[ch*2+1]&15; if (!period) period=1; if (vol<15) out += square(phase[ch], 3579545.0f/(32.0f*period), (15-vol)*55); }
    { int vol=gen::psgReg[7]&15; if (vol<15) { if ((phase[3]++ % 32)==0) { uint16_t bit=(noiseLfsr[3]^(noiseLfsr[3]>>1))&1; noiseLfsr[3]=(noiseLfsr[3]>>1)|(bit<<14); } out += (noiseLfsr[3]&1 ? 1:-1)*(15-vol)*45; } }
    // Six low-cost sine-like FM voices from YM2612 key-on/frequency registers.
    for (int ch=0; ch<6; ++ch) {
      int bank=ch/3, c=ch%3; uint8_t key=gen::ymReg[0][0x28];
      bool on = (key & (1 << (c+1))) && ((key & 4) ? bank==1 : bank==0);
      int fnum=gen::ymReg[bank][0xA0+c] | ((gen::ymReg[bank][0xA4+c]&7)<<8);
      int block=(gen::ymReg[bank][0xA4+c]>>3)&7;
      if (on && fnum) { float hz=(fnum * 0.047f) * (1 << block); ymPhase[ch] += (uint32_t)((double)hz*4294967296.0/EMU32_AUDIO_SAMPLE_RATE); int32_t tri=triangle(ymPhase[ch], hz, 180); out += tri; }
    }
    return out/2;
  }
  static void frame(uint8_t system, uint32_t frameUs) {
    if (!EmuAudio::isReady()) return;
    uint32_t total = (uint64_t)EMU32_AUDIO_SAMPLE_RATE * frameUs + frameRemainder;
    size_t count = total / 1000000UL; frameRemainder = total % 1000000UL;
    if (count > 512) count = 512;
    for (size_t i=0; i<count; ++i) {
      int32_t v=0;
      switch (system) {
        case SYS_NES: v=nesSample(); break;
        case SYS_GB: v=gbSample(gIo, phase, &noiseLfsr[0]); break;
        case SYS_GBC: v=gbSample(gcIo, phase, &noiseLfsr[1]); break;
        case SYS_A2600: v=a2600Sample(); break;
        case SYS_GENESIS: v=genesisSample(); break;
      }
      // The lightweight register generators produce conservative raw amplitudes.
      // Raise them before the final 1/4 backend gain so ordinary game sounds are audible.
      v *= 2;
      if (v > 30000) v=30000; if (v < -30000) v=-30000;
      mix[i]=(int16_t)v;
    }
    EmuAudio::writeMono(mix, count);
  }
}
