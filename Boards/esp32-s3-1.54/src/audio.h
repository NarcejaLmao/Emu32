#pragma once
// Emu32 I2S audio output backend for Waveshare ESP32-S3 1.54-inch boards.
// Board audio pins: MCLK=8, BCLK=9, LRCK=10, DIN=11, DOUT=12, amplifier enable=7.
// This provides the I2S hardware output path for the per-system sound generators.
#include <Arduino.h>
#include <ESP_I2S.h>

#ifndef EMU32_AUDIO_SAMPLE_RATE
#define EMU32_AUDIO_SAMPLE_RATE 22050
#endif
#ifndef EMU32_AUDIO_BLOCK_SAMPLES
#define EMU32_AUDIO_BLOCK_SAMPLES 256
#endif

namespace EmuAudio {
  static constexpr int PIN_MCLK = 8;
  static constexpr int PIN_BCLK = 9;
  static constexpr int PIN_LRCK = 10;
  static constexpr int PIN_DIN  = 11;
  static constexpr int PIN_DOUT = 12;
  static constexpr int PIN_AMP  = 7;

  static I2SClass i2s;
  static bool ready = false;
  static int16_t block[EMU32_AUDIO_BLOCK_SAMPLES * 2];

  inline bool begin() {
    pinMode(PIN_AMP, OUTPUT);
    digitalWrite(PIN_AMP, LOW); // keep amplifier muted while configuring
    i2s.setPins(PIN_BCLK, PIN_LRCK, PIN_DOUT, PIN_DIN, PIN_MCLK);
    ready = i2s.begin(I2S_MODE_STD, EMU32_AUDIO_SAMPLE_RATE,
                      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    if (ready) digitalWrite(PIN_AMP, HIGH);
    return ready;
  }

  // Submit signed 16-bit mono PCM; duplicated to left/right. Keep calls in short
  // chunks so audio generation does not monopolize the emulator's frame loop.
  inline size_t writeMono(const int16_t *samples, size_t count) {
    if (!ready || !samples || count == 0) return 0;
    size_t done = 0;
    while (done < count) {
      const size_t n = min((size_t)EMU32_AUDIO_BLOCK_SAMPLES, count - done);
      for (size_t i = 0; i < n; ++i) {
        block[i * 2] = samples[done + i];
        block[i * 2 + 1] = samples[done + i];
      }
      const size_t bytes = i2s.write((uint8_t *)block, n * 2 * sizeof(int16_t));
      const size_t frames = bytes / (2 * sizeof(int16_t));
      done += frames;
      if (frames == 0) break;
    }
    return done;
  }


  inline bool isReady() { return ready; }
}
