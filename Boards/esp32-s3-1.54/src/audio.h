#pragma once
// Waveshare ESP32-S3-LCD-1.54 / Touch-LCD-1.54 onboard ES8311 + NS4150B speaker.
// Hardware pinout follows Waveshare's official Arduino 01_audio_out example.
#include <Arduino.h>
#include <Wire.h>
#include <ESP_I2S.h>
#include <math.h>

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
  static constexpr int PIN_DOUT = 12; // ESP32 -> ES8311 DAC data
  static constexpr int PIN_DIN  = 11; // ES8311 ADC data -> ESP32 (unused for playback)
  static constexpr int PIN_AMP  = 7;  // NS4150B PA_CTRL, active HIGH
  static constexpr int PIN_I2C_SDA = 42;
  static constexpr int PIN_I2C_SCL = 41;
  static constexpr uint8_t ES8311_ADDR = 0x18;
  // Moderate game-audio gain: audible without the previous excessive output.
  static constexpr int SOFTWARE_GAIN_NUM = 1;
  static constexpr int SOFTWARE_GAIN_DEN = 2; // 50% PCM amplitude for clearly audible gameplay

  static I2SClass i2s;
  static bool ready = false;
  static bool codecReady = false;
  static int16_t block[EMU32_AUDIO_BLOCK_SAMPLES * 2];

  inline bool writeCodecReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ES8311_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  // Configure the ES8311 DAC over I2C. Sending I2S alone is silent until this
  // codec is reset, clocked, and its analog/DAC output stages are powered up.
  inline bool initCodec() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    Wire.beginTransmission(ES8311_ADDR);
    if (Wire.endTransmission() != 0) return false;

    bool ok = true;
    ok &= writeCodecReg(0x00, 0x1F); // reset
    delay(20);
    ok &= writeCodecReg(0x00, 0x00); // release reset
    ok &= writeCodecReg(0x00, 0x80); // power on
    delay(10);
    ok &= writeCodecReg(0x01, 0x3F); // enable codec clocks
    // 22.05 kHz, 16-bit I2S slave input, MCLK = sample rate * 256.
    ok &= writeCodecReg(0x02, 0x00);
    ok &= writeCodecReg(0x03, 0x10);
    ok &= writeCodecReg(0x04, 0x10);
    ok &= writeCodecReg(0x05, 0x00);
    ok &= writeCodecReg(0x06, 0x03); // BCLK divider
    ok &= writeCodecReg(0x07, 0x00);
    ok &= writeCodecReg(0x08, 0xFF); // LRCK divider
    ok &= writeCodecReg(0x09, 0x0C); // 16-bit I2S format
    ok &= writeCodecReg(0x0A, 0x0C);
    ok &= writeCodecReg(0x0D, 0x01); // power up analog circuitry (critical)
    ok &= writeCodecReg(0x0E, 0x02);
    ok &= writeCodecReg(0x12, 0x00); // power up DAC
    ok &= writeCodecReg(0x13, 0x10); // enable output driver
    ok &= writeCodecReg(0x1C, 0x6A); // DAC/ADC filter setting from ES8311 reference init
    ok &= writeCodecReg(0x31, 0x00); // DAC unmute
    ok &= writeCodecReg(0x32, 0xA0); // Lower codec output to a moderate ~50% target; no startup beep
    ok &= writeCodecReg(0x37, 0x08); // bypass DAC EQ
    delay(10);
    return ok;
  }

  inline bool begin() {
    pinMode(PIN_AMP, OUTPUT);
    digitalWrite(PIN_AMP, LOW); // keep amp muted while codec and I2S start
    codecReady = initCodec();
    if (!codecReady) return false;

    i2s.setPins(PIN_BCLK, PIN_LRCK, PIN_DOUT, PIN_DIN, PIN_MCLK);
    ready = i2s.begin(I2S_MODE_STD, EMU32_AUDIO_SAMPLE_RATE,
                      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);
    if (ready) {
      delay(5);
      digitalWrite(PIN_AMP, HIGH);
    }
    return ready;
  }

  // Submit signed 16-bit mono PCM; duplicate it to both codec slots.
  inline size_t writeMono(const int16_t *samples, size_t count) {
    if (!ready || !samples || count == 0) return 0;
    size_t done = 0;
    while (done < count) {
      const size_t n = min((size_t)EMU32_AUDIO_BLOCK_SAMPLES, count - done);
      for (size_t i = 0; i < n; ++i) {
        // Apply a software gain cap to avoid clipping while keeping game audio audible.
        int32_t quiet = ((int32_t)samples[done + i] * SOFTWARE_GAIN_NUM) / SOFTWARE_GAIN_DEN;
        block[i * 2] = (int16_t)quiet;
        block[i * 2 + 1] = (int16_t)quiet;
      }
      const size_t bytes = i2s.write((uint8_t *)block, n * 2 * sizeof(int16_t));
      const size_t frames = bytes / (2 * sizeof(int16_t));
      done += frames;
      if (frames == 0) break;
    }
    return done;
  }

  // Optional diagnostic tone helper. It is deliberately not called at startup.
  // Send lowercase t in Serial Monitor (115200 baud) to test the speaker at moderate volume.
  inline void testTone() {
    if (!ready) return;
    int16_t tone[EMU32_AUDIO_BLOCK_SAMPLES];
    const uint32_t total = EMU32_AUDIO_SAMPLE_RATE / 5; // 200 ms
    uint32_t phase = 0;
    const uint32_t step = (uint32_t)((880.0 * 4294967296.0) / EMU32_AUDIO_SAMPLE_RATE);
    uint32_t sent = 0;
    while (sent < total) {
      size_t n = min((uint32_t)EMU32_AUDIO_BLOCK_SAMPLES, total - sent);
      for (size_t i = 0; i < n; ++i) {
        phase += step;
        tone[i] = (phase & 0x80000000UL) ? 3000 : -3000;
      }
      if (writeMono(tone, n) == 0) break;
      sent += n;
    }
  }

  inline bool isReady() { return ready; }
  inline bool isCodecReady() { return codecReady; }
}
