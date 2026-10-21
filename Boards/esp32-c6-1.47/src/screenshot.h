// ============================================================================
//  Screenshots: hold SELECT and press RT (right trigger) during a game to save
//  the current picture as a 24-bit BMP in /screenshots on the SD card.
//
//  The LCD is write-only (no MISO), so the picture cannot be read back from the
//  panel. Instead, while a capture is armed, each core's LCD output code also
//  copies every row it sends into a temporary buffer (shotRow / shotRowNext).
//  The buffer exists only for the capture (PSRAM if present, else internal
//  RAM), so normal play costs one pointer check per row and no extra memory.
//  The on-screen FPS counter and the "SHOT SAVED" label are drawn straight to
//  the LCD and are never part of the screenshot.
//
//  File name: /screenshots/<ROM name>_NNN.bmp  (NNN = first free number)
//
//  Needs SHOT_FS defined by the sketch before this header (SD_MMC or SD), and
//  must be included after sd_rom.h / controller.h and BEFORE the core headers.
// ============================================================================
#ifndef EMU32_SCREENSHOT_H
#define EMU32_SCREENSHOT_H

#ifndef SHOT_FS
#error "Define SHOT_FS (SD_MMC or SD) before including screenshot.h"
#endif

#include <esp_heap_caps.h>

#define SHOT_DIR        "/screenshots"
#define SHOT_W          172                // widest picture any core draws (GAME_W)
#define SHOT_TIMEOUT    30                 // give up if no complete frame arrives within this many loop passes
#define SHOT_MSG_MS     1500               // how long the on-screen label stays up

static uint16_t *shotBuf  = nullptr;       // non-null only while a capture is armed (wire format: byte-swapped RGB565)
static uint16_t  shotH    = 0;             // rows in the picture being captured
static uint16_t  shotRows = 0;             // rows received so far
static uint16_t  shotSeqY = 0;             // row cursor for cores that push rows in order (NES)
static uint8_t   shotWait = 0;             // loop passes since arming
static bool      shotRequest = false;      // hotkey pressed: arm before the next frame
static uint16_t  shotNext = 1;             // next file number to try
static char      shotLastBase[44] = "";

// On-screen label (drawn by shotTick() in the sketch, like the FPS counter)
static char      shotMsg[28] = "";
static bool      shotMsgOk = true;
static uint32_t  shotMsgUntil = 0;

// Height (in rows) of the picture the given core outputs. Defined in core_manager.h,
// where the cores' size macros are visible.
static uint16_t shotPicH(uint8_t sys);

// ---- hooks called from the cores' LCD output code ---------------------------
static inline __attribute__((always_inline)) void shotRow(int y, const uint16_t *line, int w) {
  if (__builtin_expect(shotBuf == nullptr, 1)) return;
  if (y < 0 || y >= (int)shotH) return;
  if (w > SHOT_W) w = SHOT_W;
  memcpy(shotBuf + (size_t)y * SHOT_W, line, (size_t)w * 2);
  shotRows++;
}
static inline __attribute__((always_inline)) void shotSeqBegin() { shotSeqY = 0; }
static inline __attribute__((always_inline)) void shotRowNext(const uint16_t *line, int w) {
  if (__builtin_expect(shotBuf == nullptr, 1)) return;
  shotRow(shotSeqY++, line, w);
}
// A whole-picture solid fill (GBC "LCD off" white screen) does not go through outLine.
static void shotFillAll(uint16_t wireColour) {
  if (!shotBuf) return;
  for (size_t i = 0; i < (size_t)SHOT_W * shotH; i++) shotBuf[i] = wireColour;
  shotRows = shotH;
}

// ---- helpers ------------------------------------------------------------------
static void shotSetMsg(const char *m, bool ok) {
  strlcpy(shotMsg, m, sizeof(shotMsg));
  shotMsgOk = ok;
  shotMsgUntil = millis() + SHOT_MSG_MS;
  if (!shotMsgUntil) shotMsgUntil = 1;
}

static void shotAbort() {                  // drop any armed capture (leaving a game, failed frame, ...)
  if (shotBuf) { heap_caps_free(shotBuf); shotBuf = nullptr; }
  shotRows = 0; shotRequest = false;
}

// Armed from loop() just before a frame is emulated, so the capture covers one whole frame.
static bool shotBegin() {
  if (shotBuf) return true;
  shotH = shotPicH(curSys);
  if (shotH == 0 || shotH > 192) shotH = 129;
  const size_t bytes = (size_t)SHOT_W * shotH * 2;
  uint16_t *p = nullptr;
  if (psramFound()) p = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p)           p = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!p) {
    Serial.printf("SHOT: no RAM for %u byte capture buffer (largest internal block %u)\n",
                  (unsigned)bytes, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    shotSetMsg("NO RAM FOR SHOT", false);
    return false;
  }
  memset(p, 0, bytes);
  shotRows = 0; shotSeqY = 0; shotWait = 0;
  shotBuf = p;
  lcdDirty = true;                         // GBC skips unchanged bands unless told the LCD is stale: force a full repaint
  return true;
}

// ROM file name without folder or extension, with anything unsafe for a file name replaced by '_'.
static void shotBaseName(char *out, size_t n) {
  const char *s = strrchr(currentRomPath, '/');
  s = s ? s + 1 : currentRomPath;
  const char *dot = strrchr(s, '.');
  size_t stop = (dot && dot != s) ? (size_t)(dot - s) : strlen(s);
  size_t len = 0;
  for (size_t i = 0; i < stop && len < n - 1; i++) {
    const char c = s[i];
    const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                    c == ' ' || c == '-' || c == '_' || c == '(' || c == ')';
    out[len++] = ok ? c : '_';
  }
  out[len] = 0;
  if (!len) strlcpy(out, "shot", n);
}

static void shotPut32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// Write the captured picture as a bottom-up 24-bit BMP. Returns true only if every byte was written.
static bool shotWriteBmp(const char *path) {
  File f = SHOT_FS.open(path, FILE_WRITE);
  if (!f) return false;
  const uint32_t rowBytes = (uint32_t)SHOT_W * 3;            // 516: already a multiple of 4, so no row padding
  const uint32_t imgBytes = rowBytes * shotH;
  uint8_t hdr[54] = { 'B', 'M' };
  shotPut32(hdr + 2, 54 + imgBytes);
  shotPut32(hdr + 10, 54);                                   // pixel data offset
  shotPut32(hdr + 14, 40);                                   // BITMAPINFOHEADER size
  shotPut32(hdr + 18, SHOT_W);
  shotPut32(hdr + 22, shotH);
  hdr[26] = 1;                                               // planes
  hdr[28] = 24;                                              // bits per pixel
  shotPut32(hdr + 34, imgBytes);
  shotPut32(hdr + 38, 2835); shotPut32(hdr + 42, 2835);      // ~72 dpi
  bool ok = f.write(hdr, sizeof(hdr)) == sizeof(hdr);

  static uint8_t out[SHOT_W * 3 * 4];                        // 4 rows per SD write
  int y = (int)shotH - 1;
  while (ok && y >= 0) {
    int nRows = 0;
    for (; nRows < 4 && y >= 0; nRows++, y--) {
      const uint16_t *src = shotBuf + (size_t)y * SHOT_W;
      uint8_t *d = out + (size_t)nRows * rowBytes;
      for (int x = 0; x < SHOT_W; x++) {
        const uint16_t p = src[x];
        const uint16_t v = (p >> 8) | (p << 8);              // undo the wire byte swap
        const uint8_t r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
        d[x * 3 + 0] = (b5 << 3) | (b5 >> 2);                // BMP stores B, G, R
        d[x * 3 + 1] = (g6 << 2) | (g6 >> 4);
        d[x * 3 + 2] = (r5 << 3) | (r5 >> 2);
      }
    }
    const size_t want = (size_t)nRows * rowBytes;
    ok = f.write(out, want) == want;
  }
  f.close();
  if (!ok) SHOT_FS.remove(path);
  return ok;
}

static void shotSave() {
  char base[44], path[120];
  shotBaseName(base, sizeof(base));
  if (strcmp(base, shotLastBase) != 0) { strlcpy(shotLastBase, base, sizeof(shotLastBase)); shotNext = 1; }

  bool ok = false;
  const char *why = "SHOT FAILED";
  uint16_t n = shotNext;
  if (!sdMount()) {
    why = "NO SD CARD";
  } else if (!sdEnsureDir(SHOT_DIR)) {
    why = "SHOT DIR FAILED";
  } else {
    for (; n < 10000; n++) {
      snprintf(path, sizeof(path), SHOT_DIR "/%s_%03u.bmp", base, (unsigned)n);
      if (!SHOT_FS.exists(path)) break;
    }
    if (n >= 10000) why = "TOO MANY SHOTS";
    else ok = shotWriteBmp(path);
  }
  shotAbort();                                               // free the capture buffer first
  if (ok) {
    shotNext = n + 1;
    char m[28]; snprintf(m, sizeof(m), "SHOT %03u SAVED", (unsigned)n);
    shotSetMsg(m, true);
    Serial.printf("SHOT: saved %s\n", path);
  } else {
    shotSetMsg(why, false);
    Serial.printf("SHOT: failed (%s)\n", why);
  }
}

// ---- called from loop() ---------------------------------------------------------
// SELECT (View) + RT. Edge-triggered: hold it and you still get one screenshot.
static void shotHotkey() {
  static bool was = false;
  const bool now = (padState & 0x04) && bleRT;
  if (now && !was) shotRequest = true;
  was = now;
}

// Right before coreFrame(): arm the capture if the hotkey was pressed.
static void shotArm() {
  if (!shotRequest) return;
  shotRequest = false;
  shotBegin();
}

// Right after coreFrame(): finish (or time out) an armed capture. Returns true if the SD card was touched,
// so the caller can re-sync the frame timer.
static bool shotService() {
  if (!shotBuf) return false;
  if (shotRows >= shotH) { shotSave(); return true; }
  if (++shotWait > SHOT_TIMEOUT) {
    shotAbort();
    shotSetMsg("SHOT FAILED", false);
    Serial.println("SHOT: no complete frame was drawn");
  }
  return false;
}

#endif
