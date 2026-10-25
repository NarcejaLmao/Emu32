// ============================================================================
//  Screenshots: hold SELECT and press RT (right trigger) during a game to save
//  the current picture as a PNG in /screenshots on the SD card.
//
//  The LCD is write-only (no MISO), so the picture cannot be read back from the
//  panel. Instead, while a capture is armed, each core's LCD output code also
//  copies every row it sends into a temporary buffer (shotRow / shotRowNext).
//  The buffer exists only for the capture (PSRAM if present, else internal
//  RAM), so normal play costs one pointer check per row and no extra memory.
//  The on-screen FPS counter and the "SHOT SAVED" label are drawn straight to
//  the LCD and are never part of the screenshot.
//
//  File name: /screenshots/<ROM name>_NNN.png  (NNN = first free number)
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
static uint16_t  shotRows = 0;             // distinct rows received so far
static uint8_t   shotRowSeen[192] = {};     // prevents duplicate scanlines counting as a complete frame
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
  // Validate every input before pointer arithmetic or memcpy. A negative width
  // cast to size_t would otherwise become a huge copy and corrupt memory.
  if (__builtin_expect(shotBuf == nullptr, 1)) return;
  if (line == nullptr || y < 0 || y >= (int)shotH) return;
  if (shotH == 0 || shotH > sizeof(shotRowSeen) / sizeof(shotRowSeen[0])) return;
  if (w <= 0 || w > SHOT_W) return;

  memcpy(shotBuf + (size_t)y * SHOT_W, line, (size_t)w * sizeof(*line));
  if (!shotRowSeen[y]) {
    shotRowSeen[y] = 1;
    shotRows++;
  }
}
static inline __attribute__((always_inline)) void shotSeqBegin() { shotSeqY = 0; }
static inline __attribute__((always_inline)) void shotRowNext(const uint16_t *line, int w) {
  if (__builtin_expect(shotBuf == nullptr, 1)) return;
  // Stop at the captured frame height. Letting the 16-bit cursor wrap could
  // make excess scanlines overwrite the beginning of the screenshot.
  if (shotSeqY >= shotH) return;
  shotRow(shotSeqY, line, w);
  ++shotSeqY;
}
// A whole-picture solid fill (GBC "LCD off" white screen) does not go through outLine.
static void shotFillAll(uint16_t wireColour) {
  if (!shotBuf) return;
  for (size_t i = 0; i < (size_t)SHOT_W * shotH; i++) shotBuf[i] = wireColour;
  memset(shotRowSeen, 1, shotH);
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
  memset(shotRowSeen, 0, sizeof(shotRowSeen));
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

// ---- minimal PNG writer ---------------------------------------------------------
// 8-bit RGB, no interlace. The pixel data is wrapped in *stored* (uncompressed) deflate blocks, so no
// compression library is needed and nothing but a small staging buffer is used. Every PNG reader opens it;
// the file is simply not compressed (about 3 bytes per pixel, ~80 KB for a 172x155 picture).
static uint32_t pngCrcTab[256];
static bool     pngCrcReady = false;
static File     pngF;
static bool     pngOk;
static uint32_t pngCrc, pngA, pngB, pngBlockLeft, pngRawLeft;
static uint8_t  pngBuf[1024];
static size_t   pngN;

static void pngFlush() {
  if (pngN && pngOk) pngOk = pngF.write(pngBuf, pngN) == pngN;
  pngN = 0;
}
static void pngOut(const uint8_t *p, size_t n, bool inCrc) {
  if (inCrc) {
    uint32_t c = pngCrc;
    for (size_t i = 0; i < n; i++) c = pngCrcTab[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    pngCrc = c;
  }
  while (n) {
    size_t take = sizeof(pngBuf) - pngN;
    if (take > n) take = n;
    memcpy(pngBuf + pngN, p, take);
    pngN += take; p += take; n -= take;
    if (pngN == sizeof(pngBuf)) pngFlush();
  }
}
static void pngBe32(uint8_t *d, uint32_t v) { d[0] = v >> 24; d[1] = v >> 16; d[2] = v >> 8; d[3] = v; }
static void pngChunkBegin(uint32_t len, const char *type) {
  uint8_t l[4]; pngBe32(l, len);
  pngOut(l, 4, false);
  pngCrc = 0xFFFFFFFFu;
  pngOut((const uint8_t *)type, 4, true);
}
static void pngChunkEnd() { uint8_t c[4]; pngBe32(c, ~pngCrc); pngOut(c, 4, false); }

// Feed raw (filtered) scanline bytes: splits them into stored deflate blocks and keeps the Adler-32.
static void pngRaw(const uint8_t *p, size_t n) {
  while (n && pngOk) {
    if (pngBlockLeft == 0) {
      const uint32_t size = pngRawLeft > 65535 ? 65535 : pngRawLeft;
      const uint8_t h[5] = { (uint8_t)(pngRawLeft <= 65535 ? 1 : 0), (uint8_t)size, (uint8_t)(size >> 8),
                             (uint8_t)~size, (uint8_t)(~size >> 8) };
      pngOut(h, 5, true);
      pngBlockLeft = size;
    }
    size_t take = n < pngBlockLeft ? n : pngBlockLeft;
    pngOut(p, take, true);
    for (size_t i = 0; i < take; i++) { pngA += p[i]; if (pngA >= 65521) pngA -= 65521; pngB += pngA; if (pngB >= 65521) pngB -= 65521; }
    pngBlockLeft -= take; pngRawLeft -= take; p += take; n -= take;
  }
}

// Write the captured picture as a PNG. Returns true only if every byte was written.
static bool shotWritePng(const char *path) {
  if (!pngCrcReady) {
    for (uint32_t i = 0; i < 256; i++) { uint32_t c = i; for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1; pngCrcTab[i] = c; }
    pngCrcReady = true;
  }
  pngF = SHOT_FS.open(path, FILE_WRITE);
  if (!pngF) return false;
  pngOk = true; pngN = 0; pngA = 1; pngB = 0; pngBlockLeft = 0;

  const uint32_t rowRaw = 1 + (uint32_t)SHOT_W * 3;                  // filter byte + RGB
  pngRawLeft = rowRaw * shotH;
  const uint32_t nBlocks = (pngRawLeft + 65534) / 65535;
  const uint32_t idatLen = 2 + pngRawLeft + 5 * nBlocks + 4;         // zlib header + blocks + Adler-32

  static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
  pngOut(sig, 8, false);

  uint8_t ihdr[13];
  pngBe32(ihdr, SHOT_W); pngBe32(ihdr + 4, shotH);
  ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;   // 8-bit, RGB, deflate, no filter method, no interlace
  pngChunkBegin(13, "IHDR"); pngOut(ihdr, 13, true); pngChunkEnd();

  pngChunkBegin(idatLen, "IDAT");
  const uint8_t zh[2] = { 0x78, 0x01 };
  pngOut(zh, 2, true);
  static uint8_t row[1 + SHOT_W * 3];
  for (int y = 0; y < (int)shotH && pngOk; y++) {
    const uint16_t *src = shotBuf + (size_t)y * SHOT_W;
    row[0] = 0;                                                      // filter: none
    for (int x = 0; x < SHOT_W; x++) {
      const uint16_t q = src[x];
      const uint16_t v = (q >> 8) | (q << 8);                        // undo the wire byte swap
      const uint8_t r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
      row[1 + x * 3 + 0] = (r5 << 3) | (r5 >> 2);
      row[1 + x * 3 + 1] = (g6 << 2) | (g6 >> 4);
      row[1 + x * 3 + 2] = (b5 << 3) | (b5 >> 2);
    }
    pngRaw(row, rowRaw);
  }
  uint8_t ad[4]; pngBe32(ad, (pngB << 16) | pngA);
  pngOut(ad, 4, true);
  pngChunkEnd();

  pngChunkBegin(0, "IEND"); pngChunkEnd();
  pngFlush();
  pngF.close();
  const bool ok = pngOk && pngRawLeft == 0;
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
  // Wrap the cached counter and rescan from the beginning. Otherwise, after
  // screenshot 9999, captures would stay disabled even if older files were deleted.
  if (n >= 10000) n = 1;
  if (!sdMount()) {
    why = "NO SD CARD";
  } else if (!sdEnsureDir(SHOT_DIR)) {
    why = "SHOT DIR FAILED";
  } else {
    for (; n < 10000; n++) {
      snprintf(path, sizeof(path), SHOT_DIR "/%s_%03u.png", base, (unsigned)n);
      if (!SHOT_FS.exists(path)) break;
    }
    if (n >= 10000) why = "TOO MANY SHOTS";
    else ok = shotWritePng(path);
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
