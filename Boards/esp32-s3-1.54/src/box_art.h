// Optional SD-card PNG box-art renderer for the ROM browser.
// Install the Arduino library "PNGdec" (Larry Bank) to enable PNG decoding.
// Without it, the firmware still builds and the browser shows a helpful placeholder.
#pragma once

// The main sketch includes <PNGdec.h> and defines EMU32_HAS_PNGDEC 1 so the
// Arduino IDE links the library. Without that define, art is disabled.
#ifndef EMU32_HAS_PNGDEC
  #define EMU32_HAS_PNGDEC 0
#endif

static const char *const boxArtFolder[SYS_COUNT] = { "nes", "gb", "gbc", "a2600", "genesis" };

#if EMU32_HAS_PNGDEC
struct BoxArtDrawContext {
  int x, y, w, h;
  uint16_t background;
};
#ifndef BOX_ART_MAX_W
  #define BOX_ART_MAX_W 320   // widest source PNG (matches the line buffers below)
#endif
static PNG boxArtPng;
static uint16_t boxArtLine[BOX_ART_MAX_W];
static uint16_t boxArtScaledLine[320];
// Cache the last decoded thumbnail. This avoids reopening and decoding the PNG
// every time menuDraw() redraws the selected row. The browser preview is at
// most 78x88 pixels (RGB565: 13.7 KiB).
#ifndef BOX_ART_CACHE_W
  #define BOX_ART_CACHE_W 78
#endif
#ifndef BOX_ART_CACHE_H
  #define BOX_ART_CACHE_H 88
#endif
static uint16_t boxArtPixels[BOX_ART_CACHE_W * BOX_ART_CACHE_H];
static char boxArtCachedPath[160];
static int boxArtCachedMaxW = 0, boxArtCachedMaxH = 0;
static int boxArtCachedW = 0, boxArtCachedH = 0;
static bool boxArtCacheValid = false;
static bool boxArtBuildingCache = false;

static void *boxArtOpenFile(const char *filename, int32_t *fileSize) {
  if (!filename || !fileSize) return nullptr;
  File *f = new File;
  if (!f) return nullptr;
  *f = SD_MMC.open(filename, FILE_READ);
  if (!*f) { delete f; return nullptr; }
  const uint32_t n = f->size();
  if (!n || n > 8U * 1024U * 1024U) { f->close(); delete f; return nullptr; }
  *fileSize = (int32_t)n;
  return f;
}
static void boxArtCloseFile(void *handle) {
  File *f = static_cast<File *>(handle);
  if (f) { f->close(); delete f; }
}
static int32_t boxArtReadFile(PNGFILE *file, uint8_t *buf, int32_t len) {
  if (!file || !file->fHandle || !buf || len <= 0) return 0;
  File *f = static_cast<File *>(file->fHandle);
  return (int32_t)f->read(buf, (size_t)len);
}
static int32_t boxArtSeekFile(PNGFILE *file, int32_t position) {
  if (!file || !file->fHandle || position < 0) return 0;
  File *f = static_cast<File *>(file->fHandle);
  return f->seek((uint32_t)position) ? 1 : 0;
}
static int boxArtDrawLine(PNGDRAW *draw) {
  if (!draw || !draw->pUser || draw->iWidth <= 0 || draw->iWidth > BOX_ART_MAX_W) return 0;
  BoxArtDrawContext *ctx = static_cast<BoxArtDrawContext *>(draw->pUser);
  if (!ctx) return 0;

  // Validate every destination bound before scaling or touching either buffer.
  // boxArtScaledLine is fixed at 320 pixels, while the cache has its own limits.
  const int scaledCapacity = (int)(sizeof(boxArtScaledLine) / sizeof(boxArtScaledLine[0]));
  if (ctx->w <= 0 || ctx->w > scaledCapacity || ctx->h <= 0) return 0;
  if (boxArtBuildingCache &&
      (ctx->w > BOX_ART_CACHE_W || ctx->h > BOX_ART_CACHE_H)) return 0;

  boxArtPng.getLineAsRGB565(draw, boxArtLine, PNG_RGB565_BIG_ENDIAN, 0x00181818);
  const int sourceW = draw->iWidth;
  const int sourceH = boxArtPng.getHeight();
  if (sourceH <= 0) return 0;

  // Nearest-neighbour scaling keeps memory use low: only one decoded source
  // scanline and one scaled output scanline are held at a time.
  const int firstY = (draw->y * ctx->h) / sourceH;
  const int nextY = ((draw->y + 1) * ctx->h) / sourceH;
  int yStart = firstY, yEnd = nextY;
  if (yEnd <= yStart) return 1; // this source line is skipped when downscaling
  if (yEnd > ctx->h) yEnd = ctx->h;
  for (int dy = yStart; dy < yEnd; ++dy) {
    for (int dx = 0; dx < ctx->w; ++dx) {
      int sx = (dx * sourceW) / ctx->w;
      if (sx >= sourceW) sx = sourceW - 1;
      boxArtScaledLine[dx] = boxArtLine[sx];
    }
    if (boxArtBuildingCache) {
      // Bounds were checked before entering the scaling loop.
      memcpy(&boxArtPixels[dy * BOX_ART_CACHE_W], boxArtScaledLine,
             (size_t)ctx->w * sizeof(boxArtPixels[0]));
    }
    // Always render the line now; when caching is enabled the same pixels are
    // also retained for subsequent redraws.
    SPI.beginTransaction(lcdCfg);
    lcdWindow(ctx->x, ctx->y + dy, ctx->x + ctx->w - 1, ctx->y + dy);
    digitalWrite(PIN_LCD_DC, HIGH);
    digitalWrite(PIN_LCD_CS, LOW);
    SPI.writeBytes((uint8_t *)boxArtScaledLine, (size_t)ctx->w * sizeof(boxArtScaledLine[0]));
    digitalWrite(PIN_LCD_CS, HIGH);
    SPI.endTransaction();
  }
  return 1;
}
#endif

static void boxArtPlaceholder(int x, int y, int w, int h, const char *message) {
  const uint16_t panel = col565(22, 30, 46);
  lcdFill(x, y, w, h, panel);
  lcdFill(x, y, w, 1, col565(45, 220, 190));
  lcdFill(x, y + h - 1, w, 1, col565(45, 220, 190));
  // Draw the tiny placeholder label only inside the artwork rectangle; using
  // lcdTextRow() here would clear a full-width stripe and erase menu entries.
  const int textW = (int)strlen(message) * 4;
  const int tx = x + (w - textW) / 2;
  const int ty = y + (h - 5) / 2;
  if (textW <= 0 || textW > 320 || tx < 0) return;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x, ty, x + w - 1, ty + 4);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int row = 0; row < 5; ++row) {
    for (int px = 0; px < w; ++px) fillBuf[px] = panel;
    for (int i = 0; message[i]; ++i) {
      const uint16_t g = glyph(message[i]);
      for (int col = 0; col < 3; ++col)
        if ((g >> (14 - (row * 3 + col))) & 1) {
          const int px = tx - x + i * 4 + col;
          if (px >= 0 && px < w) fillBuf[px] = col565(145, 160, 180);
        }
    }
    SPI.writeBytes((uint8_t *)fillBuf, w * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}


// Find the art file for a ROM. FAT is case-insensitive on most hosts, but a
// card written elsewhere may carry ".PNG" / ".png" or doubled extensions
// ("Game.png.png" from hidden Windows extensions), so try a few variants and
// finally scan the folder case-insensitively.
static bool boxArtResolve(const char *name, char *out, size_t cap) {
  static const char *const exts[] = { ".png", ".PNG", ".Png", ".png.png" };
  for (size_t i = 0; i < sizeof(exts) / sizeof(exts[0]); ++i) {
    if (snprintf(out, cap, "/imgs/%s/%s%s", boxArtFolder[curSys], name, exts[i]) >= (int)cap) return false;
    if (SD_MMC.exists(out)) return true;
  }
  char dirPath[40];
  snprintf(dirPath, sizeof(dirPath), "/imgs/%s", boxArtFolder[curSys]);
  File dir = SD_MMC.open(dirPath);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return false; }
  const size_t nl = strlen(name);
  bool found = false;
  for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
    if (!e.isDirectory()) {
      const char *n = e.name();
      const char *slash = strrchr(n, '/');
      if (slash) n = slash + 1;
      if (strncasecmp(n, name, nl) == 0 && strncasecmp(n + nl, ".png", 4) == 0) {
        snprintf(out, cap, "%s/%s", dirPath, n);
        found = true;
      }
    }
    e.close();
    if (found) break;
  }
  dir.close();
  return found;
}

// Draw /imgs/<system>/<ROM filename without extension>.png. The PNG's own
// aspect ratio is preserved and it is centered inside the preview rectangle.
static bool boxArtDraw(const char *romName, int x, int y, int maxW, int maxH) {
  const uint16_t panel = col565(22, 30, 46);
  lcdFill(x, y, maxW, maxH, panel);
  if (!romName || curSys >= SYS_COUNT) {
    boxArtPlaceholder(x, y, maxW, maxH, "NO ART");
    return false;
  }
  char name[ROM_NAME_LEN];
  baseName(romName, name, sizeof(name));
  char path[160];
  if (!sdMount()) {
    boxArtPlaceholder(x, y, maxW, maxH, "NO SD");
    return false;
  }
  if (!boxArtResolve(name, path, sizeof(path))) {
    Serial.printf("BoxArt: not found: /imgs/%s/%s.png\n", boxArtFolder[curSys], name);
#if EMU32_HAS_PNGDEC
    boxArtCacheValid = false;
#endif
    boxArtPlaceholder(x, y, maxW, maxH, "NO ART");
    return false;
  }
#if EMU32_HAS_PNGDEC
  // A cache hit still paints the preview, but skips SD open + PNG decoding.
  if (boxArtCacheValid && boxArtCachedMaxW == maxW && boxArtCachedMaxH == maxH &&
      strcmp(boxArtCachedPath, path) == 0) {
    const int drawX = x + (maxW - boxArtCachedW) / 2;
    const int drawY = y + (maxH - boxArtCachedH) / 2;
    SPI.beginTransaction(lcdCfg);
    digitalWrite(PIN_LCD_DC, HIGH);
    digitalWrite(PIN_LCD_CS, LOW);
    for (int row = 0; row < boxArtCachedH; ++row) {
      lcdWindow(drawX, drawY + row, drawX + boxArtCachedW - 1, drawY + row);
      SPI.writeBytes((uint8_t *)&boxArtPixels[row * BOX_ART_CACHE_W], boxArtCachedW * 2);
    }
    digitalWrite(PIN_LCD_CS, HIGH);
    SPI.endTransaction();
    return true;
  }
  boxArtCacheValid = false;
  BoxArtDrawContext ctx = { x, y, maxW, maxH, panel };
  const int rc = boxArtPng.open(path, boxArtOpenFile, boxArtCloseFile, boxArtReadFile,
                                boxArtSeekFile, boxArtDrawLine);
  if (rc != 0) {
    Serial.printf("BoxArt: open failed (%d) for %s\n", rc, path);
    boxArtPng.close();
    // PNG_TOO_BIG and friends: the image is wider than PNGdec's line buffer.
    boxArtPlaceholder(x, y, maxW, maxH, rc == PNG_TOO_BIG ? "PNG TOO WIDE" : "BAD PNG");
    return false;
  }
  const int iw = boxArtPng.getWidth(), ih = boxArtPng.getHeight();
  Serial.printf("BoxArt: %s %dx%d\n", path, iw, ih);
  if (iw <= 0 || ih <= 0) {
    boxArtPng.close();
    boxArtPlaceholder(x, y, maxW, maxH, "BAD PNG");
    return false;
  }
  if (iw > BOX_ART_MAX_W) {
    boxArtPng.close();
    boxArtPlaceholder(x, y, maxW, maxH, "PNG TOO WIDE");
    return false;
  }
  if (boxArtPng.isInterlaced()) {
    boxArtPng.close();
    boxArtPlaceholder(x, y, maxW, maxH, "INTERLACED");
    return false;
  }
  const float scaleX = (float)maxW / (float)iw;
  const float scaleY = (float)maxH / (float)ih;
  const float scale = scaleX < scaleY ? scaleX : scaleY;
  ctx.w = (int)(iw * scale); ctx.h = (int)(ih * scale);
  if (ctx.w < 1) ctx.w = 1;
  if (ctx.h < 1) ctx.h = 1;
  if (ctx.w > maxW) ctx.w = maxW;
  if (ctx.h > maxH) ctx.h = maxH;
  ctx.x = x + (maxW - ctx.w) / 2;
  ctx.y = y + (maxH - ctx.h) / 2;
  // Cache only thumbnails that fit the fixed preview buffer. If a future UI
  // asks for a larger preview, fall back to direct rendering without caching.
  boxArtBuildingCache = ctx.w <= BOX_ART_CACHE_W && ctx.h <= BOX_ART_CACHE_H;
  const int decoded = boxArtPng.decode(&ctx, 0);   // skip CRC check: faster, tolerant
  boxArtBuildingCache = false;
  boxArtPng.close();
  if (decoded == PNG_SUCCESS) {
    if (ctx.w <= BOX_ART_CACHE_W && ctx.h <= BOX_ART_CACHE_H) {
      strlcpy(boxArtCachedPath, path, sizeof(boxArtCachedPath));
      boxArtCachedMaxW = maxW; boxArtCachedMaxH = maxH;
      boxArtCachedW = ctx.w; boxArtCachedH = ctx.h;
      boxArtCacheValid = true;
    }
    return true;
  }
  Serial.printf("BoxArt: decode failed (%d)\n", decoded);
  boxArtPlaceholder(x, y, maxW, maxH, "BAD PNG");
#else
  (void)path;
  boxArtPlaceholder(x, y, maxW, maxH, "NEED PNGDEC");
#endif
  return false;
}
