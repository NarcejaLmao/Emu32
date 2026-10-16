// Optional SD-card PNG box-art renderer for the ROM browser.
// Install the Arduino library "PNGdec" (Larry Bank) to enable PNG decoding.
// Without it, the firmware still builds and the browser shows a helpful placeholder.
#pragma once

#if defined(__has_include)
  #if __has_include(<PNGdec.h>)
    #include <PNGdec.h>
    #define EMU32_HAS_PNGDEC 1
  #endif
#endif
#ifndef EMU32_HAS_PNGDEC
  #define EMU32_HAS_PNGDEC 0
#endif

static const char *const boxArtFolder[SYS_COUNT] = { "nes", "gb", "gbc", "a2600" };

#if EMU32_HAS_PNGDEC
struct BoxArtDrawContext {
  int x, y, w, h;
  uint16_t background;
};
static PNG boxArtPng;
static uint16_t boxArtLine[320];
static uint16_t boxArtScaledLine[320];

static void *boxArtOpenFile(const char *filename, int32_t *fileSize) {
  if (!filename || !fileSize) return nullptr;
  File *f = new File;
  if (!f) return nullptr;
  *f = SD.open(filename, FILE_READ);
  if (!*f) { delete f; return nullptr; }
  const uint32_t n = f->size();
  if (!n || n > 2U * 1024U * 1024U) { f->close(); delete f; return nullptr; }
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
  if (!draw || !draw->pUser || draw->iWidth <= 0 || draw->iWidth > 320) return 0;
  BoxArtDrawContext *ctx = static_cast<BoxArtDrawContext *>(draw->pUser);
  boxArtPng.getLineAsRGB565(draw, boxArtLine, PNG_RGB565_BIG_ENDIAN, 0x00181818);
  const int sourceW = draw->iWidth;
  const int sourceH = boxArtPng.getHeight();
  if (sourceH <= 0 || ctx->w <= 0 || ctx->h <= 0) return 0;

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
    SPI.beginTransaction(lcdCfg);
    lcdWindow(ctx->x, ctx->y + dy, ctx->x + ctx->w - 1, ctx->y + dy);
    digitalWrite(PIN_LCD_DC, HIGH);
    digitalWrite(PIN_LCD_CS, LOW);
    SPI.writeBytes((uint8_t *)boxArtScaledLine, ctx->w * 2);
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

// Draw /imgs/<system>/<ROM filename without extension>.png. The PNG's own
// aspect ratio is preserved and it is centered inside the preview rectangle.
static bool boxArtDraw(const char *romName, int x, int y, int maxW, int maxH) {
  const uint16_t panel = col565(22, 30, 46);
  lcdFill(x, y, maxW, maxH, panel);
  if (!romName || curSys >= SYS_COUNT) {
    boxArtPlaceholder(x, y, maxW, maxH, "NO BOX ART");
    return false;
  }
  char name[ROM_NAME_LEN];
  baseName(romName, name, sizeof(name));
  char path[160];
  if (snprintf(path, sizeof(path), "/imgs/%s/%s.png", boxArtFolder[curSys], name) >= (int)sizeof(path)) {
    boxArtPlaceholder(x, y, maxW, maxH, "BAD ART PATH");
    return false;
  }
#if EMU32_HAS_PNGDEC
  BoxArtDrawContext ctx = { x, y, maxW, maxH, panel };
  // open() owns the SD File through the callbacks; only decode modest images
  // to keep scanline memory and SPI transfer time bounded on the ESP32-C6.
  if (boxArtPng.open(path, boxArtOpenFile, boxArtCloseFile, boxArtReadFile,
                     boxArtSeekFile, boxArtDrawLine) != 0) {
    const int iw = boxArtPng.getWidth(), ih = boxArtPng.getHeight();
    if (iw > 0 && ih > 0 && iw <= 320 && ih <= 640 && !boxArtPng.isInterlaced()) {
      const float scaleX = (float)maxW / (float)iw;
      const float scaleY = (float)maxH / (float)ih;
      const float scale = scaleX < scaleY ? scaleX : scaleY;
      ctx.w = (int)(iw * scale); ctx.h = (int)(ih * scale);
      if (ctx.w < 1) ctx.w = 1;
      if (ctx.h < 1) ctx.h = 1;
      ctx.x = x + (maxW - ctx.w) / 2;
      ctx.y = y + (maxH - ctx.h) / 2;
      const int decoded = boxArtPng.decode(&ctx, PNG_CHECK_CRC);
      boxArtPng.close();
      if (decoded == PNG_SUCCESS) return true;
    } else {
      boxArtPng.close();
    }
  } else {
    boxArtPng.close();
  }
  boxArtPlaceholder(x, y, maxW, maxH, "BAD PNG");
#else
  (void)path;
  boxArtPlaceholder(x, y, maxW, maxH, "PNGDEC NEEDED");
#endif
  return false;
}
