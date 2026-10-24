// Extracted from the main sketch to keep this subsystem modular.
// ============================================================================
//  LCD (ST7789) driver
// ============================================================================
static SPISettings lcdCfg(LCD_SPI_HZ, MSBFIRST, SPI_MODE0);
static volatile bool lcdDirty = true;   // set whenever something other than the emulator painted the LCD: next frame resends every band
static uint16_t fillBuf[LCD_LONG];
static uint16_t outLine[GAME_W];


static uint16_t col565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  return (c >> 8) | (c << 8);
}

static void lcdSend(bool dc, const uint8_t *d, size_t n) {
  digitalWrite(PIN_LCD_DC, dc ? HIGH : LOW);
  digitalWrite(PIN_LCD_CS, LOW);
  SPI.writeBytes(d, n);
  digitalWrite(PIN_LCD_CS, HIGH);
}
static void lcdCmd0(uint8_t c) { lcdSend(false, &c, 1); }
static void lcdCmd1(uint8_t c, const uint8_t *d, size_t n) { lcdSend(false, &c, 1); lcdSend(true, d, n); }

static void lcdWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  if (scrW > scrH) {                               // landscape: the panel's short axis is Y
    y0 += LCD_XOFF; y1 += LCD_XOFF;
    x0 += LCD_YOFF; x1 += LCD_YOFF;
  } else {
    x0 += LCD_XOFF; x1 += LCD_XOFF;
    y0 += LCD_YOFF; y1 += LCD_YOFF;
  }
  uint8_t a[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  uint8_t b[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  lcdCmd1(0x2A, a, 4);
  lcdCmd1(0x2B, b, 4);
  lcdCmd0(0x2C);
}

static void lcdFill(int x, int y, int w, int h, uint16_t c) {
  // Clip to the current logical screen before forming LCD coordinates. Without
  // clipping, negative coordinates wrap when lcdWindow converts them to uint16_t,
  // and rectangles extending past an edge can address invalid panel RAM.
  if (w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= scrW || y >= scrH || w <= 0 || h <= 0) return;
  if (w > scrW - x) w = scrW - x;
  if (h > scrH - y) h = scrH - y;
  // fillBuf holds only LCD_LONG pixels; keep the transfer within its capacity.
  if (w <= 0 || h <= 0 || w > LCD_LONG) return;
  for (int i = 0; i < w; i++) fillBuf[i] = c;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x, y, x + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int r = 0; r < h; r++) SPI.writeBytes((uint8_t *)fillBuf, w * 2);
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// Use for UI/game transitions: clear every logical pixel and force band-based
// emulator renderers to repaint even if their frame hashes have not changed.
static void lcdClearTransition(uint16_t color = 0) {
  lcdFill(0, 0, LCD_W, LCD_H, color);
  lcdDirty = true;
}

// Draw a pre-swapped RGB565 image (e.g. the boot logo) at x,y. One row at a time
// through fillBuf, so the source can sit in flash.
static void lcdBlit(int x, int y, int w, int h, const uint16_t *img) {
  // Keep the original stride for source addressing while clipping the visible
  // rectangle. lcdWindow uses unsigned coordinates, so negative values must
  // never reach it (they would wrap to the opposite edge of panel RAM).
  if (!img || w <= 0 || h <= 0 || w > LCD_LONG) return;
  const int srcW = w;
  int srcX = 0, srcY = 0;
  if (x < 0) { srcX = -x; w -= srcX; x = 0; }
  if (y < 0) { srcY = -y; h -= srcY; y = 0; }
  if (x >= scrW || y >= scrH || w <= 0 || h <= 0) return;
  if (w > scrW - x) w = scrW - x;
  if (h > scrH - y) h = scrH - y;
  if (w <= 0 || h <= 0 || w > LCD_LONG) return;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x, y, x + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int r = 0; r < h; r++) {
    const uint16_t *src = img + (size_t)(r + srcY) * srcW + srcX;
    memcpy(fillBuf, src, (size_t)w * sizeof(fillBuf[0]));
    SPI.writeBytes((uint8_t *)fillBuf, (size_t)w * sizeof(fillBuf[0]));
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// Screen rotation: 4 orientations, 90 degrees clockwise per step (0 = portrait, 1 = landscape,
// 2 = portrait flipped, 3 = landscape flipped). Changes MADCTL, swaps the logical screen size,
// re-centres the game picture and blanks the screen; the caller redraws.
static void lcdSetRotation(uint8_t rot) {
  lcdDirty = true;
  lcdRot = rot & 3;
  const bool land = lcdRot & 1;
  scrW  = land ? LCD_LONG  : LCD_SHORT;
  scrH  = land ? LCD_SHORT : LCD_LONG;
  gameX = (scrW - GAME_W) / 2;
  gameY = land ? (scrH - GAME_H) / 2 : 0;          // portrait: picture pinned to the top
  SPI.beginTransaction(lcdCfg);
  uint8_t d[] = { lcdMadctl[lcdRot] };
  lcdCmd1(0x36, d, 1);
  SPI.endTransaction();
  lcdClearTransition(0);
}

static void lcdInit() {
  pinMode(PIN_LCD_CS, OUTPUT);  digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT);
  pinMode(PIN_LCD_BL, OUTPUT);  digitalWrite(PIN_LCD_BL, LOW);
  digitalWrite(PIN_LCD_RST, HIGH); delay(10);
  digitalWrite(PIN_LCD_RST, LOW);  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);

  SPI.beginTransaction(lcdCfg);
  lcdCmd0(0x01); delay(150);                       // software reset
  lcdCmd0(0x11); delay(120);                       // sleep out
  { uint8_t d[] = { 0x55 };                        lcdCmd1(0x3A, d, 1); }   // 16 bit colour
  { uint8_t d[] = { lcdMadctl[lcdRot] };           lcdCmd1(0x36, d, 1); }   // orientation (portrait at boot)
  { uint8_t d[] = { 0x0C,0x0C,0x00,0x33,0x33 };    lcdCmd1(0xB2, d, 5); }
  { uint8_t d[] = { 0x35 };                        lcdCmd1(0xB7, d, 1); }
  { uint8_t d[] = { 0x35 };                        lcdCmd1(0xBB, d, 1); }
  { uint8_t d[] = { 0x2C };                        lcdCmd1(0xC0, d, 1); }
  { uint8_t d[] = { 0x01 };                        lcdCmd1(0xC2, d, 1); }
  { uint8_t d[] = { 0x13 };                        lcdCmd1(0xC3, d, 1); }
  { uint8_t d[] = { 0x20 };                        lcdCmd1(0xC4, d, 1); }
  { uint8_t d[] = { 0x0F };                        lcdCmd1(0xC6, d, 1); }
  { uint8_t d[] = { 0xA4,0xA1 };                   lcdCmd1(0xD0, d, 2); }
  { uint8_t d[] = { 0xD0,0x08,0x11,0x08,0x0C,0x15,0x39,0x33,0x50,0x36,0x13,0x14,0x29,0x2D };
                                                   lcdCmd1(0xE0, d, 14); }
  { uint8_t d[] = { 0xD0,0x08,0x10,0x08,0x06,0x06,0x39,0x44,0x51,0x0B,0x16,0x14,0x2F,0x31 };
                                                   lcdCmd1(0xE1, d, 14); }
  lcdCmd0(LCD_INVERT ? 0x21 : 0x20);               // inversion (this IPS panel needs it on; see LCD_INVERT)
  lcdCmd0(0x13);                                   // normal mode
  lcdCmd0(0x29);                                   // display on
  SPI.endTransaction();
  lcdFill(0, 0, LCD_W, LCD_H, 0);
  digitalWrite(PIN_LCD_BL, HIGH);
}

// ---- tiny 3x5 font for error screens ---------------------------------------
static const uint16_t fontLetters[26] = {
  0x2BED,0x6BAE,0x3923,0x6B6E,0x79A7,0x79A4,0x396B,0x5BED,0x7497,0x126A,0x5BAD,0x4927,0x5FED,
  0x6B6D,0x2B6A,0x6BA4,0x2B7B,0x6BAD,0x388E,0x7492,0x5B6F,0x5B6A,0x5BFD,0x5AAD,0x5A92,0x72A7
};
static const uint16_t fontDigits[10] = {
  0x7B6F,0x2C97,0x62A7,0x628E,0x5BC9,0x798E,0x39AA,0x7292,0x2AAA,0x2ACE
};
static uint16_t glyph(char c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c >= 'A' && c <= 'Z') return fontLetters[c - 'A'];
  if (c >= '0' && c <= '9') return fontDigits[c - '0'];
  if (c == '.') return 0x0002;
  if (c == '/') return 0x12A4;
  if (c == '-') return 0x01C0;
  if (c == '_') return 0x0007;
  if (c == '(') return 0x2922;
  if (c == ')') return 0x224A;
  if (c == '[') return 0x6926;
  if (c == ']') return 0x324B;
  if (c == '\'') return 0x2400;
  if (c == ',') return 0x0014;
  if (c == '!') return 0x2482;
  if (c == ':') return 0x0410;
  if (c == '+') return 0x05D0;
  return 0;
}
// Draw one full-width row of text (LCD_W px wide, h px tall) in a single burst.
// Reuses fillBuf as the line buffer. Characters are 4*scale px apart.
static void lcdTextRow(int y, int h, const char *s, int scale, uint16_t fg, uint16_t bg, int x0 = 8) {
  const int len = strlen(s), textH = 5 * scale, top = (h - textH) / 2;
  SPI.beginTransaction(lcdCfg);
  lcdWindow(0, y, LCD_W - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int ly = 0; ly < h; ly++) {
    for (int x = 0; x < LCD_W; x++) fillBuf[x] = bg;
    if (ly >= top && ly < top + textH) {
      const int r = (ly - top) / scale;
      for (int i = 0; i < len; i++) {
        const uint16_t g = glyph(s[i]);
        const int gx = x0 + i * 4 * scale;
        if (gx + 3 * scale > LCD_W) break;
        if (!g) continue;
        for (int c = 0; c < 3; c++)
          if ((g >> (14 - (r * 3 + c))) & 1)
            for (int k = 0; k < scale; k++) fillBuf[gx + c * scale + k] = fg;
      }
    }
    SPI.writeBytes((uint8_t *)fillBuf, LCD_W * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

// Full-screen message: two centred-ish lines of text on a solid background.
static void showMessage(const char *l1, const char *l2, uint16_t bg) {
  lcdClearTransition(bg);
  lcdTextRow(LCD_H / 2 - 38, 20, l1, 2, col565(255, 255, 255), bg);
  lcdTextRow(LCD_H / 2 - 8,  20, l2, 2, col565(255, 200, 200), bg);
}

static void fatal(const char *l1, const char *l2) {
  Serial.printf("ERROR: %s / %s\n", l1, l2);
  showMessage(l1, l2, col565(110, 0, 0));
  while (true) delay(1000);
}

