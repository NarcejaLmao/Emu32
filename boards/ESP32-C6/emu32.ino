// =====================================================================================
//  Emu32  -  NES + GAME BOY + ATARI 2600 + MASTER SYSTEM EMULATOR  -  Waveshare ESP32-C6-LCD-1.47 (ST7789, 172x320, landscape)
// =====================================================================================
//  - At boot you first CHOOSE THE SYSTEM (NES, GAME BOY, ATARI 2600 or MASTER SYSTEM), then a GAME MENU lists every ROM in that
//    system's folder inside the "roms" folder on the microSD card:   "roms/nes" -> .nes files,   "roms/gb" -> .gb / .gbc files,
//    "roms/a2600" -> .a26 / .bin / .rom files,   "roms/sms" -> .sms files.
//      System screen: D-pad Up/Down (the list scrolls when there are more systems than fit) + A or Start to open the list.
//      Game list:     D-pad Up/Down (Left/Right = page) + A or Start to play,  B = back to the system screen.
//      The BOOT button is the Bluetooth PAIR button (in the menus and while playing).
//    The chosen ROM is stored and the board restarts once, then loads that ROM into RAM and runs it.
//  - While playing:  SELECT + X (hold Select, press X) = in-game menu (Resume / Reset game / Frameskip / FPS counter /
//                    Quit to game list / Quit to system menu).  Hold Start + Select = force quit to the "select system" screen.
//  - BOOT button = PAIR a Bluetooth LE gamepad (put the controller in pairing mode first, then press BOOT).
//    The paired controller is remembered and reconnects by itself after that.
//  - NES mappers: 0 (NROM), 1 (MMC1), 2 (UxROM), 3 (CNROM), 4 (MMC3), 7 (AxROM), 66 (GxROM).
//  - Game Boy (DMG): MBC1 / MBC3 / MBC5 cartridges, no sound, no save files, 4-shade green screen.
//  - Atari 2600 (NTSC colours): 2K / 4K / F8 / FA / F6 / F4 / E0 / 3F cartridges (auto-detected), no sound. D-pad = joystick, A or B = fire, Start = console RESET, Select = console SELECT.
//    Both difficulty switches are in position B (see A26_CONSOLE_DEFAULT). Picture is scaled to 229x172.
//  - Sega Master System (NTSC): Sega mapper + Codemasters mapper, 1 controller, no sound, no save files. D-pad = pad, A = button 1, B = button 2,
//    Start = PAUSE. Picture (256x192) is scaled to 229x172. ROMs are STREAMED FROM THE SD CARD (16 KB banks cached in free RAM), not loaded whole into RAM.
//  - No sound (the board has no audio hardware). NES / Game Boy picture is scaled to 197x172.
//
//  ADDING A FUTURE CONSOLE:  1) add a SYS_xxx id + bump NUM_SYS, 2) add one row to the SYS[] table (name, folder, extensions, colour),
//    3) add a loader + init branch in setup() and a frame / frame-time branch in runFrame() / frameUs(). The system screen scrolls by itself.
//
//  Controller notes: the ESP32-C6 only has Bluetooth LOW ENERGY (no Bluetooth Classic), so it works
//  with BLE gamepads: Xbox One S / Series controllers, 8BitDo (BLE mode), most generic BLE HID pads.
//  It will NOT work with PS4 / PS5 / Switch Pro controllers (those are Bluetooth Classic).
//    A button   = right / top face button (Xbox B or Y)
//    B button   = bottom / left face button (Xbox A or X)
//    Start      = Menu       Select = View        D-pad or left stick = directions
//
//  Files:  emu32.ino (this file: menus, Bluetooth, LED, LCD, glue) and the "src" folder next to it with
//    emu_common.h (shared pins / helpers) and nes_core.h, gb_core.h, a2600_core.h, sms_core.h (the four emulator cores).
//
//  Libraries (Library Manager):  "GFX Library for Arduino" (Moon On Our Nation)
//                                "NimBLE-Arduino" by h2zero, version 2.x
//  Board:  ESP32C6 Dev Module, Tools -> USB CDC On Boot -> Enabled
//  SD card: FAT32, with a folder named  roms  containing  nes  (your .nes files),  gb  (your .gb files),  a2600  (your .a26 / .bin files),  sms  (your .sms files).
// =====================================================================================
#pragma GCC optimize("O3")

#include <vector>
#include <algorithm>
#include <Arduino_GFX_Library.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <NimBLEDevice.h>
#include "src/emu_common.h"     // pins, screen geometry, SD / pad / LCD-row helpers shared by the cores
#include "src/nes_core.h"       // NES core
#include "src/gb_core.h"        // Game Boy core
#include "src/a2600_core.h"     // Atari 2600 core
#include "src/sms_core.h"       // Sega Master System core

#define PAD_DEBUG 0                    // 1 = print the controller bits to the Serial Monitor when they change (to check Start / Select)
#define FORCE_QUIT_MS 1000             // hold Start + Select this long to force quit to the system menu (0 = instantly)
#define A26_CONSOLE_DEFAULT 0          // console switches always on: 0 = both difficulty switches in B, colour TV. Add A26_DIFF0A / A26_DIFF1A for position A
#define LCD_SPI_HZ 80000000UL          // LCD SPI clock. One 2600 frame is ~79 KB, so 40 MHz alone costs ~16 ms per frame. Try 60000000UL or 40000000UL if you see glitches

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, GFX_NOT_DEFINED);
// rotation 1 = landscape. If the picture is upside down, change 1 to 3.
Arduino_GFX *panel = new Arduino_ST7789(bus, LCD_RST, 1, true, 172, 320, 34, 0, 34, 0);

// =====================================================================================
//  Onboard RGB LED (shows the Bluetooth state)
// =====================================================================================
#define LED_LEVEL    40    // brightness cap 0-255
#define LED_WRITE_MS 33
// Color order fix: 0 = as is, 1 = red/green swapped, 2 = green/blue swapped, 3 = red/blue swapped
#define LED_ORDER 1

static void ledRaw(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(LED_PIN, r, g, b);
#else
  neopixelWrite(LED_PIN, r, g, b);
#endif
}

static void ledShow(uint8_t r, uint8_t g, uint8_t b) {
  static uint8_t lr = 0, lg = 0, lb = 0;
  static bool first = true;
  static unsigned long lastWr = 0;
  uint8_t o[3] = {r, g, b};
  for (int i = 0; i < 3; i++) {
    uint16_t v = ((uint16_t)o[i] * LED_LEVEL) / 255;
    if (o[i] > 0 && v == 0) v = 1;
    o[i] = (uint8_t)v;
  }
  if (!first && o[0] == lr && o[1] == lg && o[2] == lb) return;
  unsigned long now = millis();
  if (!first && now - lastWr < LED_WRITE_MS) return;
  first = false; lastWr = now;
  lr = o[0]; lg = o[1]; lb = o[2];
#if LED_ORDER == 1
  ledRaw(lg, lr, lb);
#elif LED_ORDER == 2
  ledRaw(lr, lb, lg);
#elif LED_ORDER == 3
  ledRaw(lb, lg, lr);
#else
  ledRaw(lr, lg, lb);
#endif
}

// =====================================================================================
//  ATARI 2600 glue. The emulator itself (6507 + TIA + RIOT + cartridge banking) is in a2600_core.h;
//  this part only loads the ROM from the SD card, maps the controller and scales the picture.
// =====================================================================================
static uint8_t a26Xmap[A26_OUT_W];      // output column -> 2600 pixel (160 -> 229)

static bool a26LoadRom(const char *path) {
  if (!sdMount()) { romError = "SD card not found"; return false; }
  File f = SD.open(path, FILE_READ);
  if (!f) { romError = "ROM not found on SD"; sdUnmount(); return false; }
  size_t sz = f.size();
  if (sz < 2048) { romError = "ROM file is too small"; f.close(); sdUnmount(); return false; }
  romBuf = (uint8_t *)malloc(sz);
  if (!romBuf) {
    snprintf(romErrBuf, sizeof(romErrBuf), "ROM too big (%u KB)", (unsigned)(sz / 1024));
    romError = romErrBuf; f.close(); sdUnmount(); return false;
  }
  size_t got = 0;
  while (got < sz) {
    int n = f.read(romBuf + got, min((size_t)4096, sz - got));
    if (n <= 0) break;
    got += n;
  }
  f.close(); sdUnmount();
  if (got != sz) { romError = "Could not read ROM"; return false; }
  if (!a26Load(romBuf, (uint32_t)sz)) { romError = a26Error ? a26Error : "Cart type not supported"; return false; }   // romBuf must stay allocated
  Serial.printf("A2600 ROM: %u KB, mapper %d%s\n", (unsigned)(sz / 1024), (int)a26Map, a26SC ? " + Superchip RAM" : "");
  return true;
}

// Called by the core once per visible row (160 pixels). 192 rows -> 172 LCD rows, so some rows are dropped.
static void a26RowCb(int y, const uint16_t *src) {
  int o0 = (y * SH) / A26_H, o1 = ((y + 1) * SH) / A26_H;
  for (int r = o0; r < o1; r++) {
    uint16_t *dst = rowSlot(r);
    for (int ox = 0; ox < A26_OUT_W; ox++) dst[ox] = src[a26Xmap[ox]];
    rowDone();
  }
}

static void a26ReadPad() {                                // padBits -> joystick 1 + console switches
  uint8_t p = padBits, j = 0, c = A26_CONSOLE_DEFAULT;
  if (p & 0x10) j |= A26_UP;
  if (p & 0x20) j |= A26_DOWN;
  if (p & 0x40) j |= A26_LEFT;
  if (p & 0x80) j |= A26_RIGHT;
  if (p & 0x03) j |= A26_FIRE;                            // A or B
  if (p & 0x04) c |= A26_SELECT;
  if (p & 0x08) c |= A26_RESET;
  a26Pad0 = j; a26Console = c;
}

static void a26RunFrame(bool draw) {
  a26ReadPad();
  a26Frame(draw);
  flushRows();                                            // send the last rows of the picture
}

// =====================================================================================
//  MASTER SYSTEM glue. The core streams the ROM from the SD card while playing, so the SD card and the LCD share the SPI bus at run time.
//  Called by the core after every SD access: make sure neither chip is left selected so the LCD's next transfer starts clean
//  (the LCD driver sets its own SPI clock/mode for every transfer).
// =====================================================================================
static void sdBusRestore() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(LCD_CS, HIGH);
}

// =====================================================================================
//  Bluetooth LE gamepad (HID over GATT)
// =====================================================================================
// States for the UI/LED
enum { BT_IDLE, BT_SCAN, BT_CONNECTING, BT_CONNECTED, BT_FAILED };
static volatile int btState = BT_IDLE;
static volatile bool pairRequest = false;
static volatile uint32_t pairStartMs = 0;
static volatile uint32_t reconnectUntil = 0;      // after a restart (e.g. entering a game): until this time, look for the saved pad on our own
static Preferences prefs;
static char savedAddr[24] = "";
static NimBLEClient *hidClient = nullptr;

// --- tiny HID report-descriptor parser: finds buttons, hat and the left stick ---
struct HidField { uint8_t rid; uint16_t bitOff; uint8_t size; uint16_t page; uint16_t usage; int32_t lmin, lmax; };
static HidField fields[48];
static int nFields = 0;
static int maxBtn = 0;

static int32_t sext(uint32_t v, int bytes) {
  if (bytes == 1) return (int8_t)v;
  if (bytes == 2) return (int16_t)v;
  return (int32_t)v;
}

static void parseHid(const uint8_t *d, size_t n) {
  nFields = 0; maxBtn = 0;
  uint16_t page = 0; int32_t lmin = 0, lmax = 0;
  uint32_t rsize = 0, rcount = 0, bitOff = 0; uint8_t rid = 0;
  uint16_t usages[32]; int nu = 0; uint16_t umin = 0; bool hasRange = false;
  size_t i = 0;
  while (i < n) {
    uint8_t b = d[i++];
    if (b == 0xFE) { if (i < n) i += 2 + d[i]; continue; }          // long item: skip
    int sz = b & 3; if (sz == 3) sz = 4;
    int type = (b >> 2) & 3, tag = b >> 4;
    uint32_t val = 0;
    for (int k = 0; k < sz && i + k < n; k++) val |= (uint32_t)d[i + k] << (8 * k);
    i += sz;
    if (type == 1) {                                               // global items
      switch (tag) {
        case 0: page = val; break;
        case 1: lmin = sext(val, sz); break;
        case 2: lmax = (lmin < 0) ? sext(val, sz) : (int32_t)val; break;
        case 7: rsize = val; break;
        case 8: rid = val; bitOff = 0; break;
        case 9: rcount = val; break;
      }
    } else if (type == 2) {                                        // local items
      switch (tag) {
        case 0: if (nu < 32) usages[nu++] = val; break;
        case 1: umin = val; hasRange = true; break;
      }
    } else if (type == 0) {                                        // main items
      if (tag == 8) {                                              // Input
        if (!(val & 1)) {
          for (uint32_t k = 0; k < rcount; k++) {
            uint16_t u = 0;
            if (nu > 0) u = usages[k < (uint32_t)nu ? k : nu - 1];
            else if (hasRange) u = umin + k;
            if (nFields < 48 && (page == 1 || page == 9 || (page == 0x0C && (val & 2) && (u == 0x223 || u == 0x224)))) {   // + consumer "AC Home" / "AC Back" = the Guide button
              fields[nFields++] = {rid, (uint16_t)bitOff, (uint8_t)rsize, page, u, lmin, lmax};
            }
            if (page == 9 && (int)u > maxBtn) maxBtn = u;
            bitOff += rsize;
          }
        } else bitOff += rsize * rcount;
      }
      if (tag >= 8 && tag <= 12) { nu = 0; hasRange = false; }
    }
  }
}

static int32_t getBits(const uint8_t *d, size_t len, uint32_t off, int size, bool sgn) {
  uint32_t v = 0;
  for (int i = 0; i < size; i++) {
    uint32_t bit = off + i; size_t by = bit >> 3;
    if (by >= len) return 0;
    v |= (uint32_t)((d[by] >> (bit & 7)) & 1) << i;
  }
  if (sgn && size < 32 && (v & (1u << (size - 1)))) v |= ~((1u << size) - 1);
  return (int32_t)v;
}

// In-game menu = Select + X. The BLE code only records whether X is down (X shares the NES "B" bit with A,
// so it has its own flag); loop() watches for Select + X, then opens the menu.
static volatile bool padX = false;

static void decodeReport(uint8_t rid, const uint8_t *d, size_t len) {
  uint8_t out = 0;
#if PAD_DEBUG
  {
    static uint8_t lastRep[8][24]; static uint8_t lastLen[8];
    uint8_t slot = rid & 7; size_t n = len < 24 ? len : 24;
    if (lastLen[slot] != n || memcmp(lastRep[slot], d, n) != 0) {
      memcpy(lastRep[slot], d, n); lastLen[slot] = (uint8_t)n;
      Serial.printf("rid=%u len=%u:", rid, (unsigned)len);
      for (size_t i = 0; i < n; i++) Serial.printf(" %02X", d[i]);
      Serial.println();
    }
  }
#endif
  if (nFields > 0) {
    uint32_t btn = 0; int hat = -1; int ax = 0, ay = 0; bool gotX = false, gotY = false;
    bool gotPad = false, anyField = false, g224 = false;
    for (int i = 0; i < nFields; i++) {
      const HidField &f = fields[i];
      if (f.rid != rid) continue;
      anyField = true;
      int32_t v = getBits(d, len, f.bitOff, f.size, f.lmin < 0);
      if (f.page == 0x0C) { if (f.usage == 0x224 && v) g224 = true; continue; }   // consumer "AC Back" = View button (Select); Home etc. are ignored
      if (f.page == 1 && f.usage == 0x85) continue;                // "System Main Menu" (Xbox button) report: not part of the pad bits
      gotPad = true;
      if (f.page == 9) { if (v && f.usage > 0 && f.usage < 32) btn |= 1u << f.usage; }
      else if (f.page == 1) {
        if (f.usage == 0x39) { if (v >= f.lmin && v <= f.lmax) hat = v - f.lmin; }
        else if (f.usage == 0x30 && !gotX) { gotX = true; float r = f.lmax - f.lmin; if (v < f.lmin + r * 0.3f) ax = -1; else if (v > f.lmax - r * 0.3f) ax = 1; }
        else if (f.usage == 0x31 && !gotY) { gotY = true; float r = f.lmax - f.lmin; if (v < f.lmin + r * 0.3f) ay = -1; else if (v > f.lmax - r * 0.3f) ay = 1; }
      }
    }
    if (!anyField || !gotPad) return;                              // a report that is not the pad (battery, Xbox button...) must not clear the held buttons
    padX = (btn & (1u << 4)) != 0;                                 // Xbox X
    if (btn & ((1u << 2) | (1u << 5))) out |= 0x01;                // NES A  <- Xbox B / Y
    if (btn & ((1u << 1) | (1u << 3) | (1u << 4))) out |= 0x02;    // NES B  <- Xbox A / X
    if (g224 && gotPad) out |= 0x04;                               // View sent as consumer "AC Back" = Select
    bool few = (maxBtn <= 10);
    if (btn & ((1u << 11) | (1u << 9) | (few ? (1u << 7) : 0))) out |= 0x04;   // Select
    if (btn & ((1u << 12) | (1u << 10) | (few ? (1u << 8) : 0))) out |= 0x08;  // Start
    if (hat == 0 || hat == 1 || hat == 7) out |= 0x10;
    if (hat == 3 || hat == 4 || hat == 5) out |= 0x20;
    if (hat == 5 || hat == 6 || hat == 7) out |= 0x40;
    if (hat == 1 || hat == 2 || hat == 3) out |= 0x80;
    if (ay < 0) out |= 0x10; if (ay > 0) out |= 0x20;
    if (ax < 0) out |= 0x40; if (ax > 0) out |= 0x80;
  } else if (len >= 15) {                                           // no usable descriptor: assume the Xbox BLE layout
    uint8_t hat = d[12]; uint8_t b1 = d[13], b2 = d[14];
    if (b1 & 0x02) out |= 0x01; if (b1 & 0x10) out |= 0x01;         // B, Y
    if (b1 & 0x01) out |= 0x02; if (b1 & 0x08) out |= 0x02;         // A, X
    if (b2 & 0x04) out |= 0x04; if (b2 & 0x08) out |= 0x08;        // View, Menu
    padX = (b1 & 0x08) != 0;                                        // X
    if (hat == 1 || hat == 2 || hat == 8) out |= 0x10;
    if (hat == 4 || hat == 5 || hat == 6) out |= 0x20;
    if (hat == 6 || hat == 7 || hat == 8) out |= 0x40;
    if (hat == 2 || hat == 3 || hat == 4) out |= 0x80;
  } else return;
  padBits = out;
}

static NimBLERemoteCharacteristic *subChr[10];
static uint8_t subRid[10];
static int nSub = 0;

static void onHidNotify(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
  for (int i = 0; i < nSub; i++)
    if (subChr[i] == chr) { decodeReport(subRid[i], data, len); return; }
}

class PadClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {}
  void onDisconnect(NimBLEClient *c, int reason) override { padBits = 0; padX = false; btState = BT_IDLE; }
};
static PadClientCallbacks padCallbacks;

static bool connectHid(const NimBLEAddress &addr) {
  if (!hidClient) {
    hidClient = NimBLEDevice::createClient();
    hidClient->setClientCallbacks(&padCallbacks, false);
    hidClient->setConnectTimeout(8000);
  }
  if (!hidClient->connect(addr)) return false;
  hidClient->secureConnection();                                    // HID pads want an encrypted link (bonding)
  NimBLERemoteService *svc = hidClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!svc) { hidClient->disconnect(); return false; }

  nFields = 0;
  NimBLERemoteCharacteristic *map = svc->getCharacteristic(NimBLEUUID((uint16_t)0x2A4B));
  if (map && map->canRead()) {
    auto v = map->readValue();
    parseHid((const uint8_t *)v.data(), v.size());
  }
  nSub = 0;
  const auto &chars = svc->getCharacteristics(true);
  for (auto *c : chars) {
    if (c->getUUID() != NimBLEUUID((uint16_t)0x2A4D) || !c->canNotify()) continue;
    uint8_t rid = 0; bool isInput = true;
    NimBLERemoteDescriptor *d = c->getDescriptor(NimBLEUUID((uint16_t)0x2908));
    if (d) {
      auto rv = d->readValue();
      if (rv.size() >= 2) { rid = ((const uint8_t *)rv.data())[0]; isInput = (((const uint8_t *)rv.data())[1] == 1); }
    }
    if (!isInput || nSub >= 10) continue;
    subChr[nSub] = c; subRid[nSub] = rid; nSub++;
    c->subscribe(true, onHidNotify);
  }
  if (nSub == 0) { hidClient->disconnect(); return false; }
#if PAD_DEBUG
  Serial.printf("HID: %d parsed fields, %d buttons, %d input reports:", nFields, maxBtn, nSub);
  for (int i = 0; i < nSub; i++) Serial.printf(" rid%u", subRid[i]);
  Serial.println();
#endif
  return true;
}

static bool looksLikePad(const NimBLEAdvertisedDevice *d) {
  if (d->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) return true;
  if (d->haveName()) {
    std::string n = d->getName();
    const char *keys[] = {"ontroller", "amepad", "Xbox", "8Bit", "Joy", "Wireless Pad"};
    for (auto k : keys) if (n.find(k) != std::string::npos) return true;
  }
  return false;
}

// Starts the BLE stack (idempotent).
static bool btReady = false;
static void btInit() {
  if (btReady) return;
  NimBLEDevice::init("Emu32");
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(120);
  btReady = true;
}

static void btTask(void *arg) {
  btInit();
  NimBLEScan *scan = NimBLEDevice::getScan();
  for (;;) {
    if (btState == BT_CONNECTED && hidClient && hidClient->isConnected()) {
      if (pairRequest) { hidClient->disconnect(); padBits = 0; }   // BOOT again = drop this pad and scan for a different one
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }
    if (btState == BT_CONNECTED) {                                   // the link was lost while running: don't auto-reconnect later
      btState = BT_IDLE;
      if (prefs.getUChar("btlink", 0)) prefs.putUChar("btlink", 0);
    }
    bool pairing = pairRequest;
    if (pairing && millis() - pairStartMs > 30000) { pairRequest = false; pairing = false; btState = BT_IDLE; }
    bool reconn = !pairing && reconnectUntil && savedAddr[0];        // looking for the remembered pad only (no BOOT press needed)
    if (reconn && (int32_t)(millis() - reconnectUntil) > 0) {
      reconnectUntil = 0; reconn = false; btState = BT_IDLE;
      if (prefs.getUChar("btlink", 0)) prefs.putUChar("btlink", 0);
    }
    if (!pairing && !reconn) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }   // otherwise never scan on its own: only after BOOT is pressed

    btState = BT_SCAN;
    NimBLEScanResults res = scan->getResults(3000, false);
    const NimBLEAdvertisedDevice *best = nullptr;
    for (int i = 0; i < res.getCount(); i++) {
      const NimBLEAdvertisedDevice *d = res.getDevice(i);
      if (!looksLikePad(d)) continue;
      if (reconn && strcasecmp(d->getAddress().toString().c_str(), savedAddr) != 0) continue;
      if (!best || d->getRSSI() > best->getRSSI()) best = d;
    }
    if (!best) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }          // keep scanning until a pad is found or the 30 s window ends

    btState = BT_CONNECTING;
    NimBLEAddress addr = best->getAddress();
    std::string addrStr = addr.toString();
    if (connectHid(addr)) {
      btState = BT_CONNECTED;
      pairRequest = false; reconnectUntil = 0;
      if (!prefs.getUChar("btlink", 0)) prefs.putUChar("btlink", 1);     // remembered: reconnect by itself after the next restart (game launch)
      if (strcasecmp(addrStr.c_str(), savedAddr) != 0) {
        strncpy(savedAddr, addrStr.c_str(), sizeof(savedAddr) - 1);
        prefs.putString("pad", savedAddr);
      }
    } else {
      btState = BT_FAILED;
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// =====================================================================================
//  Status bars (left / right of the picture) and error screen
// =====================================================================================
static void drawLeftBar(const char *l1, const char *l2, uint16_t col) {
  panel->fillRect(0, 0, rowX0, 40, 0);
  panel->setTextSize(1);
  panel->setTextColor(col);
  panel->setCursor(2, 4);  panel->print(l1);
  int maxCh = (rowX0 - 2) / 6;
  static const char *const LONG1[] = { "PAIRING", "CONNECT", "SEARCH" };       // a narrow side bar: shorten the title there
  static const char *const SHORT1[] = { "PAIR", "CONN", "FIND" };
  if ((int)strlen(l1) > (rowX0 - 1) / 6)
    for (int i = 0; i < 3; i++) if (!strcmp(l1, LONG1[i])) { panel->fillRect(0, 4, rowX0, 8, 0); panel->setCursor(2, 4); panel->print(SHORT1[i]); break; }
  if ((int)strlen(l2) <= maxCh) { panel->setCursor(2, 16); panel->print(l2); }
  else {                                                  // narrow bar (Atari 2600 picture): split the 2nd line in two
    const char *cut = strpbrk(l2, " =");
    int n = cut ? (int)(cut - l2) + (*cut == '=' ? 1 : 0) : maxCh;
    char a[16]; snprintf(a, sizeof(a), "%.*s", n, l2);
    const char *rest = l2 + n; if (*rest == ' ') rest++;
    panel->setCursor(2, 16); panel->print(a);
    panel->setCursor(2, 28); panel->print(rest);
  }
}

static void showError(const char *msg, const char *h1, const char *h2, const char *h3) {
  panel->fillScreen(0);
  panel->setTextSize(2);
  panel->setTextColor(C(255, 90, 90));
  panel->setCursor(10, 20); panel->print("Emu32");
  panel->setTextColor(C(255, 255, 255));
  panel->setCursor(10, 60); panel->print(msg);
  panel->setTextSize(1);
  panel->setTextColor(C(180, 200, 255));
  panel->setCursor(10, 100); panel->print(h1);
  panel->setCursor(10, 114); panel->print(h2);
  panel->setCursor(10, 128); panel->print(h3);
}

// BOOT pressed: scan for 30 s. The saved pad (and its bond) is kept, so it simply reconnects; a different pad
// that is in pairing mode will be connected and become the new saved pad.
static void startPairing() {
  reconnectUntil = 0;
  pairStartMs = millis();
  pairRequest = true;
}

// The onboard LED mirrors the Bluetooth state (used by both the menu and the game screen).
static void ledForBt(int bs, uint32_t nowMs) {
  switch (bs) {
    case BT_CONNECTED:  ledShow(0, 255, 60); break;
    case BT_CONNECTING: ledShow(0, 200, 255); break;
    case BT_SCAN:       if ((nowMs / 250) % 2) ledShow(0, 0, 255); else ledShow(0, 0, 40); break;
    case BT_FAILED:     ledShow(255, 0, 0); break;
    default:            ledShow(255, 110, 0); break;
  }
}

// =====================================================================================
//  Menus: 1) choose the system (NES / GAME BOY)   2) list of the ROMs in that system's folder
// =====================================================================================
#define NES_DIR    "/roms/nes"
#define GB_DIR     "/roms/gb"
#define A26_DIR    "/roms/a2600"
#define SMS_DIR    "/roms/sms"
#define MAX_GAMES  256
#define MENU_ROWS  7
#define MENU_ROW_H 18
#define MENU_TOP   24
#define MENU_CHARS 25          // characters that fit in one row at text size 2 (12 px each)

enum { SYS_NES = 0, SYS_GB = 1, SYS_A26 = 2, SYS_SMS = 3 };
#define NUM_SYS 4                // number of rows in SYS[] below: add a console there and the system screen scrolls to fit it
#define CHOOSER_ROWS 3           // system cards visible at once (the list scrolls beyond that)

struct SysInfo {
  const char *name;              // card title on the system screen
  const char *title;             // short name for the game-list header
  const char *dir;               // ROM folder on the SD card
  uint16_t accent;
  const char *exts[4];           // ROM file extensions (nullptr = unused slot)
  const char *emptyMsg;          // shown when the folder has no ROMs
};
static const SysInfo SYS[NUM_SYS] = {
  { "NES",        "NES",     NES_DIR, C(255, 90, 90),   { ".nes", nullptr, nullptr, nullptr }, "No .nes files in \"roms/nes\"" },
  { "GAME BOY",   "GAME BOY", GB_DIR, C(150, 215, 110), { ".gb", ".gbc", nullptr, nullptr },    "No .gb files in \"roms/gb\"" },
  { "ATARI 2600", "2600",    A26_DIR, C(240, 160, 50),  { ".a26", ".bin", ".rom", nullptr },    "No .a26/.bin files in \"roms/a2600\"" },
  { "MASTER SYSTEM", "SMS",  SMS_DIR, C(60, 140, 255),  { ".sms", nullptr, nullptr, nullptr },    "No .sms files in \"roms/sms\"" },
};
enum { MODE_EMU, MODE_MENU, MODE_ERROR };
static int appMode = MODE_MENU;
static int emuSys = SYS_NES;            // which emulator runs when appMode == MODE_EMU
static bool errAutoReturn = false;      // ROM load errors go back to the menu by themselves

static std::vector<String> gameLists[NUM_SYS];   // only filled while the menu is running (nothing is allocated when playing)
static const char *menuError = nullptr;
static int menuLevel = 0;                  // 0 = system chooser, 1 = game list
static int menuSys = 0;                    // system shown in the game list / highlighted in the chooser
static int menuSel = 0, menuTop = 0;
static int chooserTop = 0;                 // first system card shown on the system screen
static uint32_t selSince = 0;
static int lastScroll = 0;
static int menuShownBt = -1;

static inline std::vector<String> &gl() { return gameLists[menuSys]; }
static inline const char *sysName(int s) { return SYS[s].name; }
static inline const char *sysTitle(int s) { return SYS[s].title; }
static inline const char *sysDir(int s) { return SYS[s].dir; }

static bool hasExt(const String &n, const char *ext) {
  int l = n.length(), e = strlen(ext);
  return l > e && n.substring(l - e).equalsIgnoreCase(ext);
}

static void scanDir(const char *path, std::vector<String> &out, const char *const *exts) {
  out.clear();
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
  for (File e = dir.openNextFile(); e && out.size() < MAX_GAMES; e = dir.openNextFile()) {
    if (!e.isDirectory()) {
      String n = e.name();
      int sl = n.lastIndexOf('/');
      if (sl >= 0) n = n.substring(sl + 1);
      bool match = false;
      for (int k = 0; k < 4 && exts[k] && !match; k++) match = hasExt(n, exts[k]);
      if (n.length() > 3 && n[0] != '.' && match) out.push_back(n);
    }
    e.close();
  }
  dir.close();
  std::sort(out.begin(), out.end(), [](const String &a, const String &b) { return strcasecmp(a.c_str(), b.c_str()) < 0; });
}

// Creates the ROM folders (/roms/...) and the BIOS folders (/bios/...) on the SD card if they are missing (SD must already be mounted).
// mkdir() doesn't create parent folders, so /roms and /bios are made before their sub-folders.
static void ensureRomDirs() {
  const char *dirs[NUM_SYS + 3];
  int nd = 0;
  dirs[nd++] = ROMS_DIR;
  for (int i = 0; i < NUM_SYS; i++) dirs[nd++] = SYS[i].dir;
  dirs[nd++] = BIOS_DIR;
  dirs[nd++] = GB_BIOS_DIR;
  for (int i = 0; i < nd; i++) {
    const char *d = dirs[i];
    if (SD.exists(d)) continue;
    if (SD.mkdir(d)) Serial.printf("Created %s\n", d);
    else             Serial.printf("Could not create %s\n", d);
  }
}

// Every system's folder is scanned once at boot (before the LCD starts, because the SD card shares its SPI pins).
static void scanGames() {
  if (!sdMount()) { menuError = "SD card not found"; return; }
  ensureRomDirs();                                      // first boot with a blank card: make roms/nes, roms/gb, roms/a2600
  bool any = false;
  for (int i = 0; i < NUM_SYS; i++) { scanDir(SYS[i].dir, gameLists[i], SYS[i].exts); if (!gameLists[i].empty()) any = true; }
  sdUnmount();
  if (!any) menuError = "No games found on SD";
}

static inline int menuItems() { return (int)gl().size(); }
static String menuLabel(int i) {
  const String &n = gl()[i];
  int dot = n.lastIndexOf('.');
  return dot > 0 ? n.substring(0, dot) : n;                         // hide the extension
}

static void menuDrawRow(int slot, int scroll) {
  int i = menuTop + slot, y = MENU_TOP + slot * MENU_ROW_H;
  bool sel = (i == menuSel);
  panel->fillRect(0, y, SW - 6, MENU_ROW_H, sel ? C(40, 80, 200) : 0);
  if (i >= menuItems()) return;
  String s = menuLabel(i);
  if ((int)s.length() > MENU_CHARS) {
    if (sel) s = s.substring(scroll, scroll + MENU_CHARS);          // selected long names scroll sideways
    else     s = s.substring(0, MENU_CHARS - 2) + "..";
  }
  uint16_t col = sel ? C(255, 255, 255) : C(190, 200, 220);
  panel->setTextSize(2);
  panel->setTextColor(col);
  panel->setCursor(8, y + 2);
  panel->print(s);
}

static void menuDrawBar() {
  int n = menuItems(), h = MENU_ROWS * MENU_ROW_H;
  panel->fillRect(SW - 4, MENU_TOP, 4, h, C(30, 30, 30));
  if (n > MENU_ROWS) {
    int th = max(8, h * MENU_ROWS / n);
    int ty = MENU_TOP + (h - th) * menuTop / (n - MENU_ROWS);
    panel->fillRect(SW - 4, ty, 4, th, C(140, 150, 180));
  }
}

static void menuDrawList() {
  for (int s = 0; s < MENU_ROWS; s++) menuDrawRow(s, 0);
  menuDrawBar();
  if (menuItems() == 0) {
    panel->setTextSize(1);
    panel->setTextColor(C(255, 190, 80));
    panel->setCursor(8, MENU_TOP + 8);
    panel->print(SYS[menuSys].emptyMsg);
    panel->setCursor(8, MENU_TOP + 24);
    panel->print("Press B to go back.");
  }
}

static void menuDrawStatus(int bs, bool pairNow) {
  const char *t; uint16_t col;
  switch (bs) {
    case BT_CONNECTED:  t = "PAD OK";     col = C(90, 255, 120); break;
    case BT_CONNECTING: t = "CONNECTING"; col = C(120, 220, 255); break;
    case BT_SCAN:       if (pairNow) { t = "PAIRING - pad mode"; col = C(90, 160, 255); }
                        else         { t = "SEARCHING";          col = C(150, 150, 150); } break;
    case BT_FAILED:     t = "FAILED";     col = C(255, 90, 90); break;
    default:            if (pairNow) { t = "PAIRING - pad mode"; col = C(90, 160, 255); }
                        else         { t = "NO PAD";             col = C(255, 190, 80); } break;
  }
  panel->fillRect(190, 0, SW - 190, 22, C(25, 25, 25));
  panel->setTextSize(1);
  panel->setTextColor(col);
  panel->setCursor(SW - 6 - 6 * (int)strlen(t), 7);
  panel->print(t);
}

static void menuDrawChooser() {
  if (menuSys < chooserTop) chooserTop = menuSys;                   // keep the highlighted system on screen: the list scrolls
  if (menuSys >= chooserTop + CHOOSER_ROWS) chooserTop = menuSys - CHOOSER_ROWS + 1;
  chooserTop = constrain(chooserTop, 0, max(0, NUM_SYS - CHOOSER_ROWS));
  for (int slot = 0; slot < CHOOSER_ROWS; slot++) {
    int i = chooserTop + slot, y = 26 + slot * 42;                  // 38 px cards between the header and the hint line
    if (i >= NUM_SYS) { panel->fillRect(0, y, SW - 6, 38, 0); continue; }
    bool sel = (i == menuSys);
    panel->fillRect(8, y, SW - 22, 38, sel ? C(40, 80, 200) : C(28, 30, 40));
    panel->fillRect(8, y, 6, 38, SYS[i].accent);
    panel->setTextSize(3);
    panel->setTextColor(sel ? C(255, 255, 255) : C(190, 200, 220));
    panel->setCursor(26, y + 2);
    panel->print(sysName(i));
    char t[24]; snprintf(t, sizeof(t), "%d games", (int)gameLists[i].size());
    panel->setTextSize(1);
    panel->setTextColor(sel ? C(210, 225, 255) : C(130, 140, 160));
    panel->setCursor(26, y + 28);
    panel->print(t);
  }
  int h = CHOOSER_ROWS * 42 - 4;                                    // scroll bar on the right edge (only when there is more to scroll to)
  panel->fillRect(SW - 4, 26, 4, h, 0);
  if (NUM_SYS > CHOOSER_ROWS) {
    panel->fillRect(SW - 4, 26, 4, h, C(30, 30, 30));
    int th = max(8, h * CHOOSER_ROWS / NUM_SYS);
    int ty = 26 + (h - th) * chooserTop / (NUM_SYS - CHOOSER_ROWS);
    panel->fillRect(SW - 4, ty, 4, th, C(140, 150, 180));
  }
}

static void menuDrawAll() {
  menuShownBt = -1;                                                 // force the Bluetooth status to be redrawn
  panel->fillScreen(0);
  panel->fillRect(0, 0, SW, 22, C(25, 25, 25));
  panel->setTextSize(2);
  panel->setTextColor(menuLevel == 0 ? C(255, 255, 255) : SYS[menuSys].accent);
  panel->setCursor(8, 3);
  if (menuLevel == 0) panel->print("SELECT SYSTEM");
  else { panel->print(sysTitle(menuSys)); panel->print(" GAMES"); }
  panel->setTextSize(1);
  panel->setTextColor(C(120, 120, 120));
  if (menuLevel == 0) {
    menuDrawChooser();
    panel->setTextSize(1);
    panel->setTextColor(C(120, 120, 120));
    panel->setCursor(4, 154);
    panel->print("Up/Down: choose    A: open    BOOT: pair controller");
  } else {
    menuDrawList();
    panel->setTextSize(1);                                         // the rows leave the text at size 2, so set the hint's size and color again
    panel->setTextColor(C(120, 120, 120));
    panel->setCursor(4, MENU_TOP + MENU_ROWS * MENU_ROW_H + 3);   // hint line at y=153, ends at y=161 (well inside the 172 px screen)
    panel->print("A: Play    B: Back    BOOT: Pair pad");
  }
}

static void menuInit() {
  menuLevel = 0;
  menuSys = constrain((int)prefs.getUChar("lastsys", 0), 0, NUM_SYS - 1);
  menuSel = 0; menuTop = 0;
  menuDrawAll();
}

static void menuEnterList() {
  menuLevel = 1;
  char lastKey[8]; snprintf(lastKey, sizeof(lastKey), "last%d", menuSys);
  String last = prefs.getString(lastKey, "");
  menuSel = 0;
  for (int i = 0; i < menuItems(); i++) if (gl()[i].equalsIgnoreCase(last)) { menuSel = i; break; }
  menuTop = constrain(menuSel - MENU_ROWS / 2, 0, max(0, menuItems() - MENU_ROWS));
  selSince = millis(); lastScroll = 0;
  menuDrawAll();
}

static void menuBack() {
  menuLevel = 0;
  menuDrawAll();
}

static void menuMove(int d) {
  int n = menuItems();
  if (n == 0) return;
  if (d == 1 || d == -1) menuSel = (menuSel + d + n) % n;           // single steps wrap around
  else                   menuSel = constrain(menuSel + d, 0, n - 1);  // page jumps stop at the ends
  if (menuSel < menuTop) menuTop = menuSel;
  if (menuSel >= menuTop + MENU_ROWS) menuTop = menuSel - MENU_ROWS + 1;
  selSince = millis(); lastScroll = 0;
  menuDrawList();
}

static void menuActivate() {
  if (menuItems() == 0) return;
  String name = gl()[menuSel];
  prefs.putString("sel", String(sysDir(menuSys)) + "/" + name);   // picked up once by setup() after the restart
  char lastKey[8]; snprintf(lastKey, sizeof(lastKey), "last%d", menuSys);
  prefs.putString(lastKey, name);
  prefs.putUChar("lastsys", (uint8_t)menuSys);
  panel->fillScreen(0);
  panel->setTextSize(2);
  panel->setTextColor(C(255, 255, 255));
  panel->setCursor(10, 60); panel->print("Loading...");
  panel->setTextSize(1);
  panel->setTextColor(C(180, 200, 255));
  panel->setCursor(10, 90); panel->print(menuLabel(menuSel).substring(0, 50));
  delay(100);
  ESP.restart();   // restart so the ROM is read from the SD card before the LCD takes over the SPI pins
}

static void menuTick() {
  uint32_t now = millis();

  // BOOT button = pair a Bluetooth controller (the controller does the menu navigation)
  static bool prevBoot = false;
  bool b = (digitalRead(BTN) == LOW);
  if (b && !prevBoot) startPairing();
  prevBoot = b;

  // Gamepad
  static uint8_t prevPad = 0;
  static uint32_t repeatAt = 0;
  uint8_t p = padBits, pressed = p & ~prevPad;
  if (menuLevel == 0) {
    if (pressed & 0xA0)      { menuSys = (menuSys + 1) % NUM_SYS; menuDrawChooser(); }              // Down / Right = next system
    else if (pressed & 0x50) { menuSys = (menuSys + NUM_SYS - 1) % NUM_SYS; menuDrawChooser(); }    // Up / Left = previous system
    else if (pressed & 0x09) menuEnterList();                       // A or Start
  } else {
    if      (pressed & 0x20) { menuMove(1);  repeatAt = now + 400; }
    else if (pressed & 0x10) { menuMove(-1); repeatAt = now + 400; }
    else if (pressed & 0x80) menuMove(MENU_ROWS);
    else if (pressed & 0x40) menuMove(-MENU_ROWS);
    else if (pressed & 0x02) menuBack();                            // B = back to the system screen
    else if (pressed & 0x09) menuActivate();                        // A or Start
    else if (repeatAt && now >= repeatAt && (p & 0x30) && (p & 0x30) != 0x30) {
      menuMove((p & 0x20) ? 1 : -1); repeatAt = now + 120;          // hold Up/Down to scroll
    }
    if (!(p & 0x30)) repeatAt = 0;
  }
  prevPad = p;

  // Scroll the selected name sideways when it is too long for the row
  if (menuLevel == 1 && menuItems() > 0) {
    int L = (int)menuLabel(menuSel).length();
    if (L > MENU_CHARS && now - selSince > 1200) {
      int span = L - MENU_CHARS;
      int pos = ((now - selSince - 1200) / 250) % (span + 6);
      if (pos > span) pos = span;
      if (pos != lastScroll) { lastScroll = pos; menuDrawRow(menuSel - menuTop, pos); }
    }
  }

  // Bluetooth status (top right) and LED
  static bool shownPair = false;
  int bs = btState; bool pn = pairRequest;
  if (bs != menuShownBt || pn != shownPair) { menuShownBt = bs; shownPair = pn; menuDrawStatus(bs, pn); }
  ledForBt(bs, now);
}

static void errorTick() {
  static uint32_t t0 = millis();
  uint32_t now = millis();
  if (now - t0 > 500 && digitalRead(BTN) == LOW) ESP.restart();     // BOOT = back to the menu / retry
  if (errAutoReturn && now - t0 > 5000) ESP.restart();
  delay(50);
}

// =====================================================================================
//  In-game menu (Select + X) and force quit (Start + Select)
// =====================================================================================
// Status bar / FPS state (file level so the in-game menu can force a redraw after it closes)
static int hudBt = -1;
static bool hudPair = false;
static uint32_t hudFpsMs = 0, hudFpsFrames = 0;

enum { FS_AUTO = 0, FS_OFF, FS_1, FS_2, FS_3, FS_COUNT };           // frameskip modes
static uint8_t fsMode = FS_AUTO;     // Auto = skip pictures only when the emulator falls behind; 1/2/3 = always skip that many in a row
static bool showFps = true;
static String curRomPath;            // the ROM that is running (used by "Reset game")
static const char *const FS_NAMES[FS_COUNT] = { "Auto", "Off", "1", "2", "3" };

enum { OV_RESUME, OV_RESET, OV_FSKIP, OV_FPS, OV_QLIST, OV_QSYS, OV_COUNT };

// Restart into the menu. toList = open that system's game list, otherwise the "select system" screen.
// (A restart is needed because the SD card and the LCD share the SPI pins.)
static void quitToMenu(bool toList) {
  prefs.putUChar("nosplash", 1);                                     // no 3 s logo when you come from a game
  if (toList) prefs.putUChar("openlist", (uint8_t)(emuSys + 1));
  panel->fillScreen(0);
  panel->setTextSize(2); panel->setTextColor(C(255, 255, 255));
  panel->setCursor(10, 70); panel->print("Loading menu...");
  delay(100);
  ESP.restart();
}

// Hold Start + Select for FORCE_QUIT_MS = back to the "select system" screen. Called every frame (and inside the in-game menu).
static void forceQuitCheck(bool hint = true) {
  static uint32_t comboAt = 0;
#if PAD_DEBUG
  static uint8_t lastPad = 0xFF;
  if (padBits != lastPad) { lastPad = padBits; Serial.printf("padBits=%02X (Start=08 Select=04)\n", lastPad); }
#endif
  if ((padBits & 0x0C) == 0x0C) {
    if (!comboAt) {
      comboAt = millis();
      if (hint) drawLeftBar("QUIT?", "keep hold", C(255, 190, 80));       // proves that BOTH buttons are being read
    } else if (millis() - comboAt >= FORCE_QUIT_MS) quitToMenu(false);
  } else if (comboAt) {
    comboAt = 0;
    if (hint) hudBt = -1;                                                // released early: bring the normal status text back
  }
}

static void ovLabel(int i, char *t, size_t n) {
  switch (i) {
    case OV_RESUME: snprintf(t, n, "Resume"); break;
    case OV_RESET:  snprintf(t, n, "Reset game"); break;
    case OV_FSKIP:  snprintf(t, n, "Frameskip: %s", FS_NAMES[fsMode]); break;
    case OV_FPS:    snprintf(t, n, "FPS counter: %s", showFps ? "On" : "Off"); break;
    case OV_QLIST:  snprintf(t, n, "Quit to game list"); break;
    default:        snprintf(t, n, "Quit to system menu"); break;
  }
}

static void ovDrawRow(int i, int sel) {
  int y = MENU_TOP + i * MENU_ROW_H;
  char t[32]; ovLabel(i, t, sizeof(t));
  panel->fillRect(0, y, SW, MENU_ROW_H, i == sel ? C(40, 80, 200) : 0);
  panel->setTextSize(2);
  panel->setTextColor(i == sel ? C(255, 255, 255) : C(190, 200, 220));
  panel->setCursor(8, y + 2);
  panel->print(t);
}

static void ovDrawAll(int sel) {
  panel->fillScreen(0);
  panel->fillRect(0, 0, SW, 22, C(25, 25, 25));
  panel->setTextSize(2);
  panel->setTextColor(SYS[emuSys].accent);
  panel->setCursor(8, 3); panel->print("PAUSED");
  for (int i = 0; i < OV_COUNT; i++) ovDrawRow(i, sel);
  panel->setTextSize(1);
  panel->setTextColor(C(120, 120, 120));
  panel->setCursor(4, MENU_TOP + OV_COUNT * MENU_ROW_H + 6);
  panel->print("A: Select   Left/Right: Change   B: Resume");
}

// Runs until the player resumes (or quits / resets, which restart the board).
static void inGameMenu() {
  flushRows();
  int sel = 0;
  uint8_t prev = padBits;
  bool prevBoot = (digitalRead(BTN) == LOW);
  ovDrawAll(sel);
  for (bool done = false; !done; ) {
    uint32_t now = millis();
    uint8_t p = padBits, pressed = p & ~prev; prev = p;
    bool boot = (digitalRead(BTN) == LOW);
    if (boot && !prevBoot) { while (digitalRead(BTN) == LOW) delay(10); break; }   // BOOT also closes it (e.g. pad lost)
    prevBoot = boot;

    int old = sel;
    if (pressed & 0x20) sel = (sel + 1) % OV_COUNT;
    if (pressed & 0x10) sel = (sel + OV_COUNT - 1) % OV_COUNT;
    if (sel != old) { ovDrawRow(old, sel); ovDrawRow(sel, sel); }
    bool a = pressed & 0x01, left = pressed & 0x40, right = pressed & 0x80;
    if (pressed & 0x02) done = true;                                 // B = resume
    switch (sel) {
      case OV_RESUME: if (a) done = true; break;
      case OV_RESET:
        if (a) {
          if (curRomPath.length()) prefs.putString("sel", curRomPath);   // reload the same ROM = a clean power-on reset
          panel->fillScreen(0); panel->setTextSize(2); panel->setTextColor(C(255, 255, 255));
          panel->setCursor(10, 70); panel->print("Resetting...");
          delay(100); ESP.restart();
        }
        break;
      case OV_FSKIP:
        if (a || right) fsMode = (fsMode + 1) % FS_COUNT;
        else if (left)  fsMode = (fsMode + FS_COUNT - 1) % FS_COUNT;
        if (a || left || right) { prefs.putUChar("fskip", fsMode); ovDrawRow(sel, sel); }
        break;
      case OV_FPS:
        if (a || left || right) { showFps = !showFps; prefs.putUChar("showfps", showFps ? 1 : 0); ovDrawRow(sel, sel); }
        break;
      case OV_QLIST: if (a) quitToMenu(true); break;
      default:       if (a) quitToMenu(false); break;
    }
    forceQuitCheck(false);
    ledForBt(btState, now);
    delay(10);
  }
  panel->fillScreen(0);                                              // the next frame repaints the whole picture
}

// =====================================================================================
//  Startup splash screen (shown for 3 seconds before the system selection menu)
// =====================================================================================
#define SPLASH_MS 3000

static void drawCentered(const char *txt, int size, int y) {
  int w = (int)strlen(txt) * 6 * size;                              // built-in font: 6 px per character at size 1
  panel->setTextSize(size);
  panel->setCursor((SW - w) / 2, y);
  panel->print(txt);
}

static void showSplash() {
  panel->fillScreen(0);
  panel->setTextColor(C(90, 160, 255));
  drawCentered("Emu32", 7, 32);                                     // big title
  panel->setTextColor(C(190, 200, 220));
  drawCentered("For all your retro", 2, 104);                       // smaller tagline (two lines to fit the screen)
  drawCentered("gaming needs.", 2, 124);
  delay(SPLASH_MS);
}

// =====================================================================================
//  setup / loop
// =====================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(BTN, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  ledShow(255, 120, 0);

  prefs.begin("emu32", false);
  fsMode = (uint8_t)constrain((int)prefs.getUChar("fskip", FS_AUTO), 0, FS_COUNT - 1);
  showFps = prefs.getUChar("showfps", 1) != 0;
  bool skipSplash = prefs.getUChar("nosplash", 0) != 0;                // set when you quit from a game
  int openList = prefs.getUChar("openlist", 0);                        // 1 + system whose game list to open
  if (skipSplash) prefs.putUChar("nosplash", 0);
  if (openList)   prefs.putUChar("openlist", 0);
  String sel = prefs.getString("sel", "");
  if (sel.length()) {
    curRomPath = sel;
    prefs.putString("sel", "");                         // one-shot: a reset after this returns to the menu
    emuSys = SYS_NES;
    for (int i = 0; i < NUM_SYS; i++) if (sel.startsWith(String(SYS[i].dir) + "/")) { emuSys = i; break; }
    if (emuSys == SYS_SMS) btInit();                    // start the Bluetooth stack first: the SMS ROM cache then sizes itself from the heap that is really left (BLE needs a lot of heap, and it used to be started after the ROM cache had taken everything)
    bool ok = (emuSys == SYS_GB)  ? gbLoadRom(sel.c_str())
            : (emuSys == SYS_A26) ? a26LoadRom(sel.c_str())
            : (emuSys == SYS_SMS) ? smsStreamOpen(sel.c_str())   // opens the ROM on the SD card only (no copy in RAM); the card is re-mounted after the LCD starts
            :                       loadRom(sel.c_str());   // SD first, while the LCD is not using the SPI pins
    if (ok) appMode = MODE_EMU;
    else { appMode = MODE_ERROR; errAutoReturn = true; }
  } else {
    scanGames();                                        // same reason: list the SD card before starting the LCD
    appMode = menuError ? MODE_ERROR : MODE_MENU;
  }

  if (!panel->begin(LCD_SPI_HZ)) {
    Serial.println("Display init failed");
    while (true) delay(1000);
  }
  panel->fillScreen(0);

  if (appMode == MODE_ERROR) {
    const char *msg = romError ? romError : (menuError ? menuError : "Error");
    Serial.println(msg);
    if (errAutoReturn) showError(msg, "Going back to the game menu...", "", "(or press BOOT now)");
    else               showError(msg, "Put ROMs in roms/nes, gb, a2600 or sms", "(.nes .gb .gbc .a26 .bin .rom .sms) on a", "FAT32 microSD card, then press BOOT.");
    ledShow(255, 0, 0);
    return;
  }

  // Bluetooth is needed both for navigating the menu and for playing
  String s = prefs.getString("pad", "");
  strncpy(savedAddr, s.c_str(), sizeof(savedAddr) - 1);
  if (savedAddr[0] && prefs.getUChar("btlink", 0)) reconnectUntil = millis() + 15000;   // pad was connected before this restart (game launch / quit): find it again
  xTaskCreate(btTask, "bt", 8192, nullptr, 2, nullptr);   // Bluetooth runs in its own task

  if (appMode == MODE_MENU) {
    if (!skipSplash) showSplash();                      // "Emu32" for 3 s (Bluetooth keeps connecting in the background)
    menuInit();
    if (openList >= 1 && openList <= NUM_SYS) {         // "Quit to game list": jump straight back into that system's list
      menuSys = openList - 1;
      if (menuItems() > 0) menuEnterList();
    }
    return;
  }

  if (emuSys == SYS_GB) {
    gbInit();
    gbReset();
    Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
    return;
  }

  if (emuSys == SYS_A26) {
    for (int i = 0; i < A26_OUT_W; i++) a26Xmap[i] = (i * A26_W) / A26_OUT_W;
    rowW = A26_OUT_W; rowX0 = A26_OUT_X0;                // the 2600 picture is wider than the NES / GB one
    a26RowFn = a26RowCb;
    a26Reset();
    Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
    return;
  }

  if (emuSys == SYS_SMS) {
    smsAfterSd = sdBusRestore;
    smsInit();
    rowW = SMS_OUT_W; rowX0 = SMS_OUT_X0;                // same 229 px wide picture as the 2600
    if (!smsStreamResume()) {                            // LCD is up: mount the card again on the shared SPI bus and re-open the ROM (needed before smsReset maps the banks)
      appMode = MODE_ERROR; errAutoReturn = true;
      const char *msg = romError ? romError : "SD card lost";
      Serial.println(msg);
      showError(msg, "Going back to the game menu...", "", "(or press BOOT now)");
      ledShow(255, 0, 0);
      return;
    }
    smsReset();
    Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
    return;
  }

  nesInit();
  nesReset();

  Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
}

static void runFrame(bool draw) {
  switch (emuSys) {
    case SYS_GB:  gbFrame(draw); break;
    case SYS_A26: a26RunFrame(draw); break;
    case SYS_SMS: smsFrame(draw); break;
    default:      emuFrame(draw); break;
  }
}

static uint32_t frameUs() {
  switch (emuSys) {
    case SYS_GB:  return GB_FRAME_US;
    case SYS_A26: return a26FrameUs();
    case SYS_SMS: return SMS_FRAME_US;
    default:      return FRAME_US;
  }
}

void loop() {
  if (appMode == MODE_ERROR) { errorTick(); return; }
  if (appMode == MODE_MENU)  { menuTick(); delay(10); return; }

  static uint32_t nextUs = micros();
  static bool skipNext = false;
  static int skipCount = 0;
  static uint32_t frameNo = 0;
  static bool prevBtn = false;

  // BOOT button = pair a Bluetooth controller
  bool b = (digitalRead(BTN) == LOW);
  if (b && !prevBtn) startPairing();
  prevBtn = b;

  // Hold Start + Select = force quit to the "select system" screen
  forceQuitCheck();

  // Select + X = in-game menu; the game is paused while it is open (fires once per press of the combo)
  static bool comboUsed = false;
  bool combo = (padBits & 0x04) && padX;
  if (!combo) comboUsed = false;
  if (combo && !comboUsed) {
    comboUsed = true;
    inGameMenu();
    nextUs = micros(); skipNext = false; skipCount = 0;              // restart the frame pacing after the pause
    hudBt = -1; hudFpsMs = millis(); hudFpsFrames = 0;               // redraw the status bars
    prevBtn = (digitalRead(BTN) == LOW);
    return;
  }

  runFrame(!skipNext);
  frameNo++; hudFpsFrames++;

  // frame pacing: wait if we are ahead of real time
  nextUs += frameUs();
  int32_t slack = (int32_t)(nextUs - micros());
  if (slack > 0) {
    if (slack > 2000) delay((slack - 1000) / 1000);
    while ((int32_t)(nextUs - micros()) > 0) { }
  } else {
    if (slack < -60000) nextUs = micros();               // way behind: just carry on from now
    if ((frameNo & 3) == 0) delay(1);                    // let the Bluetooth task and watchdog breathe
  }

  // frameskip: which pictures to drop
  if (fsMode == FS_AUTO) {                               // drop the next picture only when we are behind (at most 2 in a row)
    if (slack > 0) { skipNext = false; skipCount = 0; }
    else if (skipCount < 2) { skipNext = true; skipCount++; }
    else { skipNext = false; skipCount = 0; }
  } else if (fsMode == FS_OFF) {                         // never drop pictures
    skipNext = false; skipCount = 0;
  } else {                                               // fixed: draw one picture, then skip 1 / 2 / 3
    int n = fsMode - FS_OFF;
    if (skipCount < n) { skipNext = true; skipCount++; } else { skipNext = false; skipCount = 0; }
  }

  // status bars
  uint32_t nowMs = millis();
  if (nowMs - hudFpsMs >= 1000) {
    int fps = hudFpsFrames; hudFpsFrames = 0; hudFpsMs = nowMs;
    panel->fillRect(rowX0 + rowW, 0, SW - rowX0 - rowW, 12, 0);
    if (showFps) {
      char t[16]; snprintf(t, sizeof(t), "%d FPS", fps);
      panel->setTextSize(1); panel->setTextColor(C(160, 160, 160));
      int barW = SW - rowX0 - rowW;                      // centre the text in the bar
      panel->setCursor(rowX0 + rowW + (barW > 40 ? 4 : 0), 4); panel->print(t);
    }
  }
  int bs = btState;
  bool pairNow = pairRequest;
  if (bs != hudBt || pairNow != hudPair) {
    hudBt = bs; hudPair = pairNow;
    switch (bs) {
      case BT_CONNECTED:  drawLeftBar("PAD OK", "", C(90, 255, 120)); break;
      case BT_CONNECTING: drawLeftBar("CONNECT", "...", C(120, 220, 255)); break;
      case BT_SCAN:       if (pairNow) drawLeftBar("PAIRING", "pad mode", C(90, 160, 255)); else drawLeftBar("SEARCH", "for pad", C(150, 150, 150)); break;
      case BT_FAILED:     drawLeftBar("FAILED", "BOOT=retry", C(255, 90, 90)); break;
      default:            if (pairNow) drawLeftBar("PAIRING", "pad mode", C(90, 160, 255)); else drawLeftBar("NO PAD", "BOOT=pair", C(255, 190, 80)); break;
    }
  }

  // LED shows the Bluetooth state
  ledForBt(bs, nowMs);
}
