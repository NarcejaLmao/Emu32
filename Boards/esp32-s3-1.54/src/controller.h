// Extracted from the main sketch to keep this subsystem modular.
// ============================================================================
//  Bluetooth gamepad (BLE HID host)
//  The ESP32-S3 only has Bluetooth LE, so the controller must support BLE HID
//  (Xbox Wireless Controller, 8BitDo in BLE mode, ...). Classic-only pads such
//  as a DualShock 4 / Switch Pro in classic mode cannot work with this chip.
// ============================================================================
#define PAIR_PIN        0        // on-board BOOT button (GPIO0)
#define PAIR_SCAN_MS    30000    // give up scanning after this long
#define BOOT_DEBUG      0        // set to 1: print the raw BOOT pin level whenever it changes (serial 115200)
#define BLE_DEBUG       0        // set to 1 temporarily to diagnose BLE pairing/HID reports

enum PairState : uint8_t { PAIR_IDLE, PAIR_STARTING, PAIR_SCAN, PAIR_CONNECT };
static volatile PairState pairState = PAIR_IDLE;
static volatile bool      bleRT  = false;  // right trigger pulled (analog trigger thresholded)
static volatile uint16_t  bleBtn = 0;      // raw Xbox button bits from the last HID report
static volatile uint8_t   bleDir = 0;      // d-pad + left stick as UP/DOWN/LEFT/RIGHT bits (0xF0)
// Xbox hat encoding, learned from the first report after connecting: -1 unknown,
// 1 = Xbox style (0 = centred, 1..8 = N..NW), 0 = plain HID style (8/15 = centred, 0..7 = N..NW).
static volatile int8_t    bleHatMode = -1;
static volatile bool      bleConnected = false;
static bool               bleInited = false;
// Crash-loop breaker: set in setup() when the previous reset was a crash (panic / watchdog /
// brownout). While set, the saved pad is NOT auto-reconnected (startup or menus), so a fault in
// the BLE reconnect path cannot reboot the device forever. Pressing BOOT to pair clears it.
static bool               bleAutoDisabled = false;
// Boot-loop breaker for the BLE *stack itself*. A crash anywhere else must not stop the stack from
// starting at boot: if it is skipped, the first BOOT press happens after a game/menu has taken the
// internal RAM and pairing fails with "not enough internal RAM". So the stack is always started at
// boot, unless the previous boot died *inside* NimBLEDevice::init() (marker survives a soft reset).
#define BLE_GUARD_MAGIC 0xB1E0B1E0u
// Internal RAM the BLE stack needs to start. While it is not running, emulator cores must leave this
// much free (they spill into PSRAM instead), otherwise a later BOOT press can never start BLE.
#define BLE_INTERNAL_NEED (52u * 1024u)
static RTC_NOINIT_ATTR uint32_t bleInitGuard;
static bool bleBootInitAllowed() {
  if (esp_reset_reason() == ESP_RST_POWERON) bleInitGuard = 0;        // RTC RAM is garbage after power-on
  if (bleInitGuard == BLE_GUARD_MAGIC) { bleInitGuard = 0; return false; }
  return true;
}
static NimBLEAddress      bleTarget;
static NimBLEClient      *bleClient = nullptr;

// ---- BOOT button: interrupt-latched --------------------------------------------
// The menus/emulators block the main loop for long stretches (BLE connect attempts, SD loads), and
// a polled button is simply missed when a tap falls inside one of those. A falling-edge interrupt
// latches the press so readPad() sees it no matter when it gets around to looking.
static volatile bool     bootPressLatched = false;
static bool              bootIsrAttached  = false;
// No GPIO interrupt (an ISR on GPIO0 rebooted the S3 when BOOT was pressed): a tiny task polls the
// pin every 5 ms and latches a debounced press, so a tap is still caught while the loop is blocked.
static void bootButtonTask(void *) {
  bool prev = false; uint8_t low = 0;
  for (;;) {
    const bool lo = (digitalRead(PAIR_PIN) == LOW);
    if (lo) { if (low < 255) low++; } else low = 0;
    if (lo && !prev && low >= 3) { bootPressLatched = true; prev = true; }   // ~15 ms stable LOW
    if (!lo) prev = false;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
static void bootButtonInit() {
  if (bootIsrAttached) return;
  pinMode(PAIR_PIN, INPUT_PULLUP);
  xTaskCreate(bootButtonTask, "boot_btn", 2048, nullptr, 1, nullptr);
  bootIsrAttached = true;
}

// True only while the radio link is really up (the bleConnected flag alone can go stale
// if the pad drops while a connect attempt is still running).
static inline bool padLinked() { return bleConnected && bleClient && bleClient->isConnected(); }

// ---- on-board WS2812 RGB LED: pairing status ---------------------------------
//   blue, flashing : scanning / connecting to a controller
//   green, 1 flash : controller paired
//   red            : pairing failed (nothing found, connect or encryption error)
// Runs in its own small task so the blue flashing keeps going while the connect
// step blocks the main loop.
#define PIN_LED         -1       // This board has NO RGB LED. GPIO38 is the LCD SCLK: driving it as a
                                 // WS2812 (neopixelWrite) hijacks the display clock and blanks/garbles the screen.
#define LED_LEVEL       32       // brightness 0-255 (the WS2812 is very bright at full power)
#define LED_BLINK_MS    250      // blue flash half-period
#define LED_OK_MS       500      // green flash length
#define LED_FAIL_MS     1500     // red length

static volatile uint8_t  ledResult = 0;      // 0 none, 1 paired (green), 2 failed (red)
static volatile uint32_t ledUntil  = 0;      // millis() at which the result colour ends

static void ledSignal(bool ok) {
  ledUntil  = millis() + (ok ? LED_OK_MS : LED_FAIL_MS);
  ledResult = ok ? 1 : 2;
}

static void ledTask(void *) {
  uint32_t last = 0xFFFFFFFF;
  for (;;) {
    uint8_t r = 0, g = 0, b = 0;
    if (ledResult && (int32_t)(millis() - ledUntil) < 0) {
      if (ledResult == 1) g = LED_LEVEL; else r = LED_LEVEL;
    } else {
      ledResult = 0;
      if (pairState != PAIR_IDLE && ((millis() / LED_BLINK_MS) & 1)) b = LED_LEVEL;
    }
    const uint32_t key = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (key != last) { neopixelWrite(PIN_LED, r, g, b); last = key; }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---- screen flash: pairing feedback for boards without a status LED --------------
// The backlight is flashed from its own task, so it keeps flashing while the connect step blocks
// the main loop, and it is impossible to miss:
//   while scanning / connecting : backlight dips every half second
//   paired                      : 1 flash        failed / nothing found : 4 flashes
static volatile uint8_t flashReq = 0;            // one-shot flashes requested by the pairing code
static bool             flashTaskStarted = false;

static void backlightBlink(int n) { flashReq = (uint8_t)(n < 1 ? 1 : (n > 20 ? 20 : n)); }   // non-blocking

static void flashTask(void *) {
  for (;;) {
    if (flashReq) {
      const uint8_t n = flashReq; flashReq = 0;
      for (uint8_t i = 0; i < n; i++) {
        digitalWrite(PIN_LCD_BL, LOW);  vTaskDelay(pdMS_TO_TICKS(110));
        digitalWrite(PIN_LCD_BL, HIGH); vTaskDelay(pdMS_TO_TICKS(110));
      }
    } else if (pairState != PAIR_IDLE) {
      digitalWrite(PIN_LCD_BL, LOW);  vTaskDelay(pdMS_TO_TICKS(130));
      digitalWrite(PIN_LCD_BL, HIGH); vTaskDelay(pdMS_TO_TICKS(370));
    } else {
      vTaskDelay(pdMS_TO_TICKS(30));
    }
  }
}

static void ledInit() {
  bootButtonInit();                              // latch BOOT presses from the very start
  if (!flashTaskStarted) {
    flashTaskStarted = true;
    xTaskCreate(flashTask, "flash", 3072, nullptr, 1, nullptr);
  }
  if (PIN_LED < 0) return;                       // no LED on this board: never touch a GPIO
  neopixelWrite(PIN_LED, 0, 0, 0);
  xTaskCreate(ledTask, "led", 2048, nullptr, 1, nullptr);
}

// Decode one HID input report into raw controller state (bleBtn / bleDir).
// Supports the 15-byte Xbox BLE layout plus a common compact HID gamepad layout
// [buttons low, buttons high, hat, LX, LY, RX, RY, ...]. Report IDs are removed
// in bleNotify() when the Report Reference descriptor identifies a matching ID.
// HID has no universal gamepad byte layout; unknown layouts are logged and ignored
// rather than being misread as random button presses.
#define XB_A     0x0001
#define XB_B     0x0002
#define XB_X     0x0008
#define XB_RB    0x0080
#define XB_Y     0x0010
#define XB_VIEW  0x0400
#define XB_MENU  0x0800

static uint8_t decodeHidHat(uint8_t hat) {
  // HID hat values: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW;
  // 8 and 15 are commonly used for neutral.
  uint8_t s = 0;
  if (hat == 0 || hat == 1 || hat == 7) s |= 0x10;
  if (hat == 3 || hat == 4 || hat == 5) s |= 0x20;
  if (hat == 5 || hat == 6 || hat == 7) s |= 0x40;
  if (hat == 1 || hat == 2 || hat == 3) s |= 0x80;
  return s;
}

static uint8_t decodeXboxHat(uint8_t hat) {
  // The Xbox Wireless Controller declares its hat switch with logical range 1..8 and 0 = centred
  // (1=N, 2=NE, ... 8=NW). Decoding it as 0=N made the idle pad look like "UP held", which pinned
  // every menu to its first entry. Some other pads send the plain HID encoding (0..7, 8/15 = centred),
  // so the encoding is detected from the first report after connecting (a pad that is not being
  // touched reports its centred value: 0 -> Xbox style, 8/15 -> plain HID).
  hat &= 0x0F;
  if (bleHatMode < 0) {
    if (hat == 0) bleHatMode = 1;
    else if (hat == 8 || hat == 15) bleHatMode = 0;
  }
  if (bleHatMode == 0) return decodeHidHat(hat);
  if (hat == 0 || hat > 8) return 0;                // centred / invalid
  return decodeHidHat(hat - 1);                     // 1..8 -> 0..7
}

static void decodeReport(const uint8_t *d, size_t n) {
  if (!d) return;
  uint8_t s = 0;
  uint16_t buttons = 0;
  bool rt = false;

  if (n == 15) {
    // Xbox Wireless Controller BLE input report. Its documented layout is 15 bytes;
    // do not treat longer, extended HID reports as Xbox packets based on length alone.
    const uint16_t lx = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    const uint16_t ly = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    s |= decodeXboxHat(d[12]);
    if (lx < 16000) s |= 0x40; else if (lx > 49000) s |= 0x80;
    if (ly < 16000) s |= 0x10; else if (ly > 49000) s |= 0x20;
    buttons = (uint16_t)d[13] | ((uint16_t)d[14] << 8);
    rt = (((uint16_t)d[10] | ((uint16_t)d[11] << 8)) > 300);   // RT is a 10-bit analog axis (0..1023)
  } else if (n >= 7 && n < 15) {
    // Compact/common HID gamepad report. Longer unknown reports are ignored below. Axes are unsigned 8-bit values centered
    // near 128; buttons are a little-endian bitfield and hat uses HID's 0..7/8 values.
    // Mapping: buttons 1..4 = A/B/X/Y, 6 = RB, 9 = View/Select, 10 = Menu/Start.
    const uint16_t rawButtons = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    if (rawButtons & (1u << 0)) buttons |= XB_A;
    if (rawButtons & (1u << 1)) buttons |= XB_B;
    if (rawButtons & (1u << 2)) buttons |= XB_X;
    if (rawButtons & (1u << 3)) buttons |= XB_Y;
    if (rawButtons & (1u << 5)) buttons |= XB_RB;
    rt = (rawButtons & (1u << 7)) != 0;                // button 8 = RT on common HID pads
    if (rawButtons & (1u << 8)) buttons |= XB_VIEW;
    if (rawButtons & (1u << 9)) buttons |= XB_MENU;
    s |= decodeHidHat(d[2] & 0x0F);
    if (d[3] < 64) s |= 0x40; else if (d[3] > 192) s |= 0x80;
    if (d[4] < 64) s |= 0x10; else if (d[4] > 192) s |= 0x20;
    // Ignore RX/RY (right stick). The frontend has no right-stick movement
    // mapping, and treating those axes as D-pad input caused stray movement
    // whenever a player adjusted the right stick.
  } else {
#if BLE_DEBUG
    Serial.printf("BLE: ignoring unsupported short HID report (%u bytes)\n", (unsigned)n);
#endif
    return; // may be battery/status data; don't overwrite the last gamepad state
  }

  bleDir = s;
  bleBtn = buttons;
  bleRT  = rt;
}

// Turn the raw pad state into button bits (A B SEL START UP DOWN LEFT RIGHT).
//  Menus : the controller's own layout - its A button is "A" (OK), its B is "B" (back).
//  Games : the emulated system's layout. The NES / Game Boy A button sits on the
//          right and B on the left, so Xbox B/Y -> A and Xbox A/X -> B. The Atari 2600
//          has a single fire button, so every face button fires.
// Genesis C button (the 8-bit padState has no spare bit); read by genesis_core.h.
static bool padBtnC = false;

static uint8_t padLayout(uint16_t b, uint8_t dir, bool game, uint8_t sys) {
  uint8_t s = dir;
  padBtnC = false;
  if (b & XB_VIEW) s |= 0x04;                       // View -> SELECT
  if (b & XB_MENU) s |= 0x08;                       // Menu -> START
  if (!game) {
    if (b & XB_A) s |= 0x01;                        // native A -> A
    if (b & XB_B) s |= 0x02;                        // native B -> B
  } else if (sys == SYS_GENESIS) {                  // Sega A/B/C on the Xbox X / A / B (Y also = C)
    if (b & XB_X) s |= 0x01;                        // -> A
    if (b & XB_A) s |= 0x02;                        // -> B
    if (b & (XB_B | XB_Y)) padBtnC = true;          // -> C
  } else if (sys == SYS_A2600) {
    if (b & (XB_A | XB_B | XB_X | XB_Y)) s |= 0x01; // fire
  } else {                                          // NES, Game Boy
    if (b & (XB_B | XB_Y)) s |= 0x01;               // -> A
    if (b & (XB_A | XB_X)) s |= 0x02;               // -> B
  }
  return s;
}

struct HidReportRef { NimBLERemoteCharacteristic *chr; uint8_t id; };
static HidReportRef hidReportRefs[16];
static uint8_t hidReportRefCount = 0;

static void bleNotify(NimBLERemoteCharacteristic *chr, uint8_t *d, size_t n, bool) {
  if (!d || !n) return;
  // Report Reference descriptors are read during subscription, not inside the
  // asynchronous notification callback (which must not perform blocking GATT reads).
  for (uint8_t i = 0; i < hidReportRefCount; ++i) {
    if (hidReportRefs[i].chr == chr) {
      const uint8_t reportId = hidReportRefs[i].id;
      if (reportId != 0 && n > 1 && d[0] == reportId) { ++d; --n; }
      break;
    }
  }
#if BLE_DEBUG
  static uint32_t last = 0;
  if (millis() - last > 250) {
    last = millis();
    Serial.printf("HID[%u]:", (unsigned)n);
    for (size_t i = 0; i < n && i < 20; i++) Serial.printf(" %02X", d[i]);
    Serial.println();
  }
#endif
  decodeReport(d, n);
}

// Is this advertiser a gamepad? HID service, gamepad/joystick appearance, or a
// telltale name (some pads leave the service UUID out of their advertisement).
static bool looksLikePad(const NimBLEAdvertisedDevice *dev) {
  if (dev->haveAppearance()) {
    uint16_t ap = dev->getAppearance();
    if (ap == 0x03C1 || ap == 0x03C2) return false;                                   // keyboard / mouse
    if (ap == 0x03C3 || ap == 0x03C4 || ap == 0x03C5 || ap == 0x03C0) return true;  // joystick/gamepad/HID
  }
  if (dev->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) return true;
  std::string n = dev->getName();
  static const char *names[] = { "controller", "gamepad", "xbox", "8bitdo", "joy-con", "pro con", "wireless pad" };
  for (const char *k : names) if (strcasestr(n.c_str(), k)) return true;
  return false;
}

static uint32_t bleDeadline = 0;                 // end of the pairing window (millis)
static const char *bleFail = "";               // short reason of the last failed pairing (shown on screen)
static inline int bleStatus() { return (pairState != PAIR_IDLE) ? 1 : (padLinked() ? 2 : (bleFail[0] ? 3 : 0)); }

class PairScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    if (pairState != PAIR_SCAN) return;
    bool pad = looksLikePad(dev);
#if BLE_DEBUG
    Serial.printf("BLE: saw %s  \"%s\"  rssi %d%s\n", dev->getAddress().toString().c_str(),
                  dev->getName().c_str(), dev->getRSSI(), pad ? "  <- gamepad" : "");
#endif
    if (!pad) return;
    if (!dev->isConnectable()) return;               // e.g. a beacon / a pad not in pairing mode
    Serial.printf("BLE: gamepad found: %s \"%s\" - connecting\n", dev->getAddress().toString().c_str(), dev->getName().c_str());
    bleTarget = dev->getAddress();
    pairState = PAIR_CONNECT;                        // loop() does the connect
    NimBLEDevice::getScan()->stop();
  }
  void onScanEnd(const NimBLEScanResults &, int) override {
    // A late end-of-scan event from a scan we stopped ourselves must not abort a new one.
    if (pairState == PAIR_SCAN && (int32_t)(millis() - bleDeadline) >= -1500) {
      pairState = PAIR_IDLE;
      bleFail = "NO PAD FOUND";
      Serial.println("BLE: no controller found - is it in pairing mode?");
      ledSignal(false);
      backlightBlink(4);
    }
  }
};
class PadClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *, int reason) override {
    // Clear both raw HID state and the combined state consumed by the emulator.
    // Otherwise the last held button can remain visible until the next menu tick.
    bleConnected = false;
    bleBtn = 0; bleRT = false;
    bleDir = 0; bleHatMode = -1;
    padState = 0;
    Serial.printf("BLE: controller disconnected (reason %d)\n", reason);
  }
};
static PairScanCB  pairScanCB;
static PadClientCB padClientCB;

// One-time BLE stack setup (shared by pairing and auto-reconnect).
// NimBLEDevice::init() waits for the controller/host to sync and can block forever if the radio stack
// never comes up. It therefore runs in its own task and bleInit() gives up after BLE_INIT_TIMEOUT_MS,
// so the rest of the firmware keeps booting (BLE simply stays off).
#define BLE_INIT_TIMEOUT_MS 6000
static volatile uint8_t bleInitState = 0;     // 0 idle, 1 running, 2 ok, 3 failed
static bool             bleInitGaveUp = false;

static void bleInitTask(void *) {
  bleInitState = NimBLEDevice::init("Emu32") ? 2 : 3;
  vTaskDelete(nullptr);
}

static void bleConfigureSecurity() {
  NimBLEDevice::setSecurityAuth(true, false, true);      // bonding, no MITM, secure conn. (Just Works)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  // HID pads insist on getting the keys of both sides (encryption + identity) when bonding
  NimBLEDevice::setSecurityInitKey(0x01 | 0x02);
  NimBLEDevice::setSecurityRespKey(0x01 | 0x02);
}

static bool bleInit() {
  if (bleInited) return true;
  if (bleInitState == 2) {                               // a timed-out init finished late: accept it
    bleConfigureSecurity(); bleInited = true; bleInitGaveUp = false; return true;
  }
  if (bleInitGaveUp || bleInitState == 1) return false;  // never start a second init on a stuck stack
  const uint32_t fr = heap_caps_get_free_size(MALLOC_CAP_INTERNAL), lb = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  Serial.printf("BLE: init start (internal free %u, largest block %u)\n", (unsigned)fr, (unsigned)lb);
  if (lb < 30000) { Serial.printf("BLE: largest internal block %u too small - not starting BLE (free %u)\n", (unsigned)lb, (unsigned)fr); return false; }
  if (fr < 45000) { Serial.println("BLE: not enough internal RAM to start BLE - reboot, BLE must start at boot"); return false; }   // would panic -> reboot

  bleInitState = 1;
  bleInitGuard = BLE_GUARD_MAGIC;                        // cleared again on every path that returns normally
  if (xTaskCreate(bleInitTask, "ble_init", 8192, nullptr, 1, nullptr) != pdPASS) {
    bleInitState = 0;
    bleInitGuard = 0;
    Serial.println("BLE: could not create init task");
    return false;
  }
  const uint32_t t0 = millis();
  while (bleInitState == 1 && millis() - t0 < BLE_INIT_TIMEOUT_MS) delay(10);
  bleInitGuard = 0;
  if (bleInitState == 1) {
    bleInitGaveUp = true;
    Serial.printf("BLE: init TIMED OUT after %u ms - BLE disabled for this boot\n", (unsigned)BLE_INIT_TIMEOUT_MS);
    return false;
  }
  if (bleInitState != 2) {
    bleInitState = 0;
    Serial.printf("BLE: init failed (free heap %u)\n", (unsigned)ESP.getFreeHeap());
    return false;
  }
  bleConfigureSecurity();
  bleInited = true;
  Serial.printf("BLE: init done (internal free %u)\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  return true;
}

// Last connected controller, kept in flash (the bond keys are kept by NimBLE).
static bool bleLoadSaved() {
  Preferences p;
  if (!p.begin("emu32", true)) return false;             // namespace not created yet
  String a = p.getString("pad", "");
  uint8_t t = p.getUChar("padtype", 0);
  p.end();
  if (a.length() < 17) return false;
  bleTarget = NimBLEAddress(std::string(a.c_str()), t);
  return true;
}
static bool bleHaveSaved() {
  Preferences p;
  if (!p.begin("emu32", true)) return false;
  bool h = p.getString("pad", "").length() >= 17;
  p.end();
  return h;
}
static void bleSaveLast() {
  Preferences p;
  if (!p.begin("emu32", false)) return;
  p.putString("pad", bleTarget.toString().c_str());
  p.putUChar("padtype", bleTarget.getType());
  p.end();
}
static void bleForgetLast() {
  Preferences p;
  if (!p.begin("emu32", false)) return;
  p.remove("pad"); p.remove("padtype");
  p.end();
}

// Pairing setup runs outside the Arduino loop. Some NimBLE operations (especially
// disconnect/scan startup) can wait on the host task; doing them in readPad() made
// the whole UI appear frozen immediately after "pairing requested".
static void blePairRun() {
  Serial.println("BLE: pairing worker started"); Serial.flush();
  bleAutoDisabled = false;
  bleFail = "";

  Serial.println("BLE: checking stack"); Serial.flush();
  if (!bleInit()) {
    bleFail = "BLE INIT";
    Serial.println("BLE: initialization unavailable");
    pairState = PAIR_IDLE;
    ledSignal(false); backlightBlink(4);
    return;
  }
  Serial.println("BLE: stack ready"); Serial.flush();

  // Disconnect an existing connection before scanning, but keep its bond keys.
  // Deleting every bond on each pairing attempt can break other saved peripherals
  // and is not required to discover a new HID controller.
  if (bleClient) {
    Serial.println("BLE: checking previous client"); Serial.flush();
    if (bleClient->isConnected()) {
      Serial.println("BLE: disconnecting previous controller"); Serial.flush();
      bleClient->disconnect();
      Serial.println("BLE: previous controller disconnected"); Serial.flush();
    }
  }
  bleConnected = false; bleBtn = 0; bleRT = false; bleDir = 0; bleHatMode = -1;
  bleForgetLast();

  Serial.printf("BLE: preparing scan (heap %u, internal %u)\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.flush();
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (!scan) {
    bleFail = "NO SCAN";
    Serial.println("BLE: scan object unavailable");
    pairState = PAIR_IDLE;
    ledSignal(false); backlightBlink(4);
    return;
  }
  if (scan->isScanning()) {
    Serial.println("BLE: stopping previous scan"); Serial.flush();
    scan->stop();
  }
  scan->setScanCallbacks(&pairScanCB, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  bleDeadline = millis() + PAIR_SCAN_MS;
  pairState = PAIR_SCAN;
  Serial.println("BLE: starting scan now - put controller in pairing mode"); Serial.flush();
  if (!scan->start(PAIR_SCAN_MS, false)) {
    pairState = PAIR_IDLE;
    bleFail = "SCAN START";
    Serial.println("BLE: scan failed to start");
    ledSignal(false); backlightBlink(4);
  } else {
    Serial.println("BLE: scan started successfully"); Serial.flush();
  }
}

// The worker is a permanent task whose stack and TCB live in .bss. A running emulator core can
// leave only ~1 KB of internal heap, so allocating a task stack at BOOT-press time fails. This
// task is created once at boot (before any core grabs RAM) and just sleeps until notified.
static TaskHandle_t   blePairTask = nullptr;
static StaticTask_t   blePairTcb;
static StackType_t    blePairStack[4096] __attribute__((aligned(16)));
static void blePairWorker(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    blePairRun();
  }
}
static void blePairTaskInit() {
  if (blePairTask) return;
  blePairTask = xTaskCreateStatic(blePairWorker, "ble_pair", sizeof(blePairStack), nullptr, 2,
                                  blePairStack, &blePairTcb);
  Serial.println(blePairTask ? "BLE: pairing worker reserved" : "BLE: pairing worker reservation FAILED");
}

static void blePairStart() {
  if (pairState != PAIR_IDLE) return;
  Serial.println("BLE: pairing requested"); Serial.flush();
  backlightBlink(1);
  pairState = PAIR_STARTING; // block auto-reconnect before the worker is scheduled
  if (!blePairTask) blePairTaskInit();                   // normally done in setup()
  if (!blePairTask) {
    pairState = PAIR_IDLE;
    bleFail = "PAIR TASK";
    Serial.println("BLE: pairing worker not available");
    ledSignal(false); backlightBlink(4);
    return;
  }
  Serial.printf("BLE: waking pairing worker (free internal %u, largest block %u)\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  xTaskNotifyGive(blePairTask);
}
// Subscribe to the pad's HID input reports. onlyInput = skip report characteristics that
// are declared as output/feature in their Report Reference descriptor.
static int bleSubscribeReports(bool onlyInput) {
  int subs = 0;
  hidReportRefCount = 0;
  NimBLERemoteService *svc = bleClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!svc) { Serial.println("BLE: device has no HID service"); return 0; }
  for (auto *c : svc->getCharacteristics(true)) {
    if (c->getUUID() != NimBLEUUID((uint16_t)0x2A4D) || !c->canNotify()) continue;
    if (onlyInput) {
      // Report Reference descriptor: [report id, type]; only take INPUT reports (type 1)
      NimBLERemoteDescriptor *rr = c->getDescriptor(NimBLEUUID((uint16_t)0x2908));
      if (rr) {
        NimBLEAttValue v = rr->readValue();
        if (v.size() >= 2 && v.data()[1] != 1) continue;
        if (v.size() >= 2 && hidReportRefCount < (sizeof(hidReportRefs) / sizeof(hidReportRefs[0]))) {
          hidReportRefs[hidReportRefCount++] = { c, v.data()[0] };
        }
      }
    }
    if (c->subscribe(true, bleNotify)) subs++;
  }
  return subs;
}

static inline bool linkEncrypted() { return bleClient->isConnected() && bleClient->getConnInfo().isEncrypted(); }

// Connect + encrypt + subscribe. pairing = true when this is a brand new pairing (a failed
// attempt then throws away the half-made bond so the next try starts clean).
static bool bleConnectPad(int attempts = 3, uint32_t timeoutMs = 8000, bool pairing = false) {
  bleConnected = false; bleBtn = 0; bleRT = false; bleDir = 0; bleHatMode = -1;
  if (!bleClient) {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&padClientCB, false);
    bleClient->setConnectionParams(12, 40, 0, 400);      // 15-50 ms interval, 4 s supervision timeout
  }
  bleClient->setConnectTimeout(timeoutMs);
  for (int attempt = 1; attempt <= attempts; attempt++) {
    if (bleClient->isConnected()) bleClient->disconnect();
    delay(pairing ? 300 : 100);                          // let the radio / pad settle (scan just stopped)
    Serial.printf("BLE: connecting to %s (try %d)\n", bleTarget.toString().c_str(), attempt);

    bleFail = "CONNECT";
    // no MTU exchange: HID reports are tiny and some pads stall on it
    if (!bleClient->connect(bleTarget, true, false, false)) { Serial.println("BLE: connect failed"); delay(400); continue; }

    // Encryption. Many pads start the pairing themselves right after connecting, and calling
    // secureConnection() on top of that "fails" although the pairing works - so give the pad a
    // moment, ask only if nothing happened, ignore the return value and look at the real state.
    bleFail = "ENCRYPT";
    uint32_t t0 = millis();
    while (millis() - t0 < 1200 && bleClient->isConnected() && !linkEncrypted()) delay(50);
    if (bleClient->isConnected() && !linkEncrypted()) bleClient->secureConnection();
    t0 = millis();
    while (millis() - t0 < 6000 && bleClient->isConnected() && !linkEncrypted()) delay(50);
    if (!bleClient->isConnected()) {
      Serial.println("BLE: link dropped during pairing");
      if (pairing) NimBLEDevice::deleteBond(bleTarget);
      delay(400); continue;
    }
    Serial.println(linkEncrypted() ? "BLE: link encrypted" : "BLE: link NOT encrypted - trying anyway");

    bleFail = "NO HID";
    int subs = bleSubscribeReports(true);
    if (!subs && bleClient->isConnected()) subs = bleSubscribeReports(false);
    if (!subs) {
      Serial.println("BLE: no HID input report to subscribe to");
      if (pairing) NimBLEDevice::deleteBond(bleTarget);
      bleClient->disconnect(); delay(400); continue;
    }
    // Pads that refuse the bond drop the link right after subscribing: make sure it holds.
    delay(600);
    if (!bleClient->isConnected()) {
      Serial.println("BLE: link dropped right after connecting");
      if (pairing) NimBLEDevice::deleteBond(bleTarget);
      delay(400); continue;
    }
    Serial.printf("BLE: subscribed to %d report(s), link is up\n", subs);
    bleFail = "";
    bleConnected = true;
    return true;
  }
  bleConnected = false;
  return false;
}

// Called once per frame from loop(); the connect step briefly blocks emulation.
// If a connect attempt fails while the pairing window (PAIR_SCAN_MS) is still open, scan again
// (the pad may have restarted advertising under a new address) instead of giving up: the LED
// stays blue and only turns red when the whole window has run out.
// Background auto-reconnect of the last paired controller. Runs from the menus only (a connect
// attempt blocks for ~2 s, which would stutter a running game), so: switch the pad on whenever you
// like and it links up within a few seconds; a pad that dropped mid-game reconnects on the next menu.
// With no pad connected nobody can navigate the menus anyway, so the short stalls are harmless.
#define BLE_RETRY_MS     5000    // pause between background attempts
#define BLE_RETRY_TMO_MS 1500    // connect timeout of one background attempt
static uint32_t bleRetryAt = 0;
static void bleBackgroundReconnect() {
  if (inGame || pairState != PAIR_IDLE || padLinked()) { bleRetryAt = millis() + 1500; return; }
  if (bleAutoDisabled) return;                             // previous boot crashed: only manual BOOT pairing
  if ((int32_t)(millis() - bleRetryAt) < 0) return;
  bleRetryAt = millis() + BLE_RETRY_MS;
  if (digitalRead(PAIR_PIN) == LOW || bootPressLatched) return;  // BOOT pressed: the user is about to pair a new pad
  if (!bleLoadSaved() || !bleInit()) return;               // nothing saved yet: pair once with BOOT
  Serial.println("BLE: looking for the saved controller...");
  const bool ok = bleConnectPad(1, BLE_RETRY_TMO_MS, false) && padLinked();
  bleFail = "";                                            // pad simply being off is not an error to show
  if (ok) { Serial.println("BLE: saved controller connected"); backlightBlink(1); }
}

static void bleTick() {
  if (pairState != PAIR_CONNECT) { bleBackgroundReconnect(); return; }
  uint32_t t0 = millis();
  bool ok = bleConnectPad(2, 6000, true) && padLinked();
  Serial.printf("BLE: connect step took %u ms\n", (unsigned)(millis() - t0));
  if (ok) {
    // After bonding, remember the pad's real (identity) address: the one it advertised while
    // pairing can be a random one that changes, which would break the auto-reconnect.
    NimBLEAddress id = bleClient->getConnInfo().getIdAddress();
    if (!id.isNull()) bleTarget = id;
    bleSaveLast();
    ledSignal(true);
    pairState = PAIR_IDLE;
    Serial.println("BLE: controller connected");
    backlightBlink(1);
    return;
  }
  int32_t left = (int32_t)(bleDeadline - millis());
  if (left > 4000) {
    Serial.printf("BLE: attempt failed (%s) - scanning again (%d s left)\n", bleFail, (int)(left / 1000));
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
    delay(200);
    pairState = PAIR_SCAN;
    if (scan->start((uint32_t)left, false)) return;
    Serial.println("BLE: rescan failed to start");
  }
  ledSignal(false);
  pairState = PAIR_IDLE;
  Serial.println("BLE: pairing failed, press BOOT to retry");
  backlightBlink(4);
}

// Startup: reconnect the last controller. It must be switched on (not in pairing
// mode). Blocks for a few seconds; returns true when it is connected.
static bool bleAutoReconnect() {
  if (bleAutoDisabled) { Serial.println("BLE: auto-reconnect skipped (previous boot crashed)"); return false; }
  if (!bleLoadSaved() || !bleInit()) return false;
  Serial.printf("BLE: reconnecting to saved controller %s\n", bleTarget.toString().c_str());
  bool ok = bleConnectPad(2, 3000, false) && padLinked();
  bleFail = "";                                          // not an error worth showing in the menu
  Serial.println(ok ? "BLE: saved controller connected" : "BLE: saved controller not found (BOOT = pair a new one)");
  return ok;
}

// Clear stale input when changing between game and menu contexts. This intentionally
// clears only the cached Bluetooth state and combined snapshot; physical GPIO buttons
// are sampled again by the next readPad() call.
static inline void controllerResetInput() {
  bleBtn = 0; bleRT = false;
  bleDir = 0;
  padState = 0;
}

static void readPad() {
  // Debounce the physical buttons without blocking the emulation/menu loop.
  // Mechanical switches can briefly toggle several times during one press;
  // accept a new state only after it has remained stable for 12 ms.
  static bool physicalStable[8] = {};
  static bool physicalRawPrev[8] = {};
  static uint32_t physicalChangedAt[8] = {};
  const uint32_t inputNowMs = millis();

  uint8_t s = 0;
  for (int i = 0; i < 8; i++) {
    const bool rawPressed = padPins[i] >= 0 && digitalRead(padPins[i]) == LOW;
    if (rawPressed != physicalRawPrev[i]) {
      physicalRawPrev[i] = rawPressed;
      physicalChangedAt[i] = inputNowMs;
    } else if (physicalStable[i] != rawPressed &&
               (uint32_t)(inputNowMs - physicalChangedAt[i]) >= 12) {
      physicalStable[i] = rawPressed;
    }
    if (physicalStable[i]) s |= (1 << i);
  }
  // Ignore cached HID state as soon as the BLE link drops. The disconnect
  // callback also clears it, but this guards the frame before that callback runs.
  const bool bleActive = padLinked();
  const uint16_t activeBleButtons = bleActive ? bleBtn : 0;
  const uint8_t activeBleDirections = bleActive ? bleDir : 0;
  s |= padLayout(activeBleButtons, activeBleDirections, inGame, curSys); // Bluetooth controller
  // no opposite d-pad directions at once (some games glitch)
  if ((s & 0x30) == 0x30) s &= ~0x30;
  if ((s & 0xC0) == 0xC0) s &= ~0xC0;
  padState = s;

  // BOOT button: only job is to start pairing a Bluetooth controller.
  // Require a stable level before accepting the edge so contact bounce cannot
  // start pairing more than once for a single physical press.
  static bool bootStable = false;
  static bool bootRawPrev = false;
  static uint32_t bootRawChangedAt = 0;
  bootButtonInit();                                        // one-time: attach the press-latch interrupt
  const bool bootRaw = (digitalRead(PAIR_PIN) == LOW);
#if BOOT_DEBUG
  { static int dbgLast = -1; if ((int)bootRaw != dbgLast) { dbgLast = bootRaw; Serial.printf("BOOT pin (GPIO%d): %s\n", PAIR_PIN, bootRaw ? "LOW (pressed)" : "HIGH"); } }
#endif
  const uint32_t bootNowMs = millis();
  if (bootPressLatched) {                                  // a press arrived (possibly while we were blocked)
    bootPressLatched = false;
    Serial.println("BOOT: button pressed - starting pairing");
    blePairStart();
    bootStable = bootRaw;                                  // still held -> the polled path must not fire again
    bootRawPrev = bootRaw;
    bootRawChangedAt = bootNowMs;
    return;
  }
  if (bootRaw != bootRawPrev) {
    bootRawPrev = bootRaw;
    bootRawChangedAt = bootNowMs;
  } else if (bootStable != bootRaw && (uint32_t)(bootNowMs - bootRawChangedAt) >= 35) {
    bootStable = bootRaw;
    if (bootStable) blePairStart();
  }
}

// SELECT+RB: rotate the whole display 90 degrees clockwise (4 orientations). Call once per
// frame / menu tick after readPad() + bleTick(). Returns true on the tick it rotated; the
// screen has been blanked, so the caller must redraw.
static bool rotateHotkey() {
  static bool was = false;
  const bool now = padLinked() && (padState & 0x04) && (bleBtn & XB_RB);
  const bool fire = now && !was;
  was = now;
  if (fire) lcdSetRotation(lcdRot + 1);
  return fire;
}

