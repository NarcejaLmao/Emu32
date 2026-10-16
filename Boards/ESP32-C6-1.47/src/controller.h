// Extracted from the main sketch to keep this subsystem modular.
// ============================================================================
//  Bluetooth gamepad (BLE HID host)
//  The ESP32-C6 only has Bluetooth LE, so the controller must support BLE HID
//  (Xbox Wireless Controller, 8BitDo in BLE mode, ...). Classic-only pads such
//  as a DualShock 4 / Switch Pro in classic mode cannot work with this chip.
// ============================================================================
#define PAIR_PIN        9        // on-board BOOT button
#define PAIR_SCAN_MS    30000    // give up scanning after this long
#define BLE_DEBUG       1        // print every scanned device + raw HID reports

enum PairState : uint8_t { PAIR_IDLE, PAIR_SCAN, PAIR_CONNECT };
static volatile PairState pairState = PAIR_IDLE;
static volatile uint16_t  bleBtn = 0;      // raw Xbox button bits from the last HID report
static volatile uint8_t   bleDir = 0;      // d-pad + left stick as UP/DOWN/LEFT/RIGHT bits (0xF0)
static volatile bool      bleConnected = false;
static bool               bleInited = false;
static NimBLEAddress      bleTarget;
static NimBLEClient      *bleClient = nullptr;

// True only while the radio link is really up (the bleConnected flag alone can go stale
// if the pad drops while a connect attempt is still running).
static inline bool padLinked() { return bleConnected && bleClient && bleClient->isConnected(); }

// ---- on-board WS2812 RGB LED: pairing status ---------------------------------
//   blue, flashing : scanning / connecting to a controller
//   green, 1 flash : controller paired
//   red            : pairing failed (nothing found, connect or encryption error)
// Runs in its own small task so the blue flashing keeps going while the connect
// step blocks the main loop.
#define PIN_LED         8        // on-board WS2812 (GPIO 8)
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

static void ledInit() {
  neopixelWrite(PIN_LED, 0, 0, 0);
  xTaskCreate(ledTask, "led", 2048, nullptr, 1, nullptr);
}

static void backlightBlink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LCD_BL, LOW);  delay(70);
    digitalWrite(PIN_LCD_BL, HIGH); delay(70);
  }
}

// Decode one HID input report into raw controller state (bleBtn / bleDir).
// Layout below is the Xbox Wireless Controller (BLE) report:
//   0-1 LX  2-3 LY  4-5 RX  6-7 RY  8-9 LT  10-11 RT  12 hat  13-14 buttons
// For other pads, watch the BLE_DEBUG output and adjust this function.
// Xbox button bits (as used by padLayout below):
#define XB_A     0x0001
#define XB_B     0x0002
#define XB_X     0x0008
#define XB_RB    0x0080
#define XB_Y     0x0010
#define XB_VIEW  0x0400
#define XB_MENU  0x0800

static void decodeReport(const uint8_t *d, size_t n) {
  if (n < 15) return;                               // battery / Guide / other short report: ignore, don't clear buttons
  uint8_t s = 0;
  uint16_t lx = d[0] | (d[1] << 8), ly = d[2] | (d[3] << 8);
  uint8_t  hat = d[12];

  if (hat == 1 || hat == 2 || hat == 8) s |= 0x10;  // up
  if (hat >= 4 && hat <= 6)             s |= 0x20;  // down
  if (hat >= 6 && hat <= 8)             s |= 0x40;  // left
  if (hat >= 2 && hat <= 4)             s |= 0x80;  // right

  if (lx < 16000) s |= 0x40; else if (lx > 49000) s |= 0x80;   // left stick
  if (ly < 16000) s |= 0x10; else if (ly > 49000) s |= 0x20;

  bleDir = s;
  bleBtn = d[13] | (d[14] << 8);
}

// Turn the raw pad state into button bits (A B SEL START UP DOWN LEFT RIGHT).
//  Menus : the controller's own layout - its A button is "A" (OK), its B is "B" (back).
//  Games : the emulated system's layout. The NES / Game Boy A button sits on the
//          right and B on the left, so Xbox B/Y -> A and Xbox A/X -> B. The Atari 2600
//          has a single fire button, so every face button fires.
static uint8_t padLayout(uint16_t b, uint8_t dir, bool game, uint8_t sys) {
  uint8_t s = dir;
  if (b & XB_VIEW) s |= 0x04;                       // View -> SELECT
  if (b & XB_MENU) s |= 0x08;                       // Menu -> START
  if (!game) {
    if (b & XB_A) s |= 0x01;                        // native A -> A
    if (b & XB_B) s |= 0x02;                        // native B -> B
  } else if (sys == SYS_A2600) {
    if (b & (XB_A | XB_B | XB_X | XB_Y)) s |= 0x01; // fire
  } else {                                          // NES, Game Boy
    if (b & (XB_B | XB_Y)) s |= 0x01;               // -> A
    if (b & (XB_A | XB_X)) s |= 0x02;               // -> B
  }
  return s;
}

static void bleNotify(NimBLERemoteCharacteristic *, uint8_t *d, size_t n, bool) {
#if BLE_DEBUG
  static uint32_t last = 0;
  if (millis() - last > 250) {                       // rate-limited dump
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
    bleBtn = 0;
    bleDir = 0;
    padState = 0;
    Serial.printf("BLE: controller disconnected (reason %d)\n", reason);
  }
};
static PairScanCB  pairScanCB;
static PadClientCB padClientCB;

// One-time BLE stack setup (shared by pairing and auto-reconnect).
static bool bleInit() {
  if (bleInited) return true;
  if (!NimBLEDevice::init("Emu32")) {
    Serial.printf("BLE: init failed (free heap %u)\n", (unsigned)ESP.getFreeHeap());
    return false;
  }
  NimBLEDevice::setSecurityAuth(true, false, true);      // bonding, no MITM, secure conn. (Just Works)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  // HID pads insist on getting the keys of both sides (encryption + identity) when bonding
  NimBLEDevice::setSecurityInitKey(0x01 | 0x02);
  NimBLEDevice::setSecurityRespKey(0x01 | 0x02);
  bleInited = true;
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

static void blePairStart() {
  if (pairState != PAIR_IDLE) return;
  bleFail = "";
  if (!bleInit()) { bleFail = "BLE INIT"; ledSignal(false); backlightBlink(4); return; }
  if (bleClient && bleClient->isConnected()) bleClient->disconnect();
  bleConnected = false; bleBtn = 0; bleDir = 0;
  NimBLEDevice::deleteAllBonds();                          // stale keys make re-pairing fail
  bleForgetLast();                                         // old controller no longer bonded
  Serial.printf("BLE: scanning - put the controller in pairing mode (heap %u)\n",
                (unsigned)ESP.getFreeHeap());
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (scan->isScanning()) scan->stop();
  scan->setScanCallbacks(&pairScanCB, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  bleDeadline = millis() + PAIR_SCAN_MS;
  pairState = PAIR_SCAN;
  if (!scan->start(PAIR_SCAN_MS, false)) {
    pairState = PAIR_IDLE;
    bleFail = "SCAN START";
    Serial.println("BLE: scan failed to start");
    ledSignal(false);
    backlightBlink(4);
    return;
  }
  backlightBlink(2);
}

// Subscribe to the pad's HID input reports. onlyInput = skip report characteristics that
// are declared as output/feature in their Report Reference descriptor.
static int bleSubscribeReports(bool onlyInput) {
  int subs = 0;
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
  bleConnected = false; bleBtn = 0; bleDir = 0;
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
static void bleTick() {
  if (pairState != PAIR_CONNECT) return;
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
  bleBtn = 0;
  bleDir = 0;
  padState = 0;
}

static void readPad() {
  uint8_t s = 0;
  for (int i = 0; i < 8; i++)
    if (padPins[i] >= 0 && digitalRead(padPins[i]) == LOW) s |= (1 << i);
  s |= padLayout(bleBtn, bleDir, inGame, curSys);   // Bluetooth controller
  // no opposite d-pad directions at once (some games glitch)
  if ((s & 0x30) == 0x30) s &= ~0x30;
  if ((s & 0xC0) == 0xC0) s &= ~0xC0;
  padState = s;

  // BOOT button: only job is to start pairing a Bluetooth controller
  static bool bootWas = false;
  bool bootNow = (digitalRead(PAIR_PIN) == LOW);
  if (bootNow && !bootWas) blePairStart();
  bootWas = bootNow;
}

// SELECT+RB: rotate the whole display 90 degrees clockwise (4 orientations). Call once per
// frame / menu tick after readPad() + bleTick(). Returns true on the tick it rotated; the
// screen has been blanked, so the caller must redraw.
static bool rotateHotkey() {
  static bool was = false;
  const bool now = (padState & 0x04) && (bleBtn & XB_RB);
  const bool fire = now && !was;
  was = now;
  if (fire) lcdSetRotation(lcdRot + 1);
  return fire;
}

