// gate_sign.cpp - Carpark Entry Gate Sign usermod for WLED v16 (library-style, REGISTER_USERMOD)
//
// Two inputs from Inner Range Inception relays (each pulls its GPIO to GND when on):
//   A (pinClosed) = closed reed mirror: on when the gate is fully closed, or a lock output is on
//   B (pinOpen)   = open reed mirror:   on when the gate is fully open,   or a lock output is on
// The reeds are never both secure, so A and B on together = valid grant; how long they stay
// on together tells a grant (150 ms - 4 s) from a latched hold (>= 4 s) or a stuck output.
//
// All decisions live in gate_sign_core.h; this file is the WLED glue:
//   - ISR edge capture into a 128-entry ring buffer (esp_timer_get_time() stamps)
//   - pin ownership through PinManager
//   - preset application (only when the preset number changes, never 0)
//   - Config -> Usermods -> GateSign settings, with guard rails
//   - Info page diagnostics
#include "wled.h"
#include "gate_sign_core.h"
#ifdef ARDUINO_ARCH_ESP32
  #include "driver/gpio.h"
  #include "esp_timer.h"
#endif

using namespace gatesign;

#define GATESIGN_RB 128

static const char _gs_name[]   PROGMEM = "GateSign";
static const char _gs_note[]   PROGMEM = "Any valid card, UHF tag or RF remote read after the gate reaches open is treated as VALID USER BEHIND. Confirm the UHF reader's repeat-read hold-off covers a full gate cycle.";

class GateSignUsermod : public Usermod {
  // ---------------- settings ----------------
  bool    enabled      = true;
  int8_t  pinClosed    = 9;     // input A (MatrixPortal S3 header A2)
  int8_t  pinOpen      = 10;    // input B (MatrixPortal S3 header A3)
  bool    pullup       = true;
  bool    activeLow    = true;
  bool    drivePresets = true;
  uint8_t pBlank = 0, pGreen = 0, pAmber = 0, pRed = 0, pProceed = 0, pBehind = 0, pFault = 0;
  Config  cfg;

  // ---------------- runtime ----------------
  Decoder dec;
  Sign    sign;
  bool    initDone      = false;
  bool    pinOk         = false;
  int8_t  allocA        = -1;
  int8_t  allocB        = -1;
  bool    reinitPending = false;
  uint8_t lastApplied   = 0;
  uint32_t lostEdges    = 0;

  // ---------------- ISR ring buffer (bit 0 = level, bit 1 = input B) ----------------
  static volatile uint16_t rbHead;
  static volatile uint16_t rbTail;
  static int64_t           rbT[GATESIGN_RB];
  static uint8_t           rbL[GATESIGN_RB];
  static volatile uint32_t rbOverflows;
  static volatile bool     rbOverflowFlag;
  static int8_t            isrPinA;
  static int8_t            isrPinB;
  static portMUX_TYPE      rbMux;

  // Defined outside the class: an IRAM_ATTR function defined in the class body is
  // implicitly inline, and the Xtensa linker then fails with "literal placed after use".
  static void IRAM_ATTR isrA();
  static void IRAM_ATTR isrB();
  static void IRAM_ATTR push(int64_t t, uint8_t code);

  bool levelToOn(uint8_t lvl) const { return activeLow ? (lvl == 0) : (lvl != 0); }

  void deinitPins() {
    if (allocA >= 0) { detachInterrupt(digitalPinToInterrupt(allocA)); PinManager::deallocatePin(allocA, PinOwner::UM_Unspecified); }
    if (allocB >= 0) { detachInterrupt(digitalPinToInterrupt(allocB)); PinManager::deallocatePin(allocB, PinOwner::UM_Unspecified); }
    allocA = allocB = -1;
    pinOk = false;
  }

  void initPins() {
    deinitPins();
    if (!enabled || pinClosed < 0 || pinOpen < 0 || pinClosed == pinOpen) return;
    if (!PinManager::allocatePin(pinClosed, false, PinOwner::UM_Unspecified)) {
      DEBUG_PRINTF_P(PSTR("GateSign: GPIO%d not available\n"), pinClosed);
      return;
    }
    allocA = pinClosed;
    if (!PinManager::allocatePin(pinOpen, false, PinOwner::UM_Unspecified)) {
      DEBUG_PRINTF_P(PSTR("GateSign: GPIO%d not available\n"), pinOpen);
      deinitPins();
      return;
    }
    allocB = pinOpen;
    pinMode(pinClosed, pullup ? INPUT_PULLUP : INPUT);
    pinMode(pinOpen,   pullup ? INPUT_PULLUP : INPUT);
    isrPinA = pinClosed; isrPinB = pinOpen;
    attachInterrupt(digitalPinToInterrupt(pinClosed), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinOpen),   isrB, CHANGE);
    // Clear the buffer and read the real levels atomically, so no edge is lost or doubled.
    portENTER_CRITICAL(&rbMux);
    rbHead = rbTail = 0;
    rbOverflowFlag = false;
    uint8_t la = (uint8_t)gpio_get_level((gpio_num_t)pinClosed);
    uint8_t lb = (uint8_t)gpio_get_level((gpio_num_t)pinOpen);
    int64_t now = esp_timer_get_time();
    portEXIT_CRITICAL(&rbMux);
    dec.begin(&cfg, levelToOn(la), levelToOn(lb), now);
    sign.onRawChange((uint32_t)(now / 1000));  // (re)start the idle timer
    pinOk = true;
  }

  uint8_t presetFor(uint8_t s) const {
    switch (s) {
      case ST_BOOT: case ST_REST: case ST_HOLD: return pBlank;
      case ST_GREEN:   return pGreen;
      case ST_NOGRANT: case ST_RED: return pRed;
      case ST_AMBER:   return pAmber;
      case ST_PROCEED: return pProceed;
      case ST_BEHIND:  return pBehind;
      case ST_FAULT:   return pFault;
      default:         return 0;
    }
  }

  void applyIfChanged() {
    if (!drivePresets) return;
    uint8_t p = presetFor(sign.st);
    if (p == 0 || p == lastApplied) return;  // 0 = not set: leave the sign alone
    lastApplied = p;
    applyPreset(p);
  }

public:
  void setup() override {
    cfg.applyGuards();
    sign.begin(&cfg, millis());
    initPins();
    initDone = true;
  }

  void loop() override {
    if (!initDone) return;
    if (reinitPending) { reinitPending = false; initPins(); }   // also releases the pins when disabled
    if (!enabled) return;

    if (pinOk) {
      // Snapshot the time and the buffer head together: every edge drained is <= nowUs.
      portENTER_CRITICAL(&rbMux);
      int64_t  nowUs = esp_timer_get_time();
      uint16_t head  = rbHead;
      bool     ovf   = rbOverflowFlag;
      rbOverflowFlag = false;
      portEXIT_CRITICAL(&rbMux);

      bool rawChanged = false;
      int64_t lastEdgeUs = 0;
      while (rbTail != head) {
        int64_t t = rbT[rbTail];
        uint8_t l = rbL[rbTail];
        rbTail = (uint16_t)((rbTail + 1) % GATESIGN_RB);
        dec.edge((uint8_t)(l >> 1), levelToOn(l & 1), t);
        rawChanged = true; lastEdgeUs = t;
      }
      if (ovf) {
        lostEdges = rbOverflows;
        dec.resync(levelToOn((uint8_t)gpio_get_level((gpio_num_t)pinClosed)),
                   levelToOn((uint8_t)gpio_get_level((gpio_num_t)pinOpen)), nowUs);
        rawChanged = true; lastEdgeUs = nowUs;
      }
      dec.poll(nowUs);
      if (rawChanged) sign.onRawChange((uint32_t)(lastEdgeUs / 1000));

      uint32_t nowMs = (uint32_t)(nowUs / 1000);
      DecodedEvent e;
      while (dec.pop(e)) sign.onEvent(e.ev, nowMs, (uint32_t)(e.segStartUs / 1000), e.late);
      sign.tick(nowMs, dec.closedLevel());
    }
    applyIfChanged();
  }

  // ---------------- Info page ----------------
  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    uint32_t nowMs = millis();

    JsonArray a = user.createNestedArray(F("Gate sign"));
    if (!enabled)    { a.add(F("disabled")); return; }
    if (!pinOk)      { a.add(pinOpen < 0 ? F("open reed pin not set") : F("input pins not allocated")); return; }
    String s = stateName(sign.st);
    if (!drivePresets) s += F(" [monitor only]");
    a.add(s);

    a = user.createNestedArray(F("Gate inputs"));
    String in = F("closed reed "); in += dec.inputA() ? F("ON") : F("off");
    in += F(", open reed ");       in += dec.inputB() ? F("ON") : F("off");
    in += F(" = "); in += zoneName(dec.currentZone()); in += F(" for ");
    in += String((uint32_t)((esp_timer_get_time() - dec.zoneStart()) / 1000000)); in += F(" s");
    a.add(in);

    a = user.createNestedArray(F("Gate last event"));
    if (sign.lastEvent == EV_NONE) a.add(F("none yet"));
    else {
      String le = eventName(sign.lastEvent); le += ' ';
      le += String((nowMs - sign.lastEventMs) / 1000); le += F(" s ago");
      a.add(le);
    }

    a = user.createNestedArray(F("Gate zones (ms, newest first)"));
    String pl;
    ZoneSeg zs;
    for (uint8_t i = 0; dec.logAt(i, zs); i++) {
      if (i) pl += F(" · ");
      pl += zoneName(zs.z); pl += ' '; pl += String(zs.ms);
    }
    a.add(pl.length() ? pl : String(F("-")));

    a = user.createNestedArray(F("Gate event counts"));
    String c;
    c += F("grant ");     c += String(sign.counts[EV_GRANT]);
    c += F(", arrived "); c += String(sign.counts[EV_ARRIVED]);
    c += F(", left ");    c += String(sign.counts[EV_LEFT]);
    c += F(", hold ");    c += String(sign.counts[EV_HOLD]);
    c += F(", closed ");  c += String(sign.counts[EV_CLOSED]);
    c += F(", unauth ");  c += String(sign.counts[EV_UNAUTH]);
    c += F(", touch ");   c += String(sign.counts[EV_TOUCH]);
    c += F(", stuck ");   c += String(sign.counts[EV_STUCK]);
    a.add(c);

    a = user.createNestedArray(F("Gate early PROCEED"));
    String ep;
    if (!cfg.earlyProceed) ep = F("off");
    else { ep = String(sign.earlyCount); ep += F(" shown, "); ep += String(sign.earlyTimeouts); ep += F(" seal timeouts (back to STOP)"); }
    a.add(ep);

    a = user.createNestedArray(F("Gate noise"));
    String n = F("glitches "); n += String(dec.glitches);
    n += F(", lost edges "); n += String(lostEdges);
    a.add(n);

    a = user.createNestedArray(F("Gate last VALID USER BEHIND"));
    if (sign.behindDeltaValid) {
      String b = String(sign.lastBehindDeltaMs / 1000.0f, 1); b += F(" s after arrival");
      a.add(b);
    } else a.add(F("none yet"));

    a = user.createNestedArray(F("Gate time since last input change"));
    a.add(String((nowMs - sign.lastRawChangeMs) / 1000)); a.add(F(" s"));

    a = user.createNestedArray(F("Gate note"));
    a.add(FPSTR(_gs_note));
  }

  // ---------------- config ----------------
  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_gs_name));
    top[F("enabled")]          = enabled;
    top[F("amberMs")]          = cfg.amberMs;
    top[F("behindMs")]         = cfg.behindMs;
    top[F("proceedMs")]        = cfg.proceedMs;
    top[F("earlyProceed")]     = cfg.earlyProceed;
    top[F("proceedConfirmMs")] = cfg.proceedConfirmMs;
    top[F("greenMaxMs")]       = cfg.greenMaxMs;
    top[F("grantValidMs")]     = cfg.grantValidMs;
    top[F("fullCycleMs")]      = cfg.fullCycleMs;
    top[F("openRepeatMs")]     = cfg.openRepeatMs;
    top[F("watchdogMs")]       = cfg.watchdogMs;
    top[F("idleFaultHours")]   = cfg.idleFaultHours;

    JsonObject p = top.createNestedObject(F("presets"));
    p[F("presetBlank")]   = pBlank;
    p[F("presetGreen")]   = pGreen;
    p[F("presetAmber")]   = pAmber;
    p[F("presetRed")]     = pRed;
    p[F("presetProceed")] = pProceed;
    p[F("presetBehind")]  = pBehind;
    p[F("presetFault")]   = pFault;

    JsonObject d = top.createNestedObject(F("inputs"));
    d[F("pinClosed")]      = pinClosed;
    d[F("pinOpen")]        = pinOpen;
    d[F("pullup")]         = pullup;
    d[F("activeLow")]      = activeLow;
    d[F("drivePresets")]   = drivePresets;
    d[F("glitchMs")]       = cfg.glitchMs;
    d[F("closedStableMs")] = cfg.closedStableMs;
    d[F("touchMinMs")]     = cfg.touchMinMs;
    d[F("openStableMs")]   = cfg.openStableMs;
    d[F("leftStableMs")]   = cfg.leftStableMs;
    d[F("notClosedMs")]    = cfg.notClosedMs;
    d[F("fastUnauthMs")]   = cfg.fastUnauthMs;
    d[F("grantMinMs")]     = cfg.grantMinMs;
    d[F("holdMs")]         = cfg.holdMs;
    d[F("stuckMs")]        = cfg.stuckMs;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_gs_name)];
    if (top.isNull()) return false;

    bool    oldEnabled = enabled;
    int8_t  oldA = pinClosed, oldB = pinOpen;
    bool    oldPullup = pullup, oldActiveLow = activeLow;
    uint8_t oldPresets[7] = {pBlank, pGreen, pAmber, pRed, pProceed, pBehind, pFault};
    bool    oldDrive = drivePresets;
    Config  c = cfg;   // start from current values so a missing key keeps its value

    bool ok = true;
    ok &= getJsonValue(top[F("enabled")],          enabled,            true);
    ok &= getJsonValue(top[F("amberMs")],          c.amberMs,          2000);
    ok &= getJsonValue(top[F("behindMs")],         c.behindMs,         3000);
    ok &= getJsonValue(top[F("proceedMs")],        c.proceedMs,        10000);
    ok &= getJsonValue(top[F("earlyProceed")],     c.earlyProceed,     true);
    ok &= getJsonValue(top[F("proceedConfirmMs")], c.proceedConfirmMs, 5000);
    ok &= getJsonValue(top[F("greenMaxMs")],       c.greenMaxMs,       15000);
    ok &= getJsonValue(top[F("grantValidMs")],     c.grantValidMs,     6000);
    ok &= getJsonValue(top[F("fullCycleMs")],      c.fullCycleMs,      18000);
    ok &= getJsonValue(top[F("openRepeatMs")],     c.openRepeatMs,     5000);
    ok &= getJsonValue(top[F("watchdogMs")],       c.watchdogMs,       20000);
    ok &= getJsonValue(top[F("idleFaultHours")],   c.idleFaultHours,   24);

    JsonObject p = top[F("presets")];
    ok &= getJsonValue(p[F("presetBlank")],   pBlank,   0);
    ok &= getJsonValue(p[F("presetGreen")],   pGreen,   0);
    ok &= getJsonValue(p[F("presetAmber")],   pAmber,   0);
    ok &= getJsonValue(p[F("presetRed")],     pRed,     0);
    ok &= getJsonValue(p[F("presetProceed")], pProceed, 0);
    ok &= getJsonValue(p[F("presetBehind")],  pBehind,  0);
    ok &= getJsonValue(p[F("presetFault")],   pFault,   0);

    JsonObject d = top[F("inputs")];
    ok &= getJsonValue(d[F("pinClosed")],      pinClosed,        9);
    ok &= getJsonValue(d[F("pinOpen")],        pinOpen,          10);
    ok &= getJsonValue(d[F("pullup")],         pullup,           true);
    ok &= getJsonValue(d[F("activeLow")],      activeLow,        true);
    ok &= getJsonValue(d[F("drivePresets")],   drivePresets,     true);
    ok &= getJsonValue(d[F("glitchMs")],       c.glitchMs,       20);
    ok &= getJsonValue(d[F("closedStableMs")], c.closedStableMs, 600);
    ok &= getJsonValue(d[F("touchMinMs")],     c.touchMinMs,     150);
    ok &= getJsonValue(d[F("openStableMs")],   c.openStableMs,   200);
    ok &= getJsonValue(d[F("leftStableMs")],   c.leftStableMs,   200);
    ok &= getJsonValue(d[F("notClosedMs")],    c.notClosedMs,    1000);
    ok &= getJsonValue(d[F("fastUnauthMs")],   c.fastUnauthMs,   300);
    ok &= getJsonValue(d[F("grantMinMs")],     c.grantMinMs,     150);
    ok &= getJsonValue(d[F("holdMs")],         c.holdMs,         4000);
    ok &= getJsonValue(d[F("stuckMs")],        c.stuckMs,        60000);

    c.applyGuards();   // guard rails, whatever was typed
    cfg = c;           // Decoder and Sign hold a pointer to cfg: live update, no reflash

    if (initDone) {
      if (pinClosed != oldA || pinOpen != oldB || pullup != oldPullup || activeLow != oldActiveLow || enabled != oldEnabled)
        reinitPending = true;  // re-initialise from loop(), not from the web task
      uint8_t newPresets[7] = {pBlank, pGreen, pAmber, pRed, pProceed, pBehind, pFault};
      if (memcmp(oldPresets, newPresets, 7) != 0 || drivePresets != oldDrive)
        lastApplied = 0;       // re-apply the current state's preset with the new numbers
    }
    return ok;
  }

  // Hints on the settings page. Printed straight to the stream (no fixed buffers to truncate).
  void appendConfigData(Print& s) override {
    s.print(F("addInfo('GateSign:presets:presetBlank',1,'REST, HOLD, BOOT (0 = not set)');"));
    s.print(F("addInfo('GateSign:presets:presetRed',1,'STOP - WAIT FOR GATE');"));
    s.print(F("addInfo('GateSign:presets:presetAmber',1,'CLEAR GATE AND WAIT');"));
    s.print(F("addInfo('GateSign:presets:presetGreen',1,'ENTER');"));
    s.print(F("addInfo('GateSign:presets:presetBehind',1,'VALID USER BEHIND - PROCEED');"));
    s.print(F("addInfo('GateSign:presets:presetFault',1,'CHECK SENSORS');"));
    s.print(F("addInfo('GateSign:inputs:pinClosed',1,'A: closed reed mirror (GPIO9)');"));
    s.print(F("addInfo('GateSign:inputs:pinOpen',1,'B: open reed mirror (GPIO10)');"));
    s.print(F("addInfo('GateSign:inputs:drivePresets',1,'off = monitor only');"));
    s.print(F("addInfo('GateSign:inputs:grantMinMs',1,'ms both on = grant');"));
    s.print(F("addInfo('GateSign:inputs:holdMs',1,'ms both on = hold (blank)');"));
    s.print(F("addInfo('GateSign:inputs:stuckMs',1,'ms both on = CHECK SENSORS');"));
    s.print(F("addInfo('GateSign:inputs:touchMinMs',1,'ms closed reed on = early PROCEED');"));
    s.print(F("addInfo('GateSign:earlyProceed',1,'PROCEED on first closed-reed make');"));
    s.print(F("addInfo('GateSign:proceedConfirmMs',1,'ms for the reed to seal, else STOP');"));
    s.print(F("addInfo('GateSign:idleFaultHours',1,'h (0 = off)');"));
    s.print(F("addInfo('GateSign:behindMs',1,'ms (min 2000)');"));
  }

  uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

#ifdef GATESIGN_HOST_SIM
  // test hooks
  const Sign&    simSign() const { return sign; }
  const Decoder& simDec()  const { return dec; }
  uint8_t        simLastApplied() const { return lastApplied; }
  void           simSetPins(int8_t a, int8_t b) { pinClosed = a; pinOpen = b; }
  void           simSetPresets(uint8_t b, uint8_t g, uint8_t a, uint8_t r, uint8_t p, uint8_t be, uint8_t f) {
    pBlank = b; pGreen = g; pAmber = a; pRed = r; pProceed = p; pBehind = be; pFault = f;
  }
#endif
};

volatile uint16_t GateSignUsermod::rbHead = 0;
volatile uint16_t GateSignUsermod::rbTail = 0;
int64_t           GateSignUsermod::rbT[GATESIGN_RB];
uint8_t           GateSignUsermod::rbL[GATESIGN_RB];
volatile uint32_t GateSignUsermod::rbOverflows = 0;
volatile bool     GateSignUsermod::rbOverflowFlag = false;
int8_t            GateSignUsermod::isrPinA = 9;
int8_t            GateSignUsermod::isrPinB = 10;
portMUX_TYPE      GateSignUsermod::rbMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR GateSignUsermod::push(int64_t t, uint8_t code) {
  portENTER_CRITICAL_ISR(&rbMux);
  uint16_t next = (uint16_t)((rbHead + 1) % GATESIGN_RB);
  if (next == rbTail) { rbOverflows++; rbOverflowFlag = true; }
  else { rbT[rbHead] = t; rbL[rbHead] = code; rbHead = next; }
  portEXIT_CRITICAL_ISR(&rbMux);
}

void IRAM_ATTR GateSignUsermod::isrA() {
  int64_t t = esp_timer_get_time();
  push(t, (uint8_t)((uint8_t)gpio_get_level((gpio_num_t)isrPinA) & 1));
}

void IRAM_ATTR GateSignUsermod::isrB() {
  int64_t t = esp_timer_get_time();
  push(t, (uint8_t)(((uint8_t)gpio_get_level((gpio_num_t)isrPinB) & 1) | 2));
}

static GateSignUsermod gate_sign;
REGISTER_USERMOD(gate_sign);
