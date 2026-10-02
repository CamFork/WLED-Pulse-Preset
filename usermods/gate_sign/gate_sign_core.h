// gate_sign_core.h - Carpark entry gate sign: two-input decoder + sign state machine.
//
// Pure C++ (no Arduino / WLED includes) so the exact same logic runs on the
// ESP32 and in the host-side simulation. The usermod (gate_sign.cpp) owns the
// ISR, pin, presets, config and Info page; this file owns every decision.
//
// Time bases:
//   Decoder - int64 microseconds from the ISR timestamps (esp_timer_get_time()).
//   Sign    - uint32 milliseconds (millis()), unsigned subtraction everywhere.
//   On ESP32 millis() == esp_timer_get_time()/1000, so the two agree.
#pragma once
#include <stdint.h>

namespace gatesign {

enum Event : uint8_t {
  EV_NONE = 0, EV_CLOSED, EV_NOT_CLOSED, EV_UNAUTH, EV_GRANT, EV_ARRIVED, EV_LEFT, EV_HOLD,
  EV_TOUCH,   // closed reed first makes (gate reaching closed)
  EV_STUCK,   // both inputs on far longer than any hold (output stuck or welded)
  EV_COUNT
};

enum State : uint8_t {
  ST_BOOT = 0, ST_REST, ST_GREEN, ST_NOGRANT, ST_AMBER, ST_RED, ST_PROCEED, ST_BEHIND, ST_HOLD, ST_FAULT, ST_COUNT
};

inline const char* eventName(uint8_t e) {
  static const char* const n[] = {"none", "CLOSED", "NOT_CLOSED", "UNAUTH", "GRANT", "ARRIVED", "LEFT", "HOLD", "TOUCH", "STUCK"};
  return e < EV_COUNT ? n[e] : "?";
}
inline const char* stateName(uint8_t s) {
  static const char* const n[] = {"BOOT", "REST", "GREEN", "NOGRANT", "AMBER", "RED", "PROCEED", "BEHIND", "HOLD", "FAULT"};
  return s < ST_COUNT ? n[s] : "?";
}


struct Config {
  // --- sign behaviour ---
  uint32_t amberMs        = 2000;
  uint32_t behindMs       = 3000;   // guard >= 2000
  uint32_t proceedMs      = 10000;
  uint32_t greenMaxMs     = 15000;
  uint32_t grantValidMs   = 6000;
  uint32_t fullCycleMs    = 18000;
  uint32_t openRepeatMs   = 5000;
  uint32_t watchdogMs     = 20000;
  uint32_t idleFaultHours = 24;     // 0 = off
  // --- early PROCEED (gate still closing; confirmed by the closed reed sealing) ---
  bool     earlyProceed     = true;   // PROCEED on the closed reed's first make
  uint32_t proceedConfirmMs = 5000;   // closed reed must seal within this, else back to STOP
  // --- inputs (A = closed reed mirror, B = open reed mirror; A + B together = lock output on) ---
  uint32_t glitchMs       = 20;     // a raw level must hold this long to count
  uint32_t closedStableMs = 600;    // closed reed on (open reed off) this long = gate closed
  uint32_t touchMinMs     = 150;    // closed reed on this long = gate reaching closed (early PROCEED)
  uint32_t openStableMs   = 200;    // open reed on this long = gate fully open
  uint32_t leftStableMs   = 200;    // both off this long after open = gate left open
  uint32_t notClosedMs    = 1000;   // closed reed off this long = gate not closed
  uint32_t fastUnauthMs   = 300;    // closed reed off this long straight after closed, no grant = unauthorised
  uint32_t grantMinMs     = 150;    // both on this long = valid grant
  uint32_t holdMs         = 4000;   // both on this long = lock output latched (operator hold)
  uint32_t stuckMs        = 60000;  // both on this long = stuck output -> CHECK SENSORS

  void applyGuards() {
    if (glitchMs < 1)            glitchMs = 1;
    if (glitchMs > 60)           glitchMs = 60;
    if (behindMs < 2000)         behindMs = 2000;
    if (idleFaultHours > 720)    idleFaultHours = 720;
    if (proceedConfirmMs < 1000)  proceedConfirmMs = 1000;
    if (proceedConfirmMs > 30000) proceedConfirmMs = 30000;
    if (closedStableMs < 300)    closedStableMs = 300;
    if (touchMinMs < 50)         touchMinMs = 50;
    if (touchMinMs > closedStableMs) touchMinMs = closedStableMs;
    if (openStableMs < 50)       openStableMs = 50;
    if (openStableMs > 2000)     openStableMs = 2000;
    if (leftStableMs < 50)       leftStableMs = 50;
    if (leftStableMs > 2000)     leftStableMs = 2000;
    if (notClosedMs < 300)       notClosedMs = 300;
    if (fastUnauthMs < 100)      fastUnauthMs = 100;
    if (fastUnauthMs > notClosedMs) fastUnauthMs = notClosedMs;
    if (grantMinMs < 50)         grantMinMs = 50;
    if (grantMinMs > 1000)       grantMinMs = 1000;
    if (holdMs < grantMinMs + 1000) holdMs = grantMinMs + 1000;
    if (stuckMs < holdMs + 10000)   stuckMs = holdMs + 10000;
  }
};

struct DecodedEvent {
  Event   ev;
  int64_t tUs;         // logical time the event became true
  int64_t segStartUs;  // start of the segment that produced it
  bool    late;        // decoded only after its segment had already ended (loop stall)
};

// -----------------------------------------------------------------------------
// Decoder: two inputs (A = closed reed mirror, B = open reed mirror) -> events.
// The reeds can never both be secure, so A and B on together means a lock output
// is on (valid grant); how long they stay on together tells grant from hold.
// -----------------------------------------------------------------------------
enum Zone : uint8_t { Z_MOVING = 0, Z_CLOSED, Z_OPEN, Z_BOTH };
inline const char* zoneName(uint8_t z) {
  static const char* const n[] = {"moving", "closed", "open", "both"};
  return z < 4 ? n[z] : "?";
}
struct ZoneSeg { uint8_t z; uint32_t ms; };

class Decoder {
public:
  static const uint8_t LOG_N = 12;
  static const uint8_t Q_N   = 32;
  const Config* cfg = nullptr;

  bool     started = false;
  // per-input debounce
  bool     deb[2]  = {false, false};
  bool     raw[2]  = {false, false};
  int64_t  rawSince[2] = {0, 0};

  uint8_t  zone = Z_MOVING;
  int64_t  zoneStartUs = 0;
  // per-zone-segment flags
  bool fGrant = false, fHold = false, fStuck = false, fTouch = false, fClosed = false, fArr = false, fLeft = false, fUnauthChecked = false;
  // history
  uint8_t  confirmedPos   = Z_MOVING;  // last stable position: CLOSED, OPEN or MOVING
  bool     unauthArmed    = false;     // this closed-reed-off period began straight after a CLOSED
  bool     aOff           = false;     // closed input currently off
  int64_t  aOffSinceUs    = 0;
  bool     fNotClosed     = false;
  bool     grantEver      = false;
  int64_t  lastGrantUs    = 0;

  uint32_t glitches = 0, queueDrops = 0;
  ZoneSeg  log[LOG_N];
  uint8_t  logHead = 0, logCount = 0;

  void begin(const Config* c, bool aOn, bool bOn, int64_t nowUs) {
    cfg = c; started = true;
    deb[0] = raw[0] = aOn; deb[1] = raw[1] = bOn;
    rawSince[0] = rawSince[1] = nowUs;
    zone = zoneOf(aOn, bOn); zoneStartUs = nowUs;
    clearSegFlags();
    confirmedPos = Z_MOVING; unauthArmed = false;
    aOff = !aOn; aOffSinceUs = nowUs; fNotClosed = false;
    qHead = qTail = 0;
  }

  bool closedLevel() const { return zone == Z_CLOSED; }   // gate closed (for the watchdog)
  bool inputA() const { return deb[0]; }
  bool inputB() const { return deb[1]; }
  uint8_t currentZone() const { return zone; }
  int64_t zoneStart() const { return zoneStartUs; }

  void edge(uint8_t which, bool on, int64_t tUs) {
    if (!started || which > 1) return;
    settle(tUs);
    if (on == raw[which]) return;
    if (raw[which] != deb[which]) { glitches++; raw[which] = on; return; }  // reverted inside the glitch window
    raw[which] = on; rawSince[which] = tUs;
  }
  void resync(bool aOn, bool bOn, int64_t nowUs) {
    if (!started) return;
    if (aOn != raw[0]) edge(0, aOn, nowUs);
    if (bOn != raw[1]) edge(1, bOn, nowUs);
  }
  void poll(int64_t nowUs) {
    if (!started) return;
    settle(nowUs);
    int64_t effEnd = nowUs;    // a pending change caps the confirmed hold time
    for (uint8_t i = 0; i < 2; i++) if (raw[i] != deb[i] && rawSince[i] < effEnd) effEnd = rawSince[i];
    if (effEnd > zoneStartUs) check(effEnd);
  }
  bool pop(DecodedEvent& out) {
    if (qHead == qTail) return false;
    out = q[qTail]; qTail = (uint8_t)((qTail + 1) % Q_N); return true;
  }
  bool logAt(uint8_t i, ZoneSeg& s) const {
    if (i >= logCount) return false;
    s = log[(uint8_t)((logHead + LOG_N - 1 - i) % LOG_N)]; return true;
  }

private:
  DecodedEvent q[Q_N];
  uint8_t qHead = 0, qTail = 0;
  bool inCommit = false;

  static uint8_t zoneOf(bool a, bool b) { return a ? (b ? Z_BOTH : Z_CLOSED) : (b ? Z_OPEN : Z_MOVING); }
  void clearSegFlags() { fGrant = fHold = fStuck = fTouch = fClosed = fArr = fLeft = fUnauthChecked = false; }
  void emit(Event e, int64_t t) {
    uint8_t n = (uint8_t)((qHead + 1) % Q_N);
    if (n == qTail) { queueDrops++; return; }
    q[qHead].ev = e; q[qHead].tUs = t; q[qHead].segStartUs = zoneStartUs; q[qHead].late = inCommit;
    qHead = n;
  }
  bool grantRecent(int64_t t) const { return grantEver && (t - lastGrantUs) < (int64_t)cfg->grantValidMs * 1000; }

  // Confirm debounced changes, in time order, up to tUs.
  void settle(int64_t tUs) {
    for (;;) {
      int idx = -1; int64_t best = 0;
      for (uint8_t i = 0; i < 2; i++)
        if (raw[i] != deb[i] && tUs - rawSince[i] >= (int64_t)cfg->glitchMs * 1000 && (idx < 0 || rawSince[i] < best)) { idx = i; best = rawSince[i]; }
      if (idx < 0) return;
      bool a = deb[0], b = deb[1];
      if (idx == 0) a = raw[0]; else b = raw[1];
      changeZone(zoneOf(a, b), best);
      deb[idx] = raw[idx];
    }
  }

  // Time-based events for the current zone, evaluated at time t.
  void check(int64_t t) {
    const Config& c = *cfg;
    uint32_t held = (uint32_t)((t - zoneStartUs) / 1000);
    auto at = [&](uint32_t ms) { return zoneStartUs + (int64_t)ms * 1000; };
    switch (zone) {
      case Z_BOTH:
        if (!fGrant && held >= c.grantMinMs) { fGrant = true; grantEver = true; lastGrantUs = at(c.grantMinMs); emit(EV_GRANT, lastGrantUs); }
        if (!fHold  && held >= c.holdMs)     { fHold = true;  emit(EV_HOLD,  at(c.holdMs)); }
        if (!fStuck && held >= c.stuckMs)    { fStuck = true; emit(EV_STUCK, at(c.stuckMs)); }
        break;
      case Z_CLOSED:
        if (!fTouch  && held >= c.touchMinMs)     { fTouch = true;  emit(EV_TOUCH,  at(c.touchMinMs)); }
        if (!fClosed && held >= c.closedStableMs) { fClosed = true; confirmedPos = Z_CLOSED; emit(EV_CLOSED, at(c.closedStableMs)); }
        break;
      case Z_OPEN:
        if (!fArr && held >= c.openStableMs) { fArr = true; confirmedPos = Z_OPEN; emit(EV_ARRIVED, at(c.openStableMs)); }
        break;
      case Z_MOVING:
        if (!fLeft && held >= c.leftStableMs && confirmedPos == Z_OPEN) { fLeft = true; confirmedPos = Z_MOVING; emit(EV_LEFT, at(c.leftStableMs)); }
        break;
    }
    // closed reed off (gate open or moving): unauthorised-open and not-closed checks
    if (aOff) {
      uint32_t offMs = (uint32_t)((t - aOffSinceUs) / 1000);
      if (!fUnauthChecked && offMs >= c.fastUnauthMs) {
        fUnauthChecked = true;
        int64_t te = aOffSinceUs + (int64_t)c.fastUnauthMs * 1000;
        if (unauthArmed && !grantRecent(te)) emit(EV_UNAUTH, te);
      }
      if (!fNotClosed && offMs >= c.notClosedMs) { fNotClosed = true; emit(EV_NOT_CLOSED, aOffSinceUs + (int64_t)c.notClosedMs * 1000); }
    }
  }

  void changeZone(uint8_t nz, int64_t tc) {
    if (nz == zone) return;
    inCommit = true; check(tc); inCommit = false;   // late-detect thresholds (loop stalls)
    int64_t dur = tc - zoneStartUs; if (dur < 0) dur = 0;
    log[logHead] = {zone, (uint32_t)((dur + 500) / 1000)};
    logHead = (uint8_t)((logHead + 1) % LOG_N);
    if (logCount < LOG_N) logCount++;
    bool wasClosedFired = (zone == Z_CLOSED && fClosed);
    bool aWasOn = (zone == Z_CLOSED || zone == Z_BOTH);
    bool aNowOn = (nz == Z_CLOSED || nz == Z_BOTH);
    if (aWasOn && !aNowOn) { aOff = true; aOffSinceUs = tc; fNotClosed = false; fUnauthChecked = false; unauthArmed = wasClosedFired; }
    if (aNowOn) aOff = false;
    bool keepUnauthCheck = aOff && !(aWasOn && !aNowOn);
    bool saved = fUnauthChecked;
    zone = nz; zoneStartUs = tc;
    clearSegFlags();
    if (keepUnauthCheck) fUnauthChecked = saved;   // still the same closed-reed-off period
  }
};

// -----------------------------------------------------------------------------
// Sign: events + timers -> sign state
// -----------------------------------------------------------------------------
class Sign {
public:
  const Config* cfg = nullptr;

  State    st = ST_BOOT;
  uint32_t stSinceMs = 0;
  bool     faultIdle = false;

  // cycle variables
  bool     cycleActive      = false;
  bool     visitedOpen      = false;
  uint32_t notClosedSince   = 0;
  bool     grantEver        = false;
  uint32_t lastGrantMs      = 0;
  uint32_t lastEventMs      = 0;
  uint8_t  lastEvent        = EV_NONE;
  bool     leftDuringBehind = false;
  bool     arrivedEver      = false;
  uint32_t lastArrivedMs    = 0;
  bool     leftSeen         = false;   // gate has left open in this cycle (cleared by ARRIVED / CLOSED)
  bool     earlyPending     = false;   // PROCEED shown before the closed reed sealed
  bool     earlyUsed        = false;   // once per closing (cleared by ARRIVED / CLOSED)
  uint32_t earlyCount       = 0, earlyTimeouts = 0;

  // diagnostics
  uint32_t counts[EV_COUNT] = {0};
  bool     behindDeltaValid = false;
  uint32_t lastBehindDeltaMs = 0;
  uint32_t lastRawChangeMs  = 0;

  void begin(const Config* c, uint32_t nowMs) {
    cfg = c;
    st = ST_BOOT; stSinceMs = nowMs;
    faultIdle = false;
    cycleActive = visitedOpen = leftDuringBehind = false;
    leftSeen = earlyPending = earlyUsed = false;
    lastEventMs = nowMs;
    lastRawChangeMs = nowMs;
  }

  void onRawChange(uint32_t tMs) {
    lastRawChangeMs = tMs;
    if (faultIdle) { faultIdle = false; enter(ST_REST, tMs); }
  }

  // late: after a loop stall a CLOSED can be decoded only once its ON has already ended. The
  // gate is then no longer (or no longer continuously) closed, so PROCEED is withheld.
  void onEvent(Event e, uint32_t nowMs, uint32_t segStartMs, bool late = false) {
    if (e >= EV_COUNT) return;
    counts[e]++;
    lastEvent = e;
    lastEventMs = nowMs;
    const Config& c = *cfg;
    switch (e) {
      case EV_CLOSED:
        if (st == ST_HOLD) enter(ST_REST, nowMs);
        else if (cycleActive && (visitedOpen || (uint32_t)(nowMs - notClosedSince) >= c.fullCycleMs)) enter(late ? ST_REST : ST_PROCEED, nowMs);
        else if (!cycleActive && st == ST_PROCEED) { /* keep its timer */ }
        else if (!cycleActive && st == ST_GREEN && grantRecent(nowMs)) { /* keep GREEN */ }
        else enter(ST_REST, nowMs);
        cycleActive = false; visitedOpen = false; leftDuringBehind = false;
        leftSeen = false; earlyUsed = false;
        break;

      case EV_UNAUTH:
        startCycle(segStartMs);
        enter(ST_NOGRANT, nowMs);
        break;

      case EV_NOT_CLOSED: {
        bool was = cycleActive;
        if (!was) startCycle(segStartMs);
        if (st == ST_BOOT) { enter(ST_RED, nowMs); visitedOpen = true; }
        else if (!was && (st == ST_REST || st == ST_PROCEED || st == ST_GREEN)) {
          if (grantRecent(nowMs)) { if (st != ST_GREEN) enter(ST_GREEN, nowMs); }
          else enter(ST_NOGRANT, nowMs);
        }
        break;
      }

      case EV_GRANT:
        if (st == ST_HOLD) break;
        grantEver = true; lastGrantMs = nowMs;
        if (!cycleActive) enter(ST_GREEN, nowMs);          // (re)starts the green timer
        else if (visitedOpen) {
          enter(ST_BEHIND, nowMs);
          leftDuringBehind = false;
          if (arrivedEver) { behindDeltaValid = true; lastBehindDeltaMs = nowMs - lastArrivedMs; }
        } else {
          if (st != ST_GREEN) enter(ST_GREEN, nowMs);      // repeat press while opening: never BEHIND
        }
        break;

      case EV_ARRIVED:
        visitedOpen = true;
        if (!cycleActive) startCycle(nowMs);
        leftSeen = false; earlyUsed = false;   // back at open (also a reversal while closing)
        if (arrivedEver && (uint32_t)(nowMs - lastArrivedMs) < c.openRepeatMs) {
          if (st == ST_PROCEED && earlyPending) enter(ST_AMBER, nowMs);   // reversed after an early PROCEED
          break;
        }
        arrivedEver = true; lastArrivedMs = nowMs;
        if (st == ST_HOLD || st == ST_BEHIND) break;
        enter(ST_AMBER, nowMs);
        break;

      case EV_LEFT:
        if (st == ST_HOLD) break;
        if (leftSeen) break;      // a second leave with no arrival between
        leftSeen = true;
        if (st == ST_BEHIND) { leftDuringBehind = true; break; }
        if (st == ST_RED) break;
        enter(ST_RED, nowMs);   // GREEN, NOGRANT, AMBER, FAULT (and, as a safe fallback, any other state)
        break;

      case EV_HOLD:
        enter(ST_HOLD, nowMs);
        break;

      case EV_TOUCH:   // closed reed first makes while the gate is closing
        if (c.earlyProceed && cycleActive && visitedOpen && leftSeen && !earlyUsed && st == ST_RED) startEarly(nowMs);
        break;

      case EV_STUCK:
        enter(ST_FAULT, nowMs);
        break;

      default: break;
    }
  }

  void tick(uint32_t nowMs, bool inputOn) {
    const Config& c = *cfg;
    uint32_t in = nowMs - stSinceMs;
    switch (st) {
      case ST_AMBER:   if (in >= c.amberMs) enter(ST_RED, nowMs); break;
      case ST_BEHIND:  if (in >= c.behindMs) enter(leftDuringBehind ? ST_RED : ST_AMBER, nowMs); break;
      case ST_GREEN:   if (in >= c.greenMaxMs) enter(cycleActive ? ST_RED : ST_REST, nowMs); break;
      case ST_PROCEED:
        if (earlyPending) {   // shown before the closed reed sealed: it must seal in time
          if (in >= c.proceedConfirmMs) { earlyTimeouts++; enter(ST_RED, nowMs); }
        } else if (in >= c.proceedMs) enter(cycleActive ? ST_RED : ST_REST, nowMs);
        break;
      default: break;
    }
    if (cycleActive && !inputOn && st != ST_HOLD && st != ST_FAULT &&
        (uint32_t)(nowMs - lastEventMs) >= c.watchdogMs) {
      enter(ST_FAULT, nowMs);
    }
    if (c.idleFaultHours && !faultIdle &&
        (uint32_t)(nowMs - lastRawChangeMs) >= c.idleFaultHours * 3600000UL) {
      faultIdle = true;
      enter(ST_FAULT, nowMs);
    }
  }

private:
  bool grantRecent(uint32_t nowMs) const {
    return grantEver && (uint32_t)(nowMs - lastGrantMs) < cfg->grantValidMs;
  }
  void startCycle(uint32_t sinceMs) {
    if (cycleActive) return;
    cycleActive = true;
    notClosedSince = sinceMs;
  }
  void startEarly(uint32_t nowMs) {
    enter(ST_PROCEED, nowMs);
    earlyPending = true; earlyUsed = true; earlyCount++;
  }
  void enter(State s, uint32_t nowMs) {
    st = s;
    stSinceMs = nowMs;
    earlyPending = false;   // any state change ends an early PROCEED (CLOSED re-enters PROCEED as confirmed)
  }
};

} // namespace gatesign
