// gate_sign_core.h - Carpark entry gate sign: relay decoder + sign state machine.
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
  EV_NONE = 0, EV_CLOSED, EV_NOT_CLOSED, EV_UNAUTH, EV_GRANT, EV_ARRIVED, EV_LEFT, EV_HOLD, EV_COUNT
};

enum State : uint8_t {
  ST_BOOT = 0, ST_REST, ST_GREEN, ST_NOGRANT, ST_AMBER, ST_RED, ST_PROCEED, ST_BEHIND, ST_HOLD, ST_FAULT, ST_COUNT
};

inline const char* eventName(uint8_t e) {
  static const char* const n[] = {"none", "CLOSED", "NOT_CLOSED", "UNAUTH", "GRANT", "ARRIVED", "LEFT", "HOLD"};
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
  uint32_t closedStableMs = 600;    // guard >= longest pulse + 2*tol (480); 600 leaves margin over the 400 ms leave pulse
  uint32_t greenMaxMs     = 15000;
  uint32_t grantValidMs   = 6000;
  uint32_t fullCycleMs    = 18000;
  uint32_t openRepeatMs   = 5000;
  uint32_t watchdogMs     = 20000;
  uint32_t idleFaultHours = 24;     // 0 = off
  // --- decoder (must match the Inception actions) ---
  // Inception pulse on/off times are limited to 200, 300, 400, 500, 750 ms, 1 s ... (200 ms minimum)
  uint32_t glitchMs       = 20;
  uint32_t tolMs          = 40;     // guard <= 45 (keeps the 200/300/400 ms ranges apart)
  uint32_t grantMs        = 200;    // 200 on / 200 off
  uint32_t grantCount     = 3;      // guard >= 3
  uint32_t holdOnMs       = 200;    // 200 on / 500 off
  uint32_t holdGapMs      = 500;
  uint32_t holdCount      = 2;      // guard >= 2 (the first pulse of a run needs no gap match)
  uint32_t arrivedMs      = 300;    // 300 on / 300 off
  uint32_t leftMs         = 400;    // 400 on / 400 off
  uint32_t markerSteadyMs = 500;    // guard >= longest marker + 2*tol
  uint32_t notClosedMs    = 1000;   // guard >= longest gap + 2*tol, and >= 600
  uint32_t fastUnauthMs   = 300;    // guard >= grant gap + 2*tol, and >= 150

  void applyGuards() {
    if (tolMs > 45)            tolMs = 45;
    if (glitchMs < 1)          glitchMs = 1;
    if (glitchMs > 60)         glitchMs = 60;
    if (behindMs < 2000)       behindMs = 2000;
    if (grantCount < 3)        grantCount = 3;
    if (grantCount > 200)      grantCount = 200;
    if (holdCount < 2)         holdCount = 2;
    if (holdCount > 200)       holdCount = 200;
    if (idleFaultHours > 720)  idleFaultHours = 720;
    // Thresholds derived from the pulse table, so a retimed Inception action can't make them overlap:
    uint32_t minClosed = maxPulseMs() + 2 * tolMs;                 // steady ON outlasts any pulse
    if (minClosed < 400) minClosed = 400;
    if (closedStableMs < minClosed) closedStableMs = minClosed;
    uint32_t maxMarker = arrivedMs > leftMs ? arrivedMs : leftMs;  // a repeat gap is never "steady"
    if (markerSteadyMs < maxMarker + 2 * tolMs) markerSteadyMs = maxMarker + 2 * tolMs;
    uint32_t maxGap = holdGapMs;                                   // steady OFF outlasts any gap
    if (leftMs > maxGap) maxGap = leftMs;
    if (markerSteadyMs > maxGap) maxGap = markerSteadyMs;
    uint32_t minNotClosed = maxGap + 2 * tolMs;
    if (minNotClosed < 600) minNotClosed = 600;
    if (notClosedMs < minNotClosed) notClosedMs = minNotClosed;
    uint32_t minUnauth = grantMs + 2 * tolMs;                      // a grant gap over closed is never UNAUTH
    if (minUnauth < 150) minUnauth = 150;
    if (fastUnauthMs < minUnauth) fastUnauthMs = minUnauth;
  }
  uint32_t maxPulseMs() const {
    uint32_t m = grantMs;
    if (holdOnMs  > m) m = holdOnMs;
    if (arrivedMs > m) m = arrivedMs;
    if (leftMs    > m) m = leftMs;
    return m;
  }
  // CLOSED threshold for an ON that starts inside a marker burst (see Decoder::commit)
  uint32_t mergedCloseGuardMs() const { return 2 * maxPulseMs() + 3 * tolMs; }
};

struct DecodedEvent {
  Event   ev;
  int64_t tUs;         // logical time the event became true
  int64_t segStartUs;  // start of the segment that produced it
  bool    late;        // decoded only after its segment had already ended (loop stall)
};

struct Segment { bool on; uint32_t ms; };

// -----------------------------------------------------------------------------
// Decoder: raw edges -> debounced segments -> seven events
// -----------------------------------------------------------------------------
class Decoder {
public:
  static const uint8_t LOG_N = 12;
  static const uint8_t Q_N   = 32;

  const Config* cfg = nullptr;

  // debounce
  bool    started    = false;
  bool    debOn      = false;
  bool    rawOn      = false;
  int64_t rawSinceUs = 0;
  int64_t segStartUs = 0;
  int64_t lastRawUs  = 0;

  // per-segment level-event flags
  bool firedClosed    = false;
  bool checkedUnauth  = false;
  bool firedNotClosed = false;
  bool closeGuard     = false;   // this ON began inside a marker burst

  // history
  bool     prevOnClosed = false;       // the ON segment before this OFF produced CLOSED
  uint32_t lastOffMs    = 0xFFFFFFFFu; // OFF gap before the current/next ON (boot = "long")
  bool     grantEver    = false;
  int64_t  lastGrantUs  = 0;

  // pulse runs / latches
  uint8_t grantRun = 0, holdRun = 0;
  bool grantLatch = false, holdLatch = false, arrLatch = false, leftLatch = false;
  bool lastPulseMatched = false;   // the last ON pulse had a known signal width (any burst)

  // stats
  uint32_t glitches = 0;
  uint32_t queueDrops = 0;
  Segment  log[LOG_N];
  uint8_t  logHead = 0, logCount = 0;

  void begin(const Config* c, bool on, int64_t nowUs) {
    cfg = c;
    started = true;
    debOn = rawOn = on;
    rawSinceUs = segStartUs = lastRawUs = nowUs;
    firedClosed = checkedUnauth = firedNotClosed = closeGuard = false;
    prevOnClosed = false;
    lastOffMs = 0xFFFFFFFFu;
    clearRuns();
    qHead = qTail = 0;
  }

  bool level() const { return debOn; }
  int64_t segmentStartUs() const { return segStartUs; }

  // Feed one raw edge (level after the edge, ISR timestamp). Edges must arrive in time order.
  void edge(bool on, int64_t tUs) {
    if (!started) return;
    lastRawUs = tUs;
    if (rawOn != debOn && tUs - rawSinceUs >= (int64_t)cfg->glitchMs * 1000) commit(rawSinceUs, rawOn);
    if (on == rawOn) return;                       // duplicate level (e.g. after resync)
    if (rawOn != debOn) { glitches++; rawOn = on; return; } // reverted inside the glitch window
    rawOn = on;
    rawSinceUs = tUs;
  }

  // After ring-buffer overflow: accept the real pin level and forget partial bursts.
  void resync(bool on, int64_t nowUs) {
    if (!started) return;
    if (on != rawOn) edge(on, nowUs);
    clearRuns();
  }

  // Evaluate time-based level events. Call every loop with the current time.
  void poll(int64_t nowUs) {
    if (!started) return;
    if (rawOn != debOn && nowUs - rawSinceUs >= (int64_t)cfg->glitchMs * 1000) commit(rawSinceUs, rawOn);
    int64_t effEnd = (rawOn != debOn) ? rawSinceUs : nowUs; // a pending change caps the confirmed hold time
    if (effEnd > segStartUs) levelCheck(effEnd - segStartUs);
  }

  bool pop(DecodedEvent& out) {
    if (qHead == qTail) return false;
    out = q[qTail];
    qTail = (uint8_t)((qTail + 1) % Q_N);
    return true;
  }

  // newest first
  bool logAt(uint8_t i, Segment& s) const {
    if (i >= logCount) return false;
    s = log[(uint8_t)((logHead + LOG_N - 1 - i) % LOG_N)];
    return true;
  }

private:
  DecodedEvent q[Q_N];
  uint8_t qHead = 0, qTail = 0;
  bool inCommit = false;

  void emit(Event e, int64_t t) {
    uint8_t n = (uint8_t)((qHead + 1) % Q_N);
    if (n == qTail) { queueDrops++; return; }
    q[qHead].ev = e; q[qHead].tUs = t; q[qHead].segStartUs = segStartUs; q[qHead].late = inCommit;
    qHead = n;
  }

  void clearRuns() {
    grantRun = holdRun = 0;
    grantLatch = holdLatch = arrLatch = leftLatch = false;
    lastPulseMatched = false;
  }

  bool near(uint32_t v, uint32_t target) const {
    return v + cfg->tolMs >= target && v <= target + cfg->tolMs;
  }

  bool grantRecent(int64_t tUs) const {
    return grantEver && (tUs - lastGrantUs) < (int64_t)cfg->grantValidMs * 1000;
  }

  void levelCheck(int64_t heldUs) {
    uint32_t heldMs = (uint32_t)(heldUs / 1000);
    if (debOn) {
      uint32_t thr = cfg->closedStableMs;
      if (closeGuard && cfg->mergedCloseGuardMs() > thr) thr = cfg->mergedCloseGuardMs();
      if (!firedClosed && heldMs >= thr) {
        firedClosed = true;
        emit(EV_CLOSED, segStartUs + (int64_t)thr * 1000);
        clearRuns();
      }
    } else {
      if (!checkedUnauth && heldMs >= cfg->fastUnauthMs) {
        checkedUnauth = true;  // evaluated once per OFF segment, at the 150 ms mark
        int64_t te = segStartUs + (int64_t)cfg->fastUnauthMs * 1000;
        if (prevOnClosed && !grantRecent(te)) emit(EV_UNAUTH, te);
      }
      if (!firedNotClosed && heldMs >= cfg->notClosedMs) {
        firedNotClosed = true;
        emit(EV_NOT_CLOSED, segStartUs + (int64_t)cfg->notClosedMs * 1000);
        clearRuns();
      }
    }
  }

  void classify(uint32_t w, uint32_t g, int64_t tEnd) {
    const Config& c = *cfg;
    bool gLong = g >= c.markerSteadyMs;
    if (gLong) { arrLatch = false; leftLatch = false; }
    lastPulseMatched = near(w, c.grantMs) || near(w, c.holdOnMs) || near(w, c.arrivedMs) || near(w, c.leftMs);

    // GRANT: 100 ms pulses, 100 ms gaps (first pulse of a run: any gap)
    if (near(w, c.grantMs)) {
      if (grantRun > 0 && near(g, c.grantMs)) { if (grantRun < 250) grantRun++; }
      else { grantRun = 1; grantLatch = false; }
      if (grantRun >= c.grantCount && !grantLatch) {
        grantLatch = true; grantEver = true; lastGrantUs = tEnd;
        emit(EV_GRANT, tEnd);
      }
    } else { grantRun = 0; grantLatch = false; }

    // HOLD: 100 ms pulses, 500 ms gaps (first pulse of a run: any gap)
    if (near(w, c.holdOnMs)) {
      if (holdRun > 0 && near(g, c.holdGapMs)) { if (holdRun < 250) holdRun++; }
      else { holdRun = 1; holdLatch = false; }
      if (holdRun >= c.holdCount && !holdLatch) { holdLatch = true; emit(EV_HOLD, tEnd); }
    } else { holdRun = 0; holdLatch = false; }

    // ARRIVED: 200 ms pulse after a steady gap, or a 200 ms repeat gap. Once per burst.
    if (near(w, c.arrivedMs) && (gLong || near(g, c.arrivedMs))) {
      if (!arrLatch) emit(EV_ARRIVED, tEnd);
      arrLatch = true;
    } else arrLatch = false;

    // LEFT: 300 ms pulse after a steady gap, or a 300 ms repeat gap. Once per burst.
    if (near(w, c.leftMs) && (gLong || near(g, c.leftMs))) {
      if (!leftLatch) emit(EV_LEFT, tEnd);
      leftLatch = true;
    } else leftLatch = false;
  }

  void commit(int64_t tc, bool newOn) {
    int64_t dur = tc - segStartUs;
    if (dur < 0) dur = 0;
    inCommit = true;
    levelCheck(dur);                               // late-detect thresholds (loop stalls)
    inCommit = false;
    uint32_t durMs = (uint32_t)((dur + 500) / 1000);
    if (debOn) { classify(durMs, lastOffMs, tc); prevOnClosed = firedClosed; }
    else       { lastOffMs = durMs; }
    log[logHead] = {debOn, durMs};
    logHead = (uint8_t)((logHead + 1) % LOG_N);
    if (logCount < LOG_N) logCount++;

    debOn = newOn;
    segStartUs = tc;
    firedClosed = checkedUnauth = firedNotClosed = false;
    // An ON that starts inside a burst (short gap after a pulse of known width) may be two
    // pulses merged: a burst cut mid-ON by the next burst, a waiting marker starting straight
    // after a grant, or a late cut leaving an ON sliver. It must outlast any possible merge
    // (2 x longest pulse + tolerances) before it counts as CLOSED.
    closeGuard = debOn && lastPulseMatched && lastOffMs < cfg->markerSteadyMs;
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
        if (arrivedEver && (uint32_t)(nowMs - lastArrivedMs) < c.openRepeatMs) break;
        arrivedEver = true; lastArrivedMs = nowMs;
        if (st == ST_HOLD || st == ST_BEHIND) break;
        enter(ST_AMBER, nowMs);
        break;

      case EV_LEFT:
        if (st == ST_HOLD) break;
        if (st == ST_BEHIND) { leftDuringBehind = true; break; }
        if (st == ST_RED) break;
        enter(ST_RED, nowMs);   // GREEN, NOGRANT, AMBER, FAULT (and, as a safe fallback, any other state)
        break;

      case EV_HOLD:
        enter(ST_HOLD, nowMs);
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
      case ST_PROCEED: if (in >= c.proceedMs) enter(cycleActive ? ST_RED : ST_REST, nowMs); break;
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
  void enter(State s, uint32_t nowMs) {
    st = s;
    stSinceMs = nowMs;
  }
};

} // namespace gatesign
