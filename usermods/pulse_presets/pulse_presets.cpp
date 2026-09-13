#include "wled.h"

/*
 * Usermod "pulse_presets"
 *
 * Watches a GPIO pin wired to a PLC dry relay contact. Counts the pulses in a
 * "burst" (a run of pulses separated by gaps shorter than gapTimeoutMs), measures
 * their average width, and compares that (count, width) pair against a small
 * table of patterns. On a match it calls applyPreset() to fire the configured
 * WLED preset.
 *
 * All of pin, timing and the pattern table are editable live under
 * Config -> Usermods -> "pulse_presets" - no recompiling needed to retune them.
 *
 * Wiring: the PLC relay's dry contact should short the pin to GND when active.
 * With "Active low" enabled (the default) the pin is configured INPUT_PULLUP,
 * so no external resistor is needed - just PLC-relay-common -> GND and
 * PLC-relay-NO -> this GPIO.
 *
 * The settings page has no concept of a custom "button" - it's auto-generated
 * from plain fields. So "restoreDefaults" is a checkbox: tick it and hit the
 * page's own Save button, and every other field here snaps back to default.
 *
 * Each pattern's "width" field also gets a dropdown of common PLC pulse
 * durations (200-750ms, 1-45s) injected next to it in the settings page -
 * pick one to fill the field, or ignore the dropdown and type any raw ms
 * value straight into the field, same as before. See appendConfigData().
 */

#ifndef PULSE_PRESETS_MAX_PATTERNS
  #define PULSE_PRESETS_MAX_PATTERNS 6
#endif

// Single source of truth for both the initial values below and what
// "restoreDefaults" resets back to - keeps the two from drifting apart.
#define PULSE_PRESETS_DEFAULT_ENABLED           true
#define PULSE_PRESETS_DEFAULT_PIN               -1
#define PULSE_PRESETS_DEFAULT_ACTIVE_LOW        true
#define PULSE_PRESETS_DEFAULT_DEBOUNCE_MS       30
#define PULSE_PRESETS_DEFAULT_GAP_TIMEOUT_MS    600
#define PULSE_PRESETS_DEFAULT_WIDTH_TOLERANCE_MS 60

class UsermodPulsePresets : public Usermod {
  private:
    // ---- persistent config ----
    bool    enabled         = PULSE_PRESETS_DEFAULT_ENABLED;
    int8_t  pin             = PULSE_PRESETS_DEFAULT_PIN;     // -1 = not configured
    bool    activeLow       = PULSE_PRESETS_DEFAULT_ACTIVE_LOW;  // true: contact pulls pin to GND when active (use INPUT_PULLUP)
    uint16_t debounceMs      = PULSE_PRESETS_DEFAULT_DEBOUNCE_MS;       // ignore edges faster than this (contact bounce)
    uint16_t gapTimeoutMs    = PULSE_PRESETS_DEFAULT_GAP_TIMEOUT_MS;    // silence on the line for this long closes the current burst
    uint16_t widthToleranceMs = PULSE_PRESETS_DEFAULT_WIDTH_TOLERANCE_MS; // +/- allowed difference between measured and configured pulse width
    bool    restoreDefaults = false; // one-shot checkbox: true -> reset everything below on next Save

    uint8_t  patCounts[PULSE_PRESETS_MAX_PATTERNS]  = {0}; // 0 = slot unused
    uint16_t patWidthsMs[PULSE_PRESETS_MAX_PATTERNS] = {0};
    uint8_t  patPresets[PULSE_PRESETS_MAX_PATTERNS] = {0};

    // ---- runtime state ----
    bool          initDone        = false;
    bool          rawActive       = false; // last raw (undebounced) active reading
    bool          debouncedActive = false; // debounced logical pin state
    unsigned long lastEdgeTime    = 0;     // time of last raw transition, for debounce
    unsigned long pulseStartTime  = 0;
    unsigned long lastPulseEndTime = 0;
    bool          burstOpen       = false; // true while a burst is still waiting on the gap timeout
    uint8_t       burstPulseCount = 0;
    uint32_t      burstWidthSum   = 0;

    // ---- diagnostics, shown in Info panel ----
    uint32_t totalBursts       = 0;
    uint32_t totalMatches      = 0;
    uint8_t  lastBurstCount    = 0;
    uint16_t lastBurstAvgWidth = 0;
    bool     lastBurstMatched  = false;
    uint8_t  lastMatchedPreset = 0;

    static const char _name[];
    static const char _enabled[];
    static const char _pin[];
    static const char _activeLow[];
    static const char _debounceMs[];
    static const char _gapTimeoutMs[];
    static const char _widthToleranceMs[];
    static const char _restoreDefaults[];
    static const char _patCounts[];
    static const char _patWidthsMs[];
    static const char _patPresets[];
    static const char _infoPreset[];
    static const char _infoStats[];

    // Clears any in-progress burst so a disable/reconfigure can never resume
    // into a stale, half-counted burst.
    void resetBurstState() {
      burstOpen       = false;
      burstPulseCount = 0;
      burstWidthSum   = 0;
    }

    void configurePin() {
      resetBurstState();
      if (pin < 0) return;
      if (PinManager::allocatePin(pin, false, PinOwner::UM_Unspecified)) {
        pinMode(pin, activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);
        // Deliberately do NOT trust whatever the pin happens to read right now -
        // it may be mid-pulse (wiring settling, or reconfigured while the PLC was
        // mid-burst). Always start from "idle" and require a real, clean rising
        // edge before we start treating anything as an active pulse.
        rawActive       = false;
        debouncedActive = false;
        lastEdgeTime    = millis();
      } else {
        DEBUG_PRINTLN(F("PulsePresets: pin allocation failed"));
        pin = -1;
      }
    }

    void finalizeBurst() {
      if (burstPulseCount == 0) return;
      uint16_t avgWidth = (uint16_t)(burstWidthSum / burstPulseCount);
      lastBurstCount    = burstPulseCount;
      lastBurstAvgWidth = avgWidth;
      lastBurstMatched  = false;
      totalBursts++;

      for (uint8_t i = 0; i < PULSE_PRESETS_MAX_PATTERNS; i++) {
        if (patCounts[i] == 0) continue; // slot unused
        if (patCounts[i] != burstPulseCount) continue;
        int32_t diff = (int32_t)avgWidth - (int32_t)patWidthsMs[i];
        if (diff < 0) diff = -diff;
        if (diff <= (int32_t)widthToleranceMs) {
          applyPreset(patPresets[i]);
          lastBurstMatched  = true;
          lastMatchedPreset = patPresets[i];
          totalMatches++;
          break; // first matching slot wins
        }
      }

      burstPulseCount = 0;
      burstWidthSum   = 0;
    }

  public:
    void setup() override {
      configurePin();
      initDone = true;
    }

    void loop() override {
      if (!enabled || pin < 0 || !initDone) {
        // if the usermod was just disabled mid-burst, don't let it resume
        // a stale burst count if it's re-enabled later
        if (burstOpen || burstPulseCount > 0) resetBurstState();
        return;
      }
      // Note: unlike most usermod loop()s, we deliberately do NOT skip this
      // while strip.isUpdating() - this usermod never touches a pixel, and on
      // a large/slow matrix skipping here would delay edge detection enough
      // to throw off pulse-width measurement against widthToleranceMs.

      unsigned long now = millis();
      bool raw = digitalRead(pin);
      bool activeRaw = activeLow ? !raw : raw;

      if (activeRaw != rawActive) {
        rawActive = activeRaw;
        lastEdgeTime = now;
      }

      if (activeRaw != debouncedActive && (now - lastEdgeTime) >= debounceMs) {
        debouncedActive = activeRaw;
        if (debouncedActive) {
          // rising edge: pulse starts
          pulseStartTime = now;
        } else {
          // falling edge: pulse ends
          unsigned long width = now - pulseStartTime;
          if (burstPulseCount < 250) burstPulseCount++; // guard against runaway counts
          burstWidthSum += width;
          lastPulseEndTime = now;
          burstOpen = true;
        }
      }

      if (burstOpen && !debouncedActive && (now - lastPulseEndTime) >= gapTimeoutMs) {
        finalizeBurst();
        burstOpen = false;
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray last = user.createNestedArray(FPSTR(_name));
      if (lastBurstCount == 0) {
        last.add(F("no pulses seen yet"));
      } else {
        char buf[64];
        snprintf_P(buf, sizeof(buf), PSTR("last: %ux %ums avg -> %s"),
                   lastBurstCount, lastBurstAvgWidth,
                   lastBurstMatched ? "matched" : "no match");
        last.add(String(buf));
      }

      JsonArray matched = user.createNestedArray(FPSTR(_infoPreset));
      if (lastBurstMatched) matched.add(lastMatchedPreset);
      else matched.add(F("none"));

      JsonArray stats = user.createNestedArray(FPSTR(_infoStats));
      stats.add(totalBursts);
      stats.add(F("/"));
      stats.add(totalMatches);
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)]           = enabled;
      top[FPSTR(_pin)]               = pin;
      top[FPSTR(_activeLow)]         = activeLow;
      top[FPSTR(_debounceMs)]        = debounceMs;
      top[FPSTR(_gapTimeoutMs)]      = gapTimeoutMs;
      top[FPSTR(_widthToleranceMs)]  = widthToleranceMs;

      JsonArray counts  = top.createNestedArray(FPSTR(_patCounts));
      JsonArray widths  = top.createNestedArray(FPSTR(_patWidthsMs));
      JsonArray presets = top.createNestedArray(FPSTR(_patPresets));
      for (uint8_t i = 0; i < PULSE_PRESETS_MAX_PATTERNS; i++) {
        counts.add(patCounts[i]);
        widths.add(patWidthsMs[i]);
        presets.add(patPresets[i]);
      }

      // Written last so it renders at the bottom of the settings panel -
      // always shows as unticked, since readFromConfig() clears it right
      // after acting on it.
      top[FPSTR(_restoreDefaults)] = restoreDefaults;
    }

    bool readFromConfig(JsonObject& root) override {
      int8_t oldPin       = pin;
      bool   oldActiveLow = activeLow;

      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, true);
      configComplete &= getJsonValue(top[FPSTR(_pin)], pin, -1);
      configComplete &= getJsonValue(top[FPSTR(_activeLow)], activeLow, true);
      configComplete &= getJsonValue(top[FPSTR(_debounceMs)], debounceMs, 30);
      configComplete &= getJsonValue(top[FPSTR(_gapTimeoutMs)], gapTimeoutMs, 600);
      configComplete &= getJsonValue(top[FPSTR(_widthToleranceMs)], widthToleranceMs, 60);
      configComplete &= getJsonValue(top[FPSTR(_restoreDefaults)], restoreDefaults, false);

      JsonArray counts  = top[FPSTR(_patCounts)];
      JsonArray widths  = top[FPSTR(_patWidthsMs)];
      JsonArray presets = top[FPSTR(_patPresets)];
      for (uint8_t i = 0; i < PULSE_PRESETS_MAX_PATTERNS; i++) {
        patCounts[i]   = (!counts.isNull()  && i < counts.size())  ? (uint8_t)(counts[i]   | patCounts[i])  : patCounts[i];
        patWidthsMs[i] = (!widths.isNull()  && i < widths.size())  ? (uint16_t)(widths[i]  | patWidthsMs[i]) : patWidthsMs[i];
        patPresets[i]  = (!presets.isNull() && i < presets.size()) ? (uint8_t)(presets[i] | patPresets[i]) : patPresets[i];

        // Only a slot that's actually in use (count != 0) gets its preset id
        // clamped - preset 0 doesn't exist and preset 255 is WLED's special
        // "temporary preset" id, so an unclamped typo here would silently do
        // something other than what was configured.
        if (patCounts[i] != 0) patPresets[i] = (uint8_t)max(1, min(250, (int)patPresets[i]));
      }
      configComplete &= !(counts.isNull()  || counts.size()  != PULSE_PRESETS_MAX_PATTERNS);
      configComplete &= !(widths.isNull()  || widths.size()  != PULSE_PRESETS_MAX_PATTERNS);
      configComplete &= !(presets.isNull() || presets.size() != PULSE_PRESETS_MAX_PATTERNS);

      // "restoreDefaults" checkbox was ticked and saved - override everything
      // just read above with defaults, then clear the flag so it doesn't keep
      // resetting on every subsequent save.
      if (restoreDefaults) {
        enabled          = PULSE_PRESETS_DEFAULT_ENABLED;
        pin              = PULSE_PRESETS_DEFAULT_PIN;
        activeLow        = PULSE_PRESETS_DEFAULT_ACTIVE_LOW;
        debounceMs       = PULSE_PRESETS_DEFAULT_DEBOUNCE_MS;
        gapTimeoutMs     = PULSE_PRESETS_DEFAULT_GAP_TIMEOUT_MS;
        widthToleranceMs = PULSE_PRESETS_DEFAULT_WIDTH_TOLERANCE_MS;
        for (uint8_t i = 0; i < PULSE_PRESETS_MAX_PATTERNS; i++) {
          patCounts[i]   = 0;
          patWidthsMs[i] = 0;
          patPresets[i]  = 0;
        }
        restoreDefaults = false;
      }

      if (!initDone) {
        // first load, prior to setup() - setup() will configure the pin
      } else if (pin != oldPin || activeLow != oldActiveLow) {
        // pin or its polarity was changed live from the Usermod Settings page -
        // reconfigure now so pinMode() actually matches the new activeLow setting
        if (oldPin >= 0) PinManager::deallocatePin(oldPin, PinOwner::UM_Unspecified);
        configurePin();
      }

      return configComplete;
    }

    void appendConfigData() override {
      oappend(F("addInfo('pulse_presets:pin',1,'GPIO from the PLC relay dry contact');"));
      oappend(F("addInfo('pulse_presets:activeLow',1,'on = contact shorts pin to GND when active (use INPUT_PULLUP)');"));
      oappend(F("addInfo('pulse_presets:gapTimeoutMs',1,'silence on the line (ms) that closes a burst');"));
      oappend(F("addInfo('pulse_presets:widthToleranceMs',1,'+/- ms allowed when matching a pulse width');"));
      oappend(F("addInfo('pulse_presets:restoreDefaults',1,'check this and hit Save to reset ALL settings on this page back to defaults');"));
      for (uint8_t i = 0; i < PULSE_PRESETS_MAX_PATTERNS; i++) {
        // NOTE: this buffer must comfortably fit the longest of the three
        // formatted strings below (currently ~110 chars incl. terminator).
        // snprintf_P silently TRUNCATES anything longer instead of erroring -
        // a truncated line loses its closing ');' and breaks JS parsing for
        // the *entire* settings page script (every usermod's panel, and the
        // pin-dropdown feature), not just this one line. If you lengthen any
        // of these strings later, bump this size and recheck.
        char str[160];
        // txt2 (4th arg) prints BEFORE the field as "Pattern N: " so the three
        // fields belonging to one pattern (count/width/preset) are visually
        // tied together; txt (3rd arg) prints after the field as a helper.
        snprintf_P(str, sizeof(str), PSTR("addInfo('pulse_presets:patCounts[]',%u,'pulses in this burst (0 = disabled)','Pattern %u: ');"), i, i+1);
        oappend(str);
        snprintf_P(str, sizeof(str), PSTR("addInfo('pulse_presets:patWidthsMs[]',%u,'expected average width, ms - or pick a preset below','Pattern %u: ');"), i, i+1);
        oappend(str);
        snprintf_P(str, sizeof(str), PSTR("addInfo('pulse_presets:patPresets[]',%u,'WLED preset to fire when this pattern matches (1-250)','Pattern %u: ');"), i, i+1);
        oappend(str);
      }

      // Add a dropdown of common PLC pulse-duration presets next to each of the
      // 6 "width" number fields above (200-750ms, 1-45s - matched to typical PLC
      // pulse-output timer options; widths this usermod matches are always a
      // fraction of a second to a handful of seconds, so nothing longer is
      // offered). Picking an option writes its ms value straight into the
      // existing number field; picking "custom" leaves that field alone so any
      // raw ms value can still be typed in directly - the field itself is never
      // replaced or restricted, this only adds a shortcut next to it.
      oappend(F(
        "(function(){"
        "var P=[[200,'200ms'],[300,'300ms'],[400,'400ms'],[500,'500ms'],[750,'750ms'],"
        "[1000,'1s'],[2000,'2s'],[3000,'3s'],[4000,'4s'],[5000,'5s'],[10000,'10s'],"
        "[15000,'15s'],[20000,'20s'],[30000,'30s'],[45000,'45s']];"
        "var a=d.getElementsByName('pulse_presets:patWidthsMs[]');"
        "for(var i=0;i<a.length;i++){"
        "var n=a[i];if(!n||n.tagName!=='INPUT')continue;"
        "var s=cE('select');s.style.marginLeft='6px';"
        "var v=parseInt(n.value)||0,m=false;"
        "var o=cE('option');o.value='c';o.text='custom';s.appendChild(o);"
        "for(var j=0;j<P.length;j++){"
        "var p=cE('option');p.value=P[j][0];p.text=P[j][1];s.appendChild(p);"
        "if(P[j][0]==v){p.selected=true;m=true;}"
        "}"
        "if(!m)s.options[0].selected=true;"
        "s.onchange=function(){if(this.value!=='c')this.previousElementSibling.value=this.value;};"
        "n.insertAdjacentElement('afterend',s);"
        "}"
        "})();"
      ));
    }
};

const char UsermodPulsePresets::_name[]             PROGMEM = "pulse_presets";
const char UsermodPulsePresets::_enabled[]          PROGMEM = "enabled";
const char UsermodPulsePresets::_pin[]              PROGMEM = "pin";
const char UsermodPulsePresets::_activeLow[]        PROGMEM = "activeLow";
const char UsermodPulsePresets::_debounceMs[]       PROGMEM = "debounceMs";
const char UsermodPulsePresets::_gapTimeoutMs[]     PROGMEM = "gapTimeoutMs";
const char UsermodPulsePresets::_widthToleranceMs[] PROGMEM = "widthToleranceMs";
const char UsermodPulsePresets::_restoreDefaults[]  PROGMEM = "restoreDefaults";
const char UsermodPulsePresets::_patCounts[]        PROGMEM = "patCounts";
const char UsermodPulsePresets::_patWidthsMs[]      PROGMEM = "patWidthsMs";
const char UsermodPulsePresets::_patPresets[]       PROGMEM = "patPresets";
const char UsermodPulsePresets::_infoPreset[]       PROGMEM = "pulse_presets: preset";
const char UsermodPulsePresets::_infoStats[]        PROGMEM = "pulse_presets: stats";

static UsermodPulsePresets pulse_presets;
REGISTER_USERMOD(pulse_presets);
