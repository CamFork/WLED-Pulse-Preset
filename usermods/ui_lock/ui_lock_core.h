// ui_lock_core.h - pure helpers for the UiLock usermod (no Arduino/WLED includes; host-testable)
#pragma once
#include <stdint.h>
#include <string.h>

namespace uilock {

enum Access : uint8_t { OPEN = 0, PAGE, API };

inline bool startsWith(const char* s, const char* p) { return strncmp(s, p, strlen(p)) == 0; }

// What a locked-out browser may not reach.
//   PAGE: the main UI itself -> gets the login page
//   API : control endpoints the main UI drives -> 401
//   OPEN: everything else (settings pages keep their own PIN; read-only JSON they need stays open)
inline Access classify(const char* url, bool isPost, bool lockApi) {
  if (!strcmp(url, "/") || !strcmp(url, "/index.htm")) return isPost ? OPEN : PAGE;
  if (!lockApi) return OPEN;
  if (!strcmp(url, "/ws")) return API;                       // websocket: state + control
  if (startsWith(url, "/win")) return API;                   // legacy HTTP API
  if (startsWith(url, "/json")) {
    const char* r = url + 5;
    if (!*r || !strcmp(r, "/")) return API;                  // GET all / POST state
    if (isPost) return startsWith(r, "/cfg") ? OPEN : API;   // /json/cfg POST is checked by WLED's own PIN
    if (startsWith(r, "/state") || startsWith(r, "/si")) return API;
    return OPEN;                                             // info, eff, pal, fxdata, nodes, cfg, pins, net ...
  }
  return OPEN;
}

// Looks like a bare IPv4 address (used to spot captive-portal probes in AP mode)
inline bool looksLikeIp(const char* h) {
  int dots = 0; bool digit = false;
  for (; *h; h++) {
    if (*h == '.') { dots++; digit = false; }
    else if (*h >= '0' && *h <= '9') digit = true;
    else return false;
  }
  return dots == 3 && digit;
}

inline void toHex(uint64_t v, char out[17]) {
  static const char* hx = "0123456789abcdef";
  for (int i = 15; i >= 0; i--) { out[i] = hx[v & 0xF]; v >>= 4; }
  out[16] = 0;
}

// Find "name=<16 hex>" in a Cookie header. Returns true and the token value.
inline bool cookieToken(const char* cookie, const char* name, uint64_t& token) {
  size_t nl = strlen(name);
  const char* p = cookie;
  while (p && *p) {
    while (*p == ' ' || *p == ';') p++;
    if (!strncmp(p, name, nl) && p[nl] == '=') {
      const char* v = p + nl + 1;
      uint64_t t = 0; int n = 0;
      for (; n < 16; n++) {
        char c = v[n]; int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else break;
        t = (t << 4) | (uint64_t)d;
      }
      if (n == 16 && (v[16] == 0 || v[16] == ';' || v[16] == ' ')) { token = t; return true; }
      return false;
    }
    p = strchr(p, ';');
  }
  return false;
}

struct SessionTable {
  static const uint8_t N = 8;
  uint64_t tok[N] = {0};
  uint32_t seen[N] = {0};
  bool     used[N] = {false};

  // idleMs = 0: sessions last until reboot / password change
  bool check(uint64_t t, uint32_t now, uint32_t idleMs) {
    if (!t) return false;
    for (uint8_t i = 0; i < N; i++) {
      if (!used[i] || tok[i] != t) continue;
      if (idleMs && (uint32_t)(now - seen[i]) > idleMs) { used[i] = false; return false; }
      seen[i] = now;
      return true;
    }
    return false;
  }
  void add(uint64_t t, uint32_t now) {
    uint8_t slot = 0; uint32_t oldest = 0;
    for (uint8_t i = 0; i < N; i++) {
      if (!used[i]) { slot = i; break; }
      uint32_t age = now - seen[i];
      if (age >= oldest) { oldest = age; slot = i; }   // table full: replace the least recently used
    }
    tok[slot] = t; seen[slot] = now; used[slot] = true;
  }
  void remove(uint64_t t) { for (uint8_t i = 0; i < N; i++) if (used[i] && tok[i] == t) used[i] = false; }
  void clear() { for (uint8_t i = 0; i < N; i++) used[i] = false; }
  uint8_t count(uint32_t now, uint32_t idleMs) const {
    uint8_t c = 0;
    for (uint8_t i = 0; i < N; i++) if (used[i] && (!idleMs || (uint32_t)(now - seen[i]) <= idleMs)) c++;
    return c;
  }
};

} // namespace uilock
