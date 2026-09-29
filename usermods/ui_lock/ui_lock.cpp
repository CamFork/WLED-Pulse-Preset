// ui_lock.cpp - UiLock usermod for WLED v16: a login page in front of the main UI.
//
// Rudimentary protection by design: plain HTTP, password sent unencrypted, session cookie
// can be sniffed on the network. It stops casual access to the sign's controls, nothing more.
//
// How it works
//  - setup() registers a web handler. WLED calls UsermodManager::setup() before initServer(),
//    so this handler is asked first about every request. It only claims a request when the
//    browser is NOT logged in and the URL is protected; otherwise WLED handles it as normal.
//  - Protected: the main page ("/"), and (lockApi, default on) the control API the main page
//    drives: /json (all + state + si), /win, /ws. Read-only JSON (info, effects, palettes,
//    cfg, pins, net...) stays open so the settings pages still work; they keep WLED's own PIN.
//  - Login = POST /login -> per-browser session cookie. Logout = GET /logout.
//    Sessions end after idleMinutes without a page/API request, on reboot, on a password
//    change, or with "logoutAll".
//  - The password is stored in /uilock_sec.txt, not in cfg.json: WLED serves /json/cfg to
//    anyone, but never serves a file whose name contains "sec".
#include "wled.h"
#include "ui_lock_core.h"
#ifdef ARDUINO_ARCH_ESP32
  #include "esp_system.h"
#endif

using namespace uilock;

static const char _ul_name[]   PROGMEM = "UiLock";
static const char _ul_file[]   PROGMEM = "/uilock_sec.txt";
static const char _ul_cookie[] PROGMEM = "uilock";

#define UILOCK_PW_MAX 32
#define UILOCK_RETRY_MS 3000

class UiLockUsermod;

class UiLockGate : public AsyncWebHandler {
  UiLockUsermod* um;
public:
  explicit UiLockGate(UiLockUsermod* u) : um(u) {}
  bool canHandle(AsyncWebServerRequest* request) override;
  void handleRequest(AsyncWebServerRequest* request) override;
};

class UiLockUsermod : public Usermod {
  friend class UiLockGate;

  // settings
  bool     enabled           = true;
  bool     lockApi           = true;
  uint16_t idleMinutes       = 60;
  bool     acceptSettingsPin = false;

  // runtime
  char     password[UILOCK_PW_MAX + 1] = "";
  bool     saveNeeded  = false;
  bool     initDone    = false;
  uint32_t lastFailMs  = 0;
  bool     failPending = false;
  uint32_t failCount   = 0;
  uint32_t loginCount  = 0;
  SessionTable sessions;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  uint32_t idleMs() const { return (uint32_t)idleMinutes * 60000UL; }

  bool sessionValid(AsyncWebServerRequest* request) {
    AsyncWebHeader* h = request->getHeader(F("Cookie"));
    if (!h) return false;
    uint64_t t;
    if (!cookieToken(h->value().c_str(), _ul_cookie, t)) return false;
    portENTER_CRITICAL(&mux);
    bool ok = sessions.check(t, millis(), idleMs());
    portEXIT_CRITICAL(&mux);
    return ok;
  }

  static uint64_t newToken() {
    uint64_t t = ((uint64_t)esp_random() << 32) | (uint64_t)esp_random();
    return t ? t : 1;
  }

  void loadPassword() {
    password[0] = 0;
    File f = WLED_FS.open(FPSTR(_ul_file), "r");
    if (!f) return;
    size_t n = f.readBytes(password, UILOCK_PW_MAX);
    password[n] = 0;
    f.close();
  }

  void savePassword() {
    if (!password[0]) { WLED_FS.remove(FPSTR(_ul_file)); return; }
    File f = WLED_FS.open(FPSTR(_ul_file), "w");
    if (!f) return;
    f.print(password);
    f.close();
  }

  void sendLoginPage(AsyncWebServerRequest* request, int code, const __FlashStringHelper* msg) {
    String p;
    p.reserve(1400);
    p += F("<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
           "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>");
    p += serverDescription;
    p += F(" - login</title><style>"
           "body{background:#111;color:#fff;font-family:Verdana,Helvetica,sans-serif;margin:0;"
           "display:flex;align-items:center;justify-content:center;min-height:100vh}"
           ".c{background:#222;padding:24px 28px;border-radius:16px;width:280px;text-align:center}"
           "h2{margin:0 0 6px;font-size:20px}p{color:#aaa;font-size:13px;margin:4px 0 16px}"
           "input{width:100%;box-sizing:border-box;padding:10px;border-radius:10px;border:0;"
           "background:#333;color:#fff;font-size:16px;margin-bottom:12px}"
           "button{width:100%;padding:10px;border:0;border-radius:10px;background:#444;color:#fff;"
           "font-size:16px;cursor:pointer}button:hover{background:#555}"
           ".e{color:#f66;min-height:18px;font-size:13px;margin:0 0 8px}"
           "a{color:#888;font-size:12px;display:block;margin-top:14px}"
           "</style></head><body><form class=\"c\" method=\"post\" action=\"/login\">"
           "<h2>");
    p += serverDescription;
    p += F("</h2><p>Enter the password to use this sign's controls</p><div class=\"e\">");
    if (msg) p += msg;
    p += F("</div><input type=\"password\" name=\"pw\" maxlength=\"32\" autocomplete=\"current-password\" autofocus>"
           "<button type=\"submit\">Log in</button>"
           "<a href=\"/settings\">Settings (PIN protected)</a></form></body></html>");
    AsyncWebServerResponse* r = request->beginResponse(code, F("text/html"), p);
    r->addHeader(F("Cache-Control"), F("no-store"));
    request->send(r);
  }

  void handleLogin(AsyncWebServerRequest* request) {
    if (!active()) { redirectHome(request, nullptr); return; }
    uint32_t now = millis();
    if (failPending && (uint32_t)(now - lastFailMs) < UILOCK_RETRY_MS) {
      sendLoginPage(request, 429, F("Too many attempts. Wait a few seconds."));
      return;
    }
    const String& pw = request->arg(F("pw"));
    bool ok = pw.length() > 0 && strcmp(pw.c_str(), password) == 0;
    if (!ok && acceptSettingsPin && strlen(settingsPIN) > 0 && pw.length() == 4)
      ok = strncmp(settingsPIN, pw.c_str(), 4) == 0;
    if (!ok) {
      failPending = true; lastFailMs = now; failCount++;
      sendLoginPage(request, 401, F("Incorrect password."));
      return;
    }
    failPending = false;
    loginCount++;
    uint64_t t = newToken();
    portENTER_CRITICAL(&mux);
    sessions.add(t, now);
    portEXIT_CRITICAL(&mux);
    char hex[17]; toHex(t, hex);
    String c = FPSTR(_ul_cookie); c += '='; c += hex; c += F("; Path=/; HttpOnly; SameSite=Strict");
    redirectHome(request, c.c_str());
  }

  void handleLogout(AsyncWebServerRequest* request) {
    AsyncWebHeader* h = request->getHeader(F("Cookie"));
    uint64_t t;
    if (h && cookieToken(h->value().c_str(), _ul_cookie, t)) {
      portENTER_CRITICAL(&mux);
      sessions.remove(t);
      portEXIT_CRITICAL(&mux);
    }
    String c = FPSTR(_ul_cookie); c += F("=; Path=/; Max-Age=0; SameSite=Strict");
    redirectHome(request, c.c_str());
  }

  static void redirectHome(AsyncWebServerRequest* request, const char* setCookie) {
    AsyncWebServerResponse* r = request->beginResponse(302);
    r->addHeader(F("Location"), F("/"));
    r->addHeader(F("Cache-Control"), F("no-store"));
    if (setCookie) r->addHeader(F("Set-Cookie"), setCookie);
    request->send(r);
  }

public:
  bool active() const { return enabled && password[0]; }

  void setup() override {
    loadPassword();
    server.addHandler(new UiLockGate(this));   // registered before WLED's own handlers (see header)
    server.on(F("/login"), HTTP_POST, [this](AsyncWebServerRequest* r) { handleLogin(r); });
    server.on(F("/login"), HTTP_GET,  [](AsyncWebServerRequest* r) { redirectHome(r, nullptr); });
    server.on(F("/logout"), HTTP_GET, [this](AsyncWebServerRequest* r) { handleLogout(r); });
    initDone = true;
  }

  void loop() override {
    if (saveNeeded) { saveNeeded = false; savePassword(); }  // file write from loop, not the web task
  }

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray a = user.createNestedArray(F("UI lock"));
    if (!enabled)          { a.add(F("off")); return; }
    if (!password[0])      { a.add(F("off (no password set)")); return; }
    portENTER_CRITICAL(&mux);
    uint8_t n = sessions.count(millis(), idleMs());
    portEXIT_CRITICAL(&mux);
    String s = lockApi ? F("on (page + API), ") : F("on (page only), ");
    s += String(n); s += F(" logged in, ");
    s += String(failCount); s += F(" failed logins");
    a.add(s);
  }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_ul_name));
    top[F("enabled")]           = enabled;
    top[F("password")]          = "";          // never written to cfg.json (see header)
    top[F("lockApi")]           = lockApi;
    top[F("idleMinutes")]       = idleMinutes;
    top[F("acceptSettingsPin")] = acceptSettingsPin;
    top[F("logoutAll")]         = false;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_ul_name)];
    if (top.isNull()) return false;
    bool ok = true;
    ok &= getJsonValue(top[F("enabled")],           enabled,           true);
    ok &= getJsonValue(top[F("lockApi")],           lockApi,           true);
    ok &= getJsonValue(top[F("idleMinutes")],       idleMinutes,       60);
    ok &= getJsonValue(top[F("acceptSettingsPin")], acceptSettingsPin, false);
    if (idleMinutes > 10080) idleMinutes = 10080;   // max 1 week; 0 = until reboot

    bool dropSessions = false;
    const char* pw = top[F("password")] | "";
    if (pw[0]) {                                     // a new password was typed and saved
      strlcpy(password, pw, sizeof(password));
      saveNeeded = true;
      dropSessions = true;
    }
    bool logoutAll = false;
    getJsonValue(top[F("logoutAll")], logoutAll, false);
    if (logoutAll) dropSessions = true;
    if (dropSessions) {
      portENTER_CRITICAL(&mux);
      sessions.clear();
      portEXIT_CRITICAL(&mux);
    }
    return ok;
  }

  void appendConfigData(Print& s) override {
    // mask the password field and label the options
    s.print(F("(function(){var e=d.getElementsByName('UiLock:password');"
              "for(var i=0;i<e.length;i++)if(e[i].type=='text'){e[i].type='password';e[i].autocomplete='new-password';}})();"));
    s.print(F("addInfo('UiLock:password',1,'blank = keep current (max 32)');"));
    s.print(F("addInfo('UiLock:lockApi',1,'also block /json state, /win, /ws');"));
    s.print(F("addInfo('UiLock:idleMinutes',1,'min (0 = until reboot)');"));
    s.print(F("addInfo('UiLock:acceptSettingsPin',1,'settings PIN also logs in');"));
    s.print(F("addInfo('UiLock:logoutAll',1,'end every session on save');"));
  }

  uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }
};

bool UiLockGate::canHandle(AsyncWebServerRequest* request) {
  if (!um->initDone || !um->active()) return false;
  Access a = classify(request->url().c_str(), request->method() == HTTP_POST, um->lockApi);
  if (a == OPEN) return false;
  // AP mode: let WLED's captive portal redirect foreign hosts first (it lands back on "/").
  if (apActive && request->hasHeader(F("Host"))) {
    const String& host = request->getHeader(F("Host"))->value();
    if (!looksLikeIp(host.c_str()) && host.indexOf(F("wled.me")) < 0 &&
        host.indexOf(cmDNS) < 0 && host.indexOf(':') < 0) return false;
  }
  return !um->sessionValid(request);
}

void UiLockGate::handleRequest(AsyncWebServerRequest* request) {
  Access a = classify(request->url().c_str(), request->method() == HTTP_POST, um->lockApi);
  if (a == PAGE) { um->sendLoginPage(request, 200, nullptr); return; }
  AsyncWebServerResponse* r = request->beginResponse(401, F("application/json"), F("{\"error\":\"locked\"}"));
  r->addHeader(F("Cache-Control"), F("no-store"));
  request->send(r);
}

static UiLockUsermod ui_lock;
REGISTER_USERMOD(ui_lock);
