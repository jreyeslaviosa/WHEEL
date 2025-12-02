/*******************************************************
 * Encoder + Selector + OSC + Web UI (No OTA, No JSON, No Scan)
 * - Web UI protected by Basic-Auth
 * - SSID dropdown via saved suggestions (no active scan)
 * - Wi-Fi config via web UI; STA with AP fallback; mDNS
 * - POST /reboot (protected)
 * - Selector: sliding window + hysteresis (no clamp)
 * - Increment: free-running (never clamped)
 ********************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ESP32Encoder.h>

#include <OSCMessage.h>
#include <OSCBundle.h>

#include <Adafruit_NeoPixel.h>

// =================== Build Info =======================
static const char* kAppName    = "EncoderNode";
static const char* kAppVersion = "1.6.3-no-scan";

// ==================== Pins & HW =======================
static const uint8_t PIN_NEOPIXEL = 4;     // status pixel
static const uint8_t PIN_ENC_CLK  = 14;    // encoder B
static const uint8_t PIN_ENC_DT   = 27;    // encoder A
static const uint8_t PIN_BUTTON   = 13;    // pushbutton to GND

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ==================== Networking ======================
static const uint16_t kOscOutPort = 4210;
static const uint16_t kOscInPort  = 4211;  // not used, but opened
static const char*    kMDNSName   = "encoder";

WebServer server(80);
WiFiUDP   udp;

// ======================== NVS =========================
Preferences prefs;

// ======================= Config =======================
struct Config {
  // Wi-Fi (STA) credentials
  String ssid      = "TP-Link_CEC8";
  String password  = "42338239";

  // SSID suggestions (for dropdown)
  String ssidList  = "TP-Link_CEC8, Encoder-AP";

  // Web auth
  String webUser   = "admin";
  String webPass   = "encoder";

  // OSC target
  String laptopIp  = "192.168.0.44";

  // Selector
  int32_t selectorStepCounts = 1000;  // counts per selector index
  int32_t selectorMax        = 5;     // positions: 0..(N-1)
  bool    selectorMode       = true;  // enable selector mapping
  bool    useHysteresis      = true;  // anti-jitter

  // Increment (free-running)
  int32_t incrementStepCounts = 100;  // counts per increment

  // Optional floats (compat)
  float   tdIncrement = 0.010f;
  float   tdFilter    = 1.000f;

  void load() {
    prefs.begin("config", false);
    ssid                  = prefs.getString("wifiSsid", ssid);
    password              = prefs.getString("wifiPass", password);
    ssidList              = prefs.getString("ssidList", ssidList);
    webUser               = prefs.getString("webUser", webUser);
    webPass               = prefs.getString("webPass", webPass);
    laptopIp              = prefs.getString("laptopIp", laptopIp);
    selectorStepCounts    = prefs.getInt("selStep", selectorStepCounts);
    selectorMax           = prefs.getInt("selMax", selectorMax);
    selectorMode          = prefs.getBool("selMode", selectorMode);
    useHysteresis         = prefs.getBool("hyst",  useHysteresis);
    incrementStepCounts   = prefs.getInt("stepSize", incrementStepCounts);
    tdIncrement           = prefs.getFloat("tdInc", tdIncrement);
    tdFilter              = prefs.getFloat("tdFilt", tdFilter);
  }
  void save() {
    prefs.putString("wifiSsid", ssid);
    prefs.putString("wifiPass", password);
    prefs.putString("ssidList", ssidList);
    prefs.putString("webUser",  webUser);
    prefs.putString("webPass",  webPass);
    prefs.putString("laptopIp", laptopIp);
    prefs.putInt("selStep",     selectorStepCounts);
    prefs.putInt("selMax",      selectorMax);
    prefs.putBool("selMode",    selectorMode);
    prefs.putBool("hyst",       useHysteresis);
    prefs.putInt("stepSize",    incrementStepCounts);
    prefs.putFloat("tdInc",     tdIncrement);
    prefs.putFloat("tdFilt",    tdFilter);
  }
} CFG;

// ======================== State =======================
struct State {
  // Encoder
  ESP32Encoder enc;
  long         rawCount = 0;

  // Selector sliding window
  long         selBaseCount = 0;  // raw count that maps to index 0
  int          selIndex     = 0;  // 0..(CFG.selectorMax-1)

  // Free-running increment
  long         incResid     = 0;
  long         lastRawForInc = 0;
  int          incValue     = 0;

  // Button (debounce)
  int          buttonLast   = HIGH;
  uint32_t     buttonLastMs = 0;

  // Network
  bool         udpReady     = false;
} ST;

// =================== Small Utilities ==================
static inline bool toBool(const String& s) {
  String t = s; t.toLowerCase();
  return (t == "1" || t == "true" || t == "on" || t == "yes");
}
void setStatusPixel(uint8_t r, uint8_t g, uint8_t b) {
  static uint8_t lr=255, lg=255, lb=255;
  if (r==lr && g==lg && b==lb) return; // update only on change
  lr=r; lg=g; lb=b;
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.setBrightness(50);
  pixel.show();
}
void sendOSC(const char* path, int32_t v) {
  if (!ST.udpReady) return;
  OSCBundle b; b.add(path).add((int32_t)v);
  udp.beginPacket(CFG.laptopIp.c_str(), kOscOutPort);
  b.send(udp);
  udp.endPacket();
}
void sendOSC(const char* path, double v) {
  if (!ST.udpReady) return;
  OSCBundle b; b.add(path).add((double)v);
  udp.beginPacket(CFG.laptopIp.c_str(), kOscOutPort);
  b.send(udp);
  udp.endPacket();
}
// Button debounce (edge detect; sets outVal)
bool readButtonDebounced(uint8_t pin, int& outVal, uint16_t debounceMs = 12) {
  int val = digitalRead(pin);
  uint32_t now = millis();
  if (val != ST.buttonLast) {
    if (now - ST.buttonLastMs >= debounceMs) {
      ST.buttonLast = val;
      ST.buttonLastMs = now;
      outVal = val;
      return true;
    }
  }
  outVal = ST.buttonLast;
  return false;
}

// ================= Selector & Increment =================
void selectorUpdate(long raw) {
  if (!CFG.selectorMode) return;
  if (CFG.selectorMax < 2) CFG.selectorMax = 2;
  const long step = (long)CFG.selectorStepCounts;

  long minC = ST.selBaseCount;
  long maxC = ST.selBaseCount + (CFG.selectorMax - 1) * step;

  // Slide window if we pushed beyond ends (no clamping of encoder)
  if (raw > maxC) {
    ST.selBaseCount += (raw - maxC);
    minC = ST.selBaseCount;
    maxC = ST.selBaseCount + (CFG.selectorMax - 1) * step;
  } else if (raw < minC) {
    ST.selBaseCount += (raw - minC);
    minC = ST.selBaseCount;
    maxC = ST.selBaseCount + (CFG.selectorMax - 1) * step;
  }

  const long HYST = CFG.useHysteresis ? max(step / 6, 8L) : 0L;

  // Decision space: clamp virtual pos to window
  long vpos = raw;
  if (vpos < minC) vpos = minC;
  if (vpos > maxC) vpos = maxC;

  int nextIdx = ST.selIndex;

  long currLow  = minC + ST.selIndex * step;
  long currHigh = currLow + step;

  if (ST.selIndex < (CFG.selectorMax - 1)) {
    if (vpos >= (currHigh + HYST)) nextIdx = ST.selIndex + 1;
  }
  if (ST.selIndex > 0) {
    if (vpos <= (currLow - HYST)) nextIdx = ST.selIndex - 1;
  }
  if (vpos >= (maxC - HYST)) nextIdx = CFG.selectorMax - 1;
  if (vpos <= (minC + HYST)) nextIdx = 0;

  if (nextIdx != ST.selIndex) {
    ST.selIndex = nextIdx;
    sendOSC("/selector", (int32_t)ST.selIndex);
    Serial.printf("[SEL] index=%d\n", ST.selIndex);
  }
}

void incrementUpdate(long raw) {
  long d = raw - ST.lastRawForInc;
  ST.lastRawForInc = raw;

  ST.incResid += d;

  const long step = (long)CFG.incrementStepCounts;
  if (step <= 0) return;

  if (ST.incResid >= step) {
    int chunks = ST.incResid / step;
    for (int i = 0; i < chunks; ++i) {
      ++ST.incValue;
      sendOSC("/increment", (double)ST.incValue);
    }
    ST.incResid -= (long)chunks * step;
  } else if (ST.incResid <= -step) {
    int chunks = (-ST.incResid) / step;
    for (int i = 0; i < chunks; ++i) {
      --ST.incValue;
      sendOSC("/increment", (double)ST.incValue);
    }
    ST.incResid += (long)chunks * step;
  }
}

// =================== Auth helper =======================
bool ensureAuth() {
  if (!server.authenticate(CFG.webUser.c_str(), CFG.webPass.c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// =================== Web UI: HTML ======================
String htmlHeader(const String& title) {
  String h;
  h.reserve(512);
  h += "<!doctype html><html><head><meta charset='utf-8'><title>";
  h += title;
  h += "</title><style>body{font-family:system-ui;margin:24px;max-width:760px}";
  h += "input,select,button{padding:6px;margin:6px 0;box-sizing:border-box}";
  h += "input,select{width:100%}.row{margin:12px 0}.btn{padding:8px 14px}";
  h += ".ok{color:green}.warn{color:#b00}.muted{color:#666}</style>";
  h += "</head><body>";
  h += "<h1>";
  h += kAppName; h += " <small style='font-weight:400;color:#666'>v"; h += kAppVersion; h += "</small></h1>";
  return h;
}
String htmlFooter() {
  return String("<p class='muted' style='margin-top:16px'><a href=\"/\">Back</a></p></body></html>");
}

// Build datalist options from CFG.ssidList (comma-separated)
String buildSSIDDataListOptions() {
  String out;
  String list = CFG.ssidList;
  list.trim();
  if (list.length() == 0) return out;

  int start = 0;
  while (start < (int)list.length()) {
    int comma = list.indexOf(',', start);
    String item = (comma == -1) ? list.substring(start) : list.substring(start, comma);
    item.trim();
    if (item.length() > 0) {
      // escape HTML quotes minimally
      item.replace("\"", "&quot;");
      out += "<option value=\"" + item + "\">";
    }
    if (comma == -1) break;
    start = comma + 1;
  }
  return out;
}

void handleRoot() {
  if (!ensureAuth()) return;
  String ip = (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED)
              ? WiFi.localIP().toString() : WiFi.softAPIP().toString();

  String s = htmlHeader("Status");
  s += "<p><b>Device IP:</b> " + ip + "</p>";
  s += "<p><b>OSC target:</b> " + CFG.laptopIp + ":" + String(kOscOutPort) + "</p>";
  s += "<p><b>Selector:</b> index <code>" + String(ST.selIndex) + "</code> of <code>" + String(CFG.selectorMax) + "</code></p>";
  s += "<p><b>Selector mode:</b> " + String(CFG.selectorMode ? "ON" : "OFF") + "</p>";
  s += "<p><b>Increment:</b> value <code>" + String(ST.incValue) + "</code></p>";
  s += "<p><a href=\"/config\">/config</a></p>";
  s += htmlFooter();
  server.send(200, "text/html", s);
}

void handleConfig() {
  if (!ensureAuth()) return;

  bool saved = false;
  bool wifiAction = false;

  // --- Apply updates (GET form) ---
  if (server.hasArg("laptopIp"))     { CFG.laptopIp = server.arg("laptopIp"); saved = true; }
  if (server.hasArg("stepSize"))     { int v=server.arg("stepSize").toInt(); if (v>0&&v<=200000) CFG.incrementStepCounts=v, saved=true; }
  if (server.hasArg("selStep"))      { int v=server.arg("selStep").toInt();  if (v>=1&&v<=1000000) CFG.selectorStepCounts=v, saved=true; }
  if (server.hasArg("selMax"))       { int v=server.arg("selMax").toInt();   if (v>=2&&v<=128) CFG.selectorMax=v, saved=true; }
  if (server.hasArg("selMode"))      { CFG.selectorMode = toBool(server.arg("selMode")); saved=true; }
  if (server.hasArg("hyst"))         { CFG.useHysteresis = toBool(server.arg("hyst")); saved=true; }
  if (server.hasArg("tdInc"))        { CFG.tdIncrement = server.arg("tdInc").toFloat(); saved=true; }
  if (server.hasArg("tdFilter"))     { CFG.tdFilter    = server.arg("tdFilter").toFloat(); saved=true; }

  // Web auth updates
  if (server.hasArg("webUser"))      { CFG.webUser = server.arg("webUser"); saved = true; }
  if (server.hasArg("webPass"))      { CFG.webPass = server.arg("webPass"); saved = true; }

  // Wi-Fi creds (and optional connect now)
  if (server.hasArg("ssid"))         { CFG.ssid = server.arg("ssid"); saved = true; }
  if (server.hasArg("wifiPass"))     { CFG.password = server.arg("wifiPass"); saved = true; }
  if (server.hasArg("connectNow"))   { wifiAction = true; }

  // SSID suggestions list
  if (server.hasArg("ssidList"))     { CFG.ssidList = server.arg("ssidList"); saved = true; }

  // Actions
  if (server.hasArg("action")) {
    long raw = ST.rawCount;
    if (server.arg("action") == "reanchor") {
      ST.selBaseCount = raw; ST.selIndex = 0; saved = true;
    } else if (server.arg("action") == "resetInc") {
      ST.lastRawForInc = raw; ST.incResid = 0; ST.incValue = 0; saved = true;
    }
  }

  if (saved) CFG.save();

  // Handle Wi-Fi connect request
  String wifiMsg;
  if (wifiAction) {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(CFG.ssid.c_str(), CFG.password.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      udp.begin(kOscInPort);
      ST.udpReady = true;
      wifiMsg = "<p class='ok'>Connected as " + WiFi.localIP().toString() + ".</p>";
      setStatusPixel(0,80,0); // green
    } else {
      // fall back to AP so the UI stays reachable
      ST.udpReady = false;
      WiFi.mode(WIFI_AP);
      const char* AP_SSID = "Encoder-AP";
      const char* AP_PASS = "12345678";
      WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
      wifiMsg = "<p class='warn'>STA connect failed. AP is running (SSID: Encoder-AP, pass: 12345678)</p>";
      setStatusPixel(80,40,0); // amber/red
    }
  }

  // --- Page render ---
  String s = htmlHeader("Config");
  if (saved)   s += "<p class='ok'>Saved.</p>";
  if (wifiMsg.length()) s += wifiMsg;

  // Main config form
  s += "<form action='/config' method='get'>";

  s += "<h3>Web Login</h3>";
  s += "<div class='row'>Username:<input name='webUser' type='text' value='" + CFG.webUser + "'></div>";
  s += "<div class='row'>Password:<input name='webPass' type='text' value='" + CFG.webPass + "'></div>";

  s += "<h3>Wi-Fi (STA) Credentials</h3>";
  // datalist-driven SSID dropdown (suggestions)
  s += "<datalist id='ssidSuggestions'>" + buildSSIDDataListOptions() + "</datalist>";
  s += "<div class='row'>SSID:<input list='ssidSuggestions' name='ssid' type='text' value='" + CFG.ssid + "' placeholder='Pick from list or type manually'></div>";
  s += "<div class='row'>Password:<input name='wifiPass' type='text' value='" + CFG.password + "'></div>";
  s += "<div class='row'><label><input type='checkbox' name='connectNow' value='1'> Save & Connect now</label></div>";

  s += "<div class='row'>Known SSIDs (comma-separated):";
  s += "<input name='ssidList' type='text' value='" + CFG.ssidList + "'>";
  s += "<div class='muted'>These populate the SSID dropdown above. Add/remove as needed.</div></div>";

  s += "<h3>OSC</h3>";
  s += "<div class='row'>Laptop IP (OSC target):<input name='laptopIp' type='text' value='" + CFG.laptopIp + "'></div>";

  s += "<h3>/increment</h3>";
  s += "<div class='row'>stepSize (counts):<input name='stepSize' type='number' min='1' max='200000' value='" + String(CFG.incrementStepCounts) + "'></div>";

  s += "<h3>Selector</h3>";
  s += "<div class='row'>selector_space_step (counts/index):<input name='selStep' type='number' min='1' max='1000000' value='" + String(CFG.selectorStepCounts) + "'></div>";
  s += "<div class='row'>selectorMax (positions, 2..128):<input name='selMax' type='number' min='2' max='128' value='" + String(CFG.selectorMax) + "'></div>";
  s += "<div class='row'>selectorMode:<select name='selMode'><option value='true' ";
  s += (CFG.selectorMode ? "selected" : ""); s += ">ON</option><option value='false' ";
  s += (!CFG.selectorMode ? "selected" : ""); s += ">OFF</option></select></div>";
  s += "<div class='row'>hysteresis:<select name='hyst'><option value='true' ";
  s += (CFG.useHysteresis ? "selected" : ""); s += ">ON</option><option value='false' ";
  s += (!CFG.useHysteresis ? "selected" : ""); s += ">OFF</option></select></div>";

  s += "<h3>Actions</h3>";
  s += "<div class='row'><button class='btn' onclick=\"this.form.action='/config?action=reanchor';this.form.submit();return false;\">Re-anchor selector (current → index 0)</button></div>";
  s += "<div class='row'><button class='btn' onclick=\"this.form.action='/config?action=resetInc';this.form.submit();return false;\">Reset /increment</button></div>";

  s += "<div class='row'><input class='btn' type='submit' value='Save'></div>";
  s += "</form>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

// ==================== /reboot (POST) ==================
void handleReboot() {
  if (!ensureAuth()) return;
  server.send(200, "text/plain", "Rebooting…");
  server.client().flush();
  delay(100);
  ESP.restart();
}

// =================== WiFi / mDNS / HTTP ===============
void bringUpNetwork() {
  Serial.printf("Wi-Fi STA connecting to \"%s\"…\n", CFG.ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(CFG.ssid.c_str(), CFG.password.c_str());

  const uint32_t kTimeoutMs = 12000;
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < kTimeoutMs) {
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) {
    ST.udpReady = true;
    Serial.print("✅ STA connected: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin(kMDNSName)) {
      Serial.println("mDNS: http://encoder.local/");
    }
    udp.begin(kOscInPort);
    setStatusPixel(0, 80, 0); // green
  } else {
    ST.udpReady = false;
    Serial.println("❌ STA failed → starting AP…");
    WiFi.mode(WIFI_AP);
    const char* AP_SSID = "Encoder-AP";
    const char* AP_PASS = "12345678";
    WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
    delay(200);
    Serial.printf("📶 AP started (%s / %s). IP: %s\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
    MDNS.begin(kMDNSName);
    setStatusPixel(80, 40, 0); // amber
  }

  // Routes (all protected in handlers)
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/reboot", HTTP_POST, handleReboot);

  server.begin();
  Serial.println("HTTP server on :80");
}

// ====================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);
  delay(150);

  pixel.begin();
  setStatusPixel(40, 40, 0); // amber at boot

  CFG.load();
  Serial.printf("Boot %s v%s\n", kAppName, kAppVersion);
  Serial.printf("Loaded: stepSize=%d selStep=%d selMax=%d selMode=%d hyst=%d laptopIp=%s web=%s/%s ssidList=[%s]\n",
    CFG.incrementStepCounts, CFG.selectorStepCounts, CFG.selectorMax,
    CFG.selectorMode, CFG.useHysteresis, CFG.laptopIp.c_str(),
    CFG.webUser.c_str(), CFG.webPass.c_str(), CFG.ssidList.c_str());

  // Encoder & button
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  ST.buttonLast   = digitalRead(PIN_BUTTON);
  ST.buttonLastMs = millis();

  ST.enc.attachHalfQuad(PIN_ENC_DT, PIN_ENC_CLK);
  ST.enc.setCount(720 * 2); // arbitrary anchor
  delay(2);

  long raw0 = ST.enc.getCount();
  ST.selBaseCount  = raw0;  // selector index 0 anchored here
  ST.selIndex      = 0;
  ST.lastRawForInc = raw0;
  ST.incResid      = 0;
  ST.incValue      = 0;

  bringUpNetwork();
}

void loop() {
  server.handleClient();

  // Read encoder raw count
  ST.rawCount = ST.enc.getCount();

  // Selector + increment
  selectorUpdate(ST.rawCount);
  incrementUpdate(ST.rawCount);

  // Button (debounced). LOW = pressed (INPUT_PULLUP wiring)
  int btnVal;
  if (readButtonDebounced(PIN_BUTTON, btnVal)) {
    sendOSC("/button", (int32_t)btnVal);
    Serial.printf("[BTN] %s\n", (btnVal == LOW ? "PRESS" : "RELEASE"));
  }

  delay(5);
}
