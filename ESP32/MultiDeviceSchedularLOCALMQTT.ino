/* Robust Weekly Timer - DS3231 + NTP fallback (PKT / UTC+5)
   Persistent STA + AP always on + mDNS + Static IP + MQTT remote control
   ESP32 Arduino core 2.0.x AND 3.x compatible
*/

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <sys/time.h>
#include <time.h>
#include <Wire.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ---------------- Pins & constants ----------------
#define RELAY_PIN 27
#define LED_PIN   2
#define RELAY_ACTIVE_LOW true

const char* AP_SSID = "YOUR_AP_SSID";
const char* AP_PASS = "YOUR_AP_PWD";
const char* MDNS_HOST = "yourlocaladdress"; // you can access the device from your router through http://yourlocaladdress.local", once router credentials are entered

const long gmtOffset_sec = 5 * 3600;
const int  daylightOffset_sec = 0;

const unsigned long NTP_RESYNC_INTERVAL   = 6UL * 3600UL * 1000UL;
const unsigned long WIFI_RETRY_INTERVAL   = 30000UL;
const unsigned long RTC_CHECK_INTERVAL    = 900000UL;
const unsigned long MQTT_RETRY_INTERVAL   = 5000UL;
const unsigned long MQTT_HEARTBEAT_MS     = 30000UL;

// ---------------- Globals ----------------
WebServer   server(80);
DNSServer   dnsServer;
RTC_DS3231  rtc;

char ntpSSID[32]     = "";
char ntpPassword[64] = "";

bool mdnsStarted = false;

enum StaState { STA_IDLE, STA_CONNECTING, STA_CONNECTED, STA_FAILED };
StaState staState = STA_IDLE;
unsigned long staConnectStart = 0;
const unsigned long STA_CONNECT_TIMEOUT = 12000UL;
unsigned long lastWifiRetry = 0;

unsigned long lastNtpSyncMillis = 0;
bool          ntpEverSynced     = false;
char          lastNtpSyncStr[24] = "never";

// MQTT globals
WiFiClientSecure  espClient;
PubSubClient      mqttClient(espClient);
unsigned long     lastMqttRetry     = 0;
unsigned long     lastMqttHeartbeat = 0;
bool              mqttEverConnected = false;
char              mqttStatusStr[32] = "disabled";

// Cached topic strings
char topicStatusRelay[64];
char topicStatusMode[64];
char topicStatusOverride[64];
char topicStatusTime[64];
char topicStatusSource[64];
char topicStatusTemp[64];
char topicStatusLastNtp[64];
char topicStatusOnline[64];
char topicCmdOverride[64];

// last published relay state (to detect change)
int8_t lastPublishedRelay = -1;

// ---------------- Config ----------------
struct Config {
  bool overrideActive;
  bool overrideStateOn;
  uint32_t overrideUntil;
  int schedule[7][8][2];

  bool useStatic;
  char staticIP[16];
  char gateway[16];
  char subnet[16];
  char dns1[16];
  char dns2[16];

  // MQTT
  bool mqttEnabled;
  char mqttHost[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[64];
  char mqttClientId[32];
  char mqttTopicBase[32];
} cfg;

const char* CONFIG_PATH = "/config.json";

// ---------------- Forward declarations ----------------
bool syncRTCfromNTP();
bool rtcAvailable();
void updateAllRTCs(const DateTime& dt);
DateTime nowDT();
uint32_t nowUnix();
bool isValidDateTime(const DateTime& t);
void saveConfig();
void loadConfig();
void setDefaults();
void initFS();
void setRelay(bool on);
void applyLogicOnce();
bool validateDayIntervals(int day, String &errMsg);
const char* timeSourceName();

void startAP();
void connectSTA(bool blocking = false);
void wifiWatchdog();
void applyStaticConfig();
void clearStaticConfig();
void startMDNS();
String staStatusText();

void buildTopics();
void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttPublishStatus(bool force = false);
void mqttPublishOnline();
void mqttLoop();

// ---------------- Time helpers ----------------
String minuteToHHMM(int mins) {
  if (mins < 0) return String("--:--");
  if (mins >= 1440) return String("24:00");
  int h = mins / 60;
  int m = mins % 60;
  char b[6]; snprintf(b, sizeof(b), "%02d:%02d", h, m);
  return String(b);
}

bool isValidDateTime(const DateTime& t) {
  return (t.year()   >= 2020 && t.year()   <= 2099 &&
          t.month()  >= 1    && t.month()  <= 12   &&
          t.day()    >= 1    && t.day()    <= 31   &&
          t.hour()   >= 0    && t.hour()   <= 23   &&
          t.minute() >= 0    && t.minute() <= 59   &&
          t.second() >= 0    && t.second() <= 59);
}

bool rtcAvailable() {
  Wire.beginTransmission(0x68);
  return (Wire.endTransmission() == 0);
}

void updateAllRTCs(const DateTime& dt) {
  if (!isValidDateTime(dt)) {
    Serial.println("[updateAllRTCs] Rejecting invalid DateTime");
    return;
  }
  if (rtcAvailable() && rtc.begin()) {
    rtc.adjust(dt);
    Serial.println("[RTC] DS3231 adjusted (local PKT fields)");
  }
  timeval tv;
  tv.tv_sec  = dt.unixtime() - gmtOffset_sec;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  Serial.printf("[RTC] ESP32 clock updated (UTC=%lu, PKT=%lu)\n",
                (unsigned long)tv.tv_sec, (unsigned long)dt.unixtime());
}

DateTime getCurrentTime() {
  if (rtcAvailable() && rtc.begin()) {
    DateTime now = rtc.now();
    if (isValidDateTime(now)) return now;
    Serial.println("[WARN] RTC returned invalid date/time; falling back to system");
  }
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    return DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
  Serial.println("[ERROR] No valid time source available!");
  return DateTime(2025, 1, 1, 0, 0, 0);
}

DateTime nowDT() {
  if (rtcAvailable() && rtc.begin()) {
    DateTime t = rtc.now();
    if (isValidDateTime(t)) return t;
  }
  time_t sys = time(nullptr);
  return DateTime(sys + gmtOffset_sec);
}

uint32_t nowUnix() {
  if (rtcAvailable() && rtc.begin()) {
    DateTime t = rtc.now();
    if (isValidDateTime(t)) return t.unixtime() - gmtOffset_sec;
  }
  return (uint32_t)time(nullptr);
}

const char* timeSourceName() {
  if (rtcAvailable() && rtc.begin()) {
    DateTime t = rtc.now();
    if (isValidDateTime(t)) return "DS3231";
  }
  if (ntpEverSynced) return "NTP";
  struct tm ti;
  if (getLocalTime(&ti)) return "System";
  return "Fallback";
}

// ---------------- RTC watchdog ----------------
unsigned long lastRTCcheck = 0;
void checkRTCs() {
  DateTime now = getCurrentTime();
  if (!isValidDateTime(now)) {
    Serial.println("[WATCHDOG] Invalid time detected, attempting NTP resync...");
    if (syncRTCfromNTP()) {
      DateTime refreshed = getCurrentTime();
      if (isValidDateTime(refreshed)) {
        updateAllRTCs(refreshed);
        Serial.println("[WATCHDOG] Resync success");
      }
    } else {
      Serial.println("[WATCHDOG] NTP resync failed");
    }
  } else {
    Serial.printf("[WATCHDOG] Time OK: %04d-%02d-%02d %02d:%02d:%02d\n",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());
  }
}

// ---------------- NTP ----------------
bool syncRTCfromNTP() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NTP] STA not connected; cannot sync. Will retry later.");
    return false;
  }
  Serial.println("[NTP] Requesting NTP time over existing STA link...");
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) {
    Serial.println("[NTP] getLocalTime timeout");
    return false;
  }
  time_t raw = mktime(&timeinfo);
  DateTime dtLocal(raw + gmtOffset_sec);
  Serial.printf("[NTP] Got PKT local time: %04d-%02d-%02d %02d:%02d:%02d\n",
                dtLocal.year(), dtLocal.month(), dtLocal.day(),
                dtLocal.hour(), dtLocal.minute(), dtLocal.second());
  updateAllRTCs(dtLocal);

  DateTime now = nowDT();
  snprintf(lastNtpSyncStr, sizeof(lastNtpSyncStr), "%04d-%02d-%02d %02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute());
  lastNtpSyncMillis = millis();
  ntpEverSynced     = true;
  Serial.println("[NTP] RTC + system updated from NTP (PKT)");
  mqttPublishStatus(true);
  return true;
}

// ---------------- Filesystem ----------------
void setDefaults() {
  cfg.overrideActive   = false;
  cfg.overrideStateOn  = false;
  cfg.overrideUntil    = 0;
  for (int d = 0; d < 7; d++)
    for (int i = 0; i < 8; i++) { cfg.schedule[d][i][0] = -1; cfg.schedule[d][i][1] = -1; }

  strcpy(ntpSSID, "your_ntp_SSID");
  strcpy(ntpPassword, "your_ntp_PASS");

  cfg.useStatic = false;
  strcpy(cfg.staticIP, ""); strcpy(cfg.gateway, "");
  strcpy(cfg.subnet, "");   strcpy(cfg.dns1, ""); strcpy(cfg.dns2, "");

  cfg.mqttEnabled = false;
  strcpy(cfg.mqttHost, ""); cfg.mqttPort = 8883;
  strcpy(cfg.mqttUser, ""); strcpy(cfg.mqttPass, "");
  strcpy(cfg.mqttClientId, "GeyserE26-esp32");
  strcpy(cfg.mqttTopicBase, "GeyserE26");

  saveConfig();
}

void saveConfig() {
  DynamicJsonDocument doc(4096);
  doc["overrideActive"]  = cfg.overrideActive;
  doc["overrideStateOn"] = cfg.overrideStateOn;
  doc["overrideUntil"]   = cfg.overrideUntil;
  doc["ntpSSID"]         = ntpSSID;
  doc["ntpPassword"]     = ntpPassword;

  doc["useStatic"] = cfg.useStatic;
  doc["staticIP"]  = cfg.staticIP;
  doc["gateway"]   = cfg.gateway;
  doc["subnet"]    = cfg.subnet;
  doc["dns1"]      = cfg.dns1;
  doc["dns2"]      = cfg.dns2;

  doc["mqttEnabled"]  = cfg.mqttEnabled;
  doc["mqttHost"]     = cfg.mqttHost;
  doc["mqttPort"]     = cfg.mqttPort;
  doc["mqttUser"]     = cfg.mqttUser;
  doc["mqttPass"]     = cfg.mqttPass;
  doc["mqttClientId"] = cfg.mqttClientId;
  doc["mqttTopicBase"]= cfg.mqttTopicBase;

  JsonArray sched = doc.createNestedArray("schedule");
  for (int d = 0; d < 7; d++) {
    JsonArray day = sched.createNestedArray();
    for (int i = 0; i < 8; i++) {
      JsonArray interval = day.createNestedArray();
      interval.add(cfg.schedule[d][i][0]);
      interval.add(cfg.schedule[d][i][1]);
    }
  }

  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) { Serial.println("Failed to open config for writing"); return; }
  serializeJson(doc, f);
  f.close();
}

void loadConfig() {
  if (!LittleFS.exists(CONFIG_PATH)) { Serial.println("No config; creating defaults"); setDefaults(); return; }
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) { Serial.println("Cannot open config file"); setDefaults(); return; }
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.println("Config parse failed; using defaults"); setDefaults(); return; }

  cfg.overrideActive  = doc["overrideActive"]  | false;
  cfg.overrideStateOn = doc["overrideStateOn"] | false;
  cfg.overrideUntil   = doc["overrideUntil"]   | 0;

  const char* ssid = doc["ntpSSID"]     | "";
  const char* pass = doc["ntpPassword"] | "";
  strncpy(ntpSSID, ssid, sizeof(ntpSSID) - 1);         ntpSSID[sizeof(ntpSSID)-1] = '\0';
  strncpy(ntpPassword, pass, sizeof(ntpPassword) - 1); ntpPassword[sizeof(ntpPassword)-1] = '\0';

  cfg.useStatic = doc["useStatic"] | false;
  strncpy(cfg.staticIP, doc["staticIP"] | "", sizeof(cfg.staticIP)-1); cfg.staticIP[sizeof(cfg.staticIP)-1]='\0';
  strncpy(cfg.gateway,  doc["gateway"]  | "", sizeof(cfg.gateway)-1);  cfg.gateway[sizeof(cfg.gateway)-1]='\0';
  strncpy(cfg.subnet,   doc["subnet"]   | "", sizeof(cfg.subnet)-1);   cfg.subnet[sizeof(cfg.subnet)-1]='\0';
  strncpy(cfg.dns1,     doc["dns1"]     | "", sizeof(cfg.dns1)-1);     cfg.dns1[sizeof(cfg.dns1)-1]='\0';
  strncpy(cfg.dns2,     doc["dns2"]     | "", sizeof(cfg.dns2)-1);     cfg.dns2[sizeof(cfg.dns2)-1]='\0';

  cfg.mqttEnabled = doc["mqttEnabled"] | false;
  strncpy(cfg.mqttHost, doc["mqttHost"] | "", sizeof(cfg.mqttHost)-1); cfg.mqttHost[sizeof(cfg.mqttHost)-1]='\0';
  cfg.mqttPort = doc["mqttPort"] | 8883;
  strncpy(cfg.mqttUser, doc["mqttUser"] | "", sizeof(cfg.mqttUser)-1); cfg.mqttUser[sizeof(cfg.mqttUser)-1]='\0';
  strncpy(cfg.mqttPass, doc["mqttPass"] | "", sizeof(cfg.mqttPass)-1); cfg.mqttPass[sizeof(cfg.mqttPass)-1]='\0';
  strncpy(cfg.mqttClientId, doc["mqttClientId"] | "smarttimer-esp32", sizeof(cfg.mqttClientId)-1); cfg.mqttClientId[sizeof(cfg.mqttClientId)-1]='\0';
  strncpy(cfg.mqttTopicBase, doc["mqttTopicBase"] | "smarttimer", sizeof(cfg.mqttTopicBase)-1); cfg.mqttTopicBase[sizeof(cfg.mqttTopicBase)-1]='\0';

  JsonArray sched = doc["schedule"].as<JsonArray>();
  if (!sched) { setDefaults(); return; }
  for (int d = 0; d < 7; d++) {
    JsonArray day = sched[d].as<JsonArray>();
    for (int i = 0; i < 8; i++) {
      JsonArray inter = day[i].as<JsonArray>();
      cfg.schedule[d][i][0] = inter[0] | -1;
      cfg.schedule[d][i][1] = inter[1] | -1;
    }
  }
}

void initFS() {
  Serial.print("Mounting LittleFS... ");
  if (LittleFS.begin()) { Serial.println("OK"); return; }
  Serial.println("FAILED - attempting format...");
  if (LittleFS.begin(true)) {
    Serial.println("Formatted and mounted LittleFS");
    setDefaults();
    return;
  }
  Serial.println("Format attempt failed.");
}

// ---------------- WiFi management ----------------
void startAP() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setHostname(MDNS_HOST);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("[AP] IP: "); Serial.println(apIP);
  dnsServer.start(53, "smarttimer.home", apIP);
  Serial.println("[DNS] AP resolver: smarttimer.home -> AP IP");
}

void applyStaticConfig() {
  if (!cfg.useStatic) return;
  IPAddress ip, gw, sn, d1, d2;
  if (!ip.fromString(cfg.staticIP)) { Serial.println("[STA] Bad staticIP"); return; }
  if (!gw.fromString(cfg.gateway))  { Serial.println("[STA] Bad gateway");  return; }
  if (!sn.fromString(cfg.subnet))   { Serial.println("[STA] Bad subnet");   return; }
  IPAddress dns1 = ip, dns2 = ip;
  if (strlen(cfg.dns1) > 0) { d1.fromString(cfg.dns1); dns1 = d1; }
  if (strlen(cfg.dns2) > 0) { d2.fromString(cfg.dns2); dns2 = d2; }
  if (!WiFi.config(ip, gw, sn, dns1, dns2)) Serial.println("[STA] WiFi.config failed");
  else Serial.printf("[STA] Static IP applied: %s\n", cfg.staticIP);
}

void clearStaticConfig() {
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
}

void startMDNS() {
  if (mdnsStarted) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "model", "SmartIntervalController");
    mdnsStarted = true;
    Serial.printf("[mDNS] Started: http://%s.local\n", MDNS_HOST);
  } else {
    Serial.println("[mDNS] Failed to start");
  }
}

void connectSTA(bool blocking) {
  if (strlen(ntpSSID) == 0) { Serial.println("[STA] No SSID configured"); staState = STA_IDLE; return; }
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);
  applyStaticConfig();
  Serial.printf("[STA] Connecting to '%s'...\n", ntpSSID);
  WiFi.begin(ntpSSID, ntpPassword);
  staState = STA_CONNECTING;
  staConnectStart = millis();
  if (blocking) {
    while (millis() - staConnectStart < STA_CONNECT_TIMEOUT) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      staState = STA_CONNECTED;
      Serial.print("[STA] Connected. IP: "); Serial.println(WiFi.localIP());
      startMDNS();
    } else {
      staState = STA_FAILED;
      Serial.println("[STA] Connect failed (blocking)");
    }
  }
}

void wifiWatchdog() {
  unsigned long nowms = millis();
  if (staState == STA_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      staState = STA_CONNECTED;
      Serial.print("[STA] Connected. IP: "); Serial.println(WiFi.localIP());
      startMDNS();
    } else if (nowms - staConnectStart > STA_CONNECT_TIMEOUT) {
      staState = STA_FAILED;
      Serial.println("[STA] Connect timeout");
    }
    return;
  }
  if (staState == STA_CONNECTED) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[STA] Connection lost; will retry");
      staState = STA_FAILED;
      mdnsStarted = false;
    }
    return;
  }
  if (strlen(ntpSSID) == 0) return;
  if (nowms - lastWifiRetry < WIFI_RETRY_INTERVAL) return;
  lastWifiRetry = nowms;
  Serial.println("[STA] Retry attempt...");
  connectSTA(false);
}

String staStatusText() {
  switch (staState) {
    case STA_CONNECTED:  return "Connected";
    case STA_CONNECTING: return "Connecting...";
    case STA_FAILED:     return "Disconnected";
    default:             return "Idle";
  }
}

// ---------------- MQTT ----------------
void buildTopics() {
  const char* b = cfg.mqttTopicBase;
  snprintf(topicStatusRelay,    sizeof(topicStatusRelay),    "%s/status/relay",    b);
  snprintf(topicStatusMode,     sizeof(topicStatusMode),     "%s/status/mode",     b);
  snprintf(topicStatusOverride, sizeof(topicStatusOverride), "%s/status/override", b);
  snprintf(topicStatusTime,     sizeof(topicStatusTime),     "%s/status/time",     b);
  snprintf(topicStatusSource,   sizeof(topicStatusSource),   "%s/status/source",   b);
  snprintf(topicStatusTemp,     sizeof(topicStatusTemp),     "%s/status/temp",     b);
  snprintf(topicStatusLastNtp,  sizeof(topicStatusLastNtp),  "%s/status/last_ntp", b);
  snprintf(topicStatusOnline,   sizeof(topicStatusOnline),   "%s/status/online",   b);
  snprintf(topicCmdOverride,    sizeof(topicCmdOverride),    "%s/cmd/override",    b);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic);
  String p;
  p.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
  p.trim();

  Serial.printf("[MQTT] RX topic=%s payload=%s\n", topic, p.c_str());

  if (t == topicCmdOverride) {
    String cmd = p;
    cmd.toLowerCase();

    if (cmd == "clear") {
      cfg.overrideActive = false;
      cfg.overrideUntil  = 0;
      saveConfig();
      Serial.println("[MQTT] override cleared");
      mqttPublishStatus(true);
      return;
    }

    bool wantOn = false;
    bool wantOff = false;
    long mins = -1;

    if (cmd.startsWith("on")) { wantOn = true; }
    else if (cmd.startsWith("off")) { wantOff = true; }

    if (wantOn || wantOff) {
      int colon = cmd.indexOf(':');
      if (colon > 0) {
        String num = cmd.substring(colon + 1);
        num.trim();
        mins = num.toInt();
        if (mins <= 0) mins = -1;
      }
      cfg.overrideActive  = true;
      cfg.overrideStateOn = wantOn;
      cfg.overrideUntil   = (mins <= 0) ? 0 : (nowUnix() + (uint32_t)mins * 60);
      saveConfig();
      Serial.printf("[MQTT] override set: %s, minutes=%ld\n", wantOn ? "ON" : "OFF", mins);
      mqttPublishStatus(true);
    } else {
      Serial.println("[MQTT] unknown override command");
    }
  }
}

void mqttReconnect() {
  if (!cfg.mqttEnabled) { strcpy(mqttStatusStr, "disabled"); return; }
  if (WiFi.status() != WL_CONNECTED) { strcpy(mqttStatusStr, "no sta"); return; }
  if (strlen(cfg.mqttHost) == 0)     { strcpy(mqttStatusStr, "no host"); return; }
  if (mqttClient.connected()) return;

  unsigned long nowms = millis();
  if (nowms - lastMqttRetry < MQTT_RETRY_INTERVAL) return;
  lastMqttRetry = nowms;

  espClient.setInsecure(); // skip cert validation (still TLS-encrypted)
  mqttClient.setServer(cfg.mqttHost, cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttClient.setBufferSize(512);

  Serial.printf("[MQTT] Connecting to %s:%u ...\n", cfg.mqttHost, cfg.mqttPort);

  bool ok = false;
  if (strlen(cfg.mqttUser) > 0) {
    ok = mqttClient.connect(
      cfg.mqttClientId,
      cfg.mqttUser,
      cfg.mqttPass,
      topicStatusOnline, 0, true, "offline");
  } else {
    ok = mqttClient.connect(
      cfg.mqttClientId,
      topicStatusOnline, 0, true, "offline");
  }

  if (ok) {
    Serial.println("[MQTT] Connected");
    strcpy(mqttStatusStr, "connected");
    mqttEverConnected = true;
    mqttClient.subscribe(topicCmdOverride);
    mqttPublishOnline();
    mqttPublishStatus(true);
    lastMqttHeartbeat = millis();
  } else {
    Serial.printf("[MQTT] Connect failed rc=%d\n", mqttClient.state());
    strcpy(mqttStatusStr, "connect fail");
  }
}

void mqttPublishOnline() {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topicStatusOnline, "online", true);
}

void mqttPublishStatus(bool force) {
  if (!cfg.mqttEnabled) return;
  if (!mqttClient.connected()) return;

  bool relayOn = (digitalRead(RELAY_PIN) == (RELAY_ACTIVE_LOW ? LOW : HIGH));

  if (!force) {
    unsigned long nowms = millis();
    if (nowms - lastMqttHeartbeat < MQTT_HEARTBEAT_MS) {
      if (lastPublishedRelay == (int8_t)relayOn) return;
    }
  }
  lastMqttHeartbeat = millis();
  lastPublishedRelay = (int8_t)relayOn;

  mqttClient.publish(topicStatusRelay, relayOn ? "ON" : "OFF", true);

  const char* mode = cfg.overrideActive ? "Override" : "Schedule";
  mqttClient.publish(topicStatusMode, mode, true);

  String ovrStr;
  if (!cfg.overrideActive) {
    ovrStr = "none";
  } else {
    String statePart = cfg.overrideStateOn ? "on" : "off";
    if (cfg.overrideUntil == 0) {
      ovrStr = statePart + "_indefinite";
    } else {
      DateTime until((uint32_t)cfg.overrideUntil + gmtOffset_sec);
      char b[32];
      snprintf(b, sizeof(b), "%s_until_%02d:%02d", statePart.c_str(), until.hour(), until.minute());
      ovrStr = b;
    }
  }
  mqttClient.publish(topicStatusOverride, ovrStr.c_str(), true);

  DateTime now = nowDT();
  char tb[32];
  snprintf(tb, sizeof(tb), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  mqttClient.publish(topicStatusTime, tb, true);

  mqttClient.publish(topicStatusSource, timeSourceName(), true);

  if (rtcAvailable() && rtc.begin()) {
    char tb2[12];
    snprintf(tb2, sizeof(tb2), "%.1f", rtc.getTemperature());
    mqttClient.publish(topicStatusTemp, tb2, true);
  } else {
    mqttClient.publish(topicStatusTemp, "N/A", true);
  }

  mqttClient.publish(topicStatusLastNtp, lastNtpSyncStr, true);
}

void mqttLoop() {
  if (!cfg.mqttEnabled) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      mqttReconnect();
    } else {
      mqttClient.loop();
      mqttPublishStatus(false);
    }
  } else {
    strcpy(mqttStatusStr, "no sta");
  }
}

// ---------------- Relay ----------------
void setRelay(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
    digitalWrite(LED_PIN,   on ? HIGH : LOW);
  } else {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
    digitalWrite(LED_PIN,   on ? LOW : HIGH);
  }
}

// ---------------- Logic ----------------
void applyLogicOnce() {
  DateTime now = nowDT();
  uint32_t nowu = nowUnix();

  if (cfg.overrideActive && cfg.overrideUntil != 0) {
    if (nowu >= cfg.overrideUntil) {
      cfg.overrideActive = false;
      cfg.overrideUntil  = 0;
      saveConfig();
    }
  }

  if (cfg.overrideActive) {
    setRelay(cfg.overrideStateOn);
    return;
  }

  int dow  = now.dayOfTheWeek();
  int mins = now.hour() * 60 + now.minute();
  bool on  = false;
  for (int i = 0; i < 8; i++) {
    int s = cfg.schedule[dow][i][0];
    int e = cfg.schedule[dow][i][1];
    if (s >= 0 && e >= 0 && s <= mins && mins < e) { on = true; break; }
  }
  setRelay(on);
}

bool validateDayIntervals(int day, String &errMsg) {
  int lastStop = -1;
  for (int i = 0; i < 8; i++) {
    int s = cfg.schedule[day][i][0];
    int e = cfg.schedule[day][i][1];
    if (s < 0 && e < 0) continue;
    if (s < 0 || e < 0) { errMsg = "Interval " + String(i+1) + " incomplete"; return false; }
    if (e <= s)         { errMsg = "Interval " + String(i+1) + ": Stop must be > Start"; return false; }
    if (s < lastStop)   { errMsg = "Interval " + String(i+1) + " overlaps previous"; return false; }
    lastStop = e;
  }
  return true;
}

bool findNextTransition(DateTime from, DateTime &when, bool &willTurnOn) {
  for (int offset = 0; offset < 8; offset++) {
    DateTime dayStart(from.year(), from.month(), from.day(), 0, 0, 0);
    dayStart = dayStart + TimeSpan(offset, 0, 0, 0);
    int dow = dayStart.dayOfTheWeek();
    int curMins = (offset == 0) ? (from.hour() * 60 + from.minute()) : -1;
    for (int i = 0; i < 8; i++) {
      int s = cfg.schedule[dow][i][0];
      int e = cfg.schedule[dow][i][1];
      if (s < 0 || e < 0) continue;
      if (s > curMins) {
        when = DateTime(dayStart.year(), dayStart.month(), dayStart.day(), s/60, s%60, 0);
        willTurnOn = true;
        return true;
      }
      if (e > curMins) {
        when = DateTime(dayStart.year(), dayStart.month(), dayStart.day(), e/60, e%60, 0);
        willTurnOn = false;
        return true;
      }
    }
  }
  return false;
}

// ---------------- HTML: common style ----------------
String commonStyle() {
  String s;
  s += "<style>"
       "body{margin:0;font-family:'Segoe UI',sans-serif;background:#f4f6f8;color:#333;text-align:center;}"
       "header{padding:15px;background:#fff;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
       "h1{margin:0;font-size:22px;color:#0066cc;}"
       "h2{margin:0 0 10px 0;font-size:18px;color:#0066cc;}"
       ".container{padding:20px;}"
       ".status-box{background:#fff;margin:20px auto;padding:15px;border-radius:10px;max-width:360px;box-shadow:0 2px 4px rgba(0,0,0,0.1);text-align:left;}"
       ".status-box div{margin:4px 0;}"
       ".button{display:block;width:85%;margin:10px auto;padding:14px;font-size:16px;background-color:#4CAF50;color:white;border:none;border-radius:8px;cursor:pointer;transition:background-color 0.2s;text-decoration:none;}"
       ".button:hover{background-color:#45a049;}"
       ".small{font-size:14px;color:#666;margin-top:10px;}"
       ".card{background:#fff;margin:15px auto;padding:15px;border-radius:10px;max-width:520px;box-shadow:0 2px 4px rgba(0,0,0,0.1);text-align:left;}"
       ".row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #eee;font-size:14px;}"
       ".row:last-child{border-bottom:none;}"
       ".lbl{color:#666;}"
       ".val{font-weight:bold;color:#222;text-align:right;}"
       ".pill{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;color:#fff;}"
       ".ok{background:#28a745;}.bad{background:#dc3545;}.warn{background:#ffc107;color:#333;}.neutral{background:#6c757d;}"
       "table.acc{width:100%;border-collapse:collapse;font-size:14px;}"
       "table.acc th,table.acc td{border:1px solid #ddd;padding:6px;text-align:left;}"
       "table.acc th{background:#f0f0f0;}"
       "input,select{width:95%;padding:10px;margin:6px 0;border:1px solid #ccc;border-radius:8px;font-size:15px;box-sizing:border-box;}"
       "label{display:block;text-align:left;font-size:14px;color:#444;margin-top:10px;}"
       "</style>";
  return s;
}

// ---------------- Dashboard ----------------
void handleRoot() {
  DateTime now = nowDT();
  String html;
  html.reserve(4500);
  html += "<!doctype html><html lang='en'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Smart Interval Controller</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>Weekly Smart Timer</h1></header><div class='container'>";
  html += "<div class='status-box'>";

  char tb[64];
  snprintf(tb, sizeof(tb), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  html += "<div><strong>Time:</strong> " + String(tb) + "</div>";
  html += "<div><strong>Time source:</strong> " + String(timeSourceName()) + "</div>";

  bool currentRelay = (digitalRead(RELAY_PIN) == (RELAY_ACTIVE_LOW ? LOW : HIGH));
  html += "<div><strong>Status:</strong> " + String(currentRelay ? "ON" : "OFF") + "</div>";

  if (cfg.overrideActive) {
    uint32_t rem = 0;
    uint32_t nowu = nowUnix();
    if (cfg.overrideUntil > nowu) rem = (cfg.overrideUntil - nowu) / 60;
    html += "<div><strong>Override:</strong> ACTIVE, forcing " +
            String(cfg.overrideStateOn ? "ON" : "OFF") +
            (cfg.overrideUntil ? (", expires in " + String(rem) + " min") : ", indefinite") + "</div>";
  } else {
    html += "<div><strong>Mode:</strong> Schedule</div>";
  }

  String staLine = (staState == STA_CONNECTED) ? String(WiFi.localIP().toString()) : staStatusText();
  html += "<div><strong>Router:</strong> " + staLine + "</div>";

  html += "<div><strong>MQTT:</strong> " + String(mqttStatusStr) + "</div>";

  DateTime when; bool willTurnOn = false;
  if (findNextTransition(now, when, willTurnOn)) {
    char nb[64];
    snprintf(nb, sizeof(nb), "%s at %02d:%02d %s",
             willTurnOn ? "ON" : "OFF",
             when.hour(), when.minute(),
             (when.day() == now.day()) ? "today" : "later");
    html += "<div><strong>Next:</strong> " + String(nb) + "</div>";
  } else {
    html += "<div><strong>Next:</strong> —</div>";
  }

  if (rtcAvailable() && rtc.begin()) {
    html += "<div class='small'><strong>Temperature:</strong> " + String(rtc.getTemperature(), 1) + " &deg;C</div>";
  } else {
    html += "<div class='small'><strong>Temperature:</strong> N/A</div>";
  }
  html += "</div>";

  html += "<a class='button' href='/schedule'>Edit Schedule</a>";
  html += "<a class='button' href='/override'>Manual Override</a>";
  html += "<a class='button' href='/wifi'>Wi-Fi Setup</a>";
  html += "<a class='button' href='#' onclick='setTimeFromBrowser();return false;'>Set Time from Browser</a>";
  html += "<a class='button' href='/network'>Network Info</a>";
  html += "<a class='button' href='/mqtt'>MQTT Setup</a>";
  html += "<a class='button' href='/reboot'>Reboot Device</a>";

  html += "<script>"
          "function setTimeFromBrowser(){"
          "var d=new Date();"
          "var qs='?y='+d.getFullYear()+'&M='+(d.getMonth()+1)+'&D='+d.getDate()+'&h='+d.getHours()+'&m='+d.getMinutes()+'&s='+d.getSeconds();"
          "fetch('/updatetime'+qs).then(r=>{if(r.ok)alert('RTC updated');else alert('Update failed');});}"
          "</script>";

  html += "<div class='small' style='margin-top:20px;'>AP SSID: <b>" + String(AP_SSID) +
          "</b> | Password: <b>" + String(AP_PASS) + "</b></div>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- Network Info page ----------------
void handleNetworkPage() {
  DateTime now = nowDT();
  String html;
  html.reserve(6000);
  html += "<!doctype html><html lang='en'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Network Info</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>Network &amp; Status Info</h1></header><div class='container'>";

  // Time
  {
    html += "<div class='card'><h2>Time</h2>";
    char lb[64];
    snprintf(lb, sizeof(lb), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    html += "<div class='row'><span class='lbl'>Local (PKT)</span><span class='val'>" + String(lb) + "</span></div>";

    time_t utcT = now.unixtime() - gmtOffset_sec;
    struct tm utm; gmtime_r(&utcT, &utm);
    char ub[64];
    snprintf(ub, sizeof(ub), "%04d-%02d-%02d %02d:%02d:%02d",
             utm.tm_year + 1900, utm.tm_mon + 1, utm.tm_mday,
             utm.tm_hour, utm.tm_min, utm.tm_sec);
    html += "<div class='row'><span class='lbl'>UTC</span><span class='val'>" + String(ub) + "</span></div>";
    html += "<div class='row'><span class='lbl'>Source</span><span class='val'>" + String(timeSourceName()) + "</span></div>";
    html += "<div class='row'><span class='lbl'>Last NTP sync</span><span class='val'>" + String(lastNtpSyncStr) + "</span></div>";
    if (rtcAvailable() && rtc.begin()) {
      html += "<div class='row'><span class='lbl'>RTC temperature</span><span class='val'>" + String(rtc.getTemperature(), 1) + " &deg;C</span></div>";
    }
    html += "</div>";
  }

  // AP
  {
    html += "<div class='card'><h2>Access Point</h2>";
    html += "<div class='row'><span class='lbl'>SSID</span><span class='val'>" + String(AP_SSID) + "</span></div>";
    html += "<div class='row'><span class='lbl'>IP</span><span class='val'>" + WiFi.softAPIP().toString() + "</span></div>";
    html += "<div class='row'><span class='lbl'>Clients</span><span class='val'>" + String(WiFi.softAPgetStationNum()) + "</span></div>";
    html += "</div>";
  }

  // STA
  {
    html += "<div class='card'><h2>Router (STA)</h2>";
    html += "<div class='row'><span class='lbl'>Status</span><span class='val'>" + staStatusText() + "</span></div>";
    html += "<div class='row'><span class='lbl'>SSID</span><span class='val'>" + String(ntpSSID) + "</span></div>";
    if (staState == STA_CONNECTED) {
      int rssi = WiFi.RSSI();
      String qual = (rssi > -60) ? "Good" : (rssi > -75) ? "Fair" : "Weak";
      html += "<div class='row'><span class='lbl'>IP</span><span class='val'>" + WiFi.localIP().toString() + "</span></div>";
      html += "<div class='row'><span class='lbl'>Signal</span><span class='val'>" + String(rssi) + " dBm (" + qual + ")</span></div>";
      html += "<div class='row'><span class='lbl'>MAC</span><span class='val'>" + WiFi.macAddress() + "</span></div>";
      html += "<div class='row'><span class='lbl'>Gateway</span><span class='val'>" + WiFi.gatewayIP().toString() + "</span></div>";
    }
    html += "</div>";
  }

  // MQTT
  {
    html += "<div class='card'><h2>MQTT</h2>";
    html += "<div class='row'><span class='lbl'>Enabled</span><span class='val'>" + String(cfg.mqttEnabled ? "Yes" : "No") + "</span></div>";
    html += "<div class='row'><span class='lbl'>Status</span><span class='val'>" + String(mqttStatusStr) + "</span></div>";
    html += "<div class='row'><span class='lbl'>Host</span><span class='val'>" + String(cfg.mqttHost) + ":" + String(cfg.mqttPort) + "</span></div>";
    html += "<div class='row'><span class='lbl'>Topic base</span><span class='val'>" + String(cfg.mqttTopicBase) + "</span></div>";
    html += "</div>";
  }

  // Today
  {
    int dow  = now.dayOfTheWeek();
    int mins = now.hour() * 60 + now.minute();
    static const char* dayNames[7] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
    html += "<div class='card'><h2>Today's Schedule (" + String(dayNames[dow]) + ")</h2>";
    bool any = false;
    for (int i = 0; i < 8; i++) {
      int s = cfg.schedule[dow][i][0];
      int e = cfg.schedule[dow][i][1];
      if (s < 0 || e < 0) continue;
      any = true;
      bool active = (s <= mins && mins < e);
      html += "<div class='row'><span class='lbl'>" + minuteToHHMM(s) + " &rarr; " + minuteToHHMM(e) +
              (active ? " <span class='pill ok'>ACTIVE</span>" : "") +
              "</span><span class='val'>Interval " + String(i+1) + "</span></div>";
    }
    if (!any) html += "<div class='small'>No intervals for today.</div>";
    html += "</div>";
  }

  html += "<a class='button' href='/'>Back to Dashboard</a>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- Reboot ----------------
void handleReboot() {
  if (server.method() == HTTP_POST) {
    server.send(200, "text/html",
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:'Segoe UI';text-align:center;margin-top:60px;background:#f4f6f8;}"
      ".msg{display:inline-block;padding:16px 30px;background:#fff3cd;color:#856404;border:1px solid #ffeeba;border-radius:10px;font-size:17px;}"
      "</style></head><body><div class='msg'>Rebooting device... reconnect in ~10 seconds.</div></body></html>");
    delay(500);
    ESP.restart();
  }
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Reboot Device</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>Reboot Device</h1></header><div class='container'>";
  html += "<div class='card'><p>The device will restart. Schedules and overrides are preserved in flash.</p>";
  html += "<form method='POST'><button class='button' type='submit'>Confirm Reboot</button></form>";
  html += "<a class='button' href='/'>Cancel / Back</a></div>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- MQTT Setup page ----------------
void handleMqttPage() {
  if (server.method() == HTTP_POST) {
    bool enabled = server.hasArg("enabled");
    String host  = server.arg("host");
    int    port  = server.arg("port").toInt();
    String user  = server.arg("user");
    String pass  = server.arg("pass");
    String cid   = server.arg("cid");
    String base  = server.arg("base");

    if (port <= 0 || port > 65535) port = 8883;
    if (base.length() == 0) base = "smarttimer";
    if (cid.length() == 0)  cid  = "smarttimer-esp32";

    // Apply
    cfg.mqttEnabled = enabled;
    strncpy(cfg.mqttHost, host.c_str(), sizeof(cfg.mqttHost)-1); cfg.mqttHost[sizeof(cfg.mqttHost)-1]='\0';
    cfg.mqttPort = (uint16_t)port;
    strncpy(cfg.mqttUser, user.c_str(), sizeof(cfg.mqttUser)-1); cfg.mqttUser[sizeof(cfg.mqttUser)-1]='\0';
    strncpy(cfg.mqttPass, pass.c_str(), sizeof(cfg.mqttPass)-1); cfg.mqttPass[sizeof(cfg.mqttPass)-1]='\0';
    strncpy(cfg.mqttClientId, cid.c_str(), sizeof(cfg.mqttClientId)-1); cfg.mqttClientId[sizeof(cfg.mqttClientId)-1]='\0';
    strncpy(cfg.mqttTopicBase, base.c_str(), sizeof(cfg.mqttTopicBase)-1); cfg.mqttTopicBase[sizeof(cfg.mqttTopicBase)-1]='\0';

    saveConfig();
    buildTopics();

    // Force reconnect attempt
    if (mqttClient.connected()) mqttClient.disconnect();
    lastMqttRetry = 0;
    mqttReconnect();

    String color = mqttClient.connected() ? "#28a745" : "#ffc107";
    String msg   = mqttClient.connected() ? "MQTT Saved & Connected!" : "MQTT Saved. Waiting for connect...";

    server.send(200, "text/html",
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<script>setTimeout(function(){window.location.href='/';},1800);</script>"
      "<style>body{font-family:'Segoe UI';text-align:center;margin-top:60px;background:#f4f6f8;}"
      ".msg{display:inline-block;padding:16px 30px;background:" + color + ";color:#fff;border-radius:10px;font-size:17px;}"
      "</style></head><body><div class='msg'>" + msg + "</div></body></html>");
    return;
  }

  // GET form
  String html;
  html.reserve(4000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>MQTT Setup</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>MQTT Setup</h1></header><div class='container'>";
  html += "<form method='POST' class='card'>";
  html += "<label><input type='checkbox' name='enabled' " + String(cfg.mqttEnabled ? "checked" : "") + " style='width:auto;'> Enable MQTT</label>";
  html += "<label>Broker host</label><input name='host' value='" + String(cfg.mqttHost) + "' placeholder='abc123.emqx.cloud'>";
  html += "<label>Port (8883 for TLS)</label><input name='port' value='" + String(cfg.mqttPort) + "'>";
  html += "<label>Username</label><input name='user' value='" + String(cfg.mqttUser) + "'>";
  html += "<label>Password</label><input name='pass' type='password' value='" + String(cfg.mqttPass) + "'>";
  html += "<label>Client ID</label><input name='cid' value='" + String(cfg.mqttClientId) + "'>";
  html += "<label>Topic base</label><input name='base' value='" + String(cfg.mqttTopicBase) + "'>";
  html += "<button class='button' type='submit'>Test &amp; Save</button>";
  html += "</form>";
  html += "<div class='card small' style='text-align:left;'>";
  html += "<b>Commands</b> (publish to <code>" + String(cfg.mqttTopicBase) + "/cmd/override</code>):<br>";
  html += "<code>on</code>, <code>off</code>, <code>clear</code>, <code>on:60</code>, <code>off:30</code><br><br>";
  html += "<b>Status topics</b> (subscribe):<br>";
  html += "<code>" + String(cfg.mqttTopicBase) + "/status/#</code>";
  html += "</div>";
  html += "<a class='button' href='/'>Back to Dashboard</a>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- Schedule ----------------
void handleSchedule() {
  if (server.method() == HTTP_POST) {
    for (int d = 0; d < 7; d++) {
      for (int i = 0; i < 8; i++) {
        String sname = "s" + String(d) + "_" + String(i);
        String ename = "e" + String(d) + "_" + String(i);
        if (server.hasArg(sname) && server.hasArg(ename)) {
          cfg.schedule[d][i][0] = server.arg(sname).toInt();
          cfg.schedule[d][i][1] = server.arg(ename).toInt();
        } else {
          cfg.schedule[d][i][0] = -1;
          cfg.schedule[d][i][1] = -1;
        }
      }
    }
    String errAll = ""; bool ok = true;
    for (int d = 0; d < 7; d++) {
      String msg;
      if (!validateDayIntervals(d, msg)) {
        ok = false;
        static const char* daynames[7] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
        errAll += String(daynames[d]) + ": " + msg + "<br>";
      }
    }
    if (!ok) {
      String out = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                   "<style>body{font-family:Arial;margin:20px;}h3{color:#c00;}.error{color:red;}"
                   ".box{border:1px solid #ccc;padding:10px;border-radius:6px;background:#f9f9f9;}"
                   ".btn{display:inline-block;padding:10px 16px;margin-top:16px;background:#0066cc;color:#fff;text-decoration:none;border-radius:6px;}"
                   "</style></head><body>";
      out += "<h3>Schedule Error</h3><p class='error'>One or more days have invalid or overlapping intervals:</p>";
      out += "<div class='box'>" + errAll + "</div>";
      out += "<a href='/schedule' class='btn'>Back to Schedule</a></body></html>";
      server.send(200, "text/html", out);
      return;
    }
    saveConfig();
    server.send(200, "text/html",
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<script>setTimeout(function(){window.location.href='/'},1000);</script>"
      "<body style='font-family:Arial;text-align:center;margin-top:40px;'>"
      "<div style='display:inline-block;padding:10px 20px;background:#dff0d8;color:#3c763d;"
      "border:1px solid #3c763d;border-radius:6px;'>Data Saved!</div><br><br>"
      "<a href='/' style='display:inline-block;padding:10px 20px;background:#0066cc;color:#fff;"
      "text-decoration:none;border-radius:6px;'>Home</a></body></html>");
    return;
  }

  static const char* days[7] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  server.sendContent("<style>");
  server.sendContent("body{font-family:Arial;font-size:16px;margin:10px;}");
  server.sendContent("h2{text-align:center;color:#0066cc;}");
  server.sendContent("table{border-collapse:collapse;width:100%;overflow-x:auto;display:block;}");
  server.sendContent("th,td{border:1px solid #ccc;padding:6px;text-align:left;}");
  server.sendContent("th{background:#f0f0f0;}");
  server.sendContent("tr:nth-child(4n+2){background:#caffa2;}");
  server.sendContent("tr:nth-child(4n+3){background:#caffa2;}");
  server.sendContent("tr:nth-child(4n+4){background:#92bffc;}");
  server.sendContent("tr:nth-child(4n+5){background:#92bffc;}");
  server.sendContent("select{font-size:14px;padding:3px;}");
  server.sendContent("button,input[type=submit]{background:#0066cc;color:white;border:none;border-radius:5px;padding:8px 14px;margin:6px;font-size:16px;}");
  server.sendContent("button:hover,input[type=submit]:hover{background:#004c99;cursor:pointer;}");
  server.sendContent(".container{max-width:100%;overflow-x:auto;}");
  server.sendContent("</style>");
  server.sendContent("<title>Schedule</title></head><body>");
  server.sendContent("<h3>Weekly Schedule (8 intervals/day)</h3>");
  server.sendContent("<form method='POST'><table border='1' cellspacing='0' cellpadding='4'><tr><th>Timer</th>");
  for (int d = 0; d < 7; d++) server.sendContent("<th>" + String(days[d]) + "</th>");
  server.sendContent("</tr>");

  for (int row = 0; row < 8; row++) {
    server.sendContent("<tr><td>ON </td>");
    for (int d = 0; d < 7; d++) {
      String sname = "s" + String(d) + "_" + String(row);
      server.sendContent("<td><select name='" + sname + "' id='" + sname + "' onchange='disableStop(this,\"e" + String(d) + "_" + String(row) + "\")'>");
      server.sendContent("<option value='-1'");
      if (cfg.schedule[d][row][0] < 0) server.sendContent(" selected");
      server.sendContent(">--</option>");
      String buf; buf.reserve(1024);
      for (int h = 0; h < 24; h++) {
        for (int m = 0; m < 60; m += 30) {
          int val = h * 60 + m;
          char tb[8]; snprintf(tb, sizeof(tb), "%02d:%02d", h, m);
          buf += "<option value='" + String(val) + "'";
          if (val == cfg.schedule[d][row][0]) buf += " selected";
          buf += ">" + String(tb) + "</option>";
          if (buf.length() > 900) { server.sendContent(buf); buf = ""; }
        }
      }
      if (buf.length()) server.sendContent(buf);
      server.sendContent("</select></td>");
    }
    server.sendContent("</tr>");

    server.sendContent("<tr><td>OFF </td>");
    for (int d = 0; d < 7; d++) {
      String ename = "e" + String(d) + "_" + String(row);
      server.sendContent("<td><select name='" + ename + "' id='" + ename + "'>");
      server.sendContent("<option value='-1'");
      if (cfg.schedule[d][row][1] < 0 || cfg.schedule[d][row][0] < 0) server.sendContent(" selected");
      server.sendContent(">--</option>");
      String buf; buf.reserve(1024);
      for (int h = 0; h < 24; h++) {
        for (int m = 0; m < 60; m += 30) {
          int val = h * 60 + m;
          char tb[8]; snprintf(tb, sizeof(tb), "%02d:%02d", h, m);
          buf += "<option value='" + String(val) + "'";
          if (val == cfg.schedule[d][row][1]) buf += " selected";
          buf += ">" + String(tb) + "</option>";
          if (buf.length() > 900) { server.sendContent(buf); buf = ""; }
        }
      }
      buf += "<option value='1440'";
      if (cfg.schedule[d][row][1] == 1440) buf += " selected";
      buf += ">24:00</option>";
      if (buf.length()) server.sendContent(buf);
      server.sendContent("</select></td>");
    }
    server.sendContent("</tr>");
  }

  server.sendContent(
    "</table><br><div style='text-align:center;'>"
    "<input type='submit' value='Save' style='display:inline-block;padding:10px 16px;background:#0066cc;color:#fff;"
    "border:none;border-radius:6px;font-family:Arial;font-size:16px;margin-right:10px;'>"
    "<a href='/' style='display:inline-block;padding:10px 16px;background:#0066cc;color:#fff;"
    "text-decoration:none;border-radius:6px;font-family:Arial;font-size:16px;'>Home</a>"
    "</div></form>");

  server.sendContent(
    "<div style='text-align:center;margin:15px 0;'>"
    "<button type='button' onclick='copySunday()' style='padding:10px 16px;background:#0066cc;color:#fff;border:none;"
    "border-radius:6px;font-size:16px;'>Copy Sunday to All Days</button></div>");

  server.sendContent("<script>");
  server.sendContent("function copySunday(){");
  server.sendContent(" for (var row=0; row<8; row++){");
  server.sendContent("   var startSun=document.querySelector(\"select[name='s0_\"+row+\"']\");");
  server.sendContent("   var stopSun=document.querySelector(\"select[name='e0_\"+row+\"']\");");
  server.sendContent("   if(startSun && stopSun){");
  server.sendContent("     var valStart=startSun.value; var valStop=stopSun.value;");
  server.sendContent("     for (var d=1; d<7; d++){");
  server.sendContent("       var tgtStart=document.querySelector(\"select[name='s\"+d+\"_\"+row+\"']\");");
  server.sendContent("       var tgtStop=document.querySelector(\"select[name='e\"+d+\"_\"+row+\"']\");");
  server.sendContent("       if(tgtStart){ tgtStart.value = valStart; }");
  server.sendContent("       if(tgtStop){ tgtStop.value = valStop; }");
  server.sendContent("       if(tgtStart){ disableStop(tgtStart, 'e'+d+'_'+row); }");
  server.sendContent("     }");
  server.sendContent("   }");
  server.sendContent(" }");
  server.sendContent("}");
  server.sendContent("</script>");

  server.sendContent("<script>");
  server.sendContent("function disableStop(startSel, stopId){");
  server.sendContent(" var stopSel=document.getElementById(stopId);");
  server.sendContent(" if(startSel.value=='-1'){ stopSel.disabled=true; stopSel.value='-1'; } else { stopSel.disabled=false; }");
  server.sendContent("}");
  server.sendContent("window.onload=function(){");
  server.sendContent(" var allStarts=document.querySelectorAll('select[name^=s]');");
  server.sendContent(" for(var i=0;i<allStarts.length;i++){");
  server.sendContent("   var sel=allStarts[i];");
  server.sendContent("   var stopId=sel.name.replace('s','e');");
  server.sendContent("   disableStop(sel,stopId);");
  server.sendContent(" }");
  server.sendContent("};");
  server.sendContent("</script></body></html>");
}

// ---------------- Override ----------------
void handleOverridePage() {
  if (server.method() == HTTP_POST) {
    String state = server.arg("state");
    float hours  = server.arg("hours").toFloat();
    int mins = (int)(hours * 60);
    if (state == "on") {
      cfg.overrideActive = true; cfg.overrideStateOn = true;
      cfg.overrideUntil = (mins <= 0) ? 0 : nowUnix() + (uint32_t)mins * 60;
    } else if (state == "off") {
      cfg.overrideActive = true; cfg.overrideStateOn = false;
      cfg.overrideUntil = (mins <= 0) ? 0 : nowUnix() + (uint32_t)mins * 60;
    } else if (state == "clear") {
      cfg.overrideActive = false; cfg.overrideUntil = 0;
    }
    saveConfig();
    mqttPublishStatus(true);

    server.send(200, "text/html",
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<script>setTimeout(function(){window.location.href='/'},1200);</script>"
      "<style>body{font-family:'Segoe UI';text-align:center;margin-top:40px;background:#f4f6f8;}"
      ".msg{display:inline-block;padding:12px 24px;background:#dff0d8;color:#3c763d;border:1px solid #3c763d;border-radius:10px;}"
      "</style></head><body><div class='msg'>Override Updated!</div><br><a href='/'> Dashboard </a></body></html>");
    return;
  }

  String statusText, color;
  if (cfg.overrideActive) {
    time_t nowT = nowUnix();
    if (cfg.overrideUntil > nowT) {
      uint32_t remaining = cfg.overrideUntil - nowT;
      uint16_t minsLeft = remaining / 60;
      uint8_t hrs = minsLeft / 60;
      uint8_t mins = minsLeft % 60;
      statusText = String("ACTIVE — Forced ") + (cfg.overrideStateOn ? "ON" : "OFF") +
                   " (" + String(hrs) + "h " + String(mins) + "m remaining)";
      color = cfg.overrideStateOn ? "#28a745" : "#dc3545";
    } else {
      statusText = "Expired — Override no longer active";
      color = "#6c757d";
    }
  } else {
    statusText = "Inactive";
    color = "#6c757d";
  }

  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Manual Override</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>Manual Override</h1></header><div class='container'>";
  html += "<form method='POST'>";
  html += "<label for='hours'>Duration (hours):</label>";
  html += "<select name='hours' id='hours'>";
  html += "<option value='0.5'>0.5</option><option value='1.0'>1.0</option>";
  html += "<option value='1.5'>1.5</option><option value='2.0'>2.0</option>";
  html += "<option value='2.5'>2.5</option><option value='3.0'>3.0</option>";
  html += "<option value='3.5'>3.5</option><option value='4.0'>4.0</option>";
  html += "</select>";
  html += "<button class='button' name='state' value='on'>Force ON</button>";
  html += "<button class='button' name='state' value='off'>Force OFF</button>";
  html += "<button class='button' name='state' value='clear' style='background:#888;'>Clear Override</button>";
  html += "</form>";
  html += "<a class='button' href='/'>Back to Dashboard</a>";
  html += "<div class='small' style='color:" + color + ";font-weight:bold;margin-top:15px;'>" + statusText + "</div>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- Wi-Fi setup ----------------
void handleWiFiPage() {
  if (server.method() == HTTP_POST) {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    bool   wantStatic = server.hasArg("useStatic");
    String sIP  = server.arg("staticIP");
    String sGw  = server.arg("gateway");
    String sSn  = server.arg("subnet");
    String sD1  = server.arg("dns1");
    String sD2  = server.arg("dns2");

    char oldSSID[32];     strncpy(oldSSID, ntpSSID, sizeof(oldSSID));         oldSSID[sizeof(oldSSID)-1]='\0';
    char oldPass[64];     strncpy(oldPass, ntpPassword, sizeof(oldPass));     oldPass[sizeof(oldPass)-1]='\0';
    bool oldStatic = cfg.useStatic;
    char oldIP[16];  strncpy(oldIP, cfg.staticIP, sizeof(oldIP));  oldIP[sizeof(oldIP)-1]='\0';
    char oldGw[16];  strncpy(oldGw, cfg.gateway,  sizeof(oldGw));  oldGw[sizeof(oldGw)-1]='\0';
    char oldSn[16];  strncpy(oldSn, cfg.subnet,   sizeof(oldSn));  oldSn[sizeof(oldSn)-1]='\0';
    char oldD1[16];  strncpy(oldD1, cfg.dns1,     sizeof(oldD1));  oldD1[sizeof(oldD1)-1]='\0';
    char oldD2[16];  strncpy(oldD2, cfg.dns2,     sizeof(oldD2));  oldD2[sizeof(oldD2)-1]='\0';

    strncpy(ntpSSID, ssid.c_str(), sizeof(ntpSSID)-1);         ntpSSID[sizeof(ntpSSID)-1]='\0';
    strncpy(ntpPassword, pass.c_str(), sizeof(ntpPassword)-1); ntpPassword[sizeof(ntpPassword)-1]='\0';
    cfg.useStatic = wantStatic;
    strncpy(cfg.staticIP, sIP.c_str(), sizeof(cfg.staticIP)-1); cfg.staticIP[sizeof(cfg.staticIP)-1]='\0';
    strncpy(cfg.gateway,  sGw.c_str(), sizeof(cfg.gateway)-1);  cfg.gateway[sizeof(cfg.gateway)-1]='\0';
    strncpy(cfg.subnet,   sSn.c_str(), sizeof(cfg.subnet)-1);   cfg.subnet[sizeof(cfg.subnet)-1]='\0';
    strncpy(cfg.dns1,     sD1.c_str(), sizeof(cfg.dns1)-1);     cfg.dns1[sizeof(cfg.dns1)-1]='\0';
    strncpy(cfg.dns2,     sD2.c_str(), sizeof(cfg.dns2)-1);     cfg.dns2[sizeof(cfg.dns2)-1]='\0';

    mdnsStarted = false;
    WiFi.disconnect(false, false);
    delay(200);
    if (cfg.useStatic) applyStaticConfig(); else clearStaticConfig();
    connectSTA(true);

    bool connected = (staState == STA_CONNECTED);
    String msgColor, msgText;

    if (connected) {
      configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
      struct tm tinfo;
      bool ntpOK = getLocalTime(&tinfo, 5000);
      saveConfig();
      if (ntpOK) { msgColor = "#28a745"; msgText = "Wi-Fi and NTP test successful!"; }
      else       { msgColor = "#ffc107"; msgText = "Wi-Fi connected, but NTP failed."; }
    } else {
      strncpy(ntpSSID, oldSSID, sizeof(ntpSSID)-1);         ntpSSID[sizeof(ntpSSID)-1]='\0';
      strncpy(ntpPassword, oldPass, sizeof(ntpPassword)-1); ntpPassword[sizeof(ntpPassword)-1]='\0';
      cfg.useStatic = oldStatic;
      strncpy(cfg.staticIP, oldIP, sizeof(cfg.staticIP)-1); cfg.staticIP[sizeof(cfg.staticIP)-1]='\0';
      strncpy(cfg.gateway,  oldGw, sizeof(cfg.gateway)-1);  cfg.gateway[sizeof(cfg.gateway)-1]='\0';
      strncpy(cfg.subnet,   oldSn, sizeof(cfg.subnet)-1);   cfg.subnet[sizeof(cfg.subnet)-1]='\0';
      strncpy(cfg.dns1,     oldD1, sizeof(cfg.dns1)-1);     cfg.dns1[sizeof(cfg.dns1)-1]='\0';
      strncpy(cfg.dns2,     oldD2, sizeof(cfg.dns2)-1);     cfg.dns2[sizeof(cfg.dns2)-1]='\0';
      if (oldStatic) applyStaticConfig(); else clearStaticConfig();
      connectSTA(false);
      msgColor = "#dc3545";
      msgText  = "Wi-Fi connection failed. Old settings kept.";
    }

    String resultScript =
      "<script>"
      "let overlay=document.createElement('div');"
      "overlay.style.position='fixed';overlay.style.top='0';overlay.style.left='0';"
      "overlay.style.width='100%';overlay.style.height='100%';"
      "overlay.style.background='rgba(0,0,0,0.6)';"
      "overlay.style.display='flex';overlay.style.justifyContent='center';overlay.style.alignItems='center';"
      "overlay.style.zIndex='9999';"
      "let box=document.createElement('div');"
      "box.style.background='" + msgColor + "';box.style.color='white';"
      "box.style.padding='30px 60px';box.style.borderRadius='16px';box.style.fontSize='22px';"
      "box.style.fontFamily='Segoe UI';box.style.boxShadow='0 8px 30px rgba(0,0,0,0.5)';"
      "box.style.textAlign='center';box.innerHTML='<b>" + msgText + "</b>';"
      "overlay.appendChild(box);document.body.appendChild(overlay);"
      "setTimeout(()=>{overlay.style.transition='opacity 0.8s';overlay.style.opacity='0';"
      "setTimeout(()=>{location='/'},1000);},2500);"
      "</script>";

    server.send(200, "text/html",
      "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:'Segoe UI';text-align:center;margin-top:60px;font-size:20px;}</style>"
      "</head><body>"
      "<div style='position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);"
      "display:flex;align-items:center;justify-content:center;color:#fff;font-size:24px;'>"
      "<div>Testing Wi-Fi... please wait</div></div>"
      + resultScript + "</body></html>");
    return;
  }

  String html;
  html.reserve(4000);
  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Wi-Fi Setup</title>";
  html += commonStyle();
  html += "</head><body>";
  html += "<header><h1>Wi-Fi Setup &amp; Test</h1></header><div class='container'>";
  html += "<form method='POST' onsubmit='showOverlay()' class='card'>";
  html += "<label>Router SSID</label><input name='ssid' value='" + String(ntpSSID) + "' required>";
  html += "<label>Router Password</label><input name='pass' type='password' value='" + String(ntpPassword) + "' required>";
  html += "<hr><label><input type='checkbox' name='useStatic' " + String(cfg.useStatic ? "checked" : "") + " style='width:auto;'> Use static IP on router LAN</label>";
  html += "<label>Static IP</label><input name='staticIP' value='" + String(cfg.staticIP) + "' placeholder='192.168.1.50'>";
  html += "<label>Gateway</label><input name='gateway' value='" + String(cfg.gateway) + "' placeholder='192.168.1.1'>";
  html += "<label>Subnet</label><input name='subnet' value='" + String(cfg.subnet) + "' placeholder='255.255.255.0'>";
  html += "<label>DNS 1</label><input name='dns1' value='" + String(cfg.dns1) + "' placeholder='8.8.8.8 (optional)'>";
  html += "<label>DNS 2</label><input name='dns2' value='" + String(cfg.dns2) + "' placeholder='1.1.1.1 (optional)'>";
  html += "<button class='button' type='submit'>Test &amp; Save</button>";
  html += "</form>";
  html += "<a class='button' href='/'>Back to Dashboard</a>";
  html += "<div id='overlay' style='position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.6);display:none;justify-content:center;align-items:center;color:#fff;font-size:24px;'><div>Connecting...</div></div>";
  html += "<script>function showOverlay(){document.getElementById('overlay').style.display='flex';}</script>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ---------------- Update time ----------------
void handleUpdateTime() {
  if (!(server.hasArg("y") && server.hasArg("M") && server.hasArg("D") &&
        server.hasArg("h") && server.hasArg("m") && server.hasArg("s"))) {
    server.send(400, "text/plain", "missing params");
    return;
  }
  DateTime dt(server.arg("y").toInt(), server.arg("M").toInt(), server.arg("D").toInt(),
              server.arg("h").toInt(), server.arg("m").toInt(), server.arg("s").toInt());
  updateAllRTCs(dt);
  mqttPublishStatus(true);
  server.send(200, "text/plain", "ok");
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Smart Interval Controller booting ===");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  if (RELAY_ACTIVE_LOW) { digitalWrite(RELAY_PIN, HIGH); digitalWrite(LED_PIN, HIGH); }
  else                  { digitalWrite(RELAY_PIN, LOW);  digitalWrite(LED_PIN, LOW);  }

  initFS();
  loadConfig();
  buildTopics();

  setenv("TZ", "PKT-5", 1);
  tzset();

  startAP();

  Wire.begin();
  bool rtcOK = rtcAvailable() && rtc.begin();
  if (rtcOK) {
    Serial.println("[RTC] DS3231 found");
    DateTime rnow = rtc.now();
    if (isValidDateTime(rnow)) {
      Serial.printf("[RTC] Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                    rnow.year(), rnow.month(), rnow.day(),
                    rnow.hour(), rnow.minute(), rnow.second());
      timeval tv; tv.tv_sec = rnow.unixtime(); tv.tv_usec = 0;
      settimeofday(&tv, nullptr);
    }
  } else {
    Serial.println("[RTC] DS3231 NOT found");
  }

  if (strlen(ntpSSID) > 0 && strcmp(ntpSSID, "your_ntp_SSID") != 0) {
    connectSTA(false);
    unsigned long t0 = millis();
    while (millis() - t0 < 10000 && staState != STA_CONNECTED && staState != STA_FAILED) {
      delay(100);
      wifiWatchdog();
    }
    if (staState == STA_CONNECTED) syncRTCfromNTP();
  }

  // MQTT first attempt after STA
  if (cfg.mqttEnabled) {
    buildTopics();
    mqttReconnect();
  }

  server.on("/",             HTTP_GET,  handleRoot);
  server.on("/schedule",     HTTP_GET,  handleSchedule);
  server.on("/schedule",     HTTP_POST, handleSchedule);
  server.on("/override",     HTTP_GET,  handleOverridePage);
  server.on("/override",     HTTP_POST, handleOverridePage);
  server.on("/wifi",         HTTP_GET,  handleWiFiPage);
  server.on("/wifi",         HTTP_POST, handleWiFiPage);
  server.on("/mqtt",         HTTP_GET,  handleMqttPage);
  server.on("/mqtt",         HTTP_POST, handleMqttPage);
  server.on("/updatetime",   HTTP_GET,  handleUpdateTime);
  server.on("/network",      HTTP_GET,  handleNetworkPage);
  server.on("/reboot",       HTTP_GET,  handleReboot);
  server.on("/reboot",       HTTP_POST, handleReboot);

  server.begin();
  Serial.println("[HTTP] Server started");
  Serial.printf("[INFO] AP SSID: %s  |  AP IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ---------------- Loop ----------------
unsigned long lastLogic = 0;
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  wifiWatchdog();
  mqttLoop();

  unsigned long nowms = millis();
  if (nowms - lastLogic >= 1000UL) {
    lastLogic = nowms;
    applyLogicOnce();
  }

  if (millis() - lastRTCcheck > RTC_CHECK_INTERVAL) {
    lastRTCcheck = millis();
    checkRTCs();
  }

  if (ntpEverSynced && (millis() - lastNtpSyncMillis > NTP_RESYNC_INTERVAL)) {
    Serial.println("[NTP] Periodic resync...");
    syncRTCfromNTP();
  }
}
