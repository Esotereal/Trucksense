/*****************************************************************************************
  TruckSense LTE Uplink (LilyGO T-SIM7670G S3) - v2  (SELF-HEALING, BLE REMOVED)

  CHANGES FROM v1 (the diagnosis: boots=1 / 16.8h uptime proved the flapping was pure
  MQTT reconnects, NOT reboots/brownouts -- so this is a software-only tuning fix):
    - REMOVED all Bluetooth/BLE code. Not used for debugging anymore, and dropping the
      BLE stack frees a big chunk of the ESP32-S3's RAM, which helps long-run stability.
    - keepAlive restored to 60s (v1 used 30s, too twitchy for LTE jitter -> false
      disconnects -> the flapping you saw). 60s is the value your original build ran
      stable on for hours.
    - REMOVED the 10-minute proactive reconnect. It was belt-and-suspenders that, over
      a cellular modem, showed up as real "offline" blips and caused churn. The watchdog,
      the 4-minute reboot-on-prolonged-outage, and normal drop detection cover it without
      the self-inflicted flapping.

  KEPT (the stuff doing real good):
    - Hardware watchdog: reboots if loop() ever hangs.
    - Reboot escalation: clean restart if offline > REBOOT_IF_DOWN_MS.
    - Non-blocking, bounded loop (no infinite reconnect loops, capped UART ingest).
    - Diagnostics in the heartbeat: boots=<count> last_reset=<reason>.
    - Throttled publishing (data savings) + sensor value sanity-ranging.

  Sensor UART protocol:  V=13.02,T=24.6\n   /   V=12.88,T=nan\n
  Wiring:  LilyGO GPIO44 (RXD) <- Sensor ESP32 TX ;  shared GND mandatory
******************************************************************************************/

#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7600          // SIM7670G speaks the SIM7600 AT dialect in TinyGSM
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#include "esp_task_wdt.h"               // hardware watchdog
#include "esp_system.h"                 // esp_reset_reason()

#include "secrets.h"

// ------------------- MQTT Topics -------------------
static const char* TOPIC_VOLTAGE   = "trucksense/hino01/voltage";
static const char* TOPIC_TEMP_OUT  = "trucksense/hino01/temp_outside";
static const char* TOPIC_STATUS    = "trucksense/hino01/status";      // online/offline (LWT)
static const char* TOPIC_NET_HEALTH= "trucksense/hino01/wifi";        // LTE health text (legacy name)
static const char* TOPIC_SENSORLINK= "trucksense/hino01/sensor_link"; // sensor UART age + last values
static const char* TOPIC_HEARTBEAT = "trucksense/hino01/heartbeat";   // uptime + sensor age + diag

// ------------------- APN + MQTT config -------------------------
static const char* APN       = SECRET_APN;
static const char* APN_USER  = SECRET_APN_USER;
static const char* APN_PASS  = SECRET_APN_PASS;

static const char* MQTT_BROKER = SECRET_MQTT_BROKER;   // VPS IP
static const int   MQTT_PORT   = SECRET_MQTT_PORT;     // 1883
static const char* MQTT_USER   = SECRET_MQTT_USER;
static const char* MQTT_PASS   = SECRET_MQTT_PASS;
static const char* MQTT_CLIENT = SECRET_MQTT_CLIENT_ID;

// ------------------- LilyGO modem pins --------------------------
#define MODEM_RX_PIN       10
#define MODEM_TX_PIN       11
#define MODEM_PWRKEY_PIN   18
#define MODEM_DTR_PIN       9
#define MODEM_BAUD     115200

// ------------------- UART link from Sensor ESP32 ----------------
#define SENSOR_UART_RX_PIN 44
#define SENSOR_UART_TX_PIN 43
#define SENSOR_UART_BAUD   115200

// ------------------- Optional LED -------------------------------
#define LED_PIN  2  // set to -1 if not used

// ===================================================================
//  SELF-HEAL / WATCHDOG TUNING  (safe to adjust)
// ===================================================================
static const uint32_t      WDT_TIMEOUT_S          = 180;    // reboot if loop() hangs this long
static const unsigned long PUBLISH_INTERVAL_MS    = 20000;  // voltage/temp publish cadence (data savings)
static const unsigned long MQTT_RETRY_BACKOFF_MS  = 5000;   // wait between reconnect attempts while down
static const unsigned long REBOOT_IF_DOWN_MS      = 240000; // 4 min continuously offline -> full reboot
static const size_t        UART_MAX_BYTES_PER_LOOP= 256;    // cap sensor bytes per loop pass
static const int           MQTT_KEEPALIVE_S       = 60;     // tolerant of LTE jitter (v1's 30 was too tight)

// ------------------- Modem / MQTT objects -----------------------
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient net(modem);
PubSubClient mqtt(net);

// ------------------- Sensor UART -------------------------------
HardwareSerial SensorLink(2);
String sensorBuf;

float lastV = NAN;
float lastT = NAN;
unsigned long lastSensorPacketMs = 0;

// ------------------- Timers / state ----------------------------
unsigned long lastNetReportMs         = 0;
unsigned long lastSensorLinkPublishMs = 0;
unsigned long lastHeartbeatMs         = 0;
unsigned long lastPublishMs           = 0;   // throttles voltage/temp
unsigned long lastMqttOkMs            = 0;   // last time mqtt.connected() was true
unsigned long lastMqttRetryMs         = 0;   // last reconnect attempt

bool gNetOk  = false;   // cached LTE state (for LED; avoids spamming modem with AT cmds)
bool gMqttOk = false;   // cached MQTT state

const unsigned long NET_REPORT_INTERVAL_MS      = 180000; // 3 min
const unsigned long SENSORLINK_PUB_INTERVAL_MS  = 15000;  // 15 sec
const unsigned long HEARTBEAT_INTERVAL_MS       = 10000;  // 10 sec

// Reboot diagnostics. RTC memory survives software/watchdog/brownout resets but is
// cleared on a true power loss -- which is how we tell "wedged & self-rebooted" from
// "lost power entirely".
RTC_DATA_ATTR uint32_t rebootCount = 0;
String gResetReason = "?";

String resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_SW:        return "SW";        // our own ESP.restart() self-heal
    case ESP_RST_PANIC:     return "PANIC";     // a crash
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";  // our watchdog caught a hang
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";  // power dipped (loose wire / cranking / weak buck)
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_EXT:       return "EXT";
    default:                return "UNKNOWN";
  }
}

static inline void logMsg(const String& m) { Serial.println(m); }   // serial-only now (BLE removed)

// ------------------- LED patterns -------------------------------
enum LedMode { LED_BOOT, LED_NO_NET, LED_NO_MQTT, LED_OK };
LedMode ledMode = LED_BOOT;
unsigned long lastLedTick = 0;
const unsigned long LED_TICK_INTERVAL_MS = 150;

static inline void safeLedWrite(bool on) {
  if (LED_PIN < 0) return;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// Uses cached flags only -- no modem AT-commands here.
void updateLedMode() {
  if (!gNetOk)        ledMode = LED_NO_NET;
  else if (!gMqttOk)  ledMode = LED_NO_MQTT;
  else                ledMode = LED_OK;
}

void ledStateMachine() {
  static int phase = 0;
  unsigned long now = millis();
  if (now - lastLedTick < LED_TICK_INTERVAL_MS) return;
  lastLedTick = now;

  updateLedMode();
  switch (ledMode) {
    case LED_BOOT:    if (LED_PIN >= 0) safeLedWrite(!digitalRead(LED_PIN)); break;
    case LED_NO_NET:  phase = (phase + 1) % 8;  safeLedWrite(phase < 2); break;        // fast blink = no LTE
    case LED_NO_MQTT: phase = (phase + 1) % 12; safeLedWrite(phase == 0 || phase == 2); break; // double-blink = no broker
    case LED_OK:      safeLedWrite(true); break;                                       // solid = connected
  }
}

// ------------------- Watchdog ----------------------------------
void watchdogInit() {
  // Task-WDT API differs between Arduino-ESP32 v2.x and v3.x; guard for both.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = (uint32_t)(WDT_TIMEOUT_S * 1000),
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
#endif
  logMsg("Watchdog armed (" + String(WDT_TIMEOUT_S) + "s)");
}

static inline void feedWatchdog() { esp_task_wdt_reset(); }

// ------------------- Modem bring-up -----------------------------
void modemPowerOnPulse() {
  pinMode(MODEM_DTR_PIN, OUTPUT);  digitalWrite(MODEM_DTR_PIN, LOW);
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);  delay(100);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH); delay(1200);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);  delay(2000);
}

bool connectCellular() {
  logMsg("Starting modem UART");
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(300);
  modemPowerOnPulse();

  logMsg("Initializing modem");
  if (!modem.init()) { logMsg("modem.init failed"); return false; }
  logMsg("Modem: " + modem.getModemInfo());

  logMsg("Waiting for network");
  if (!modem.waitForNetwork(60000L)) { logMsg("No network (timeout)"); return false; }
  logMsg("Network connected");

  logMsg("Connecting APN");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { logMsg("APN connect failed"); return false; }
  logMsg("Data connected");
  logMsg("IP: " + modem.localIP().toString());
  return true;
}

// Lightweight runtime check/repair of the LTE layers. Only called WHILE MQTT is down,
// so there's no live data stream to clash with.
bool ensureCellular() {
  if (!modem.isNetworkConnected()) {
    logMsg("Network down; waiting up to 45s");
    if (!modem.waitForNetwork(45000L)) { logMsg("Network timeout"); return false; }
  }
  if (!modem.isGprsConnected()) {
    logMsg("Data session down; reconnecting APN");
    if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { logMsg("APN reconnect failed"); return false; }
    logMsg("Data reconnected, IP: " + modem.localIP().toString());
  }
  return true;
}

// Single bounded MQTT connect attempt.
bool tryConnectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setSocketTimeout(10);

  logMsg("MQTT connecting...");
  bool ok = mqtt.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS, TOPIC_STATUS, 1, true, "offline");
  if (ok) { logMsg("MQTT connected"); mqtt.publish(TOPIC_STATUS, "online", true); }
  else    { logMsg("MQTT connect failed, state=" + String(mqtt.state())); }
  return ok;
}

// ------------------- Sensor parsing -----------------------------
bool parseSensorLine(const String& line, float& v, float& t, bool& hasTemp) {
  int iv = line.indexOf("V=");
  int it = line.indexOf(",T=");
  if (iv < 0 || it <= iv) return false;
  String sv = line.substring(iv + 2, it);
  String st = line.substring(it + 3);  st.trim();
  v = sv.toFloat();
  if (st.equalsIgnoreCase("nan")) { hasTemp = false; t = NAN; }
  else                            { hasTemp = true;  t = st.toFloat(); }
  return true;
}

// Pure ingest: read UART, parse, cache latest values. Bounded so it can never hog loop().
void handleSensorUart() {
  size_t processed = 0;
  while (SensorLink.available() && processed < UART_MAX_BYTES_PER_LOOP) {
    char c = (char)SensorLink.read();
    processed++;
    if (c == '\n') {
      sensorBuf.trim();
      float v, t; bool hasTemp;
      if (parseSensorLine(sensorBuf, v, t, hasTemp)) {
        if (v > 0.0f && v < 20.0f)              lastV = v;   // sanity-range (truck UART is noisy)
        if (hasTemp && t > -50.0f && t < 90.0f) lastT = t;
        lastSensorPacketMs = millis();
      } else {
        logMsg("Bad sensor packet: " + sensorBuf);
      }
      sensorBuf = "";
    } else if (c != '\r') {
      sensorBuf += c;
      if (sensorBuf.length() > 120) sensorBuf = "";
    }
  }
}

// ------------------- Publishers ---------------------------------
void publishSensorData() {
  if (!isnan(lastV)) mqtt.publish(TOPIC_VOLTAGE, String(lastV, 3).c_str(), true);
  if (!isnan(lastT)) mqtt.publish(TOPIC_TEMP_OUT, String(lastT, 3).c_str(), true);
  logMsg("Pub V=" + String(lastV, 3) + " T=" + (isnan(lastT) ? String("nan") : String(lastT, 2)));
}

void publishNetHealth() {
  int16_t csq = modem.getSignalQuality();
  String payload = String("CSQ=") + csq + " IP=" + modem.localIP().toString();
  mqtt.publish(TOPIC_NET_HEALTH, payload.c_str(), true);
  logMsg("Net health: " + payload);
}

void publishSensorLinkStatus() {
  unsigned long now = millis();
  unsigned long age = (lastSensorPacketMs == 0) ? 999999UL : (now - lastSensorPacketMs);
  String msg = "age_ms=" + String(age);
  if (!isnan(lastV)) msg += " lastV=" + String(lastV, 2);
  if (!isnan(lastT)) msg += " lastT=" + String(lastT, 1);
  mqtt.publish(TOPIC_SENSORLINK, msg.c_str(), true);
  logMsg("Sensor link: " + msg);
}

void publishHeartbeat() {
  unsigned long now = millis();
  unsigned long uptime_s = now / 1000UL;
  unsigned long age = (lastSensorPacketMs == 0) ? 999999UL : (now - lastSensorPacketMs);
  String hb = "uptime_s="       + String(uptime_s) +
              " sensor_age_ms=" + String(age) +
              " boots="         + String(rebootCount) +
              " last_reset="    + gResetReason;
  mqtt.publish(TOPIC_HEARTBEAT, hb.c_str(), true);
  logMsg("Heartbeat: " + hb);
}

// ------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  rebootCount++;
  gResetReason = resetReasonStr();

  if (LED_PIN >= 0) { pinMode(LED_PIN, OUTPUT); safeLedWrite(false); }
  ledMode = LED_BOOT;

  Serial.println("\nTruckSense LTE uplink boot (v2)");
  Serial.println("Reset reason: " + gResetReason + "   boot #" + String(rebootCount));

  watchdogInit();

  SensorLink.begin(SENSOR_UART_BAUD, SERIAL_8N1, SENSOR_UART_RX_PIN, SENSOR_UART_TX_PIN);
  logMsg("Sensor UART ready (RX=" + String(SENSOR_UART_RX_PIN) + ")");

  int cellTries = 0;
  while (!connectCellular()) {
    feedWatchdog();
    logMsg("Cellular init failed, retry in 8s");
    delay(8000);
    if (++cellTries >= 10) { logMsg("Cellular unrecoverable -> reboot"); delay(200); ESP.restart(); }
  }
  gNetOk = true;

  if (tryConnectMQTT()) gMqttOk = true;

  lastMqttOkMs = millis();   // start the offline timer fresh
  logMsg("System ready");
}

// ------------------- Loop ---------------------------------------
void loop() {
  feedWatchdog();        // pet the watchdog every pass; if loop hangs, chip reboots
  ledStateMachine();     // cached state only

  unsigned long now = millis();

  if (mqtt.connected()) {
    gMqttOk = true; gNetOk = true;
    lastMqttOkMs = now;
    mqtt.loop();         // service keepalive (60s) -> detects a truly dead link
  } else {
    gMqttOk = false;
    // Investigate lower layers ONLY while MQTT is down (no live data to disturb).
    if (now - lastMqttRetryMs >= MQTT_RETRY_BACKOFF_MS) {
      lastMqttRetryMs = now;
      if (ensureCellular()) {
        gNetOk = true;
        if (tryConnectMQTT()) { gMqttOk = true; lastMqttOkMs = now; }
      } else {
        gNetOk = false;
      }
    }
    // Escalation: offline too long -> clean reboot.
    if (now - lastMqttOkMs >= REBOOT_IF_DOWN_MS) {
      logMsg("Offline > " + String(REBOOT_IF_DOWN_MS / 1000) + "s -> rebooting to self-heal");
      delay(200);
      ESP.restart();
    }
  }

  handleSensorUart();    // bounded ingest, always runs

  // Timed publishes (only when connected).
  if (mqtt.connected()) {
    if (now - lastPublishMs >= PUBLISH_INTERVAL_MS)              { lastPublishMs = now;           publishSensorData(); }
    if (now - lastSensorLinkPublishMs >= SENSORLINK_PUB_INTERVAL_MS) { lastSensorLinkPublishMs = now; publishSensorLinkStatus(); }
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)         { lastHeartbeatMs = now;         publishHeartbeat(); }
    if (now - lastNetReportMs >= NET_REPORT_INTERVAL_MS)        { lastNetReportMs = now;         publishNetHealth(); }
  }
}
