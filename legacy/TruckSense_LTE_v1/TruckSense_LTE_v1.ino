/*****************************************************************************************
  TruckSense LTE Uplink (LilyGO T-SIM7670G S3) - SELF-HEALING BUILD

  WHAT CHANGED vs the original (and why):
  -------------------------------------------------------------------------------------
  1. HARDWARE WATCHDOG (esp_task_wdt)
       If loop() ever hangs (modem AT call stuck, reconnect spin, etc.) the chip
       reboots itself after WDT_TIMEOUT_S. This is the ultimate backstop for any wedge.

  2. STALE-SOCKET SELF-HEAL  <-- this is the bug that caused "boards on, HA frozen"
       The old code published every few seconds, so PubSubClient never had an idle gap
       to send a keepalive ping, so it never noticed when the link silently died. The
       device kept thinking it was connected and published into a dead socket forever.
       Fix: (a) publishing is throttled, and (b) we PROACTIVELY tear down and rebuild the
       MQTT connection every PROACTIVE_RECONNECT_MS, which forces any silently-dead link
       to be detected and re-established.

  3. NON-BLOCKING RECONNECT
       The old connectMQTT() had an infinite while() loop that could trap the whole
       program. Now every reconnect is a single bounded attempt; the loop keeps running.

  4. REBOOT ESCALATION
       If we stay disconnected for REBOOT_IF_DOWN_MS, we do a clean ESP.restart() so the
       device comes back from a fresh boot instead of sitting dark.

  5. SAFER MODEM POLLING
       The old LED code hammered the modem with status AT-commands ~6x/second while the
       data link was live (a likely contributor to wedging). Now the LED uses cached
       state and we only query the modem's lower layers WHILE MQTT is already down.

  6. BOUNDED SENSOR UART
       handleSensorUart() now processes a capped number of bytes per pass so a chatty
       sensor can't starve the rest of loop() (this is what froze the heartbeat topic).

  7. DIAGNOSTICS IN THE HEARTBEAT
       Heartbeat now reports boots=<count> and last_reset=<reason>. After any future
       outage you'll see WHY it rebooted: BROWNOUT (power dip / loose wire),
       TASK_WDT (a hang we caught), PANIC (a crash), or SW (our own self-heal reboot).
       A high boots count that keeps climbing = it's wedge-rebooting; a count that
       resets to 1 = it lost power entirely. This tells us brownout vs. wedge for real.

  Sensor UART protocol (unchanged):  V=13.02,T=24.6\n   /   V=12.88,T=nan\n
  Wiring (unchanged):
    LilyGO GPIO44 (RXD) <- Sensor ESP32 TX ;  shared GND mandatory
******************************************************************************************/

#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7600          // SIM7670G speaks the SIM7600 AT dialect in TinyGSM
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "esp_task_wdt.h"               // hardware watchdog
#include "esp_system.h"                 // esp_reset_reason()

#include "secrets.h"

// ------------------- MQTT Topics (base telemetry) -------------------
static const char* TOPIC_VOLTAGE   = "trucksense/hino01/voltage";
static const char* TOPIC_TEMP_OUT  = "trucksense/hino01/temp_outside";
static const char* TOPIC_STATUS    = "trucksense/hino01/status";      // online/offline ONLY (LWT)
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
static const uint32_t      WDT_TIMEOUT_S          = 180;     // reboot if loop() hangs this long (must exceed any single blocking modem call)
static const unsigned long PUBLISH_INTERVAL_MS    = 20000;   // how often to publish voltage/temp (was ~every packet). 20s = big data savings + lets keepalive work
static const unsigned long MQTT_RETRY_BACKOFF_MS  = 5000;    // wait between reconnect attempts while down
static const unsigned long PROACTIVE_RECONNECT_MS = 600000;  // 10 min: refresh the MQTT socket to kill silently-stale links
static const unsigned long REBOOT_IF_DOWN_MS      = 240000;  // 4 min continuously offline -> full reboot
static const size_t        UART_MAX_BYTES_PER_LOOP= 256;     // cap sensor bytes per loop pass so it can't starve everything else

// ------------------- BLE (field debugging) ----------------------
#define BLE_SERVICE_UUID   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic* bleTx = nullptr;
bool bleConnected = false;

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

static inline void bleLog(const String& msg) {
  if (bleConnected && bleTx) {
    bleTx->setValue(msg.c_str());
    bleTx->notify();
  }
}

static inline void logBoth(const String& msg) {
  Serial.println(msg);
  bleLog(msg);
}

void setupBLE() {
  BLEDevice::init("TruckSense-BLE");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  BLEService* service = server->createService(BLE_SERVICE_UUID);

  bleTx = service->createCharacteristic(
    BLE_CHAR_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  bleTx->addDescriptor(new BLE2902());

  BLECharacteristic* rx = service->createCharacteristic(
    BLE_CHAR_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  (void)rx;

  service->start();
  BLEDevice::startAdvertising();
  logBoth("BLE ready");
}

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
unsigned long lastProactiveReconnectMs= 0;   // last forced socket refresh

bool gNetOk  = false;   // cached LTE state (for LED + logic; avoids spamming modem with AT cmds)
bool gMqttOk = false;   // cached MQTT state

const unsigned long NET_REPORT_INTERVAL_MS      = 180000; // 3 min
const unsigned long SENSORLINK_PUB_INTERVAL_MS  = 15000;  // 15 sec
const unsigned long HEARTBEAT_INTERVAL_MS       = 10000;  // 10 sec

// Reboot diagnostics. RTC memory survives software/watchdog/brownout resets,
// but is cleared on a true power loss -- which is exactly what lets us tell the
// difference between "it wedged and self-rebooted" vs "it lost power".
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

// ------------------- LED patterns -------------------------------
enum LedMode { LED_BOOT, LED_NO_NET, LED_NO_MQTT, LED_OK };
LedMode ledMode = LED_BOOT;
unsigned long lastLedTick = 0;
const unsigned long LED_TICK_INTERVAL_MS = 150;

static inline void safeLedWrite(bool on) {
  if (LED_PIN < 0) return;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// Uses cached flags only -- no modem AT-commands here (that used to clash with the data link).
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
    case LED_BOOT:
      if (LED_PIN >= 0) safeLedWrite(!digitalRead(LED_PIN));
      break;
    case LED_NO_NET:                       // fast blink  = no LTE
      phase = (phase + 1) % 8;
      safeLedWrite(phase < 2);
      break;
    case LED_NO_MQTT:                      // double-blink = LTE ok, broker not connected
      phase = (phase + 1) % 12;
      safeLedWrite(phase == 0 || phase == 2);
      break;
    case LED_OK:                           // solid ON    = fully connected & publishing
      safeLedWrite(true);
      break;
  }
}

// ------------------- Watchdog ----------------------------------
void watchdogInit() {
  // The task-WDT API changed between Arduino-ESP32 v2.x and v3.x, so we guard it.
  // If this block ever fails to compile, tell me your Arduino-ESP32 core version.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = (uint32_t)(WDT_TIMEOUT_S * 1000),
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdtCfg);   // already started by the core -> just set our timeout
  }
  esp_task_wdt_add(NULL);                // watch the loop task
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
#endif
  logBoth("Watchdog armed (" + String(WDT_TIMEOUT_S) + "s)");
}

static inline void feedWatchdog() { esp_task_wdt_reset(); }

// ------------------- Modem bring-up -----------------------------
void modemPowerOnPulse() {
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(1200);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);

  delay(2000);
}

// Full modem bring-up (used at boot). Returns true once data is connected.
bool connectCellular() {
  logBoth("Starting modem UART");
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(300);

  modemPowerOnPulse();

  logBoth("Initializing modem");
  if (!modem.init()) { logBoth("modem.init failed"); return false; }

  logBoth("Modem: " + modem.getModemInfo());

  logBoth("Waiting for network");
  if (!modem.waitForNetwork(60000L)) { logBoth("No network (timeout)"); return false; }
  logBoth("Network connected");

  logBoth("Connecting APN");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { logBoth("APN connect failed"); return false; }

  logBoth("Data connected");
  logBoth("IP: " + modem.localIP().toString());
  return true;
}

// Lightweight runtime check/repair of the LTE layers.
// Only called WHILE MQTT is down, so there is no live data stream to clash with.
bool ensureCellular() {
  if (!modem.isNetworkConnected()) {
    logBoth("Network down; waiting up to 45s");
    if (!modem.waitForNetwork(45000L)) { logBoth("Network timeout"); return false; }
  }
  if (!modem.isGprsConnected()) {
    logBoth("Data session down; reconnecting APN");
    if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { logBoth("APN reconnect failed"); return false; }
    logBoth("Data reconnected, IP: " + modem.localIP().toString());
  }
  return true;
}

// ------------------- MQTT connect (single bounded attempt) ------
bool tryConnectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(10);

  logBoth("MQTT connecting...");
  bool ok = mqtt.connect(
    MQTT_CLIENT, MQTT_USER, MQTT_PASS,
    TOPIC_STATUS, 1, true, "offline"   // LWT: broker publishes "offline" if we vanish
  );

  if (ok) {
    logBoth("MQTT connected");
    mqtt.publish(TOPIC_STATUS, "online", true);
  } else {
    logBoth("MQTT connect failed, state=" + String(mqtt.state()));
  }
  return ok;
}

// ------------------- Sensor parsing -----------------------------
bool parseSensorLine(const String& line, float& v, float& t, bool& hasTemp) {
  int iv = line.indexOf("V=");
  int it = line.indexOf(",T=");
  if (iv < 0 || it <= iv) return false;

  String sv = line.substring(iv + 2, it);
  String st = line.substring(it + 3);
  st.trim();

  v = sv.toFloat();

  if (st.equalsIgnoreCase("nan")) { hasTemp = false; t = NAN; }
  else                            { hasTemp = true;  t = st.toFloat(); }
  return true;
}

// Pure ingest: read UART, parse, and cache the latest values. NO publishing here
// (publishing moved to the timed section of loop()). Bounded so it can never hog the loop.
void handleSensorUart() {
  size_t processed = 0;
  while (SensorLink.available() && processed < UART_MAX_BYTES_PER_LOOP) {
    char c = (char)SensorLink.read();
    processed++;

    if (c == '\n') {
      sensorBuf.trim();
      float v, t; bool hasTemp;
      if (parseSensorLine(sensorBuf, v, t, hasTemp)) {
        // Sanity-range the values before trusting them (a truck is electrically noisy,
        // and a garbled packet shouldn't publish a nonsense voltage/temperature).
        if (v > 0.0f && v < 20.0f)               lastV = v;
        if (hasTemp && t > -50.0f && t < 90.0f)  lastT = t;
        lastSensorPacketMs = millis();
      } else {
        logBoth("Bad sensor packet: " + sensorBuf);
      }
      sensorBuf = "";
    } else if (c != '\r') {
      sensorBuf += c;
      if (sensorBuf.length() > 120) sensorBuf = "";  // overflow guard
    }
  }
}

// ------------------- Publishers ---------------------------------
void publishSensorData() {
  if (!isnan(lastV)) mqtt.publish(TOPIC_VOLTAGE, String(lastV, 3).c_str(), true);
  if (!isnan(lastT)) mqtt.publish(TOPIC_TEMP_OUT, String(lastT, 3).c_str(), true);
  logBoth("Pub V=" + String(lastV, 3) + " T=" + (isnan(lastT) ? String("nan") : String(lastT, 2)));
}

void publishNetHealth() {
  int16_t csq = modem.getSignalQuality();
  String payload = String("CSQ=") + csq + " IP=" + modem.localIP().toString();
  mqtt.publish(TOPIC_NET_HEALTH, payload.c_str(), true);
  logBoth("Net health: " + payload);
}

void publishSensorLinkStatus() {
  unsigned long now = millis();
  unsigned long age = (lastSensorPacketMs == 0) ? 999999UL : (now - lastSensorPacketMs);

  String msg = "age_ms=" + String(age);
  if (!isnan(lastV)) msg += " lastV=" + String(lastV, 2);
  if (!isnan(lastT)) msg += " lastT=" + String(lastT, 1);

  mqtt.publish(TOPIC_SENSORLINK, msg.c_str(), true);
  logBoth("Sensor link: " + msg);
}

void publishHeartbeat() {
  unsigned long now = millis();
  unsigned long uptime_s = now / 1000UL;
  unsigned long age = (lastSensorPacketMs == 0) ? 999999UL : (now - lastSensorPacketMs);

  // Now includes reboot count + last reset reason -> future outages self-diagnose.
  String hb = "uptime_s="        + String(uptime_s) +
              " sensor_age_ms="  + String(age) +
              " boots="          + String(rebootCount) +
              " last_reset="     + gResetReason;
  mqtt.publish(TOPIC_HEARTBEAT, hb.c_str(), true);
  logBoth("Heartbeat: " + hb);
}

// ------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  rebootCount++;                       // count this boot
  gResetReason = resetReasonStr();     // why did we (re)boot?

  if (LED_PIN >= 0) { pinMode(LED_PIN, OUTPUT); safeLedWrite(false); }
  ledMode = LED_BOOT;

  Serial.println("\nTruckSense LTE uplink boot");
  Serial.println("Reset reason: " + gResetReason + "   boot #" + String(rebootCount));

  watchdogInit();                      // arm the hardware watchdog immediately
  setupBLE();

  SensorLink.begin(SENSOR_UART_BAUD, SERIAL_8N1, SENSOR_UART_RX_PIN, SENSOR_UART_TX_PIN);
  logBoth("Sensor UART ready (RX=" + String(SENSOR_UART_RX_PIN) + ")");

  // Bring up LTE. Each attempt is bounded; we feed the watchdog between tries and
  // give up (reboot) after enough failures rather than looping forever.
  int cellTries = 0;
  while (!connectCellular()) {
    feedWatchdog();
    logBoth("Cellular init failed, retry in 8s");
    delay(8000);
    if (++cellTries >= 10) {
      logBoth("Cellular unrecoverable -> reboot");
      delay(200);
      ESP.restart();
    }
  }
  gNetOk = true;

  if (tryConnectMQTT()) { gMqttOk = true; }

  unsigned long t = millis();
  lastMqttOkMs            = t;          // start the offline timer fresh
  lastProactiveReconnectMs= t;
  logBoth("System ready");
}

// ------------------- Loop ---------------------------------------
void loop() {
  feedWatchdog();        // <-- pet the watchdog every pass; if loop hangs, chip reboots
  ledStateMachine();     // uses cached state only

  unsigned long now = millis();

  // ---------- Connection management ----------
  if (mqtt.connected()) {
    gMqttOk = true;
    gNetOk  = true;          // if MQTT is up, the whole LTE stack is proven good
    lastMqttOkMs = now;
    mqtt.loop();             // service keepalive + any inbound

    // Proactive stale-link guard: periodically force a fresh socket. If the link had
    // silently died (broker gave up but modem still "thinks" it's open), reconnect
    // will fail and kick us into recovery below instead of publishing into the void.
    if (now - lastProactiveReconnectMs >= PROACTIVE_RECONNECT_MS) {
      lastProactiveReconnectMs = now;
      logBoth("Proactive MQTT refresh (stale-link guard)");
      mqtt.disconnect();
      gMqttOk = false;       // reconnect happens next pass
    }
  } else {
    gMqttOk = false;

    // Only poke the modem's lower layers while MQTT is down (no live data to disturb).
    if (now - lastMqttRetryMs >= MQTT_RETRY_BACKOFF_MS) {
      lastMqttRetryMs = now;
      if (ensureCellular()) {
        gNetOk = true;
        if (tryConnectMQTT()) {
          gMqttOk = true;
          lastMqttOkMs = now;
          lastProactiveReconnectMs = now;
        }
      } else {
        gNetOk = false;
      }
    }

    // Escalation: if we can't get back for REBOOT_IF_DOWN_MS, reboot clean.
    if (now - lastMqttOkMs >= REBOOT_IF_DOWN_MS) {
      logBoth("Offline > " + String(REBOOT_IF_DOWN_MS / 1000) + "s -> rebooting to self-heal");
      delay(200);
      ESP.restart();
    }
  }

  // ---------- Sensor ingest (bounded, always runs) ----------
  handleSensorUart();

  // ---------- Timed publishes (only when truly connected) ----------
  if (mqtt.connected()) {
    if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
      lastPublishMs = now;
      publishSensorData();
    }
    if (now - lastSensorLinkPublishMs >= SENSORLINK_PUB_INTERVAL_MS) {
      lastSensorLinkPublishMs = now;
      publishSensorLinkStatus();
    }
    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs = now;
      publishHeartbeat();
    }
    if (now - lastNetReportMs >= NET_REPORT_INTERVAL_MS) {
      lastNetReportMs = now;
      publishNetHealth();
    }
  }
}
