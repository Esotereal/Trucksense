/*****************************************************************************************
  TruckSense Sensor Node (HiLetGo ESP32) - v2  (with hardware watchdog)

  Role: reads battery voltage (ADC) + outside temp (DS18B20) and sends "V=...,T=..."
  to the LilyGO LTE board over UART2 every 5 seconds.

  CHANGE FROM v1:
    - Added a HARDWARE WATCHDOG. This board has no remote reset path (the UART to the
      LilyGO is effectively one-way), so the only way it can recover from a hang -- for
      example a locked OneWire bus, a known DS18B20 failure mode -- is to watch itself
      and reboot. This closes the last self-heal gap (Failure #2: this board went silent
      for ~3 days). After a reboot it simply resumes sending; the LilyGO keeps publishing.

  Wiring (unchanged):
    ADC_PIN 35  <- battery voltage divider (0-25V module)
    GPIO4       <- DS18B20 data (4.7k pull-up to 3.3V)
    GPIO17 (TX) -> LilyGO GPIO44 (RX) ;  shared GND mandatory
******************************************************************************************/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "esp_task_wdt.h"     // hardware watchdog
#include "esp_system.h"       // esp_reset_reason()

// ---- Sensor wiring ----
#define ADC_PIN         35    // ADC1, stable on ESP32
#define ONE_WIRE_PIN     4    // DS18B20 data
const float VOLTAGE_MULTIPLIER = 5.35f;   // calibrated against a multimeter

// ---- UART link to LilyGO ----
HardwareSerial Link(2);        // UART2
static const int LINK_TX = 17; // -> LilyGO GPIO44 (RX)
static const int LINK_RX = 16; // optional reverse line

// ---- Watchdog ----
// Loop period is ~5.8s (5s delay + sensor reads), so the timeout must exceed that with
// margin. 15s is comfortable and still catches any indefinite hang (e.g. a locked
// OneWire bus) and reboots the board.
static const uint32_t WDT_TIMEOUT_S = 15;

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds(&oneWire);

String resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_TASK_WDT: return "TASK_WDT";   // watchdog caught a hang
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";   // power dipped
    case ESP_RST_EXT:      return "EXT";
    default:               return "UNKNOWN";
  }
}

float readVoltageAvg(int samples = 20) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(ADC_PIN);
    delay(2);
  }
  float raw = sum / (float)samples;
  return (raw / 4095.0f) * 3.3f * VOLTAGE_MULTIPLIER;
}

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
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\nSensor ESP32 boot. Last reset: " + resetReasonStr());

  analogReadResolution(12);
  ds.begin();
  Link.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);

  watchdogInit();
  Serial.println("Sensor ESP32 ready. Sending V/T every 5s over UART2 (TX=17). Watchdog armed (" + String(WDT_TIMEOUT_S) + "s).");
}

void loop() {
  esp_task_wdt_reset();        // pet the watchdog each pass; a hang here -> auto-reboot

  ds.requestTemperatures();
  float tC = ds.getTempCByIndex(0);
  float v  = readVoltageAvg();

  // Packet: V=13.020,T=24.600   (or T=nan if the DS18B20 isn't responding)
  if (tC == DEVICE_DISCONNECTED_C) {
    Link.printf("V=%.2f,T=nan\n", v);
    Serial.printf("TX: V=%.2f,T=nan\n", v);
  } else {
    Link.printf("V=%.3f,T=%.3f\n", v, tC);
    Serial.printf("TX: V=%.3f,T=%.3f\n", v, tC);
  }

  delay(5000);
}
