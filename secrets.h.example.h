# ====================================================================
#  TruckSense .gitignore
#  MOST IMPORTANT LINE: secrets.h is never committed. Commit
#  secrets.h.example instead, and keep your real credentials local.
# ====================================================================

# --- Secrets / credentials (NEVER commit) ---
#pragma once

#define SECRET_WIFI_SSID       "INSERTHERE"
#define SECRET_WIFI_PASS       "INSERTHERE"

#define SECRET_MQTT_BROKER     "IPADDRESS"
#define SECRET_MQTT_PORT       0000
#define SECRET_MQTT_USER       "INSERTHERE"
#define SECRET_MQTT_PASS       "INSERTHERE"

#define SECRET_MQTT_CLIENT_ID  "INSERTHERE"
