// ===========================================================
// telemetry.cpp — WiFi WebSocket telemetry
// Uses links2004/WebSockets library
// AP mode: node creates its own WiFi network
// Station mode: connects to existing router
// ===========================================================
#include "telemetry.h"
#include "config.h"
#include <WiFi.h>
#include <WebSocketsServer.h>

static WebSocketsServer _ws(TELEMETRY_PORT);
static bool    _ready   = false;
static uint8_t _clients = 0;

static void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      _clients++;
      {
        IPAddress ip = _ws.remoteIP(num);
        Serial.printf("[WS] Client #%d connected: %d.%d.%d.%d\n",
                      num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;

    case WStype_DISCONNECTED:
      if (_clients > 0) _clients--;
      Serial.printf("[WS] Client #%d disconnected.\n", num);
      break;

    case WStype_TEXT:
      // handle commands from client e.g. {"target":270}
      if (length > 0) {
        Serial.printf("[WS] Received: %s\n", payload);
      }
      break;

    default:
      break;
  }
}

void telemetryInit() {
#if WIFI_AP_MODE
  // access point mode: node acts as router
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WiFi] AP started: SSID='%s' IP=%d.%d.%d.%d\n",
                WIFI_SSID, ip[0], ip[1], ip[2], ip[3]);
#else
  // station mode: connect to existing network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to '%s'", WIFI_SSID);
  uint32_t startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] WARNING: No connection — continuing without WiFi.");
    return;
  }
#endif

  _ws.begin();
  _ws.onEvent(wsEvent);
  _ready = true;
  Serial.printf("[WS] WebSocket server started on port %d.\n", TELEMETRY_PORT);
  Serial.printf("[WS] Connect to: ws://<IP>:%d\n", TELEMETRY_PORT);
}

void telemetryUpdate() {
  if (!_ready) return;
  _ws.loop();
}

void telemetrySend(const TelemetryData& d) {
  if (!_ready || _clients == 0) return;

  char json[512];
  const char* modeStr;
  switch (d.envelope_mode) {
    case MODE_NORMAL:   modeStr = "NORMAL";   break;
    case MODE_DEGRADED: modeStr = "DEGRADED"; break;
    case MODE_DIRECT:   modeStr = "DIRECT";   break;
    default:            modeStr = "DISARM";   break;
  }

  auto healthStr = [](uint8_t h) -> const char* {
    switch (h) {
      case 0: return "OK";
      case 1: return "WARN";
      default: return "FAIL";
    }
  };

  snprintf(json, sizeof(json),
    "{"
    "\"heading\":%.1f,"
    "\"pitch\":%.1f,"
    "\"roll\":%.1f,"
    "\"heading_error\":%.2f,"
    "\"lift_throttle\":%.3f,"
    "\"thrust_throttle\":%.3f,"
    "\"yaw_rate\":%.2f,"
    "\"altitude\":%.2f,"
    "\"vspeed\":%.3f,"
    "\"ias_kt\":%.1f,"
    "\"envelope_mode\":\"%s\","
    "\"nodes\":["
    "{\"id\":1,\"health\":\"%s\",\"heading\":%.1f},"
    "{\"id\":2,\"health\":\"%s\",\"heading\":%.1f},"
    "{\"id\":3,\"health\":\"%s\",\"heading\":%.1f}"
    "]}",
    d.heading, d.pitch, d.roll,
    d.heading_error, d.lift_throttle, d.thrust_throttle, d.yaw_rate,
    d.altitude, d.vspeed, d.ias_kt,
    modeStr,
    healthStr(d.node_health[1]), d.node_heading[1],
    healthStr(d.node_health[2]), d.node_heading[2],
    healthStr(d.node_health[3]), d.node_heading[3]
  );

  _ws.broadcastTXT(json);
}

uint8_t telemetryGetClientCount() { return _clients; }
bool    telemetryIsReady()        { return _ready;   }
