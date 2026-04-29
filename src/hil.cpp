// ===========================================================
// hil.cpp — Hardware-In-the-Loop via OpenSim WebSocket hub
// ===========================================================
#include "hil.h"
#include "config.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

static WebSocketsClient _ws;
static bool _connected  = false;
static bool _hasState   = false;

static float _hdg   = 0.0f;
static float _pitch = 0.0f;
static float _roll  = 0.0f;
static float _alt   = 0.0f;
static float _spd   = 0.0f;
static float _vs    = 0.0f;

// ---------------------------------------------------------------------------
// Minimal JSON field extractor — finds "key":value in a flat patch object.
// Only handles numeric values. Returns true if key found.
static bool _jsonFloat(const char* json, const char* key, float& out) {
  // search for "key":
  char search[32];
  snprintf(search, sizeof(search), "\"%s\":", key);
  const char* p = strstr(json, search);
  if (!p) return false;
  p += strlen(search);
  while (*p == ' ') p++;
  out = atof(p);
  return true;
}

static bool _jsonHasKey(const char* json, const char* key) {
  char search[32];
  snprintf(search, sizeof(search), "\"%s\":", key);
  return strstr(json, search) != nullptr;
}

// ---------------------------------------------------------------------------
static void _onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      _connected = true;
      Serial.printf("[HIL] Connected to hub ws://%s:%d\n", HIL_HUB_HOST, HIL_HUB_PORT);
      {
        char join[128];
        snprintf(join, sizeof(join),
                 "{\"type\":\"JOIN\",\"room\":\"%s\",\"role\":\"HIL\"}", HIL_ROOM);
        _ws.sendTXT(join);
        Serial.printf("[HIL] Joined room '%s'\n", HIL_ROOM);
      }
      break;

    case WStype_DISCONNECTED:
      _connected = false;
      _hasState  = false;
      Serial.println("[HIL] Disconnected from hub");
      break;

    case WStype_TEXT: {
      const char* json = (const char*)payload;
      // Only handle STATE_PATCH messages
      if (!strstr(json, "\"STATE_PATCH\"")) return;
      // Find patch object
      const char* patch = strstr(json, "\"patch\":");
      if (!patch) return;

      float v;
      if (_jsonFloat(patch, "hdg",   v)) _hdg   = v;
      if (_jsonFloat(patch, "pitch", v)) _pitch = v;
      if (_jsonFloat(patch, "roll",  v)) _roll  = v;
      if (_jsonFloat(patch, "alt",   v)) _alt   = v;
      if (_jsonFloat(patch, "spd",   v)) _spd   = v;
      if (_jsonFloat(patch, "vs",    v)) _vs    = v;

      _hasState = true;
      break;
    }

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
void hilInit() {
  Serial.printf("[HIL] Connecting to WiFi '%s'...", HIL_WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(HIL_WIFI_SSID, HIL_WIFI_PASS);

  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[HIL] WiFi failed — HIL disabled");
    return;
  }
  Serial.printf("\n[HIL] WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());

  _ws.begin(HIL_HUB_HOST, HIL_HUB_PORT, "/");
  _ws.onEvent(_onWsEvent);
  _ws.setReconnectInterval(3000);
}

void hilUpdate() {
  _ws.loop();
}

bool hilIsConnected() { return _connected; }
bool hilHasState()    { return _hasState;  }

float hilGetHeading() { return _hdg;   }
float hilGetPitch()   { return _pitch; }
float hilGetRoll()    { return _roll;  }
float hilGetAltFt()   { return _alt;   }
float hilGetSpdKt()   { return _spd;   }
float hilGetVsFpm()   { return _vs;    }

void hilSendControls(float rollT, float pitchT, float spdT) {
  if (!_connected) return;
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"STATE_PATCH\",\"room\":\"%s\","
           "\"patch\":{\"rollT\":%.1f,\"pitchT\":%.1f,\"spdT\":%.0f,\"ap\":false}}",
           HIL_ROOM, rollT, pitchT, spdT);
  _ws.sendTXT(buf);
}
