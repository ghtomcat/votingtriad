#pragma once
// ===========================================================
// telemetry.h — WiFi WebSocket telemetry (active master only)
// WebSocket server on port 8080, JSON every 100ms
// Runs as own AP or connects to existing network
// ===========================================================
#include <Arduino.h>
#include "voting.h"
#include "can_bus.h"

struct TelemetryData {
  float heading;
  float pitch;
  float roll;
  float heading_error;
  float lift_throttle;
  float thrust_throttle;
  float yaw_rate;
  float altitude;    // meters AGL
  float vspeed;      // m/s vertical speed
  float ias_kt;      // indicated airspeed, knots (MPXV7002DP)
  EnvelopeMode envelope_mode;
  float   node_heading[4];  // index 1-3
  uint8_t node_health[4];   // index 1-3
};

void    telemetryInit();
void    telemetryUpdate();
void    telemetrySend(const TelemetryData& data);
uint8_t telemetryGetClientCount();
bool    telemetryIsReady();
