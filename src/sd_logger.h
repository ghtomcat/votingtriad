#pragma once
// ===========================================================
// sd_logger.h — SD card logging (active master only)
// CSV format, SPI on LilyGo T-CAN485 pins
// File: /hoverlog.csv, flushed every 5s
// ===========================================================
#include <Arduino.h>

struct LogData {
  uint32_t timestamp;
  float heading_1, heading_2, heading_3;
  float voted_heading;
  float heading_error;
  float lift_throttle;
  float thrust_throttle;
  float yaw_rate;
  float altitude;   // meters AGL
  float vspeed;     // m/s vertical speed
  float ias_kt;     // indicated airspeed, knots (MPXV7002DP)
  uint8_t envelope_mode;
  uint8_t node1_health, node2_health, node3_health;
};

bool sdInit();
void sdLog(const LogData& data);
bool sdIsReady();
