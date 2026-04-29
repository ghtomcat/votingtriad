#pragma once
// ===========================================================
// can_bus.h — CAN bus communication via ESP32 TWAI driver
// VotePacket split into 3 CAN frames of 8 bytes each:
//   Frame A (0x100+id): Heading, Pitch, Roll, Health, NodeID
//   Frame B (0x200+id): Heading error, Yaw rate, Accel X, Timestamp
//   Frame C (0x300+id): Altitude, VSpeed (reserved[4])
// ===========================================================
#include <Arduino.h>
#include <stdint.h>

struct VotePacket {
  uint8_t  node_id;
  float    heading;        // degrees 0–360
  float    pitch;          // degrees
  float    roll;           // degrees
  float    heading_error;  // deviation from target heading
  float    yaw_rate;       // degrees/second
  float    accel_x;        // m/s²
  float    altitude;       // metres AGL
  float    vspeed;         // m/s vertical speed
  uint8_t  health;         // 0=OK 1=WARN 2=FAIL
  uint32_t timestamp;      // millis()
};

enum NodeHealth : uint8_t {
  HEALTH_OK   = 0,
  HEALTH_WARN = 1,
  HEALTH_FAIL = 2
};

void     canInit();
void     canSendPacket(const VotePacket& pkt);
void     canReceive();
bool     canGetNodePacket(uint8_t node_id, VotePacket& out);
uint32_t canGetLastRxAge(uint8_t node_id);
