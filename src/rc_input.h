#pragma once
// ===========================================================
// rc_input.h — CRSF RC input (ExpressLRS / RadioMaster Pocket)
// Single wire, 420000 baud, 16 channels, 11-bit values (172–1811)
// Frame: [addr 0xC8][len][type 0x16][22-byte payload][crc8 poly 0xD5]
// Call rcUpdate() every loop iteration
// ===========================================================
#include <Arduino.h>

// RCMode — driven by RC_MODE_CHANNEL (CH6 by default)
enum RCMode : uint8_t {
  RC_MANUAL     = 0,
  RC_ASSISTED   = 1,
  RC_AUTONOMOUS = 2
};

// Call once in setup()
void rcInit();

// Call every loop iteration — parses any new CRSF bytes
void rcUpdate();

// Raw µs value for channel ch (1–16), 1000–2000
uint16_t rcGetPWM(uint8_t ch);

// Normalized value:
//   CH1 (aileron/rudder): -1.0 ... 0.0 ... +1.0
//   All other channels:    0.0 ............. 1.0
float rcGetNormalized(uint8_t ch);

// Mode switch state from RC_MODE_CHANNEL
RCMode rcGetMode();

// True if a valid CRSF packet was received within RC_TIMEOUT_MS
bool rcIsActive();

// Diagnostic counters — useful for debugging signal issues
void rcGetStats(uint32_t& rxBytes, uint32_t& goodPkts, uint32_t& badPkts);
