// ===========================================================
// ina226.cpp — INA226 power monitor, direct I2C (no library)
// Registers: 0x00 config, 0x01 shunt V, 0x02 bus V,
//            0x03 power, 0x04 current, 0x05 calibration
// ===========================================================
#include "ina226.h"
#include "config.h"
#include <Wire.h>

#define INA226_ADDR       0x41
#define INA226_SAMPLERATE 100   // ms between reads

// INA226 registers
#define REG_CONFIG   0x00
#define REG_SHUNT_V  0x01
#define REG_BUS_V    0x02
#define REG_POWER    0x03
#define REG_CURRENT  0x04
#define REG_CAL      0x05

// Calibration for 0.01Ω shunt, 1mA current LSB
// Cal = 0.00512 / (current_LSB * R_shunt) = 0.00512 / (0.001 * 0.01) = 512
#define INA226_CAL        512
#define INA226_CURRENT_LSB 0.001f   // A per LSB
#define INA226_POWER_LSB  (25.0f * INA226_CURRENT_LSB)  // W per LSB
#define INA226_BUS_LSB    0.00125f  // V per LSB

static bool     _ready      = false;
static float    _voltage    = 0.0f;
static float    _current    = 0.0f;
static float    _power      = 0.0f;
static uint32_t _lastSample = 0;

static bool writeReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, int16_t& out) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2) != 2) return false;
  out = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool inaInit() {
  // probe
  Wire.beginTransmission(INA226_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.printf("[INA226] Not found at 0x%02X\n", INA226_ADDR);
    return false;
  }

  // config: avg=16, bus 1.1ms, shunt 1.1ms, continuous shunt+bus
  // BADC=0b0101 (1.1ms), SADC=0b0101, MODE=0b111, AVG=0b001 (4 samples)
  // 0x4527 = avg=4, 1.1ms conv, continuous
  if (!writeReg(REG_CONFIG, 0x4527)) return false;
  if (!writeReg(REG_CAL, INA226_CAL)) return false;

  _ready = true;
  Serial.printf("[INA226] Init OK at 0x%02X — 0.01Ω shunt, 1mA LSB\n", INA226_ADDR);
  return true;
}

void inaUpdate() {
  if (!_ready) return;
  uint32_t now = millis();
  if (now - _lastSample < INA226_SAMPLERATE) return;
  _lastSample = now;

  int16_t raw;

  if (readReg(REG_BUS_V, raw))
    _voltage = (float)raw * INA226_BUS_LSB;

  if (readReg(REG_CURRENT, raw))
    _current = (float)raw * INA226_CURRENT_LSB;

  if (readReg(REG_POWER, raw))
    _power = (float)(uint16_t)raw * INA226_POWER_LSB;
}

float inaGetVoltage() { return _voltage; }
float inaGetCurrent() { return _current; }
float inaGetPower()   { return _power;   }
bool  inaIsReady()    { return _ready;   }
