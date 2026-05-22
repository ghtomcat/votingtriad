#pragma once
// ===========================================================
// ina226.h — INA226 power monitor (M5Stack INA226 unit)
// I2C address 0x41, shared bus GPIO32/33
// Monitors avionics rail: voltage, current, power
// Shunt: 0.01Ω, range: 0–10A, 0–36V
// ===========================================================
#include <Arduino.h>

bool  inaInit();
void  inaUpdate();            // call every loop — rate-limited internally

float inaGetVoltage();        // V  — bus voltage
float inaGetCurrent();        // A  — current through shunt
float inaGetPower();          // W  — bus voltage × current
bool  inaIsReady();
