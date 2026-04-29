#pragma once
// ===========================================================
// bmp390.h — BMP390 barometer wrapper
// I2C shared with BNO055 (SDA=32, SCL=33), address 0x76
// Provides altitude (AGL), vertical speed, pressure, temperature
// ===========================================================
#include <Arduino.h>

// Call once in setup() — after imuInit() so Wire is already started
bool bmpInit();

// Call every loop iteration — internally rate-limited to BMP390_SAMPLERATE
void bmpUpdate();

float bmpGetAltitude();     // meters above boot point (AGL)
float bmpGetVSpeed();       // m/s vertical speed (+ = climbing)
float bmpGetPressure();     // hPa
float bmpGetTemperature();  // °C
bool  bmpIsReady();
