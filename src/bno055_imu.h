#pragma once
// ===========================================================
// bno055_imu.h — BNO055 IMU Wrapper
// DFRobot SEN0374, I2C Adresse 0x28
// Liefert Euler-Winkel: Heading, Pitch, Roll
// ===========================================================
#include <Arduino.h>

// Initialisiert I2C und BNO055
// Gibt true zurück wenn erfolgreich
bool imuInit();

// Liest neue Daten vom BNO055 (non-blocking, Intervall in config.h)
// Muss im Loop aufgerufen werden
void imuUpdate();

// Getter für Euler-Winkel
float imuGetHeading();  // 0–360 Grad (Magnetisches Nord)
float imuGetPitch();    // -180–180 Grad
float imuGetRoll();     // -180–180 Grad

// Liefert Yaw-Rate (Drehgeschwindigkeit um Z-Achse) in Grad/s
float imuGetYawRate();

// Liefert Beschleunigung in X-Richtung in m/s²
float imuGetAccelX();

// Kalibrierungsstatus (0 = unkalibriert, 3 = vollständig)
uint8_t imuGetCalibration();

// Gibt true zurück wenn BNO055 verfügbar und initialisiert
bool imuIsReady();
