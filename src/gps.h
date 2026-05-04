#pragma once
// ===========================================================
// gps.h — GNSS MAX-M10S NMEA parser (GNSS MAX 3 Click)
// UART: Serial2, GPIO35 RX, 9600 baud
// Parses GGA (position, altitude, fix) and RMC (speed, track)
// ===========================================================
#include <Arduino.h>

// Call once in setup()
void gpsInit();

// Call every loop iteration — reads available Serial2 bytes
void gpsUpdate();

bool  gpsHasFix();            // true once valid GGA fix received
float gpsGetLat();            // degrees, + = N
float gpsGetLon();            // degrees, + = E
float gpsGetAltMSL();         // metres above mean sea level
float gpsGetSpeedKt();        // knots
float gpsGetTrack();          // degrees true (0–360)
uint8_t gpsGetSats();         // satellites used
uint32_t gpsGetAge();         // ms since last valid fix
