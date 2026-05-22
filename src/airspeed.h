#pragma once
// ===========================================================
// airspeed.h — MPXV7002DP differential pressure → IAS
// Analog sensor on GPIO35 (ADC1_CH7, input-only)
// Powered from 3.3 V; zero-calibrated at boot.
//
// Transfer function (Freescale MPXV7002D):
//   Vout = Vs × (0.2 × P_kPa + 0.5)   (P in kPa, ±2 kPa range)
//   → IAS = sqrt(2 × P_Pa / ρ)         (m/s, sea-level density)
//
// Pitot wiring: P1 = pitot (total pressure), P2 = static port.
// ===========================================================
#include <Arduino.h>

bool     airspeedInit();      // ADC setup + zero calibration at startup
void     airspeedUpdate();    // call every loop — rate-limited internally

float    airspeedGetPa();     // differential pressure, Pa  (≥ 0)
float    airspeedGetIAS_ms(); // indicated airspeed, m/s
float    airspeedGetIAS_kt(); // indicated airspeed, knots
bool     airspeedIsReady();
bool     airspeedIsStall();   // true when IAS < AIRSPEED_STALL_KT
