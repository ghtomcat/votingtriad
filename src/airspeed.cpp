// ===========================================================
// airspeed.cpp — MPXV7002DP differential pressure sensor
//
// ADC1_CH7 / GPIO35 — input-only pin on LilyGo T-CAN485.
// Attenuation: ADC_11db (0–3.9 V range, 12-bit).
//
// Boot: averages AIRSPEED_ZERO_SAMPLES readings → zero offset.
//       Compensates for manufacturing offset, supply variance,
//       and temperature.
//
// Each update: 8-sample median → pressure → IAS.
//   8 reads × ~100 µs ≈ 800 µs per update (negligible at 20 ms rate).
//
// Pressure formula (from zero-corrected raw counts):
//   V_delta = (raw − zero) × ADC_VMAX_V / 4095
//   P_kPa   = V_delta / (Vs × 0.2)
//   P_Pa    = P_kPa × 1000
//   IAS_ms  = sqrt(2 × P_Pa / ρ)
//
// Where:
//   ADC_VMAX_V = AIRSPEED_ADC_VMAX_MV / 1000  (3900 mV for DB_11)
//   Vs         = AIRSPEED_SENSOR_VCC           (3.3 V default)
// ===========================================================
#include "airspeed.h"
#include "config.h"
#include <Arduino.h>

static const float ADC_VMAX_V  = AIRSPEED_ADC_VMAX_MV / 1000.0f;
static const float ADC_COUNTS  = 4095.0f;

// Precomputed Pa-per-raw-count scale factor (avoids float div in update)
// P_Pa = (raw − zero) × PA_PER_COUNT
static const float PA_PER_COUNT =
    (ADC_VMAX_V * 1000.0f) / (ADC_COUNTS * AIRSPEED_SENSOR_VCC * 0.2f);

static bool     _ready      = false;
static int32_t  _zeroRaw    = 2048;
static float    _pressure   = 0.0f;   // Pa
static float    _ias_ms     = 0.0f;   // m/s
static uint32_t _lastSample = 0;

// ---------------------------------------------------------------------------
// 8-sample median — sort in place, return average of two middle values
// Eliminates spike noise from ADC at the cost of ~800 µs per call.
static int32_t readMedian8() {
  int16_t s[8];
  for (uint8_t i = 0; i < 8; i++) {
    s[i] = (int16_t)analogRead(PIN_AIRSPEED_ADC);
    delayMicroseconds(150);
  }
  // insertion sort (fast for 8 elements)
  for (uint8_t i = 1; i < 8; i++) {
    int16_t key = s[i];
    int8_t  j   = (int8_t)i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  return ((int32_t)s[3] + (int32_t)s[4]) / 2;  // midpoint average
}

// ---------------------------------------------------------------------------
bool airspeedInit() {
  analogSetPinAttenuation(PIN_AIRSPEED_ADC, ADC_11db);

  // discard the first few reads — ADC needs a moment to settle
  for (uint8_t i = 0; i < 8; i++) {
    analogRead(PIN_AIRSPEED_ADC);
    delay(2);
  }

  // zero calibration: average AIRSPEED_ZERO_SAMPLES reads
  int64_t sum = 0;
  for (uint8_t i = 0; i < AIRSPEED_ZERO_SAMPLES; i++) {
    sum += analogRead(PIN_AIRSPEED_ADC);
    delay(2);
  }
  _zeroRaw = (int32_t)(sum / AIRSPEED_ZERO_SAMPLES);

  _ready = true;
  Serial.printf("[AIRSPEED] MPXV7002DP on GPIO%d  zero=%ld  scale=%.4f Pa/count\n",
                PIN_AIRSPEED_ADC, (long)_zeroRaw, PA_PER_COUNT);
  return true;
}

// ---------------------------------------------------------------------------
void airspeedUpdate() {
  if (!_ready) return;
  uint32_t now = millis();
  if (now - _lastSample < AIRSPEED_SAMPLERATE_MS) return;
  _lastSample = now;

  int32_t raw   = readMedian8();
  int32_t delta = raw - _zeroRaw;
  if (delta < 0) delta = 0;           // negative → treat as zero airspeed

  float p_pa = (float)delta * PA_PER_COUNT;
  if (p_pa < AIRSPEED_NOISE_FLOOR_PA) p_pa = 0.0f;

  _pressure = p_pa;
  _ias_ms   = (p_pa > 0.0f) ? sqrtf(2.0f * p_pa / RHO_SEA_LEVEL_KGM3) : 0.0f;
}

// ---------------------------------------------------------------------------
float airspeedGetPa()     { return _pressure; }
float airspeedGetIAS_ms() { return _ias_ms; }
float airspeedGetIAS_kt() { return _ias_ms * 1.94384f; }
bool  airspeedIsReady()   { return _ready; }
bool  airspeedIsStall()   { return _ready && (airspeedGetIAS_kt() < AIRSPEED_STALL_KT); }
