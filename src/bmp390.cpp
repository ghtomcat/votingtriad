// ===========================================================
// bmp390.cpp — BMP390 barometer wrapper
// Shares I2C bus with BNO055 — Wire already started by imuInit()
// Altitude is relative to boot point (AGL)
// Vertical speed uses IIR low-pass filter to reduce pressure noise
// ===========================================================
#include "bmp390.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

static Adafruit_BMP3XX _bmp;
static bool     _ready       = false;
static float    _altitude    = 0.0f;
static float    _pressure    = 1013.25f;
static float    _temperature = 0.0f;
static float    _vspeed      = 0.0f;
static float    _refAltitude = 0.0f;
static float    _prevAlt     = 0.0f;
static uint32_t _lastSample  = 0;

bool bmpInit() {
  if (!_bmp.begin_I2C(BMP390_I2C_ADDR)) {
    Serial.println("[BMP] ERROR: BMP390 not found! Check I2C wiring.");
    return false;
  }

  _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  _bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // settle with several readings before locking reference altitude
  for (uint8_t i = 0; i < 10; i++) {
    _bmp.performReading();
    delay(20);
  }

  _refAltitude = _bmp.readAltitude(BMP390_SEA_LEVEL_HPA);
  _prevAlt     = 0.0f;
  _pressure    = _bmp.pressure / 100.0f;
  _temperature = _bmp.temperature;
  _ready       = true;

  Serial.printf("[BMP] BMP390 initialized. Ref alt: %.1fm  %.2f hPa  %.1f°C\n",
                _refAltitude, _pressure, _temperature);
  return true;
}

void bmpUpdate() {
  if (!_ready) return;

  uint32_t now = millis();
  if (now - _lastSample < BMP390_SAMPLERATE) return;
  float dt = (now - _lastSample) / 1000.0f;
  _lastSample = now;

  if (!_bmp.performReading()) return;

  _pressure    = _bmp.pressure / 100.0f;
  _temperature = _bmp.temperature;

  float seaAlt = _bmp.readAltitude(BMP390_SEA_LEVEL_HPA);
  float newAlt = seaAlt - _refAltitude;

  // IIR low-pass: raw vspeed is noisy, smooth heavily
  float rawVSpeed = (newAlt - _prevAlt) / dt;
  _vspeed  = _vspeed * 0.85f + rawVSpeed * 0.15f;
  _prevAlt = newAlt;
  _altitude = newAlt;
}

float bmpGetAltitude()    { return _altitude;    }
float bmpGetVSpeed()      { return _vspeed;      }
float bmpGetPressure()    { return _pressure;    }
float bmpGetTemperature() { return _temperature; }
bool  bmpIsReady()        { return _ready;       }
