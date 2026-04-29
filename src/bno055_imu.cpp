// ===========================================================
// bno055_imu.cpp — BNO055 IMU wrapper
// Uses Adafruit_BNO055 library
// I2C on SDA=IO32, SCL=IO33 (LilyGo T-CAN485)
// ===========================================================
#include "bno055_imu.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

static Adafruit_BNO055 bno(55, BNO055_I2C_ADDR);

static bool     _ready        = false;
static float    _heading      = 0.0f;
static float    _pitch        = 0.0f;
static float    _roll         = 0.0f;
static float    _yaw_rate     = 0.0f;
static float    _accel_x      = 0.0f;
static float    _prev_heading = 0.0f;
static uint32_t _lastSample   = 0;

bool imuInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!bno.begin()) {
    Serial.println("[IMU] ERROR: BNO055 not found! Check I2C wiring.");
    return false;
  }

  // external crystal for better accuracy
  bno.setExtCrystalUse(true);

  _ready = true;
  Serial.println("[IMU] BNO055 initialized OK.");
  return true;
}

void imuUpdate() {
  if (!_ready) return;

  uint32_t now = millis();
  if (now - _lastSample < BNO055_SAMPLERATE) return;
  float dt = (now - _lastSample) / 1000.0f;
  _lastSample = now;

  // read Euler angles
  // Adafruit convention: orientation.x = heading (yaw), y = roll, z = pitch
  sensors_event_t event;
  bno.getEvent(&event);

  float newHeading = event.orientation.x;
  _pitch = event.orientation.z;
  _roll  = event.orientation.y;

  // yaw rate in deg/s — handle 0/360 wrap
  float delta = newHeading - _prev_heading;
  if (delta >  180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;
  _yaw_rate     = delta / dt;
  _prev_heading = newHeading;
  _heading      = newHeading;

  // linear acceleration X
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  _accel_x = linAccel.x();
}

float   imuGetHeading()     { return _heading;  }
float   imuGetPitch()       { return _pitch;    }
float   imuGetRoll()        { return _roll;     }
float   imuGetYawRate()     { return _yaw_rate; }
float   imuGetAccelX()      { return _accel_x;  }
bool    imuIsReady()        { return _ready;    }

uint8_t imuGetCalibration() {
  if (!_ready) return 0;
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  return min(min(sys, gyro), min(accel, mag));
}
