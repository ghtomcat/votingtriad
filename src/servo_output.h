#pragma once
// ===========================================================
// servo_output.h — Servo/ESC output via PCA9685 (I2C, 50Hz)
// PCA9685 shares I2C bus with BNO055 (SDA=32, SCL=33)
// Channel mapping is set by VEHICLE_TYPE in config.h
// ===========================================================
#include <Arduino.h>
#include "config.h"

void servoInit();
void servoDisarm();

#if VEHICLE_TYPE == VEHICLE_AIRPLANE
  void servoSetAileron(float normalized);   // -1.0 left,  +1.0 right
  void servoSetElevator(float normalized);  // -1.0 down,  +1.0 up
  void servoSetRudder(float normalized);    // -1.0 left,  +1.0 right
  void servoSetThrottle(float normalized);  //  0.0 off,   +1.0 full
#else
  void servoSetRudder(float normalized);    // -1.0 left,  +1.0 right
  void servoSetThrust(float normalized);    //  0.0 off,   +1.0 full
  void servoSetLift(float normalized);      //  0.0 off,   +1.0 full (capped by MAX_LIFT_THROTTLE)
#endif
