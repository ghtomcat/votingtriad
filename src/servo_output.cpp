// ===========================================================
// servo_output.cpp — PCA9685 16-channel PWM servo/ESC output
// I2C shared with BNO055 (SDA=32, SCL=33), address 0x40
// 50Hz, 12-bit resolution (4096 steps over 20ms period)
//   1000µs → 205   (min / off)
//   1500µs → 307   (center / neutral)
//   2000µs → 410   (max / full throttle)
// ===========================================================
#include "servo_output.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

static Adafruit_PWMServoDriver _pca(PCA9685_I2C_ADDR);
static bool _pcaReady = false;

static uint16_t usToDuty(uint16_t us) {
  us = constrain(us, (uint16_t)SERVO_MIN_US, (uint16_t)SERVO_MAX_US);
  return (uint16_t)((float)us / 20000.0f * 4096.0f);
}

static void setPWMBidirectional(uint8_t ch, float norm) {
  if (!_pcaReady) return;
  float us = SERVO_MID_US + norm * (SERVO_MAX_US - SERVO_MID_US);
  _pca.setPWM(ch, 0, usToDuty((uint16_t)constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US)));
}

static void setPWMUnidirectional(uint8_t ch, float norm) {
  if (!_pcaReady) return;
  float us = SERVO_MIN_US + norm * (SERVO_MAX_US - SERVO_MIN_US);
  _pca.setPWM(ch, 0, usToDuty((uint16_t)constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US)));
}

void servoInit() {
  Wire.end();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // probe I2C address before init — begin() does not verify presence
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[SERVO] WARNING: PCA9685 not found — servo output disabled.");
    _pcaReady = false;
    return;
  }

  _pca.begin();
  _pca.setPWMFreq(SERVO_FREQ_HZ);
  _pcaReady = true;
  servoDisarm();
#if VEHICLE_TYPE == VEHICLE_AIRPLANE
  Serial.printf("[SERVO] PCA9685 (0x%02X) AIRPLANE: AIL=CH%d ELE=CH%d RUD=CH%d THR=CH%d\n",
                PCA9685_I2C_ADDR,
                PCA9685_CH_AILERON, PCA9685_CH_ELEVATOR,
                PCA9685_CH_RUDDER,  PCA9685_CH_THROTTLE);
#else
  Serial.printf("[SERVO] PCA9685 (0x%02X) HOVERCRAFT: RUD=CH%d THR=CH%d LIFT=CH%d\n",
                PCA9685_I2C_ADDR,
                PCA9685_CH_RUDDER, PCA9685_CH_THRUST, PCA9685_CH_LIFT);
#endif
}

void servoDisarm() {
  if (!_pcaReady) return;
#if VEHICLE_TYPE == VEHICLE_AIRPLANE
  _pca.setPWM(PCA9685_CH_AILERON,  0, usToDuty(SERVO_MID_US));
  _pca.setPWM(PCA9685_CH_ELEVATOR, 0, usToDuty(SERVO_MID_US));
  _pca.setPWM(PCA9685_CH_RUDDER,   0, usToDuty(SERVO_MID_US));
  _pca.setPWM(PCA9685_CH_THROTTLE, 0, usToDuty(SERVO_MIN_US));
#else
  _pca.setPWM(PCA9685_CH_RUDDER, 0, usToDuty(SERVO_MID_US));
  _pca.setPWM(PCA9685_CH_THRUST, 0, usToDuty(SERVO_MIN_US));
  _pca.setPWM(PCA9685_CH_LIFT,   0, usToDuty(SERVO_MIN_US));
#endif
}

#if VEHICLE_TYPE == VEHICLE_AIRPLANE

void servoSetAileron(float normalized)  { setPWMBidirectional(PCA9685_CH_AILERON,  normalized); }
void servoSetElevator(float normalized) { setPWMBidirectional(PCA9685_CH_ELEVATOR, normalized); }
void servoSetRudder(float normalized)   { setPWMBidirectional(PCA9685_CH_RUDDER,   normalized); }
void servoSetThrottle(float normalized) { setPWMUnidirectional(PCA9685_CH_THROTTLE, normalized); }

#else

void servoSetRudder(float normalized)  { setPWMBidirectional(PCA9685_CH_RUDDER,  normalized); }
void servoSetThrust(float normalized)  { setPWMUnidirectional(PCA9685_CH_THRUST,  normalized); }
void servoSetLift(float normalized)    { setPWMUnidirectional(PCA9685_CH_LIFT,    normalized); }

#endif
