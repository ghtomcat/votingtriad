// ===========================================================
// envelope.cpp — Envelope protection & PID heading hold
// NORMAL LAW:   full limits active, PID available
// DEGRADED:     limits reduced to 70%
// DIRECT LAW:   no limits, direct passthrough
// DISARM:       all outputs = 0, locked
// ===========================================================
#include "envelope.h"
#include "config.h"

static float    _pidIntegral  = 0.0f;
static float    _pidPrevError = 0.0f;
static uint32_t _lastPidTime  = 0;
static float    _targetHeading = 0.0f;  // frozen on assisted mode entry

void envelopeInit() {
  envelopeResetPID();
  _lastPidTime = millis();
  Serial.println("[ENV] Envelope protection initialized.");
}

void envelopeResetPID() {
  _pidIntegral  = 0.0f;
  _pidPrevError = 0.0f;
}

// heading error with 0/360 wrap
static float headingError(float target, float current) {
  float err = target - current;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// PID — returns rudder correction -1.0..+1.0
static float computePID(float error, float dt) {
  _pidIntegral += error * dt;
  // anti-windup: clamp integral
  _pidIntegral = constrain(_pidIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  float derivative = (dt > 0.0f) ? ((error - _pidPrevError) / dt) : 0.0f;
  _pidPrevError = error;

  float output = PID_KP * error + PID_KI * _pidIntegral + PID_KD * derivative;
  return constrain(output / 90.0f, -1.0f, 1.0f);  // ±90° → ±1.0 normalized
}

ControlOutput envelopeApply(
  float rawRudder,
  float rawThrust,
  float rawLift,
  EnvelopeMode mode,
  float yawRate,
  float targetHeading,
  float currentHeading,
  uint8_t rcMode)
{
  ControlOutput out = {0.0f, 0.0f, 0.0f, false};

  // DISARM: all outputs locked
  if (mode == MODE_DISARM) {
    Serial.println("[ENV] DISARM — all outputs locked!");
    return out;
  }

  out.armed = true;

  // --- DIRECT LAW: no limits, 1:1 passthrough ---
  if (mode == MODE_DIRECT) {
    out.rudder = constrain(rawRudder, -1.0f, 1.0f);
    out.thrust = constrain(rawThrust,  0.0f, 1.0f);
    out.lift   = constrain(rawLift,    0.0f, 1.0f);
    return out;
  }

  // --- NORMAL / DEGRADED: limits active ---
  float liftLimit    = MAX_LIFT_THROTTLE;
  float yawRateLimit = MAX_YAW_RATE;
  float thrustLimit  = 1.0f;

  if (mode == MODE_DEGRADED) {
    // degraded: reduce all limits to 70%
    liftLimit    *= 0.70f;
    thrustLimit  *= 0.70f;
    yawRateLimit *= 0.70f;
  }

  // --- yaw rate limiter ---
  // if yaw rate too high: suppress rudder input in that direction
  float rudderOut = constrain(rawRudder, -1.0f, 1.0f);
  if (yawRate >  yawRateLimit && rudderOut > 0.0f) rudderOut = 0.0f;
  if (yawRate < -yawRateLimit && rudderOut < 0.0f) rudderOut = 0.0f;

  // --- assisted mode: PID heading hold ---
  if (rcMode == 1) {
    uint32_t now = millis();
    float dt = (now - _lastPidTime) / 1000.0f;
    _lastPidTime = now;

    // stick input beyond deadband → pilot is steering, update target heading
    if (fabsf(rawRudder) * (RC_PWM_MAX - RC_PWM_MIN) / 2.0f > RC_DEADBAND) {
      _targetHeading = currentHeading;  // new target = current heading
      envelopeResetPID();
      rudderOut = rawRudder;
    } else {
      // no stick input → PID holds target heading
      float err = headingError(targetHeading, currentHeading);
      rudderOut = computePID(err, dt);
    }
  }

  // --- thrust & lift with envelope limits ---
  float thrustOut = constrain(rawThrust, 0.0f, thrustLimit);
  float liftOut   = constrain(rawLift,   0.0f, liftLimit);

  out.rudder = rudderOut;
  out.thrust = thrustOut;
  out.lift   = liftOut;
  return out;
}
