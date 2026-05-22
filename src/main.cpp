// ===========================================================
// main.cpp — Hovercraft Voting Triad Firmware v1.0
// Airbus-style envelope protection via 3-node CAN voting
//
// Node ID is set via build flag: -D NODE_ID=1/2/3
//
// BOOT SEQUENCE:
//  1. WS2812 WHITE     — initializing
//  2. BNO055 init      — wait for calibration
//  3. CAN bus init
//  4. Wait for all 3 nodes (max 5s)
//  5. GREEN            — system ready
//
// LOOP (non-blocking, millis()-based):
//  - IMU update        (every 20ms)
//  - CAN+RS485 send    (every 20ms / 60ms slot)
//  - CAN+RS485 receive (every loop iteration)
//  - Voting update     (every loop iteration)
//  - LED update        (every 100ms)
//  - [Master only] RC read, envelope, servo, log, telemetry
// ===========================================================

#include <Arduino.h>
#include "config.h"
#include "comms.h"
#include "bno055_imu.h"
#include "voting.h"
#include "envelope.h"
#include "rc_input.h"
#include "servo_output.h"
#include "sd_logger.h"
#include "telemetry.h"
#include "led_status.h"
#include "bmp390.h"
#include "gps.h"
#include "ina226.h"
#include "airspeed.h"
#ifdef HIL_MODE
#include "hil.h"
#endif

// ===========================================================
// TIMING (all non-blocking via millis())
// ===========================================================
static uint32_t _lastCommsSend  = 0;
static uint32_t _lastLedUpdate  = 0;
static uint32_t _lastLogUpdate  = 0;
static uint32_t _lastTelemetry  = 0;
static uint32_t _lastLoopTime   = 0;
static uint32_t _lastImuPrint   = 0;

// true once at least one other node has been heard
static bool _canConnected = false;

// target heading for heading hold — frozen on assisted mode entry
static float  _targetHeading = 0.0f;
static RCMode _lastRcMode    = RC_MANUAL;

// ===========================================================
// Heading error with 0/360 wrap
// ===========================================================
static float headingError(float target, float current) {
  float err = target - current;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ===========================================================
// SETUP
// ===========================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.printf("\n=== Hovercraft Voting Triad — Node %d ===\n", NODE_ID);
  Serial.printf("=== Build: %s %s ===\n", __DATE__, __TIME__);

  // 1. LED: WHITE — initializing
  ledInit();

  // 2. BNO055
  if (!imuInit()) {
    Serial.println("[BOOT] ERROR: IMU not available. Continuing without IMU...");
    // graceful degradation — no hard crash
  } else {
    Serial.print("[BOOT] Waiting for BNO055 calibration...");
    uint32_t bnoStart = millis();
    while (imuGetCalibration() < 1 && millis() - bnoStart < BOOT_BNO_TIMEOUT_MS) {
      imuUpdate();
      Serial.print(".");
      delay(100);
    }
    Serial.printf(" Calibration: %d/3\n", imuGetCalibration());
  }

  // 3. BMP390 barometer
  if (!bmpInit()) {
    Serial.println("[BOOT] WARNING: BMP390 not available. Altitude data disabled.");
  }

  // 3b. GPS
  gpsInit();

  // 3c. INA226 power monitor
  inaInit();

  // 3d. MPXV7002DP airspeed (sensor must be unpressurised during boot)
  airspeedInit();

  // 4. CAN + RS485 (comms manages both channels — skipped in solo)
#ifndef SOLO_NODE
  commsInit();
#endif

  // 4. Voting
  votingInit(NODE_ID);

  // 5. Envelope protection
  envelopeInit();

  // 6. Master-specific init
  // (prepared on all nodes — any node can be promoted to master)
  rcInit();
  servoInit();
  servoDisarm();

  if (NODE_ID == 1 || IS_MASTER) {
    sdInit();
    telemetryInit();
  }

  // 7. Wait for all 3 nodes (5s timeout)
#ifdef SOLO_NODE
  Serial.println("[BOOT] SOLO_NODE: Skipping node wait — single node test.");
  uint8_t nodeCount = 1;
  _canConnected = true;  // no other nodes expected — show mode colour, not search blink
#else
  Serial.print("[BOOT] Waiting for all 3 nodes...");
  uint32_t bootStart = millis();
  uint8_t seenNodes  = (1 << NODE_ID);  // mark own node as seen

  while (millis() - bootStart < BOOT_WAIT_ALL_NODES_MS) {
    commsReceive();
    for (uint8_t id = 1; id <= 3; id++) {
      VotePacket pkt;
      if (commsGetNodePacket(id, pkt)) seenNodes |= (1 << id);
    }
    if ((seenNodes & 0b1110) == 0b1110) break;  // bits 1,2,3 set = all present
    delay(50);
  }

  uint8_t nodeCount = __builtin_popcount(seenNodes >> 1);
  Serial.printf("\n[BOOT] %d/3 nodes found (bitmask: 0b%d%d%d)\n",
                nodeCount,
                (seenNodes >> 3) & 1,
                (seenNodes >> 2) & 1,
                (seenNodes >> 1) & 1);

  _canConnected = (nodeCount >= 2);
#endif
  _targetHeading = imuGetHeading();

  Serial.printf("[BOOT] Node %d ready. Master: %s\n",
                NODE_ID, votingIsMaster() ? "YES" : "NO");

#ifdef HIL_MODE
  Serial.println("[HIL] HIL_MODE active — connecting to OpenSim hub...");
  hilInit();
#endif

  Serial.println("[BOOT] System started.\n");
}

// ===========================================================
// MAIN LOOP
// ===========================================================
void loop() {
  uint32_t now = millis();

  // rate-limit main loop (~200Hz)
  if (now - _lastLoopTime < MAIN_LOOP_INTERVAL_MS) return;
  _lastLoopTime = now;

  // -----------------------------------------------------------
  // 1. IMU + barometer update (internally rate-limited to 20ms)
  // -----------------------------------------------------------
#ifdef HIL_MODE
  hilUpdate();
#else
  imuUpdate();
  bmpUpdate();
#endif
  gpsUpdate();
  inaUpdate();
  airspeedUpdate();

  // -----------------------------------------------------------
  // 2. Receive CAN + RS485 (non-blocking, both channels)
  // -----------------------------------------------------------
#ifndef SOLO_NODE
  commsReceive();

  // check connectivity: at least one other node active on any channel?
  for (uint8_t id = 1; id <= 3; id++) {
    if (id == NODE_ID) continue;
    if (commsGetLastRxAge(id) < NODE_FAIL_TIMEOUT_MS) {
      _canConnected = true;
      break;
    }
  }
#endif
  rcUpdate();

  // -----------------------------------------------------------
  // 3. Build and send VotePacket (every 20ms)
  //    CAN: immediate — RS485: own time-division slot
  // -----------------------------------------------------------
  if (now - _lastCommsSend >= CAN_TX_INTERVAL_MS) {
    _lastCommsSend = now;

#ifdef HIL_MODE
    float hdg = hilHasState() ? hilGetHeading() : 0.0f;
#else
    float hdg = imuIsReady() ? imuGetHeading() : 0.0f;
#endif
    float tgtHdg = _targetHeading;

    VotePacket myPkt;
    myPkt.node_id       = NODE_ID;
#ifdef HIL_MODE
    myPkt.heading       = hilGetHeading();
    myPkt.pitch         = hilGetPitch();
    myPkt.roll          = hilGetRoll();
    myPkt.heading_error = headingError(tgtHdg, hdg);
    myPkt.yaw_rate      = 0.0f;
    myPkt.accel_x       = 0.0f;
    myPkt.health        = hilHasState() ? HEALTH_OK : HEALTH_WARN;
    myPkt.altitude      = hilGetAltFt() * 0.3048f;  // ft → m
    myPkt.vspeed        = hilGetVsFpm() * 0.00508f;  // fpm → m/s
#else
    myPkt.heading       = hdg;
    myPkt.pitch         = imuGetPitch();
    myPkt.roll          = imuGetRoll();
    myPkt.heading_error = headingError(tgtHdg, hdg);
    myPkt.yaw_rate      = imuGetYawRate();
    myPkt.accel_x       = imuGetAccelX();
    myPkt.health        = !imuIsReady()           ? HEALTH_WARN :
                          imuGetCalibration() < 2  ? HEALTH_WARN :
                                                     HEALTH_OK;
    myPkt.altitude      = bmpIsReady() ? bmpGetAltitude() : 0.0f;
    myPkt.vspeed        = bmpIsReady() ? bmpGetVSpeed()   : 0.0f;
#endif
    myPkt.timestamp     = now;

#ifndef SOLO_NODE
    commsSend(myPkt);
#endif

    // -----------------------------------------------------------
    // 4. Voting update (after own packet is sent)
    // -----------------------------------------------------------
    votingUpdate(myPkt);
  }

  // -----------------------------------------------------------
  // 5. Master functions (RC, servo, envelope)
  // -----------------------------------------------------------
  if (votingIsMaster()) {
    EnvelopeMode mode = votingGetMode();
    RCMode rcMode     = rcGetMode();

    // freeze target heading on assisted mode entry
    if (rcMode == RC_ASSISTED && _lastRcMode != RC_ASSISTED) {
      _targetHeading = imuGetHeading();
      envelopeResetPID();
      Serial.printf("[ASSIST] Heading Hold active: target = %.1f°\n", _targetHeading);
    }
    _lastRcMode = rcMode;

    float currentHdg = imuGetHeading();

    if (!rcIsActive()) {
      servoDisarm();
    } else {
#if VEHICLE_TYPE == VEHICLE_AIRPLANE
      float ail = (float)((int)rcGetPWM(RC_CH_AILERON)  - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
      float ele = (float)((int)rcGetPWM(RC_CH_ELEVATOR) - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
      float rud = (float)((int)rcGetPWM(RC_CH_RUDDER)   - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
      float thr = (float)((int)rcGetPWM(RC_CH_THROTTLE) - RC_PWM_MIN) / (float)(RC_PWM_MAX - RC_PWM_MIN);
      ail = constrain(ail, -1.0f, 1.0f);
      ele = constrain(ele, -1.0f, 1.0f);
      thr = constrain(thr,  0.0f, 1.0f);
#ifdef HIL_MODE
      // In HIL mode: send controls to OpenSim instead of PCA9685
      float rollT  = ail * HIL_MAX_BANK_DEG;
      float pitchT = ele * HIL_MAX_PITCH_DEG;
      float spdT   = thr * HIL_MAX_SPD_KT;
      hilSendControls(rollT, pitchT, spdT);
#else
      servoSetAileron(ail);
      servoSetElevator(ele);
      servoSetRudder(constrain(rud, -1.0f, 1.0f));
      servoSetThrottle(thr);
#endif
#else
      float rawRudder = rcGetNormalized(RC_CH_RUDDER);
      float rawThrust = rcGetNormalized(RC_CH_THRUST);
      float rawLift   = rcGetNormalized(RC_CH_LIFT);

      ControlOutput out = envelopeApply(
        rawRudder, rawThrust, rawLift,
        mode,
        imuGetYawRate(),
        _targetHeading,
        currentHdg,
        (uint8_t)rcMode
      );

      if (out.armed) {
        servoSetRudder(out.rudder);
        servoSetThrust(out.thrust);
        servoSetLift(out.lift);
      } else {
        servoDisarm();
      }
#endif
    }

    // -----------------------------------------------------------
    // 6. SD logging (every 100ms)
    // -----------------------------------------------------------
    if (now - _lastLogUpdate >= SD_LOG_INTERVAL_MS) {
      _lastLogUpdate = now;

      NodeStatus ns1 = votingGetNodeStatus(1);
      NodeStatus ns2 = votingGetNodeStatus(2);
      NodeStatus ns3 = votingGetNodeStatus(3);

      LogData logEntry;
      logEntry.timestamp       = now;
      logEntry.heading_1       = ns1.heading;
      logEntry.heading_2       = ns2.heading;
      logEntry.heading_3       = ns3.heading;
      logEntry.voted_heading   = votingGetConsensusHeading();
      logEntry.heading_error   = headingError(_targetHeading, imuGetHeading());
      logEntry.lift_throttle   = rcGetNormalized(3);
      logEntry.thrust_throttle = rcGetNormalized(2);
      logEntry.yaw_rate        = imuGetYawRate();
      logEntry.altitude        = bmpGetAltitude();
      logEntry.vspeed          = bmpGetVSpeed();
      logEntry.ias_kt          = airspeedGetIAS_kt();
      logEntry.envelope_mode   = (uint8_t)votingGetMode();
      logEntry.node1_health    = (uint8_t)ns1.health;
      logEntry.node2_health    = (uint8_t)ns2.health;
      logEntry.node3_health    = (uint8_t)ns3.health;

      sdLog(logEntry);
    }

    // -----------------------------------------------------------
    // 7. Telemetry (every 100ms)
    // -----------------------------------------------------------
    if (now - _lastTelemetry >= TELEMETRY_INTERVAL_MS) {
      _lastTelemetry = now;

      TelemetryData tel;
      tel.heading         = imuGetHeading();
      tel.pitch           = imuGetPitch();
      tel.roll            = imuGetRoll();
      tel.heading_error   = headingError(_targetHeading, imuGetHeading());
      tel.lift_throttle   = rcGetNormalized(3);
      tel.thrust_throttle = rcGetNormalized(2);
      tel.yaw_rate        = imuGetYawRate();
      tel.altitude        = bmpGetAltitude();
      tel.vspeed          = bmpGetVSpeed();
      tel.ias_kt          = airspeedGetIAS_kt();
      tel.envelope_mode   = votingGetMode();

      for (uint8_t id = 1; id <= 3; id++) {
        NodeStatus ns = votingGetNodeStatus(id);
        tel.node_heading[id] = ns.heading;
        tel.node_health[id]  = (uint8_t)ns.health;
      }

      telemetrySend(tel);
      telemetryUpdate();
    }
  } else {
    // non-master: servo stays safe; telemetry not needed
    servoDisarm();
  }

#ifdef SOLO_NODE
  if (now - _lastImuPrint >= 1000) {
    _lastImuPrint = now;
    uint32_t rxBytes, goodPkts, badPkts;
    rcGetStats(rxBytes, goodPkts, badPkts);
#ifdef HIL_MODE
    float ail = (float)((int)rcGetPWM(RC_CH_AILERON)  - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
    float ele = (float)((int)rcGetPWM(RC_CH_ELEVATOR) - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
    float thr = (float)((int)rcGetPWM(RC_CH_THROTTLE) - RC_PWM_MIN) / (float)(RC_PWM_MAX - RC_PWM_MIN);
    Serial.printf("[HIL] RC:%s  AIL:%+.2f  ELE:%+.2f  THR:%.2f  good:%lu  bad:%lu  bytes:%lu  ws:%s\n",
                  rcIsActive() ? "OK" : "FAIL",
                  ail, ele, thr, goodPkts, badPkts, rxBytes,
                  hilIsConnected() ? "OK" : "FAIL");
#else
    const char* modeName[] = {"MANUAL", "ASSISTED", "AUTO"};
    Serial.printf("[RC]  Active: %-3s  CH1:%4d  CH2:%4d  CH3:%4d  CH4:%4d  Mode: %s  good:%lu\n",
                  rcIsActive() ? "YES" : "NO",
                  rcGetPWM(1), rcGetPWM(2), rcGetPWM(3), rcGetPWM(4),
                  modeName[rcGetMode()], goodPkts);
    Serial.printf("[GPS] Fix:%-3s  Sats:%2d  Lat:%.6f  Lon:%.6f  Alt:%.1fm  Spd:%.1fkt  Trk:%.1f°\n",
                  gpsHasFix() ? "YES" : "NO",
                  gpsGetSats(), gpsGetLat(), gpsGetLon(),
                  gpsGetAltMSL(), gpsGetSpeedKt(), gpsGetTrack());
    Serial.printf("[PWR] %.2fV  %.3fA  %.2fW\n",
                  inaGetVoltage(), inaGetCurrent(), inaGetPower());
    Serial.printf("[IAS] %.1f kt  %.1f Pa%s\n",
                  airspeedGetIAS_kt(), airspeedGetPa(),
                  airspeedIsStall() ? "  *** STALL ***" : "");
#endif
  }
#endif

  // -----------------------------------------------------------
  // 8. LED update (every 100ms)
  // -----------------------------------------------------------
  if (now - _lastLedUpdate >= 100) {
    _lastLedUpdate = now;
    RCMode rcMode = votingIsMaster() ? rcGetMode() : RC_MANUAL;
    ledUpdate(votingGetMode(), rcMode, _canConnected);
  }
}
