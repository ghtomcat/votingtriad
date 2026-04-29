// ===========================================================
// ground_test.cpp — VotingTriad Ground Verification Suite
// Build: pio run -e node1_test -t upload
//
// Phases:
//   1. Hardware self-test             (automated)
//   2. Voting algorithm unit test     (automated)
//   3. RC input / CRSF                (semi-automated)
//   4. Control surface verification   (interactive)
//   5. CAN bus & multi-node voting    (interactive, all 3 nodes)
//   6. Pre-flight sign-off checklist  (interactive)
// ===========================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "comms.h"
#include "bno055_imu.h"
#include "bmp390.h"
#include "voting.h"
#include "envelope.h"
#include "rc_input.h"
#include "servo_output.h"
#include "sd_logger.h"
#include "led_status.h"

// ===========================================================
// Result tracking
// ===========================================================
static uint16_t _pass, _fail, _warn;

static void resetCounters() { _pass = _fail = _warn = 0; }

static void chk(const char* name, bool ok, const char* detail = nullptr) {
  if (ok) _pass++; else _fail++;
  Serial.printf("  [%s] %s", ok ? "PASS" : "FAIL", name);
  if (detail) Serial.printf(" (%s)", detail);
  Serial.println();
}

static void wrn(const char* name, const char* detail = nullptr) {
  _warn++;
  Serial.printf("  [WARN] %s", name);
  if (detail) Serial.printf(" (%s)", detail);
  Serial.println();
}

static void phaseSummary(const char* name) {
  Serial.println();
  Serial.printf("  %s: %d passed  %d failed  %d warnings\n", name, _pass, _fail, _warn);
  Serial.println(_fail == 0 ? "  Result: PHASE OK" : "  Result: PHASE FAILED — resolve before flight");
}

// ===========================================================
// UI helpers
// ===========================================================
static void sep(char c = '-') {
  for (int i = 0; i < 60; i++) Serial.print(c);
  Serial.println();
}

static void waitEnter() {
  Serial.print("  [Press ENTER] ");
  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();
  Serial.println();
}

static bool yesNo(const char* prompt) {
  Serial.printf("  %s [y/n]: ", prompt);
  while (true) {
    while (!Serial.available()) delay(50);
    char c = (char)Serial.read();
    if (c == 'y' || c == 'Y') { Serial.println("y"); return true;  }
    if (c == 'n' || c == 'N') { Serial.println("n"); return false; }
  }
}

// ===========================================================
// Phase 1: Hardware Self-Test
// ===========================================================
static void phase1_hardware() {
  sep('=');
  Serial.println("PHASE 1: HARDWARE SELF-TEST");
  sep();
  resetCounters();

  // --- I2C bus scan ---
  Serial.println("  I2C bus scan:");
  bool foundBno = false, foundPca = false, foundBmp = false;
  bool foundIna = false, foundGps = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    const char* lbl = "unknown";
    if (addr == 0x28) { lbl = "BNO055";  foundBno = true; }
    if (addr == 0x40) { lbl = "PCA9685"; foundPca = true; }
    if (addr == 0x41) { lbl = "INA226";  foundIna = true; }
    if (addr == 0x42) { lbl = "GPS";     foundGps = true; }
    if (addr == 0x76) { lbl = "BMP390";  foundBmp = true; }
    Serial.printf("    0x%02X  [%s]\n", addr, lbl);
  }

  chk("BNO055  @ 0x28", foundBno);
  chk("PCA9685 @ 0x40", foundPca);
  chk("BMP390  @ 0x76", foundBmp);
  if (!foundIna) wrn("INA226 @ 0x41 not found — power monitor not installed yet");
  if (!foundGps) wrn("GPS @ 0x42 not found — GPS not installed yet");

  // --- IMU ---
  Serial.println();
  bool imuOk = imuInit();
  chk("BNO055 init", imuOk);

  if (imuOk) {
    Serial.print("  Calibrating BNO055 (up to 10s, rotate board)...");
    uint32_t t = millis();
    while (imuGetCalibration() < 2 && millis() - t < 10000) {
      imuUpdate(); delay(100); Serial.print(".");
    }
    Serial.println();
    uint8_t cal = imuGetCalibration();
    char buf[16]; snprintf(buf, sizeof(buf), "cal=%d/3", cal);
    chk("IMU calibration >= 2/3", cal >= 2, buf);

    if (imuIsReady()) {
      imuUpdate();
      float h = imuGetHeading(), p = imuGetPitch(), r = imuGetRoll();
      char detail[48]; snprintf(detail, sizeof(detail), "hdg=%.1f pitch=%.1f roll=%.1f", h, p, r);
      chk("IMU values plausible", h >= 0.0f && h < 360.0f && fabsf(p) < 90.0f, detail);
    }
  }

  // --- BMP390 ---
  Serial.println();
  bool bmpOk = bmpInit();
  chk("BMP390 init", bmpOk);

  if (bmpOk) {
    Serial.print("  Sampling altitude (20 × 60ms)...");
    float rd[20];
    for (int i = 0; i < 20; i++) { bmpUpdate(); rd[i] = bmpGetAltitude(); delay(60); Serial.print("."); }
    Serial.println();

    float mean = 0;
    for (int i = 0; i < 20; i++) mean += rd[i];
    mean /= 20.0f;

    float sq = 0;
    for (int i = 0; i < 20; i++) { float d = rd[i] - mean; sq += d * d; }
    float sd = sqrtf(sq / 20.0f);

    char buf[32]; snprintf(buf, sizeof(buf), "sigma=%.2fm", sd);
    chk("Altitude noise sigma < 0.5m at rest", sd < 0.5f, buf);

    float vs = bmpGetVSpeed();
    char vb[32]; snprintf(vb, sizeof(vb), "%.2f m/s", vs);
    chk("VSpeed near zero at rest (|v| < 0.3 m/s)", fabsf(vs) < 0.3f, vb);

    char pb[40]; snprintf(pb, sizeof(pb), "%.1f hPa  %.1f°C", bmpGetPressure(), bmpGetTemperature());
    Serial.printf("  Info: pressure=%s\n", pb);
  }

  // --- SD card ---
  Serial.println();
  chk("SD card init", sdInit());

  phaseSummary("Phase 1");
}

// ===========================================================
// Phase 2: Voting Algorithm Unit Test
// ===========================================================
// Local re-implementation of voting.cpp so we can feed known
// inputs and verify outputs without needing live CAN traffic.
// Any divergence between this and voting.cpp is itself a bug.

static float simDelta(float a, float b) {
  float d = a - b;
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return fabsf(d);
}

struct SimNode { float hdg; float alt; bool fail; };

struct SimResult {
  uint8_t      agreeCount;
  uint8_t      healthyCount;
  float        consensusHdg;
  float        consensusAlt;
  EnvelopeMode mode;
};

static SimResult simVote(SimNode n[4]) {
  uint8_t healthyCnt = 0;
  for (int i = 1; i <= 3; i++) if (!n[i].fail) healthyCnt++;

  bool  hdgOk[4] = {};
  float sinSum = 0, cosSum = 0;
  uint8_t sumHdgCnt = 0;

  for (uint8_t a = 1; a <= 3; a++) {
    if (n[a].fail) continue;
    for (uint8_t b = 1; b <= 3; b++) {
      if (a == b || n[b].fail) continue;
      if (simDelta(n[a].hdg, n[b].hdg) < HEADING_TOLERANCE) { hdgOk[a] = true; break; }
    }
    if (hdgOk[a]) {
      sinSum += sinf(n[a].hdg * DEG_TO_RAD);
      cosSum += cosf(n[a].hdg * DEG_TO_RAD);
      sumHdgCnt++;
    }
  }

  float sumAlt = 0; uint8_t sumAltCnt = 0;
  for (uint8_t a = 1; a <= 3; a++) {
    if (n[a].fail) continue;
    bool agrees = false;
    for (uint8_t b = 1; b <= 3; b++) {
      if (a == b || n[b].fail) continue;
      if (fabsf(n[a].alt - n[b].alt) < ALTITUDE_TOLERANCE) { agrees = true; break; }
    }
    if (agrees) { sumAlt += n[a].alt; sumAltCnt++; }
  }

  uint8_t agreeCnt = sumHdgCnt;
  if (agreeCnt == 0 && healthyCnt > 0) agreeCnt = 1;

  // Circular mean for heading (same fix as voting.cpp)
  float consHdg = n[1].hdg;
  if (sumHdgCnt > 0) {
    consHdg = atan2f(sinSum, cosSum) * RAD_TO_DEG;
    if (consHdg < 0.0f) consHdg += 360.0f;
  }

  float consAlt = (sumAltCnt > 0) ? sumAlt / sumAltCnt : n[1].alt;

  EnvelopeMode mode = (agreeCnt >= 3) ? MODE_NORMAL  :
                      (agreeCnt >= 2) ? MODE_DEGRADED :
                      (agreeCnt >= 1) ? MODE_DIRECT   : MODE_DISARM;

  return { agreeCnt, healthyCnt, consHdg, consAlt, mode };
}

static void phase2_votingMath() {
  sep('=');
  Serial.println("PHASE 2: VOTING ALGORITHM UNIT TEST");
  sep();
  resetCounters();

  // --- headingDelta wrap-around ---
  chk("headingDelta(359,  1) = 2°",   fabsf(simDelta(359.0f, 1.0f)   - 2.0f)   < 0.01f);
  chk("headingDelta(  1,359) = 2°",   fabsf(simDelta(1.0f, 359.0f)   - 2.0f)   < 0.01f);
  chk("headingDelta(180,  0) = 180°", fabsf(simDelta(180.0f, 0.0f)   - 180.0f) < 0.01f);
  chk("headingDelta( 90, 90) = 0°",   fabsf(simDelta(90.0f, 90.0f))             < 0.01f);

  // --- 3/3 agree → NORMAL LAW ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.5f,100.2f,false}, {91.0f,100.1f,false}};
    SimResult r = simVote(n);
    char b[48]; snprintf(b, sizeof(b), "mode=%s agree=%d hdg=%.1f", votingModeName(r.mode), r.agreeCount, r.consensusHdg);
    chk("3/3 agree → NORMAL LAW",       r.mode == MODE_NORMAL && r.agreeCount == 3, b);
    chk("Consensus heading near 90.5°", fabsf(r.consensusHdg - 90.5f) < 0.2f);
  }

  // --- 2/3 agree, 1 outlier → DEGRADED ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.0f,100.0f,false}, {190.0f,100.0f,false}};
    SimResult r = simVote(n);
    char b[48]; snprintf(b, sizeof(b), "mode=%s agree=%d hdg=%.1f", votingModeName(r.mode), r.agreeCount, r.consensusHdg);
    chk("2/3 agree, 1 outlier → DEGRADED",    r.mode == MODE_DEGRADED && r.agreeCount == 2, b);
    chk("Outlier excluded from consensus hdg", fabsf(r.consensusHdg - 90.0f) < 0.1f);
  }

  // --- 2 healthy agree, 1 FAIL node → DEGRADED ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.0f,100.0f,false}, {90.0f,100.0f,true}};
    SimResult r = simVote(n);
    char b[32]; snprintf(b, sizeof(b), "mode=%s healthy=%d", votingModeName(r.mode), r.healthyCount);
    chk("2 healthy agree, 1 FAIL → DEGRADED", r.mode == MODE_DEGRADED, b);
  }

  // --- 1 healthy, 2 FAIL → DIRECT LAW ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.0f,100.0f,true}, {90.0f,100.0f,true}};
    SimResult r = simVote(n);
    char b[32]; snprintf(b, sizeof(b), "mode=%s healthy=%d", votingModeName(r.mode), r.healthyCount);
    chk("1 healthy, 2 FAIL → DIRECT LAW", r.mode == MODE_DIRECT, b);
  }

  // --- All FAIL → DISARM ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,true}, {90.0f,100.0f,true}, {90.0f,100.0f,true}};
    SimResult r = simVote(n);
    char b[32]; snprintf(b, sizeof(b), "mode=%s agree=%d", votingModeName(r.mode), r.agreeCount);
    chk("All FAIL → DISARM", r.mode == MODE_DISARM && r.agreeCount == 0, b);
  }

  // --- Heading 0/360 wrap-around: circular mean, not naive average ---
  {
    SimNode n[4] = {{}, {359.0f,0.0f,false}, {1.0f,0.0f,false}, {0.5f,0.0f,false}};
    SimResult r = simVote(n);
    // Correct answer ~0.17°; naive average would give ~120°
    bool nearZero = r.consensusHdg < 5.0f || r.consensusHdg > 355.0f;
    char b[40]; snprintf(b, sizeof(b), "consensus=%.2f° (expect ~0°)", r.consensusHdg);
    chk("Wrap-around consensus: 359/1/0.5° → ~0° (circular mean)", r.mode == MODE_NORMAL && nearZero, b);
  }

  // --- Altitude outlier excluded ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.0f,100.5f,false}, {90.0f,115.0f,false}};
    SimResult r = simVote(n);
    char b[40]; snprintf(b, sizeof(b), "alt=%.2fm (expect 100.25)", r.consensusAlt);
    chk("Altitude outlier excluded (115m vs 100/100.5m)", fabsf(r.consensusAlt - 100.25f) < 0.1f, b);
  }

  // --- Altitude: 2 FAIL, 1 healthy → no consensus, fallback to own ---
  {
    SimNode n[4] = {{}, {90.0f,100.0f,false}, {90.0f,200.0f,true}, {90.0f,300.0f,true}};
    SimResult r = simVote(n);
    chk("Altitude fallback to own when 2 FAIL", fabsf(r.consensusAlt - 100.0f) < 0.1f);
  }

  phaseSummary("Phase 2");
}

// ===========================================================
// Phase 3: RC Input (CRSF)
// ===========================================================
static void phase3_rc() {
  sep('=');
  Serial.println("PHASE 3: RC INPUT (CRSF)");
  sep();
  resetCounters();

  rcInit();

  Serial.println("  Power on transmitter. Waiting for CRSF link (15s)...");
  uint32_t t = millis();
  while (!rcIsActive() && millis() - t < 15000) {
    rcUpdate(); delay(50);
    if ((millis() - t) % 2000 < 50) Serial.print(".");
  }
  Serial.println();
  chk("CRSF link established", rcIsActive());

  if (!rcIsActive()) {
    Serial.println("  No CRSF link — skipping remaining RC tests.");
    phaseSummary("Phase 3");
    return;
  }

  // Drain 500ms of fresh packets
  uint32_t end = millis() + 500;
  while (millis() < end) { rcUpdate(); delay(5); }

  // All channels in range
  bool allOk = true;
  for (uint8_t ch = 1; ch <= 16; ch++) {
    uint16_t v = rcGetPWM(ch);
    if (v < 900 || v > 2100) { allOk = false; Serial.printf("    CH%d out of range: %dµs\n", ch, v); }
  }
  chk("All 16 channels in 900–2100µs", allOk);

  // Sticks at center
  Serial.println("  Center all sticks, then press ENTER.");
  waitEnter();
  end = millis() + 300; while (millis() < end) { rcUpdate(); delay(5); }

  for (uint8_t ch = 1; ch <= 4; ch++) {
    uint16_t v = rcGetPWM(ch);
    char b[24]; snprintf(b, sizeof(b), "CH%d=%dµs", ch, v);
    chk("Stick channel at center ±100µs", abs((int)v - 1500) < 100, b);
  }

  // Mode switch
  Serial.println("  Mode switch → pos 1 (low), then ENTER.");
  waitEnter();
  end = millis() + 200; while (millis() < end) { rcUpdate(); delay(5); }
  chk("Mode switch pos.1 → MANUAL", rcGetMode() == RC_MANUAL);

  Serial.println("  Mode switch → pos 2 (mid), then ENTER.");
  waitEnter();
  end = millis() + 200; while (millis() < end) { rcUpdate(); delay(5); }
  chk("Mode switch pos.2 → ASSISTED", rcGetMode() == RC_ASSISTED);

  Serial.println("  Mode switch → pos 3 (high), then ENTER.");
  waitEnter();
  end = millis() + 200; while (millis() < end) { rcUpdate(); delay(5); }
  chk("Mode switch pos.3 → AUTONOMOUS", rcGetMode() == RC_AUTONOMOUS);

  // Failsafe
  Serial.println("  Turn off transmitter, then press ENTER.");
  waitEnter();
  delay(RC_TIMEOUT_MS + 200); rcUpdate();
  chk("RC failsafe triggers within timeout", !rcIsActive());

  Serial.println("  Turn transmitter back on, then press ENTER.");
  waitEnter();
  t = millis(); while (!rcIsActive() && millis() - t < 5000) { rcUpdate(); delay(50); }
  chk("RC link recovers after TX power-on", rcIsActive());

  phaseSummary("Phase 3");
}

// ===========================================================
// Phase 4: Control Surface / Servo Test
// ===========================================================
static void phase4_servo() {
  sep('=');
  Serial.println("PHASE 4: CONTROL SURFACE VERIFICATION");
  sep();
  Serial.println("  *** REMOVE ALL PROPELLERS / ROTORS BEFORE PROCEEDING ***");
  if (!yesNo("Props removed and area clear of obstructions?")) {
    Serial.println("  Servo test skipped.");
    return;
  }
  resetCounters();

  servoInit();
  servoDisarm();
  Serial.println("  All channels armed to neutral.");
  chk("Servos at neutral / ESCs at minimum",
      yesNo("All surfaces at neutral, ESCs silent (armed but not spinning)?"));

  // Rudder
  Serial.println();
  Serial.println("  Rudder sweep:");
  servoSetRudder(-1.0f); delay(800);
  chk("Rudder: full LEFT deflection",  yesNo("Rudder deflects fully LEFT?"));
  servoSetRudder( 0.0f); delay(300);
  servoSetRudder( 1.0f); delay(800);
  chk("Rudder: full RIGHT deflection", yesNo("Rudder deflects fully RIGHT?"));
  servoSetRudder( 0.0f);

  // Thrust ESC arm
  Serial.println();
  Serial.println("  Thrust ESC: hold at zero throttle for 3s (arming)...");
  servoSetThrust(0.0f); delay(3000);
  chk("Thrust ESC armed",
      yesNo("Thrust ESC produced arming beep sequence?"));

  // Lift ESC arm
  Serial.println();
  Serial.println("  Lift ESC: hold at zero throttle for 3s (arming)...");
  servoSetLift(0.0f); delay(3000);
  chk("Lift ESC armed",
      yesNo("Lift ESC produced arming beep sequence?"));

  servoDisarm();
  Serial.println("  All channels returned to disarmed state.");

  phaseSummary("Phase 4");
}

// ===========================================================
// Phase 5: CAN Bus & Multi-Node Voting
// ===========================================================
static void phase5_nodes() {
  sep('=');
  Serial.println("PHASE 5: CAN BUS & MULTI-NODE VOTING");
  sep();
  Serial.println("  All 3 nodes must be powered and running flight firmware.");
  if (!yesNo("All 3 nodes powered and CAN bus connected?")) {
    Serial.println("  Phase 5 skipped."); return;
  }
  resetCounters();

  if (!imuIsReady()) imuInit();
  if (!bmpIsReady()) bmpInit();
  commsInit();
  votingInit(NODE_ID);

  // Wait for all 3 nodes (10s)
  Serial.print("  Waiting for all 3 nodes...");
  uint8_t seen = (1 << NODE_ID);
  uint32_t t = millis();
  while (millis() - t < 10000) {
    commsReceive();
    for (uint8_t id = 1; id <= 3; id++) {
      VotePacket pkt;
      if (commsGetNodePacket(id, pkt)) seen |= (1 << id);
    }
    if ((seen & 0b1110) == 0b1110) break;
    delay(100); Serial.print(".");
  }
  Serial.println();

  uint8_t nodeCount = __builtin_popcount(seen >> 1);
  char nb[20]; snprintf(nb, sizeof(nb), "%d/3 nodes seen", nodeCount);
  chk("All 3 nodes visible on CAN bus", nodeCount == 3, nb);

  // Run voting for 1s
  auto runVoting = [](uint32_t ms) {
    uint32_t end = millis() + ms;
    while (millis() < end) {
      imuUpdate(); bmpUpdate(); commsReceive();
      VotePacket p = {};
      p.node_id  = NODE_ID;
      p.heading  = imuGetHeading();
      p.pitch    = imuGetPitch();
      p.roll     = imuGetRoll();
      p.health   = HEALTH_OK;
      p.altitude = bmpIsReady() ? bmpGetAltitude() : 0.0f;
      p.timestamp = millis();
      commsSend(p); votingUpdate(p); delay(20);
    }
  };

  runVoting(1500);
  EnvelopeMode mode = votingGetMode();
  char mb[40]; snprintf(mb, sizeof(mb), "mode=%s healthy=%d", votingModeName(mode), votingGetHealthyCount());
  chk("3-node consensus: NORMAL LAW", mode == MODE_NORMAL, mb);
  chk("All 3 nodes reported healthy",  votingGetHealthyCount() == 3);

  // --- Node dropout ---
  Serial.println();
  Serial.println("  Unplug ONE node (not this one), then press ENTER.");
  waitEnter();
  runVoting(2000);

  mode = votingGetMode();
  snprintf(mb, sizeof(mb), "mode=%s healthy=%d", votingModeName(mode), votingGetHealthyCount());
  chk("1 node lost → DEGRADED", mode == MODE_DEGRADED, mb);

  Serial.println("  Reconnect the node, then press ENTER.");
  waitEnter();
  runVoting(3000);
  mode = votingGetMode();
  snprintf(mb, sizeof(mb), "mode=%s", votingModeName(mode));
  chk("Node recovery → NORMAL LAW restored", mode == MODE_NORMAL, mb);

  // --- Master promotion ---
  Serial.println();
  Serial.printf("  This is Node %d. isMaster = %s\n", NODE_ID, votingIsMaster() ? "YES" : "NO");
  if (NODE_ID == 1) {
    Serial.println("  Unplug Node 1 is this node — cannot self-test promotion here.");
    Serial.println("  Run this test from Node 2 or 3 to verify master promotion.");
  } else {
    Serial.println("  Unplug Node 1 (the normal master), then press ENTER.");
    waitEnter();
    runVoting(2000);
    chk("Master promotion: this node becomes master", votingIsMaster());
    Serial.println("  Reconnect Node 1, then press ENTER.");
    waitEnter();
    runVoting(2000);
    chk("Master relinquished back to Node 1", !votingIsMaster());
  }

  phaseSummary("Phase 5");
}

// ===========================================================
// Phase 6: Pre-Flight Sign-Off Checklist
// ===========================================================
static void phase6_preflight() {
  sep('=');
  Serial.println("PHASE 6: PRE-FLIGHT SIGN-OFF CHECKLIST");
  sep();
  Serial.println("  All items must be Y. Sign-off is recorded in this log.");
  Serial.println();
  resetCounters();

  chk("Control surface directions correct (Phase 4 passed)",
      yesNo("All surfaces move in the correct direction?"));
  chk("RC link active and all channels respond",
      yesNo("TX/RX link active, all sticks respond correctly?"));
  chk("All 3 nodes in NORMAL LAW",
      yesNo("All nodes show NORMAL LAW on serial output?"));
  chk("IMU calibration >= 2/3 on all nodes",
      yesNo("IMU calibration >= 2/3 on all nodes?"));
  chk("Altitude reads near ground zero AGL",
      yesNo("All BMP390s read within +/-2m of zero AGL?"));
  chk("Battery voltage within limits",
      yesNo("Battery voltage checked with multimeter and within spec?"));
  chk("CG within flight envelope",
      yesNo("Centre of gravity within specified range?"));
  chk("Propellers / rotors secure",
      yesNo("All propellers/rotors secure, undamaged, balanced?"));
  chk("Range check completed at 30m",
      yesNo("Full control response verified at 30m with props off?"));
  chk("Flight area clear",
      yesNo("Flight area clear of people, animals, and obstacles?"));

  Serial.println();
  phaseSummary("Pre-Flight Checklist");

  sep('=');
  if (_fail == 0) {
    Serial.printf("  CLEARED FOR FLIGHT — Node %d — %s %s\n", NODE_ID, __DATE__, __TIME__);
  } else {
    Serial.printf("  NOT CLEARED — %d item(s) failed — DO NOT FLY\n", _fail);
  }
  sep('=');
}

// ===========================================================
// Menu
// ===========================================================
static void printMenu() {
  sep('=');
  Serial.printf("  VotingTriad Ground Verification Suite — Node %d\n", NODE_ID);
  Serial.printf("  Build: %s %s\n", __DATE__, __TIME__);
  sep();
  Serial.println("  [A] Auto-Test: phases 1 + 2 (no human input)");
  Serial.println("  [1] Phase 1: Hardware Self-Test");
  Serial.println("  [2] Phase 2: Voting Algorithm Unit Test");
  Serial.println("  [3] Phase 3: RC Input (CRSF)");
  Serial.println("  [4] Phase 4: Control Surface Verification");
  Serial.println("  [5] Phase 5: CAN Bus & Multi-Node Voting");
  Serial.println("  [6] Phase 6: Pre-Flight Sign-Off Checklist");
  Serial.println("  [?] Print this menu");
  sep('=');
  Serial.print("  > ");
}

// ===========================================================
// Arduino entry points
// ===========================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(600);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  ledInit();
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char cmd = (char)Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println();

  switch (cmd) {
    case 'A': case 'a': phase1_hardware(); phase2_votingMath(); break;
    case '1': phase1_hardware();  break;
    case '2': phase2_votingMath(); break;
    case '3': phase3_rc();        break;
    case '4': phase4_servo();     break;
    case '5': phase5_nodes();     break;
    case '6': phase6_preflight(); break;
    case '?': break;
    default:  Serial.println("  Unknown command."); break;
  }

  Serial.println();
  printMenu();
}
