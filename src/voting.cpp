// ===========================================================
// voting.cpp — 2-of-3 voting logic
// Analyses packets from all 3 nodes, determines consensus on
// heading and altitude, manages envelope mode and master promotion.
// ===========================================================
#include "voting.h"
#include "config.h"
#include "comms.h"

static uint8_t      _myId         = 1;
static EnvelopeMode _mode         = MODE_DISARM;
static EnvelopeMode _prevMode     = MODE_DISARM;
static float        _consensusHdg = 0.0f;
static float        _consensusAlt = 0.0f;
static uint8_t      _healthyCount = 0;
static bool         _isMaster     = false;

static NodeStatus _nodes[4] = {};

static float headingDelta(float a, float b) {
  float d = a - b;
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return fabsf(d);
}

void votingInit(uint8_t myNodeId) {
  _myId     = myNodeId;
  _isMaster = (myNodeId == 1);
  for (int i = 1; i <= 3; i++) {
    _nodes[i].active       = false;
    _nodes[i].health       = HEALTH_FAIL;
    _nodes[i].heading      = 0.0f;
    _nodes[i].altitude     = 0.0f;
    _nodes[i].lastRxAge_ms = 0xFFFFFFFF;
  }
  Serial.printf("[VOTE] Init: Node %d — starting role: %s\n",
                _myId, _isMaster ? "MASTER" : "VOTER");
}

void votingUpdate(const VotePacket& myPacket) {
  // --- 1. Store own data ---
  _nodes[_myId].active       = true;
  _nodes[_myId].heading      = myPacket.heading;
  _nodes[_myId].altitude     = myPacket.altitude;
  _nodes[_myId].health       = (NodeHealth)myPacket.health;
  _nodes[_myId].lastRxAge_ms = 0;

  // --- 2. Read received packets from other nodes ---
  for (uint8_t id = 1; id <= 3; id++) {
    if (id == _myId) continue;

    uint32_t age = commsGetLastRxAge(id);
    _nodes[id].lastRxAge_ms = age;

    VotePacket pkt;
    if (commsGetNodePacket(id, pkt)) {
      _nodes[id].heading  = pkt.heading;
      _nodes[id].altitude = pkt.altitude;

      if (age < NODE_WARN_TIMEOUT_MS) {
        _nodes[id].health = HEALTH_OK;
        _nodes[id].active = true;
      } else if (age < NODE_FAIL_TIMEOUT_MS) {
        _nodes[id].health = HEALTH_WARN;
        _nodes[id].active = true;
      } else {
        _nodes[id].health = HEALTH_FAIL;
        _nodes[id].active = false;
      }
    } else {
      _nodes[id].health = HEALTH_FAIL;
      _nodes[id].active = false;
    }
  }

  // --- 3. Count healthy nodes ---
  _healthyCount = 0;
  for (uint8_t id = 1; id <= 3; id++) {
    if (_nodes[id].health != HEALTH_FAIL) _healthyCount++;
  }

  // --- 4. Heading consensus ---
  float   hdg[4];
  bool    hdgOk[4]   = {};
  float   sumHdg     = 0.0f;
  uint8_t sumHdgCnt  = 0;

  for (uint8_t id = 1; id <= 3; id++) hdg[id] = _nodes[id].heading;

  for (uint8_t a = 1; a <= 3; a++) {
    if (_nodes[a].health == HEALTH_FAIL) continue;
    for (uint8_t b = 1; b <= 3; b++) {
      if (a == b || _nodes[b].health == HEALTH_FAIL) continue;
      if (headingDelta(hdg[a], hdg[b]) < HEADING_TOLERANCE) {
        hdgOk[a] = true;
        break;
      }
    }
    if (hdgOk[a]) { sumHdg += hdg[a]; sumHdgCnt++; }
  }

  uint8_t agreeCount = sumHdgCnt;
  if (agreeCount == 0 && _healthyCount > 0) agreeCount = 1;

  // Circular mean to handle the 0°/360° wrap correctly
  if (sumHdgCnt > 0) {
    float sinSum = 0.0f, cosSum = 0.0f;
    for (uint8_t id = 1; id <= 3; id++) {
      if (!hdgOk[id]) continue;
      sinSum += sinf(hdg[id] * DEG_TO_RAD);
      cosSum += cosf(hdg[id] * DEG_TO_RAD);
    }
    _consensusHdg = atan2f(sinSum, cosSum) * RAD_TO_DEG;
    if (_consensusHdg < 0.0f) _consensusHdg += 360.0f;
  } else {
    _consensusHdg = myPacket.heading;
  }

  // --- 5. Altitude consensus ---
  float   alt[4];
  float   sumAlt    = 0.0f;
  uint8_t sumAltCnt = 0;

  for (uint8_t id = 1; id <= 3; id++) alt[id] = _nodes[id].altitude;

  for (uint8_t a = 1; a <= 3; a++) {
    if (_nodes[a].health == HEALTH_FAIL) continue;
    bool agrees = false;
    for (uint8_t b = 1; b <= 3; b++) {
      if (a == b || _nodes[b].health == HEALTH_FAIL) continue;
      if (fabsf(alt[a] - alt[b]) < ALTITUDE_TOLERANCE) {
        agrees = true;
        break;
      }
    }
    if (agrees) { sumAlt += alt[a]; sumAltCnt++; }
  }

  _consensusAlt = (sumAltCnt > 0) ? sumAlt / sumAltCnt : myPacket.altitude;

  // --- 6. Envelope mode ---
  EnvelopeMode newMode;
  if      (agreeCount >= 3) newMode = MODE_NORMAL;
  else if (agreeCount >= 2) newMode = MODE_DEGRADED;
  else if (agreeCount >= 1) newMode = MODE_DIRECT;
  else                      newMode = MODE_DISARM;

  if (newMode != _prevMode) {
    Serial.printf("[VOTE] MODE: %s → %s (agreed: %d/3, healthy: %d/3)\n",
                  votingModeName(_prevMode), votingModeName(newMode),
                  agreeCount, _healthyCount);
    _prevMode = newMode;
  }
  _mode = newMode;

  // --- 7. Master promotion — lowest-ID healthy node ---
  uint8_t masterCandidate = 0;
  for (uint8_t id = 1; id <= 3; id++) {
    if (_nodes[id].health != HEALTH_FAIL) { masterCandidate = id; break; }
  }

  bool wasMaster = _isMaster;
  _isMaster = (masterCandidate == _myId);

  if (_isMaster != wasMaster) {
    if (_isMaster)
      Serial.printf("[VOTE] NODE %d → MASTER PROMOTION!\n", _myId);
    else
      Serial.printf("[VOTE] NODE %d → VOTER (master: node %d)\n", _myId, masterCandidate);
  }
}

EnvelopeMode votingGetMode()              { return _mode;          }
float        votingGetConsensusHeading()  { return _consensusHdg;  }
float        votingGetConsensusAltitude() { return _consensusAlt;  }
uint8_t      votingGetHealthyCount()      { return _healthyCount;  }
bool         votingIsMaster()             { return _isMaster;      }

NodeStatus votingGetNodeStatus(uint8_t node_id) {
  if (node_id < 1 || node_id > 3) return {};
  return _nodes[node_id];
}

const char* votingModeName(EnvelopeMode mode) {
  switch (mode) {
    case MODE_NORMAL:   return "NORMAL LAW";
    case MODE_DEGRADED: return "DEGRADED";
    case MODE_DIRECT:   return "DIRECT LAW";
    case MODE_DISARM:   return "DISARM";
    default:            return "UNKNOWN";
  }
}
