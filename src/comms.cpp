// ===========================================================
// comms.cpp — Communication abstraction
// Manages CAN as primary channel and RS485 as fallback.
// Automatically selects the freshest packet per node.
// ===========================================================
#include "comms.h"
#include "config.h"
#include "can_bus.h"
#include "rs485.h"

// timestamp of the last CAN frame received from any node
static uint32_t _lastAnyCanRx = 0;
static bool     _canAlive      = false;
static bool     _rs485Active   = false;
static bool     _prevCanAlive  = true;  // for transition logging

// active transport per node [1-3]
static Transport _transport[4] = {TRANSPORT_NONE};

// log transport changes only on transition
static void checkTransportChange() {
  bool newCanAlive = (millis() - _lastAnyCanRx) < COMMS_CAN_DEAD_MS;

  if (newCanAlive != _prevCanAlive) {
    if (!newCanAlive) {
      Serial.println("[COMMS] *** CAN BUS DOWN — switching to RS485 fallback ***");
    } else {
      Serial.println("[COMMS] CAN bus recovered — RS485 fallback deactivated.");
    }
    _prevCanAlive = newCanAlive;
  }

  _canAlive    = newCanAlive;
  _rs485Active = !newCanAlive;
}

void commsInit() {
  canInit();
  rs485Init();
  _lastAnyCanRx = millis();  // assume CAN alive at startup
  _canAlive     = true;
  Serial.println("[COMMS] CAN (primary) + RS485 (fallback) initialized.");
}

void commsSend(const VotePacket& pkt) {
  canSendPacket(pkt);
  rs485Send(pkt);
}

void commsReceive() {
  canReceive();
  rs485Receive();

  // track CAN activity: if any node heard over CAN → CAN is alive
  for (uint8_t id = 1; id <= 3; id++) {
    if (id == NODE_ID) continue;
    if (canGetLastRxAge(id) < COMMS_CAN_DEAD_MS) {
      _lastAnyCanRx = millis();
      break;
    }
  }

  checkTransportChange();

  // update transport selection per node
  for (uint8_t id = 1; id <= 3; id++) {
    if (id == NODE_ID) continue;

    uint32_t canAge = canGetLastRxAge(id);
    uint32_t rs4Age = rs485GetLastRxAge(id);

    // prefer CAN if fresher than RS485 or CAN bus is alive
    if (canAge <= rs4Age && canAge != 0xFFFFFFFF) {
      _transport[id] = TRANSPORT_CAN;
    } else if (rs4Age != 0xFFFFFFFF) {
      _transport[id] = TRANSPORT_RS485;
    } else {
      _transport[id] = TRANSPORT_NONE;
    }
  }
}

bool commsGetNodePacket(uint8_t node_id, VotePacket& out) {
  if (node_id < 1 || node_id > 3) return false;

  uint32_t canAge = canGetLastRxAge(node_id);
  uint32_t rs4Age = rs485GetLastRxAge(node_id);

  // use CAN packet if fresher (or RS485 has none)
  if (canAge <= rs4Age) {
    return canGetNodePacket(node_id, out);
  } else {
    return rs485GetNodePacket(node_id, out);
  }
}

uint32_t commsGetLastRxAge(uint8_t node_id) {
  if (node_id < 1 || node_id > 3) return 0xFFFFFFFF;
  uint32_t canAge = canGetLastRxAge(node_id);
  uint32_t rs4Age = rs485GetLastRxAge(node_id);
  return min(canAge, rs4Age);
}

Transport commsGetTransport(uint8_t node_id) {
  if (node_id < 1 || node_id > 3) return TRANSPORT_NONE;
  return _transport[node_id];
}

bool commsIsCANAlive()    { return _canAlive;    }
bool commsIsRS485Active() { return _rs485Active; }

void commsPrintStatus() {
  const char* transportName[] = {"NONE", "CAN", "RS485"};
  Serial.printf("[COMMS] CAN: %s | RS485 fallback: %s\n",
                _canAlive ? "OK" : "FAIL",
                _rs485Active ? "ACTIVE" : "standby");
  for (uint8_t id = 1; id <= 3; id++) {
    if (id == NODE_ID) {
      Serial.printf("  Node %d (local): CAN age=%lums, RS485 age=%lums\n",
                    id, canGetLastRxAge(id), rs485GetLastRxAge(id));
    } else {
      Serial.printf("  Node %d: transport=%s | CAN age=%lums | RS485 age=%lums\n",
                    id, transportName[_transport[id]],
                    canGetLastRxAge(id), rs485GetLastRxAge(id));
    }
  }
  Serial.printf("  RS485 stats: %lu OK, %lu CRC errors\n",
                rs485GetRxOK(), rs485GetRxErr());
}
