// ===========================================================
// rs485.cpp — RS485 fallback communication
// UART2: RX=IO21, TX=IO22, EN=IO17
// Half-duplex: EN HIGH = transmit, EN LOW = receive
// Time-division multiplexing prevents collisions without arbitration
// ===========================================================
#include "rs485.h"
#include "config.h"

// received packet table
static VotePacket _rxPkts[4]    = {};
static uint32_t   _rxTimes[4]  = {};
static bool       _rxHasData[4] = {};

// statistics
static uint32_t _rxOK  = 0;
static uint32_t _rxErr = 0;

// frame parser state machine
static uint8_t _buf[RS485_FRAME_LEN];
static uint8_t _bufPos  = 0;
static bool    _inFrame = false;

// last transmit slot (for slot timing)
static uint32_t _lastSendSlot = 0xFFFFFFFF;

// ===========================================================
// CRC8 (Dallas/Maxim)
// ===========================================================
static uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc ^ byte) & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
      byte <<= 1;
    }
  }
  return crc;
}

// float → int16_t with scale and clamp
static int16_t f2i16(float v, float scale) {
  float s = v * scale;
  if (s >  32767.0f) s =  32767.0f;
  if (s < -32768.0f) s = -32768.0f;
  return (int16_t)s;
}

// set EN pin: transmit or receive mode
static void setTxMode(bool tx) {
  digitalWrite(PIN_RS485_EN, tx ? HIGH : LOW);
  if (tx) delayMicroseconds(10);
}

// build and send a frame
static void sendFrame(const VotePacket& pkt) {
  uint8_t frame[RS485_FRAME_LEN];

  frame[0] = RS485_START_BYTE;
  frame[1] = pkt.node_id;

  uint16_t hdg  = (uint16_t)constrain(pkt.heading * 10.0f, 0.0f, 3600.0f);
  int16_t  pit  = f2i16(pkt.pitch,         10.0f);
  int16_t  rol  = f2i16(pkt.roll,          10.0f);
  int16_t  herr = f2i16(pkt.heading_error, 10.0f);
  int16_t  yawr = f2i16(pkt.yaw_rate,      10.0f);
  int16_t  accx = f2i16(pkt.accel_x,      100.0f);
  uint16_t ts   = (uint16_t)(pkt.timestamp / 10);

  // big-endian encoding
  frame[2]  = (hdg  >> 8) & 0xFF;  frame[3]  = hdg  & 0xFF;
  frame[4]  = (pit  >> 8) & 0xFF;  frame[5]  = pit  & 0xFF;
  frame[6]  = (rol  >> 8) & 0xFF;  frame[7]  = rol  & 0xFF;
  frame[8]  = (herr >> 8) & 0xFF;  frame[9]  = herr & 0xFF;
  frame[10] = (yawr >> 8) & 0xFF;  frame[11] = yawr & 0xFF;
  frame[12] = (accx >> 8) & 0xFF;  frame[13] = accx & 0xFF;
  frame[14] = pkt.health;
  frame[15] = (ts   >> 8) & 0xFF;  frame[16] = ts   & 0xFF;

  // CRC over payload bytes 1–16
  frame[17] = crc8(&frame[1], 16);
  frame[18] = RS485_END_BYTE;

  setTxMode(true);
  Serial2.write(frame, RS485_FRAME_LEN);
  Serial2.flush();
  setTxMode(false);
}

// parse a complete frame and store in table
static void parseFrame(const uint8_t* f) {
  if (f[18] != RS485_END_BYTE) { _rxErr++; return; }

  uint8_t calcCrc = crc8(&f[1], 16);
  if (calcCrc != f[17]) {
    _rxErr++;
    Serial.printf("[RS485] CRC error: expected 0x%02X, got 0x%02X\n",
                  calcCrc, f[17]);
    return;
  }

  uint8_t nid = f[1];
  if (nid < 1 || nid > 3 || nid == NODE_ID) return;

  uint16_t hdg  = ((uint16_t)f[2]  << 8) | f[3];
  int16_t  pit  = ((int16_t) f[4]  << 8) | f[5];
  int16_t  rol  = ((int16_t) f[6]  << 8) | f[7];
  int16_t  herr = ((int16_t) f[8]  << 8) | f[9];
  int16_t  yawr = ((int16_t) f[10] << 8) | f[11];
  int16_t  accx = ((int16_t) f[12] << 8) | f[13];
  uint16_t ts   = ((uint16_t)f[15] << 8) | f[16];

  _rxPkts[nid].node_id       = nid;
  _rxPkts[nid].heading       = hdg  / 10.0f;
  _rxPkts[nid].pitch         = pit  / 10.0f;
  _rxPkts[nid].roll          = rol  / 10.0f;
  _rxPkts[nid].heading_error = herr / 10.0f;
  _rxPkts[nid].yaw_rate      = yawr / 10.0f;
  _rxPkts[nid].accel_x       = accx / 100.0f;
  _rxPkts[nid].health        = f[14];
  _rxPkts[nid].timestamp     = (uint32_t)ts * 10;

  _rxTimes[nid]   = millis();
  _rxHasData[nid] = true;
  _rxOK++;
}

// ===========================================================
// Public API
// ===========================================================

void rs485Init() {
  pinMode(PIN_RS485_EN, OUTPUT);
  setTxMode(false);  // default: receive mode

  Serial2.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  Serial.printf("[RS485] Init: TX=IO%d RX=IO%d EN=IO%d @ %d baud\n",
                PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_EN, RS485_BAUD);
}

bool rs485Send(const VotePacket& pkt) {
  // transmit slot: node 1 = 0ms, node 2 = 20ms, node 3 = 40ms
  uint32_t slotOffset = (uint32_t)(NODE_ID - 1) * RS485_SLOT_MS;
  uint32_t cyclePos   = millis() % RS485_CYCLE_MS;

  bool inMySlot = (cyclePos >= slotOffset &&
                   cyclePos <  slotOffset + RS485_SLOT_MS);

  // transmit only once per slot
  uint32_t currentSlot = millis() / RS485_CYCLE_MS;
  if (!inMySlot || currentSlot == _lastSendSlot) return false;

  _lastSendSlot = currentSlot;
  sendFrame(pkt);
  return true;
}

void rs485Receive() {
  // non-blocking: process all available bytes
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (!_inFrame) {
      if (b == RS485_START_BYTE) {
        _buf[0]  = b;
        _bufPos  = 1;
        _inFrame = true;
      }
    } else {
      if (_bufPos < RS485_FRAME_LEN) {
        _buf[_bufPos++] = b;
      }
      if (_bufPos == RS485_FRAME_LEN) {
        parseFrame(_buf);
        _inFrame = false;
        _bufPos  = 0;
      }
    }
  }
}

bool rs485GetNodePacket(uint8_t node_id, VotePacket& out) {
  if (node_id < 1 || node_id > 3) return false;
  if (!_rxHasData[node_id])       return false;
  out = _rxPkts[node_id];
  return true;
}

uint32_t rs485GetLastRxAge(uint8_t node_id) {
  if (node_id < 1 || node_id > 3) return 0xFFFFFFFF;
  if (!_rxHasData[node_id])       return 0xFFFFFFFF;
  return millis() - _rxTimes[node_id];
}

uint32_t rs485GetRxOK()  { return _rxOK;  }
uint32_t rs485GetRxErr() { return _rxErr; }
