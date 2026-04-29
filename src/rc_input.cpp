// ===========================================================
// rc_input.cpp — CRSF RC input parser (ExpressLRS / RadioMaster)
// CRSF frame: [addr 0xC8][len][type][payload...][crc8 poly 0xD5]
// RC channels frame (type 0x16): 16ch × 11 bits packed into 22 bytes
// Channel range: 172 (1000µs) — 992 (1500µs) — 1811 (2000µs)
// ===========================================================
#include "rc_input.h"
#include "config.h"

#define CRSF_BAUDRATE        420000
#define CRSF_SYNC            0xC8   // flight controller address
#define CRSF_TYPE_RC         0x16
#define CRSF_RC_PAYLOAD_LEN  22
#define CRSF_MAX_FRAME_LEN   64
#define CRSF_MAX_CH          16
#define CRSF_CH_MIN          172
#define CRSF_CH_MAX          1811

static uint8_t  _buf[CRSF_MAX_FRAME_LEN];
static uint8_t  _bufIdx   = 0;
static bool     _inFrame  = false;
static uint8_t  _frameLen = 0;   // = len byte value

static uint16_t _ch[CRSF_MAX_CH + 1];   // 1-indexed; [0] unused
static uint32_t _lastPacket = 0;
static uint32_t _rxBytes    = 0;
static uint32_t _goodPkts   = 0;
static uint32_t _badPkts    = 0;

static uint8_t crsfCrc8(const uint8_t* buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
  }
  return crc;
}

static uint16_t crsfToUs(uint16_t raw) {
  // 172 → 1000µs, 1811 → 2000µs (linear)
  float us = (float)(raw - CRSF_CH_MIN) * 1000.0f / (float)(CRSF_CH_MAX - CRSF_CH_MIN) + 1000.0f;
  if (us < 1000.0f) return 1000;
  if (us > 2000.0f) return 2000;
  return (uint16_t)us;
}

// 16 channels × 11 bits LSB-first packed into 22 bytes
static void parseChannels(const uint8_t* p) {
  uint16_t raw[CRSF_MAX_CH + 1];
  raw[1]  = ( p[0]       | (uint16_t)p[1]  << 8)                         & 0x7FF;
  raw[2]  = ( p[1]  >> 3 | (uint16_t)p[2]  << 5)                         & 0x7FF;
  raw[3]  = ( p[2]  >> 6 | (uint16_t)p[3]  << 2 | (uint16_t)p[4]  << 10) & 0x7FF;
  raw[4]  = ( p[4]  >> 1 | (uint16_t)p[5]  << 7)                         & 0x7FF;
  raw[5]  = ( p[5]  >> 4 | (uint16_t)p[6]  << 4)                         & 0x7FF;
  raw[6]  = ( p[6]  >> 7 | (uint16_t)p[7]  << 1 | (uint16_t)p[8]  <<  9) & 0x7FF;
  raw[7]  = ( p[8]  >> 2 | (uint16_t)p[9]  << 6)                         & 0x7FF;
  raw[8]  = ( p[9]  >> 5 | (uint16_t)p[10] << 3)                         & 0x7FF;
  raw[9]  = ( p[11]      | (uint16_t)p[12] << 8)                         & 0x7FF;
  raw[10] = ( p[12] >> 3 | (uint16_t)p[13] << 5)                         & 0x7FF;
  raw[11] = ( p[13] >> 6 | (uint16_t)p[14] << 2 | (uint16_t)p[15] << 10) & 0x7FF;
  raw[12] = ( p[15] >> 1 | (uint16_t)p[16] << 7)                         & 0x7FF;
  raw[13] = ( p[16] >> 4 | (uint16_t)p[17] << 4)                         & 0x7FF;
  raw[14] = ( p[17] >> 7 | (uint16_t)p[18] << 1 | (uint16_t)p[19] <<  9) & 0x7FF;
  raw[15] = ( p[19] >> 2 | (uint16_t)p[20] << 6)                         & 0x7FF;
  raw[16] = ( p[20] >> 5 | (uint16_t)p[21] << 3)                         & 0x7FF;

  for (uint8_t i = 1; i <= CRSF_MAX_CH; i++) _ch[i] = crsfToUs(raw[i]);
}

void rcInit() {
  for (uint8_t i = 1; i <= CRSF_MAX_CH; i++) _ch[i] = RC_PWM_MID;
  // TX pin = -1 (receive only)
  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RC_RX, PIN_RC_TX);
  Serial.printf("[RC] CRSF initialized on GPIO%d/GPIO%d (Serial1, 420000 baud).\n", PIN_RC_RX, PIN_RC_TX);
}

void rcUpdate() {
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    _rxBytes++;

    if (!_inFrame) {
      if (b == CRSF_SYNC) {
        _buf[0]  = b;
        _bufIdx  = 1;
        _inFrame = true;
      }
      continue;
    }

    _buf[_bufIdx++] = b;

    // second byte is the frame length
    if (_bufIdx == 2) {
      _frameLen = b;
      if (_frameLen < 2 || (uint8_t)(2 + _frameLen) > CRSF_MAX_FRAME_LEN) {
        _inFrame = false; _bufIdx = 0; _badPkts++;
      }
      continue;
    }

    // wait until full frame is buffered: sync(1) + len(1) + frameLen bytes
    if (_bufIdx < (uint8_t)(2 + _frameLen)) continue;

    _inFrame = false;
    _bufIdx  = 0;

    uint8_t type     = _buf[2];
    uint8_t payLen   = _frameLen - 2;           // excludes type and crc
    uint8_t crcRx    = _buf[1 + _frameLen];     // last byte of frame
    uint8_t crcCalc  = crsfCrc8(&_buf[2], _frameLen - 1);  // type + payload

    if (crcRx != crcCalc) { _badPkts++; continue; }

    if (type == CRSF_TYPE_RC && payLen == CRSF_RC_PAYLOAD_LEN) {
      parseChannels(&_buf[3]);
      _lastPacket = millis();
      _goodPkts++;
    }
  }
}

uint16_t rcGetPWM(uint8_t ch) {
  if (ch < 1 || ch > CRSF_MAX_CH) return RC_PWM_MID;
  return _ch[ch];
}

float rcGetNormalized(uint8_t ch) {
  uint16_t pwm = rcGetPWM(ch);
  if (ch == 1) {
    if (abs((int)pwm - RC_PWM_MID) < RC_DEADBAND) return 0.0f;
    float norm = (float)(pwm - RC_PWM_MID) / (float)(RC_PWM_MAX - RC_PWM_MID);
    return constrain(norm, -1.0f, 1.0f);
  } else {
    float norm = (float)(pwm - RC_PWM_MIN) / (float)(RC_PWM_MAX - RC_PWM_MIN);
    return constrain(norm, 0.0f, 1.0f);
  }
}

RCMode rcGetMode() {
  uint16_t val = rcGetPWM(RC_MODE_CHANNEL);
  if      (val < RC_MODE_MANUAL_MAX)   return RC_MANUAL;
  else if (val < RC_MODE_ASSISTED_MAX) return RC_ASSISTED;
  else                                  return RC_AUTONOMOUS;
}

bool rcIsActive() {
  return (millis() - _lastPacket) < RC_TIMEOUT_MS;
}

void rcGetStats(uint32_t& rxBytes, uint32_t& goodPkts, uint32_t& badPkts) {
  rxBytes  = _rxBytes;
  goodPkts = _goodPkts;
  badPkts  = _badPkts;
}
