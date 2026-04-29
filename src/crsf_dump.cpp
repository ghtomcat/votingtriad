// crsf_dump.cpp — CRSF frame decoder for receiver diagnostics
// Flash: pio run -e crsf_dump -t upload

#include <Arduino.h>

#define PIN_RC_RX     25   // receiver TX → GPIO25
#define PIN_RC_TX      5   // ESP32 TX → receiver RX
#define CRSF_BAUD     420000
#define CRSF_SYNC     0xC8
#define MAX_FRAME     64

static uint8_t  _buf[MAX_FRAME];
static uint8_t  _idx      = 0;
static bool     _inFrame  = false;
static uint8_t  _frameLen = 0;
static uint32_t _rxBytes  = 0;
static uint32_t _lastReport = 0;

static uint8_t crsfCrc8(const uint8_t* buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

static void decodeFrame(uint8_t type, const uint8_t* payload, uint8_t payLen) {
  switch (type) {

    case 0x16: {  // RC channels
      if (payLen < 22) { Serial.println("[RC] short frame"); return; }
      uint16_t ch1 = (payload[0] | (uint16_t)payload[1] << 8) & 0x7FF;
      uint16_t ch2 = (payload[1] >> 3 | (uint16_t)payload[2] << 5) & 0x7FF;
      uint16_t ch3 = (payload[2] >> 6 | (uint16_t)payload[3] << 2 | (uint16_t)payload[4] << 10) & 0x7FF;
      uint16_t ch4 = (payload[4] >> 1 | (uint16_t)payload[5] << 7) & 0x7FF;
      Serial.printf("[RC]   CH1:%4d  CH2:%4d  CH3:%4d  CH4:%4d  (raw, 172-1811)\n",
                    ch1, ch2, ch3, ch4);
      break;
    }

    case 0x14: {  // Link statistics
      if (payLen < 10) { Serial.println("[LINK] short frame"); return; }
      int8_t  rssi1 = -(int8_t)payload[0];
      int8_t  rssi2 = -(int8_t)payload[1];
      uint8_t lq    = payload[2];
      int8_t  snr   = (int8_t)payload[3];
      uint8_t ant   = payload[4];
      uint8_t mode  = payload[5];
      uint8_t txpwr = payload[6];
      int8_t  drssi = -(int8_t)payload[7];
      uint8_t dlq   = payload[8];
      Serial.printf("[LINK] RSSI:%ddBm  LQ:%d%%  SNR:%ddB  Ant:%d  Mode:%d  TXpwr:%d  DL_RSSI:%ddBm  DL_LQ:%d%%\n",
                    rssi1, lq, snr, ant, mode, txpwr, drssi, dlq);
      break;
    }

    case 0x08:   // battery sensor
    case 0x1C:   // GPS
    case 0x0E:   // ELRS info
    default: {
      Serial.printf("[0x%02X] payLen=%d  raw:", type, payLen);
      for (uint8_t i = 0; i < payLen && i < 16; i++)
        Serial.printf(" %02X", payload[i]);
      Serial.println();
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CRSF Decoder — GPIO34, 420000 baud ===");
  Serial2.begin(CRSF_BAUD, SERIAL_8N1, PIN_RC_RX, PIN_RC_TX);
}

void loop() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    _rxBytes++;

    if (!_inFrame) {
      if (b == CRSF_SYNC) {
        _buf[0] = b; _idx = 1; _inFrame = true;
      }
      continue;
    }

    _buf[_idx++] = b;

    if (_idx == 2) {
      _frameLen = b;
      if (_frameLen < 2 || (2 + _frameLen) > MAX_FRAME) {
        _inFrame = false; _idx = 0;
      }
      continue;
    }

    if (_idx < (uint8_t)(2 + _frameLen)) continue;

    _inFrame = false; _idx = 0;

    uint8_t type    = _buf[2];
    uint8_t payLen  = _frameLen - 2;
    uint8_t crcRx   = _buf[1 + _frameLen];
    uint8_t crcCalc = crsfCrc8(&_buf[2], _frameLen - 1);

    if (crcRx != crcCalc) {
      Serial.printf("[BAD CRC] type=0x%02X got=0x%02X want=0x%02X\n", type, crcRx, crcCalc);
      continue;
    }

    decodeFrame(type, &_buf[3], payLen);
  }

  uint32_t now = millis();
  if (now - _lastReport >= 2000) {
    _lastReport = now;
    if (_rxBytes == 0) Serial.println("[SILENT] no bytes from receiver");
    _rxBytes = 0;
  }
}
