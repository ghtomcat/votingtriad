// ===========================================================
// can_bus.cpp — CAN bus via ESP32 TWAI (built-in driver)
// No external library needed — driver/twai.h is part of ESP-IDF
// VotePacket split into 3 CAN frames of 8 bytes each
// ===========================================================
#include "can_bus.h"
#include "config.h"
#include "driver/twai.h"

// received packet table [index = node_id, 1-3]
static VotePacket _rxPackets[4]  = {};
static uint32_t   _rxTimes[4]   = {};
static bool       _rxHasData[4] = {};

// --- Frame A (8 bytes) ---
// [0-1] heading ×10 as uint16_t  (0..3600 → 0.0..360.0°)
// [2-3] pitch   ×10 as int16_t   (±1800 → ±180.0°)
// [4-5] roll    ×10 as int16_t
// [6]   health  (0=OK 1=WARN 2=FAIL)
// [7]   node_id (1/2/3)

// --- Frame B (8 bytes) ---
// [0-1] heading_error ×10  as int16_t  (±3276.7°)
// [2-3] yaw_rate      ×10  as int16_t  (±3276.7°/s)
// [4-5] accel_x       ×100 as int16_t  (±327.67 m/s²)
// [6-7] timestamp/10       as uint16_t (wraps every ~655s)

// --- Frame C (8 bytes) ---
// [0-1] altitude ×10  as int16_t  (±3276.7m AGL, 0.1m resolution)
// [2-3] vspeed   ×100 as int16_t  (±327.67 m/s, 0.01 m/s resolution)
// [4-7] reserved (zeros — airspeed/GPS in future)

static int16_t f2i16(float v, float scale) {
  float s = v * scale;
  if (s >  32767.0f) s =  32767.0f;
  if (s < -32768.0f) s = -32768.0f;
  return (int16_t)s;
}

void canInit() {
#ifdef SOLO_NODE
  pinMode(PIN_CAN_SE, OUTPUT);
  digitalWrite(PIN_CAN_SE, HIGH);  // transceiver off — no bus errors without other nodes
  Serial.println("[CAN] SOLO_NODE: CAN bus disabled.");
  return;
#endif

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 20;

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err != ESP_OK) {
    Serial.printf("[CAN] ERROR: driver_install failed (%d)\n", err);
    return;
  }
  twai_start();

  // silent pin LOW → transceiver active
  pinMode(PIN_CAN_SE, OUTPUT);
  digitalWrite(PIN_CAN_SE, LOW);

  Serial.printf("[CAN] TWAI initialized. Node %d TX IDs: 0x%03X / 0x%03X\n",
                NODE_ID, CAN_BASE_A + NODE_ID, CAN_BASE_B + NODE_ID);
}

void canSendPacket(const VotePacket& pkt) {
#ifdef SOLO_NODE
  return;
#endif

  twai_message_t msg = {};
  msg.extd             = 0;  // standard frame (11-bit ID)
  msg.data_length_code = 8;

  // --- Frame A: heading, pitch, roll, health, node ID ---
  msg.identifier = CAN_BASE_A + pkt.node_id;
  uint16_t hdg = (uint16_t)constrain(pkt.heading * 10.0f, 0.0f, 3600.0f);
  int16_t  pit = f2i16(pkt.pitch, 10.0f);
  int16_t  rol = f2i16(pkt.roll,  10.0f);
  msg.data[0] = (hdg >> 8) & 0xFF;  msg.data[1] = hdg & 0xFF;
  msg.data[2] = (pit >> 8) & 0xFF;  msg.data[3] = pit & 0xFF;
  msg.data[4] = (rol >> 8) & 0xFF;  msg.data[5] = rol & 0xFF;
  msg.data[6] = pkt.health;
  msg.data[7] = pkt.node_id;
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  // --- Frame B: heading error, yaw rate, accel X, timestamp ---
  msg.identifier = CAN_BASE_B + pkt.node_id;
  int16_t  herr = f2i16(pkt.heading_error, 10.0f);
  int16_t  yawr = f2i16(pkt.yaw_rate,      10.0f);
  int16_t  accx = f2i16(pkt.accel_x,      100.0f);
  uint16_t ts   = (uint16_t)(pkt.timestamp / 10);
  msg.data[0] = (herr >> 8) & 0xFF;  msg.data[1] = herr & 0xFF;
  msg.data[2] = (yawr >> 8) & 0xFF;  msg.data[3] = yawr & 0xFF;
  msg.data[4] = (accx >> 8) & 0xFF;  msg.data[5] = accx & 0xFF;
  msg.data[6] = (ts   >> 8) & 0xFF;  msg.data[7] = ts   & 0xFF;
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  // --- Frame C: altitude, vspeed ---
  msg.identifier = CAN_BASE_C + pkt.node_id;
  int16_t alt = f2i16(pkt.altitude, 10.0f);
  int16_t vsp = f2i16(pkt.vspeed,  100.0f);
  msg.data[0] = (alt >> 8) & 0xFF;  msg.data[1] = alt & 0xFF;
  msg.data[2] = (vsp >> 8) & 0xFF;  msg.data[3] = vsp & 0xFF;
  msg.data[4] = 0;  msg.data[5] = 0;  msg.data[6] = 0;  msg.data[7] = 0;
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void canReceive() {
#ifdef SOLO_NODE
  return;
#endif

  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    uint32_t id = msg.identifier;

    if (id >= (uint32_t)(CAN_BASE_A + 1) && id <= (uint32_t)(CAN_BASE_A + 3)) {
      uint8_t nid = (uint8_t)(id - CAN_BASE_A);
      if (nid == NODE_ID) continue;

      uint16_t hdg = ((uint16_t)msg.data[0] << 8) | msg.data[1];
      int16_t  pit = ((int16_t) msg.data[2] << 8) | msg.data[3];
      int16_t  rol = ((int16_t) msg.data[4] << 8) | msg.data[5];

      _rxPackets[nid].node_id = nid;
      _rxPackets[nid].heading = hdg / 10.0f;
      _rxPackets[nid].pitch   = pit / 10.0f;
      _rxPackets[nid].roll    = rol / 10.0f;
      _rxPackets[nid].health  = msg.data[6];
      _rxTimes[nid]   = millis();
      _rxHasData[nid] = true;
    }
    else if (id >= (uint32_t)(CAN_BASE_B + 1) && id <= (uint32_t)(CAN_BASE_B + 3)) {
      uint8_t nid = (uint8_t)(id - CAN_BASE_B);
      if (nid == NODE_ID) continue;

      int16_t  herr = ((int16_t) msg.data[0] << 8) | msg.data[1];
      int16_t  yawr = ((int16_t) msg.data[2] << 8) | msg.data[3];
      int16_t  accx = ((int16_t) msg.data[4] << 8) | msg.data[5];
      uint16_t ts   = ((uint16_t)msg.data[6] << 8) | msg.data[7];

      _rxPackets[nid].heading_error = herr / 10.0f;
      _rxPackets[nid].yaw_rate      = yawr / 10.0f;
      _rxPackets[nid].accel_x       = accx / 100.0f;
      _rxPackets[nid].timestamp     = (uint32_t)ts * 10;
    }
    else if (id >= (uint32_t)(CAN_BASE_C + 1) && id <= (uint32_t)(CAN_BASE_C + 3)) {
      uint8_t nid = (uint8_t)(id - CAN_BASE_C);
      if (nid == NODE_ID) continue;

      int16_t alt = ((int16_t)msg.data[0] << 8) | msg.data[1];
      int16_t vsp = ((int16_t)msg.data[2] << 8) | msg.data[3];

      _rxPackets[nid].altitude = alt / 10.0f;
      _rxPackets[nid].vspeed   = vsp / 100.0f;
    }
  }
}

bool canGetNodePacket(uint8_t node_id, VotePacket& out) {
  if (node_id < 1 || node_id > 3) return false;
  if (!_rxHasData[node_id])       return false;
  out = _rxPackets[node_id];
  return true;
}

uint32_t canGetLastRxAge(uint8_t node_id) {
  if (node_id < 1 || node_id > 3) return 0xFFFFFFFF;
  if (!_rxHasData[node_id])       return 0xFFFFFFFF;
  return millis() - _rxTimes[node_id];
}
