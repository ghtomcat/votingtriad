#pragma once
// ===========================================================
// config.h — Central configuration for all nodes
// ===========================================================

// --- Node ID set via build flag; fallback = 1 ---
#ifndef NODE_ID
  #define NODE_ID 1
#endif
#ifndef IS_MASTER
  #define IS_MASTER 0
#endif

// ===========================================================
// PIN ASSIGNMENT — LilyGo T-CAN485
// ===========================================================

// CAN Bus
#define PIN_CAN_RX      26
#define PIN_CAN_TX      27
#define PIN_CAN_SE      23  // Silent Enable (LOW = active)

// RS485 (unused in this FW, reserved)
#define PIN_RS485_RX    21
#define PIN_RS485_TX    22
#define PIN_RS485_EN    17

// SD Card (SPI)
#define PIN_SD_MOSI     15
#define PIN_SD_MISO      2
#define PIN_SD_SCLK     14
#define PIN_SD_CS       13

// WS2812 LED
#define PIN_LED          4

// I2C for BNO055
#define PIN_I2C_SDA     32
#define PIN_I2C_SCL     33

// CRSF RC input — single wire, 420000 baud (ExpressLRS / RadioMaster)
#define PIN_RC_RX       25  // Serial1 RX — connect to receiver CRSF TX pin
#define PIN_RC_TX        5  // Serial1 TX — connect to receiver CRSF RX pin (telemetry)

// Servo / ESC outputs — driven by PCA9685 over I2C (not direct GPIO)
#define PCA9685_I2C_ADDR  0x40  // default (all addr pins low)

// ===========================================================
// VEHICLE TYPE — selects channel mapping and control law
// ===========================================================
#define VEHICLE_HOVERCRAFT  0
#define VEHICLE_AIRPLANE    1
#define VEHICLE_TYPE        VEHICLE_AIRPLANE  // change to VEHICLE_HOVERCRAFT for hovercraft

#if VEHICLE_TYPE == VEHICLE_AIRPLANE
  // PCA9685 output channels
  #define PCA9685_CH_AILERON   0
  #define PCA9685_CH_ELEVATOR  1
  #define PCA9685_CH_RUDDER    2
  #define PCA9685_CH_THROTTLE  3
  // RC input channels (Mode 2 standard)
  #define RC_CH_AILERON        1
  #define RC_CH_ELEVATOR       2
  #define RC_CH_THROTTLE       3
  #define RC_CH_RUDDER         4
#else
  // PCA9685 output channels
  #define PCA9685_CH_RUDDER    0
  #define PCA9685_CH_THRUST    1
  #define PCA9685_CH_LIFT      2
  // RC input channels
  #define RC_CH_RUDDER         1
  #define RC_CH_THRUST         2
  #define RC_CH_LIFT           3
#endif

// ===========================================================
// CAN BUS
// ===========================================================
#define CAN_SPEED_KBPS      500
#define CAN_TX_INTERVAL_MS   20   // VotePacket every 20ms

// CAN Message IDs
// Frame A (Heading/Pitch/Roll/Health): 0x100 + node_id
// Frame B (Error/Yaw/Accel/Time):     0x200 + node_id
// Frame C (Altitude/VSpeed):          0x300 + node_id
#define CAN_BASE_A          0x100
#define CAN_BASE_B          0x200
#define CAN_BASE_C          0x300

// ===========================================================
// BNO055
// ===========================================================
#define BNO055_I2C_ADDR     0x28
#define BNO055_SAMPLERATE   20   // ms between IMU samples

// ===========================================================
// BMP390 BAROMETER
// ===========================================================
#define BMP390_I2C_ADDR       0x77  // SDO to VCC; use 0x76 if SDO to GND
#define BMP390_SAMPLERATE     20    // ms between samples (50Hz)
#define BMP390_SEA_LEVEL_HPA  1013.25f  // standard atmosphere; adjust for local QNH

// ===========================================================
// VOTING & ENVELOPE PROTECTION
// ===========================================================

#define HEADING_TOLERANCE   5.0f   // degrees for consensus
#define ALTITUDE_TOLERANCE  10.0f  // metres for altitude consensus

// Envelope limits
#define MAX_LIFT_THROTTLE   0.80f  // 80% ESC cap
#define MAX_THRUST_ACCEL    0.80f  // g
#define MAX_YAW_RATE        30.0f  // degrees/second

// Node health timeouts
#define NODE_WARN_TIMEOUT_MS  100  // ms without packet → WARN
#define NODE_FAIL_TIMEOUT_MS  500  // ms without packet → FAIL

// ===========================================================
// PID — Heading Hold (Assisted Mode)
// ===========================================================
#define PID_KP              0.80f
#define PID_KI              0.01f
#define PID_KD              0.05f
#define PID_INTEGRAL_LIMIT  10.0f  // anti-windup

// ===========================================================
// RC INPUT
// ===========================================================
#define RC_PWM_MIN          1000   // µs
#define RC_PWM_MID          1500   // µs
#define RC_PWM_MAX          2000   // µs
#define RC_DEADBAND           50   // µs either side of center
#define RC_TIMEOUT_MS        250   // ms without signal → failsafe

// Which iBUS channel carries the mode switch
#define RC_MODE_CHANNEL      6     // CH6: Manual / Assisted / Autonomous

// Mode switch thresholds
#define RC_MODE_MANUAL_MAX  1300   // µs: Manual
#define RC_MODE_ASSISTED_MAX 1700  // µs: Assisted (Heading Hold)
// > 1700 µs: Autonomous

// ===========================================================
// SERVO / ESC OUTPUT (PCA9685, 50Hz, 12-bit)
// ===========================================================
#define SERVO_FREQ_HZ        50    // standard servo/ESC frequency
#define SERVO_MIN_US       1000    // µs = full left / off
#define SERVO_MID_US       1500    // µs = center / neutral
#define SERVO_MAX_US       2000    // µs = full right / full throttle

// ===========================================================
// WS2812 LED
// ===========================================================
#define NUM_LEDS              1
#define LED_BRIGHTNESS       80    // 0-255
#define LED_BLINK_MS        500    // slow blink interval (CAN search)

// ===========================================================
// TELEMETRY / WIFI (Node 1 / Master only)
// ===========================================================
#define WIFI_SSID         "HovercraftAP"
#define WIFI_PASSWORD     "hovercraft123"
#define WIFI_AP_MODE       true            // true = own access point

#define TELEMETRY_PORT       8080
#define TELEMETRY_INTERVAL_MS 100  // JSON every 100ms

// ===========================================================
// SD LOGGING (Node 1 / Master only)
// ===========================================================
#define SD_LOG_INTERVAL_MS  100   // log every 100ms
#define SD_LOG_FILENAME     "/hoverlog.csv"

// ===========================================================
// BOOT SEQUENCE
// ===========================================================
#define BOOT_WAIT_ALL_NODES_MS  5000  // max wait for all 3 nodes
#define BOOT_BNO_TIMEOUT_MS     3000  // max wait for BNO055 calibration

// ===========================================================
// HIL — Hardware-In-the-Loop via OpenSim WebSocket hub
// ===========================================================
#define HIL_WIFI_SSID    "TP-Link_DEF8"
#define HIL_WIFI_PASS    "22480828"     // fill in your WiFi password
#define HIL_HUB_HOST     "192.168.1.102"
#define HIL_HUB_PORT     3000
#define HIL_ROOM         "hil-corsair"
#define HIL_UPDATE_MS    20                  // 50Hz control send rate

// Corsair envelope limits for HIL (from f4u1a.json)
#define HIL_MAX_BANK_DEG   90.0f
#define HIL_MAX_PITCH_DEG  30.0f
#define HIL_MAX_SPD_KT    362.0f

// ===========================================================
// AIRSPEED — MPXV7002DP differential pressure sensor
// ADC1_CH7 / GPIO35 (input-only pin on LilyGo T-CAN485)
// Pitot: P1 = total pressure, P2 = static port
// ===========================================================

// ADC pin — input-only, safe with WiFi (ADC1)
#define PIN_AIRSPEED_ADC      34

// Sensor supply voltage (V). If powering from 3.3 V rail, use 3.3f.
// If powering from 5 V with a voltage divider on the output, use 5.0f
// and ensure the divided output never exceeds 3.3 V on the ESP32 pin.
#define AIRSPEED_SENSOR_VCC   3.3f

// ESP32 ADC full-scale for ADC_11db attenuation (mV).
// Nominal 3900 mV; tune slightly if zero offset drifts after calibration.
#define AIRSPEED_ADC_VMAX_MV  3900

// Zero calibration: samples averaged at boot (sensor must be unpressurised)
#define AIRSPEED_ZERO_SAMPLES 64

// Update rate
#define AIRSPEED_SAMPLERATE_MS  20    // ms between updates (50 Hz)

// Noise floor — differential pressures below this are treated as zero
// ~2 Pa ≈ 2.6 kt; keeps the IAS at 0 when stationary
#define AIRSPEED_NOISE_FLOOR_PA 2.0f

// Air density at sea level, ISA (kg/m³)
// For altitude-corrected TAS, update with baro + temperature data
#define RHO_SEA_LEVEL_KGM3    1.225f

// Stall warning threshold (knots)
#define AIRSPEED_STALL_KT     35.0f

// ===========================================================
// SYSTEM
// ===========================================================
#define SERIAL_BAUD         115200
#define MAIN_LOOP_INTERVAL_MS  5   // main loop rate ~200Hz
