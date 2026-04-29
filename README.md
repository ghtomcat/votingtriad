# VotingTriad — Airbus-Style Voting Flight Computer

A 3-node fault-tolerant flight computer for RC aircraft and hovercraft, built on the LilyGo T-CAN485 (ESP32). Inspired by Airbus envelope protection: three independent nodes continuously vote on vehicle state via CAN bus. The majority rules. Any node can become master.

Includes full **Hardware-In-the-Loop (HIL)** integration with the [OpenSim](../OpenSim) browser-based flight simulator — fly a simulated Corsair with real RC sticks and real flight computer logic.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      CAN Bus (500 kbps)                         │
│                   + RS485 fallback (115200)                     │
│                                                                 │
│  ┌────────────┐       ┌────────────┐       ┌────────────┐       │
│  │   Node 1   │◄─────►│   Node 2   │◄─────►│   Node 3   │       │
│  │   MASTER   │       │   VOTER    │       │   VOTER    │       │
│  │            │       │            │       │            │       │
│  │ BNO055 IMU │       │ BNO055 IMU │       │ BNO055 IMU │       │
│  │ BMP390 Bar │       │ BMP390 Bar │       │ BMP390 Bar │       │
│  │ CRSF RC    │       │            │       │            │       │
│  │ PCA9685    │       │            │       │            │       │
│  │ SD Logger  │       │            │       │            │       │
│  │ Telemetry  │       │            │       │            │       │
│  └────────────┘       └────────────┘       └────────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

### Voting Logic

Each node broadcasts its IMU data every 20ms. The master collects all three and votes:

| Agreement | Envelope Mode | Description                        |
|-----------|---------------|------------------------------------|
| 3/3 nodes | NORMAL LAW    | Full envelope protection active    |
| 2/3 nodes | DEGRADED      | Reduced limits, warning LED        |
| 1/3 nodes | DIRECT LAW    | Pass-through only, no protection   |
| 0/3 nodes | DISARM        | All outputs locked                 |

Any node can be promoted to master if the current master goes silent for >500ms.

---

## Hardware

### LilyGo T-CAN485

ESP32-based board with onboard CAN transceiver and RS485 transceiver. All three nodes use identical hardware.

**Exposed GPIO header:** IO25, IO32, IO33, IO5, IO12, IO34, IO35, IO18  
**Note:** IO34 and IO35 are input-only — cannot be used as UART TX or digital output.

### Pin Assignment (all nodes)

| Function          | GPIO | Notes                                  |
|-------------------|------|----------------------------------------|
| CAN RX            | 26   | Onboard transceiver                    |
| CAN TX            | 27   | Onboard transceiver                    |
| CAN Silent/Enable | 23   | LOW = active                           |
| RS485 RX          | 21   | Onboard transceiver                    |
| RS485 TX          | 22   | Onboard transceiver                    |
| RS485 Enable      | 17   | HIGH = transmit, LOW = receive         |
| I2C SDA           | 32   | BNO055 + BMP390 + PCA9685             |
| I2C SCL           | 33   | BNO055 + BMP390 + PCA9685             |
| WS2812 LED        | 4    | 470Ω series resistor recommended       |
| SD MOSI           | 15   | SPI                                    |
| SD MISO           | 2    | SPI                                    |
| SD SCLK           | 14   | SPI                                    |
| SD CS             | 13   | SPI                                    |

### Pin Assignment (Node 1 / Master only)

| Function          | GPIO | Notes                                  |
|-------------------|------|----------------------------------------|
| CRSF RC RX        | 25   | Receiver TX → GPIO25 (Serial1)         |
| CRSF RC TX        | 5    | ESP32 TX → Receiver RX (telemetry)     |

### I2C Devices (shared bus, GPIO32/33)

| Device  | Address | Function                     |
|---------|---------|------------------------------|
| BNO055  | 0x28    | IMU — heading, pitch, roll, yaw rate, accel |
| BMP390  | 0x76    | Barometer — altitude, vertical speed |
| PCA9685 | 0x40    | PWM servo/ESC driver (16 channels, 50Hz) |

---

## Sensor Wiring

### BNO055 IMU

```
BNO055          LilyGo T-CAN485
────────────────────────────────
VCC     ──►    3.3V
GND     ──►    GND
SDA     ──►    GPIO32
SCL     ──►    GPIO33
ADR     ──►    GND  (I2C address 0x28)
```

### BMP390 Barometer

```
BMP390          LilyGo T-CAN485
────────────────────────────────
VCC     ──►    3.3V
GND     ──►    GND
SDA     ──►    GPIO32
SCL     ──►    GPIO33
SDO     ──►    GND  (I2C address 0x76)
```

### PCA9685 Servo Driver

```
PCA9685         LilyGo T-CAN485
────────────────────────────────
VCC     ──►    3.3V  (logic)
GND     ──►    GND
SDA     ──►    GPIO32
SCL     ──►    GPIO33
V+      ──►    5–6V  (servo power, separate supply)
OE      ──►    GND   (output enable, active LOW)
A0–A5   ──►    GND   (address 0x40)
```

All servo/ESC signal wires connect to PCA9685 channels. The PCA9685 V+ rail must have its own power supply — do not power servos from the ESP32 3.3V pin.

---

## Vehicle Types

The firmware supports two vehicle configurations, selected at compile time in `src/config.h`:

```cpp
#define VEHICLE_TYPE  VEHICLE_AIRPLANE   // or VEHICLE_HOVERCRAFT
```

### Airplane (VEHICLE_AIRPLANE)

| PCA9685 Channel | Function  | RC Channel | Control          |
|-----------------|-----------|------------|------------------|
| CH0             | Aileron   | CH1        | Roll (±1.0)      |
| CH1             | Elevator  | CH2        | Pitch (±1.0)     |
| CH2             | Rudder    | CH4        | Yaw (±1.0)       |
| CH3             | Throttle  | CH3        | Thrust (0–1.0)   |

Standard Mode 2 channel mapping. Aileron center = 1500µs, throttle min = 1000µs.

### Hovercraft (VEHICLE_HOVERCRAFT)

| PCA9685 Channel | Function | RC Channel | Control        |
|-----------------|----------|------------|----------------|
| CH0             | Rudder   | CH1        | Yaw (±1.0)     |
| CH1             | Thrust   | CH2        | Thrust (0–1.0) |
| CH2             | Lift     | CH3        | Lift (0–1.0)   |

---

## RC Input — ExpressLRS / CRSF

The firmware reads CRSF (Crossfire Serial Protocol) from an ExpressLRS receiver (tested with RadioMaster RP4 TD).

**Protocol:** 420000 baud, 8N1, on Serial1 (GPIO25 RX / GPIO5 TX)  
**Frame rate:** ~150Hz (16 channels, 11-bit resolution)  
**Failsafe:** outputs disarm after 250ms without a valid frame

### CRSF Wiring

```
RP4 TD Receiver     LilyGo T-CAN485
───────────────────────────────────
TX          ──►    GPIO25  (CRSF data to ESP32)
RX          ──►    GPIO5   (telemetry from ESP32)
GND         ──►    GND
5V          ──►    5V
```

### RC Modes (CH6)

| PWM Value  | Mode       | Behavior                                      |
|------------|------------|-----------------------------------------------|
| < 1300µs   | MANUAL     | Direct pass-through, envelope limits active   |
| 1300–1700µs| ASSISTED   | Heading hold via PID (airplane: direct)       |
| > 1700µs   | AUTONOMOUS | Reserved for future autopilot                 |

---

## CAN Bus

Primary inter-node communication. All three nodes are on a single bus.

```
Node 1 ─── Node 2 ─── Node 3
  CAN H ────────────────────── 120Ω
  CAN L ────────────────────── 120Ω
```

120Ω termination resistors at both ends of the bus. Use twisted pair cable, max ~20m at 500 kbps.

Each node sends three CAN frames every 20ms:

| CAN ID     | Contents                              |
|------------|---------------------------------------|
| 0x100+node | Heading, pitch, roll, health          |
| 0x200+node | Heading error, yaw rate, accel X, time|
| 0x300+node | Altitude, vertical speed              |

### RS485 Fallback

If CAN is silent for >200ms, the system automatically switches to RS485 (time-division multiplexed, one slot per node per 60ms cycle). CAN recovery is detected automatically and the fallback deactivates.

---

## Envelope Protection

### NORMAL LAW limits (hovercraft)

| Parameter        | Limit | Unit  |
|------------------|-------|-------|
| Max lift ESC     | 80%   | —     |
| Max thrust accel | 0.8   | g     |
| Max yaw rate     | 30    | °/s   |
| Heading tolerance| 5     | °     |

### Heading Hold PID (ASSISTED mode)

Target heading is frozen at the moment ASSISTED mode is engaged. The PID corrects yaw to hold that heading.

```
Kp = 0.80   (proportional — response speed)
Ki = 0.01   (integral — eliminates steady-state error)
Kd = 0.05   (derivative — damps overshoot)
Anti-windup: integral clamped to ±10
```

Tune in `src/config.h`:
```cpp
#define PID_KP  0.80f
#define PID_KI  0.01f
#define PID_KD  0.05f
```

---

## Hardware-In-the-Loop (HIL)

HIL mode connects the ESP32 to the [OpenSim](../OpenSim) browser simulator over WiFi. The ESP32 reads simulated sensor state instead of real IMU/barometer data, and sends control outputs back to the sim instead of driving the PCA9685.

```
RC Transmitter
      │ CRSF
      ▼
   ESP32 (HIL mode)
      │ WebSocket (WiFi)
      ▼
   hub.js (Node.js relay)
      │ WebSocket
      ▼
   OpenSim (browser)
      │ physics simulation
      ▼
   Corsair F4U renders on screen
```

The full voting logic, envelope protection, and heading hold all run on real hardware against simulated sensor data. This is functionally equivalent to an iron-bird rig.

### HIL Configuration

In `src/config.h`:

```cpp
#define HIL_WIFI_SSID    "your-network"
#define HIL_WIFI_PASS    "your-password"
#define HIL_HUB_HOST     "192.168.1.x"   // machine running hub.js
#define HIL_HUB_PORT     3000
#define HIL_ROOM         "hil-corsair"
```

### HIL WebSocket Protocol

**Sim → ESP32** (STATE_PATCH, 50Hz):
```json
{ "type": "STATE_PATCH", "room": "hil-corsair",
  "patch": { "hdg": 270.5, "pitch": 1.2, "roll": 0.8,
             "alt": 1500, "vs": 100, "spd": 180 } }
```

**ESP32 → Sim** (STATE_PATCH, on each control update):
```json
{ "type": "STATE_PATCH", "room": "hil-corsair",
  "patch": { "rollT": 15.0, "pitchT": 5.0, "spdT": 180.0, "ap": false } }
```

### HIL Envelope Limits (Corsair F4U)

| Parameter     | Limit  | Unit   |
|---------------|--------|--------|
| Max bank      | 90     | °      |
| Max pitch     | 30     | °      |
| Max speed     | 362    | knots  |

---

## LED Status

| Color           | Meaning                                  |
|-----------------|------------------------------------------|
| White           | Boot / initializing                      |
| Blue blinking   | Searching for other nodes on CAN/RS485   |
| Green           | NORMAL LAW — all 3 nodes healthy         |
| Yellow          | DEGRADED — one node failed               |
| Red             | DIRECT LAW — two nodes failed            |
| Blue solid      | AUTONOMOUS mode active                   |
| Magenta blink   | DISARM — critical failure                |

---

## SD Card Logging (Node 1 / Master)

Logs to `/hoverlog.csv` every 100ms:

```
timestamp, heading_1, heading_2, heading_3, voted_heading,
heading_error, lift_throttle, thrust_throttle, yaw_rate,
altitude, vspeed, envelope_mode,
node1_health, node2_health, node3_health
```

---

## Software Setup

### Requirements

- [PlatformIO](https://platformio.org) CLI or IDE extension
- Python 3.x
- USB driver for ESP32 (CP210x or CH340 depending on board revision)

### Dependencies (installed automatically by PlatformIO)

- `adafruit/Adafruit BNO055`
- `adafruit/Adafruit Unified Sensor`
- `adafruit/Adafruit PWM Servo Driver Library`
- `adafruit/Adafruit BMP3XX Library`
- `fastled/FastLED`
- `links2004/WebSockets`

### Build Environments

| Environment    | Use                                          |
|----------------|----------------------------------------------|
| `node1`        | Node 1 production firmware (master)          |
| `node2`        | Node 2 production firmware                   |
| `node3`        | Node 3 production firmware                   |
| `node1_solo`   | Node 1 standalone — no CAN required          |
| `node1_hil`    | Node 1 + HIL via OpenSim WebSocket           |
| `node1_test`   | Node 1 ground test suite (interactive menu)  |
| `node2_test`   | Node 2 ground test suite                     |
| `node3_test`   | Node 3 ground test suite                     |
| `servo_test`   | Minimal PCA9685 sweep test                   |
| `crsf_dump`    | Raw CRSF frame decoder — receiver diagnostics|

### Flash

```bash
# Production
pio run -e node1 -t upload
pio run -e node2 -t upload
pio run -e node3 -t upload

# HIL (set WiFi credentials in config.h first)
pio run -e node1_hil -t upload

# CRSF diagnostics
pio run -e crsf_dump -t upload

# Monitor
pio device monitor -b 115200
```

---

## File Structure

```
src/
├── main.cpp            — Setup, main loop (non-blocking, millis-based)
├── config.h            — All pins, constants, vehicle type, HIL config
├── can_bus.cpp/h       — CAN send/receive, VotePacket encoding
├── rs485.cpp/h         — RS485 fallback, time-division multiplexing
├── comms.cpp/h         — Transport abstraction (CAN primary, RS485 fallback)
├── bno055_imu.cpp/h    — BNO055 wrapper (heading, pitch, roll, yaw rate, accel)
├── bmp390.cpp/h        — BMP390 barometer (altitude, vertical speed)
├── voting.cpp/h        — 3-node voting logic, master election, envelope mode
├── envelope.cpp/h      — Envelope protection, PID heading hold
├── rc_input.cpp/h      — CRSF parser, 16 channels, failsafe
├── servo_output.cpp/h  — PCA9685 PWM driver, vehicle-type-configurable
├── hil.cpp/h           — HIL WebSocket client (WiFi → hub → OpenSim)
├── sd_logger.cpp/h     — SD card CSV logging
├── telemetry.cpp/h     — WiFi WebSocket telemetry output
├── led_status.cpp/h    — WS2812 status LED
├── ground_test.cpp     — Interactive ground verification suite
├── servo_test.cpp      — Minimal PCA9685 sweep test
└── crsf_dump.cpp       — CRSF frame decoder for receiver diagnostics
```

---

## Troubleshooting

**BNO055 not found:**
- I2C address: 0x28 (ADR pin to GND) or 0x29 (ADR to VCC)
- Check SDA=GPIO32, SCL=GPIO33, pull-ups present (4.7kΩ to 3.3V)
- Run ground test suite (`node1_test`) for interactive I2C scan

**CAN bus down:**
- CAN silent pin (GPIO23) must be LOW for active mode
- 120Ω termination at both ends of the bus
- All nodes share common GND
- RS485 fallback activates automatically — check `[COMMS]` serial output

**RC shows FAIL:**
- Check receiver LED: blue = link up, other states vary by firmware
- Verify CRSF wiring: receiver TX → GPIO25, receiver RX → GPIO5
- Flash `crsf_dump` to see raw frames and link statistics
- GPIO34/35 are input-only — cannot be used for CRSF RX reliably on this board

**Receiver stuck in WiFi mode:**
- Single clean power cycle (off, wait 5s, on once) to exit WiFi mode
- Do not triple-cycle — that re-enters WiFi mode on ELRS receivers
- Access receiver web UI via its IP on the local network to clear WiFi credentials

**Servos not moving:**
- PCA9685 V+ rail powered? (servos need 5–6V, separate from logic)
- OE pin on PCA9685 connected to GND?
- Common GND between power supply and ESP32?
- Run `servo_test` environment for isolated PCA9685 sweep

**HIL: Corsair not responding to sticks:**
- Check serial for `ws:OK` — WebSocket must be connected
- Open OpenSim with `?hil` parameter: `http://localhost:8080?mission=...&hil`
- Check browser console for `[HIL] rx:` messages confirming controls received
- Verify hub.js is running and both SIM and HIL clients are in the same room

---

## License

MIT License — see [LICENSE](LICENSE).
