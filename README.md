# Hovercraft Voting Triad — Firmware v1.0

Airbus-Style Envelope Protection für Timos Hovercraft.  
Drei unabhängige Nodes stimmen via CAN Bus über den Systemzustand ab.

---

## Systemübersicht

```
┌─────────────────────────────────────────────────────────┐
│                    CAN Bus (500 kbps)                   │
│                                                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │  Node 1  │◄──►│  Node 2  │◄──►│  Node 3  │          │
│  │  MASTER  │    │  VOTER   │    │  VOTER   │          │
│  │          │    │          │    │          │          │
│  │ BNO055   │    │ BNO055   │    │ BNO055   │          │
│  │ RC Input │    │          │    │          │          │
│  │ Servo    │    │          │    │          │          │
│  │ SD Log   │    │          │    │          │          │
│  │ WiFi     │    │          │    │          │          │
│  └──────────┘    └──────────┘    └──────────┘          │
└─────────────────────────────────────────────────────────┘
```

**Voting-Logik:**
- 3/3 Nodes einig → **NORMAL LAW** (volle Schutzfunktionen)
- 2/3 Nodes einig → **DEGRADED** (reduzierte Limits)
- 1/3 Nodes einig → **DIRECT LAW** (nur Direktsteuerung)
- 0/3 → **DISARM** (alle Outputs gesperrt)

---

## Hardware: LilyGo T-CAN485

### Pin-Belegung (alle drei Nodes identisch)

| Funktion        | GPIO | Hinweis                        |
|-----------------|------|--------------------------------|
| CAN RX          | 26   | Zu CAN-Transceiver RXD         |
| CAN TX          | 27   | Zu CAN-Transceiver TXD         |
| CAN Silent      | 23   | LOW = aktiv, HIGH = stumm       |
| I2C SDA (BNO055)| 32   | 4.7kΩ Pull-Up zu 3.3V          |
| I2C SCL (BNO055)| 33   | 4.7kΩ Pull-Up zu 3.3V          |
| WS2812 LED      |  4   | 470Ω Serienwiderstand empfohlen |
| SD MOSI         | 15   | SPI                            |
| SD MISO         |  2   | SPI                            |
| SD SCLK         | 14   | SPI                            |
| SD CS           | 13   | SPI                            |

### Zusätzlich nur auf Node 1 (Master)

| Funktion        | GPIO | Hinweis                        |
|-----------------|------|--------------------------------|
| RC CH1 Rudder   | 34   | Input-Only, kein Pull-Up!      |
| RC CH2 Thrust   | 35   | Input-Only, kein Pull-Up!      |
| RC CH3 Lift     | 36   | Input-Only (VP Pin)            |
| RC CH4 Mode     | 39   | Input-Only (VN Pin)            |
| Rudder Servo    | 25   | PWM 50Hz                       |
| Thrust ESC      | 18   | PWM 50Hz                       |
| Lift ESC        | 19   | PWM 50Hz                       |

> **⚠️ Wichtig:** GPIO32/33 werden für I2C (BNO055) verwendet.  
> Die Servo-Ausgänge nutzen daher GPIO18/19 statt der ursprünglich  
> vorgesehenen GPIO32/33. Bitte entsprechend verdrahten!

---

## BNO055 Verdrahtung (DFRobot SEN0374)

```
BNO055       LilyGo T-CAN485
─────────────────────────────
VCC    ──►  3.3V
GND    ──►  GND
SDA    ──►  GPIO32  (mit 4.7kΩ Pull-Up zu 3.3V)
SCL    ──►  GPIO33  (mit 4.7kΩ Pull-Up zu 3.3V)
ADR    ──►  GND     (I2C-Adresse 0x28)
```

---

## CAN Bus Verdrahtung

Alle drei Nodes werden parallel am CAN Bus angeschlossen:

```
Node 1 CAN H ──┬── Node 2 CAN H ──┬── Node 3 CAN H
               │                  │
Node 1 CAN L ──┴── Node 2 CAN L ──┴── Node 3 CAN L
                                                   │
                                          120Ω Abschlusswiderstand
                                          (an beiden Enden des Bus!)
```

**CAN Kabelempfehlung:** Twisted Pair, max. 20m bei 500 kbps.

---

## Software-Setup (PlatformIO)

### Voraussetzungen
- PlatformIO IDE oder CLI
- Python 3.x
- USB-Treiber für ESP32 (CP210x oder CH340)

### Installation

```bash
# Repository klonen
git clone https://github.com/ghtomcat/hovercraft-triad
cd hovercraft-triad

# Abhängigkeiten werden automatisch installiert
```

### Flashen

```bash
# Node 1 (Master) flashen
pio run -e node1 -t upload

# Node 2 flashen
pio run -e node2 -t upload

# Node 3 flashen
pio run -e node3 -t upload

# Serial Monitor
pio device monitor -b 115200
```

---

## WiFi Konfiguration

In `src/config.h` anpassen:

```cpp
#define WIFI_SSID     "HovercraftAP"   // Name des Access Points
#define WIFI_PASSWORD "hovercraft123"  // Passwort
#define WIFI_AP_MODE  true             // true = eigener AP, false = Router
```

**Telemetrie-URL:** `ws://192.168.4.1:8080` (im AP-Modus)

### JSON-Format (alle 100ms):
```json
{
  "heading": 270.5,
  "pitch": 1.2,
  "roll": 0.8,
  "heading_error": -3.2,
  "lift_throttle": 0.72,
  "thrust_throttle": 0.45,
  "yaw_rate": 2.1,
  "envelope_mode": "NORMAL",
  "nodes": [
    {"id": 1, "health": "OK", "heading": 270.5},
    {"id": 2, "health": "OK", "heading": 271.0},
    {"id": 3, "health": "OK", "heading": 270.2}
  ]
}
```

---

## RC-Steuerung

| Kanal | Funktion      | 1000µs    | 1500µs  | 2000µs     |
|-------|---------------|-----------|---------|------------|
| CH1   | Rudder        | Voll links| Mitte   | Voll rechts|
| CH2   | Thrust ESC    | AUS       | 50%     | Vollgas    |
| CH3   | Lift ESC      | AUS       | 50%     | 80% max    |
| CH4   | Mode Switch   | Manual    | Assisted| Autonomous |

### Betriebsmodi (CH4)

- **MANUAL** (<1300µs): Direkte Steuerung, Envelope aktiv
- **ASSISTED** (1300–1700µs): Heading Hold via PID
  - Ohne Stick-Input: hält das Heading beim Modus-Eintritt
  - Mit Stick-Input: direkte Steuerung, Heading wird aktualisiert
- **AUTONOMOUS** (>1700µs): Zukünftig für GPS-Navigation

---

## LED Status

| Farbe        | Bedeutung                              |
|--------------|----------------------------------------|
| WEISS        | Boot / Initialisierung                 |
| BLAU blinkt  | Suche CAN-Verbindung                   |
| GRÜN         | NORMAL LAW — alle 3 Nodes OK           |
| GELB         | DEGRADED — ein Node ausgefallen        |
| ROT          | DIRECT LAW — zwei Nodes ausgefallen    |
| BLAU         | AUTONOMOUS MODE aktiv                  |
| MAGENTA blinkt | DISARM — kritischer Fehler           |

---

## SD-Karten Logging

CSV-Datei `/hoverlog.csv` auf der SD-Karte:

```
timestamp,heading_1,heading_2,heading_3,voted_heading,heading_error,
lift_throttle,thrust_throttle,yaw_rate,envelope_mode,
node1_health,node2_health,node3_health
```

Flush alle 5 Sekunden. SD-Karte kann während Betrieb entnommen  
werden — Datenverlust max. 5s.

---

## Node Failure & Promotion

**Node 1 fällt aus:**
- Node 2 erkennt Timeout nach 500ms
- Node 2 übernimmt Master-Rolle
- LED auf Node 2 wechselt zu GELB
- RC-Input und Servo-Ausgänge auf Node 2 aktiviert

**Node 1 kommt zurück:**
- Voting erkennt Node 1 als gesunden Node
- Node 1 übernimmt automatisch wieder die Master-Rolle
- System wechselt zurück zu NORMAL LAW

> **Hinweis:** Bei Master-Promotion übernimmt Node 2 die Steuerung.  
> RC-Empfänger und Servos müssen an **allen** Nodes angeschlossen sein,  
> oder alternativ über einen RC-Bus-Multiplexer umgeschaltet werden.

---

## Envelope Protection Limits

| Parameter        | Wert  | Einheit |
|------------------|-------|---------|
| Max Lift ESC     | 80%   | %       |
| Max Thrust Accel | 0.8   | g       |
| Max Yaw Rate     | 30    | °/s     |
| Heading Toleranz | 5     | °       |

---

## PID Heading Hold

```
Kp = 0.80   — Proportional (Reaktionsschnelligkeit)
Ki = 0.01   — Integral (eliminiert bleibende Abweichung)
Kd = 0.05   — Differential (dämpft Überschwingen)
Anti-Windup: Integral limitiert auf ±10
```

Anpassung in `src/config.h`:
```cpp
#define PID_KP  0.80f
#define PID_KI  0.01f
#define PID_KD  0.05f
```

---

## Dateistruktur

```
src/
├── main.cpp          — Setup, Loop (non-blocking)
├── config.h          — Alle Konstanten und Pins
├── can_bus.cpp/h     — CAN Send/Receive, VotePacket
├── bno055_imu.cpp/h  — IMU Wrapper (Heading, Pitch, Roll)
├── voting.cpp/h      — Voting-Logik, Envelope-Mode
├── envelope.cpp/h    — Envelope Protection, PID
├── rc_input.cpp/h    — PWM RC Eingang (Interrupt)
├── servo_output.cpp/h — PWM Servo/ESC Ausgang (LEDC)
├── sd_logger.cpp/h   — SD-Karten CSV-Logging
├── telemetry.cpp/h   — WiFi WebSocket Telemetrie
└── led_status.cpp/h  — WS2812 Status-LED
```

---

## Troubleshooting

**BNO055 nicht gefunden:**
- I2C-Adresse prüfen: Standard = 0x28 (ADR Pin auf GND)
- Pull-Up-Widerstände vorhanden? (4.7kΩ an SDA + SCL)
- SDA = GPIO32, SCL = GPIO33 korrekt verbunden?

**Kein CAN-Signal:**
- CAN-Silent Pin (GPIO23) auf LOW?
- 120Ω Abschlusswiderstände an beiden Bus-Enden?
- Alle Nodes mit gleichem GND verbunden?
- Alle Nodes mit 500 kbps konfiguriert?

**ESCs reagieren nicht:**
- ESCs brauchen Kalibrierung? (1000µs Signal für 2s beim Einschalten)
- GPIO18/19 korrekt verbunden (nicht GPIO32/33)?
- `servoInit()` aufgerufen?

**LED bleibt weiß:**
- BNO055 oder CAN Init schlägt fehl → `Serial Monitor` öffnen für Debug-Ausgabe

---

## Lizenz

MIT License — frei verwendbar für Timos Hovercraft-Projekt.
