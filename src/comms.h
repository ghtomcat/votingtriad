#pragma once
// ===========================================================
// comms.h — Kommunikations-Abstraktion: CAN + RS485 Fallback
//
// Logik:
//  - Primär: CAN Bus (20ms Intervall, niedrige Latenz)
//  - Fallback: RS485 (60ms Zyklus, aktiviert wenn CAN ausfällt)
//
//  CAN gilt als ausgefallen wenn für ALLE anderen Nodes
//  kein CAN-Frame innerhalb von COMMS_CAN_DEAD_MS empfangen wurde.
//  (Einzelner toter Node ≠ CAN-Ausfall)
//
//  Der aktive Transport wird pro Node separat gewählt:
//  → CAN-Daten < RS485-Daten in Aktualität → CAN bevorzugt
//  → Bei CAN-Ausfall: RS485-Daten verwendet
//
//  Statusänderungen werden auf Serial geloggt.
// ===========================================================
#include <Arduino.h>
#include "can_bus.h"

// Nach dieser Zeit ohne JEGLICHEN CAN-Frame → CAN als tot markieren
#define COMMS_CAN_DEAD_MS   1000

// Mögliche Transporte (für Status-Abfrage)
enum Transport : uint8_t {
  TRANSPORT_NONE   = 0,
  TRANSPORT_CAN    = 1,
  TRANSPORT_RS485  = 2
};

// Initialisiert CAN und RS485
void commsInit();

// Sendet Paket über BEIDE Kanäle (redundant)
void commsSend(const VotePacket& pkt);

// Empfängt von beiden Kanälen (non-blocking)
void commsReceive();

// Bestes verfügbares Paket für node_id (CAN bevorzugt)
bool commsGetNodePacket(uint8_t node_id, VotePacket& out);

// Alter des besten verfügbaren Pakets für node_id
uint32_t commsGetLastRxAge(uint8_t node_id);

// Welcher Transport wird für node_id verwendet?
Transport commsGetTransport(uint8_t node_id);

// Ist CAN Bus insgesamt aktiv?
bool commsIsCANAlive();

// Ist RS485 Fallback aktiv?
bool commsIsRS485Active();

// Gibt Statusstring für Debug aus (z.B. Serial-Ausgabe)
void commsPrintStatus();
