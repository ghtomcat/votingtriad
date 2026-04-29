#pragma once
// ===========================================================
// voting.h — Voting-Logik und Envelope-Mode-Bestimmung
// Bestimmt Konsens aus 3 Nodes, erkennt Node-Ausfälle,
// managed Master-Promotion
// ===========================================================
#include <Arduino.h>
#include "can_bus.h"

// Betriebsmodi (Airbus-Style Envelope Modes)
enum EnvelopeMode : uint8_t {
  MODE_NORMAL   = 0,  // 3/3 Nodes einig → volle Schutzfunktionen
  MODE_DEGRADED = 1,  // 2/3 Nodes einig → reduzierter Schutz
  MODE_DIRECT   = 2,  // 1/3 Nodes einig → nur Direktsteuerung
  MODE_DISARM   = 3   // 0/3 → alle Ausgänge gesperrt
};

struct NodeStatus {
  bool       active;
  NodeHealth health;
  float      heading;       // degrees
  float      altitude;      // metres AGL
  uint32_t   lastRxAge_ms;
};

// Voting-Logik initialisieren (setzt lokalen Node als Referenz)
void votingInit(uint8_t myNodeId);

// Hauptfunktion: muss im Loop aufgerufen werden
// Aktualisiert Konsens, Modus und Master-Status
void votingUpdate(const VotePacket& myPacket);

// Aktueller Envelope-Modus
EnvelopeMode votingGetMode();

// Consensus heading — average of agreeing nodes (NORMAL/DEGRADED only)
float votingGetConsensusHeading();

// Consensus altitude — average of agreeing nodes
float votingGetConsensusAltitude();

// Anzahl aktuell gesunder Nodes
uint8_t votingGetHealthyCount();

// Gibt true zurück wenn dieser Node gerade aktiver Master ist
// (Node 1 standard, Node 2/3 nach Promotion)
bool votingIsMaster();

// Status eines bestimmten Nodes abfragen (1-3)
NodeStatus votingGetNodeStatus(uint8_t node_id);

// Gibt den human-readable Mode-Namen zurück (für Serial/Log)
const char* votingModeName(EnvelopeMode mode);
