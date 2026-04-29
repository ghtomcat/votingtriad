#pragma once
// ===========================================================
// rs485.h — RS485 Fallback-Kommunikation
// Aktiviert automatisch wenn CAN Bus ausfällt
// Hardware: UART2 auf IO21/IO22, Enable-Pin IO17
//
// Protokoll (19 Byte pro Frame):
//  [0]     0xAA         — Startbyte
//  [1]     node_id      — Absender (1-3)
//  [2-3]   heading ×10  — uint16_t big-endian
//  [4-5]   pitch ×10    — int16_t  big-endian
//  [6-7]   roll ×10     — int16_t  big-endian
//  [8-9]   hdg_error ×10 — int16_t big-endian
//  [10-11] yaw_rate ×10 — int16_t  big-endian
//  [12-13] accel_x ×100 — int16_t  big-endian
//  [14]    health       — uint8_t
//  [15-16] timestamp/10 — uint16_t big-endian
//  [17]    CRC8         — über Bytes [1..16]
//  [18]    0x55         — Endbyte
//
// Zeitmultiplex (halbduplex, 60ms Zyklus):
//  Node 1: sendet in Slot 0-20ms
//  Node 2: sendet in Slot 20-40ms
//  Node 3: sendet in Slot 40-60ms
// ===========================================================
#include <Arduino.h>
#include "can_bus.h"   // für VotePacket

#define RS485_FRAME_LEN     19
#define RS485_START_BYTE  0xAA
#define RS485_END_BYTE    0x55
#define RS485_BAUD       115200
#define RS485_CYCLE_MS       60   // Gesamtzyklus: 3 × 20ms Slots
#define RS485_SLOT_MS        20   // Länge eines Sende-Slots

// Initialisiert UART2 und EN-Pin
void rs485Init();

// Sendet VotePacket wenn eigener Slot aktiv ist (non-blocking)
// Gibt true zurück wenn in diesem Aufruf gesendet wurde
bool rs485Send(const VotePacket& pkt);

// Empfängt und parsed eingehende Frames (non-blocking State Machine)
// Aktualisiert interne Paket-Tabelle bei erfolgreichem Frame
void rs485Receive();

// Gibt zuletzt empfangenes Paket von Node id zurück
bool rs485GetNodePacket(uint8_t node_id, VotePacket& out);

// Millisekunden seit letztem RS485-Empfang von Node id
uint32_t rs485GetLastRxAge(uint8_t node_id);

// Statistiken für Debug
uint32_t rs485GetRxOK();    // erfolgreich geparste Frames
uint32_t rs485GetRxErr();   // CRC-Fehler / ungültige Frames
