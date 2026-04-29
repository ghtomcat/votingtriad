#pragma once
// ===========================================================
// led_status.h — WS2812 LED Status-Anzeige
// Farb-Kodierung:
//   WEISS  = Boot / Initialisierung
//   BLINKT = Suche CAN-Verbindung
//   GRÜN   = NORMAL LAW (alle 3 Nodes OK)
//   GELB   = DEGRADED (ein Node ausgefallen)
//   ROT    = DIRECT LAW (zwei Nodes ausgefallen)
//   BLAU   = AUTONOMOUS MODE
//   MAGENTA = DISARM / Fehler
// ===========================================================
#include <Arduino.h>
#include "voting.h"
#include "rc_input.h"

// Initialisiert FastLED
void ledInit();

// Aktualisiert LED basierend auf aktuellem Zustand
// Muss regelmäßig im Loop aufgerufen werden
void ledUpdate(EnvelopeMode mode, RCMode rcMode, bool canConnected);
