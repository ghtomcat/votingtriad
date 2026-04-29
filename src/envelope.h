#pragma once
// ===========================================================
// envelope.h — Envelope Protection & PID Heading Hold
// Begrenzt alle Ausgaben auf sichere Werte
// PID-Regler für Assisted Mode (Heading Hold)
// ===========================================================
#include <Arduino.h>
#include "voting.h"

// Struktur für aufbereitete Steuer-Outputs
struct ControlOutput {
  float rudder;         // -1.0 bis +1.0 (links/rechts)
  float thrust;         // 0.0 bis 1.0
  float lift;           // 0.0 bis MAX_LIFT_THROTTLE
  bool  armed;          // false = alle Outputs gesperrt
};

// Initialisiert PID-State
void envelopeInit();

// Wendet Envelope-Limits auf Raw-Inputs an
// rawRudder, rawThrust, rawLift: normierte Eingaben (0.0–1.0 bzw. ±1.0)
// mode: aktueller Envelope-Modus
// yawRate: aktuelle Gierrate (Grad/s)
// targetHeading: Soll-Heading (für Assisted Mode)
// currentHeading: Ist-Heading
// rcMode: 0=MANUAL, 1=ASSISTED, 2=AUTONOMOUS
// Gibt berechnete, geclamped ControlOutputs zurück
ControlOutput envelopeApply(
  float rawRudder,
  float rawThrust,
  float rawLift,
  EnvelopeMode mode,
  float yawRate,
  float targetHeading,
  float currentHeading,
  uint8_t rcMode
);

// PID-Regler zurücksetzen (z.B. bei Mode-Wechsel)
void envelopeResetPID();
