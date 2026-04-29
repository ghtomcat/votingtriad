#pragma once
// ===========================================================
// hil.h — Hardware-In-the-Loop via OpenSim WebSocket hub
// ESP32 joins hub room, receives sim state, sends RC controls.
// Build with -D HIL_MODE to activate.
// ===========================================================
#include <Arduino.h>

void    hilInit();
void    hilUpdate();                          // call every loop tick

bool    hilIsConnected();                     // hub WebSocket up?
bool    hilHasState();                        // at least one sim state received?

// Sim state — use these instead of real IMU/baro in HIL mode
float   hilGetHeading();                      // degrees
float   hilGetPitch();                        // degrees
float   hilGetRoll();                         // degrees
float   hilGetAltFt();                        // feet
float   hilGetSpdKt();                        // knots
float   hilGetVsFpm();                        // ft/min

// Send RC-derived controls back to sim
void    hilSendControls(float rollT, float pitchT, float spdT);
