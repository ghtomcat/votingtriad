// ===========================================================
// led_status.cpp — WS2812 LED via FastLED
// One LED on GPIO4, shows system state
// Blink logic is non-blocking (millis())
// ===========================================================
#include "led_status.h"
#include "config.h"
#include <FastLED.h>

static CRGB     _leds[NUM_LEDS];
static uint32_t _lastBlink  = 0;
static bool     _blinkState = false;

void ledInit() {
  FastLED.addLeds<WS2812, PIN_LED, GRB>(_leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  // boot color: WHITE
  _leds[0] = CRGB::White;
  FastLED.show();
  Serial.println("[LED] WS2812 initialized — WHITE (boot).");
}

void ledUpdate(EnvelopeMode mode, RCMode rcMode, bool canConnected) {
  uint32_t now = millis();

  // searching for CAN: slow blue blink
  if (!canConnected) {
    if (now - _lastBlink >= LED_BLINK_MS) {
      _lastBlink  = now;
      _blinkState = !_blinkState;
      _leds[0] = _blinkState ? CRGB::Blue : CRGB::Black;
      FastLED.show();
    }
    return;
  }

  // autonomous mode: solid blue
  if (rcMode == RC_AUTONOMOUS) {
    _leds[0] = CRGB::Blue;
    FastLED.show();
    return;
  }

  // color based on envelope mode
  CRGB color;
  switch (mode) {
    case MODE_NORMAL:
      color = CRGB::Green;    // all OK
      break;

    case MODE_DEGRADED:
      color = CRGB::Yellow;   // one node down
      break;

    case MODE_DIRECT:
      color = CRGB::Red;      // two nodes down
      break;

    case MODE_DISARM:
    default:
      // DISARM: fast magenta blink — critical warning
      if (now - _lastBlink >= (LED_BLINK_MS / 2)) {
        _lastBlink  = now;
        _blinkState = !_blinkState;
        _leds[0] = _blinkState ? CRGB::Magenta : CRGB::Black;
        FastLED.show();
      }
      return;
  }

  _leds[0] = color;
  FastLED.show();
}
