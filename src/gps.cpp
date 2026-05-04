// ===========================================================
// gps.cpp — GNSS MAX-M10S NMEA parser
// Parses $GNGGA (fix, position, altitude) and $GNRMC (speed, track)
// No external library — minimal footprint
// ===========================================================
#include "gps.h"
#include "config.h"

#define GPS_BAUD      9600
#define GPS_RX_PIN    35
#define GPS_BUF_LEN   128

static char    _buf[GPS_BUF_LEN];
static uint8_t _bufIdx   = 0;

static bool    _hasFix   = false;
static float   _lat      = 0.0f;
static float   _lon      = 0.0f;
static float   _altMSL   = 0.0f;
static float   _speedKt  = 0.0f;
static float   _track    = 0.0f;
static uint8_t _sats     = 0;
static uint32_t _lastFix = 0;

// ---------------------------------------------------------------------------
// Parse a decimal NMEA coordinate field: DDDMM.MMMM → decimal degrees
static float nmeaToDecDeg(const char* field) {
  float raw = atof(field);
  int deg   = (int)(raw / 100);
  float min = raw - (float)(deg * 100);
  return (float)deg + min / 60.0f;
}

// ---------------------------------------------------------------------------
// Lightweight field extractor — returns pointer to the Nth comma-separated
// field within a NMEA sentence (0 = sentence type e.g. "$GNGGA")
static const char* nmeaField(const char* sentence, uint8_t n) {
  const char* p = sentence;
  for (uint8_t i = 0; i < n; i++) {
    p = strchr(p, ',');
    if (!p) return nullptr;
    p++;
  }
  return p;
}

static float fieldFloat(const char* sentence, uint8_t n) {
  const char* f = nmeaField(sentence, n);
  return f ? atof(f) : 0.0f;
}

static char fieldChar(const char* sentence, uint8_t n) {
  const char* f = nmeaField(sentence, n);
  return f ? f[0] : '\0';
}

// ---------------------------------------------------------------------------
// $GNGGA — fix data
// Fields: type,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,,
static void parseGGA(const char* s) {
  const char* fixField = nmeaField(s, 6);
  if (!fixField || fixField[0] == '0') return;  // no fix

  float lat = fieldFloat(s, 2);
  char  latH = fieldChar(s, 3);
  float lon = fieldFloat(s, 4);
  char  lonH = fieldChar(s, 5);
  float alt  = fieldFloat(s, 9);
  uint8_t sats = (uint8_t)fieldFloat(s, 7);

  _lat    = (latH == 'S') ? -nmeaToDecDeg(nmeaField(s, 2)) : nmeaToDecDeg(nmeaField(s, 2));
  _lon    = (lonH == 'W') ? -nmeaToDecDeg(nmeaField(s, 4)) : nmeaToDecDeg(nmeaField(s, 4));
  _altMSL = alt;
  _sats   = sats;
  _hasFix = true;
  _lastFix = millis();
}

// ---------------------------------------------------------------------------
// $GNRMC — recommended minimum
// Fields: type,time,status,lat,N/S,lon,E/W,speed(kt),track,date,...
static void parseRMC(const char* s) {
  char status = fieldChar(s, 2);
  if (status != 'A') return;  // not active

  _speedKt = fieldFloat(s, 7);
  _track   = fieldFloat(s, 8);
}

// ---------------------------------------------------------------------------
static uint8_t nmeaChecksum(const char* s) {
  uint8_t crc = 0;
  // checksum covers chars between $ and * exclusive
  for (const char* p = s + 1; *p && *p != '*'; p++)
    crc ^= (uint8_t)*p;
  return crc;
}

static bool validateChecksum(const char* s) {
  const char* star = strchr(s, '*');
  if (!star || strlen(star) < 3) return false;
  uint8_t received = (uint8_t)strtol(star + 1, nullptr, 16);
  return received == nmeaChecksum(s);
}

// ---------------------------------------------------------------------------
static void processLine(const char* s) {
  if (!validateChecksum(s)) return;

  if (strncmp(s, "$GNGGA", 6) == 0 || strncmp(s, "$GPGGA", 6) == 0)
    parseGGA(s);
  else if (strncmp(s, "$GNRMC", 6) == 0 || strncmp(s, "$GPRMC", 6) == 0)
    parseRMC(s);
}

// ---------------------------------------------------------------------------
void gpsInit() {
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
  Serial.printf("[GPS] GNSS MAX-M10S on GPIO%d (Serial2, %d baud)\n",
                GPS_RX_PIN, GPS_BAUD);
}

void gpsUpdate() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      if (_bufIdx > 0) {
        _buf[_bufIdx] = '\0';
        processLine(_buf);
        _bufIdx = 0;
      }
      continue;
    }
    if (_bufIdx < GPS_BUF_LEN - 1)
      _buf[_bufIdx++] = c;
    else
      _bufIdx = 0;  // overflow — discard
  }
}

bool    gpsHasFix()     { return _hasFix && (millis() - _lastFix) < 2000; }
float   gpsGetLat()     { return _lat;     }
float   gpsGetLon()     { return _lon;     }
float   gpsGetAltMSL()  { return _altMSL;  }
float   gpsGetSpeedKt() { return _speedKt; }
float   gpsGetTrack()   { return _track;   }
uint8_t gpsGetSats()    { return _sats;    }
uint32_t gpsGetAge()    { return _hasFix ? millis() - _lastFix : 0xFFFFFFFF; }
