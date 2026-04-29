// ===========================================================
// sd_logger.cpp — SD card logging
// SPI pins: MOSI=15, MISO=2, SCLK=14, CS=13
// CSV header written on first run; flush every 5s
// ===========================================================
#include "sd_logger.h"
#include "config.h"
#include <SPI.h>
#include <SD.h>

static bool      _ready     = false;
static File      _logFile;
static uint32_t  _lastFlush = 0;
static uint32_t  _lineCount = 0;
static const uint32_t FLUSH_INTERVAL_MS = 5000;

static const char* CSV_HEADER =
  "timestamp,heading_1,heading_2,heading_3,voted_heading,"
  "heading_error,lift_throttle,thrust_throttle,yaw_rate,"
  "altitude_m,vspeed_ms,"
  "envelope_mode,node1_health,node2_health,node3_health\n";

bool sdInit() {
  // custom SPI pins for LilyGo T-CAN485
  SPI.begin(PIN_SD_SCLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("[SD] ERROR: SD card not found! Logging disabled.");
    return false;
  }

  // open log file in append mode or create new
  bool newFile = !SD.exists(SD_LOG_FILENAME);
  _logFile = SD.open(SD_LOG_FILENAME, FILE_APPEND);

  if (!_logFile) {
    Serial.println("[SD] ERROR: Could not open log file!");
    return false;
  }

  // write header only for new files
  if (newFile) {
    _logFile.print(CSV_HEADER);
    _logFile.flush();
  }

  _ready = true;
  uint64_t freeBytes = (uint64_t)(SD.totalBytes() - SD.usedBytes());
  Serial.printf("[SD] Logging active: %s (%.1f MB free)\n",
                SD_LOG_FILENAME, freeBytes / 1048576.0f);
  return true;
}

void sdLog(const LogData& d) {
  if (!_ready) return;

  const char* modeStr;
  switch (d.envelope_mode) {
    case 0: modeStr = "NORMAL";   break;
    case 1: modeStr = "DEGRADED"; break;
    case 2: modeStr = "DIRECT";   break;
    default: modeStr = "DISARM";  break;
  }

  auto healthStr = [](uint8_t h) -> const char* {
    switch (h) {
      case 0: return "OK";
      case 1: return "WARN";
      default: return "FAIL";
    }
  };

  _logFile.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.2f,%.3f,%.3f,%.2f,%.2f,%.3f,%s,%s,%s,%s\n",
    d.timestamp,
    d.heading_1, d.heading_2, d.heading_3,
    d.voted_heading, d.heading_error,
    d.lift_throttle, d.thrust_throttle, d.yaw_rate,
    d.altitude, d.vspeed,
    modeStr,
    healthStr(d.node1_health),
    healthStr(d.node2_health),
    healthStr(d.node3_health)
  );

  _lineCount++;

  // flush periodically to avoid data loss
  if (millis() - _lastFlush >= FLUSH_INTERVAL_MS) {
    _logFile.flush();
    _lastFlush = millis();
    Serial.printf("[SD] Flush: %lu lines saved.\n", _lineCount);
  }
}

bool sdIsReady() { return _ready; }
