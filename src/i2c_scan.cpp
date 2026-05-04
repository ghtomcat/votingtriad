// i2c_scan.cpp — I2C bus scanner
// Flash: pio run -e i2c_scan -t upload

#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA 32
#define PIN_SCL 33

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== I2C Scanner ===");
  Wire.begin(PIN_SDA, PIN_SCL);
}

void loop() {
  Serial.println("Scanning...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Device found at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  No devices found.");
  Serial.println();
  delay(3000);
}
