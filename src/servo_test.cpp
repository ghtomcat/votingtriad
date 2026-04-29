#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(32, 33);

  // I2C scan
  Serial.println("I2C scan:");
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found: 0x%02X\n", addr);
      found = true;
    }
  }
  if (!found) Serial.println("  Nothing found.");

  pca.begin();
  pca.setPWMFreq(50);
  Serial.println("Sweeping CH0...");
}

void loop() {
  Serial.println("MIN  (1000us)");
  pca.setPWM(0, 0, 205); delay(1000);
  Serial.println("MID  (1500us)");
  pca.setPWM(0, 0, 307); delay(1000);
  Serial.println("MAX  (2000us)");
  pca.setPWM(0, 0, 410); delay(1000);
  Serial.println("MID  (1500us)");
  pca.setPWM(0, 0, 307); delay(1000);
}
