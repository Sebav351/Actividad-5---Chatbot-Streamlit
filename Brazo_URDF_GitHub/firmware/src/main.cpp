#include <Arduino.h>

namespace {
constexpr uint8_t sensorPins[] = {32, 33, 34, 35, 36};
constexpr float jointLower[] = {-2.5F, -2.0F, 0.0F, 0.0F, 0.0F};
constexpr float jointUpper[] = {2.5F, 2.0F, 0.15F, 0.05F, 0.05F};
constexpr uint8_t jointCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
constexpr uint32_t reportIntervalMs = 50;
constexpr uint16_t adcMaximum = 4095;

uint32_t lastReportMs = 0;

float readJointPosition(uint8_t index) {
  const int rawValue = analogRead(sensorPins[index]);
  const float normalized = constrain(rawValue, 0, adcMaximum) / static_cast<float>(adcMaximum);
  return jointLower[index] + normalized * (jointUpper[index] - jointLower[index]);
}

void reportJointPositions() {
  Serial.print("J");
  for (uint8_t index = 0; index < jointCount; ++index) {
    Serial.printf(",%.4f", readJointPosition(index));
  }
  Serial.println();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  for (const uint8_t pin : sensorPins) {
    analogSetPinAttenuation(pin, ADC_11db);
  }
  Serial.println("READY,brazo_urdf,115200");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastReportMs >= reportIntervalMs) {
    lastReportMs = now;
    reportJointPositions();
  }
}
