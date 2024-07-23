#include <ArduinoJson.h>
#include <MycilaPulseAnalyzer.h>

Mycila::PulseAnalyzer pulseAnalyzer;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;
}

void loop() {
  // Serial.println("Recording pulse on pin 35");
  pulseAnalyzer.record(35);

  while (pulseAnalyzer.getState() != Mycila::PulseAnalyzer::State::RECORDED)
    delay(100);
  // Serial.println("Pulses recorded");

  // Serial.println("Analyzing pulses");
  pulseAnalyzer.analyze();

  while (pulseAnalyzer.getState() != Mycila::PulseAnalyzer::State::ANALYZED)
    delay(100);
  // Serial.println("Pulses analyzed");

  pulseAnalyzer.end();

  JsonDocument doc;
  pulseAnalyzer.toJson(doc.to<JsonObject>());
  serializeJson(doc, Serial);
  Serial.println();

  delay(2000);
}
