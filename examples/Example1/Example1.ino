#include <MycilaPulseAnalyzer.h>

Mycila::PulseAnalyzer<10> pulseAnalyzer;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;
}

void loop() {
  delay(5000);
}
