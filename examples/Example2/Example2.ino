#include <MycilaPulseAnalyzer.h>

Mycila::PulseAnalyzer pulseAnalyzer;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  // GPIO 35: ZCD input
  // GPIO 26: Output
  // Connect both pins to an oscilloscope to see the output pulse
  pulseAnalyzer.simulate(35, 26, 1);
}

void loop() {
  delay(10);
}
