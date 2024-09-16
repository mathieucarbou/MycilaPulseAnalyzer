// Run with -D MYCILA_JSON_SUPPORT

#include <ArduinoJson.h>
#include <MycilaPulseAnalyzer.h>

#include <rom/ets_sys.h>

#define PIN_OUTPUT      26
#define OUTPUT_WIDTH_US 1

Mycila::PulseAnalyzer pulseAnalyzer;

static void ARDUINO_ISR_ATTR onEdge(Mycila::PulseAnalyzer::Event e, void* arg) {
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING || e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(OUTPUT_WIDTH_US);
    digitalWrite(PIN_OUTPUT, LOW);
  }

  // if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING) {
  //   digitalWrite(PIN_OUTPUT, HIGH);
  //   delayMicroseconds(OUTPUT_WIDTH_US);
  //   digitalWrite(PIN_OUTPUT, LOW);
  // }

  // if (e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
  //   digitalWrite(PIN_OUTPUT, HIGH);
  //   delayMicroseconds(OUTPUT_WIDTH_US);
  //   digitalWrite(PIN_OUTPUT, LOW);
  // }

  // if (e == Mycila::PulseAnalyzer::Event::SIGNAL_CHANGE) {
  //   digitalWrite(PIN_OUTPUT, HIGH);
  //   delayMicroseconds(OUTPUT_WIDTH_US);
  //   digitalWrite(PIN_OUTPUT, LOW);
  // }
}

static void ARDUINO_ISR_ATTR onZeroCross(void* arg) {
  digitalWrite(PIN_OUTPUT, HIGH);
  delayMicroseconds(OUTPUT_WIDTH_US);
  digitalWrite(PIN_OUTPUT, LOW);
}

static void ARDUINO_ISR_ATTR onOffline(void* arg) {
  ets_printf("offline\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

  pulseAnalyzer.onOffline(onOffline);
  pulseAnalyzer.onEdge(onEdge);
  pulseAnalyzer.onZeroCross(onZeroCross);

  pulseAnalyzer.begin(35);
}

uint32_t lastTime = 0;
void loop() {
  if (millis() - lastTime > 500) {
    lastTime = millis();
    JsonDocument doc;
    pulseAnalyzer.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
    Serial.println();
  }
}
