// Run with -D MYCILA_JSON_SUPPORT
//
// if not running any flash operation, you could run with:
//
// -D CONFIG_ARDUINO_ISR_IRAM=1
// -D CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=1
// -D CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=1
// -D CONFIG_GPTIMER_ISR_IRAM_SAFE=1
// -D CONFIG_GPIO_CTRL_FUNC_IN_IRAM=1
//
// Otherwise, no. See:
// https://github.com/espressif/arduino-esp32/pull/4684

#include <ArduinoJson.h>
#include <MycilaPulseAnalyzer.h>

#include <Preferences.h>

#define PIN_OUTPUT      26
#define OUTPUT_WIDTH_US 1

Mycila::PulseAnalyzer pulseAnalyzer;

static uint32_t edgeCount = 0;
static void ARDUINO_ISR_ATTR onEdge(Mycila::PulseAnalyzer::Event e, void* arg) {
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING) {
    edgeCount++;
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(OUTPUT_WIDTH_US);
    digitalWrite(PIN_OUTPUT, LOW);
  }
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
    edgeCount++;
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(OUTPUT_WIDTH_US);
    digitalWrite(PIN_OUTPUT, LOW);
  }
}

static uint32_t zeroCrossCount = 0;
static void ARDUINO_ISR_ATTR onZeroCross(void* arg) {
  zeroCrossCount++;
  digitalWrite(PIN_OUTPUT, HIGH);
  delayMicroseconds(OUTPUT_WIDTH_US);
  digitalWrite(PIN_OUTPUT, LOW);
}

#if CONFIG_ARDUINO_ISR_IRAM != 1
static void flash_operation(void* arg) {
  uint64_t crashme = 0;
  while (true) {
    Preferences preferences;
    preferences.begin("crashme", false);
    preferences.putULong64("crashme", crashme);
    delay(5);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

  pulseAnalyzer.onEdge(onEdge);
  pulseAnalyzer.onZeroCross(onZeroCross);
  pulseAnalyzer.begin(35);

#if CONFIG_ARDUINO_ISR_IRAM != 1
  xTaskCreate(flash_operation, "flash_op", 4096, NULL, uxTaskPriorityGet(NULL), NULL);
#endif
}

uint32_t lastTime = 0;
void loop() {
  if (millis() - lastTime > 1000) {
    lastTime = millis();

    JsonDocument doc;
    pulseAnalyzer.toJson(doc.to<JsonObject>());

    Serial.print(edgeCount / 2 - zeroCrossCount);
    Serial.print(' ');
    serializeJson(doc, Serial);
    Serial.println();
  }
}
