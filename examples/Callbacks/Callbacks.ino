// Run with -D MYCILA_JSON_SUPPORT
//
// If not running any flash operation, you could run with:
//
// -D CONFIG_ARDUINO_ISR_IRAM=1
// -D CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=1
// -D CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=1
// -D CONFIG_GPTIMER_ISR_IRAM_SAFE=1
// -D CONFIG_GPIO_CTRL_FUNC_IN_IRAM=1
//
// Otherwise, no. See:
// https://github.com/espressif/arduino-esp32/pull/4684
//
// To shift the the ZC event, use: -D MYCILA_PULSE_ZC_SHIFT_US=100
//
#include <MycilaPulseAnalyzer.h>

#include <esp32-hal-gpio.h>
#include <esp32-hal.h>

#include <Preferences.h>

#define PIN_OUTPUT      gpio_num_t::GPIO_NUM_26
#define OUTPUT_WIDTH_US 1

Mycila::PulseAnalyzer pulseAnalyzer;

// outputs a 1 us pulse when an edge is detected
static volatile uint32_t edgeCount = 0;
static void ARDUINO_ISR_ATTR onEdge(Mycila::PulseAnalyzer::Event e, void* arg) {
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING) {
    edgeCount = edgeCount + 1;
    ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, HIGH));
    delayMicroseconds(OUTPUT_WIDTH_US);
    ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, LOW));
  }
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
    edgeCount = edgeCount + 1;
    ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, HIGH));
    delayMicroseconds(OUTPUT_WIDTH_US);
    ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, LOW));
  }
}

// outputs a 1 us pulse when the ZC event is sent
static volatile uint32_t zeroCrossCount = 0;
static void ARDUINO_ISR_ATTR onZeroCross(void* arg) {
  zeroCrossCount = zeroCrossCount + 1;
  ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, HIGH));
  delayMicroseconds(OUTPUT_WIDTH_US);
  ESP_ERROR_CHECK(gpio_set_level(PIN_OUTPUT, LOW));
}

static void flash_operation(void* arg) {
  uint64_t crashme = 0;
  while (true) {
    Preferences preferences;
    preferences.begin("crashme", false);
    preferences.putULong64("crashme", crashme);
    delay(5);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

  pulseAnalyzer.onEdge(onEdge);
  pulseAnalyzer.onZeroCross(onZeroCross);
  pulseAnalyzer.begin(35);

  // Simulate some flash operations at the same time.
  // If the code is put in IRAM, it will crash
  // (-D CONFIG_ARDUINO_ISR_IRAM=1)
  xTaskCreate(flash_operation, "flash_op", 4096, NULL, uxTaskPriorityGet(NULL), NULL);
}

uint32_t lastTime = 0;
void loop() {
  if (millis() - lastTime > 1000) {
    lastTime = millis();

    Serial.printf("%" PRIu32 " F=%" PRIu32 " Hz P=%" PRIu32 " us ", edgeCount / 2 - zeroCrossCount, pulseAnalyzer.getNominalGridFrequency(), pulseAnalyzer.getNominalGridPeriod());

#ifdef MYCILA_JSON_SUPPORT
    JsonDocument doc;
    pulseAnalyzer.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
#endif

    Serial.println();
  }
}
