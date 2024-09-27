// Run with:
//
// -D MYCILA_JSON_SUPPORT
// -D CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=1
// -D CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=1
// -D CONFIG_GPTIMER_ISR_IRAM_SAFE=1
// -D CONFIG_GPIO_CTRL_FUNC_IN_IRAM=1
//
// To shift the the ZC event, use: -D MYCILA_PULSE_ZC_SHIFT_US=100
//
#include <MycilaPulseAnalyzer.h>

#include <esp32-hal-gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

#include <esp32-hal.h>

#include <Preferences.h>

#ifdef CONFIG_IDF_TARGET_ESP32C3
  #define PIN_OUTPUT gpio_num_t::GPIO_NUM_21
#else
  #define PIN_OUTPUT gpio_num_t::GPIO_NUM_26
#endif

static void IRAM_ATTR wait1us() {
  uint64_t m = (uint64_t)esp_timer_get_time();
  uint64_t e = m + 1;
  if (m > e)
    while ((uint64_t)esp_timer_get_time() > e)
      NOP();
  while ((uint64_t)esp_timer_get_time() < e)
    NOP();
}

// outputs a 1 us pulse when an edge is detected
static volatile uint32_t edgeCount = 0;
static void IRAM_ATTR onEdge(Mycila::PulseAnalyzer::Event e, void* arg) {
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING) {
    edgeCount = edgeCount + 1;
    gpio_ll_set_level(&GPIO, PIN_OUTPUT, HIGH);
    wait1us();
    gpio_ll_set_level(&GPIO, PIN_OUTPUT, LOW);
  }
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
    edgeCount = edgeCount + 1;
    gpio_ll_set_level(&GPIO, PIN_OUTPUT, HIGH);
    wait1us();
    gpio_ll_set_level(&GPIO, PIN_OUTPUT, LOW);
  }
}

// outputs a 1 us pulse when the ZC event is sent
static volatile uint32_t zeroCrossCount = 0;
static void IRAM_ATTR onZeroCross(void* arg) {
  zeroCrossCount = zeroCrossCount + 1;
  gpio_ll_set_level(&GPIO, PIN_OUTPUT, HIGH);
  wait1us();
  gpio_ll_set_level(&GPIO, PIN_OUTPUT, LOW);
}

static void flash_operations(void* arg) {
  while (true) {
    Preferences preferences;
#if 1 // test with flash / nvm operations
    preferences.begin("crashme", false);
    preferences.putULong64("crashme", 0);
#endif
    delay(5);
  }
}

Mycila::PulseAnalyzer pulseAnalyzer;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

  pulseAnalyzer.onEdge(onEdge);
  pulseAnalyzer.onZeroCross(onZeroCross);
  pulseAnalyzer.begin(35);

  // Simulate some flash operations at the same time.
  xTaskCreate(flash_operations, "flash_op", 4096, NULL, uxTaskPriorityGet(NULL), NULL);
}

uint32_t lastTime = 0;
uint32_t lastEnd = 0;
void loop() {
  if (millis() - lastTime > 500) {

    Serial.printf("%" PRIu32 " F=%" PRIu32 " Hz P=%" PRIu32 " us ", edgeCount / 2 - zeroCrossCount, pulseAnalyzer.getNominalGridFrequency(), pulseAnalyzer.getNominalGridPeriod());

#ifdef MYCILA_JSON_SUPPORT
    JsonDocument doc;
    pulseAnalyzer.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
#endif

    Serial.println();
    lastTime = millis();
  }

  if (millis() - lastEnd > 15000) {
    Serial.println("end()");
    pulseAnalyzer.end();
    delay(3000);
    Serial.println("begin()");
    pulseAnalyzer.begin(35);

    lastEnd = millis();
    lastTime = millis();
  }
}
