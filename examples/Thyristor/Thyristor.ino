#include <MycilaPulseAnalyzer.h>

#include <esp32-hal-gpio.h>
#include <esp32-hal.h>

#include <Preferences.h>

#ifdef CONFIG_IDF_TARGET_ESP32C3
  #define PIN_THYRISTOR gpio_num_t::GPIO_NUM_21
#else
  #define PIN_THYRISTOR gpio_num_t::GPIO_NUM_26
#endif

// Minimum delay to reach the voltage required for a gate current of 30mA.
// delay_us = asin((gate_resistor * gate_current) / grid_volt_max) / pi * period_us
// delay_us = asin((330 * 0.03) / 325) / pi * 10000 = 97us
#define PHASE_DELAY_MIN_US 100

Mycila::PulseAnalyzer pulseAnalyzer;
hw_timer_t* thyristorTimer;

volatile uint32_t firingDelay = UINT32_MAX;
volatile uint32_t semiPeriod = 0;

static void IRAM_ATTR onZeroCross(void* arg) {
  // reset thyristor timer to start counting from this ZC event
  timerRestart(thyristorTimer);

  // make sure thyristor is stoped at ZC point
  gpio_set_level(PIN_THYRISTOR, LOW);

  // 100% duty cycle
  if (firingDelay < PHASE_DELAY_MIN_US)
    firingDelay = PHASE_DELAY_MIN_US;

  if (firingDelay < semiPeriod)
    timerAlarm(thyristorTimer, firingDelay, false, 0);
}

static void IRAM_ATTR onThyristorTimer() {
  gpio_set_level(PIN_THYRISTOR, HIGH);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_THYRISTOR, OUTPUT);

  thyristorTimer = timerBegin(1000000);
  timerAttachInterrupt(thyristorTimer, onThyristorTimer);
  timerStop(thyristorTimer);

  pulseAnalyzer.onZeroCross(onZeroCross);
  pulseAnalyzer.begin(35);
}

uint32_t lastTime = 0;
size_t i = 0;
float dutyCycles[5] = {0, 0.25, 0.50, 0.75, 1};
void loop() {
  if (millis() - lastTime > 1000) {
    const bool online = pulseAnalyzer.isOnline();

    if (!semiPeriod && online) {
      Serial.println("Online");
      semiPeriod = pulseAnalyzer.getNominalGridPeriod() / 2;
      firingDelay = UINT32_MAX;
      timerStart(thyristorTimer);

    } else if (semiPeriod && !online) {
      Serial.println("Offline");
      timerStop(thyristorTimer);
      semiPeriod = 0;
      firingDelay = UINT32_MAX;
    }

    if (online) {
      firingDelay = dutyCycles[i] * semiPeriod;
      Serial.printf("Duty cycle: %f, firing delay: %" PRIu32 "\n", dutyCycles[i], firingDelay);

      i = (i + 1) % 5;
    } else {
      gpio_set_level(PIN_THYRISTOR, LOW);
    }

    lastTime = millis();
  }
}
