// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 *
 * Run with:
 *  -D CONFIG_ARDUINO_ISR_IRAM=1
 *  -D CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=1
 *  -D CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=1
 *  -D CONFIG_GPTIMER_ISR_IRAM_SAFE=1
 *  -D CONFIG_GPIO_CTRL_FUNC_IN_IRAM=1
 *
 * To shift the the ZC event, use: -D MYCILA_PULSE_ZC_SHIFT_US=x
 */
#include <MycilaPulseAnalyzer.h>

#include <esp32-hal-gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

#include <inlined_gptimer.h>

#include <esp32-hal.h>

#include <esp32-hal-log.h>

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
gptimer_handle_t thyristorTimer;

volatile uint32_t firingDelay = UINT32_MAX;
volatile uint32_t semiPeriod = 0;

static void ARDUINO_ISR_ATTR onZeroCross(int16_t delay, void* arg) {
  // reset thyristor timer to start counting from this ZC event
  ESP_ERROR_CHECK(inlined_gptimer_set_raw_count(thyristorTimer, 0));

  if (!firingDelay) {
    // 100% duty cycle => leave on
    gpio_ll_set_level(&GPIO, PIN_THYRISTOR, HIGH);
    return;
  }

  // make sure thyristor is stoped at ZC point
  gpio_ll_set_level(&GPIO, PIN_THYRISTOR, LOW);

  // 100% duty cycle
  if (firingDelay < PHASE_DELAY_MIN_US)
    firingDelay = PHASE_DELAY_MIN_US;

  if (firingDelay < semiPeriod) {
    gptimer_alarm_config_t alarm_cfg;
    alarm_cfg.alarm_count = firingDelay;
    alarm_cfg.reload_count = 0;
    alarm_cfg.flags.auto_reload_on_alarm = false;
    ESP_ERROR_CHECK(inlined_gptimer_set_alarm_action(thyristorTimer, &alarm_cfg));
  }
}

static bool ARDUINO_ISR_ATTR onThyristorTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg) {
  gpio_ll_set_level(&GPIO, PIN_THYRISTOR, HIGH);
  return false;
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

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_THYRISTOR, OUTPUT);

  gptimer_config_t timer_config;
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000; // 1MHz resolution
  timer_config.flags.intr_shared = true;
  timer_config.intr_priority = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  timer_config.flags.backup_before_sleep = false;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  timer_config.flags.allow_pd = false;
#endif

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &thyristorTimer));
  gptimer_event_callbacks_t timer_callbacks;
  timer_callbacks.on_alarm = onThyristorTimer;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(thyristorTimer, &timer_callbacks, nullptr));
  ESP_ERROR_CHECK(gptimer_enable(thyristorTimer));
  ESP_ERROR_CHECK(inlined_gptimer_start(thyristorTimer));

  pulseAnalyzer.onZeroCross(onZeroCross);
  pulseAnalyzer.begin(35);

  // Simulate some flash operations at the same time.
  xTaskCreate(flash_operations, "flash_op", 4096, NULL, uxTaskPriorityGet(NULL), NULL);
}

uint32_t lastTime = 0;
size_t i = 0;
float dutyCycles[5] = {0, 0.25, 0.50, 0.75, 1};
void loop() {
  if (millis() - lastTime > 2000) {
    const bool online = pulseAnalyzer.isOnline();

    if (!semiPeriod && online) {
      Serial.println("Online");
      semiPeriod = pulseAnalyzer.getNominalGridPeriod() / 2;
      firingDelay = UINT32_MAX;

    } else if (semiPeriod && !online) {
      Serial.println("Offline");
      semiPeriod = 0;
      firingDelay = UINT32_MAX;
    }

    if (online) {
      firingDelay = dutyCycles[i] * semiPeriod;
      Serial.printf("Duty cycle: %f, firing delay: %" PRIu32 "\n", dutyCycles[i], firingDelay);

      i = (i + 1) % 5;
    } else {
      gpio_ll_set_level(&GPIO, PIN_THYRISTOR, LOW);
    }

    lastTime = millis();
  }
}
