// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"

#include <driver/gptimer.h>
#include <esp32-hal-log.h>

#ifdef MYCILA_PULSE_DEBUG
  #include <rom/ets_sys.h>
#endif

#ifdef MYCILA_LOGGER_SUPPORT
  #include <MycilaLogger.h>
extern Mycila::Logger logger;
  #define LOGD(tag, format, ...) logger.debug(tag, format, ##__VA_ARGS__)
  #define LOGI(tag, format, ...) logger.info(tag, format, ##__VA_ARGS__)
  #define LOGW(tag, format, ...) logger.warn(tag, format, ##__VA_ARGS__)
  #define LOGE(tag, format, ...) logger.error(tag, format, ##__VA_ARGS__)
#else
  #define LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
  #define LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
  #define LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
  #define LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#endif

#define TAG "PULSE"

#ifndef GPIO_IS_VALID_GPIO
  #define GPIO_IS_VALID_GPIO(gpio_num) ((gpio_num >= 0) && \
                                        (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0))
#endif

#ifdef MYCILA_JSON_SUPPORT
void Mycila::PulseAnalyzer::toJson(const JsonObject& root) const {
  root["enabled"] = isEnabled();
  root["online"] = isOnline();
  root["grid_period"] = _nominalGridPeriod;
  root["period"] = _period;
  root["period_min"] = _periodMin;
  root["period_max"] = _periodMax;
  root["width"] = _width;
  root["width_min"] = _widthMin;
  root["width_max"] = _widthMax;
}
#endif

bool Mycila::PulseAnalyzer::begin(int8_t pinZC) {
  if (isEnabled())
    return true;

  if (GPIO_IS_VALID_GPIO(pinZC)) {
    _pinZC = (gpio_num_t)pinZC;
    pinMode(_pinZC, INPUT);
  } else {
    LOGE(TAG, "Invalid ZC input pin: %" PRId8, pinZC);
    _pinZC = GPIO_NUM_NC;
    return false;
  }

  LOGI(TAG, "Enable Pulse Analyzer on pin %" PRIu8, pinZC);

  gptimer_config_t timer_config;
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000; // 1MHz resolution
  timer_config.flags.intr_shared = true;
  timer_config.intr_priority = 0;

  // watchdog timer

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_onlineTimer));
  gptimer_event_callbacks_t online_callbacks;
  online_callbacks.on_alarm = _onlineTimerISR;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(_onlineTimer, &online_callbacks, this));
  ESP_ERROR_CHECK(gptimer_enable(_onlineTimer));

  gptimer_alarm_config_t online_alarm_cfg;
  online_alarm_cfg.alarm_count = (MYCILA_PULSE_SAMPLES / 2 + 1) * MYCILA_PULSE_MAX_SEMI_PERIOD_US;
  online_alarm_cfg.reload_count = 0;
  online_alarm_cfg.flags.auto_reload_on_alarm = true;
  ESP_ERROR_CHECK(gptimer_set_alarm_action(_onlineTimer, &online_alarm_cfg));

  // zc timer

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_zcTimer));
  gptimer_event_callbacks_t zc_callbacks;
  zc_callbacks.on_alarm = _zcTimerISR;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(_zcTimer, &zc_callbacks, this));
  ESP_ERROR_CHECK(gptimer_enable(_zcTimer));

  // start ZC pulse detection

  attachInterruptArg(_pinZC, _edgeISR, this, CHANGE);

  return true;
}

void Mycila::PulseAnalyzer::end() {
  if (!isEnabled())
    return;

  LOGI(TAG, "Disable Pulse Analyzer on pin %" PRIu8, (uint8_t)_pinZC);

  gptimer_stop(_onlineTimer); // might be already stopped
  ESP_ERROR_CHECK(gptimer_disable(_onlineTimer));
  ESP_ERROR_CHECK(gptimer_del_timer(_onlineTimer));
  _onlineTimer = NULL;

  gptimer_stop(_zcTimer); // might be already stopped
  ESP_ERROR_CHECK(gptimer_disable(_zcTimer));
  ESP_ERROR_CHECK(gptimer_del_timer(_zcTimer));
  _zcTimer = NULL;

  detachInterrupt(_pinZC);

  _pinZC = GPIO_NUM_NC;

  _size = 0;
  _lastEvent = Event::SIGNAL_NONE;
  _type = Type::TYPE_UNKNOWN;

  _period = 0;
  _periodMin = 0;
  _periodMax = 0;

  _nominalGridPeriod = 0;

  _width = 0;
  _widthMin = 0;
  _widthMax = 0;
}

bool IRAM_ATTR Mycila::PulseAnalyzer::_zcTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  if (instance->_onZeroCross)
    instance->_onZeroCross(instance->_onZeroCrossArg);
  return false;
}

bool IRAM_ATTR Mycila::PulseAnalyzer::_onlineTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  gptimer_stop(instance->_zcTimer); // might be already stopped
  ESP_ERROR_CHECK(gptimer_set_raw_count(instance->_zcTimer, 0));

  gptimer_stop(instance->_onlineTimer); // might be already stopped
  ESP_ERROR_CHECK(gptimer_set_raw_count(instance->_onlineTimer, 0));
  ESP_ERROR_CHECK(gptimer_set_alarm_action(instance->_zcTimer, nullptr));

  instance->_size = 0;
  instance->_lastEvent = Event::SIGNAL_NONE;
  instance->_type = Type::TYPE_UNKNOWN;

  instance->_period = 0;
  instance->_periodMin = 0;
  instance->_periodMax = 0;

  instance->_nominalGridPeriod = 0;

  instance->_width = 0;
  instance->_widthMin = 0;
  instance->_widthMax = 0;

  return false;
}

void IRAM_ATTR Mycila::PulseAnalyzer::_edgeISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  gptimer_handle_t zcTimer = instance->_zcTimer;
  gptimer_handle_t onlineTimer = instance->_onlineTimer;

  uint64_t raw_count;
  ESP_ERROR_CHECK(gptimer_get_raw_count(onlineTimer, &raw_count));

  const uint32_t diff = static_cast<uint32_t>(raw_count);
  const uint32_t period = instance->_nominalGridPeriod / 2;

  // connected for the first time ?
  if (!diff) {
    ESP_ERROR_CHECK(gptimer_start(onlineTimer));
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("init\n");
#endif
    return;
  }

  // Filter out spurious interrupts happening during a slow rising / falling slope
  // See: https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection
  if (diff < MYCILA_PULSE_MIN_PULSE_WIDTH_US)
    return;

  // Reset Watchdog for online/offline detection
  ESP_ERROR_CHECK(gptimer_set_raw_count(onlineTimer, 0));

  // long time no see ? => reset
  if (diff > MYCILA_PULSE_MAX_PULSE_WIDTH_US) {
    instance->_size = 0;
    instance->_lastEvent = Event::SIGNAL_NONE;
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("ERR: diff\n");
#endif
    return;
  }

  // Edge detection
  const Event event = gpio_get_level(instance->_pinZC) == HIGH ? Event::SIGNAL_RISING : Event::SIGNAL_FALLING;

  // noise in edge detection ? => reset count, just in case
  // But this is still possible that the noise is caused by the wrong voltage detection above
  // so we do not update the zcTimer and we let it run if it was started
  if (instance->_lastEvent == event) {
    instance->_size = 0;
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("ERR: edge\n");
#endif
  }

  instance->_lastEvent = event;

  // set alarms for ZC ISR
  if (period) {
    switch (instance->_type) {
      case Type::TYPE_UNKNOWN:
        // try to detect the BM1Z102FJ pulses (which match the semi-period)
        if (instance->_widthMin && instance->_widthMax && period >= instance->_widthMin && period <= instance->_widthMax) {
          instance->_type = Type::TYPE_BM1Z102FJ;
          _zcTimerISR(zcTimer, nullptr, instance);

        } else {
          instance->_type = Type::TYPE_PULSE;
          ESP_ERROR_CHECK(gptimer_start(zcTimer));

          gptimer_alarm_config_t alarm_cfg;
          alarm_cfg.alarm_count = period;
          alarm_cfg.reload_count = 0;
          alarm_cfg.flags.auto_reload_on_alarm = true;
          ESP_ERROR_CHECK(gptimer_set_alarm_action(zcTimer, &alarm_cfg));

          if (event == Event::SIGNAL_FALLING) {
            int position = diff / 2;
            if (position >= MYCILA_PULSE_ZC_SHIFT_US)
              position -= MYCILA_PULSE_ZC_SHIFT_US;
            ESP_ERROR_CHECK(gptimer_set_raw_count(zcTimer, position));
          }
        }
        break;

      case Type::TYPE_BM1Z102FJ:
        _zcTimerISR(zcTimer, nullptr, instance);
        break;

      case Type::TYPE_PULSE:
        if (event == Event::SIGNAL_FALLING) {
          int position = diff / 2;
          if (position >= MYCILA_PULSE_ZC_SHIFT_US)
            position -= MYCILA_PULSE_ZC_SHIFT_US;
          ESP_ERROR_CHECK(gptimer_set_raw_count(zcTimer, position));
        }
        break;

      default:
        assert(false);
        break;
    }
  }

  // trigger callback
  if (instance->_onEdge)
    instance->_onEdge(event, instance->_onEdgeArg);

  // Pulse analysis done ?
  if (period && instance->_width)
    return;

  instance->_widths[instance->_size] = diff;
  instance->_size = instance->_size + 1;

  if (instance->_size == MYCILA_PULSE_SAMPLES) {
    uint32_t value = 0, sum = 0, min = UINT32_MAX, max = 0;

    for (size_t i = event == Event::SIGNAL_RISING ? 0 : 1; i < MYCILA_PULSE_SAMPLES; i += 2) {
      value = instance->_widths[i];
      sum += value;
      if (value < min)
        min = value;
      if (value > max)
        max = value;
    }

    if (min >= MYCILA_PULSE_MIN_PULSE_WIDTH_US && max <= MYCILA_PULSE_MAX_PULSE_WIDTH_US) {
      instance->_width = sum * 2 / MYCILA_PULSE_SAMPLES;
      instance->_widthMin = min;
      instance->_widthMax = max;

    } else {
#ifdef MYCILA_PULSE_DEBUG
      ets_printf("ERR: width\n");
#endif
    }

    // analyze pulse period
    value = 0, sum = 0, min = UINT32_MAX, max = 0;

    for (size_t i = 1; i < MYCILA_PULSE_SAMPLES; i += 2) {
      value = instance->_widths[i] + instance->_widths[i - 1];
      sum += value;
      if (value < min)
        min = value;
      if (value > max)
        max = value;
    }

    if (min >= MYCILA_PULSE_MIN_SEMI_PERIOD_US && max <= MYCILA_PULSE_MAX_SEMI_PERIOD_US) {
      value = sum * 2 / MYCILA_PULSE_SAMPLES;
      instance->_period = value;
      instance->_periodMin = min;
      instance->_periodMax = max;
      instance->_nominalGridPeriod = value ? 1000000 / ((10000000 / value + 5) / 20) : 0;
    } else {
#ifdef MYCILA_PULSE_DEBUG
      ets_printf("ERR: period\n");
#endif
    }

    // reset index for a next round of capture
    instance->_size = 0;
  }
}
