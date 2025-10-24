// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"

// memory
#include <esp_attr.h>

// gpio
#include <driver/gpio.h>
#include <esp32-hal-gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

// logging
#include <esp32-hal-log.h>

// timers
#include "priv/inlined_gptimer.h"

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

// Periods

#define MYCILA_PERIOD_48_US 20800 // for 48 Hz
#define MYCILA_PERIOD_49_US 20408 // for 49 Hz
#define MYCILA_PERIOD_50_US 20000 // for 50 Hz
#define MYCILA_PERIOD_51_US 19608 // for 51 Hz
#define MYCILA_PERIOD_52_US 19200 // for 52 Hz

#define MYCILA_PERIOD_58_US 17240 // for 58 Hz
#define MYCILA_PERIOD_59_US 16950 // for 59 Hz
#define MYCILA_PERIOD_60_US 16666 // for 60 Hz
#define MYCILA_PERIOD_61_US 16394 // for 61 Hz
#define MYCILA_PERIOD_62_US 16130 // for 62 Hz

// Semi-periods

#define MYCILA_SEMI_PERIOD_48_US 10400 // for 48 Hz
#define MYCILA_SEMI_PERIOD_49_US 10204 // for 49 Hz
#define MYCILA_SEMI_PERIOD_50_US 10000 // for 50 Hz
#define MYCILA_SEMI_PERIOD_51_US 9804  // for 51 Hz
#define MYCILA_SEMI_PERIOD_52_US 9600  // for 52 Hz

#define MYCILA_SEMI_PERIOD_58_US 8620 // for 58 Hz
#define MYCILA_SEMI_PERIOD_59_US 8475 // for 59 Hz
#define MYCILA_SEMI_PERIOD_60_US 8333 // for 60 Hz
#define MYCILA_SEMI_PERIOD_61_US 8197 // for 61 Hz
#define MYCILA_SEMI_PERIOD_62_US 8065 // for 62 Hz

// pulse width filtering to avoid spurious detections
#define MYCILA_PULSE_MIN_WIDTH_US 100
#define MYCILA_PULSE_MAX_WIDTH_US 21000

#define PERIODS_LEN 10 // array length of the PERIODS and SEMI_PERIODS arrays

static constexpr uint16_t PERIODS[] = {
  MYCILA_PERIOD_48_US,
  MYCILA_PERIOD_49_US,
  MYCILA_PERIOD_50_US,
  MYCILA_PERIOD_51_US,
  MYCILA_PERIOD_52_US,
  MYCILA_PERIOD_58_US,
  MYCILA_PERIOD_59_US,
  MYCILA_PERIOD_60_US,
  MYCILA_PERIOD_61_US,
  MYCILA_PERIOD_62_US,
};
static constexpr uint16_t SEMI_PERIODS[] = {
  MYCILA_SEMI_PERIOD_48_US,
  MYCILA_SEMI_PERIOD_49_US,
  MYCILA_SEMI_PERIOD_50_US,
  MYCILA_SEMI_PERIOD_51_US,
  MYCILA_SEMI_PERIOD_52_US,
  MYCILA_SEMI_PERIOD_58_US,
  MYCILA_SEMI_PERIOD_59_US,
  MYCILA_SEMI_PERIOD_60_US,
  MYCILA_SEMI_PERIOD_61_US,
  MYCILA_SEMI_PERIOD_62_US,
};

// search the closest value in above arrays sorted in descending order
__attribute__((always_inline)) inline static uint16_t closest(const uint16_t* array, uint16_t n) {
  int32_t left = 0, right = PERIODS_LEN - 1, mid;

  // binary search
  while (left <= right) {
    mid = left + ((right - left) >> 1);
    // found!
    if (array[mid] == n)
      return n;
    // if middle value is greater than n, search in the remaining right half
    if (array[mid] > n)
      left = mid + 1;
    else
      right = mid - 1; // right can become before left
  }

  // Safely handle bounds - ensure we don't access invalid array indices
  if (right < 0)
    return array[0]; // Use first element if right underflowed
  if (left >= PERIODS_LEN)
    return array[PERIODS_LEN - 1]; // Use last element if left overflowed

  // Both indices are valid, return the closest value
  uint16_t leftDiff = (array[left] > n) ? (array[left] - n) : (n - array[left]);
  uint16_t rightDiff = (array[right] > n) ? (array[right] - n) : (n - array[right]);

  return (leftDiff < rightDiff) ? array[left] : array[right];
}

#ifdef MYCILA_JSON_SUPPORT
void Mycila::PulseAnalyzer::toJson(const JsonObject& root) const {
  root["enabled"] = isEnabled();
  root["online"] = isOnline();
  root["type"] = static_cast<uint8_t>(_type);
  root["frequency"] = getFrequency();
  root["period"] = _period;
  root["period_min"] = _periodMin;
  root["period_max"] = _periodMax;
  root["shift"] = _shift;
  root["width"] = _width;
  root["width_min"] = _widthMin;
  root["width_max"] = _widthMax;
  root["grid"]["frequency"] = getNominalGridFrequency();
  root["grid"]["period"] = getNominalGridPeriod();
  root["grid"]["semi-period"] = getNominalGridSemiPeriod();
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  timer_config.flags.backup_before_sleep = false;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  timer_config.flags.allow_pd = false;
#endif

  // watchdog timer

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_onlineTimer));
  gptimer_event_callbacks_t online_callbacks;
  online_callbacks.on_alarm = _onlineTimerISR;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(_onlineTimer, &online_callbacks, this));
  ESP_ERROR_CHECK(gptimer_enable(_onlineTimer));
  ESP_ERROR_CHECK(gptimer_start(_onlineTimer));

  // zc timer

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_zcTimer));
  gptimer_event_callbacks_t zc_callbacks;
  zc_callbacks.on_alarm = _zcTimerISR;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(_zcTimer, &zc_callbacks, this));
  ESP_ERROR_CHECK(gptimer_enable(_zcTimer));
  ESP_ERROR_CHECK(gptimer_start(_zcTimer));

  // start ZC pulse detection

  attachInterruptArg(_pinZC, _edgeISR, this, CHANGE);

  // start watchdog timer
  gptimer_alarm_config_t online_alarm_cfg;
  online_alarm_cfg.alarm_count = 20 * MYCILA_PERIOD_48_US; // more than 400 ms
  online_alarm_cfg.reload_count = 0;
  online_alarm_cfg.flags.auto_reload_on_alarm = true;
  ESP_ERROR_CHECK(gptimer_set_alarm_action(_onlineTimer, &online_alarm_cfg));
  ESP_ERROR_CHECK(gptimer_set_raw_count(_onlineTimer, 0));

  return true;
}

void Mycila::PulseAnalyzer::end() {
  if (!isEnabled())
    return;

  LOGI(TAG, "Disable Pulse Analyzer on pin %" PRIu8, (uint8_t)_pinZC);

  ESP_ERROR_CHECK(gptimer_stop(_onlineTimer));
  ESP_ERROR_CHECK(gptimer_disable(_onlineTimer));
  ESP_ERROR_CHECK(gptimer_del_timer(_onlineTimer));
  _onlineTimer = NULL;

  ESP_ERROR_CHECK(gptimer_stop(_zcTimer));
  ESP_ERROR_CHECK(gptimer_disable(_zcTimer));
  ESP_ERROR_CHECK(gptimer_del_timer(_zcTimer));
  _zcTimer = NULL;

  detachInterrupt(_pinZC);

  _pinZC = GPIO_NUM_NC;

  _size = 0;
  _lastEvent = Event::SIGNAL_NONE;
  _type = Type::TYPE_UNKNOWN;
  _shift = 0;

  _period = 0;
  _periodMin = 0;
  _periodMax = 0;

  _nominalSemiPeriod = 0;

  _width = 0;
  _widthMin = 0;
  _widthMax = 0;
}

bool ARDUINO_ISR_ATTR Mycila::PulseAnalyzer::_zcTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  if (instance->_onZeroCross)
    instance->_onZeroCross(-instance->_shiftZC, instance->_onZeroCrossArg);
  return false;
}

bool ARDUINO_ISR_ATTR Mycila::PulseAnalyzer::_onlineTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  inlined_gptimer_set_raw_count(instance->_zcTimer, 0);
  inlined_gptimer_set_alarm_action(instance->_zcTimer, nullptr);

  instance->_size = 0;
  instance->_lastEvent = Event::SIGNAL_NONE;
  instance->_type = Type::TYPE_UNKNOWN;
  instance->_shift = 0;

  instance->_period = 0;
  instance->_periodMin = 0;
  instance->_periodMax = 0;

  instance->_nominalSemiPeriod = 0;

  instance->_width = 0;
  instance->_widthMin = 0;
  instance->_widthMax = 0;

  return false;
}

void ARDUINO_ISR_ATTR Mycila::PulseAnalyzer::_edgeISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  gptimer_handle_t zcTimer = instance->_zcTimer;
  gptimer_handle_t onlineTimer = instance->_onlineTimer;

  if (!onlineTimer || !zcTimer)
    return;

  uint64_t diff;
  if (inlined_gptimer_get_raw_count(onlineTimer, &diff) != ESP_OK)
    return;

  // Filter out spurious interrupts happening during a slow rising / falling slope
  // See: https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection
  if (diff < MYCILA_PULSE_MIN_WIDTH_US)
    return;

  // Reset Watchdog for online/offline detection
  inlined_gptimer_set_raw_count(onlineTimer, 0);

  // long time no see ? => reset
  if (diff > MYCILA_PERIOD_48_US) {
    instance->_size = 0;
    instance->_lastEvent = Event::SIGNAL_NONE;
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("ERR: diff\n");
#endif
    return;
  }

  // Edge detection
  const Event event = gpio_ll_get_level(&GPIO, instance->_pinZC) ? Event::SIGNAL_RISING : Event::SIGNAL_FALLING;

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

  // sync alarms for ZC ISR
  if (instance->_type) {
    switch (instance->_type) {
      case Type::TYPE_FULL_PERIOD:
      case Type::TYPE_SEMI_PERIOD: {
        inlined_gptimer_set_raw_count(zcTimer, (instance->_shift < 0 ? 0 : instance->_nominalSemiPeriod) - instance->_shift);
        break;
      }
      case Type::TYPE_SHORT: {
        if (event == Event::SIGNAL_FALLING) {
          int16_t pos = (static_cast<int16_t>(diff) >> 1) - instance->_shift; // position == middle of the pulse compensated by shift
          if (pos < 0)
            pos += instance->_nominalSemiPeriod;
          inlined_gptimer_set_raw_count(zcTimer, pos);
        }
        break;
      }
      default:
        assert(false);
        break;
    }
  }

  // trigger callback
  if (instance->_onEdge)
    instance->_onEdge(event, instance->_onEdgeArg);

  // Pulse analysis done ?
  if (instance->_type)
    return;

  instance->_widths[instance->_size++] = diff;

  // analyze pulse width when we have all samples
  if (instance->_size == MYCILA_PULSE_SAMPLES) {
    // analyze pulse width
    int32_t value = 0, sum = 0, min = INT32_MAX, max = 0;

    for (size_t i = event == Event::SIGNAL_RISING ? 0 : 1; i < MYCILA_PULSE_SAMPLES; i += 2) {
      value = instance->_widths[i];
      sum += value;
      if (value < min)
        min = value;
      if (value > max)
        max = value;
    }

    value = (sum << 1) / MYCILA_PULSE_SAMPLES;

    if (value >= MYCILA_PULSE_MIN_WIDTH_US && value <= MYCILA_PULSE_MAX_WIDTH_US) {
      instance->_width = value;
      instance->_widthMin = min;
      instance->_widthMax = max;

      // analyze pulse period
      value = 0, sum = 0, min = INT32_MAX, max = 0;

      for (size_t i = 1; i < MYCILA_PULSE_SAMPLES; i += 2) {
        value = instance->_widths[i] + instance->_widths[i - 1];
        sum += value;
        if (value < min)
          min = value;
        if (value > max)
          max = value;
      }

      value = (sum << 1) / MYCILA_PULSE_SAMPLES;

#ifdef MYCILA_PULSE_DEBUG
      ets_printf("DBG: value=%d\n", value);
#endif

      // value ~= 40000 at 50 Hz with JSY-MK-194G pulse of 20 ms
      // value ~= 33333 at 60 Hz with JSY-MK-194G pulse of 20 ms
      // value ~= 20000 at 50 Hz with BM1Z102FJ pulse of 10 ms
      // value ~= 16666 at 60 Hz with BM1Z102FJ pulse of 10 ms
      // -------- 16130 -------------------------------------
      // value ~= 10000 at 50 Hz with Robodyn pulse of 450 us
      // value ~=  8333 at 60 Hz with Robodyn pulse of 450 us
      if (value > MYCILA_PERIOD_62_US) {
        value >>= 1;
        min >>= 1;
        max >>= 1;

        if ((value > MYCILA_PERIOD_52_US && value < MYCILA_PERIOD_48_US) || (value > MYCILA_PERIOD_62_US && value < MYCILA_PERIOD_58_US)) {
          // full period pulses like JSY-MK-194G
          instance->_type = Type::TYPE_FULL_PERIOD;
          // JSY-MK-194G has a 100 us shift on the right (positif voltage point)
          // JSY-NK-194T has a 1000 us shift on the right (positif voltage point)
          // See: https://forum-photovoltaique.fr/viewtopic.php?p=798444#p798444
          instance->_shift = instance->_shiftZC + instance->_shiftJsySignal;

        } else if ((value > MYCILA_SEMI_PERIOD_52_US && value < MYCILA_SEMI_PERIOD_48_US) || (value > MYCILA_SEMI_PERIOD_62_US && value < MYCILA_SEMI_PERIOD_58_US)) {
          // semi period pulses like BM1Z102FJ
          instance->_type = Type::TYPE_SEMI_PERIOD;
          instance->_shift = instance->_shiftZC;
        }

      } else if ((value > MYCILA_SEMI_PERIOD_52_US && value < MYCILA_SEMI_PERIOD_48_US) || (value > MYCILA_SEMI_PERIOD_62_US && value < MYCILA_SEMI_PERIOD_58_US)) {
        // short pulses like Robodyn, ZCD from Daniel S, etc
        instance->_type = Type::TYPE_SHORT;
        instance->_shift = instance->_shiftZC;
      }

      if (instance->_type != Type::TYPE_UNKNOWN) {
        instance->_period = value;
        instance->_periodMin = min;
        instance->_periodMax = max;

        sum = 0;
        switch (instance->_type) {
          case Type::TYPE_FULL_PERIOD: {
            instance->_nominalSemiPeriod = closest(PERIODS, value) >> 1;
            sum = (instance->_shift < 0 ? 0 : instance->_nominalSemiPeriod) - instance->_shift;
            break;
          }
          case Type::TYPE_SEMI_PERIOD: {
            instance->_nominalSemiPeriod = closest(SEMI_PERIODS, value);
            sum = (instance->_shift < 0 ? 0 : instance->_nominalSemiPeriod) - instance->_shift;
            break;
          }
          case Type::TYPE_SHORT: {
            instance->_nominalSemiPeriod = closest(SEMI_PERIODS, value);
            if (event == Event::SIGNAL_FALLING)
              sum = (static_cast<int16_t>(diff) >> 1) - instance->_shift; // position == middle of the pulse compensated by shift
            else
              sum = -(static_cast<int16_t>(diff) >> 1) - instance->_shift;
            if (sum < 0)
              sum += instance->_nominalSemiPeriod;
            break;
          }
          default:
            assert(false);
            break;
        }

        // start ZC timer
        gptimer_alarm_config_t alarm_cfg;
        alarm_cfg.alarm_count = instance->_nominalSemiPeriod;
        alarm_cfg.reload_count = 0;
        alarm_cfg.flags.auto_reload_on_alarm = true;
        inlined_gptimer_set_raw_count(zcTimer, sum);
        inlined_gptimer_set_alarm_action(zcTimer, &alarm_cfg);
        return;
      }
    }

    // reset index for a next round of capture
    instance->_size = 0;
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("ERR: width\n");
#endif
  }
}
