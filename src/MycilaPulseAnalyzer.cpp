// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"
#include "Arduino.h"

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
  root["period"] = _period;
  root["period_min"] = _periodMin;
  root["period_max"] = _periodMax;
  root["frequency"] = getFrequency();
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

  // watchdog
  _onlineTimer = timerBegin(1000000);
  assert(_onlineTimer);
  timerStop(_onlineTimer);
  timerWrite(_onlineTimer, 0);
  timerAttachInterruptArg(_onlineTimer, _offlineISR, this);
  timerAlarm(_onlineTimer, (MYCILA_PULSE_SAMPLES / 2 + 1) * MYCILA_PULSE_MAX_SEMI_PERIOD_US, true, 0);

  // zc timer
  _zcTimer = timerBegin(1000000);
  assert(_zcTimer);
  timerStop(_zcTimer);
  timerWrite(_zcTimer, 0);
  timerAttachInterruptArg(_zcTimer, _zcISR, this);

  // start
  attachInterruptArg(_pinZC, _edgeISR, this, CHANGE);

  return true;
}

void Mycila::PulseAnalyzer::end() {
  if (!isEnabled())
    return;

  LOGI(TAG, "Disable Pulse Analyzer on pin %" PRIu8, (uint8_t)_pinZC);

  timerDetachInterrupt(_onlineTimer);
  timerEnd(_onlineTimer);
  _onlineTimer = nullptr;

  timerDetachInterrupt(_zcTimer);
  timerEnd(_zcTimer);
  _zcTimer = nullptr;

  detachInterrupt(_pinZC);

  _pinZC = GPIO_NUM_NC;

  _size = 0;
  _lastEvent = Event::SIGNAL_NONE;
  _type = Type::TYPE_UNKNOWN;

  _frequency = 0;
  _period = 0;
  _periodMin = 0;
  _periodMax = 0;

  _width = 0;
  _widthMin = 0;
  _widthMax = 0;
}

void Mycila::PulseAnalyzer::_offlineISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  timerStop(instance->_zcTimer);
  timerWrite(instance->_zcTimer, 0);
  timerAlarm(instance->_zcTimer, UINT64_MAX, false, 0);

  timerStop(instance->_onlineTimer);
  timerWrite(instance->_onlineTimer, 0);

  instance->_size = 0;
  instance->_lastEvent = Event::SIGNAL_NONE;
  instance->_type = Type::TYPE_UNKNOWN;

  instance->_frequency = 0;
  instance->_period = 0;
  instance->_periodMin = 0;
  instance->_periodMax = 0;

  instance->_width = 0;
  instance->_widthMin = 0;
  instance->_widthMax = 0;
}

void Mycila::PulseAnalyzer::_zcISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  if (instance->_onZeroCross)
    instance->_onZeroCross(instance->_onZeroCrossArg);
}

void Mycila::PulseAnalyzer::_edgeISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  const uint32_t diff = timerRead(instance->_onlineTimer);

  // connected for the first time ?
  if (!diff) {
    timerStart(instance->_onlineTimer);
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
  timerRestart(instance->_onlineTimer);

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

  // noise in edge detection ?
  if (instance->_lastEvent == event) {
    instance->_size = 0;
#ifdef MYCILA_PULSE_DEBUG
    ets_printf("ERR: edge\n");
#endif

  } else {
    instance->_lastEvent = event;
    instance->_widths[instance->_size++] = diff;
  }

  // set alarms for ZC ISR
  if (instance->_period) {
    switch (instance->_type) {
      case Type::TYPE_UNKNOWN:
        if (instance->_widthMin && instance->_widthMax && instance->_period >= instance->_widthMin && instance->_period <= instance->_widthMax) {
          instance->_type = Type::TYPE_BM1Z102FJ;
          _zcISR(instance);

        } else {
          instance->_type = Type::TYPE_PULSE;
          timerStart(instance->_zcTimer);
          timerAlarm(instance->_zcTimer, instance->_period, true, 0);
        }
        break;

      case Type::TYPE_BM1Z102FJ:
        _zcISR(instance);
        break;

      case Type::TYPE_PULSE:
        if (event == Event::SIGNAL_FALLING)
          timerWrite(instance->_zcTimer, diff / 2);
        break;

      default:
        assert(false);
        break;
    }
  }

  if (instance->_onEdge)
    instance->_onEdge(event, instance->_onEdgeArg);

  // Pulse analysis
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

      instance->_frequency = (10000000 / value + 5) / 10;
      instance->_period = 1000000 / instance->_frequency;
      instance->_periodMin = min;
      instance->_periodMax = max;

    } else {
#ifdef MYCILA_PULSE_DEBUG
      ets_printf("ERR: period\n");
#endif
    }

    // reset index for a next round of capture
    instance->_size = 0;
  }
}
