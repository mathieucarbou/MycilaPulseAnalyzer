// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"
#include "Arduino.h"

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

  _lastEdgeTime = 0;
  _size = 0;
  _period = 0;
  _periodMin = 0;
  _periodMax = 0;
  _width = 0;
  _widthMin = 0;
  _widthMax = 0;

  // watchdog
  _onlineTimer = timerBegin(1000000);
  assert(_onlineTimer);
  timerStop(_onlineTimer);
  timerWrite(_onlineTimer, 0);
  timerAttachInterruptArg(_onlineTimer, _offlineISR, this);
  timerAlarm(_onlineTimer, MYCILA_PULSE_SAMPLES * MYCILA_PULSE_MAX_SEMI_PERIOD_US, true, 0);

  // zc timer
  _zcTimer = timerBegin(1000000);
  assert(_zcTimer);
  timerStop(_zcTimer);
  timerWrite(_zcTimer, 0);
  timerAttachInterruptArg(_zcTimer, _onZeroCross, this);

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
}

void Mycila::PulseAnalyzer::_offlineISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  timerStop(instance->_zcTimer);
  timerWrite(instance->_zcTimer, 0);
  timerAlarm(instance->_zcTimer, UINT64_MAX, false, 0);

  timerStop(instance->_onlineTimer);
  timerWrite(instance->_onlineTimer, 0);

  instance->_size = 0;
  instance->_lastEdgeTime = 0;
  instance->_period = 0;
  instance->_periodMin = 0;
  instance->_periodMax = 0;
  instance->_width = 0;
  instance->_widthMin = 0;
  instance->_widthMax = 0;
}

void Mycila::PulseAnalyzer::_zcISR(void* arg) {
  // Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;
  // if (instance->_onZeroCross)
  //   instance->_onZeroCross(instance->_onZeroCrossArg);
}

void Mycila::PulseAnalyzer::_edgeISR(void* arg) {
  Mycila::PulseAnalyzer* instance = (Mycila::PulseAnalyzer*)arg;

  const uint32_t now = esp_timer_get_time();
  const uint32_t before = instance->_lastEdgeTime;
  const uint32_t diff = now - before;

  // connected for the first time ? just record its timestamp
  if (!before) {
    instance->_lastEdgeTime = now;
    instance->_size = 0;
    timerStart(instance->_onlineTimer);
    timerStart(instance->_zcTimer);
    return;
  }

  // Filter out spurious interrupts happening during pulse rise/fall slope
  // See: https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection
  if (diff < MYCILA_PULSE_MIN_PULSE_WIDTH_US)
    return;

  // save time
  instance->_lastEdgeTime = now;

  // long time no see ? => reset
  if (diff > MYCILA_PULSE_MAX_PULSE_WIDTH_US) {
    instance->_size = 0;
    return;
  }

  // Reset Watchdog for online/offline detection
  timerRestart(instance->_onlineTimer);

  // record new diff
  instance->_widths[instance->_size++] = diff;

  // Edge detection and ZC timer sync
  Event event = Event::SIGNAL_NONE;
  if (instance->_size > 1) {
    const uint32_t prevDiff = instance->_widths[instance->_size - 2];
    if (diff > prevDiff && diff - prevDiff > MYCILA_PULSE_EQUALITY_DELTA_US) {
      event = Event::SIGNAL_RISING;
    } else if (prevDiff > diff && prevDiff - diff > MYCILA_PULSE_EQUALITY_DELTA_US) {
      event = Event::SIGNAL_FALLING;
    } else {
      event = Event::SIGNAL_CHANGE;
    }
  }

  // Edge callback
  if (instance->_onEdge)
    instance->_onEdge(event, instance->_onEdgeArg);

  // Pulse analysis
  if (instance->_size == MYCILA_PULSE_SAMPLES) {
    if (event == Event::SIGNAL_NONE) {
      // reset index for a next round of capture
      instance->_size = 0;
      return;
    }

    uint8_t bucket;
    uint32_t width;
    uint32_t sum[2] = {0, 0};
    uint32_t min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t max[2] = {0, 0};

    if (event == Event::SIGNAL_CHANGE) {
      // If pulses are all the same (i.e. BM1Z102FJ), we consolidate the results

      // analyze pulse width (which is also the period)
      for (size_t i = 0; i < MYCILA_PULSE_SAMPLES; i++) {
        width = instance->_widths[i];
        sum[0] += width;
        if (width < min[0])
          min[0] = width;
        if (width > max[0])
          max[0] = width;
      }

      if (min[0] >= MYCILA_PULSE_MIN_PULSE_WIDTH_US && max[0] <= MYCILA_PULSE_MAX_PULSE_WIDTH_US) {
        width = sum[0] / MYCILA_PULSE_SAMPLES;
        instance->_width = width;
        instance->_widthMin = min[0];
        instance->_widthMax = max[0];
        instance->_period = width;
        instance->_periodMin = min[0];
        instance->_periodMax = max[0];
      }

      timerWrite(instance->_zcTimer, -MYCILA_PULSE_ZC_SHIFT_US);

    } else {
      // regular case: short and long pulses

      // Iterate over all entries, considering that we have 2 entries per pulse (short and long, or long and short).
      // If pulses are all the same, we will consolidate the results at the end.
      for (size_t i = 0; i < MYCILA_PULSE_SAMPLES; i++) {
        bucket = i % 2;
        width = instance->_widths[i];
        sum[bucket] += width;
        if (width < min[bucket])
          min[bucket] = width;
        if (width > max[bucket])
          max[bucket] = width;
      }

      // If pulses are not the same, we keep the shortest ones for the pulse width
      bucket = sum[0] < sum[1] ? 0 : 1;

      if (min[bucket] >= MYCILA_PULSE_MIN_PULSE_WIDTH_US && max[bucket] <= MYCILA_PULSE_MAX_PULSE_WIDTH_US) {
        width = sum[bucket] * 2 / MYCILA_PULSE_SAMPLES;
        instance->_width = width;
        instance->_widthMin = min[bucket];
        instance->_widthMax = max[bucket];
      }

      // analyze pulse period
      sum[0] = 0;
      min[0] = UINT32_MAX;
      max[0] = 0;

      for (size_t i = 1; i < MYCILA_PULSE_SAMPLES; i += 2) {
        width = instance->_widths[i] + instance->_widths[i - 1];
        sum[0] += width;
        if (width < min[0])
          min[0] = width;
        if (width > max[0])
          max[0] = width;
      }

      if (min[0] >= MYCILA_PULSE_MIN_SEMI_PERIOD_US && max[0] <= MYCILA_PULSE_MAX_SEMI_PERIOD_US) {
        width = sum[0] * 2 / MYCILA_PULSE_SAMPLES;
        instance->_period = width;
        instance->_periodMin = min[0];
        instance->_periodMax = max[0];
      }

      if (event == Event::SIGNAL_RISING)
        timerWrite(instance->_zcTimer, instance->_period - instance->_width / 2 - MYCILA_PULSE_ZC_SHIFT_US);
      else // Event::SIGNAL_FALLING
        timerWrite(instance->_zcTimer, instance->_width / 2 - MYCILA_PULSE_ZC_SHIFT_US);
    }

    if (instance->_period && instance->_width)
      timerAlarm(instance->_zcTimer, instance->_period, true, 0);

    // reset index for a next round of capture
    instance->_size = 0;
  }
}
