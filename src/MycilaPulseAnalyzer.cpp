// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"

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

template <size_t N>
void Mycila::PulseAnalyzer<N>::begin(const int8_t pin) {
  if (_enabled)
    return;

  if (GPIO_IS_VALID_GPIO(pin)) {
    _pin = (gpio_num_t)pin;
  } else {
    LOGE(TAG, "Invalid Zero-Cross pin: %" PRId8, pin);
    _pin = GPIO_NUM_NC;
    return;
  }

  LOGI(TAG, "Enable Pulse Analyzer on pin %" PRId8, _pin);

  _enabled = true;
}

template <size_t N>
void Mycila::PulseAnalyzer<N>::end() {
  if (_enabled) {
    LOGI(TAG, "Disable Pulse Analyzer on pin %" PRId8, _pin);
    _enabled = false;
    _pin = GPIO_NUM_NC;
  }
}
