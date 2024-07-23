// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#pragma once

#include <Arduino.h>
#include <MycilaCircularBuffer.h>

#ifdef MYCILA_JSON_SUPPORT
#include <ArduinoJson.h>
#endif

#define MYCILA_PULSE_VERSION "1.0.0"
#define MYCILA_PULSE_VERSION_MAJOR 1
#define MYCILA_PULSE_VERSION_MINOR 0
#define MYCILA_PULSE_VERSION_REVISION 0

namespace Mycila {
  // N is the number of samples
  template <size_t N>
  class PulseAnalyzer {
    public:
      // Zero-Cross pin
      // Note: this does not start the analyzer. This is totally ok to call begin even if the pin is used by another component.
      void begin(const int8_t pin);
      void end();

      gpio_num_t getPin() const { return _pin; }
      bool isEnabled() const { return _enabled; }

      // Pulse period in microseconds (average of the last N samples)
      uint32_t getPeriod() const { return _period; }
      // Minimum pulse period ever seen in microseconds
      uint32_t getPeriodMin() const { return _periodMin; }
      // Maximum pulse period ever seen in microseconds
      uint32_t getPeriodMax() const { return _periodMax; }
      // Pulse frequency in Hz
      float getFrequency() const { return _period == 0 ? 0 : 1e6f / _period; }
      // Current frequency in Hz
      float getCurrentFrequency() const { return getFrequency() / 2; }
      // Pulse length in microseconds (average of the last N samples)
      uint32_t getLength() const { return _length; }
      // Minimum pulse length ever seen in microseconds
      uint32_t getLengthMin() const { return _lengthMin; }
      // Maximum pulse length ever seen in microseconds
      uint32_t getLengthMax() const { return _lengthMax; }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const {
        root["enabled"] = _enabled;
        root["period"] = _period;
        root["period_min"] = _periodMin;
        root["period_max"] = _periodMax;
        root["frequency"] = getFrequency();
        root["current_frequency"] = getCurrentFrequency();
        root["length"] = _length;
        root["length_min"] = _lengthMin;
        root["length_max"] = _lengthMax;
      }
#endif

    private:
      bool _enabled = false;
      gpio_num_t _pin = GPIO_NUM_NC;
      volatile uint32_t _period = 0;
      volatile uint32_t _periodMin = 0;
      volatile uint32_t _periodMax = 0;
      volatile uint32_t _length = 0;
      volatile uint32_t _lengthMin = 0;
      volatile uint32_t _lengthMax = 0;
      Mycila::CircularBuffer<uint32_t, N> _periods;
      Mycila::CircularBuffer<uint32_t, N> _lengths;
  };
} // namespace Mycila
