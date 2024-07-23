// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#pragma once

#include "hal/gpio_types.h"

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <cstddef>
#include <cstdint>

#define MYCILA_PULSE_VERSION          "1.0.0"
#define MYCILA_PULSE_VERSION_MAJOR    1
#define MYCILA_PULSE_VERSION_MINOR    0
#define MYCILA_PULSE_VERSION_REVISION 0

#ifndef MYCILA_PULSE_SAMPLES
  #define MYCILA_PULSE_SAMPLES 100
#endif

#ifndef MYCILA_MIN_PULSE_LENGTH_US
  #define MYCILA_MIN_PULSE_LENGTH_US 50
#endif

namespace Mycila {
  class PulseAnalyzer {
    public:
      enum class State {
        IDLE,
        RECORDING,
        RECORDED,
        ANALYZING,
        ANALYZED,
        SIMULATING,
      };

      // Start recording the pulses on the given pin
      // Returns true if started and running
      void record(int8_t pinZC);

      // Analyze the recorded pulses
      // Returns true if the pulse has been analyzed
      void analyze();

      // Simulate a pulse of a given length on a given output pin when the ZC pulse is received
      void simulate(int8_t pinZC, int8_t pinOutput, uint32_t outputPulseLengthMicros = 1);

      // Stop everything
      void end();

      // Zero-Cross pin
      gpio_num_t getZCPin() const { return _pinZC; }
      gpio_num_t getOutputPin() const { return _pinOutput; }
      State getState() const { return _state; }

      // Pulse period in microseconds (average of the last N samples)
      uint32_t getPeriod() const { return _period; }
      // Minimum pulse period ever seen in microseconds
      uint32_t getPeriodMin() const { return _periodMin; }
      // Maximum pulse period ever seen in microseconds
      uint32_t getPeriodMax() const { return _periodMax; }

      // Pulse frequency in Hz
      float getFrequency() const { return _period == 0 ? 0 : 1e6f / _period; }

      // Pulse length in microseconds (average of the last N samples)
      uint32_t getLength() const { return _length; }
      // Minimum pulse length ever seen in microseconds
      uint32_t getLengthMin() const { return _lengthMin; }
      // Maximum pulse length ever seen in microseconds
      uint32_t getLengthMax() const { return _lengthMax; }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const;
#endif

    private:
      gpio_num_t _pinZC = GPIO_NUM_NC;
      volatile State _state = State::IDLE;

      // time of the last edge
      uint32_t _lastEdge = 0;

      // measured pulse period
      uint32_t _period = 0;
      uint32_t _periodMin = 0;
      uint32_t _periodMax = 0;

      // measured pulse length
      uint32_t _length = 0;
      uint32_t _lengthMin = 0;
      uint32_t _lengthMax = 0;

      // recording
      uint32_t _times[MYCILA_PULSE_SAMPLES];
      size_t _index = 0;

      // simulation
      gpio_num_t _pinOutput = GPIO_NUM_NC;
      uint32_t _outputPulseLengthMicros = 1;

      static void recordISRStatic(void* arg);
      static void simulateISRStatic(void* arg);
  };
} // namespace Mycila
