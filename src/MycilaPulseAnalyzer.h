// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#pragma once

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <driver/gptimer_types.h>
#include <hal/gpio_types.h>

#define MYCILA_PULSE_VERSION          "2.3.1"
#define MYCILA_PULSE_VERSION_MAJOR    2
#define MYCILA_PULSE_VERSION_MINOR    3
#define MYCILA_PULSE_VERSION_REVISION 1

#ifndef MYCILA_PULSE_SAMPLES
  // sample count for analysis
  #define MYCILA_PULSE_SAMPLES 50
#endif

#ifndef MYCILA_PULSE_MIN_SEMI_PERIOD_US
  // semi-period check
  #define MYCILA_PULSE_MIN_SEMI_PERIOD_US 8000 // About 62.5 Hz
#endif
#ifndef MYCILA_PULSE_MAX_SEMI_PERIOD_US
  // semi-period check
  #define MYCILA_PULSE_MAX_SEMI_PERIOD_US 10400 // about 48 Hz
#endif

#ifndef MYCILA_PULSE_MIN_PULSE_WIDTH_US
  // pulse width filtering
  #define MYCILA_PULSE_MIN_PULSE_WIDTH_US 100
#endif
#ifndef MYCILA_PULSE_MAX_PULSE_WIDTH_US
  // pulse width filtering
  #define MYCILA_PULSE_MAX_PULSE_WIDTH_US MYCILA_PULSE_MAX_SEMI_PERIOD_US
#endif

#ifndef MYCILA_PULSE_ZC_SHIFT_US
  // Shift to apply when setting the zero-crossing timer.
  // By default the zero-crossing is set at the middle of the pulse.
  // This value will be added to determine the zero-crossing timer 0 position.
  // Example: +100 will shift the zero-crossing event 100us after the middle of the pulse.
  // This only apply to pulses, not the BM1Z102FJ
  #define MYCILA_PULSE_ZC_SHIFT_US 0
#endif

// #define MYCILA_PULSE_DEBUG

namespace Mycila {
  class PulseAnalyzer {
    public:
      typedef enum {
        SIGNAL_NONE = 0,
        // rising edge of a short pulse
        SIGNAL_RISING = 0x01, // RISING
        // falling edge of a short pulse
        SIGNAL_FALLING = 0x02, // FALLING
      } Event;

      typedef enum {
        TYPE_UNKNOWN = 0,
        TYPE_PULSE = 1,
        TYPE_BM1Z102FJ = 2,
      } Type;

      typedef void (*EventCallback)(Event event, void* arg);
      typedef void (*Callback)(void* arg);

      // Callback to be called when an edge is detected
      // Callback should be in IRAM (IRAM_ATTR) and do minimal work.
      // **MUST BE CALLED BEFORE begin()**
      void onEdge(EventCallback callback, void* arg = nullptr) {
        _onEdge = callback;
        _onEdgeArg = arg;
      }

      // Callback to be called when a zero-crossing is detected
      // Callback should be in IRAM (IRAM_ATTR) and do minimal work.
      // **MUST BE CALLED BEFORE begin()**
      void onZeroCross(Callback callback, void* arg = nullptr) {
        _onZeroCross = callback;
        _onZeroCrossArg = arg;
      }

      /**
       * @brief Start the analyzer
       * @param pinZC Zero-crossing pin
       *
       * @return true if the analyzer was started
       */
      bool begin(int8_t pinZC);

      /**
       * @brief Stop the analyzer
       */
      void end();

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const;
#endif

      // true if the analyzer is enabled and running
      bool isEnabled() const { return _pinZC != GPIO_NUM_NC; }

      // true if connected to the grid
      bool isOnline() const { return isEnabled() && _period > 0; }

      gpio_num_t getZCPin() const { return _pinZC; }

      // Pulse type detected: pulse or BM1Z102FJ
      Type getType() const { return _type; }

      // last event detected: rising or falling edge
      Event getLastEvent() const { return _lastEvent; }

      // Pulse period in microseconds (average of the last N samples)
      uint32_t getPeriod() const { return _period; }
      // Minimum pulse period ever seen in microseconds
      uint32_t getMinPeriod() const { return _periodMin; }
      // Maximum pulse period ever seen in microseconds
      uint32_t getMaxPeriod() const { return _periodMax; }

      // Pulse frequency in Hz
      uint32_t getFrequency() const { return _period ? 1000000 / _period : 0; }

      // Nominal grid period in microseconds
      uint32_t getNominalGridPeriod() const { return _nominalGridPeriod; }
      // Nominal grid frequency in Hz (50 Hz / 60 Hz)
      uint32_t getNominalGridFrequency() const { return _nominalGridPeriod ? 1000000 / _nominalGridPeriod : 0; }

      // Pulse width in microseconds (average of the last N samples)
      uint32_t getWidth() const { return _width; }
      // Minimum pulse width ever seen in microseconds
      uint32_t getMinWidth() const { return _widthMin; }
      // Maximum pulse width ever seen in microseconds
      uint32_t getMaxWidth() const { return _widthMax; }

    private:
      // ISR
      static bool _onlineTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg);
      static bool _zcTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg);
      static void _edgeISR(void* arg);

      gpio_num_t _pinZC = GPIO_NUM_NC;

      // timers
      gptimer_handle_t _onlineTimer = NULL;
      gptimer_handle_t _zcTimer = NULL;

      // recording
      uint32_t _widths[MYCILA_PULSE_SAMPLES];
      size_t _size = 0;
      Event _lastEvent = SIGNAL_NONE;
      Type _type = TYPE_UNKNOWN;

      // measured pulse period
      uint32_t _period = 0;
      uint32_t _periodMin = 0;
      uint32_t _periodMax = 0;

      // nominal values
      uint32_t _nominalGridPeriod = 0;

      // measured pulse width
      uint32_t _width = 0;
      uint32_t _widthMin = 0;
      uint32_t _widthMax = 0;

      // events
      EventCallback _onEdge = nullptr;
      void* _onEdgeArg = nullptr;
      Callback _onZeroCross = nullptr;
      void* _onZeroCrossArg = nullptr;
  };
} // namespace Mycila
