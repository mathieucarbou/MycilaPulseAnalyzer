// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#pragma once

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <driver/gptimer_types.h>
#include <hal/gpio_types.h>
#include <stddef.h>

#define MYCILA_PULSE_VERSION          "3.0.8"
#define MYCILA_PULSE_VERSION_MAJOR    3
#define MYCILA_PULSE_VERSION_MINOR    0
#define MYCILA_PULSE_VERSION_REVISION 8

// sample count for analysis
#define MYCILA_PULSE_SAMPLES 50

#ifndef MYCILA_PULSE_ZC_SHIFT_US
  // Shift to apply when setting the zero-crossing timer.
  // By default the zero-crossing is set at the middle of the pulse.
  // This value will be added to determine when the zero-crossing event wil be fired from the middle of the pulse.
  //
  // Example: -100 will shift the zero-crossing event -100 us before the middle of the pulse.
  //
  // This parameter is really important to use in the case of pulses that are not centered to 0, like Robodyn.
  // If the Zero-Cross event happens too late after the real 0-voltage is crossed,
  // triacs could stay on at 100% because they will see the pulse fire command.
  //
  // It could also be used for other purposes, like delay the ZC event for 5000 us so that the ZC event is always
  // triggered at the voltage peak (90 degree angle) of the sine wave.
  #define MYCILA_PULSE_ZC_SHIFT_US -150
#endif

#ifndef MYCILA_JSY_194_SIGNAL_SHIFT_US
  // Shift used to compensate the zero-crossing signal sent by the JSY-MK-194 boards.
  // The zero-crossing signal of the JSY is sent when the positive point is detected.
  // This value is used to compensate this delay and must be set to the time the zero-crossing signal is sent after the real zero-crossing.
  // Set it to -100 us for JSY-MK-194G.
  // Set it to -1000 us for JSY-MK-194T
  #define MYCILA_JSY_194_SIGNAL_SHIFT_US -100
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
        // Robodyn, ZCD from Daniel S, etc
        TYPE_SHORT = 1,
        // ZCD based on BM1Z102FJ chip
        TYPE_SEMI_PERIOD = 2,
        // ZCd based on RENERGY RN8209G like JSY-MK-194G
        TYPE_FULL_PERIOD = 3,
      } Type;

      typedef void (*EventCallback)(Event event, void* arg);

      // Callback to be called on Zero-Crossing event
      // The event is called at the supposedly real zero-crossing point, shifted by the value set with setZeroCrossEventShift()
      // delay will be the time from now, when the real zero-crossing is supposed to happen (or has happened).
      // For example, if MYCILA_PULSE_ZC_SHIFT_US is set to -150, the delay will be 150 us.
      typedef void (*Callback)(int16_t delay, void* arg);

      // Callback to be called when an edge is detected
      // Callback should be in IRAM (ARDUINO_ISR_ATTR) and do minimal work.
      // **MUST BE CALLED BEFORE begin()**
      void onEdge(EventCallback callback, void* arg = nullptr) {
        _onEdge = callback;
        _onEdgeArg = arg;
      }

      // Callback to be called when a zero-crossing is detected
      // Callback should be in IRAM (ARDUINO_ISR_ATTR) and do minimal work.
      // **MUST BE CALLED BEFORE begin()**
      void onZeroCross(Callback callback, void* arg = nullptr) {
        _onZeroCross = callback;
        _onZeroCrossArg = arg;
      }

      // shift to apply to apply before or after zero when to send the zero-crossing event, in us
      // Default to MYCILA_PULSE_ZC_SHIFT_US
      // Call before begin(), cannot be changed after.
      void setZeroCrossEventShift(uint16_t shift) { _shiftZC = shift; }

      // shift to apply to recenter the JSY zero-cross signal to zero, in us
      // Default to MYCILA_JSY_194_SIGNAL_SHIFT_US
      // Call before begin(), cannot be changed after.
      void setJSY194SignalShift(uint16_t shift) { _shiftJsySignal = shift; }

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

      // Pulse type detected
      Type getType() const { return _type; }

      // last event detected: rising or falling edge
      Event getLastEvent() const { return _lastEvent; }

      // Pulse period in microseconds (average of the last N samples)
      uint16_t getPeriod() const { return _period; }
      // Minimum pulse period ever seen in microseconds
      uint16_t getMinPeriod() const { return _periodMin; }
      // Maximum pulse period ever seen in microseconds
      uint16_t getMaxPeriod() const { return _periodMax; }

      // Pulse frequency in Hz
      uint8_t getFrequency() const { return _period ? 1000000 / _period : 0; }

      // Nominal grid semi-period in microseconds
      uint16_t getNominalGridSemiPeriod() const { return _nominalSemiPeriod; }
      // Nominal grid period in microseconds
      uint16_t getNominalGridPeriod() const { return _nominalSemiPeriod << 1; }
      // Nominal grid frequency in Hz (50 Hz / 60 Hz)
      uint8_t getNominalGridFrequency() const { return _nominalSemiPeriod ? 1000000 / (_nominalSemiPeriod << 1) : 0; }

      // Pulse width in microseconds (average of the last N samples)
      uint16_t getWidth() const { return _width; }
      // Minimum pulse width ever seen in microseconds
      uint16_t getMinWidth() const { return _widthMin; }
      // Maximum pulse width ever seen in microseconds
      uint16_t getMaxWidth() const { return _widthMax; }

    private:
      // ISR
      static bool _onlineTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg);
      static bool _zcTimerISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* arg);
      static void _edgeISR(void* arg);

      gpio_num_t _pinZC = GPIO_NUM_NC;

      // timers
      gptimer_handle_t _onlineTimer = nullptr;
      gptimer_handle_t _zcTimer = nullptr;

      // Internal ISR variables
      uint16_t _widths[MYCILA_PULSE_SAMPLES];
      size_t _size = 0;
      Event _lastEvent = SIGNAL_NONE;
      Type _type = TYPE_UNKNOWN;

      // measured pulse period
      uint16_t _period = 0;
      uint16_t _periodMin = 0;
      uint16_t _periodMax = 0;

      // nominal values
      uint16_t _nominalSemiPeriod = 0;

      // measured pulse width
      uint16_t _width = 0;
      uint16_t _widthMin = 0;
      uint16_t _widthMax = 0;

      // shift for ZC event
      int16_t _shiftZC = MYCILA_PULSE_ZC_SHIFT_US;
      int16_t _shiftJsySignal = MYCILA_JSY_194_SIGNAL_SHIFT_US;
      int16_t _shift = 0;

      // events
      EventCallback _onEdge = nullptr;
      void* _onEdgeArg = nullptr;
      Callback _onZeroCross = nullptr;
      void* _onZeroCrossArg = nullptr;
  };
} // namespace Mycila
