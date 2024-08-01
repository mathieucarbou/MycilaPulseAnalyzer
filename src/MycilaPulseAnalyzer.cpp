// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPulseAnalyzer.h"
#include "Arduino.h"

#ifndef GPIO_IS_VALID_GPIO
  #define GPIO_IS_VALID_GPIO(gpio_num) ((gpio_num >= 0) && \
                                        (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0))
#endif

#ifndef GPIO_IS_VALID_OUTPUT_GPIO
  #define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) ((gpio_num >= 0) && \
                                               (((1ULL << (gpio_num)) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) != 0))
#endif

#ifdef MYCILA_JSON_SUPPORT
void Mycila::PulseAnalyzer::toJson(const JsonObject& root) const {
  root["state"] = static_cast<int>(_state);
  root["period"] = _period;
  root["period_min"] = _periodMin;
  root["period_max"] = _periodMax;
  root["frequency"] = getFrequency();
  root["length"] = _length;
  root["length_min"] = _lengthMin;
  root["length_max"] = _lengthMax;
}
#endif

void Mycila::PulseAnalyzer::end() {
  if (_state == State::IDLE)
    return;
  _state = State::IDLE;
  detachInterrupt(_pinZC);
}

void Mycila::PulseAnalyzer::record(int8_t pinZC) {
  if (!GPIO_IS_VALID_GPIO(pinZC)) {
    return;
  }

  if (_state != State::IDLE)
    return;

  _state = State::RECORDING;
  _pinZC = (gpio_num_t)pinZC;

  // reset
  _index = 0;
  for (size_t i = 0; i < MYCILA_PULSE_SAMPLES; i++)
    _times[i] = 0;

  // start
  attachInterruptArg(_pinZC, recordISRStatic, this, CHANGE);
}

void Mycila::PulseAnalyzer::simulate(int8_t pinZC, int8_t pinOutput, uint32_t outputPulseLengthMicros) {
  if (!GPIO_IS_VALID_GPIO(pinZC)) {
    return;
  }

  if (!GPIO_IS_VALID_OUTPUT_GPIO(pinOutput)) {
    return;
  }

  if (_state != State::IDLE)
    return;

  _state = State::SIMULATING;
  _outputPulseLengthMicros = outputPulseLengthMicros;
  _pinZC = (gpio_num_t)pinZC;
  _pinOutput = (gpio_num_t)pinOutput;
  pinMode(_pinOutput, OUTPUT);

  // start
  attachInterruptArg(_pinZC, simulateISRStatic, this, CHANGE);
}

void Mycila::PulseAnalyzer::analyze() {
  if (_state != State::RECORDED)
    return;

  size_t level;
  uint32_t len;

  // analyze pule length

  uint32_t sum[2] = {0, 0};
  uint32_t min[2] = {UINT32_MAX, UINT32_MAX};
  uint32_t max[2] = {0, 0};

  for (size_t i = 1; i < MYCILA_PULSE_SAMPLES; i++) {
    level = i % 2;
    len = _times[i] - _times[i - 1];
    sum[level] += len;
    if (len < min[level])
      min[level] = len;
    if (len > max[level])
      max[level] = len;
  }

  level = sum[0] < sum[1] ? 0 : 1;
  _length = sum[level] / (MYCILA_PULSE_SAMPLES / 2 - 1 + level);
  _lengthMin = min[level];
  _lengthMax = max[level];

  // analyze pulse period

  sum[0] = 0;
  min[0] = UINT32_MAX;
  max[0] = 0;

  for (size_t i = 2; i < MYCILA_PULSE_SAMPLES; i++) {
    len = _times[i] - _times[i - 2];
    sum[0] += len;
    if (len < min[0])
      min[0] = len;
    if (len > max[0])
      max[0] = len;
  }

  _period = sum[0] / (MYCILA_PULSE_SAMPLES - 2u);
  _periodMin = min[0];
  _periodMax = max[0];

  _analyzed = true;
}

void IRAM_ATTR Mycila::PulseAnalyzer::recordISRStatic(void* arg) {
  Mycila::PulseAnalyzer* self = (Mycila::PulseAnalyzer*)arg;

  if (self->_state != Mycila::PulseAnalyzer::State::RECORDING)
    return;

  uint32_t now = esp_timer_get_time();

  // Filter out spurious interrupts
  // See: https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection
  if (now - self->_lastEdge < MYCILA_MIN_PULSE_LENGTH_US)
    return;

  self->_lastEdge = now;

  if (self->_index < MYCILA_PULSE_SAMPLES) {
    self->_times[self->_index] = self->_lastEdge;
    self->_index = self->_index + 1;
  } else {
    detachInterrupt(self->_pinZC);
    if (self->_state == Mycila::PulseAnalyzer::State::RECORDING)
      self->_state = Mycila::PulseAnalyzer::State::RECORDED;
  }
}

void IRAM_ATTR Mycila::PulseAnalyzer::simulateISRStatic(void* arg) {
  Mycila::PulseAnalyzer* self = (Mycila::PulseAnalyzer*)arg;

  if (self->_state != State::SIMULATING) {
    detachInterrupt(self->_pinZC);
    return;
  }

  uint32_t now = esp_timer_get_time();

  // Filter out spurious interrupts
  // See: https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection
  if (now - self->_lastEdge < MYCILA_MIN_PULSE_LENGTH_US)
    return;

  self->_lastEdge = now;

  digitalWrite(self->_pinOutput, HIGH);
  delayMicroseconds(self->_outputPulseLengthMicros);
  digitalWrite(self->_pinOutput, LOW);
}
