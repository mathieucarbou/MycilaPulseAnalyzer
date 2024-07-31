# MycilaPulseAnalyzer

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Continuous Integration](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/mathieucarbou/library/MycilaPulseAnalyzer.svg)](https://registry.platformio.org/libraries/mathieucarbou/MycilaPulseAnalyzer)

ESP32 / Arduino Library to analyze pulses from a Zero-Cross Detection circuit

## Usage

### Analyze pulse period and length:

```cpp
pulseAnalyzer.record(35);

// wait for the pulses to be recorded
while (pulseAnalyzer.getState() != Mycila::PulseAnalyzer::State::RECORDED)
  delay(100);

pulseAnalyzer.analyze();

// wait for the pulses to be analyzed
while (pulseAnalyzer.getState() != Mycila::PulseAnalyzer::State::ANALYZED)
  delay(100);

pulseAnalyzer.end();

// print the results
JsonDocument doc;
pulseAnalyzer.toJson(doc.to<JsonObject>());
serializeJson(doc, Serial);
Serial.println();
```

Output:

```json
{"state":0,"period":9994,"period_min":9975,"period_max":10014,"frequency":100.0600357,"length":1168,"length_min":1154,"length_max":1182}
{"state":0,"period":9993,"period_min":9976,"period_max":10018,"frequency":100.0700455,"length":1166,"length_min":1154,"length_max":1180}
```

### Oscilloscope analysis of the Zero-Cross Detection

[![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_zc_isr_output_delay.jpeg)](https://yasolr.carbou.me/assets/img/measurements/Oscillo_zc_isr_output_delay.jpeg)

```cpp
// GPIO 35: ZCD input
// GPIO 26: Output
// Connect both pins to an oscilloscope to see the output pulse
pulseAnalyzer.simulate(35, 26, 1);
```

Here is a blog post explaining how to use this library, with some oscilloscope screenshots:

[https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection](https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection).
