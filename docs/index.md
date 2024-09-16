# MycilaPulseAnalyzer

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Continuous Integration](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/mathieucarbou/library/MycilaPulseAnalyzer.svg)](https://registry.platformio.org/libraries/mathieucarbou/MycilaPulseAnalyzer)

ESP32 / Arduino Library to analyze pulses from a Zero-Cross Detection circuit

## Supported ZCD Circuits

- Square short pulse (e.g. [Zero-Cross Detector from Daniel](https://www.pcbway.com/project/shareproject/Zero_Cross_Detector_a707a878.html))

- Slope pulse (e.g. [Robodyn ZCD](https://fr.aliexpress.com/item/1005006211999051.html))

- High precision pulse matching AC wave +/- components based on BM1Z102FJ chip (e.g. [AC VOLTAGE ZERO CROSS DETECTOR](https://www.electronics-lab.com/project/ac-voltage-zero-cross-detector/) - note that this one needs an [additional optocoupler](https://www.youtube.com/watch?v=1-9yDTj2IQw&lc=UgzWwX5jGTsKvb3e09t4AaABAg.9Vk7pMApNK39VmUSzBJooq) like TLP2630 or 6N136)

| ![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_ZCD.jpeg) | ![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_ZCD_Robodyn.jpeg) | ![](https://www.electronics-lab.com/wp-content/uploads/2021/09/Output-Delay-Setting-DSET-Pin-Setting-Resistor-R7.jpg) |

More hardware are supported, as long as they fall into one of these categories above.

## Usage

```cpp
Mycila::PulseAnalyzer pulseAnalyzer;

static void ARDUINO_ISR_ATTR onEdge(Mycila::PulseAnalyzer::Event e, void* arg) {
  if (e == Mycila::PulseAnalyzer::Event::SIGNAL_RISING || e == Mycila::PulseAnalyzer::Event::SIGNAL_FALLING) {
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(OUTPUT_WIDTH_US);
    digitalWrite(PIN_OUTPUT, LOW);
  }
}

static void ARDUINO_ISR_ATTR onZeroCross(void* arg) {
  digitalWrite(PIN_OUTPUT, HIGH);
  delayMicroseconds(OUTPUT_WIDTH_US);
  digitalWrite(PIN_OUTPUT, LOW);
}

static void ARDUINO_ISR_ATTR onOffline(void* arg) {
  ets_printf("offline\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

  pulseAnalyzer.onOffline(onOffline);
  pulseAnalyzer.onEdge(onEdge);
  pulseAnalyzer.onZeroCross(onZeroCross);

  pulseAnalyzer.begin(35);
}
```

Output:

```json
{"state":0,"period":9994,"period_min":9975,"period_max":10014,"frequency":100.0600357,"width":1168,"width_min":1154,"width_max":1182}
{"state":0,"period":9993,"period_min":9976,"period_max":10018,"frequency":100.0700455,"width":1166,"width_min":1154,"width_max":1180}
```

Here is a blog post explaining how to use this library, with some oscilloscope screenshots:

[https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection](https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection).

[![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_zc_isr_output_delay.jpeg)](https://yasolr.carbou.me/assets/img/measurements/Oscillo_zc_isr_output_delay.jpeg)

- In yellow: the ZC pulse
- In blue: the output pin pulse of 1ms
- Measured in pink: distance between the ZC pulse and the output pin pulse (which is the delay taken by the ESP to detect the pulse edge and send a pulse on the output pin)
