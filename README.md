# MycilaPulseAnalyzer

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Continuous Integration](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaPulseAnalyzer/actions/workflows/ci.yml)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/mathieucarbou/library/MycilaPulseAnalyzer.svg)](https://registry.platformio.org/libraries/mathieucarbou/MycilaPulseAnalyzer)

ESP32 / Arduino Library to analyze pulses from a Zero-Cross Detection circuit and manage the Zero-Cross pulse.

- [Features](#features)
- [Supported ZCD Circuits](#supported-zcd-circuits)
- [Usage](#usage)
- [IRAM Safety](#iram-safety)
- [Oscilloscope Views](#oscilloscope-views)
- [Use-Case: Thyristor TRIAC Control](#use-case-thyristor-triac-control)
- [Readings](#readings)

## Features

- Detect Zero-Cross pulse
- Ability to shift the Zero-Cross event (`MYCILA_PULSE_ZC_SHIFT_US`)
- Filter spurious Zero-Cross events (noise due to voltage detection)
- Online / Offline detection
- Uses only 2 timers
- **IRAM safe and supports concurrent flash operations!**
- Callbacks for:
  - Zero-Cross,
  - Rising Signal
  - Falling Signal
  - Signal Change (BM1Z102FJ)
- Measurements:
  - Period
  - Minimum Period
  - Maximum Period
  - Frequency
  - Pulse Width
  - Minimum Pulse Width
  - Maximum Pulse Width

## Supported ZCD Circuits

- Square short pulse (e.g. [Zero-Cross Detector from Daniel](https://www.pcbway.com/project/shareproject/Zero_Cross_Detector_a707a878.html))

- Slope pulse (e.g. [Robodyn ZCD](https://fr.aliexpress.com/item/1005006211999051.html))

- High precision pulse matching AC wave +/- components based on BM1Z102FJ chip (e.g. [AC VOLTAGE ZERO CROSS DETECTOR](https://www.electronics-lab.com/project/ac-voltage-zero-cross-detector/) - note that this one needs an [additional optocoupler](https://www.youtube.com/watch?v=1-9yDTj2IQw&lc=UgzWwX5jGTsKvb3e09t4AaABAg.9Vk7pMApNK39VmUSzBJooq) like TLP2630 or 6N136)

More hardware are supported, as long as they fall into one of these categories above.

| ![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_ZCD.jpeg) | ![](https://yasolr.carbou.me/assets/img/measurements/Oscillo_ZCD_Robodyn.jpeg) | ![](https://www.electronics-lab.com/wp-content/uploads/2021/09/Output-Delay-Setting-DSET-Pin-Setting-Resistor-R7.jpg) |

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

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pinMode(PIN_OUTPUT, OUTPUT);

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

## IRAM Safety

You can run the app with:

```
-D CONFIG_ARDUINO_ISR_IRAM=1
-D CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=1
-D CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=1
-D CONFIG_GPTIMER_ISR_IRAM_SAFE=1
-D CONFIG_GPIO_CTRL_FUNC_IN_IRAM=1
```

This will improve interrupt reliability (they will continue working even during flash operation).

MycilaPulse makes use of inline function of HAL layer and `ARDUINO_ISR_ATTR` to ensure that the interrupt handlers are in IRAM.

You can look at teh examples in the project to see how to use this library with IRAM safety.

## Oscilloscope Views

Here are below some oscilloscope views of 2 ZCD behaviors with a pulse sent from an ESP32 pin to display the received events.

- In yellow: the ZC pulse
- In blue: the output pin pulse of 1 us
- In red: main AC voltage
- Measured in pink: distance between the vertical lines

**Robodyn**

Here are some views of the Robodyn ZC pulse, when adding a 1 us pulse on each event: rising and falling, and a 1us pulse on the ZC event.

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/robodyn_zc.jpeg)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/robodyn_zc.jpeg)

Here is the same view, but after applying a shift of about 100 us to the ZC event:

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/robodyn_zc_delay.jpeg)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/robodyn_zc_delay.jpeg)

**Zero-Cross Detector from Daniel S.**

Here are some views of the ZC pulse, when adding a 1 us pulse on each event: rising and falling, and a 1us pulse on the ZC event.

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/zcd_zc.jpeg)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/zcd_zc.jpeg)

Here is the same view, but after applying a shift of about 100 us to the ZC event:

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/zcd_zc_delay.jpeg)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/zcd_zc_delay.jpeg)

## Use-Case: Thyristor TRIAC Control

You can look at the example in the project how to use this library to control a Thyristor / TRIAC with a zero-cross detection circuit.

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/thyristor.gif)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/thyristor.gif)

- In yellow: the ZC pulse
- In blue: the output pin pulse of 1 us
- In red: main AC voltage
- In pink: current going to the load

## Readings

Here is a blog post explaining how to use this library, with some oscilloscope screenshots:

[https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection](https://yasolr.carbou.me/blog/2024-07-31_zero-cross_pulse_detection).

[![](https://oss.carbou.me/MycilaPulseAnalyzer/assets/Oscillo_zc_isr_output_delay.jpeg)](https://oss.carbou.me/MycilaPulseAnalyzer/assets/Oscillo_zc_isr_output_delay.jpeg)
