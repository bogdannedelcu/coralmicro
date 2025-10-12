# Power Profiling — Coral Dev Board Micro + Dual OV5640 + EdgeTPU

## 🔍 Overview
This document summarizes the power consumption measurements and runtime behavior of the **Coral Dev Board Micro** (NXP i.MX RT1176) when running **EdgeTPU inference locally** with two **OV5640** cameras attached.

The goal of this test was to **minimize total power draw** during inference by disabling all unnecessary peripherals (USB, network, serial output) and measure the **baseline vs. active current**.

---

## ⚙️ Test Setup

| Component | Description |
|------------|-------------|
| **Board** | Coral Dev Board Micro (i.MX RT1176, EdgeTPU) |
| **Cameras** | 2 × OV5640 connected via CSI |
| **Model** | Custom EdgeTPU detection model |
| **Measurement** | Power supply: 5.00 V regulated / Current meter inline |
| **Interfaces** | USB, WiFi, and Ethernet **disabled** |
| **Mode** | Inference performed locally, no data transfer to host |

---

## ⚡ Measured Power Draw

| System State | Active Components | Current (A) | Power (W @ 5 V) | Notes |
|---------------|------------------|-------------|------------------|-------|
| **Idle** | CPU + basic peripherals only | **0.157 A** | **0.785 W** | Cameras + TPU off |
| **Inference** | 2× cameras + EdgeTPU + CPU | **0.260 A** | **1.30 W** | Running full inference locally |
| **Δ Active vs Idle** | — | **+0.103 A** | **+0.515 W** | Overhead from cameras + TPU |

### 🔹 Interpretation
- Total consumption during active detection ≈ **1.3 W**, which is low for dual-camera EdgeTPU processing.
- Enabling cameras and TPU adds ~0.5 W over idle.
- EdgeTPU inference duration (for lightweight detection models) is **under 100 ms**, showing efficient processing.

---

## 🧠 Optimization Notes
- USB and networking stacks were explicitly powered down to prevent background current draw.
- Serial output minimized to reduce UART and CPU activity.
- Cameras are powered and clocked only during capture periods.
- EdgeTPU remains off or clock-gated between detections when possible.

---

## ✅ Conclusions
- The Coral Dev Board Micro can sustain **dual-camera EdgeTPU inference** at ~1.3 W total power.
- Difference between idle and active states is small (≈0.5 W), demonstrating excellent hardware efficiency.
- System is suitable for **battery-powered edge AI** and **low-duty-cycle IoT inference**.

---

## 🔄 Next Steps
- Implement **GPIO-controlled sleep mode** for cameras and EdgeTPU to reduce idle current below **0.1 A**.
- Profile **deep sleep entry/exit times** using the NXP `LPM_EnterSleepMode()` API.
- Automate power logging via INA219 / INA260 sensor on the 5 V rail.
- Correlate measured current spikes with camera DMA and TPU workload in a timing trace.

---

**Author:** Bogdan Nedelcu  
**Date:** October 2025  
**Commit:** [`cd0a5114203a52f817978153d6a6bb37d9e12a2a`](https://github.com/bogdannedelcu/coralmicro/commit/cd0a5114203a52f817978153d6a6bb37d9e12a2a)


# BN Fork of Coral Dev Board Micro source code (coralmicro)

# Coral Dev Board Micro source code (coralmicro)

This repository contains all the code required to build apps for the [Coral Dev
Board Micro](https://coral.ai/products/dev-board-micro). The Dev Board Micro is
based on the NXP RT1176 microcontroller (dual-core MCU with Cortex M7 and M4)
and includes an on-board camera (324x324 px), a microphone, and a Coral Edge TPU
to accelerate TensorFlow Lite models.

The software platform for Dev Board Micro is called `coralmicro` and is based
on [FreeRTOS](https://www.freertos.org/). It also includes libraries for
compatibility with the Arduino programming language.

The `coralmicro` build system is based on CMake and includes support for Make
and Ninja builds. After you build the included projects, you can flash
them to your board with the included flashtool (`scripts/flashtool.py`).

![main](https://github.com/google-coral/coralmicro/actions/workflows/ci.yml/badge.svg?event=push)
![arduino](https://github.com/google-coral/coralmicro/actions/workflows/arduino.yml/badge.svg?event=push)


## Documentation

+ [Get Started with the Dev Board Micro](https://coral.ai/docs/dev-board-micro/get-started/)

+ [Get Started with Arduino](https://coral.ai/docs/dev-board-micro/arduino/)

+ [Build an out-of-tree project](https://github.com/google-coral/coralmicro-out-of-tree-sample/blob/main/README.md)

+ [coralmicro API reference](http://coral.ai/docs/reference/micro/)

+ [coralmicro examples](/examples/)



## Get the code

1. Clone `coralmicro` and all submodules:

    ```bash
    git clone --recurse-submodules -j8 https://github.com/google-coral/coralmicro
    ```

2. Install the required tools:

    ```bash
    cd coralmicro && bash setup.sh
    ```


## Build the code

This builds everything in a folder called `build` (or you can specify a
different path with `-b`, but if you do then you must also specify that path
everytime you call `flashtool.py`):

```bash
bash build.sh
```

## Flash the board

This example blinks the board's green LED:

```bash
python3 scripts/flashtool.py -e blink_led
```

You can see the code at [examples/blink_led/](examples/blink_led/).


### Reset the board to Serial Downloader

Flashing the Dev Board Micro might fail sometimes and you can usually solve
it by starting Serial Downloader mode in one of two ways:

+ Hold the User button while you press the Reset button.
+ Or, hold the User button while you plug in the USB cable.

Then try flashing the board again.

For more details, see the [troubleshooting info on
coral.ai](https://coral.ai/docs/dev-board-micro/get-started/#serial-downloader).


## Update the repo

Use the following commands to keep all coralmicro submodules in sync (rebasing your current branch):

```bash
git fetch origin

git rebase origin/main

git submodule update --init --recursive
```

