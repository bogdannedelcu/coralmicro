/** @file t5838.h
 *
 * @brief T5838 AAD (Always Active Detection) mode driver for i.MX RT1176 / FreeRTOS.
 *
 * Adapted from IRNAS Zephyr driver (t5838_aad_mode.c/.h) to use NXP FSL GPIO
 * drivers and FreeRTOS primitives.
 *
 * Protocol note: T5838 uses a proprietary "fake2C" two-wire protocol over
 * THSEL (data) + PDMCLK (clock) for AAD register configuration.
 *
 * Fixed pin assignments (configured automatically by Init()):
 *   THSEL  = GPIO_SD_B1_03 → GPIO_MUX4_IO06 → GPIO4, pin 6
 *   PDMCLK = GPIO_LPSR_00  → GPIO_MUX6_IO00 → GPIO6, pin 0
 *            (normal PDM function: IOMUXC_GPIO_LPSR_00_MIC_CLK, mux mode 1)
 *            (bitbang GPIO mode:   IOMUXC_GPIO_LPSR_00_GPIO_MUX6_IO00, mux mode 5)
 *
 * Usage:
 *   1. Stop the PDM AudioDriver before calling AadXModeSet().
 *   2. Call AadXModeSet() — IOMUXC switch to GPIO is done internally.
 *   3. Call RestorePdmClk() then restart the PDM AudioDriver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2023 Irnas. All rights reserved.
 * Adapted for coralmicro / i.MX RT1176 platform.
 */

#ifndef LIBS_T5838_T5838_H_
#define LIBS_T5838_T5838_H_

#include <cstdbool>
#include <cstdint>

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/MIMXRT1176_cm7.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"

namespace coralmicro {

/* ------------------------------------------------------------------ */
/* Register addresses                                                   */
/* ------------------------------------------------------------------ */
static constexpr uint8_t kT5838RegAadMode              = 0x29;
static constexpr uint8_t kT5838RegAadDFloorHi          = 0x2A;
static constexpr uint8_t kT5838RegAadDFloorLo          = 0x2B;
static constexpr uint8_t kT5838RegAadDRelPulseMinLo    = 0x2E;
static constexpr uint8_t kT5838RegAadDAbsRelPulseMinSh = 0x2F;
static constexpr uint8_t kT5838RegAadDAbsPulseMinLo    = 0x30;
static constexpr uint8_t kT5838RegAadDAbsThrLo         = 0x31;
static constexpr uint8_t kT5838RegAadDAbsThrHi         = 0x32;
static constexpr uint8_t kT5838RegAadDRelThr           = 0x33;
static constexpr uint8_t kT5838RegAadALpf              = 0x35;
static constexpr uint8_t kT5838RegAadAThr              = 0x36;

/* ------------------------------------------------------------------ */
/* Enumerations                                                         */
/* ------------------------------------------------------------------ */

enum T5838AadSelect : uint8_t {
    kT5838AadSelectNone = 0x00,
    kT5838AadSelectD1   = 0x01,
    kT5838AadSelectD2   = 0x02,
    kT5838AadSelectA    = 0x08,
};

enum T5838AadALpf : uint8_t {
    kT5838AadALpf4_4kHz = 0x01,
    kT5838AadALpf2_0kHz = 0x02,
    kT5838AadALpf1_9kHz = 0x03,
    kT5838AadALpf1_8kHz = 0x04,
    kT5838AadALpf1_6kHz = 0x05,
    kT5838AadALpf1_3kHz = 0x06,
    kT5838AadALpf1_1kHz = 0x07,
};

enum T5838AadAThr : uint8_t {
    kT5838AadAThr60dB   = 0x00,
    kT5838AadAThr65dB   = 0x02,
    kT5838AadAThr70dB   = 0x04,
    kT5838AadAThr75dB   = 0x06,
    kT5838AadAThr80dB   = 0x08,
    kT5838AadAThr85dB   = 0x0A,
    kT5838AadAThr90dB   = 0x0C,
    kT5838AadAThr95dB   = 0x0E,
    kT5838AadAThr97_5dB = 0x0F,
};

enum T5838AadDAbsThr : uint16_t {
    kT5838AadDAbsThr40dB = 0x000F,
    kT5838AadDAbsThr45dB = 0x0016,
    kT5838AadDAbsThr50dB = 0x0032,
    kT5838AadDAbsThr55dB = 0x0037,
    kT5838AadDAbsThr60dB = 0x005F,
    kT5838AadDAbsThr65dB = 0x00A0,
    kT5838AadDAbsThr70dB = 0x0113,
    kT5838AadDAbsThr75dB = 0x01E0,
    kT5838AadDAbsThr80dB = 0x0370,
    kT5838AadDAbsThr85dB = 0x062C,
    kT5838AadDAbsThr87dB = 0x07BC,
};

enum T5838AadDRelThr : uint8_t {
    kT5838AadDRelThr3dB  = 0x24,
    kT5838AadDRelThr6dB  = 0x36,
    kT5838AadDRelThr9dB  = 0x48,
    kT5838AadDRelThr12dB = 0x64,
    kT5838AadDRelThr15dB = 0x8F,
    kT5838AadDRelThr18dB = 0xCA,
    kT5838AadDRelThr20dB = 0xFF,
};

enum T5838AadDFloor : uint16_t {
    kT5838AadDFloor40dB = 0x000F,
    kT5838AadDFloor45dB = 0x0016,
    kT5838AadDFloor50dB = 0x0032,
    kT5838AadDFloor55dB = 0x0037,
    kT5838AadDFloor60dB = 0x005F,
    kT5838AadDFloor65dB = 0x00A0,
    kT5838AadDFloor70dB = 0x0113,
    kT5838AadDFloor75dB = 0x01E0,
    kT5838AadDFloor80dB = 0x0370,
    kT5838AadDFloor85dB = 0x062C,
    kT5838AadDFloor87dB = 0x07BC,
};

enum T5838AadDRelPulseMin : uint16_t {
    kT5838AadDRelPulseMin0_7ms = 0x0000,
    kT5838AadDRelPulseMin10ms  = 0x0064,
    kT5838AadDRelPulseMin19ms  = 0x00C8,
    kT5838AadDRelPulseMin29ms  = 0x012C,
};

enum T5838AadDAbsPulseMin : uint16_t {
    kT5838AadDAbsPulseMin1_1ms = 0x0000,
    kT5838AadDAbsPulseMin10ms  = 0x0064,
    kT5838AadDAbsPulseMin19ms  = 0x00C8,
    kT5838AadDAbsPulseMin29ms  = 0x012C,
    kT5838AadDAbsPulseMin48ms  = 0x01F4,
    kT5838AadDAbsPulseMin95ms  = 0x03E8,
    kT5838AadDAbsPulseMin188ms = 0x07D0,
    kT5838AadDAbsPulseMin282ms = 0x0BB8,
    kT5838AadDAbsPulseMin328ms = 0x0DAC,
};

/* ------------------------------------------------------------------ */
/* Configuration structures                                             */
/* ------------------------------------------------------------------ */

struct T5838AadAConf {
    T5838AadALpf lpf;
    T5838AadAThr thr;
};

struct T5838AadDConf {
    T5838AadDFloor       floor;
    T5838AadDRelPulseMin rel_pulse_min;
    T5838AadDAbsPulseMin abs_pulse_min;
    T5838AadDAbsThr      abs_thr;
    T5838AadDRelThr      rel_thr;
};

/* ------------------------------------------------------------------ */
/* Driver class                                                         */
/* ------------------------------------------------------------------ */

/**
 * T5838 Always Active Detection (AAD) mode driver.
 *
 * All hardware pins are fixed and configured automatically by Init().
 * Thread-safety: not internally synchronised — caller must ensure
 * mutual exclusion with the AudioDriver (PDM must be stopped).
 */
class T5838Aad {
 public:
    /** Initialise GPIO pins. Must be called once before any mode-set function. */
    bool Init();

    /** Configure device into AAD A mode. PDM must be stopped before calling. */
    bool AadAModeSet(const T5838AadAConf& conf);

    /** Configure device into AAD D1 mode. PDM must be stopped before calling. */
    bool AadD1ModeSet(const T5838AadDConf& conf);

    /** Configure device into AAD D2 mode. PDM must be stopped before calling. */
    bool AadD2ModeSet(const T5838AadDConf& conf);

    /** Disable AAD mode. Call RestorePdmClk() then restart AudioDriver after. */
    bool AadModeDisable();

    /**
     * Restore PDMCLK pad to MIC_CLK function (IOMUXC_GPIO_LPSR_00_MIC_CLK).
     * Call after AadXModeSet() / AadModeDisable() before restarting AudioDriver.
     */
    void RestorePdmClk();

    /**
     * Clock the device for >2ms to enter low-power AAD sleep.
     * Called automatically inside AadXModeSet(). Only call manually if needed.
     */
    void AadSleep();

 private:
    struct AddrDataPair {
        uint8_t addr;
        uint8_t data;
    };

    void ClockBitbang(uint16_t cycles, uint16_t half_period_us);
    bool RegWrite(uint8_t reg, uint8_t data);
    bool MultiRegWrite(const AddrDataPair* pairs, size_t count);
    bool AadUnlockSequence();
    bool AadModeSet(const AddrDataPair* write_array, size_t count);
    void SwitchClkToGpio();

    /* THSEL:  GPIO_SD_B1_03 → GPIO_MUX4_IO06 → GPIO4, pin 6 */
    static GPIO_Type* const kThselBase;
    static constexpr uint32_t kThselPin = 6u;

    /* PDMCLK: GPIO_LPSR_00 → GPIO_MUX6_IO00 → GPIO6, pin 0 */
    static GPIO_Type* const kClkBase;
    static constexpr uint32_t kClkPin = 0u;

    bool           initialized_  = false;
    bool           aad_unlocked_ = false;
    T5838AadSelect aad_mode_     = kT5838AadSelectNone;
};

}  // namespace coralmicro

#endif  // LIBS_T5838_T5838_H_
