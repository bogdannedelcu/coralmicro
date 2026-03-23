/** @file t5838.cc
 *
 * @brief T5838 AAD mode driver — adapted from IRNAS Zephyr driver for
 *        i.MX RT1176 / FreeRTOS (coralmicro platform).
 *
 * Fixed pin assignments:
 *   THSEL  = GPIO_SD_B1_03 → IOMUXC_GPIO_SD_B1_03_GPIO_MUX4_IO06 → GPIO4, pin 6
 *   PDMCLK = GPIO_LPSR_00  → IOMUXC_GPIO_LPSR_00_MIC_CLK (normal, mux mode 1)
 *                           → IOMUXC_GPIO_LPSR_00_GPIO_MUX6_IO00 (bitbang, mux mode 5)
 *                           → GPIO6, pin 0
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2023 Irnas. All rights reserved.
 * Adapted for coralmicro / i.MX RT1176 platform.
 */

#include "libs/t5838/t5838.h"

#include <cstdio>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_common.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_iomuxc.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/system_MIMXRT1176_cm7.h"

namespace coralmicro {

/* ------------------------------------------------------------------ */
/* Protocol constants (identical to IRNAS reference)                    */
/* ------------------------------------------------------------------ */
static constexpr uint16_t kFake2cStartPilotClks  = 10;
static constexpr uint16_t kFake2cZero            = 1 * kFake2cStartPilotClks;
static constexpr uint16_t kFake2cOne             = 3 * kFake2cStartPilotClks;
static constexpr uint16_t kFake2cStop            = 130; // >128 clk cycles per datasheet
static constexpr uint16_t kFake2cSpace           = 1 * kFake2cStartPilotClks;
static constexpr uint16_t kFake2cPostWriteCycles = 60;  // >50 clk cycles per datasheet
static constexpr uint16_t kFake2cPreWriteCycles  = 60;  // >50 clk cycles per datasheet
static constexpr uint8_t  kFake2cDeviceAddress   = 0x53;
static constexpr uint16_t kFake2cClkHalfPeriodUs = 5;   // half-period → ~100 kHz clock

/* >2ms of clocking required before entering AAD sleep */
static constexpr uint32_t kEnterSleepClockingUs   = 2500;
static constexpr uint16_t kEnterSleepHalfPeriodUs = 5;  // ~100 kHz


/* ------------------------------------------------------------------ */
/* Static member definitions                                            */
/* ------------------------------------------------------------------ */
/* THSEL:  GPIO_SD_B1_03 → GPIO_MUX4_IO06 → GPIO4, pin 6 */
GPIO_Type* const T5838Aad::kThselBase = GPIO4;

/* PDMCLK: GPIO_LPSR_00 → GPIO_MUX6_IO00 → GPIO6, pin 0 */
GPIO_Type* const T5838Aad::kClkBase = GPIO6;

/* ------------------------------------------------------------------ */
/* Private helpers                                                      */
/* ------------------------------------------------------------------ */

/** Switch PDMCLK pad to GPIO output mode for bitbang. */
void T5838Aad::SwitchClkToGpio() {
    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_00_GPIO_MUX6_IO00, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_00_GPIO_MUX6_IO00, 0x00U);
    const gpio_pin_config_t out_low = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    GPIO_PinInit(kClkBase, kClkPin, &out_low);
}

void T5838Aad::ClockBitbang(uint16_t cycles, uint16_t half_period_us) {
    for (uint16_t i = 0; i < cycles; ++i) {
        GPIO_PinWrite(kClkBase, kClkPin, 1);
        SDK_DelayAtLeastUs(half_period_us, SystemCoreClock);
        GPIO_PinWrite(kClkBase, kClkPin, 0);
        SDK_DelayAtLeastUs(half_period_us, SystemCoreClock);
    }
}

bool T5838Aad::RegWrite(uint8_t reg, uint8_t data) {
    const uint8_t wr_buf[3] = {
        static_cast<uint8_t>(kFake2cDeviceAddress << 1),
        reg,
        data
    };

    GPIO_PinWrite(kThselBase, kThselPin, 0);
    ClockBitbang(kFake2cPreWriteCycles, kFake2cClkHalfPeriodUs);

    GPIO_PinWrite(kThselBase, kThselPin, 1);
    ClockBitbang(kFake2cStartPilotClks, kFake2cClkHalfPeriodUs);

    GPIO_PinWrite(kThselBase, kThselPin, 0);
    ClockBitbang(kFake2cSpace, kFake2cClkHalfPeriodUs);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            const uint16_t cyc = (wr_buf[i] & (1u << (7 - j)))
                                 ? kFake2cOne : kFake2cZero;
            GPIO_PinWrite(kThselBase, kThselPin, 1);
            ClockBitbang(cyc, kFake2cClkHalfPeriodUs);
            GPIO_PinWrite(kThselBase, kThselPin, 0);
            ClockBitbang(kFake2cSpace, kFake2cClkHalfPeriodUs);
        }
    }

    GPIO_PinWrite(kThselBase, kThselPin, 1);
    ClockBitbang(kFake2cStop, kFake2cClkHalfPeriodUs);

    GPIO_PinWrite(kThselBase, kThselPin, 0);
    ClockBitbang(kFake2cPostWriteCycles, kFake2cClkHalfPeriodUs);

    return true;
}

bool T5838Aad::MultiRegWrite(const AddrDataPair* pairs, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (!RegWrite(pairs[i].addr, pairs[i].data)) {
            printf("[T5838 AAD] RegWrite failed: addr=0x%02X data=0x%02X\r\n",
                   pairs[i].addr, pairs[i].data);
            return false;
        }
    }
    return true;
}

bool T5838Aad::AadUnlockSequence() {
    static constexpr AddrDataPair kUnlock[] = {
        {0x5C, 0x00}, {0x3E, 0x00}, {0x6F, 0x00}, {0x3B, 0x00}, {0x4C, 0x00},
    };
    if (!MultiRegWrite(kUnlock, sizeof(kUnlock) / sizeof(kUnlock[0]))) {
        printf("[T5838 AAD] Unlock sequence failed\r\n");
        return false;
    }
    aad_unlocked_ = true;
    return true;
}

bool T5838Aad::AadModeSet(const AddrDataPair* write_array, size_t count) {
    bool ok = true;
    taskENTER_CRITICAL();
    if (!aad_unlocked_) {
        ok = AadUnlockSequence();
    }
    if (ok) {
        ok = MultiRegWrite(write_array, count);
    }
    if (ok) {
        AadSleep();
        GPIO_PinWrite(kClkBase, kClkPin, 0);
    }
    taskEXIT_CRITICAL();
    return ok;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

bool T5838Aad::Init() {
    /* THSEL: configure IOMUXC and GPIO */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_GPIO_MUX4_IO06, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_GPIO_MUX4_IO06, 0x00U);
    const gpio_pin_config_t out_low = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    GPIO_PinInit(kThselBase, kThselPin, &out_low);

    /* PDMCLK: switch to GPIO mode for initial state */
    SwitchClkToGpio();

    initialized_  = true;
    aad_unlocked_ = false;
    aad_mode_     = kT5838AadSelectNone;
    return true;
}

void T5838Aad::RestorePdmClk() {
    /* Restore GPIO_LPSR_00 to MIC_CLK function for the PDM peripheral */
    IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_00_MIC_CLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_LPSR_00_MIC_CLK, 0x00U);
}

void T5838Aad::AadSleep() {
    const uint32_t cycles =
        kEnterSleepClockingUs / (2u * kEnterSleepHalfPeriodUs);
    ClockBitbang(static_cast<uint16_t>(cycles), kEnterSleepHalfPeriodUs);
}

bool T5838Aad::AadAModeSet(const T5838AadAConf& conf) {
    SwitchClkToGpio();
    const AddrDataPair write_data[] = {
        {kT5838RegAadMode, 0x00},
        {kT5838RegAadALpf, static_cast<uint8_t>(conf.lpf)},
        {kT5838RegAadAThr, static_cast<uint8_t>(conf.thr)},
        {kT5838RegAadMode, static_cast<uint8_t>(kT5838AadSelectA)},
    };
    if (!AadModeSet(write_data, sizeof(write_data) / sizeof(write_data[0]))) {
        printf("[T5838 AAD] AadAModeSet failed\r\n");
        return false;
    }
    aad_mode_ = kT5838AadSelectA;
    return true;
}

bool T5838Aad::AadD1ModeSet(const T5838AadDConf& conf) {
    SwitchClkToGpio();
    const uint16_t abs_thr       = static_cast<uint16_t>(conf.abs_thr);
    const uint16_t floor         = static_cast<uint16_t>(conf.floor);
    const uint16_t rel_pulse_min = static_cast<uint16_t>(conf.rel_pulse_min);
    const uint16_t abs_pulse_min = static_cast<uint16_t>(conf.abs_pulse_min);

    const AddrDataPair write_data[] = {
        {kT5838RegAadMode,              0x00},
        {kT5838RegAadDFloorHi,          static_cast<uint8_t>((floor >> 8) & 0x1F)},
        {kT5838RegAadDFloorLo,          static_cast<uint8_t>(floor & 0xFF)},
        {0x2C,                          0x32},
        {0x2D,                          0xC0},
        {kT5838RegAadDRelPulseMinLo,    static_cast<uint8_t>(rel_pulse_min & 0xFF)},
        {kT5838RegAadDAbsRelPulseMinSh, static_cast<uint8_t>(
                                            ((abs_pulse_min >> 4) & 0xF0) |
                                            ((rel_pulse_min >> 8) & 0x0F))},
        {kT5838RegAadDAbsPulseMinLo,    static_cast<uint8_t>(abs_pulse_min & 0xFF)},
        {kT5838RegAadDAbsThrLo,         static_cast<uint8_t>(abs_thr & 0xFF)},
        {kT5838RegAadDAbsThrHi,         static_cast<uint8_t>(((abs_thr >> 8) & 0x1F) | 0x40)},
        {kT5838RegAadDRelThr,           static_cast<uint8_t>(conf.rel_thr)},
        {kT5838RegAadMode,              static_cast<uint8_t>(kT5838AadSelectD1)},
    };
    if (!AadModeSet(write_data, sizeof(write_data) / sizeof(write_data[0]))) {
        printf("[T5838 AAD] AadD1ModeSet failed\r\n");
        return false;
    }
    aad_mode_ = kT5838AadSelectD1;
    return true;
}

bool T5838Aad::AadD2ModeSet(const T5838AadDConf& conf) {
    SwitchClkToGpio();
    const uint16_t abs_thr       = static_cast<uint16_t>(conf.abs_thr);
    const uint16_t floor         = static_cast<uint16_t>(conf.floor);
    const uint16_t rel_pulse_min = static_cast<uint16_t>(conf.rel_pulse_min);
    const uint16_t abs_pulse_min = static_cast<uint16_t>(conf.abs_pulse_min);

    const AddrDataPair write_data[] = {
        {kT5838RegAadMode,              0x00},
        {kT5838RegAadDFloorHi,          static_cast<uint8_t>((floor >> 8) & 0x1F)},
        {kT5838RegAadDFloorLo,          static_cast<uint8_t>(floor & 0xFF)},
        {0x2C,                          0x32},
        {0x2D,                          0xC0},
        {kT5838RegAadDRelPulseMinLo,    static_cast<uint8_t>(rel_pulse_min & 0xFF)},
        {kT5838RegAadDAbsRelPulseMinSh, static_cast<uint8_t>(
                                            ((abs_pulse_min >> 4) & 0xF0) |
                                            ((rel_pulse_min >> 8) & 0x0F))},
        {kT5838RegAadDAbsPulseMinLo,    static_cast<uint8_t>(abs_pulse_min & 0xFF)},
        {kT5838RegAadDAbsThrLo,         static_cast<uint8_t>(abs_thr & 0xFF)},
        {kT5838RegAadDAbsThrHi,         static_cast<uint8_t>(((abs_thr >> 8) & 0x1F) | 0x40)},
        {kT5838RegAadDRelThr,           static_cast<uint8_t>(conf.rel_thr)},
        {kT5838RegAadMode,              static_cast<uint8_t>(kT5838AadSelectD2)},
    };
    if (!AadModeSet(write_data, sizeof(write_data) / sizeof(write_data[0]))) {
        printf("[T5838 AAD] AadD2ModeSet failed\r\n");
        return false;
    }
    aad_mode_ = kT5838AadSelectD2;
    return true;
}

bool T5838Aad::AadModeDisable() {
    SwitchClkToGpio();
    if (!RegWrite(kT5838RegAadMode, 0x00)) {
        printf("[T5838 AAD] AadModeDisable: RegWrite failed\r\n");
        return false;
    }
    aad_mode_     = kT5838AadSelectNone;
    aad_unlocked_ = false;
    return true;
}

}  // namespace coralmicro
