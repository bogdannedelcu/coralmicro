// B8.8 Phase 1 — USB host stack probe.
//
// Links the production `libs/usb/usb_host_task.cc` plus the minimum
// NXP RT1176 SDK USB middleware needed by `UsbHostTask::UsbHostTask()`
// into an emulator image, and exercises the constructor.  No Renode-side
// USB model is configured yet; the goal of P1 is to prove the
// production USB code COMPILES and LINKS against the emu profile, and
// to surface the complete set of SDK files pulled in transitively.
//
// The probe deliberately does NOT call `UsbHostTask::GetSingleton()` or
// `Init()` (which would create a FreeRTOS task) — that is Phase 2, once
// the Renode platform is wired up.  P1 is a one-shot constructor call
// from `main()` with progress markers on LPUART6.
//
// HARD RULE (per `feedback_emu_tpu_must_run_real_edgetpu_manager`):
// this target uses the production class verbatim, not an emu-only
// reimplementation.  The probe stays under emu/ so it does not affect
// the ARM `sentai_runtime` build.

#include <stdint.h>

// Production include order: usb_host_config.h must precede usb_host.h so the
// USB_HOST_CONFIG_* macros are visible inside the SDK headers.
#include "third_party/modified/nxp/rt1176-sdk/usb_host_config.h"
#include "libs/usb/usb_host_task.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "fsl_device_registers.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_step;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;  // referenced by FreeRTOS hooks

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootBeforeUsbHostCtor = 0x0200;
constexpr uint32_t kBootAfterUsbHostCtor = 0x0300;
constexpr uint32_t kBootSchedulerNotStarted = 0xCAFE;

void UartInit() {
    *reinterpret_cast<volatile uint32_t *>(0x40090018u) = (1u << 18) | (1u << 19);
}

void UartPutChar(char ch) {
    while ((*reinterpret_cast<volatile uint32_t *>(0x40090014u) & (1u << 23)) == 0u) {
    }
    *reinterpret_cast<volatile uint32_t *>(0x4009001Cu) = static_cast<uint8_t>(ch);
}

void UartWrite(const char *s) {
    while (*s) {
        UartPutChar(*s++);
    }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_step = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;
    UartInit();
    UartWrite("\r\nSentAI EMU USBHOST PROBE B8.8 P1\r\n");

    UartWrite("step 1: about to construct UsbHostTask\r\n");
    g_sentai_emu_boot_state = kBootBeforeUsbHostCtor;
    g_sentai_emu_step = 1;

    // Production constructor.  Does: CLOCK_EnableUsbhs1PhyPllClock,
    // CLOCK_EnableUsbhs1Clock, USB_EhciLowPowerPhyInit, USB_HostInit.
    coralmicro::UsbHostTask host_task;
    (void)host_task;

    g_sentai_emu_boot_state = kBootAfterUsbHostCtor;
    g_sentai_emu_step = 2;
    UartWrite("step 2: UsbHostTask constructor returned\r\n");

    // P1 does NOT call Init() — that would create a FreeRTOS task and
    // start spinning the EHCI driver.  Just spin in main for the probe.
    g_sentai_emu_boot_state = kBootSchedulerNotStarted;
    while (true) {
    }
}
