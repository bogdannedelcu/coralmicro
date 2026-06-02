// B8.5 ARM emulator camera frame-provider spike.
//
// VCam is a small Renode Python peripheral defined in emu/renode/sentai_rt1176.repl
// at 0x40900000.  When the host writes CONTROL.ARM=1, the peripheral fills a
// guest-RAM frame buffer with deterministic bytes (`frame_seq & 0xFF`
// repeated), increments FRAME_SEQ, sets STATUS.frame_ready, and pends NVIC
// IRQ 94 (Reserved110_IRQn) by writing the Cortex-M ISPR2 register.
//
// This target proves the full ARM ISR path:
//   1. host triggers VCam delivery + IRQ pend;
//   2. Cortex-M takes the IRQ exception, stacks registers, jumps via the
//      vector table at 0x800 to our strong override of Reserved110_IRQHandler;
//   3. the ISR ACKs the peripheral and signals the consumer task via the
//      standard FreeRTOS xTaskNotifyFromISR/portYIELD_FROM_ISR contract;
//   4. the consumer task wakes, validates the frame bytes, increments a
//      counter, and writes a marker line to LPUART6.
//
// No camera optics, no MIPI-CSI, no PXP.  The point is ISR + scheduler
// fidelity at the camera-frame-ready boundary, not pixel pipeline accuracy.

#include <stdint.h>
#include <string.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "fsl_device_registers.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_frames_consumed;
extern "C" volatile uint32_t g_sentai_emu_frames_valid;
extern "C" volatile uint32_t g_sentai_emu_irq_count;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootConsumerReady = 0x0600;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uintptr_t kVcamBase = 0x40900000u;
constexpr uint32_t kVcamStatusOffset = 0x00u;
constexpr uint32_t kVcamFramePtrOffset = 0x04u;
constexpr uint32_t kVcamFrameLenOffset = 0x08u;
constexpr uint32_t kVcamFrameSeqOffset = 0x0Cu;

constexpr IRQn_Type kVcamIrqn = Reserved110_IRQn;  // = 94

constexpr size_t kFrameBytes = 64;                 // tiny, ISR-fidelity test
constexpr size_t kConsumerStackWords = 4 * 1024;   // 16 KiB

uint8_t g_frame_buffer[kFrameBytes] __attribute__((aligned(8), section(".sdram_data")));

StaticTask_t g_consumer_tcb;
StackType_t g_consumer_stack[kConsumerStackWords] __attribute__((aligned(8)));
TaskHandle_t g_consumer_handle = nullptr;

volatile uint32_t &VcamReg(uint32_t offset) {
    return *reinterpret_cast<volatile uint32_t *>(kVcamBase + offset);
}

void UartInit() {
    static volatile uint32_t *const lpuart_ctrl =
        reinterpret_cast<volatile uint32_t *>(0x40090018u);
    *lpuart_ctrl = (1u << 18) | (1u << 19);  // RE | TE
}

void UartPutChar(char ch) {
    static volatile uint32_t *const lpuart_stat =
        reinterpret_cast<volatile uint32_t *>(0x40090014u);
    static volatile uint32_t *const lpuart_data =
        reinterpret_cast<volatile uint32_t *>(0x4009001Cu);
    while ((*lpuart_stat & (1u << 23)) == 0u) {
    }
    *lpuart_data = static_cast<uint8_t>(ch);
}

void UartWrite(const char *s) {
    while (*s) {
        UartPutChar(*s++);
    }
}

void UartHex2(uint32_t v) {
    static const char *hex = "0123456789ABCDEF";
    UartPutChar(hex[(v >> 4) & 0xFu]);
    UartPutChar(hex[v & 0xFu]);
}

void UartDec(uint32_t v) {
    char buf[12];
    int n = 0;
    if (v == 0) {
        UartPutChar('0');
        return;
    }
    while (v && n < (int)sizeof(buf)) {
        buf[n++] = '0' + (v % 10u);
        v /= 10u;
    }
    while (n > 0) {
        UartPutChar(buf[--n]);
    }
}

void ConsumerTask(void *) {
    UartInit();
    UartWrite("\r\nSentAI EMU CAMERA B8.5\r\n");

    // Hand the VCam peripheral the frame buffer it should DMA into.
    VcamReg(kVcamFramePtrOffset) = reinterpret_cast<uint32_t>(g_frame_buffer);
    VcamReg(kVcamFrameLenOffset) = static_cast<uint32_t>(kFrameBytes);

    // NVIC priority must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY
    // so the ISR can call xTaskNotifyFromISR.  configLIBRARY_MAX_SYSCALL is 2
    // with 4 priority bits, i.e. 0x20.  We use 0x80 — safely below the
    // syscall threshold.
    NVIC_SetPriority(kVcamIrqn, 8);
    NVIC_EnableIRQ(kVcamIrqn);

    UartWrite("Consumer ready, awaiting frames\r\n");
    g_sentai_emu_boot_state = kBootConsumerReady;
    g_consumer_handle = xTaskGetCurrentTaskHandle();

    while (true) {
        // Block forever until the ISR notifies.
        uint32_t notify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        (void)notify;
        ++g_sentai_emu_frames_consumed;
        g_sentai_emu_last_tick = xTaskGetTickCount();

        // Validate: first byte should equal (frame_seq & 0xFF).
        uint32_t seq = VcamReg(kVcamFrameSeqOffset);
        uint8_t expected = static_cast<uint8_t>(seq & 0xFFu);
        uint8_t got = g_frame_buffer[0];
        uint8_t tail = g_frame_buffer[kFrameBytes - 1];
        bool ok = (got == expected) && (tail == expected);
        if (ok) {
            ++g_sentai_emu_frames_valid;
        }

        UartWrite("FRAME ");
        UartDec(g_sentai_emu_frames_consumed);
        UartWrite(" seq=");
        UartDec(seq);
        UartWrite(" byte=0x");
        UartHex2(got);
        UartWrite(" ok=");
        UartPutChar(ok ? '1' : '0');
        UartWrite("\r\n");
        ++g_sentai_emu_heartbeat;
    }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_frames_consumed = 0;
volatile uint32_t g_sentai_emu_frames_valid = 0;
volatile uint32_t g_sentai_emu_irq_count = 0;
}

// Strong override of the weak alias in startup_MIMXRT1176_cm7.S.
extern "C" void Reserved110_IRQHandler(void) {
    ++g_sentai_emu_irq_count;

    // Acknowledge VCam status.frame_ready (write-1-to-clear).
    VcamReg(kVcamStatusOffset) = 1u;

    if (g_consumer_handle != nullptr) {
        BaseType_t higher = pdFALSE;
        vTaskNotifyGiveFromISR(g_consumer_handle, &higher);
        portYIELD_FROM_ISR(higher);
    }
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t task = xTaskCreateStatic(ConsumerTask, "emu_camera",
                                          kConsumerStackWords, nullptr,
                                          tskIDLE_PRIORITY + 2,
                                          g_consumer_stack, &g_consumer_tcb);
    if (!task) {
        g_sentai_emu_boot_state = kBootCreateTaskFailed;
        while (true) {
        }
    }
    g_sentai_emu_boot_state = kBootTaskCreated;

    vTaskStartScheduler();

    g_sentai_emu_boot_state = kBootSchedulerReturned;
    while (true) {
    }
}
