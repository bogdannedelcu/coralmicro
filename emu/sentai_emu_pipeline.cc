// B8.6 ARM emulator prep + flow pipeline spike.
//
// Builds on the B8.5 VCam IRQ path by inserting a two-stage task pipeline
// behind the same camera-frame-ready boundary:
//
//   Renode vcam ─CONTROL.ARM=1─► IRQ 94 ─► PrepTask ─notify─► FlowTask
//                                            │                    │
//                                            └─ scalar publish ───┘
//
// PrepTask wakes from the VCam IRQ, scans the raw frame bytes, computes
// `sum` and `avg`, and publishes them into a shared slot before notifying
// FlowTask.  FlowTask reads the slot, increments its own counter, and
// writes a per-frame marker line to LPUART6.  Each marker reports the
// frame sequence number, sum, and average so a downstream verdict can
// detect a partial pipeline failure (e.g. PrepTask runs but FlowTask
// is starved) cleanly.
//
// The contract mirrors the production sentai_runtime W11 pipeline:
//   ISR  ─FromISR notify─► PrepTask ─task notify─► FlowTask
// No queues, no event groups; the simplest pattern that exercises the
// real Cortex-M scheduler hand-offs the ARM build relies on.

#include <stdint.h>
#include <string.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "fsl_device_registers.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_irq_count;
extern "C" volatile uint32_t g_sentai_emu_prep_processed;
extern "C" volatile uint32_t g_sentai_emu_flow_consumed;
extern "C" volatile uint32_t g_sentai_emu_pipeline_errors;
extern "C" volatile uint32_t g_sentai_emu_last_sum;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTasksCreated = 0x0200;
constexpr uint32_t kBootPrepReady = 0x0700;
constexpr uint32_t kBootFlowReady = 0x0800;
constexpr uint32_t kBootBothReady = 0x0900;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uintptr_t kVcamBase = 0x40900000u;
constexpr uint32_t kVcamStatusOffset = 0x00u;
constexpr uint32_t kVcamFramePtrOffset = 0x04u;
constexpr uint32_t kVcamFrameLenOffset = 0x08u;
constexpr uint32_t kVcamFrameSeqOffset = 0x0Cu;

constexpr IRQn_Type kVcamIrqn = Reserved110_IRQn;  // = 94

constexpr size_t kFrameBytes = 64;
constexpr size_t kTaskStackWords = 4 * 1024;       // 16 KiB per task

uint8_t g_frame_buffer[kFrameBytes] __attribute__((aligned(8), section(".sdram_data")));

struct PrepSlot {
    uint32_t seq;
    uint32_t sum;
    uint32_t avg;
    volatile uint32_t valid;  // writer sets last, reader clears
};
PrepSlot g_prep_slot __attribute__((aligned(8))) = {0, 0, 0, 0};

StaticTask_t g_prep_tcb;
StackType_t g_prep_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_prep_handle = nullptr;

StaticTask_t g_flow_tcb;
StackType_t g_flow_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_flow_handle = nullptr;

volatile uint32_t &VcamReg(uint32_t offset) {
    return *reinterpret_cast<volatile uint32_t *>(kVcamBase + offset);
}

void UartInit() {
    static volatile uint32_t *const lpuart_ctrl =
        reinterpret_cast<volatile uint32_t *>(0x40090018u);
    *lpuart_ctrl = (1u << 18) | (1u << 19);
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

void PrepTask(void *) {
    g_prep_handle = xTaskGetCurrentTaskHandle();
    UartInit();
    UartWrite("\r\nSentAI EMU PIPELINE B8.6\r\n");
    UartWrite("PrepTask ready\r\n");

    VcamReg(kVcamFramePtrOffset) = reinterpret_cast<uint32_t>(g_frame_buffer);
    VcamReg(kVcamFrameLenOffset) = static_cast<uint32_t>(kFrameBytes);
    NVIC_SetPriority(kVcamIrqn, 8);
    NVIC_EnableIRQ(kVcamIrqn);

    g_sentai_emu_boot_state = kBootPrepReady;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        g_sentai_emu_last_tick = xTaskGetTickCount();

        uint32_t sum = 0;
        for (size_t i = 0; i < kFrameBytes; ++i) {
            sum += g_frame_buffer[i];
        }
        uint32_t seq = VcamReg(kVcamFrameSeqOffset);

        // Publish: fields first, valid flag last.  A trailing __DMB() makes
        // the writes observable to FlowTask before the notification arrives.
        g_prep_slot.seq = seq;
        g_prep_slot.sum = sum;
        g_prep_slot.avg = sum / kFrameBytes;
        __DMB();
        g_prep_slot.valid = 1;
        g_sentai_emu_last_sum = sum;
        ++g_sentai_emu_prep_processed;

        if (g_flow_handle != nullptr) {
            xTaskNotifyGive(g_flow_handle);
        }
    }
}

void FlowTask(void *) {
    g_flow_handle = xTaskGetCurrentTaskHandle();
    UartWrite("FlowTask ready\r\n");
    g_sentai_emu_boot_state =
        (g_sentai_emu_boot_state == kBootPrepReady) ? kBootBothReady
                                                    : kBootFlowReady;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!g_prep_slot.valid) {
            ++g_sentai_emu_pipeline_errors;
            continue;
        }

        __DMB();
        uint32_t seq = g_prep_slot.seq;
        uint32_t sum = g_prep_slot.sum;
        uint32_t avg = g_prep_slot.avg;
        g_prep_slot.valid = 0;

        ++g_sentai_emu_flow_consumed;
        ++g_sentai_emu_heartbeat;

        UartWrite("FLOW ");
        UartDec(g_sentai_emu_flow_consumed);
        UartWrite(" prep_seq=");
        UartDec(seq);
        UartWrite(" sum=");
        UartDec(sum);
        UartWrite(" avg=");
        UartDec(avg);
        UartWrite("\r\n");
    }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_irq_count = 0;
volatile uint32_t g_sentai_emu_prep_processed = 0;
volatile uint32_t g_sentai_emu_flow_consumed = 0;
volatile uint32_t g_sentai_emu_pipeline_errors = 0;
volatile uint32_t g_sentai_emu_last_sum = 0;
}

extern "C" void Reserved110_IRQHandler(void) {
    ++g_sentai_emu_irq_count;
    VcamReg(kVcamStatusOffset) = 1u;

    if (g_prep_handle != nullptr) {
        BaseType_t higher = pdFALSE;
        vTaskNotifyGiveFromISR(g_prep_handle, &higher);
        portYIELD_FROM_ISR(higher);
    }
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t prep = xTaskCreateStatic(PrepTask, "emu_prep",
                                          kTaskStackWords, nullptr,
                                          tskIDLE_PRIORITY + 3,
                                          g_prep_stack, &g_prep_tcb);
    TaskHandle_t flow = xTaskCreateStatic(FlowTask, "emu_flow",
                                           kTaskStackWords, nullptr,
                                           tskIDLE_PRIORITY + 2,
                                           g_flow_stack, &g_flow_tcb);
    if (!prep || !flow) {
        g_sentai_emu_boot_state = kBootCreateTaskFailed;
        while (true) {
        }
    }
    g_sentai_emu_boot_state = kBootTasksCreated;

    vTaskStartScheduler();

    g_sentai_emu_boot_state = kBootSchedulerReturned;
    while (true) {
    }
}
