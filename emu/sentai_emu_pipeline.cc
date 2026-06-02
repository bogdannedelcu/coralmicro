// B8.6 ARM emulator pipeline-topology spike.
//
// Builds on the B8.5 VCam IRQ path by inserting a two-stage task pipeline
// behind the same camera-frame-ready boundary:
//
//   Renode vcam ─CONTROL.ARM=1─► IRQ 94 ─► Stage1Task ─notify─► Stage2Task
//                                            │                    │
//                                            └─ scalar publish ───┘
//
// IMPORTANT NAMING NOTE.  Production sentai_runtime already owns the
// symbols `PrepTask`, `InferTask`, `FlowTask`, and `CameraTask`:
//   - `PrepTask` / `InferTask` live in examples/sentai_runtime/detection_task.cc
//     and run PXP + RGB->Y8 + INT8 quantisation, then EdgeTPU invoke.
//   - `FlowTask` lives in examples/sentai_runtime/flow_task.cc and runs
//     USADA8 / phase correlation on the prep output.
//   - `CameraTask` is a class in libs/camera/camera.cc that owns the CSI
//     receiver queue and ISR.
// This emu spike does NOT implement any of those algorithms; it only
// exercises the ISR -> task-A -> task-B notification topology.  The
// scaffolding tasks below are therefore deliberately named `Stage1Task`
// and `Stage2Task` so a future reader greppping the codebase for
// `PrepTask` or `FlowTask` does not land here by mistake.  Real
// production tasks will plug into this same boundary at a later gate
// once they can be built against the emulator profile.
//
// Stage1Task wakes from the VCam IRQ, scans the raw frame bytes, computes
// `sum` and `avg`, and publishes them into a shared slot before notifying
// Stage2Task.  Stage2Task reads the slot, increments its own counter, and
// writes a per-frame marker line to LPUART6.  Each marker reports the
// frame sequence number, sum, and average so a downstream verdict can
// detect a partial pipeline failure (e.g. Stage1Task runs but Stage2Task
// is starved) cleanly.
//
// The contract mirrors the production sentai_runtime W11 pipeline shape:
//   ISR  ─FromISR notify─► Stage1Task ─task notify─► Stage2Task
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
extern "C" volatile uint32_t g_sentai_emu_stage1_processed;
extern "C" volatile uint32_t g_sentai_emu_stage2_consumed;
extern "C" volatile uint32_t g_sentai_emu_pipeline_errors;
extern "C" volatile uint32_t g_sentai_emu_last_sum;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTasksCreated = 0x0200;
constexpr uint32_t kBootStage1Ready = 0x0700;
constexpr uint32_t kBootStage2Ready = 0x0800;
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

struct ScalarSlot {
    uint32_t seq;
    uint32_t sum;
    uint32_t avg;
    volatile uint32_t valid;  // writer sets last, reader clears
};
ScalarSlot g_scalar_slot __attribute__((aligned(8))) = {0, 0, 0, 0};

StaticTask_t g_stage1_tcb;
StackType_t g_stage1_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_stage1_handle = nullptr;

StaticTask_t g_stage2_tcb;
StackType_t g_stage2_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_stage2_handle = nullptr;

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

void Stage1Task(void *) {
    g_stage1_handle = xTaskGetCurrentTaskHandle();
    UartInit();
    UartWrite("\r\nSentAI EMU PIPELINE B8.6\r\n");
    UartWrite("Stage1Task ready\r\n");

    VcamReg(kVcamFramePtrOffset) = reinterpret_cast<uint32_t>(g_frame_buffer);
    VcamReg(kVcamFrameLenOffset) = static_cast<uint32_t>(kFrameBytes);
    NVIC_SetPriority(kVcamIrqn, 8);
    NVIC_EnableIRQ(kVcamIrqn);

    g_sentai_emu_boot_state = kBootStage1Ready;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        g_sentai_emu_last_tick = xTaskGetTickCount();

        uint32_t sum = 0;
        for (size_t i = 0; i < kFrameBytes; ++i) {
            sum += g_frame_buffer[i];
        }
        uint32_t seq = VcamReg(kVcamFrameSeqOffset);

        // Publish: fields first, valid flag last.  A trailing __DMB() makes
        // the writes observable to Stage2Task before the notification arrives.
        g_scalar_slot.seq = seq;
        g_scalar_slot.sum = sum;
        g_scalar_slot.avg = sum / kFrameBytes;
        __DMB();
        g_scalar_slot.valid = 1;
        g_sentai_emu_last_sum = sum;
        ++g_sentai_emu_stage1_processed;

        if (g_stage2_handle != nullptr) {
            xTaskNotifyGive(g_stage2_handle);
        }
    }
}

void Stage2Task(void *) {
    g_stage2_handle = xTaskGetCurrentTaskHandle();
    UartWrite("Stage2Task ready\r\n");
    g_sentai_emu_boot_state =
        (g_sentai_emu_boot_state == kBootStage1Ready) ? kBootBothReady
                                                      : kBootStage2Ready;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!g_scalar_slot.valid) {
            ++g_sentai_emu_pipeline_errors;
            continue;
        }

        __DMB();
        uint32_t seq = g_scalar_slot.seq;
        uint32_t sum = g_scalar_slot.sum;
        uint32_t avg = g_scalar_slot.avg;
        g_scalar_slot.valid = 0;

        ++g_sentai_emu_stage2_consumed;
        ++g_sentai_emu_heartbeat;

        UartWrite("STAGE2 ");
        UartDec(g_sentai_emu_stage2_consumed);
        UartWrite(" frame_seq=");
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
volatile uint32_t g_sentai_emu_stage1_processed = 0;
volatile uint32_t g_sentai_emu_stage2_consumed = 0;
volatile uint32_t g_sentai_emu_pipeline_errors = 0;
volatile uint32_t g_sentai_emu_last_sum = 0;
}

extern "C" void Reserved110_IRQHandler(void) {
    ++g_sentai_emu_irq_count;
    VcamReg(kVcamStatusOffset) = 1u;

    if (g_stage1_handle != nullptr) {
        BaseType_t higher = pdFALSE;
        vTaskNotifyGiveFromISR(g_stage1_handle, &higher);
        portYIELD_FROM_ISR(higher);
    }
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t stage1 = xTaskCreateStatic(Stage1Task, "emu_stage1",
                                            kTaskStackWords, nullptr,
                                            tskIDLE_PRIORITY + 3,
                                            g_stage1_stack, &g_stage1_tcb);
    TaskHandle_t stage2 = xTaskCreateStatic(Stage2Task, "emu_stage2",
                                            kTaskStackWords, nullptr,
                                            tskIDLE_PRIORITY + 2,
                                            g_stage2_stack, &g_stage2_tcb);
    if (!stage1 || !stage2) {
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
