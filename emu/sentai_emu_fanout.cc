// B8.7 ARM emulator multi-reader fan-out spike behind VCam IRQ.
//
// Topology:
//
//   Renode vcam ─CONTROL.ARM=1─► IRQ 94 ─FromISR notify─► Stage1Task
//                                                           │
//                                                           ├─ scan + reduce
//                                                           ├─ seqlock publish
//                                                           ├─ notify Stage2A
//                                                           └─ notify Stage2B
//                                                                  │  │
//                                                                  ▼  ▼
//                                                              Stage2A  Stage2B
//                                                                  read same slot
//                                                                  via seqlock retry
//
// IMPORTANT NAMING NOTE (same rule as B8.6).  Production sentai_runtime
// already owns `PrepTask`, `InferTask`, `FlowTask`, `CameraTask`, and uses
// a real seqlock + multi-reader pattern around the prep output slot.  This
// emu spike does NOT implement those algorithms; the per-reader work is
// only a load of the scalar slot and a UART marker.  Stand-in scaffolding
// task names (Stage1Task / Stage2ATask / Stage2BTask) are deliberately
// distinct from production symbols so a future reader greppping for
// `FlowTask` does not land here.  Captured as a HARD RULE in auto-memory.
//
// What this proves over B8.6:
//
//   * The same camera-frame-ready boundary can fan out to MULTIPLE downstream
//     consumers, in the right order, without one consumer starving the
//     other.
//   * The seqlock writer / reader primitive (version counter that is odd
//     while the writer is in progress, even when the slot is stable)
//     compiles and runs correctly on ARM CM7 FreeRTOS under Renode.  Both
//     readers see consistent, identical data per frame.
//   * Per-reader counters move on every IRQ.  A consumer drop or torn read
//     would show up as either `stage2a_consumed != stage2b_consumed != 5`
//     or as a non-zero `seqlock_torn_reads`.
//
// Real contention testing (writer overlaps reader inside a single frame
// window) needs back-to-back IRQs and is left for a later gate.  For B8.7
// the seqlock retry loop is just exercised on the happy path.

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
extern "C" volatile uint32_t g_sentai_emu_stage2a_consumed;
extern "C" volatile uint32_t g_sentai_emu_stage2b_consumed;
extern "C" volatile uint32_t g_sentai_emu_pipeline_errors;
extern "C" volatile uint32_t g_sentai_emu_seqlock_torn_reads;
extern "C" volatile uint32_t g_sentai_emu_last_sum;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTasksCreated = 0x0200;
constexpr uint32_t kBootStage1Ready = 0x0700;
constexpr uint32_t kBootStage2AReady = 0x0800;
constexpr uint32_t kBootStage2BReady = 0x0801;
constexpr uint32_t kBootAllReady = 0x0A00;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr uintptr_t kVcamBase = 0x40900000u;
constexpr uint32_t kVcamStatusOffset = 0x00u;
constexpr uint32_t kVcamFramePtrOffset = 0x04u;
constexpr uint32_t kVcamFrameLenOffset = 0x08u;
constexpr uint32_t kVcamFrameSeqOffset = 0x0Cu;

constexpr IRQn_Type kVcamIrqn = Reserved110_IRQn;  // = 94

constexpr size_t kFrameBytes = 64;
constexpr size_t kTaskStackWords = 4 * 1024;
constexpr int kSeqlockMaxRetries = 100;

uint8_t g_frame_buffer[kFrameBytes] __attribute__((aligned(8), section(".sdram_data")));

// Seqlock-protected slot.  `version` is even when stable, odd when the
// writer (Stage1Task) is in the middle of publishing fields.  Readers
// (Stage2ATask / Stage2BTask) load `version` before and after reading the
// payload and retry if either snapshot is odd or the two snapshots differ.
struct ScalarSlotV {
    volatile uint32_t version;
    uint32_t seq;
    uint32_t sum;
    uint32_t avg;
};
ScalarSlotV g_scalar_slot __attribute__((aligned(8))) = {0, 0, 0, 0};

StaticTask_t g_stage1_tcb;
StackType_t g_stage1_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_stage1_handle = nullptr;

StaticTask_t g_stage2a_tcb;
StackType_t g_stage2a_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_stage2a_handle = nullptr;

StaticTask_t g_stage2b_tcb;
StackType_t g_stage2b_stack[kTaskStackWords] __attribute__((aligned(8)));
TaskHandle_t g_stage2b_handle = nullptr;

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

// Seqlock writer.  Must be the SOLE writer of g_scalar_slot.
void SeqlockPublish(uint32_t seq, uint32_t sum, uint32_t avg) {
    uint32_t v = g_scalar_slot.version;
    g_scalar_slot.version = v + 1;  // become odd: writer in progress
    __DMB();
    g_scalar_slot.seq = seq;
    g_scalar_slot.sum = sum;
    g_scalar_slot.avg = avg;
    __DMB();
    g_scalar_slot.version = v + 2;  // become even: stable
}

// Seqlock reader.  Returns true on a stable snapshot, false if it gave up.
// Records each torn read so a downstream verdict can flag contention bugs.
bool SeqlockRead(uint32_t &seq, uint32_t &sum, uint32_t &avg) {
    for (int i = 0; i < kSeqlockMaxRetries; ++i) {
        uint32_t v1 = g_scalar_slot.version;
        if (v1 & 1u) {
            ++g_sentai_emu_seqlock_torn_reads;
            continue;
        }
        __DMB();
        seq = g_scalar_slot.seq;
        sum = g_scalar_slot.sum;
        avg = g_scalar_slot.avg;
        __DMB();
        uint32_t v2 = g_scalar_slot.version;
        if (v1 == v2) {
            return true;
        }
        ++g_sentai_emu_seqlock_torn_reads;
    }
    return false;
}

void Stage1Task(void *) {
    g_stage1_handle = xTaskGetCurrentTaskHandle();
    UartInit();
    UartWrite("\r\nSentAI EMU FANOUT B8.7\r\n");
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
        SeqlockPublish(seq, sum, sum / kFrameBytes);
        g_sentai_emu_last_sum = sum;
        ++g_sentai_emu_stage1_processed;

        if (g_stage2a_handle != nullptr) {
            xTaskNotifyGive(g_stage2a_handle);
        }
        if (g_stage2b_handle != nullptr) {
            xTaskNotifyGive(g_stage2b_handle);
        }
    }
}

void Stage2ATask(void *) {
    g_stage2a_handle = xTaskGetCurrentTaskHandle();
    UartWrite("Stage2ATask ready\r\n");
    if (g_sentai_emu_boot_state == kBootStage2BReady) {
        g_sentai_emu_boot_state = kBootAllReady;
    } else {
        g_sentai_emu_boot_state = kBootStage2AReady;
    }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t seq = 0;
        uint32_t sum = 0;
        uint32_t avg = 0;
        if (!SeqlockRead(seq, sum, avg)) {
            ++g_sentai_emu_pipeline_errors;
            continue;
        }
        ++g_sentai_emu_stage2a_consumed;
        ++g_sentai_emu_heartbeat;

        UartWrite("STAGE2A ");
        UartDec(g_sentai_emu_stage2a_consumed);
        UartWrite(" frame_seq=");
        UartDec(seq);
        UartWrite(" sum=");
        UartDec(sum);
        UartWrite(" avg=");
        UartDec(avg);
        UartWrite("\r\n");
    }
}

void Stage2BTask(void *) {
    g_stage2b_handle = xTaskGetCurrentTaskHandle();
    UartWrite("Stage2BTask ready\r\n");
    if (g_sentai_emu_boot_state == kBootStage2AReady) {
        g_sentai_emu_boot_state = kBootAllReady;
    } else {
        g_sentai_emu_boot_state = kBootStage2BReady;
    }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t seq = 0;
        uint32_t sum = 0;
        uint32_t avg = 0;
        if (!SeqlockRead(seq, sum, avg)) {
            ++g_sentai_emu_pipeline_errors;
            continue;
        }
        ++g_sentai_emu_stage2b_consumed;

        UartWrite("STAGE2B ");
        UartDec(g_sentai_emu_stage2b_consumed);
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
volatile uint32_t g_sentai_emu_stage2a_consumed = 0;
volatile uint32_t g_sentai_emu_stage2b_consumed = 0;
volatile uint32_t g_sentai_emu_pipeline_errors = 0;
volatile uint32_t g_sentai_emu_seqlock_torn_reads = 0;
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
    TaskHandle_t stage2a = xTaskCreateStatic(Stage2ATask, "emu_stage2a",
                                             kTaskStackWords, nullptr,
                                             tskIDLE_PRIORITY + 2,
                                             g_stage2a_stack, &g_stage2a_tcb);
    TaskHandle_t stage2b = xTaskCreateStatic(Stage2BTask, "emu_stage2b",
                                             kTaskStackWords, nullptr,
                                             tskIDLE_PRIORITY + 2,
                                             g_stage2b_stack, &g_stage2b_tcb);
    if (!stage1 || !stage2a || !stage2b) {
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
