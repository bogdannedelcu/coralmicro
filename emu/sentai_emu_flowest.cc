// B8.7c ARM emulator Flow-offset detection spike.
//
// Builds on B8.5 VCam IRQ + B8.6 pipeline topology.  Replaces the
// "sum-of-bytes" Stage1Task reduction with a brute-force SAD block-match
// flow estimator.  The Renode .resc injects a sequence of 32x32 grayscale
// cat frames into `g_frame_buffer` between IRQs (see
// emu/output/scenes/scene_*.bin and emu/scripts/prepare_cat_scenes.py).
// Each frame is either the BASE cat or a known X-shifted variant, so the
// runner can assert that the detected `dx` matches the injected shift.
//
// IMPORTANT NAMING NOTE.  Same HARD RULE as B8.6 / B8.7: this spike does
// NOT implement production `FlowTask` (USADA8 phase correlation in
// flow_task.cc) or production `PrepTask` (PXP + quant in detection_task.cc).
// It is a brute-force SAD demonstrator that proves the camera-frame-ready
// boundary feeds a downstream consumer that can extract motion semantics
// from the injected bytes.  Stand-in names (`Stage1Task` for the estimator,
// `Stage2Task` for the logger) keep emu test scaffolding distinct from the
// real production task symbols.
//
// History: B7 had an equivalent SIM test using the same cat BMPs that
// reported `dx = 243` (and `-129` in another run) for a 2-px shift.  The
// emulator gate here checks whether the closer-to-real-board execution
// model gives the expected constant `dx = +1` answer (one px pan per
// frame).
//
// PIXEL FORMAT CAVEAT (operator note 2026-06-02).  Production camera
// ISR delivers XRGB8888 raw frames from the OV5640 / MIPI CSI; production
// PrepTask then converts XRGB8888 → Y8 before publishing the 80x60
// FLOW_GRAY slot consumed by FlowTask.  This emu spike skips the PrepTask
// conversion stage: VCam (`emu/renode/sentai_rt1176.repl`) DMAs Y8 bytes
// directly into `g_frame_buffer` at 32x32.  The test therefore validates
// the downstream flow consumer's ability to detect offset in Y8 frames,
// NOT the camera/PrepTask format pipeline.  Modelling XRGB8888 + PrepTask
// + the 80x60 grid is a later gate; pretending the 32x32 Y8 input is the
// same as the production FLOW_GRAY_80x60 slot would be misleading.

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
extern "C" volatile int32_t  g_sentai_emu_last_dx;
extern "C" volatile int32_t  g_sentai_emu_last_dy;
extern "C" volatile uint32_t g_sentai_emu_last_sad;

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

constexpr IRQn_Type kVcamIrqn = Reserved110_IRQn;

constexpr int kFrameW = 32;
constexpr int kFrameH = 32;
constexpr int kFrameBytes = kFrameW * kFrameH;
constexpr int kSearchRange = 5;            // dx, dy in [-5, +5]
constexpr int kInnerStart = kSearchRange;  // y, x range so all candidates are in bounds
constexpr int kInnerEnd = kFrameW - kSearchRange;  // exclusive

constexpr size_t kTaskStackWords = 6 * 1024;

}  // namespace

// Frame buffers live at extern "C" linkage so the Renode .resc can resolve
// `g_frame_buffer` via `sysbus GetSymbolAddress` and feed scenes by writing
// directly into guest SDRAM at run time.  C++ anonymous-namespace mangling
// would otherwise hide the symbol from the host script.
extern "C" {
    uint8_t g_frame_buffer[32 * 32] __attribute__((aligned(8), section(".sdram_data")));
    uint8_t g_prev_buffer[32 * 32]  __attribute__((aligned(8), section(".sdram_data")));
}

namespace {

struct FlowSlot {
    uint32_t seq;
    int32_t dx;
    int32_t dy;
    uint32_t sad;
    volatile uint32_t valid;
};
FlowSlot g_flow_slot __attribute__((aligned(8))) = {0, 0, 0, 0, 0};

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

void UartDecSigned(int32_t v) {
    if (v < 0) {
        UartPutChar('-');
        v = -v;
    }
    UartDec(static_cast<uint32_t>(v));
}

// Brute-force SAD block matcher.  Convention: dx > 0 means features in
// `curr` moved right relative to `prev`.  For each candidate (dx, dy),
// compares curr[y, x] vs prev[y - dy, x - dx] over the inner region
// [kInnerStart, kInnerEnd) ^ 2 so every candidate covers the same area
// (no normalization needed).
void EstimateFlow(const uint8_t *curr, const uint8_t *prev,
                  int &best_dx, int &best_dy, uint32_t &best_sad) {
    best_sad = UINT32_MAX;
    best_dx = 0;
    best_dy = 0;
    for (int dy = -kSearchRange; dy <= kSearchRange; ++dy) {
        for (int dx = -kSearchRange; dx <= kSearchRange; ++dx) {
            uint32_t sad = 0;
            for (int y = kInnerStart; y < kInnerEnd; ++y) {
                const uint8_t *cur_row = curr + y * kFrameW;
                const uint8_t *prev_row = prev + (y - dy) * kFrameW;
                for (int x = kInnerStart; x < kInnerEnd; ++x) {
                    int c = cur_row[x];
                    int p = prev_row[x - dx];
                    int d = c - p;
                    sad += static_cast<uint32_t>(d < 0 ? -d : d);
                }
            }
            if (sad < best_sad) {
                best_sad = sad;
                best_dx = dx;
                best_dy = dy;
            }
        }
    }
}

void Stage1Task(void *) {
    g_stage1_handle = xTaskGetCurrentTaskHandle();
    UartInit();
    UartWrite("\r\nSentAI EMU FLOWEST B8.7c\r\n");
    UartWrite("Stage1Task ready\r\n");

    VcamReg(kVcamFramePtrOffset) = reinterpret_cast<uint32_t>(g_frame_buffer);
    VcamReg(kVcamFrameLenOffset) = static_cast<uint32_t>(kFrameBytes);
    // FILL_MODE = 0: keep the bytes that Renode `sysbus LoadBinary` placed
    // at g_frame_buffer between IRQs.  With the default FILL_MODE = 1 used
    // by B8.5/6/7, VCam would overwrite the loaded cat scene with its own
    // synthetic byte pattern and SAD would compare uniform images.
    constexpr uint32_t kVcamFillModeOffset = 0x18u;
    VcamReg(kVcamFillModeOffset) = 0;
    NVIC_SetPriority(kVcamIrqn, 8);
    NVIC_EnableIRQ(kVcamIrqn);

    g_sentai_emu_boot_state = kBootStage1Ready;

    bool has_prev = false;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        g_sentai_emu_last_tick = xTaskGetTickCount();
        uint32_t seq = VcamReg(kVcamFrameSeqOffset);

        int dx = 0, dy = 0;
        uint32_t sad = 0;
        if (has_prev) {
            EstimateFlow(g_frame_buffer, g_prev_buffer, dx, dy, sad);
        }
        // Copy curr -> prev for the next round.  memcpy is in newlib so we
        // open-code the byte loop to avoid surprises on the emu build.
        for (int i = 0; i < kFrameBytes; ++i) {
            g_prev_buffer[i] = g_frame_buffer[i];
        }
        has_prev = true;

        g_flow_slot.seq = seq;
        g_flow_slot.dx = dx;
        g_flow_slot.dy = dy;
        g_flow_slot.sad = sad;
        __DMB();
        g_flow_slot.valid = 1;
        g_sentai_emu_last_dx = dx;
        g_sentai_emu_last_dy = dy;
        g_sentai_emu_last_sad = sad;
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
        if (!g_flow_slot.valid) {
            ++g_sentai_emu_pipeline_errors;
            continue;
        }
        __DMB();
        uint32_t seq = g_flow_slot.seq;
        int32_t dx = g_flow_slot.dx;
        int32_t dy = g_flow_slot.dy;
        uint32_t sad = g_flow_slot.sad;
        g_flow_slot.valid = 0;

        ++g_sentai_emu_stage2_consumed;
        ++g_sentai_emu_heartbeat;

        UartWrite("FLOWEST ");
        UartDec(g_sentai_emu_stage2_consumed);
        UartWrite(" frame_seq=");
        UartDec(seq);
        UartWrite(" dx=");
        UartDecSigned(dx);
        UartWrite(" dy=");
        UartDecSigned(dy);
        UartWrite(" sad=");
        UartDec(sad);
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
volatile int32_t  g_sentai_emu_last_dx = 0;
volatile int32_t  g_sentai_emu_last_dy = 0;
volatile uint32_t g_sentai_emu_last_sad = 0;
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
