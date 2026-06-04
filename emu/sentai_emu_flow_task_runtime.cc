// sentai_emu_flow_task_runtime.cc -- B8/S214 production FlowTask benchmark.
//
// Guest-side ARM-emulator target.  This is not the old flowest toy:
// it links examples/sentai_runtime/flow_task.cc and drives it through the
// sentai_prep FLOW_GRAY_80x60 slot, matching the runtime consumer boundary.

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "examples/sentai_runtime/flow_shared.h"
#include "examples/sentai_runtime/sentai_prep.h"

extern "C" int sentai_flow_start(int cam_id);
extern "C" int sentai_flow_stop(void);
extern "C" void sentai_flow_pub_stats(uint32_t* frames_published,
                                       uint32_t* grab_fail_total,
                                       uint32_t* grab_fail_streak,
                                       int* running);

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_flow_base_ready = 0;
volatile uint32_t g_sentai_emu_flow_completed = 0;
volatile uint32_t g_sentai_emu_flow_detect_ok = 0;
volatile uint32_t g_sentai_emu_flow_fail_code = 0;
volatile uint32_t g_sentai_emu_flow_elapsed_ms = 0;
volatile uint32_t g_sentai_emu_flow_fps_x100 = 0;
volatile int32_t  g_sentai_emu_flow_last_dx_q1000 = 0;
volatile int32_t  g_sentai_emu_flow_last_dy_q1000 = 0;
volatile uint32_t g_sentai_emu_flow_last_conf = 0;

uint8_t g_sentai_emu_flow_base_y80x60[FLOW_GRAY_PIXELS]
    __attribute__((aligned(32), section(".sdram_bss")));
}

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootWaitingBase = 0x0300;
constexpr uint32_t kBootFlowStarted = 0x0400;
constexpr uint32_t kBootValidated = 0x0500;
constexpr uint32_t kBootMeasured = 0x0600;
constexpr uint32_t kBootFailed = 0xEF00;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;

constexpr int kTaskStackWords = configMINIMAL_STACK_SIZE * 12;
constexpr int kMeasureFrames = 24;
constexpr int kFlowToleranceQ1000 = 450;
constexpr uint32_t kFlowMinConfidence = 100;

struct Offset {
    int8_t x;
    int8_t y;
};

constexpr Offset kOffsets[] = {
    {0, 0},
    {2, 0},
    {2, 2},
    {-1, 1},
    {-1, -2},
    {3, -2},
    {1, 1},
    {-2, 3},
    {0, -1},
};

StaticTask_t s_task_tcb;
StackType_t s_task_stack[kTaskStackWords]
    __attribute__((aligned(8), section(".sdram_bss")));
uint8_t s_shifted[FLOW_GRAY_PIXELS]
    __attribute__((aligned(32), section(".sdram_bss")));

volatile uint32_t s_sensor_frames = 0;

void UartInit() {
    volatile uint32_t* const lpuart_ctrl =
        reinterpret_cast<volatile uint32_t*>(0x40090018u);
    *lpuart_ctrl = (1u << 18) | (1u << 19);
}

void UartPutChar(char ch) {
    volatile uint32_t* const lpuart_stat =
        reinterpret_cast<volatile uint32_t*>(0x40090014u);
    volatile uint32_t* const lpuart_data =
        reinterpret_cast<volatile uint32_t*>(0x4009001Cu);
    while ((*lpuart_stat & (1u << 23)) == 0u) {
    }
    *lpuart_data = static_cast<uint8_t>(ch);
}

void UartWrite(const char* s) {
    while (s && *s) {
        UartPutChar(*s++);
    }
}

void UartWritef(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    buf[n] = '\0';
    UartWrite(buf);
}

int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void BuildShiftedFrame(int ox, int oy) {
    for (int y = 0; y < FLOW_GRAY_H; ++y) {
        const int sy = clampi(y - oy, 0, FLOW_GRAY_H - 1);
        for (int x = 0; x < FLOW_GRAY_W; ++x) {
            const int sx = clampi(x - ox, 0, FLOW_GRAY_W - 1);
            s_shifted[y * FLOW_GRAY_W + x] =
                g_sentai_emu_flow_base_y80x60[sy * FLOW_GRAY_W + sx];
        }
    }
}

int PublishFrame(int ox, int oy) {
    BuildShiftedFrame(ox, oy);
    uint32_t mask = sentai_prep_tick_frame();
    if ((mask & (1u << SENTAI_PREP_SLOT_FLOW_GRAY_80x60)) == 0) {
        return -1;
    }
    int w = 0;
    int h = 0;
    uint8_t* dst = sentai_prep_slot_begin_write(
        SENTAI_PREP_SLOT_FLOW_GRAY_80x60, &w, &h);
    if (!dst || w != FLOW_GRAY_W || h != FLOW_GRAY_H) {
        return -2;
    }
    memcpy(dst, s_shifted, FLOW_GRAY_PIXELS);
    sentai_prep_slot_commit(SENTAI_PREP_SLOT_FLOW_GRAY_80x60);
    ++s_sensor_frames;
    return 0;
}

int WaitFlowProcessed(uint32_t want_seq, int timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t total = pdMS_TO_TICKS((uint32_t)timeout_ms);
    while ((xTaskGetTickCount() - start) < total) {
        volatile flow_shared_t* sh = &FLOW_SHARED();
        if (sh->frames_processed >= want_seq) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return -1;
}

void SnapshotFlowResult() {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    g_sentai_emu_flow_last_dx_q1000 = sh->last_dx;
    g_sentai_emu_flow_last_dy_q1000 = sh->last_dy;
    g_sentai_emu_flow_last_conf = sh->last_confidence;
}

int AbsI32(int32_t v) {
    return v < 0 ? -v : v;
}

int ValidateOffsets() {
    uint32_t result_seq = 0;
    for (uint32_t i = 0; i < (uint32_t)(sizeof(kOffsets) / sizeof(kOffsets[0])); ++i) {
        int rc = PublishFrame(kOffsets[i].x, kOffsets[i].y);
        if (rc != 0) return 100u + (int)i;
        ++result_seq;
        if (WaitFlowProcessed(result_seq, 5000) != 0) {
            return 200u + (int)i;
        }
        SnapshotFlowResult();
        const int exp_dx = (i == 0) ? 0 : (int)kOffsets[i].x - (int)kOffsets[i - 1].x;
        const int exp_dy = (i == 0) ? 0 : (int)kOffsets[i].y - (int)kOffsets[i - 1].y;
        const int32_t want_dx_q1000 = (int32_t)exp_dx * 1000;
        const int32_t want_dy_q1000 = (int32_t)exp_dy * 1000;
        int match = 0;
        if (i == 0) {
            match = (AbsI32(g_sentai_emu_flow_last_dx_q1000) <= kFlowToleranceQ1000 &&
                     AbsI32(g_sentai_emu_flow_last_dy_q1000) <= kFlowToleranceQ1000);
        } else {
            match = (g_sentai_emu_flow_last_conf >= kFlowMinConfidence &&
                     AbsI32(g_sentai_emu_flow_last_dx_q1000 - want_dx_q1000) <= kFlowToleranceQ1000 &&
                     AbsI32(g_sentai_emu_flow_last_dy_q1000 - want_dy_q1000) <= kFlowToleranceQ1000);
        }
        UartWritef("FLOW_VALIDATE frame=%lu exp_dx=%d exp_dy=%d dx_q1000=%ld dy_q1000=%ld conf=%lu match=%d\r\n",
                   (unsigned long)(i + 1),
                   exp_dx, exp_dy,
                   (long)g_sentai_emu_flow_last_dx_q1000,
                   (long)g_sentai_emu_flow_last_dy_q1000,
                   (unsigned long)g_sentai_emu_flow_last_conf,
                   match);
        if (!match) return 300u + (int)i;
    }
    g_sentai_emu_flow_detect_ok = 1;
    return 0;
}

int MeasureFps() {
    const uint32_t start_processed = FLOW_SHARED().frames_processed;
    const TickType_t t0 = xTaskGetTickCount();
    for (int i = 0; i < kMeasureFrames; ++i) {
        const Offset& off = kOffsets[i % (int)(sizeof(kOffsets) / sizeof(kOffsets[0]))];
        int rc = PublishFrame(off.x, off.y);
        if (rc != 0) return 400 + i;
        uint32_t want = start_processed + (uint32_t)i + 1u;
        if (WaitFlowProcessed(want, 5000) != 0) {
            return 500 + i;
        }
    }
    TickType_t elapsed = xTaskGetTickCount() - t0;
    if (elapsed == 0) elapsed = 1;
    g_sentai_emu_flow_elapsed_ms = (uint32_t)elapsed;
    g_sentai_emu_flow_completed = kMeasureFrames;
    g_sentai_emu_flow_fps_x100 =
        (uint32_t)((uint64_t)kMeasureFrames * 100000ull /
                   (uint64_t)elapsed);
    SnapshotFlowResult();
    UartWritef("FLOW_FPS frames=%lu elapsed_ms=%lu fps_x100=%lu last_dx_q1000=%ld last_dy_q1000=%ld conf=%lu\r\n",
               (unsigned long)g_sentai_emu_flow_completed,
               (unsigned long)g_sentai_emu_flow_elapsed_ms,
               (unsigned long)g_sentai_emu_flow_fps_x100,
               (long)g_sentai_emu_flow_last_dx_q1000,
               (long)g_sentai_emu_flow_last_dy_q1000,
               (unsigned long)g_sentai_emu_flow_last_conf);
    return 0;
}

void BenchmarkTask(void*) {
    UartInit();
    UartWrite("\r\nSentAI EMU RUNTIME FLOWTASK S214\r\n");
    sentai_prep_init();
    g_sentai_emu_boot_state = kBootWaitingBase;
    UartWrite("FLOW_WAIT_BASE\r\n");
    while (!g_sentai_emu_flow_base_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    UartWrite("FLOW_BASE_READY\r\n");

    int rc = sentai_flow_start(-1);
    UartWritef("FLOW_START rc=%d\r\n", rc);
    if (rc != 0) {
        g_sentai_emu_flow_fail_code = (uint32_t)(10 - rc);
        g_sentai_emu_boot_state = kBootFailed;
        return;
    }
    g_sentai_emu_boot_state = kBootFlowStarted;
    vTaskDelay(pdMS_TO_TICKS(20));

    rc = ValidateOffsets();
    if (rc != 0) {
        g_sentai_emu_flow_fail_code = (uint32_t)rc;
        g_sentai_emu_boot_state = kBootFailed;
        UartWritef("FLOW_VALIDATE_FAIL code=%d\r\n", rc);
        return;
    }
    g_sentai_emu_boot_state = kBootValidated;
    UartWrite("FLOW_VALIDATE_PASS\r\n");

    rc = MeasureFps();
    if (rc != 0) {
        g_sentai_emu_flow_fail_code = (uint32_t)rc;
        g_sentai_emu_boot_state = kBootFailed;
        UartWritef("FLOW_FPS_FAIL code=%d\r\n", rc);
        return;
    }
    g_sentai_emu_boot_state = kBootMeasured;
    UartWrite("FLOW_DONE\r\n");

    uint32_t frames = 0;
    uint32_t grab_fail = 0;
    uint32_t grab_streak = 0;
    int running = 0;
    sentai_flow_pub_stats(&frames, &grab_fail, &grab_streak, &running);
    UartWritef("FLOW_STATS frames=%lu grab_fail=%lu grab_streak=%lu running=%d\r\n",
               (unsigned long)frames,
               (unsigned long)grab_fail,
               (unsigned long)grab_streak,
               running);
    int stop_rc = sentai_flow_stop();
    UartWritef("FLOW_STOP rc=%d\r\n", stop_rc);

    for (;;) {
        ++g_sentai_emu_heartbeat;
        g_sentai_emu_last_tick = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

}  // namespace

extern "C" uint32_t sentai_cam_get_sensor_frames(void) {
    return s_sensor_frames;
}

extern "C" int sentai_cam_grab_latest(uint8_t** raw) {
    (void)raw;
    return -1;
}

extern "C" void sentai_cam_return_raw(int idx) {
    (void)idx;
}

extern "C" uint32_t sentai_cam_get_frame_seq(void) {
    return s_sensor_frames;
}

extern "C" int sentai_cam_is_initialized(void) {
    return 1;
}

extern "C" int sentai_detection_is_running(void) {
    return 0;
}

extern "C" int sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                                uint8_t* dst, int dw, int dh) {
    (void)src;
    (void)sw;
    (void)sh;
    (void)dst;
    (void)dw;
    (void)dh;
    return -1;
}

extern "C" int sentai_logf(const char* tag, const char* fmt, ...) {
    char msg[160];
    int off = 0;
    if (tag && tag[0]) {
        off = snprintf(msg, sizeof(msg), "[%s] ", tag);
        if (off < 0) off = 0;
        if (off >= (int)sizeof(msg)) off = (int)sizeof(msg) - 1;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg + off, sizeof(msg) - (size_t)off, fmt, ap);
    va_end(ap);
    if (n < 0) return -1;
    int len = off + n;
    if (len >= (int)sizeof(msg)) len = (int)sizeof(msg) - 1;
    msg[len++] = '\r';
    if (len < (int)sizeof(msg)) msg[len++] = '\n';
    if (len < (int)sizeof(msg)) msg[len] = '\0';
    UartWrite(msg);
    return len;
}

extern "C" int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t task = xTaskCreateStatic(BenchmarkTask, "flow_bench",
                                          kTaskStackWords, nullptr,
                                          tskIDLE_PRIORITY + 2,
                                          s_task_stack, &s_task_tcb);
    if (!task) {
        g_sentai_emu_boot_state = kBootFailed;
        while (true) {
        }
    }
    g_sentai_emu_boot_state = kBootTaskCreated;
    vTaskStartScheduler();
    g_sentai_emu_boot_state = kBootSchedulerReturned;
    while (true) {
    }
}
