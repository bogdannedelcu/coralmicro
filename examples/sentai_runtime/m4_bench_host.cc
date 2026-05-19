// m4_bench_host.cc — M7-side driver for the M4 ArUco-threshold cycle
// bench (OP-S10-W16-T3).
//
// Uses the canonical coralmicro IpcM7 API (StartM4 + MessageBuffer-
// based IPC) — same pattern as examples/multi_core_ipc.  No
// hardcoded mailbox addresses; the RPMSG infrastructure handles
// cross-core memory & cache.

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "libs/base/ipc_m7.h"
#include "examples/sentai_runtime/m4_bench_message.h"

namespace {

// One-shot result slot filled by the M4-message handler.
static SemaphoreHandle_t s_result_sem    = nullptr;   // binary, given when result lands
static volatile uint32_t s_result_cycles = 0;
static volatile uint16_t s_result_n_dets = 0;
static int               s_m4_started    = 0;

void handle_m4_message_(const uint8_t data[coralmicro::kIpcMessageBufferDataSize]) {
    const auto* msg = reinterpret_cast<const M4BenchAppMessage*>(data);
    if (msg->type != M4BenchMessageType::kBenchDone) return;
    s_result_cycles = msg->cycles;
    s_result_n_dets = msg->n_dets;
    // IpcM7 RX runs in a FreeRTOS *task*, not ISR — use plain
    // xSemaphoreGive (the …FromISR variant was the v2 bug).
    if (s_result_sem) {
        xSemaphoreGive(s_result_sem);
    }
}

}  // namespace

// Lazily boot M4 + register handler.  Idempotent.  Returns 1 on
// success, 0 if M4 didn't come up within `timeout_ms`.
extern "C" int sentai_m4_bench_start(uint32_t timeout_ms) {
    if (s_m4_started) return 1;
    if (!s_result_sem) {
        s_result_sem = xSemaphoreCreateBinary();
        if (!s_result_sem) return 0;
    }
    auto* ipc = coralmicro::IpcM7::GetSingleton();
    ipc->RegisterAppMessageHandler(handle_m4_message_);
    ipc->StartM4();
    // Wait for the M4's IPC framework to signal it's alive.
    if (!ipc->M4IsAlive(timeout_ms)) {
        return 0;
    }
    s_m4_started = 1;
    return 1;
}

// Run one bench iteration with the given block size.  out_cyc filled
// on success.  Returns 1 on success, 0 on timeout.
extern "C" int sentai_m4_bench_run(int block, uint32_t* out_cyc,
                                     uint32_t timeout_ms) {
    if (!s_m4_started || !out_cyc) return 0;
    if (block < 3) block = 3;
    if (block > 511) block = 511;

    // Drain any prior pending signal.
    xSemaphoreTake(s_result_sem, 0);
    s_result_cycles = 0;
    s_result_n_dets = 0;

    coralmicro::IpcMessage msg{};
    msg.type = coralmicro::IpcMessageType::kApp;
    auto* app = reinterpret_cast<M4BenchAppMessage*>(&msg.message.data);
    app->type   = M4BenchMessageType::kBenchGo;
    app->block  = (uint16_t)block;
    app->cycles = 0;
    app->n_dets = 0;
    coralmicro::IpcM7::GetSingleton()->SendMessage(msg);

    if (xSemaphoreTake(s_result_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return 0;
    }
    *out_cyc = s_result_cycles;
    return 1;
}

extern "C" int sentai_m4_bench_is_alive(void) {
    return s_m4_started;
}
