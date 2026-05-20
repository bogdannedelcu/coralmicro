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

// Diagnostic counters — observe whether the handler is hit at all,
// what message types arrive, and what payload bytes look like.
static volatile uint32_t s_handler_calls  = 0;   // total invocations
static volatile uint32_t s_handler_done   = 0;   // kBenchDone count
static volatile uint32_t s_handler_other  = 0;   // any other type
static volatile uint32_t s_last_type_byte = 0;
static volatile uint32_t s_last_block     = 0;

void handle_m4_message_(const uint8_t data[coralmicro::kIpcMessageBufferDataSize]) {
    s_handler_calls++;
    const auto* msg = reinterpret_cast<const M4BenchAppMessage*>(data);
    s_last_type_byte = (uint32_t)data[0];
    s_last_block     = (uint32_t)msg->block;
    if (msg->type != M4BenchMessageType::kBenchDone) {
        s_handler_other++;
        return;
    }
    s_handler_done++;
    s_result_cycles = msg->cycles;
    s_result_n_dets = msg->n_dets;
    if (s_result_sem) {
        xSemaphoreGive(s_result_sem);
    }
}

}  // namespace

extern "C" uint32_t sentai_m4_bench_handler_calls(void)  { return s_handler_calls; }
extern "C" uint32_t sentai_m4_bench_handler_done(void)   { return s_handler_done; }
extern "C" uint32_t sentai_m4_bench_handler_other(void)  { return s_handler_other; }
extern "C" uint32_t sentai_m4_bench_last_type(void)      { return s_last_type_byte; }
extern "C" uint32_t sentai_m4_bench_last_block(void)     { return s_last_block; }

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
//
// CRITICAL FIX 2026-05-20 (OP-S10-W18 debug session): block was being
// clamped to [3, 511] BEFORE being sent over IPC, which silently
// destroyed all sentinel-routed dispatch paths on M4:
//
//   0xCAFE  (51966)  — IPC smoke test         } all >> 511 → clamped
//   0xC100..0xC108   — WhyCon synth+threshold } to 511, which falls
//   0xF10F  (61711)  — Flow SAD exhaustive    } into the default
//   0xF1D7  (61911)  — Flow SAD on M4 DTCM    } ArUco threshold path
//   0xF1DD  (61917)  — Flow diamond search    } @ block=511.
//
// This invalidates the M4 numbers reported in W16 (ArUco block sweep),
// W17-T2 (WhyCon Phase A), W17-T4 (Flow SAD ablations) — they all
// measured the same code path (ArUco threshold @ b=511) regardless
// of the sentinel.  Numbers must be re-benched after this fix.
//
// HARD-RULE for any future sentinel range over IPC: clamp logic ALWAYS
// LIVES IN THE WORKER, NOT THE HOST.  Host transmits raw block bytes.
extern "C" int sentai_m4_bench_run(int block, uint32_t* out_cyc,
                                     uint32_t timeout_ms) {
    if (!s_m4_started || !out_cyc) return 0;
    /* Clamp removed — was destroying sentinel routing.  Worker applies
     * its own clamp for the legitimate ArUco-threshold range. */

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
