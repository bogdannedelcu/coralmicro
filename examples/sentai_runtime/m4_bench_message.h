// m4_bench_message.h — shared message types for the M7 ↔ M4 ArUco-
// threshold benchmark (OP-S10-W16-T3).  Pattern lifted from
// examples/multi_core_ipc/example_message.h — same kIpcMessageBufferDataSize
// constraint applies (~80 bytes).

#ifndef SENTAI_M4_BENCH_MESSAGE_H_
#define SENTAI_M4_BENCH_MESSAGE_H_

#include <stdint.h>

#include "libs/base/ipc_message_buffer.h"

/* ============================================================
 * Sentinel block values dispatched over IPC to M4 worker.
 *
 * NASA/JPL discipline (per `agent/embeded.md`): NO magic numbers in
 * code paths.  Every dispatch-sensitive constant lives here, named.
 *
 * Legitimate ArUco threshold range is [3, 511] (Bradley adaptive
 * block size).  Sentinel values are placed in non-overlapping
 * higher ranges so they can be distinguished cleanly by the
 * worker.  Host-side clamp is FORBIDDEN — see
 * feedback_ipc_clamp_in_worker_not_host hard-rule memory.
 * ============================================================ */

/* IPC smoke test — worker replies kBenchDone with cycles = MAGIC_REPLY. */
#define M4BENCH_SENTINEL_HELLO_WORLD   0xCAFEu
#define M4BENCH_SENTINEL_HELLO_REPLY   0xCAFEBABEu  /* cycles field marker */

/* WhyCon synth + Phase A (rolling Bradley threshold) — sentinel range,
 * N disks = (block & M4BENCH_WHYCON_N_MASK). */
#define M4BENCH_SENTINEL_WHYCON_LO     0xC100u   /* inclusive */
#define M4BENCH_SENTINEL_WHYCON_HI     0xC108u   /* inclusive (N=8) */
#define M4BENCH_WHYCON_N_MASK          0x000Fu

/* Flow SAD ablation sentinels.  See OP-S10-W18_flow_search.md §5. */
#define M4BENCH_SENTINEL_FLOW_OCRAM    0xF10Fu   /* exhaustive on shared OCRAM */
#define M4BENCH_SENTINEL_FLOW_DTCM     0xF1D7u   /* exhaustive on M4 local DTCM */
#define M4BENCH_SENTINEL_FLOW_DIAMOND  0xF1DDu   /* diamond (LDSP+SDSP) on OCRAM */

/* Block-size clamp range for the legitimate ArUco threshold path.
 * Applied INSIDE the worker, never on the host (HARD-RULE). */
#define M4BENCH_ARUCO_BLOCK_MIN        3
#define M4BENCH_ARUCO_BLOCK_MAX        511

enum class M4BenchMessageType : uint8_t {
    kBenchGo   = 1,   // M7 → M4: run threshold with this block size
    kBenchDone = 2,   // M4 → M7: report cycle delta + marker count
};

struct M4BenchAppMessage {
    M4BenchMessageType type;
    uint16_t           block;   // populated for kBenchGo (3..511)
    uint32_t           cycles;  // populated for kBenchDone (DWT M4 ticks)
    uint16_t           n_dets;  // populated for kBenchDone (currently 0;
                                //   bench doesn't run full PnP yet)
} __attribute__((packed));

static_assert(sizeof(M4BenchAppMessage) <=
              coralmicro::kIpcMessageBufferDataSize,
              "M4BenchAppMessage exceeds IPC message buffer payload");

#endif  // SENTAI_M4_BENCH_MESSAGE_H_
