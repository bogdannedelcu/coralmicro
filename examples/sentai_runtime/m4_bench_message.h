// m4_bench_message.h — shared message types for the M7 ↔ M4 ArUco-
// threshold benchmark (OP-S10-W16-T3).  Pattern lifted from
// examples/multi_core_ipc/example_message.h — same kIpcMessageBufferDataSize
// constraint applies (~80 bytes).

#ifndef SENTAI_M4_BENCH_MESSAGE_H_
#define SENTAI_M4_BENCH_MESSAGE_H_

#include <stdint.h>

#include "libs/base/ipc_message_buffer.h"

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
