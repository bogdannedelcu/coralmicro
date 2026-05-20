// flow_bench_shared.h — OP-S10-W17-T4 Flow SAD ablation.
//
// Shared scratch OCRAM address borrowed from the M7 .tpu_input region
// (900 KB at 0x20243400-0x20324400).  Both cores point to the SAME
// physical OCRAM address for the ablation so the comparison measures
// pure core-architecture difference, not memory-tier difference.
//
// HARD-RULE EXCEPTION vs the RPMSG "never hardcode cross-core
// addresses" rule (from OP-S10-W16-T3 — feedback_rpmsg_cross_core
// _alignment.md):
//
//   - This is a one-off bench scratch, not a long-running cross-core
//     data structure.
//   - The two cores DO NOT run concurrently during the bench (M7 is
//     idle while M4's worker task runs, and vice versa).
//   - The M7 linker's __tpu_input_start__ symbol is M7-only; the M4
//     linker has no .tpu_input section, so a shared linker symbol is
//     not available.  Hardcoded compile-time constant is the
//     pragmatic solution.
//   - The TPU is guaranteed idle during the bench (the bench REPL
//     command runs in single-shot mode, no concurrent pipeline.start).
//
// Layout inside .tpu_input (FLOW_GRAY_W=80, FLOW_GRAY_H=60):
//   +0x0000 .. +0x12C0  curr (4800 B)
//   +0x12C0 .. +0x2580  prev (4800 B)
//   total                            9.6 KB used of 900 KB available.

#ifndef FLOW_BENCH_SHARED_H_
#define FLOW_BENCH_SHARED_H_

#define FLOW_BENCH_OCRAM_BASE  0x20243400u   /* M7 .tpu_input start */
#define FLOW_BENCH_GRAY_W      80
#define FLOW_BENCH_GRAY_H      60
#define FLOW_BENCH_GRAY_PIX    (FLOW_BENCH_GRAY_W * FLOW_BENCH_GRAY_H)

#define FLOW_BENCH_CURR_PTR    ((uint8_t*)(FLOW_BENCH_OCRAM_BASE + 0))
#define FLOW_BENCH_PREV_PTR    ((uint8_t*)(FLOW_BENCH_OCRAM_BASE + FLOW_BENCH_GRAY_PIX))

/* SAD block + search range — match flow_task.cc kBlockW / kSearchRange.
 * Hardcoded here so the M4 port doesn't need to include flow_task.cc. */
#define FLOW_BENCH_BLOCK_W     32
#define FLOW_BENCH_BLOCK_H     32
#define FLOW_BENCH_SEARCH      12
#define FLOW_BENCH_SURF_DIM    (2 * FLOW_BENCH_SEARCH + 1)

#endif  /* FLOW_BENCH_SHARED_H_ */
