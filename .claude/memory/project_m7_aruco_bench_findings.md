---
name: M7 ArUco feasibility bench — PXP+SIMD+DTCM = 11× win
description: 320×240 Y8 threshold across builds #1271..#1278. Final PXP+SIMD+DTCM-staged = 1.1 ms; naive CPU = 12 ms. -O3 made naive SLOWER (icache thrash).
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---

## Final result (build #1278, after 6-phase optimisation sweep)

| Kernel | µs | vs naive (P0) |
|---|---:|---|
| naive 7×7 (-Os) | 12 103 | 1.00× baseline |
| naive 7×7 (-O3) | 15 525 | **+28% SLOWER** (icache thrash) |
| Bradley | 12 076 | 1.00× (only beats naive under -O3) |
| separable | 13 725 | 0.88× |
| PXP scalar | 2 238 | 5.41× |
| **PXP + SIMD** | **1 486** | **8.14×** |
| **PXP + SIMD + DTCM-staged src** | **1 096** | **11.04×** |
| PXP rectify 60×60→32×32 | 37 | per-marker decode |
| edge 3×3 Sobel | 3 052 | — |

## Per-phase delta on PXP-SIMD path

P0 (2600) → P1 SIMD compare (1701, **-899**) → P2 -O3 (1576, -125) →
P3 ref→DTCM (1490, -86) → P5 src→DTCM staging (1096, **-394**).
Total: -58%.  P4 (PXP rectify, 37 µs) and P6 (CMSIS-DSP build) add
infra but don't change PXP-SIMD timing directly.

## Lessons

- **PXP HW is the single biggest lever.**  Pure HW downscale takes
  ~tens of µs; everything else is the CPU post-process.
- **`__UQADD8/USUB8/SEL` SIMD = ~16× scalar `?:`** on the compare
  loop.  M7 DSP-extension is underused in this codebase.
- **Big-O ≠ wall time on Cortex-M+SDRAM.**  Bradley (O(1)/px) loses
  to naive (O(K²)/px) because the 307 KB integral buffer thrashes
  the 16 KB L1-D.
- **`-O3` is not universally a win.**  Naive 7×7 got slower with -O3
  on this M7 because the unrolled inner loop blew SDRAM I-cache.
- **`.ocram_bss` is a misnomer in this codebase** — orphan-lands in
  m_data (DTCM), NOT m_ocram.  Real OCRAM is reserved for
  `.tpu_input` (916 KB) and named sections per
  `paper/memory_map.md`.  For small buffers (<10 KB) the DTCM
  landing is fine and actually faster than OCRAM for CPU-only
  access (single-cycle dedicated bus).
- **SDRAM source-read latency matters** — staging 76 800 B from
  SDRAM to DTCM via CPU memcpy first saves ~386 µs on the
  subsequent SIMD compare.  Net win after memcpy cost: ~190 µs.

## ArUco-on-M7 frame budget

- Threshold: 1.1 ms (PXP-SIMD-staged)
- Edge: 3.0 ms (3×3 Sobel)
- Rectify: 0.04 ms × N markers
- **Total preprocessing: ~4.1 ms/frame** → 240 FPS ceiling
- Camera caps at 30 FPS → ~30 ms slack per frame for contour finder + PnP

## Production-pipeline note

Flow stack already runs PXP at 30 FPS to produce 80×60 RGB888 into
`flow_task.cc:s_pxp_scratch` (SDRAM, despite some session-memory
claims of OCRAM).  Real ArUco production path should **reuse that
buffer** + pay one RGB→Y conversion in the SIMD loop, not run a
second PXP per frame.

Reference: `examples/sentai_runtime/experiments/s111_m7_aruco_pxp_dbg/`.
