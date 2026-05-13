# s111 — M7 ArUco PXP debug + first PXP-Y8 thresholder

## What this proves

`sentai.diag.aruco_bench` runs 4 adaptive-threshold variants + Sobel
edge on a 320×240 synthetic grayscale image and reports DWT cycle
counts. PXP-HW variant was originally broken (timeout after 125 ms).
Root cause + fix below.

## Root cause (build #1270 → #1271)

PXP is held in **SFTRST + CLKGATE** (`PXP_CTRL = 0xC0000000`) until
`BOARD_InitPxp()` runs.  That function is only invoked from
`CameraTask::HandleEnableRequest()` — i.e. when MicroPython calls
`sentai.camera.init()`.  If `sentai.diag.aruco_bench` is called *before*
the camera is enabled (typical fresh-boot bench), then `PXP_Start()` is
a no-op against a powered-down peripheral and the busy-wait spins
through the full 125 ms DWT timeout.

**Fix**: call `PXP_Init(DEMO_PXP)` once at the top of `aruco_bench_run()`.
Idempotent — safe if the camera path already ran it.

Diagnostic infrastructure added: the bench captures `PXP_CTRL` and
`PXP_STAT` before+after the trigger and the busy-wait iteration count.
Pre-fix: `CTRL=0xC0000000, STAT=0x0, iters=2 751 555` (full timeout).
Post-fix: `CTRL_before=0x0, STAT_after=0x80001 (IRQ0 set), iters=3682`.

## Results (build #1271, 320×240 Y8)

| Kernel              | Time (µs) | vs naive |
|---------------------|-----------|----------|
| scan (mem floor)    |       790 | —        |
| edge (3×3 Sobel)    |     3 512 | —        |
| naive 7×7 box       |    12 103 | 1.00×    |
| Bradley integral    |    14 759 | 0.82× (SLOWER — SDRAM random access defeats O(1)) |
| separable 7+7       |    14 131 | 0.86× (SLOWER — buffer-bound) |
| **PXP HW + compare**|   **2 616** | **4.63×** |

Bradley & separable losing to naive is the canonical
"big-O ≠ wall time on memory-constrained MCU" result — both touch
extra SDRAM buffers (integral image 307 KB; horiz sums 154 KB) and
the cache (16 KB L1-D) can't absorb that.

The PXP path's 2.6 ms is dominated by the post-PXP CPU compare loop
(76 800 nearest-neighbor lookups vs the 80×60 ref). PXP HW itself is
~tens of µs.  SIMD'ing the compare → sub-1 ms is plausible.

## How to run

```bash
# board alive on /dev/ttyACM0 with build #1271+
python3 read_pxp_dbg.py
```

Pass criteria: `pxp_stat_after & 0x1 == 1` (CompleteFlag set) and
`thresh_pxp_us < 5000` (no timeout).

## Phase 1-6 optimisation sweep (build #1272..#1278)

After unblocking PXP, six incremental optimisations were added to
`aruco_bench.cc`, each measured in isolation:

| Phase | Build | Change | What it tests |
|-------|------:|--------|---------------|
| 1 | 1272 | SIMD compare (`__UQADD8` + `__USUB8` + `__SEL`) replaces scalar `?:` per-pixel | Pure SIMD payoff on the post-PXP compare |
| 2 | 1273 | `-O3 -funroll-loops` for `aruco_bench.cc` only | Compiler unroll on SDRAM-resident kernels |
| 3 | 1275 | `s_pxp_meanref` moved SDRAM → DTCM (via `.ocram_bss` orphan landing) | Cache-line-eviction reduction |
| 4 | 1276 | New PXP kernel: 60×60 ROI crop + scale to 32×32 Y8 (axis-aligned quad rectify) | Per-marker decode budget |
| 5 | 1277 | SIMD compare with 60-row SDRAM→DTCM strip staging (CPU memcpy) | Is SDRAM source-read the bottleneck? |
| 6 | 1278 | CMSIS-DSP BasicMath + Statistics q7 ops added to `libs_CMSIS-m7`; demo runs `arm_offset_q7` + `arm_max_q7` + `arm_min_q7` over 76 800 bytes | Reusable library primitives for future contour/decode steps |

### Final results (build #1278, 320×240 Y8 synthetic, min-of-3 trials)

| Kernel | µs | Δ vs naive |
|---|---:|---|
| scan (memory floor) | 0\* | — |
| edge 3×3 Sobel | 3 052 | — |
| naive 7×7 box | 15 525 | 1.00× |
| Bradley integral | 12 076 | 1.29× faster (now beats naive under -O3) |
| separable 7+7 | 13 725 | 1.13× |
| PXP HW + scalar compare | 2 238 | **6.94×** |
| **PXP HW + SIMD compare** | **1 486** | **10.45×** |
| **PXP HW + SIMD + DTCM stage** | **1 096** | **14.17×** |
| PXP rectify 60×60→32×32 | 37 | per-marker decode cost |
| CMSIS-DSP 3-pass demo (76 800 B) | 4 507 | reference for library ops |

\*Phase 2's `-O3 -funroll-loops` aggressively folded the volatile-sink
scan loop down to 0 µs.  The pre-Phase-2 floor was 790 µs.

### Headline win

- **PXP+SIMD+DTCM-staged threshold: 1.1 ms.**  Original naive CPU
  threshold was 12.1 ms.  Net speedup: 11× on the heaviest stage.
- ArUco preprocessing budget per frame: 1.1 ms threshold + 3.1 ms edge
  + 0.04 ms × N rectify ≈ **4.1 ms** ⇒ 240 FPS ceiling.  Camera caps
  at 30 FPS, so the full ArUco pipeline (contour + PnP not yet
  implemented) has ~30 ms slack per frame on M7.

### Phase-by-phase delta (which lever moved the needle)

| From → To | PXP-SIMD path µs | Δ µs | Lesson |
|---|---:|---:|---|
| Phase 0 (PXP scalar baseline) | 2 600 | — | starting point |
| Phase 1: SIMD compare | 1 701 | **-899** | quad-byte packed compare ≈ 16× on the loop alone |
| Phase 2: -O3 on file | 1 576 | -125 | compiler unroll, modest |
| Phase 3: ref in DTCM | 1 490 | -86 | cache-eviction reduction, modest |
| Phase 5: src staged to DTCM | 1 096 | **-394** | SDRAM source read was real |
| total saved | | **-1 504** | -58% |

### Lessons (post-mortem)

1. **Big-O does not predict wall time on Cortex-M7 + SDRAM.**  Bradley
   (theoretically O(1) per output pixel) lost to naive O(K²) box
   filter at -Os because its 307 KB integral image blew the 16 KB L1-D.
   -O3 partially restored the algorithmic advantage but only because
   the inner loop fits cleanly in I-cache.
2. **-O3 isn't free.**  Naive 7×7 got *slower* with -O3 + funroll-loops
   (+28%) because the bigger unrolled code thrashed SDRAM I-cache.
3. **PXP HW is the single biggest lever** — pure HW downscale is ~tens
   of µs; everything else is the CPU post-process.  Any future ArUco
   work should route through PXP wherever possible.
4. **SIMD compare via `__UQADD8/USUB8/SEL` is ~16× the scalar `?:`** —
   the M7 DSP-extension is severely underused in this codebase
   (only flow_task.cc and modsentai_hal.cc used intrinsics before).
5. **The `.ocram_bss` section name is a misnomer in this codebase** —
   the linker has no explicit rule for it, so it orphan-lands in
   m_data (DTCM).  Real OCRAM (m_ocram) is reserved for `.tpu_input`
   and the camera/MicroPython/aifes/audio sections per
   `paper/memory_map.md`.  Edge cases: edgetpu_driver.cc's 32 KB
   BulkTransferBuffer ALSO ends up in DTCM despite its OCRAM
   comment.  Beware when sizing new buffers in `.ocram_bss`.
6. **DTCM stage cost ≈ 200 µs** for 76 800-byte memcpy, but the
   SIMD-on-DTCM saves ~600 µs of SDRAM contention.  Net 386 µs win.
7. **eDMA not needed for the current win** — synchronous CPU memcpy
   into DTCM was enough.  Asynchronous eDMA + ping-pong would
   overlap memcpy with SIMD and could push another ~200 µs out, but
   diminishing returns vs implementation cost.
8. **CMSIS-DSP q7 ops are a fair reference point** — `arm_offset_q7`
   alone over 76 800 bytes is ~1.5 ms, comparable to our
   hand-written `__UQADD8` path on SDRAM (~1.5 ms in
   `k_adaptive_threshold_pxp_simd`).  CMSIS is slower than the
   DTCM-staged path because it doesn't stage; if a future kernel
   wants the convenience of named ops, factor in this overhead.

### Production-pipeline note

The flow stack already runs PXP continuously at 30 FPS to scale
640×480 → 80×60 RGB888 into `flow_task.cc:s_pxp_scratch` (SDRAM,
not OCRAM despite some session-memory claims).  A real
ArUco-on-M7 production path should **reuse that buffer** instead of
spinning up a second PXP call per frame — pay one RGB→Y conversion
in the SIMD loop (extra ~150 µs) and free the PXP for other work.
This bench measures kernels in isolation; integration is a
separate task.

## Files

- `read_pxp_dbg.py` — host driver, reads the bench dict line-by-line
  to avoid REPL chunking.
