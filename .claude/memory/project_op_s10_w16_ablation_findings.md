---
name: op-s10-w16-ablation-findings-2026-05-19
description: "OP-S10-W16 ablation FOURTH revision 2026-05-19 — earlier M4 measurements were DCE-artefacts.  The M4 bench harness wrote `s_binary` but never read it back; with LTO+O3 the M4 compiler eliminated the entire Phase-2 store loop, so previous M4 numbers measured Phase 1 (integral image) only.  Fixed at commit 576fc1d5 via XOR-fold sink.  Corrected production numbers: M7 SDRAM @ 320x240 = 12 ms, M4 OCRAM @ 320x240 = 29 ms — M7 is 2.4x faster.  M4 offload is NO LONGER a win for ArUco threshold."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Operator-driven ablation 2026-05-19, FOUR iterations.  Each round
corrected the previous narrative.  The fourth round (576fc1d5)
exposed a DCE artefact in rounds 1-3 that made M4 look ~3x faster
than it really is.  THIS file reflects the honest final picture.

## Final ablation table (all Bradley adaptive threshold)

| Setup                              | Wall-clock | Cycles  | Cyc/px | Honest? |
|------------------------------------|-----------:|--------:|-------:|--------:|
| M7 SDRAM, cache ON @ 320×240       |    12 ms   |  9.6 M  |  125   |   YES   |
| M7 SDRAM, cache OFF @ 320×240      |   148 ms   | 118.6 M | 1544   |   YES   |
| **M4 OCRAM @ 320×240 (fixed)**     |  **29 ms** | 11.7 M  |  152   | **YES** |
| ~~M4 OCRAM @ 320×240 (DCE)~~       |  9.75 ms   |  3.9 M  |   51   |    NO   |
| ~~M4 OCRAM @ 80×60 (DCE)~~         |  250 us    |  100 k  |   21   |    NO   |
| ~~M7 OCRAM @ 160×120 (vs DCE'd M4)~| 212 us     |   170 k |   8.8  | M7 yes  |
| ~~M4 OCRAM @ 160×120 (DCE)~~       |  2422 us   |   970 k |   50   |    NO   |

## What we measured + concluded

### Round 4 (commit 576fc1d5): DCE artefact uncovered

The bench harness in `m4_aruco_bench.cc` ran the threshold kernel
which writes results into `s_binary`.  **Nothing reads `s_binary`
after the kernel** — the M4 worker just measures cycles and sends
back via IPC.  Under M4 LTO + `-O3`, the compiler observed this
and performed dead-code elimination on the entire Phase-2 loop
body (compare gray vs box mean + store binary result).

`arm-none-eabi-nm` confirmed: `_ZL8s_binary` symbol was **absent**
from the M4 ELF.  The kernel was actually measuring **only Phase 1**
(integral image build, which has live uses via memory write to
`s_integral`).

Fix: XOR-fold `s_binary` into a `volatile s_bench_sink` after the
timed region.  This keeps the Phase-2 stores load-bearing without
forcing per-byte `volatile` stores (which would have biased the
measurement pessimistically with no store coalescing).

Honest re-measurement build #1380:
- M4 OCRAM @ 320×240 = **29 ms / 11.7 M cycles** (was "9.75 ms")
- Per-pixel = **152 cyc/px** at 400 MHz

**Production verdict reversed AGAIN:** M7 SDRAM (12 ms) is now
2.4× faster than M4 OCRAM (29 ms).  M4 offload of ArUco threshold
is **no longer a win** at 320×240.

### Round 1 (commit 862b5c91): "M4 1.6x faster than M7"

**WRONG**.  Compared M4-OCRAM (9.75 ms @ 320×240) against M7's
aruco_bench kernel (16 ms) — a DIFFERENT implementation, not
sentai_aruco.cc's production threshold (which actually runs at
12 ms in SDRAM).  Apples to oranges.

### Round 2 (commit e8d8a584): "M7 cache disable + M4 SIMD attempt"

- M7 cache OFF: 12x slower (148 ms) — cache is NOT thrashing,
  it's essential to hide SDRAM 50ns access penalty.
- M4 SIMD attempt: 2.5x SLOWER than M4 scalar — pack overhead
  doesn't pay off on M4F single-issue.

Narrative shift #1: M4 doesn't beat M7 because of "no cache".
M4 just happens to have OCRAM-backed integral (fast).

### Round 3 (commit 940611e2): "M7 with OCRAM @ 160×120"

THE definitive measurement.  Both cores running plain scalar
Bradley with integral in their respective OCRAM.

- M7 OCRAM: **212 us** (8.8 cyc/px, INCREDIBLY efficient)
- M4 OCRAM: **2422 us** (50 cyc/px)

**M7 is 11x faster** than M4 when both can use OCRAM.

Why M7 wins:
1. **Cache exploitation**: Phase 1 write-allocates integral
   into D-cache; Phase 2 reads are L1 hits → 1-cycle access.
2. **800 MHz clock** vs M4's 400 MHz (2x advantage).
3. **Superscalar dual-issue pipeline** on M7 vs M4F
   single-issue.

M4 only gets the OCRAM 3-cyc access; no cache reuse, no clock,
no dual-issue.

## So why NOT move ArUco to M4? (corrected verdict)

**M4 is honestly slower than M7 for this workload**, end of story:
- M7 SDRAM: **12 ms** (production, cache-warmed)
- M4 OCRAM: **29 ms** (best possible — buffers on-die)

Even with M7 paying the SDRAM access penalty and M4 enjoying
OCRAM 3-cyc access, M7 wins by 2.4×.  The factors:
1. M7 clock 800 MHz vs M4 400 MHz (2× advantage).
2. M7 D-cache absorbs SDRAM penalty for hot working-set lines;
   the 309 KB integral image fits well within 32 KB D-cache
   working-set behaviour during Phase 2 (row-stride access
   pattern keeps cache hot).
3. M7 superscalar dual-issue vs M4F single-issue.

**Implication for OP-S10-W16:** the WP's primary thesis — that M4
offload buys ~20% latency improvement — is FALSE.  The correct
move is to keep ArUco on M7 and look for other optimization
levers (smaller block, downscaled threshold, lower SafetyTask
period, model-side TPU staging shrink to free OCRAM for M7
integral).

## Implications for thesis chapter

The honest narrative for the embedded-engineering chapter:

> The dual-core RT1176 platform has a 800 MHz Cortex-M7 and a
> 400 MHz Cortex-M4F sharing the same OCRAM bank.  For the
> compute-bound ArUco adaptive-threshold kernel on a 320×240
> grayscale frame, the M7 outperforms the M4 by 2.4× even when
> the M4 enjoys uncontested OCRAM-backed working memory while
> the M7 is forced to use external SDRAM.  The M7 advantage
> compounds from (a) 2× clock, (b) D-cache absorbing the SDRAM
> 50 ns access penalty via spatial+temporal locality of the
> integral-image scan, and (c) superscalar dual-issue pipeline.
> Cortex-M4F offload is therefore not a viable accelerator for
> this workload class.  M4F's strength is reserved for
> low-latency reaction (interrupt-driven motor controllers,
> sensor fusion at <1 ms period) where the M7 cannot guarantee
> determinism due to cache/SDRAM jitter.

## Methodology lesson (worth its own thesis paragraph)

Round 1-3 measurements were INVALIDATED by a compiler dead-store
elimination artefact.  The bench harness wrote into an output
buffer but never read it; LTO+O3 saw this and removed the entire
Phase-2 store loop.  Lesson: every microbenchmark of a kernel
that "produces a side-effect buffer" must include a load-bearing
consumer of that buffer (XOR-fold to volatile sink works, full
volatile-array works but pessimistic, explicit asm-clobber works).
ALWAYS verify with `nm <obj>` that the output symbol survives
linking before trusting cycle counts.

## Diagnostic infrastructure preserved

- `sentai.aruco._verify_threshold(block)` — byte-equiv test (kept)
- `sentai.aruco._thresh_cycles()` — (old_cyc, new_cyc) pair (kept)
- `sentai.aruco._thresh_nocache(block)` — M7 cache disabled (kept)
- `sentai.diag.m4_aruco_bench(block)` — M4 OCRAM bench (kept, DCE-fixed)
- `sentai.diag.m4_bench_diag()` — M7 RX handler counters (kept)
- (removed) `sentai.aruco._thresh_ocram(block)` — M7 integral in
  OCRAM, dropped at 576fc1d5 along with the s_*_ocram buffers.

## Cross-refs

- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — M7 SIMD work.
- `[[op-s10-w16-multicore-2026-05-19]]` — parent WP.
- `[[rpmsg-cross-core-alignment-hard-rule-2026-05-19]]` — linker
  fix that unblocked the M4 IPC reply path.
- commits 862b5c91, e8d8a584, e51915bd, 940611e2, **576fc1d5** —
  the four ablation rounds (last commit invalidated R1-R3).
