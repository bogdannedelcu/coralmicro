---
name: Arena in OCRAM — SHIPPED Phase 1
description: 2026-04-23. Arena moved SDRAM→OCRAM (800 KB), s_tpu_input_buf_single removed. +0.95 FPS (41.85→42.8). Contention now dominated by instruction-fetch, not input.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
**Status: SHIPPED** (session 2026-04-23, this session).

## What shipped

- `tensor_arena` in OCRAM via new `.tpu_arena` linker section
  (800 KB) — [sentai_runtime.cc:806-808](../examples/sentai_runtime/sentai_runtime.cc#L806)
- `s_tpu_input_buf_single` removed (786 KB OCRAM freed) — [detection_task.cc:75-84](../examples/sentai_runtime/detection_task.cc#L75)
- PrepTask writes quantized output directly into input tensor's arena
  slot (which is in OCRAM) — saves one layer of indirection
- InferTask calls `sentai_tpu_invoke_with_input(tensor_buf)` with
  the same arena pointer; the pointer-swap inside is effectively a
  no-op (identity) but kept for the early_release callback hook
- Legacy `s_staging_buf` (for non-direct path) kept at 786 KB in
  SDRAM via `.tpu_input` section (now mapped to `m_sdram`)
- `kTensorArenaSize` in sentai_slow_bridge.cc updated 8 MB → 800 KB

## Measured impact

- Baseline: 41.85 FPS, 0 fails, 20 s sustained (_t_early_long)
- After: **42.8 FPS, 0 fails, 18 s sustained**
- Δ = **+0.95 FPS (+2.3 %)** — marginal but non-negative, stable

## Why the win is small

Contention during invoke is USB reading **TPU instructions (372 KB)**
from `g_model_data` heap (SDRAM).  Not input.  Arena-in-OCRAM
removed input/activation contention (already small) but left
instruction-fetch contention intact.

To push beyond 43 FPS: need to cache instructions in OCRAM too.
Requires ~372 KB free OCRAM + engineering to extract and relocate
TPU instruction blob from FlatBuffer at load time.  Current OCRAM
free ~140 KB.  Not enough.

## Known gotcha

Re-importing `_t_early_long` in same boot triggers `sentai.tpu.load()`
twice.  Second load (delete + new interpreter) with arena in OCRAM
causes subsequent invokes to fail.  Workaround: fresh reflash between
tests.  Does not affect production (load-once-per-boot pattern).
Root cause: likely cache coherency or leftover state in arena
during interpreter teardown.  Debug in future session.

## Not attempted yet (future levers)

1. **Instruction blob in OCRAM** — biggest remaining lever; needs
   extract-from-FlatBuffer + ~200 KB more OCRAM freed (shrink arena,
   move tensorflow_code back to SDRAM).
2. **Double-buffer via SDRAM staging + memcpy** — would allow
   PrepTask and InferTask true parallelism, but gain is marginal
   since invoke dominates (period = max(prep, invoke) = invoke).
3. **Instruction caching from first invoke** — store instructions
   submitted to TPU once, reuse on subsequent invokes (similar to
   desc_cache which failed for yolo_1).

## How to apply

Current shipped state is **the new baseline**: 42.8 FPS, 0 fails.
Future session starts here.  If instruction-blob-in-OCRAM is
pursued, note that 372 KB is the target extraction size from
FlatBuffer descriptors.

Git state: uncommitted changes on `feature/ov5640-camera-support`.
Stable fallback: `git stash` or revert the 3 changed files
(sentai_runtime.cc, detection_task.cc, MIMXRT1176xxxxx_cm7_ram_mp.ld,
sentai_slow_bridge.cc).
