---
name: op-s10-w15-arm-memory-budget
description: "OP-S10-W15 (open ToDo, filed 2026-05-19, not scheduled): ARM memory allocation + FreeRTOS task-priority budget WP.  Produce paper/arm_memory_budget.md design doc + audit scripts + CI soft-alarm assertions.  Promotion trigger: next build break OR thesis embedded chapter starts.  Anti-scope: design + audit only, no functional refactor.  Includes hard rule: hot/frequently-used buffers (camera blocks, gray, rgb slots) MUST be aligned to cache-line (32B on M7) — re-validate all such buffers under T1 audit script."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

WBS entry in `ideas/wbs.md` after the W10 Milestones block.
Operator-requested 2026-05-19; status ⬜ TODO (open, not
scheduled).

## Motivation

T22 ([[t22-arm-build-fix-2026-05-19]]) restored the ARM build
reactively after W12/W13/W14 silently overflowed 3 link regions.
Every new subsystem grew static buffers / cold-path code without a
global memory-budget view.  T22 bought margin (post-W14: ~30 KB
m_data, ~7 KB m_text, ~3 MB m_sdram); the NEXT subsystem will
repeat the break unless the project documents an allocation
policy.

## Scope (when promoted)

Produce a `paper/arm_memory_budget.md` design doc fixing:

1. **Memory map intent per region**: ITCM `.ramfunc` / DTCM
   `m_data` / OCRAM `m_ocram` / SDRAM `m_sdram` / SDRAM-uncached
   `m_ncamera` / heap `m_heap`.  Rationale per category — DMA
   reach, cache coherence, bus contention class (the SEMC
   bottleneck lesson from [[tpu-pipeline-ocram-tensor]] is
   load-bearing).
2. **Task priority + stack table**: every FreeRTOS task
   (CameraTask, PrepTask, InferTask, FlowTask, FR drain,
   SafetyTask, SlamTask, CalibTask, MP REPL, HTTPServer, ...)
   with `(priority, stack size, stack location, period,
   justification)`.
3. **"Where does this go?" decision flowchart** for new
   subsystems — 5-step checklist (ISR? DMA-touched? >1 KB?
   per-frame? cache-coherent-required?) → region + section
   attribute + linker snippet.
4. **Budget table** with post-T22 baseline + soft-alarm
   thresholds (80 % / 90 % warn, 95 % CI fail).
5. **Rules of engagement** — when a new feature may expand the
   budget vs when it must reuse / shrink existing allocation.

## Cache-line alignment policy (operator-emphasised 2026-05-19)

**HARD RULE for any frequently-used / hot-path buffer**: align
to 32 bytes on M7 (one D-cache line).  Misaligned buffers cost:

- An extra cache-line refill on every wrap-around access.
- False sharing if two unrelated buffers share a 32 B line
  (writer on line evicts the reader's copy).
- DMA-coherency overhead — `SCB_CleanDCache_by_Addr` rounds up to
  cache-line boundaries; misaligned ranges flush extra bytes
  belonging to neighbouring data.

**Buffers that MUST be cache-line aligned** (re-validate under
T1 audit script):

- Camera ring (`m_ncamera` / `NonCacheableCamera`) — every
  framebuffer.
- TPU staging (`s_tpu_input_buf_single`) — already
  `__attribute__((aligned(64)))`, keep at 64 B (USB EHCI prefers
  it).
- Aux slot buffers (`SLOT_GRAY_NATIVE`, `SLOT_RGB_64`,
  `SLOT_GRAY_64`) in `.sdram_bss` — currently the slot table
  computes pointers ad-hoc; T5 should retrofit a
  `SENTAI_PREP_BUF_ALIGN` macro and assert in
  `sentai_prep_init()`.
- Flow phase-correlation buffers (`flow_phase_corr.cc`).
- ArUco preprocessing buffers (`s_gray`, `s_bin`, `s_edge` in
  `aruco_bench.cc` — already `.sdram_bss`, alignment unverified).
- FR pool slots (`s_frame_pool`).

Convention to standardise:

```c
#define SENTAI_CACHE_LINE_BYTES  32
#define SENTAI_HOT_BUF_ALIGN     __attribute__((aligned(SENTAI_CACHE_LINE_BYTES)))
```

Apply to every new hot-path buffer; T5 retrofits the existing
ones in a single sweep.

## Tentative task breakdown

- **W15-T1** — Memory-map audit script: parse `output.map`,
  dump per-region top consumers, utilisation, alignment violations
  (warn on any `.sdram_bss` / `.ocram` / `m_data` buffer ≥ 256 B
  not on a 32 B boundary).  Run in CI for both ARM + SIM.
- **W15-T2** — Task priority + stack audit script: grep
  `xTaskCreate*` call sites, emit table, compare against
  `FreeRTOSConfig.h` priorities + reservation.
- **W15-T3** — `paper/arm_memory_budget.md` design doc (the 5
  scope items + the alignment policy).
- **W15-T4** — Per-region "soft alarm" assertions in CMake
  post-build: warn at 80 / 90 %, fail at 95 %.  Includes the
  alignment lint as a hard gate.
- **W15-T5** — Retrofit existing subsystems: introduce
  `SENTAI_HOT_BUF_ALIGN` + `SENTAI_MEM_COLD_PATH` macros,
  replace the scattered `__attribute__((section(...)))` +
  `aligned(...)` boilerplate across ~15 files.  Audit all hot-
  path buffers for 32 B alignment.

## Anti-scope

Design + audit only.  No functional refactor.  If any allocation
moves, document before/after.  Lifetime: 1-2 days when pulled in.
