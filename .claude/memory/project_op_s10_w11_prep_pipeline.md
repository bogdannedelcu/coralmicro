---
name: op-s10-w11-prep-pipeline
description: "OP-S10-W11 sentai_prep frame slot pipeline.  T1+T2+T3+T3.1+T4 SHIPPED 2026-05-17 (commits b104d77e/90b0523b/cc13484e/3e7eb094/332912f7).  Cross-cutting frame producer + SlamTask consumer: PrepTask fan-out + atomic-publish + seqlock tear-detection + refcount enable + zerocopy C accessors + InferTask-style perception loop (HSV+places query) + SUBSYS_SLAM health integration + SIM camera_bridge_recv mirror.  ARM + SIM both live.  EXP-s162 lifecycle + s163 e2e SIM both 5/5 PASS.  No MP buffer exposure (per [[no-heavy-data-through-mp]])."
metadata:
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

WBS: `OP-S10-W11` — cross-cutting infrastructure WP that supports
OP-S6-W1 (calib), OP-S6-W3 (aruco), OP-S10-W4 (HSV), OP-S10-W5/W6/W7
future consumers.  Placed under OP-S10 because the dominant set of
consumers is the Track A descriptor stack.

## Motivation

Operator architectural decision 2026-05-17: image processing happens
AUTOMATICALLY at camera FPS, algorithms just CONSUME prepared
buffers.  No on-demand PXP in algorithm code; no heavy buffers via MP
heap.  Mirrors the existing detection_task PrepTask + InferTask
pattern but for non-TPU consumers (HSV, places match, ArUco PnP,
future tracker).

## Phase 1 (this commit batch, 2026-05-17)

| Sub-task | Status | Commit | Notes |
|---|---|---|---|
| OP-S10-W11-T1 | ✅ SHIPPED | `b104d77e` | `sentai_prep.{h,cc}` foundation.  Slot enum, .sdram_bss storage, refcount enable/disable, atomic publish (__DMB + seq++), zerocopy accessor, frame_div hook (unused).  Commit subject was "Phase 1a" — lacked WBS prefix; flagged 2026-05-17 audit and retroactively labeled. |
| OP-S10-W11-T2 | ✅ SHIPPED | `90b0523b` | PrepTask integration: SLOT_GRAY_NATIVE populated each frame when refcount>0 via `sentai_pxp_xrgb_to_y8`.  `sentai_camera_grab_gray_zerocopy` (consumer) refactored dual-path: slot-fast + on-demand fallback.  Same WBS-prefix audit issue as T1. |
| OP-S10-W11-T3 | ✅ SHIPPED | `cc13484e`+`982b2282` | `slam_task.{h,cc}` (InferTask-style perception loop): counting-sem max=1, take(500 ms timeout), HSV compute on SLOT_RGB_64, places_query, atomic publish (DMB + result_seq).  Prereq audit fix wired `sentai_prep_init()` + camera/pipeline checks in `start_slam`. |
| OP-S10-W11-T3.1 | ✅ SHIPPED | `3e7eb094` | Seqlock tear-detection (`_begin_read`/`_end_read` with `producer_overruns` bump + first-time SERR), SERR_MOD_PREP / SERR_MOD_SLAM taxonomies, SUBSYS_SLAM health integration (success/fail on each cycle, recovering/unavailable on lifecycle).  `sentai_health.cc.obj` routed to `.sentai_slow` to keep m_text under budget.  Items M5/M6/M7/m4 deferred to **FW18**. |
| OP-S10-W11-T4 | ✅ SHIPPED | `332912f7` | SIM mirror in `camera_bridge_recv.c`: same atomic-publish contract, uses `sentai_pxp_scale` shim (scalar area-average).  Knock-on: SlamTask priority bumped `+1`→`+2` to match producer; on POSIX TIME_SLICING=1 this prevents starvation by REPL+bridge during frame bursts.  EXP-s163: 5/5 PASS, `frames_processed=15`, `t_compute_us=20`. |
| OP-S10-W11-T5 | ⬜ TODO | — | Live Gazebo end-to-end: drone hovers at 2-3 poses, SlamTask runs continuous, MP polls `slam_current()` to confirm distinct match scores per pose.  Folds in FW18 polish items if convenient. |

## Architecture (one-page)

```
            Camera EOF (TaskNotify today, CSI ISR future)
                              │
                              ▼
                         PrepTask
                              │
            ┌─────────────────┴─────────────────┐
            │ AUX slots (non-blocking)           │
            │   if (mask & SLOT_X) {            │
            │     pxp_scale(raw → slot.buf)      │
            │     __DMB(); slot.seq++           │
            │     give(s_sem_consumer_input)    │
            │   }                                │
            │                                    │
            │ TPU staging (sem-gated, existing)  │
            │   take(s_sem_input_free)          │
            │   pxp_scale_quant(raw → staging)  │
            │   give(s_sem_input_full)          │
            └──┬──────────────┬──────────────┬──┘
               ▼              ▼              ▼
           InferTask     SlamTask      Future
           (TPU dets)    (HSV+places)  AnchorTask
                                       (ArUco PnP)
```

Key invariants:
- Aux slot path: NO semaphore taken — last-frame-wins (per
  [[no-heavy-data-through-mp]] continuous-publish discipline).
- TPU staging path: sem-gated lossless (existing behaviour preserved).
- Slot enable refcount: multiple consumers can hold the same slot
  open without coordination.  Producer fires slot only if refcount>0.
- Cold-path PXP cost: ~0.7 ms per slot per frame (Y8 320×240 from
  VGA XRGB).  At 30 FPS budget (33 ms/frame) → ~2% per slot.
- ISR-ready: aux slot loop uses only PXP HW + memcpy + DMB + atomic
  uint32 increment.  Eventually callable from CSI EOF ISR directly.

## SOTA basis / no wheel reinvention

| Stage | Reference |
|---|---|
| Pre-processed slot pattern | Standard SLAM / VPR infrastructure (ORB-SLAM3 keyframe slots, FAB-MAP image queue, OpenVSLAM keyframe pool) |
| InferTask-style perception loop | Mirror of existing `detection_task.cc:infer_task_fn` shipped by sentai_runtime |
| Atomic publish (DMB + seq counter) | Lockless single-producer-multiple-reader pattern (Lamport / Vyukov SPMC ring) |
| PXP HW for color conversion | NXP AN12437 + RT1176 RM 41.4 (kPXP_OutputPixelFormatY8 etc.) |

No new algorithmic invention — pure orchestration / plumbing.

## Verified constraints

- `[[no-heavy-data-through-mp]]`: zero MP buffer exposure; all
  consumers via C accessor.
- `[[arm-hw-primitives-first]]`: PXP HW for XRGB→Y8 (no scalar).
- `[[itcm-budget]]`: `sentai_prep.cc` routed `.sdram_text`; PrepTask
  body additions in m_text (~200 B) accepted as hot-path exemption.
- `[[tpu-pipeline-aggressor]]`: aux slot path NON-BLOCKING; +0.7 ms
  PXP per frame is below the contention threshold demonstrated by
  the V22 OCRAM tensor + Cale 1 + MoverTask dead-ends.
- `[[pxp-init-required]]`: PXP_Init() already called via
  BOARD_InitCamera at sentai.camera.init; PrepTask runs after that.
- `[[gate-every-layer-no-exceptions]]`: FlowBaseline post-Phase-1b
  PASS — `dist_mean=5.95 cm`, `all4_rate=0.95`, `flow_hz=31.3`.
- `[[english-docs-only]]`: every artifact English.

## WBS audit gap (transparency)

Commits `b104d77e` (Phase 1a) and `90b0523b` (Phase 1b) shipped
WITHOUT the `OP-S10-W11-T*:` prefix on the commit subject, violating
`[[wbs-pmp-2026-05-17]]` HARD RULE.  Flagged in this session's audit
and retro-documented in `ideas/wbs.md` (legacy mapping table) +
this memory entry.  Forward commits Phase 1c+ MUST use the proper
WBS prefix.

## Cross-references

- `[[no-heavy-data-through-mp]]`, `[[arm-hw-primitives-first]]`,
  `[[itcm-budget]]`, `[[tpu-pipeline-aggressor]]`,
  `[[wbs-pmp-2026-05-17]]`.
- `examples/sentai_runtime/sentai_prep.{h,cc}` — implementation.
- `examples/sentai_runtime/detection_task.cc:prep_task_fn` — producer.
- `examples/sentai_runtime/bindings/modsentai_camera.c
  :sentai_camera_grab_gray_zerocopy` — consumer dual-path.
- `agent/agent.md` §V22 OCRAM tensor + Cale 1 + MoverTask context.

## Architectural clarifications (added 2026-05-19)

Discussion 2026-05-19 surfaced 4 facts that the original memory
under-specified; recording here so the canonical PrepTask mental
model is precise.  Spec file with the long-form derivation:
`ideas/objects_plan/OP-S10-W11_sentai_prep.md`.

1. **PrepTask is dual-purpose**, not slot-only:
   - **Path A** (predates the slot system): camera ring → PXP →
     OCRAM (`s_tpu_input_buf_single`, `.tpu_input` section) +
     optional in-place uint8→int8 cast → InferTask.  This is the
     load-bearing TPU pre-processing path; OCRAM placement is
     mandatory per [[tpu-pipeline-ocram-tensor]] (SDRAM staging
     wedges TPU via SEMC contention with CSI DMA).
   - **Path B** (W11 addition): same source frame → PXP → SDRAM
     `.sdram_bss` slot buffers → SlamTask / future descriptor
     consumers.  Independent of Path A; cold-path consumers (~1 Hz)
     don't have the contention issue.
2. **PXP DMA moves the bytes**, not CPU.  PrepTask just
   orchestrates: pointer-passes the SDRAM ring address into
   `sentai_pxp_scale`, blocks on PXP-done, then signals the next
   stage.  This is the whole point of the OCRAM lesson — PXP uses
   a bus path independent of SEMC.
3. **InferTask NEVER touches** the SDRAM camera ring or PXP.
   `detection_task.cc:15-18` header is explicit: *"staging_buf:
   only PrepTask writes; PXP hardware: only used by PrepTask"*.
   InferTask reads only from OCRAM staging → either memcpy into
   the TFLite arena (legacy path, OCRAM → OCRAM) or pointer-swaps
   in direct mode.
4. **TPU input quant is conditional**, not unconditional.  The
   storage container is always `static uint8_t
   s_tpu_input_buf_single[...]`.  `sentai_quant_uint8_to_int8` is
   called in-place only when the active model's input tensor is
   `kTfLiteInt8` (type 9); for `kTfLiteUInt8` (type 3) the step
   is skipped and the buffer goes to TPU as raw uint8.  Signed
   interpretation lives in TFLite tensor metadata, not in C
   storage type.

## GRAY_64 design decision (operator-approved 2026-05-19)

When SLOT_GRAY_64 is wired (T4 Phase 1d), it MUST be **derived
from RGB_64 via luma-cast** (~7 µs NEON on 4096 px), NOT produced
by a second independent PXP pass.  Three reasons:

1. Pixel-perfect consistency — both 64×64 slots refer to the
   SAME source frame (no PXP race window).
2. One PXP setup per frame — saves SEMC bandwidth.
3. Simpler producer code on both ARM and SIM.

Producer commits RGB_64 first, luma-casts into GRAY_64 if its
refcount > 0, commits GRAY_64.  Document the invariant in
`sentai_prep.h` when T4 lands: *"GRAY_64 corresponds to the same
source frame as RGB_64 (luma downcast, not an independent
capture)"*.  Full rationale: `ideas/wbs.md` OP-S10-W11-T4 entry.
