---
name: feedback-emu-must-not-reuse-production-task-names
description: HARD RULE — emulator test scaffolding under emu/ must NOT name its tasks PrepTask/InferTask/FlowTask/CameraTask — those are existing production symbols in sentai_runtime with specific algorithm semantics.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8b2903f8-f695-46ef-9f13-edba1cccdfeb
---

Operator note 2026-06-02 (during B8.6 review):

> "atentie la InferTask si PrepTask si CameraTask, e cod existent in
> sentai_runtime"

HARD RULE for any emulator (or other) scaffolding code:

Do **NOT** name a stand-in task `PrepTask`, `InferTask`, `FlowTask`, or
`CameraTask`.  Those names are already taken by production code:

| Name        | Production location                              | What it actually does                            |
| ----------- | ------------------------------------------------ | ------------------------------------------------ |
| `PrepTask`  | `examples/sentai_runtime/detection_task.cc`      | cam_grab + PXP downscale + RGB→Y8 + INT8 quant   |
| `InferTask` | `examples/sentai_runtime/detection_task.cc`      | EdgeTPU `Invoke()` over USB                      |
| `FlowTask`  | `examples/sentai_runtime/flow_task.cc`           | USADA8 / phase correlation on the prep output    |
| `CameraTask`| `libs/camera/camera.cc` (C++ class)              | owns CSI receiver queue + ISR + frame buffer pool |

The first three are statically scoped (their entry-point C functions are
`prep_task_fn`, `infer_task_fn`, `flow_task_fn` and they have internal
linkage), so a duplicate symbol in a separate translation unit will not
fail the linker.  But that is exactly the problem — the bug shows up at
grep time and at design-discussion time, not at compile time, and the
emu stand-in starts being mistaken for the real algorithm.

**Why this matters:** the emu spike runs in a completely different
universe (sum reduction over 64 bytes; no PXP, no TPU, no USADA8).
Anything named `PrepTask` or `FlowTask` in the emu code creates a
falsely reassuring grep / docs / commit-message landscape.

**How to apply:**

- For emu pipeline tests, use neutral stage names like `Stage1Task` /
  `Stage2Task` or descriptive ones like `EmuReduceTask` /
  `EmuMarkerTask`.  Prefix `Emu...` is always safe.
- FreeRTOS task names registered with `xTaskCreateStatic` should also be
  emu-distinct: `"emu_stage1"`, `"emu_stage2"`, never `"det_prep"` etc.
- Same rule extends to globals: do NOT name a counter
  `g_prep_processed` in the emu — call it
  `g_sentai_emu_stage1_processed` or similar.
- Commit messages and doc references should be careful too: write
  "emu Stage1Task (a stand-in)" rather than "PrepTask".
- This rule extends to any future stand-in tasks like SafetyTask,
  CrazyTask, PipelineTask, etc. if those names later become real
  production symbols.  When in doubt, prefix `Emu...`.

**Cross-refs:** [[feedback_sentai_sim_air_gapped_from_truth]] (same
spirit: do not let scaffolding pretend to be production); B8 doc
`todo/TD-S10-B8_research_arm_emulator_runtime.md` Open Caveats section.

**Historical:** an early B8.6 spike (commit 6e8be311) used the names
`PrepTask` + `FlowTask` and was flagged by the operator on review.  The
follow-up commit renamed both to `Stage1Task` / `Stage2Task` with this
rule embedded in the source comments.
