---
name: op-s10-w12-w13-shipped
description: OP-S10-W12 sentai.safety + OP-S10-W13 sentai.fr SHIPPED 2026-05-18 (commit 6303b95d). State machine + SafetyTask + Flight Recorder. SIM-only; ARM port deferred.
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

OP-S10-W12 + OP-S10-W13 SHIPPED 2026-05-18, commit `6303b95d` on branch
`integration/from-180bbb5f`. 28 files, +3860 lines.

**Why these two together:** SafetyTask is the first FR producer
(pushes per-frame gray + n_dets via `sentai_fr_push_frame`); shipped
in the same commit so end-to-end SafetyArucoBaseline validates both.

## OP-S10-W12 — sentai.safety
Firmware-side mission safety service. Replaces the host-side abort
logic from s167 (which violated [[missions-run-in-sentai-only]]).

- `sentai_safety.{h,cc}` — pure state machine. Per-check enum, sticky
  abort, stale-feed watchdog (2 s), event ring (32 entries). Aruco
  check: `n_dets < n_min for >= max_loss_ms` → latch abort.
- `sentai_safety_task.{h,cc}` — FreeRTOS task @ `tskIDLE_PRIORITY+2`.
  REUSES `sentai_camera_grab_gray_zerocopy` + `sentai_aruco_detect`;
  no detection / camera intake duplicated. Auto-inits sentai.aruco
  on task_start so missions don't have to.
- `bindings/modsentai_safety.c` — 9-fn MP API: `init / clear /
  enable_aruco / disable_aruco / task_start / task_stop / aborted /
  reason / _test_push_aruco`. No dicts on heap.
- Tests: **s171_safety_unit 8/8 PASS** (state machine via injection);
  **s170_security_aruco_baseline e2e PASS** (drone takeoff 0.6 m →
  forced 0.8 m lateral drift → markers leave FOV → abort latches at
  `n_dets=2 < 4 for 1031 ms` → emergency land 5.2 cm from origin).

Hard rule [[flowbaseline2-4markers-abort]] now enforced inside
firmware, not host Python.

## OP-S10-W13 — sentai.fr (Flight Recorder)
NASA/JPL FDR-style independent recorder. Producers push items
(O(1), non-blocking); single drain task writes to disk.

- Channels: `frames` (PGM per push, `t<ms>_n<dets>_f<seq>.pgm`),
  `events` (CSV `ts_ms,type,text`), `scalars` (CSV `ts_ms,label,value`),
  `kernel` (stub).
- Static slot pools: frames 16 × 76 KB SDRAM (320×240) up to 640×480
  max; events 256 × ~120 B; scalars 1024 × 24 B. Bounded queues with
  drop counters.
- Drain task @ `tskIDLE_PRIORITY+2`; 20 ms `vTaskDelay` poll.
- Producer/drain lock = **BINARY semaphore** (NOT priority-inheriting
  mutex). Critical for FR independence per operator spec ("sentai.fr
  trebuie sa fie independent de alte task-uri") — PI mutex tripped
  `xTaskPriorityDisinherit pxTCB == pxCurrentTCB` assertion during
  bring-up.
- Drain-side scratch = **static BSS** (NOT on task stack). FreeRTOS
  POSIX stack ≈ 16 KB; FrameSlot ≈ 76 KB → stack-copy = instant
  overflow. This was the bring-up bug.
- Bindings: text/scalar pushes only from MP; per-frame push is a C
  call from SafetyTask (preserves [[no-heavy-data-through-mp]]).

Verified stats from last s170 run:
```
frames:  pushes=135 accepted=135 drops=0 writes_ok=135 fail=0  worst_q=1
events:  pushes=14  writes_ok=14
```
Drain keeps up at 30 FPS with zero drops.

## SIM-only bug fix bundled in same commit
`sim/modsentai_sim_camera.c`: added `sim_local_now_ms_()` clock_gettime
fallback for weak `sentai_now_ms`. Without it every frame had ts_ms=0
on SIM → safety streak elapsed always 0 → abort never fired (silent
gate failure). Now monotonic.

## ARM port — deferred
SIM-only initial scope. ARM port adds:
- W12: nothing (state machine + SafetyTask already platform-neutral
  FreeRTOS code; routes to `.sdram_text` per [[itcm-budget]]).
- W13: replace fopen sinks with FxUser-backed sinks; disable `frames`
  channel by default on ARM (operator: "salvarea frames probabil nu
  va merge pe ARM pentru ca consuma f mult procesor").

## Files (non-exhaustive)
- `examples/sentai_runtime/sentai_safety.{h,cc}`
- `examples/sentai_runtime/sentai_safety_task.{h,cc}`
- `examples/sentai_runtime/sentai_fr.{h,cc}`
- `examples/sentai_runtime/bindings/modsentai_{safety,fr}.c`
- `examples/sentai_runtime/experiments/s17{0,1}_*`
- `Safety.md` — authoritative architecture doc
- `ideas/objects_plan/14_sentai_safety.md` + `15_sentai_fr.md`
- `sim/scripts/gt_recorder.py` — canonical host-side GT recorder

## Related
- [[op-s8-w1-cf2-sim-honest]] — replaced cf2 cheat plugin; W12 +
  s170 are the first end-to-end test with the honest VPE path.
- [[sim-test-must-return-home]] — s170 enforces ≤10 cm from origin.
- [[missions-run-in-sentai-only]] — mission_security_aruco.py runs
  inside `sentai_sim` MP; host is launcher + verdict only.
