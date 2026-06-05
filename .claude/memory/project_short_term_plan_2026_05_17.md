---
name: short-term-plan-2026-05-17
description: "4-week SIM-only plan post WBS migration. Calendar W1 = OP-S6-W1 (sentai.calib MP), W2 = OP-S10-W4 (HSV), W3 = OP-S10-W5+W6 (FFT + opposite-dir), W4 = OP-S10-W7 (L1 tracker w/ ArUco-shim). ARM build + cycle budget + FlowBaseline + anti-cheat audit gates per week. DNN deferred."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Approved by operator 2026-05-17**.  Focus: SIM-only progress on
objects_plan.md §23.2 critical path, ARM build kept green and
cycle-budget verified per step.  DNN integration explicitly deferred
(ArUco-as-tracklet shim per §23.2 note for L1 tracker validation).

WBS codes per `ideas/wbs.md`.  Calendar weeks `W1..W4` are tags ONLY —
paired with WBS codes (`OP-S6-W1` etc.) as the load-bearing identifiers.

**Why**: §23.2 SIM-only items remaining = calib + Track A descriptors
+ L1 tracker.  All four can ship in 4 weeks without touching HW or DNN.
HW work (`OP-S9` ARM bring-up, `OP-S10-W8` L7 demo, `OP-S10-W9`
outdoor PX4) starts after these land.

## Calendar W1 — `OP-S6-W1` sentai.calib MP binding

Port `_shared/camera_calibration.py` Kabsch 3D Procrustes to C++.
- `OP-S6-W1-T1` — `sentai_calib.{cc,h}` Kabsch on 8 pts, ~100k cyc/invoc on M7
- `OP-S6-W1-T2` — `bindings/modsentai_calib.c` (`run_takeoff_pad`, `get_R_cam_to_body`, `is_calibrated`)
- `OP-S6-W1-T3` — Persist to `/system/cam_calib.json` via FxUser
- `OP-S6-W1-T4` — `EXP-s157_calib_smoke` (perturb ±5°, recover < 0.5°)
- `OP-S6-W1-T5` — `EXP-s158_calib_takeoff` (integrate into mission_s153)
- `OP-S6-W1-T6` — ARM size delta < 8 KB

## Calendar W2 — `OP-S10-W4` HSV descriptor (Track A)

- `sentai_hsv.{cc,h}` — 8×8×8 hist + top-K → 64B, ~50k cyc on 80×60
- Extend `modsentai_places.c` with `compute_hsv(img, w, h)`
- `EXP-s159_hsv_baseline` — anti-regression goldens (mirror EXP-s141)
- Extend `hex_helpers.py:quantize` to PHOG+GIST+HSV (3/4)

## Calendar W3 — `OP-S10-W5` FFT-mag log-polar + `OP-S10-W6` opposite-direction recall

- `sentai_fft_logpolar.{cc,h}` — log-polar FFT magnitude using CMSIS-DSP
  `arm_rfft_fast_f32` on ARM, FFTW3 on SIM (existing shim pattern)
- ~150k cyc on 80×60
- `EXP-s160_fft_baseline` — anti-regression
- `EXP-s161_opposite_dir_recall` — drone flies gallery forward, then
  reverse; recall ≥ 80% per §14.  Closes §22.5 (4/4 descriptors complete).

## Calendar W4 — `OP-S10-W7` L1 tracker minimal w/ ArUco-as-tracklet

- Wire existing `sentai_tracker.cc` skeleton to detection_task output
- `sentai_tracker_aruco_shim.cc` — bypass DNN dependency for thesis MVP
  per §23.2 explicit allowance
- L5 lifter consumes `tracker.confirmed_id` → `lifter.update_bbox()`
- `EXP-s162_tracker_aruco` — 2-3 markers, orbit, stable IDs ≥ 5s
- `EXP-s163_tracker_occlusion` — hide marker 2s, recover same tracklet_id

## Per-commit discipline

```bash
cmake --build build --target sentai_runtime              # ARM must compile
arm-none-eabi-size build/.../sentai_runtime.elf          # size delta check
bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh
bash sim/scripts/audit_anti_cheat.sh
bash <new>/run.sh                                         # new test green
```

## After this plan

§23.2 SIM-only items shipped.  Then HW track: A5 (Stage 9 ARM bring-up),
A6 (L7 indoor demo), A7 (outdoor PX4), A8 (evaluation chapter).

## Cross-references

- [[next-steps-2026-05-17]] — A1 (s147-s151 migrations) already shipped
- [[experiments-start-from-origin]] — respawn gate per run
- [[sentai-sim-air-gapped-from-truth]] — anti-cheat enforcement
- [[gate-every-layer-no-exceptions]] — FlowBaseline after each layer
- §23.2 objects_plan.md — frozen thesis MVP scope
