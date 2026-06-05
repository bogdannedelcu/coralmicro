---
name: l5-shipped
description: "ObjectsPlan L5 sentai_object_lifter shipped 2026-05-15 — inverse-depth EKF (Civera 2008) in C++, ARM+SIM. Commit 17cbe3aa. Driver test 38/38 PASS. FlowBaseline PASS post. Math = Python prototype s131."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Commit `17cbe3aa` on integration/from-180bbb5f, 2026-05-15.**

L5 of ObjectsPlan ships. Mirror-image of L2/L3/L4 file layout:
- `sentai_object_lifter.h` — C ABI + embeded.md system-model docblock
- `sentai_object_lifter.cc` — implementation, shared ARM+SIM via
  `SENTAI_PLATFORM_SIM` ifdef
- `modsentai_object_lifter.c` — MicroPython binding
- `diag/_t_04_object_lifter.py` — driver test (38/38 PASS)

## Math (= Python s131 prototype, bit-for-bit equivalent)

Per-tracklet 1-state EKF:
- state: scalar ρ (inverse depth) + σ_ρ²
- constants per landmark: anchor_w (3-vec, camera world pos at first
  obs) + r_w (3-vec unit world bearing), captured at init
- init from class-prior pseudo-depth: ρ₀ = bbox_w_px / (fx · real_size)
- Joseph form variance update
- ρ-clamp guard ([0.05, 20] m depth range)
- Civera linearization gate: status → LIFTED when σ_ρ < ε·ρ² (ε=0.5)

## API surface (sentai.object_lifter)

```
init_from_bbox(tid, cls, u, v, w_px, real_m, drone, yaw) -> rc/slot
update_bbox(tid, u, v, drone, yaw, dt_s)                 -> rc
get(tid) -> dict | None
world_pos(tid) -> (x, y, z) | None
list() -> [dict, ...]
count()  / clear() / mark_lost(tid)
stats() -> dict
set_camera(fx, fy, cx, cy, R9, ofs3) -> rc
Constants: FREE TRACKING LIFTED LOST
```

Return codes (all non-raising per embeded.md):
- `-1` invalid_input / unknown_tracklet / fx≤1 NaN
- `-2` invalid_class_prior / rho_clamp_violated
- `-3` full / R9 wrong shape
- `-4` var_invalid
- `-5` behind_camera / ofs3 wrong shape
- `-6` invalid input / bearing NaN

## Memory footprint

- ARM: routed to `.sentai_slow` (m_sdram) via linker script alongside
  L2 objects + L3 places + (now) L4 servo.  ITCM unchanged.
- SDRAM: 16 slots × ~96 B = ~1.5 KB + stats + cam intrinsics ≈ 2 KB total.
- Zero heap, zero per-call malloc.

## Camera intrinsics

Default bootstrap to s130-calibrated values:
- fx=577, fy=579, cx=320, cy=240
- R_B_C = [[0,1,0],[1,0,0],[0,0,-1]]
- cam_offset_B = [-0.04, 0, -0.02]

These are REPLACEABLE at runtime via `set_camera()`. Future
`sentai.calib` (Stage 6, objects_plan.md §21) will drive this from
takeoff Kabsch 3D Procrustes calibration.

## Not yet wired

- `detection_task` integration — hook after tracker.update() to feed
  lifter from confirmed tracklets (s132 SIM integration test).
- Auto-publish to `sentai.objects` when status reaches LIFTED.
- ARM DWT timing measurement (Stage 9 verification of <200 µs/tick budget).

## How to verify

```bash
# Build SIM
cmake --build build-sim --target sentai_sim
# Run driver test
cp examples/sentai_runtime/diag/_t_04_object_lifter.py \
    build-sim/sentai_fs_root/t04_object_lifter.py
echo 'import t04_object_lifter' | ./build-sim/sim/sentai_sim
# Expect: "L5 LIFTER DRIVER PASS" (38/38)

# Build ARM (no .text overflow — linker routes to .sentai_slow)
cmake --build build --target sentai_runtime
```

## FlowBaseline gate post-commit

`bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh`:
PASS dist_mean=4.77 cm, all4_rate=1.0, n_samples=22, flow_hz=31.3.
No regression vs canonical 7.4 cm.

## What comes next

1. **s132** — SIM lateral-pass integration test driving the lifter
   end-to-end from real Gazebo frames (deferred from s131).
2. **Stage 6** — `sentai.calib` MP binding for runtime camera
   calibration (objects_plan.md §21).
3. **L6** — `sentai.explore` mission FSM (per [[no-safety-logic-in-explore]]).
4. **L7** — s125 integrated demo.

Related: [[s131-lifter-math-shipped]], [[camera-mount-calibration]],
[[itcm-budget]], [[no-broken-branch-test-reuse]],
[[gate-every-layer-no-exceptions]], [[flowbaseline-canonical-config]],
[[objectsplan-l5-handoff]].
