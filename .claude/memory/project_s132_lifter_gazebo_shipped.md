---
name: s132-lifter-gazebo-shipped
description: s132 L5 lifter Gazebo integration PASS first try 2026-05-15. cf2 takeoff z=1.5m + 4×0.1m lateral pass over marker id0. err_xy=1.7cm err_z=11.6cm sigma_rho_ratio=0.014 — math validated under real Gazebo cadence. Commit 6827e42a. FlowBaseline 7.1cm PASS post.
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Commit `6827e42a` on integration/from-180bbb5f, 2026-05-15.**

s132 closes the L5 loop end-to-end: bbox observations from real
Gazebo ArUco detections → `sentai.object_lifter` via REPL exec →
verified convergence to true marker 3D position.

## Mission profile

- Single target: marker id 0 at world `(+0.15, +0.10, 0.20)`
- cf2 force-respawn at origin, takeoff to z=1.5m, yaw=0
- `init_from_bbox` from first detection
- 4 lateral steps × 0.10 m body-Y, each followed by `update_bbox`
- Final `world_pos(0)` vs GT compared

## Results (first try PASS)

| Metric | Value | Threshold |
|---|---|---|
| status | LIFTED (2) | required |
| n_obs | 5 (1 init + 4 updates) | ≥ 4 |
| err_xy | **1.7 cm** | < 20 cm |
| err_z | **11.6 cm** | < 60 cm (monocular weak) |
| sigma_rho_ratio | **0.014** | < 0.5 |
| rejects | 0/4 updates | ≤ 1 |
| FSM | armed=0 flight=GROUND last=DISARM/0 | clean |

FlowBaseline post: dist_mean=7.1 cm (canonical 7.4), all4=1.0,
n_samples=23 — PASS, no regression.

## Architecture validated

- L5 C++ lifter receives bbox (u, v, w_px) + drone_W + yaw via REPL
- Math bit-for-bit equivalent to s131 Python prototype (no surprises)
- σ_ρ converges aggressively: 0.343 → 0.005 in 4 updates
- 0.40 m lateral baseline at z=1.5 m gives sufficient parallax for
  XY closure; Z stays loose per monocular geometry

## Key wiring notes

- Lifter defaults (fx=577, fy=579, cx=320, cy=240, R_B_C swap,
  cam_offset_B=(-0.04,0,-0.02)) match `aruco_detector.py` constants
  out of the box — `set_camera()` call is **not necessary** in SIM.
- Bbox center from `marker.corners.mean(axis=0)`, bbox_w_px from mean
  of 4 corner-to-corner edge lengths (rotation-tolerant proxy).
- `drone_W` from `cf2.stateEstimate`; lifter cares about pose deltas
  for parallax, so EKF drift is largely absorbed.
- REPL exec with float-tuple literal `(x, y, z)` parses fine through
  MicroPython parser; `exec_repr` round-trips dicts/tuples cleanly.

## What this proves vs s131

s131 = math validated on 4 static frames (replay).
s132 = math validated under:
- Real CSI ring frame cadence (~30 Hz produced, sampled at ~1 Hz)
- Real cf2 pose noise/drift from SITL EKF
- REPL exec roundtrip overhead (~100-200 ms/call)
- Real bbox extraction from corners (not pre-computed)

Numerics match s131 expectations: tight XY, weak Z, σ_ρ converges
once parallax is present.

## What s132 does NOT do (deferred)

- Wire lifter into `detection_task` C++ hot path (still REPL-driven)
- Multi-marker simultaneous tracking (only id 0 used)
- Auto-publish LIFTED → `sentai.objects` (manual query only)
- ARM bring-up + DWT timing budget (Stage 9 deferred)

These are explicit non-goals — s132 validates math+API+runtime, not
production wiring. detection_task integration follows in Stage 5
production wiring + Stage 9 ARM bring-up.

## How to verify

```bash
bash examples/sentai_runtime/experiments/s132_lifter_gazebo/run.sh
# Expect: "PASS — L5 lifter converges under Gazebo cadence"
# Exit 0 on PASS, 1 on FAIL.
```

## Related

[[l5-shipped]], [[s131-lifter-math-shipped]], [[places-l3-shipped]],
[[places-two-track-decision]], [[flowbaseline-canonical-config]],
[[gate-every-layer-no-exceptions]], [[experiments-start-from-origin]],
[[gazebo-gui-required]], [[sentai-sim-journal]].
