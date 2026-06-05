---
name: s131-lifter-math-shipped
description: s131 inverse-depth EKF math validation (Python prototype) — 2026-05-15 PASS. Commit 898e8d05. Validates Stage 5 lifter math BEFORE C++ ARM port. Synth near 1.5mm + far 2.6cm; replay on s130 real frames all 4 markers XY err <7cm (Z weak per monocular geometry).
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Commit `898e8d05` on integration/from-180bbb5f, 2026-05-15.**

s131 is a 1-day Python validation gate for Stage 5
`sentai_object_lifter` (Civera/Davison/Montiel TRO 2008) BEFORE
investing 5-7 days in the C++ ARM port. Tests the high-risk piece
(numerical tuning + frame conventions) in isolation.

## Phase A: synth_pass.py (authoritative math gate)

Controlled lateral pass (drone z=1m, ±0.30m, 6s @ 30 fps, σ_obs=1 px):
| Scenario | d (m) | Final err | ready_at_s | n_obs |
|---|---|---|---|---|
| near_aruco (d=0.8) | 0.80 | **1.5 mm** | 0.0 (init) | 180 |
| far_cube (d=6) | 6.0 | **2.6 cm** | 0.27 (after parallax) | 180 |

Both PASS. Joseph form keeps variance positive. Zero rejections.

## Phase B: replay_pass.py (informational, real s130 Gazebo frames)

Per-marker XY/Z errors:
| Marker | n_obs | XY err | Z err | Notes |
|---|---|---|---|---|
| mk0 | 3 | **1.7 mm** | 3.1 mm | best parallax in s130 motion |
| mk1 | 3 | 1.1 cm | 20 cm | limited parallax → Z weak |
| mk2 | 4 | 3.0 cm | 14 cm | 1 rho-clamp rejection (healthy) |
| mk3 | 3 | 7.0 cm | 56 cm | worst parallax → Z very weak |

Z weak-observability is a FUNDAMENTAL monocular limit with the s130
outer-loop motion (~10-20 cm lateral, no altitude change), NOT an
algorithmic bug. Strictness of s132 (dedicated lateral pass) will
tighten Z.

## Key lessons captured in code

- **Joseph form is mandatory**: scalar variance update keeps σ_ρ²>0
  even under adverse innovations (mk2 case where naïve update would
  drive ρ→-0.3).
- **ρ-clamp guard fires as expected**: per Civera §V-B, rho_clamp
  catches updates that would push depth out of bounds → reject + keep
  state. Embeded.md F (local recovery before subsystem restart).
- **innov_px must be recorded BEFORE reject path**: original code set
  innov_px=0 on reject (bug). Fixed: capture innov magnitude first.
- **bbox center ≠ marker centroid under perspective**: real-frame
  innovations are 20-70 px even when L_W is correct. Not a problem
  for monocular bearing-only EKF, but flag for future tightening.

## What this validates (and what it does NOT)

VALIDATES:
- EKF math (Jacobian, Joseph form, init from class-prior)
- Frame conventions: bearing → world via R_W_B(yaw) @ R_B_C
- No NaN/Inf under real-data noise
- Convergence in both near (d=0.8m) and far (d=6m) regimes
- Class-prior pseudo-depth init (no parallax needed initially)

DOES NOT validate (deferred):
- ARM compute budget (DWT timing on M7 — Stage 9, after L5 port)
- ARM single-precision FP precision (32-bit float)
- Real-world camera mount calibration tolerance ([[camera-mount-calibration]])
- Multi-landmark interaction (single landmark in s131)
- Non-fiducial bbox extraction noise (s132 — real classes like cube)

## Files

`examples/sentai_runtime/experiments/s131_lifter_replay/`:
- `lifter_proto.py` (~270 lines) — Landmark class, importable
- `synth_pass.py` — controlled synthetic test
- `replay_pass.py` — uses s130 captured frames + image_vs_cf2.json
- `verdict.py` — combined PASS/FAIL summary
- `run.sh` — orchestrate; exit 0/1/2 (synth-fail vs replay-fail)
- `README.md` — purpose, files, run, pass criteria

## How to reproduce

```bash
bash examples/sentai_runtime/experiments/s131_lifter_replay/run.sh
# Verbose:
S131_VERBOSE=1 bash .../run.sh
# Synth-only (don't open s130 frames):
S131_SYNTH_ONLY=1 bash .../run.sh
```

## FlowBaseline gate post-commit

`bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh`:
- dist_mean=8.3 cm (canonical 7.4 cm band)
- all4_rate=1.0
- n_samples=19, flow_hz=27.1
- PASS — no regression

## What comes next (per objectsplan-l5-handoff)

1. Pas 2 — camera calibration shared module (Kabsch 3D Procrustes)
2. L5 C++ port: `sentai_object_lifter.{h,cc}` using CMSIS-DSP arm_mat_*
3. s132 — dedicated lateral-pass SIM integration test (real lifter)

Related: [[objectsplan-l5-handoff]], [[s130-45baseline-shipped]],
[[camera-mount-calibration]], [[itcm-budget]],
[[gate-every-layer-no-exceptions]], [[flowbaseline-canonical-config]].
