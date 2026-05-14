# s131 — Inverse-depth EKF lifter replay (Python prototype)

**Goal**: validate the math of `sentai_object_lifter` (Stage 5,
Civera/Davison/Montiel TRO 2008 inverse-depth parametrization)
**before** investing 5-7 days in the C++ ARM port. Tests the
highest-risk piece (numerical tuning of EKF + frame conventions) in
isolation in Python.

## Why this test FIRST

L5 has two failure modes:
1. **Math bugs** — wrong Jacobian, wrong frame convention, EKF
   diverges. Equally bad in Python or C++, but Python iteration is 30×
   faster to fix.
2. **Numerical issues on M7** — single-precision FP, ρ→0 instability.
   Mitigated separately at port time (CMSIS-DSP Joseph form, ρ clamp).

We isolate mode 1 here, BEFORE writing any C++.

## Files

| File | Purpose | Runs in |
|---|---|---|
| `lifter_proto.py` | Pure-Python inverse-depth EKF (per-landmark) | importable |
| `synth_pass.py` | Synthetic lateral pass, known marker, controlled noise | <2 sec |
| `replay_pass.py` | s130 captured frames + cf2 telemetry post-hoc | <30 sec |
| `verdict.py` | PASS/FAIL summary across both | <1 sec |
| `run.sh` | Orchestrate all 3 in order | <1 min |

## Algorithm (per-landmark EKF)

**State**: scalar inverse depth ρ + variance σ_ρ². Anchor (camera world
position at first obs) and bearing direction r_W (unit, world frame)
are stored as constants.

**Init from class-prior pseudo-depth** (no parallax needed):
```
d₀ = fx · real_size / bbox_w_px
ρ₀ = 1/d₀
σ_ρ₀² = (0.5 · ρ₀)²              # 50% initial relative uncertainty
```

**Update step** (each new bbox observation):
```
L_W      = anchor + (1/ρ) · r_W           # landmark world
c_W(t)   = drone_W(t) + R_W_B(t) · cam_offset_B   # camera world
δ_C      = R_B_C^T · R_W_B(t)^T · (L_W - c_W(t))
(u, v)_pred = (fx · δ_C.x / δ_C.z + cx, fy · δ_C.y / δ_C.z + cy)
H        = ∂(u, v)/∂ρ                     # 2×1 Jacobian (chain rule)
S        = H · σ_ρ² · H^T + R_obs          # innovation 2×2
K        = σ_ρ² · H^T · S^-1               # Kalman gain 1×2
ρ       ← ρ + K · ((u, v)_obs - (u, v)_pred)
σ_ρ²    ← (1 - K · H) · σ_ρ²              # standard update
```

**Linearization-validity guard** (per Civera 2008):
```
σ_d / d² < ε_lin     ⇔     σ_ρ < ε_lin · d / 1 = ε_lin · 1/ρ²
```
With ε_lin = 0.5, the landmark is "ready" for use when:
```
σ_ρ < 0.5 / d² = 0.5 · ρ²
```

## Pass criteria

**synth_pass.py** (synthetic controlled test):
- σ_ρ becomes < 0.5/d² within 5 sec simulated time
- Final ||L_W_est - L_W_true|| < 0.10 m (synthetic data, no detection
  noise — strict tolerance)
- No NaN / Inf in state during the trajectory
- Joseph form keeps σ_ρ² > 0 throughout

**replay_pass.py** (s130 real Gazebo captured frames):
- σ_ρ becomes < 0.5/d² over the captured pass
- Final ||L_W_est - L_W_true|| < 0.30 m (real-frame Gazebo data with
  ArUco detection noise + bbox quantization)
- Number of frames processed ≥ 30 (require enough parallax)

**verdict.py** combines both. If either fails, exit code = 1 + JSON
contains failure category.

## Reproduction

```bash
bash examples/sentai_runtime/experiments/s131_lifter_replay/run.sh

# Verbose (per-frame state dump):
S131_VERBOSE=1 bash .../run.sh

# Synth-only (skip replay):
S131_SYNTH_ONLY=1 bash .../run.sh
```

Outputs land in `/tmp/s131_lifter_replay/`:
- `synth_result.json` / `synth_state.csv`
- `replay_result.json` / `replay_state.csv`
- `summary.json`

## What this DOES NOT validate

- ARM compute budget (deferred to L5 C++ port + on-board DWT timing).
- ARM numerical precision (single-precision FP) — deferred to L5 port.
- Real-world camera mount tolerance (deferred to Pas 2 calibration).
- Multi-landmark EKF interaction — s131 is single-landmark.
- Bbox extraction noise from non-fiducial classes — s131 uses ArUco
  bbox (high SNR); real classes (cube, cardboard) deferred to s132.

## Related

- objects_plan.md §3 Stage 5, §11.5 R1 (inverse-depth numerical
  stability), §20 Bibliography
- [[objectsplan-l5-handoff]] (memory)
- [[s130-45baseline-shipped]] (frames source)
- Sim.md §10x ([[sentai-sim-journal]] — used for capture journal)
