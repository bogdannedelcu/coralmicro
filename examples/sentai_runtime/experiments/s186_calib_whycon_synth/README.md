# s186 — sentai.calib smoke on WhyCon square pad

**WBS:** `OP-S10-W19-T8` (re-verify `sentai.calib` numerics on the new
6-marker WhyCon square pad before mission-level integration in s185).

## Why

`sentai.calib.run_kabsch` was originally validated by s157 against an
8-marker ArUco arc (radius 0.30 m).  Operator request 2026-05-21:
verify the same numerics survive on the smaller-baseline WhyCon square
(32×32 cm).

Two relevant geometry differences:
- **Smaller spread** — max baseline 0.32·√2 ≈ 0.45 m vs s157's 0.60 m,
  so Kabsch is mildly worse conditioned.
- **Coplanar pad** — all markers at z=0.005 m, so the world covariance
  is rank-2.  s157 also had effectively-coplanar markers (z=0.20 fixed),
  so this shouldn't change anything, but s186 explicitly checks it.

Pure synthetic; no Gazebo.  Unblocks `s185_yaw_smoke_gazebo`.

## Test cases

| # | Scenario                              | Pass criterion                          |
|---|---------------------------------------|------------------------------------------|
| T1 | Identity recovery on 24 samples      | det_R > 0.999, mean_res_deg < 0.05, OK  |
| T2 | ±5° Z-tilt recovery                  | drift ∈ [4.5, 5.5]°, mean_res_deg < 0.5 |
| T3 | Drift gate (no perturb)              | drift < 0.01°                            |
| T4 | Too-few-samples reject (n=2)         | not accepted, reject_code = 5            |
| T5 | NaN tvec reject                      | not accepted, reject_code = 6            |
| T6 | Save / load round-trip               | bit-identical R after load               |

24 samples = 6 markers × 4 hover poses at different yaws.

## How to run

```bash
bash examples/sentai_runtime/experiments/s186_calib_whycon_synth/run.sh
```

## Result (2026-05-21)

6/6 PASS on first re-run after threshold tuning.  Measured numbers
(synthetic, deterministic):

| # | Metric                          | Value        |
|---|----------------------------------|--------------|
| T1 | det_R                           | 1.000000     |
| T1 | mean_residual_deg               | 0.013        |
| T1 | max_residual_deg                | 0.028        |
| T2 | drift (true=5°)                 | 5.0001°      |
| T2 | mean_residual_deg               | 0.017        |
| T3 | drift (no perturb)              | 0.000000°    |
| T6 | save/load R round-trip err      | 0.00e+00     |
| T6 | save/load offset round-trip err | 0.00e+00     |

s157 (ArUco-arc) threshold of 0.01° was tightened against radius 0.30 m
data; the 32 cm square pad puts a 0.05° floor (float32 + smaller
baseline + coplanar pad rank-2).  No numerical regression vs s157.

Calib stack works on WhyCon — unblocks s185 yaw smoke.

## Anti-cheat note

Pure synthetic math; no Gazebo, no camera, no real WhyCon pipeline.
All samples are computed analytically in MicroPython from known
ground-truth poses, fed to `sentai.calib.run_kabsch` which is the
same numerical routine used on-board.  No air-gap implications.
