# s194 — Closed-loop PD per-axis ID with level-settle capture

**WBS**: OP-S10-W21-T15
**Started**: 2026-05-24
**Status**: ⬜ planning

## Claim

The s193 axis-ID failure mode (open-loop pulse + fixed-time settle →
pitch_disp and roll_disp end up nearly collinear because the drone drifts
during the 1.2 s settle window) can be removed by replacing fixed-time
settle with a **closed-loop PD that parks the drone at a target offset**
along the discovered body-axis direction.  Symmetric ±δ targets cancel
any residual drift; **level-attitude settle (150 ms of zero-attitude
RPYT) before each capture** minimises PnP-precision degradation from
camera tilt.

Specifically, for each body axis (pitch then roll):

1. **Exploratory pulse** (4°, 250 ms) → measure Δp_pad → unit vector
   `d_axis` (the direction in pad-frame where this body axis pushes
   the drone — discovered live, no `R_cam_to_body` prior needed).
2. **PD-park** drone at `p0 + δ·d_axis` (δ = 6 cm).  Attitude command
   = `Kp · err_along − Kd · vel_along`, saturated ±7°.  Loop until
   `|err_along| < 1 cm` and `|vel_along| < 5 cm/s` for 5 consecutive
   ticks (≈ 165 ms stable).
3. **Level-settle** — stream `_rpyt(0, 0, 0, thrust_PD)` for 150 ms.
   cf2 attitude PID (1 kHz inner loop) brings drone to truly level
   in < 100 ms.  Drift in this window ≤ 0.05 m/s · 0.15 s = 7.5 mm.
4. **Capture** PnP median over 0.4 s (drone level, near-stationary).
5. **PD-park** at `p0 - δ·d_axis`, level-settle, capture.
6. `axis_disp_pad = (p_+ − p_-) / 2`  — drift-canceled symmetric pair.

The two `axis_disp_pad` vectors (pitch, roll) build R_body_to_pad
directly, without Gram-Schmidt rescue.

## Pass criteria

- `dot(pitch_disp_unit, roll_disp_unit) | < 0.3`
  (iter17/18 saw ~−1; should drop toward 0)
- `|θ_p − θ_r| ∈ [80°, 100°]`
  (iter17 saw 19° apart; should be near 90° apart)
- `mag_pitch_disp ≥ 2 cm` and `mag_roll_disp ≥ 2 cm`
  (signal amplitude — by symmetric diff should be larger than iter18's
  single-sided δ)
- `rc_commit_R == 0`  (R math is orthogonal)
- PnP returns valid (n≥4 markers) on ≥ 80% of capture ticks  (FOV held)
- Calib orchestrator SAMPLE → KABSCH → AUTOTUNE phase progression
  successful with committed R (this is the downstream win)
- SIM only: drone returns to ≤ 10 cm of takeoff origin
  ([[sim-test-must-return-home]])
- SIM only: anti-cheat — SentAI sensors fed ONLY by camera frames +
  CRTP LOG, no GT injection ([[sentai-sim-air-gapped-from-truth]])

## Why a new experiment (not just s193/iter19)

Operator-stated 2026-05-24: "mai bine incepu un nou s ca sa nu
contaminam codul de ieri".  s193 mission file holds the open-loop
algorithm; preserving it lets us A/B compare numerically.  s194 ships
the closed-loop algorithm fresh.  s193 stays as the dead-end record
per [[experiments-in-own-folder-log-dead-ends]].

## How to run

```bash
bash run.sh iter1     # closed-loop PD axis ID, level-settle 150 ms
```

## Top-level files (shared across iters)

- `mission_s194.py` — mission (closed-loop PD per-axis ID + level-settle)
- `verdict.py` — symlinked from `../s193_calib_full_postT10/verdict_s193.py`
  (same acceptance check: handoff XY error + calib accepted + land XY)
- `run.sh` — launcher (per-iter sub-folder output)

## Iter results table

| Iter | Hypothesis / change | Key number | Pass? |
|---|---|---|---|
| iter1 | baseline closed-loop PD, δ=6 cm, Kp=10°/m, Kd=14°/(m/s), level-settle 150 ms | (TBD) | ⬜ |

## Tilt + PnP precision — design note

Operator concern 2026-05-24: "pe X si pe Y orice puls inclina putin
drona, e posibil sa intre intr-o oscilatie, ... e posibil sa masori
doar o perspectiva skewed".

Math answer: PnP/IPPE solves 6-DOF (R, t) jointly — drone POSITION is
unbiased w.r.t. camera tilt.  Verified by the 368-frame B2 synthetic
ablation (mixed roll/pitch frames): pose tr RMSE = 16 mm.

Statistical answer: PnP **variance** is higher when tilted (skewed
marker ellipses, partial FOV, IPPE mirror ambiguity more likely).
Mitigation: 150 ms of zero-attitude streaming before capture brings
drone to true level via cf2 inner-loop attitude PID @ 1 kHz.

Future enhancement: if iter1 shows residual contamination, expose
`sentai.crazy.attitude_get()` on SIM (currently ARM-only) and gate
captures with `|pitch|<1°, |roll|<1°`.  Tracked as [[s194-imu-gate]]
if needed.

## Risks

- **PD overshoot** — if Kp/Kd guess is off, drone oscillates instead
  of parking.  Saturation ±7° limits worst case; PARK_TIMEOUT_S = 3 s
  per leg.  Failure mode: `_pd_park` returns `None` → axis_id aborts
  cleanly, RTL via existing safety path.
- **Exploratory pulse loses FOV** — if d_axis points toward pad edge,
  drone may leave FOV during exploration.  Pulse amplitude = 4° for
  250 ms → peak velocity 0.2 m/s → max excursion 5 cm.  Pad half-width
  is 10 cm.  Safe but tight; abort if PnP fails > 6 ticks.
- **Roll d_axis collinear with pitch d_axis** — physically impossible
  unless cf2 attitude PID is degenerate.  Sanity guard:
  `|dot(d_pitch, d_roll)| > 0.8` → abort.

## Anti-cheat

- sentai_sim consumes camera frames + CRTP telemetry (pose, yaw) only
- GT recorder runs HOST-SIDE, output JSONL is post-mortem in
  `verdict.py` for the land-XY claim
- No GT topics, model poses, or world state injected into sentai_sim
- `MARKER_WORLD` is the **map** (known pad layout), not GT —
  same set exists on real HW from earlier calibration
