---
name: s193-calib-18-iters-partial-progress
description: "s193 calib bringup ground state after 18 iters 2026-05-23 evening — RPYT axis ID architecture validated, SAMPLE still failing.  Resume tomorrow."
metadata: 
  node_type: memory
  type: project
  originSessionId: d28bcfbd-5126-440f-a985-6fa6451f2400
---

s193 = first calib bringup attempt with T10 detector + 7-marker pad.
**18 iters, all FAIL** at SAMPLE→SAFETY abort, but arc of progress:

## Validated architecture (iter17/18)

1. **RPYT thrust ramp + PD altitude lock** (phases 1-3b) keeps drone
   stable airborne.  s192 iter1 PASS regression confirms.
2. **`takeoff()` (NOT `go_to()`) for handover** — operator-found bug:
   `plan_go_to` with `planner.state==IDLE` reads `traj_eval_invalid()`
   = `pos=NaN` → trajectory polynomials NaN → setpoint NaN → drone
   crashes (yaw spike, fall).  `plan_takeoff` reads cf2 Kalman state
   explicitly via CRTP handler → valid trajectory start.
   See `planner.c:106 plan_current_goal`, `pptraj.c:280
   traj_eval_invalid`.
3. **RPYT-based axis ID before handover** — drone in proven-stable
   RPYT mode, pitch+roll pulses move body XY, PnP captures pad-frame
   displacement, compute R_body_to_pad, commit_R, THEN handover.
   iter17: axis_id_rpyt RAN ✅, R committed ✅ (rc=0), handoff ✅,
   SAMPLE started.

## Open issues for tomorrow

1. **R math contamination**: drone drifts XY between pulses (no XY
   position hold in RPYT) → pitch_disp and roll_disp end up nearly
   antiparallel (dot ≈ -1).  Gram-Schmidt orthogonalization fails
   when input vectors are degenerate.  Need:
   - Use 2-pulse delta (p2−p1 instead of p1−p0) to cancel drift, OR
   - Reduce settle time, OR
   - Use cf2 hover() (Generic Commander velocity) for axis ID so cf2's
     own velocity PID brakes drone between pulses
2. **SAMPLE failure post-handoff**: even with rough R committed,
   orchestrator's SAMPLE → SAFETY abort within 4s.  Drone wanders off
   pad during the cross sweep.  Possibly orchestrator's own ExtPos
   forwarder uses our rough R + computes wrong drone_W → cf2 navigates
   wrong way.  Investigation needed in `sentai_calib_bringup.cc
   phase_sample_`.

## Key insights to remember

- **PnP yaw = ±90° is CORRECT, not a bug** — SDF GT
  `R_cam_to_body=[[0,1,0],[1,0,0],[0,0,-1]]`, so camera mount is 90°
  rotated.  pnp_yaw vs cf2_yaw differ by exactly this mount rotation.
  Don't reject PnP based on yaw mismatch — calib's JOB is to discover
  this rotation.
- **Camera bridge is fine** — Gazebo publishes 30 fps, bridge receives
  30 fps.  Launch_sim "FPS=3 Hz" warning is bug in measurement
  (counts conditionally-emitted log lines, not actual frames).
- **Frozen frames in bridge log are post-crash** — drone stationary
  on ground, camera sees same scene.  Always cross-reference with
  GT to know real position per [[always-cross-reference-logs-with-gt]].
- **cf2 Kalman has NO independent Z source** without baro/TOF/flow.
  Must send ExtPos continuously.  Single bad packet (innov >> stdDev
  = 0.01 m) corrupts Kalman almost instantly.
- **Innovation gate must allow recovery** — if Kalman corrupted, all
  subsequent valid PnP reads have high innovation → gate rejects
  forever → drone runs IMU-only → drifts.  Need force-accept after
  N rejections (currently 5 → 165 ms IMU drift, ~1.4 mm position).

## Files / state

- `mission_s193.py`: 1100+ LoC, takeoff()-based handoff +
  `_phase4_5_axis_id_rpyt` + innovation-gated `_extpos_guarded` +
  `_extpos_force_z` (z-only fallback) + `Z_FALLBACK_M=0.5` const for
  pure last-resort
- 18 iter folders with summary.json + journal.txt + verdict_s193.json
  per iter
- `ideas/external_patches.md`: added asymmetric N marker to
  upstream `sentai_whycon_small.sdf`
- `sim/scripts/audit_anti_cheat.sh`: exempted host-side validator +
  s190 journal narrative (pre-existing false positives, no new
  violations from s193)

## Iter table

| iter | warmup | handoff | axis ID | land | observation |
|---|---|---|---|---|---|
| 1 | n/a | HL takeoff | — | 1 cm (no lift) | ekf hallucinates climb |
| 4-12 | 0.5-2s tuning | go_to (NaN!) | — | 17-133 cm | yaw spike + crash |
| 13 | 0.3s | **takeoff()** | — | n/a | LUCKY stable through p5 |
| 14-16 | 0.3s | takeoff() | MP `_phase5_5` | 13-29 cm | drone unstable before axis ID |
| 17 | 0.3s | takeoff() | **RPYT phase 4.5** | 66 cm | ✅ axis ID ran, R committed |
| 18 | 0.3s | takeoff() | RPYT + Gram-Schmidt | 13 cm | by_perp collinear fail (drift) |

## Resume plan

1. Read this memory + diary/2026-05-23.md (evening section).
2. iter19 priority: fix R math via 2-pulse delta or hover-based axis ID.
3. After axis ID stable, investigate SAMPLE failure mode.
4. If SAMPLE can't be salvaged → MP-side rewrite per
   [[op-s10-w21-t14-rpyt-only-calib-idea]].

Related: [[op-s10-w17-t10-synth-bench-shipped]],
[[always-cross-reference-logs-with-gt]],
[[op-s10-w21-t14-rpyt-only-calib-idea]].
