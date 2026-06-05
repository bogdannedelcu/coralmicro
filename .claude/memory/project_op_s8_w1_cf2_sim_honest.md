---
name: op-s8-w1-cf2-sim-honest
description: "OP-S8-W1 crisis WP opened 2026-05-17.  Anti-cheat removal for cf2 SITL + path to honest VPE-anchored validation.  T1-T5 SHIPPED in opening session (cheat plugin disabled, KNOWN_POSITIONS_M aligned, VPE forwarder, position setpoint, gentle PID).  T6-T11 deferred to follow-up sessions (cf2 spawn reproducibility, GT verdict gate, new FlowBaseline canonical, audit of all prior drift claims).  All prior cf2-side drift numbers are SUSPECT until T8 closes."
metadata:
  type: project
  node_type: memory
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

## Why this WP exists

See `[[cf2-sitl-cheat-odom-gt]]` for the forensic story.  Short
version: `gz-sim-odometry-publisher-system` was feeding cf2 EKF
Gazebo's GT pose at 200 Hz, masking real flow/PnP performance in
every cf2-side test.

## Scope

Restore cf2 SITL to a SIM that mirrors real-HW behaviour:
- cf2 EKF runs on IMU + flow + (vision VPE when markers visible)
- No process in the control loop consumes Gazebo GT
- Test verdicts validate against Gazebo GT host-side, post-mortem
- Reproducibility: cf2 always spawns at the same physical pose

## Tasks (see `ideas/wbs.md` OP-S8-W1 section)

| Task | Status | Notes |
|---|---|---|
| T1 disable cheat plugin | ✅ SHIPPED | `model.sdf.jinja:393` commented out + backup |
| T2 `KNOWN_POSITIONS_M` + `MARKER_SIZE_M` align | ✅ SHIPPED | `aruco_detector.py:38` |
| T3 VPE forwarder | ✅ SHIPPED | `aruco_hover.py:_vpe_send_if_due` |
| T4 position setpoint | ✅ SHIPPED | `aruco_hover.py:hover_loop` |
| T5 gentle PID + 5 Hz VPE | ✅ SHIPPED | `DAMP_PARAMS` + `_VPE_RATE_HZ_MAX` |
| T6 reproducible cf2 spawn | ⬜ TODO | run.sh respawn is racy; cf2 ended at (0.57, -0.92) in last trial |
| T7 GT verdict gate | ⬜ TODO | trust EKF only when GT confirms |
| T8 new FlowBaseline canonical | ⬜ TODO | 3 trials, mean ± σ, with GT |
| T9 docs (Sim.md, agent.md, objects_plan §23 risks) | ⬜ TODO | |
| T10 audit prior drift claims | ⬜ TODO | s127/s130/s142/s147 etc. |
| T11 revisit OP-S10-W11-T5.A (1m square) | ⬜ TODO | s164/s165 numbers were cheat-masked |

## Hard freeze

Until T8 closes (new FlowBaseline canonical established), NO cf2
drift number from any test in this project is trustworthy for thesis
text.  Cite this WP if asked about drift validation.

## Cross-references

- `[[cf2-sitl-cheat-odom-gt]]` — forensic finding
- `[[sentai-sim-air-gapped-from-truth]]` — broader anti-cheat rule
- `[[experiments-start-from-origin]]` — reproducibility
- `[[flowbaseline-canonical-config]]` — SUPERSEDED, awaiting T8
- `ideas/op_s8_w1_handoff.md` — tomorrow's plan
