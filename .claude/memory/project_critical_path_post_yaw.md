---
name: critical-path-post-yaw-2026-05-19
description: "Big-picture critical path to thesis defense, snapshot 2026-05-19.  Yaw drift in cf2 SITL is the LAST open algorithmic question; once closed (or scoped-out to position-only flight), remaining work is execution along a known path, ~5-7 weeks: Phase 1 close W14 + L1-L7 stack (~2 wk), Phase 2 ARM bring-up OP-S9 (~1-2 wk, HIGHEST RISK), Phase 3 L7 indoor demo OP-S10-W8 + 20-run campaign (~1-2 wk), Phase 4 outdoor PX4 OP-S10-W9 (~3-4 d), Phase 5 eval chapter OP-S10-W10 (~1 wk).  Closes OP-M3..M5."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Snapshot 2026-05-19, after T22 ARM build fix + yaw control
deadlock identification.  Full text in `diary/2026-05-19.md`
section "Big picture — what's left to defense".

## Shipped foundation (SIM-validated)

L2 objects, L3 places, L4 servo, L4.5 image-only nav, L5
lifter, L6 explore, S6-W1 calib, S6-W3 aruco, S10-W1..W4
descriptors (PHOG/GIST/baseline/HSV), S10-W11 T1+T2+T3+T4
prep pipeline, S10-W12 safety, S10-W13 fr, S10-W14 autotune
algorithm.

## Open algorithmic question: yaw

T19/T20/T21 yaw control in cf2 SITL is a documented dead-end
(positive-feedback resonance when ExtPose carries PnP-derived
quat; HL `go_to(yaw=)` over-rotates with our 6 Hz PnP noise
profile).  Three strategic options to surface to operator:

1. **Fork cf2 firmware**: rewrite HL Commander to tolerate noisy
   attitude estimates.  High effort, opens a new vendor-side
   maintenance burden.
2. **Try cf2 NL3 (Mellinger) controller**: more aggressive but
   may tolerate yaw noise better.  ~1 week to evaluate.
3. **Thesis-scope reduction**: constrain to position-only flight
   (no commanded yaw).  Acceptable for the §23.1 north star —
   the demo says "drone takes off, explores, lands" not "drone
   rotates 360° between waypoints".  Easiest path, would close
   yaw entirely.

Today's diary explicitly notes yaw is OFF today's lanes —
needs operator strategy call before more implementation push.

## Critical path (assuming yaw closes or is scoped-out)

| Phase | Scope                                                    | Weeks | Closes  |
|-------|----------------------------------------------------------|-------|---------|
| 1     | Close W14 (T7 persist gains, T18-B median Z) + W11 finish + W5 FFT-mag + W6 recall + W7 L1 tracker | ~2  | —       |
| 2     | OP-S9 ARM bring-up (W1..W5: LOG RX FIFO, radio LOG, ARM flash smoke, DWT timing, 60s capture) | ~1-2 | OP-M2   |
| 3     | OP-S10-W8 L7 indoor demo (5×5 m, 4-6 ArUco, autonomous explore→revisit→land<15 cm) + 20-run campaign | ~1-2 | OP-M3   |
| 4     | OP-S10-W9 outdoor PX4 (2-3 flights, GPS GT)              | ~3-4 d| OP-M4   |
| 5     | OP-S10-W10 evaluation chapter + writeup                  | ~1   | OP-M5   |

Phase 2 carries the highest discovery risk — TPU/PXP/USB SEMC
contention under real camera load, FxUser throughput vs SIM
stub, IRQ latency budget under FR drain.  Expect 1-2 multi-day
debugging tangents.

## Total estimate

**~5-7 weeks of focused work** post-yaw decision to defense
readiness.  Phases 1, 4, 5 are well-scoped (mostly execution
+ writing).  Phases 2, 3 carry intrinsic discovery risk hard
to compress.

## Cross-cutting

- **OP-S10-W15** ARM memory budget design doc — open ToDo, not
  scheduled; promotes on next build break OR embedded chapter
  start.  See [[op-s10-w15-arm-memory-budget]].
- DNN Track B, OP-S6-W2 Multi-object Yaw-Wahba, full SM3 tracker
  — all in `FutureWork.md`, not MVP path.
