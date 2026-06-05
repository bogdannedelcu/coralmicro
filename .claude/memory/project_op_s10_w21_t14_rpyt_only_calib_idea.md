---
name: op-s10-w21-t14-rpyt-only-calib-idea
description: "FALLBACK PLAN — implement entire sentai.calib pipeline in MicroPython using Classic RPYT throughout, skipping HL handoff.  Reserve if RPYT→HL handover keeps failing."
metadata: 
  node_type: memory
  type: project
  originSessionId: d28bcfbd-5126-440f-a985-6fa6451f2400
---

**Operator-stated 2026-05-23**, during s193 iter9 post-mortem:
> "ma gandesc daca nu cumva putem sa ne intoarcem la ruta cand
> faceam calibrarea folosind RPYT... acum ca avem markerii corecti.
> Notam ideea si poate revenim asupra ei daca nu reusim. Eu tot
> sper sa reusim sa facem handover..."

## The fallback plan

If RPYT→HL handoff continues to fail (s193 iters 4-9 all failed at
SAMPLE→SAFETY despite detector fixes, marker pad fix, warmup tuning,
innovation gate, z-fallback constant), then **abandon the
`sentai.calib.run_bringup()` C-side orchestrator and re-implement
the bringup pipeline in MicroPython using Classic RPYT throughout**.

## Why this would work

Per s192 iter1 PASS proof: RPYT thrust ramp + PD altitude lock +
ExtPos warmup gives **stable hover** with the now-working T10
detector (recall 0.998).  The detector + PnP work.  The breaking
point in s193 is the **handover to HL/Generic Commander** for the
calib SAMPLE phase.  HL/Generic expect cf2 Kalman to be well-
anchored, but our camera bridge runs at 3 fps → Kalman has 333ms
lag → HL trajectory commands diverge from physical state → drone
goes chaotic.

Classic RPYT bypasses this entirely.  It writes thrust+attitude
direct to motor mixer, no Kalman dependency.

## Scope of MP re-implementation

What `sentai_calib_bringup.cc` does — and the MP equivalent:

| C-side phase | What it does | MP equivalent |
|---|---|---|
| SAMPLE | hover() to 4 cross poses, capture markers+EKF per pose | RPYT-PD navigation to (±r,0) / (0,±r) corners with PnP feedback, capture per pose |
| KABSCH | SVD on tvec ↔ marker_world correspondences | reuse `sentai.calib.run_kabsch` binding (C-side, no MP work) |
| AUTOTUNE_X/Y | Åström-Hägglund relay feedback | reuse `sentai.calib.task_start(axis, ...)` — but ensure relay worker doesn't switch to hover() internally |
| HOLD | closed-loop with kp_x, kp_y for 10s, measure drift | reuse `sentai.calib.hold_start(kp_x, kp_y, ...)` |
| SAVE | persist to /system/calib.ini | reuse `sentai.calib.save()` |

## Risk

`sentai_calib_task_start()` (relay autotune) internally uses
`sentai_crazy_hover()` for velocity setpoints.  Same Generic
Commander handoff problem.  Would need to either patch the C
worker to accept RPYT mode OR re-implement autotune in MP.

## Estimated effort

- SAMPLE phase MP rewrite: ~150 LoC (extend s191's `_phase7_cross_sweep`)
- Skip AUTOTUNE for first iter — use s174 baseline Kp=0.39 directly
- HOLD validation: ~100 LoC

Skip-autotune variant is ~250 LoC MP.  Full re-implementation
(including MP autotune) is ~500 LoC.

## When to invoke

After s193 reaches iter 12-15 of handover tuning without progress.
Current iter is 9.  Give the handover path 3-6 more iterations
before fallback.
