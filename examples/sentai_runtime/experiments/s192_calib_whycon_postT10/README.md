# s192 — WhyCon detection in Gazebo flight, post-T10

**WBS**: `OP-S10-W21-T12` (regression test against T10's detector port)
**Started**: 2026-05-23
**Status**: ✅ iter1 PASS — T10 detector regression closed in flight

## Claim

After `OP-S10-W17-T10` (commit `b4eb37ce`, OpenCV-parity port of
sentai_aruco.cc WhyCon), the embedded detector should reliably see
the 6-marker pad in Gazebo during actual cf2 flight — not just on
static PGM dumps.  The 368-frame static ablation already gave
sentai_sim recall **0.998** with sub-pixel centroid parity vs the
OpenCV reference.  This experiment is the in-flight regression
companion.

The failure mode this regressed against: yesterday's s191/s187/s190
runs all returned `z_pnp = -1.0, n = 0` for hundreds of camera ticks
during the climb/hover phase, because the pre-T10 detector under-
counted (sometimes returned 0 markers across multiple frames).

## World — verified before run

Upstream world used:
`/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf`

**6 SYMMETRIC WhyCon markers** (no 7th asymmetric `N` — that one
exists only in the run-local dataset world copy from TD-S10-B1):

```text
NW (-0.08, +0.08, 0.005)
NE (+0.08, +0.08, 0.005)
W  (-0.06,  0.00, 0.005)
E  (+0.06,  0.00, 0.005)
SW (-0.08, -0.08, 0.005)
SE (+0.08, -0.08, 0.005)

marker outer diameter: 0.0544 m
camera intrinsics:     fx=fy=288.3, cx=160, cy=120
image resolution:      320 x 240
```

Symmetric layout: this is the same world that produced the
mirror-branch PnP outliers measured by T10's offline ablation —
`get_drone_pose_tuple` uses prior-guided correspondence + yaw-anchor
mirror picker per [[feedback-yaw-anchor-mirror-picker]] to handle
the ambiguity.

## Pass criteria

iter1 is detection-only (no PD nav, no sweep, no Kabsch):

1. Drone takes off via RPYT thrust ramp (PHASES 1-4 cloned from s191).
2. During a **5 s detection-measurement window** at PD altitude hold:
   - `n_markers >= 4` in at least **80 %** of camera ticks
   - At least one tick reaches `n_markers == 6` (full pad in FOV)
3. Land via thrust ramp-down.
4. Drone lands ≤ 10 cm of physical takeoff origin (per
   [[sim-test-must-return-home]]) — soft check this iter, hard from
   iter2 onward when XY nav is added.
5. `bash sim/scripts/audit_anti_cheat.sh` PASS.

Anti-cheat: SentAI sensors fed ONLY by camera frames + CRTP LOG
per [[feedback-sentai-sim-air-gapped-from-truth]].  No GT injection,
no plugin cheats.

## How to run

```bash
bash examples/sentai_runtime/experiments/s192_calib_whycon_postT10/run.sh iter1
```

## Top-level files (shared across all iters)

- `mission_s192.py` — minimal WhyCon-detection mission (phases 1-4
  + measurement window + ramp-down land)
- `verdict_s192.py` — parses journal → detection recall + n-marker
  distribution + PnP-z distribution + landing offset
- `run.sh` — orchestrator (clone of s191's, simplified)
- `README.md` (this file)

## Iter sub-folders

```
s192_calib_whycon_postT10/
├── README.md
├── mission_s192.py
├── verdict_s192.py
├── run.sh
└── iter1/
    ├── README.md
    ├── mission_s192_journal.txt
    ├── mission_s192_summary.json
    ├── cf2_gt.jsonl
    ├── sentai_repl.log
    ├── verdict.log
    └── verdict_s192.json
```

## Iter results table

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | T10 detector port detects markers during cf2 hover | 151/151 (100%) n>=6, land 7.4 cm | ✅ |

## Result (final, after experiment completes)

iter1 PASS proves T10's detector port closes the 22-May
detector-recall blocker in flight.  Detection is 100 % at the
hover altitude (~0.92 m mean, 0.67-1.01 m range).  False-positive
rate ~15 % in flight (vs 5 % static) and PnP-valid rate 43 %
remain open diagnostics for iter2+.  Downstream s191 phases (PD
XY nav, sweep, Kabsch, hold-validate) are unblocked.

## Cross-refs

- T10 SHIPPED: commit `b4eb37ce` — synth perception bench + WhyCon
  OpenCV-parity port + 368-frame ablation
- T10 paper figures: `dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/figures/`
- Predecessor: s191 iter-1..19 (calib RPYT-only cascaded PD —
  blocked by detector under-count)
- Memory: [[op-s10-w17-t10-synth-bench-shipped]]
