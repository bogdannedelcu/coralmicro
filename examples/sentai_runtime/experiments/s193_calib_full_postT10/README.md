# s193 — calib bringup full pipeline, 7-marker pad, post-T10 detector

**WBS**: `OP-S10-W21-T4` phase-3
**Started**: 2026-05-23
**Status**: ⬜ iter1 — first full-stack rerun against post-T10 detector + 7-marker pad

## Claim

`sentai.calib.run_bringup()` runs all 8 phases (SAMPLE → KABSCH →
AUTOTUNE_X → AUTOTUNE_Y → HOLD → SAVE → DONE_OK) end-to-end on the
new 7-marker asymmetric pad without manual intervention.  Same code
that runs here on cf2 SITL is the production bringup method on a
real drone bench (per [[sentai-calib-is-production-bringup]]).

This experiment closes the loop on:
1. **T10** (commit `b4eb37ce`) — WhyCon detector OpenCV-parity port,
   368-frame static recall ≥ 0.998, sub-pixel centroid parity.
2. **s192 iter1 PASS** — detection works in actual cf2 flight at z=0.92 m.
3. **7-marker pad** — asymmetric `whycon_N` at (+0.02, +0.10, 0.005)
   added to upstream `sentai_whycon_small.sdf`
   (see `ideas/external_patches.md` 2026-05-23).  Resolves the
   dual-solution PnP ambiguity that 6-marker symmetric layouts produce.

## World — verified before run

Upstream `sentai_whycon_small.sdf` post-patch:

```text
NW (-0.08, +0.08, 0.005)
NE (+0.08, +0.08, 0.005)
W  (-0.06,  0.00, 0.005)
E  (+0.06,  0.00, 0.005)
SW (-0.08, -0.08, 0.005)
SE (+0.08, -0.08, 0.005)
N  (+0.02, +0.10, 0.005)    ← asymmetric, added 2026-05-23
```

Camera: fx=fy=288.3, cx=160, cy=120, 320×240.  Marker diameter 0.0544 m.

## Pass criteria (from OP-S10-W21 spec)

| Metric                              | Threshold        |
|-------------------------------------|------------------|
| `R_cam_to_body` drift from SDF GT   | < 1.0°           |
| `cam_offset_B` vs SDF GT (∞-norm)   | < 5 mm           |
| `kp_x`, `kp_y`                      | ∈ [0.30, 0.50]   |
| Hold-validation rms drift           | < 30 mm / 10 s   |
| Hold-validation max drift           | < 80 mm          |
| Persistence round-trip after reboot | bit-identical    |

Anti-cheat: orchestrator consumes only WhyCon PnP (camera frames via
bridge) + cf2 CRTP LOG telemetry.  No GT injection.
`[[sentai-sim-air-gapped-from-truth]]`.

## How to run

```bash
bash examples/sentai_runtime/experiments/s193_calib_full_postT10/run.sh iter1
```

## Top-level files

- `mission_s193.py` — MP orchestration: setup → pre-arm extPos warmup →
  HL takeoff → climb-seen poll → run_bringup → poll phases → assert →
  return-to-home + land
- `verdict_s193.py` — post-mortem acceptance gate vs SDF GT thresholds
- `run.sh` — orchestrator (takes iter tag arg)

## Iter results table

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | First full bringup post-T10 + 7-marker pad | TBD | TBD |

## Result (final)

(Filled in after the last iter ships.)
