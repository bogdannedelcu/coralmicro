# s183 — WhyCon Square Baseline (clean re-base of s182 iter-10c)

WBS: **OP-S10-W19-T4 step 5**.

Clean restart of the WhyCon Gazebo eval after 11 iterations in
`s182_whycon_gazebo_eval` accumulated too much history.  Same
methodology as s182 iter-10c (which converged), but with the
operator-specified SQUARE marker layout and only the clean working
artifacts.  Legacy s182 retained as historical record.

## Marker layout (operator-specified, 2026-05-20)

Square 32 × 32 cm with proportions 1 : 1 : 0.75 (`unit = 0.16 m`):

```
NW (-0.16, +0.16)            NE (+0.16, +0.16)
     ╲                              ╱
       W (-0.12, 0)    E (+0.12, 0)
     ╱                              ╲
SW (-0.16, -0.16)            SE (+0.16, -0.16)
```

4 corners at `(±0.16, ±0.16)`, 2 mid-bar markers at `(±0.12, 0)`.
Same physical scale as the prior H-pattern.  Markers used:
6× Krajník annulus at z = 0.005 m on 0.12 m PBR boxes.

## What's baked in (from s182 iter-10c)

1. **`WHYCON_PNP_ANNULUS_FACTOR = 1.166`** (analytic Krajník, scale
   sweep validated at residual = 0.77 mm).  Set in `sentai_aruco.cc`.

2. **Phase W3 concentric inner-disc validation ON by default**
   when `sentai.markers.init('whycon')`.  Kills the duplicate
   centre-dot / outer-annulus detections that the W1 fill-ratio
   gate alone lets through.

3. **`sentai_fr_push_frame()` wired from `sentai_markers_detect_frame()`**
   for post-mortem replay (used by `s182_replay.py`).

4. **Reflection-safe Kabsch** in `verdict_sota.py`: 6 coplanar
   markers → 2-fold ambiguity by reflection across the marker
   plane → pick the solution with drone above the markers.

5. **s174-style mission pattern**:
   - `takeoff(Z_HOLD=0.60, dur=2.5)` then sleep 0.5 + 4.0 s
   - `crazy.hl_stop()` + 5 × `crazy.hover(0,0,0,Z_HOLD)` @ 30 ms
   - hover-and-log loop with `crazy.hover` re-assert every 0.5 s
   - VPE-Z to cf2 EKF via `crazy.send_extpos(cf2.x, cf2.y, median_tz)`

6. **Absolute-wall-clock GT pairing** in both verdicts (legacy
   `verdict.py` + SOTA `verdict_sota.py`).  Both sentai tick
   `__ts_ms` and GT `t_wall` come from host `time.monotonic()`,
   so we pair by absolute time, not normalize-to-zero (that was
   pairing post-takeoff hover ticks with pre-takeoff GT rows,
   showing GT "on the ground" while the drone was at 0.6 m).

## Anti-cheat compliance

Per `sim/ANTI_CHEAT.md` and `[[sentai-sim-air-gapped-from-truth]]`:

- `sentai_sim` reads **camera frames** via `/tmp/sentai_cam.sock`
  (from `gz_to_uds_bridge`, distrobox) + **cf2 telemetry** via
  CRTP LOG on UDP 127.0.0.1:19850.  No ground truth.
- VPE-Z is computed from WhyCon detections (camera-derived), NOT
  from GT.  Fed to cf2 EKF via `crazy.send_extpos`.
- `gt_recorder` runs **host-side**, writes `cf2_gt.jsonl` for
  post-mortem comparison.  Never feeds back into `sentai_sim`.
- `sentai_sim` binary is linked WITHOUT `gz-transport`/`gz-msgs`
  libraries; architectural air-gap by construction.

## Running

```bash
bash examples/sentai_runtime/experiments/s183_whycon_square_baseline/run.sh
```

Outputs:
- `journal.txt` — per-tick (cf2 EKF pose, detections, VPE sent)
- `cf2_gt.jsonl` — Gazebo `dynamic_pose` (host-side, post-mortem)
- `s183_xyz_world.png` / `s183_z_timeseries.png` — legacy verdict
- `s183_kabsch_xyz_*.png` / `s183_kabsch_z_*.png` — SOTA verdict
- `s183_paired.csv` / `s183_kabsch_*.csv` — raw per-tick data

## Reference numbers (s182 iter-10c, same algorithmic stack)

| Axis | WhyCon-Kabsch MAE | max | cf2 EKF MAE |
|------|-------------------|-----|-------------|
| X    | 2.31 cm           | 4.09 cm | 0.30 cm |
| Y    | 0.11 cm (1.1 mm)  | 0.35 cm | 0.15 cm |
| Z    | 2.56 cm           | 2.74 cm | 3.35 cm |

Kabsch residual: mean 0.93 mm, max 2.53 mm (geometrically perfect).

## Files

- `mission_s183.py` — flight plan (runs INSIDE `sentai_sim`)
- `run.sh` — orchestrator (cleanup, SITL launch, bridge, mission,
  GT recorder, verdict)
- `verdict_sota.py` — SOTA Kabsch SVD + permutation-Procrustes +
  thesis-quality X/Y/Z plots
- `verdict.py` — legacy per-marker median verdict (kept for the
  quick z-time-series plot)
- `SOTA_CITATIONS.md` — algorithm-to-paper map
