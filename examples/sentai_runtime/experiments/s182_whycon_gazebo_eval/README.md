# s182 — WhyCon Gazebo evaluation (anti-cheat compliant)

WBS: **OP-S10-W19-T4 step 2**.  The canonical thesis-grade
WhyCon SIM evaluation per operator 2026-05-20: "in simulator
mai întâi bineînțeles, notăm" + `[[sentai-sim-air-gapped-from-truth]]`
hard rule.

Where this fits between s181 and s182:

| Iter | Scope | What it proves |
|---|---|---|
| s181 | controlled synth (no Gazebo, no camera, no motion) | closed-form PnP math is correct (annulus-corrected → MAE <3 cm on Z) |
| **s182** | **Gazebo cf2 SITL + downward camera streaming + WhyCon detection per frame** | **real-camera detection rate, PnP accuracy under render noise + motion blur + tilt** |

## Setup

### 1. Krajník world

`sim/gazebo/sentai_whycon_world.sdf` (copy of `sentai_crazysim.sdf`
with the 4 ArUco markers replaced by Krajník — same texture, different
asymmetric cross layout — see §"Marker layout" below).  Same file
also lives at `/home/bogdan/work/crazyflie/CrazySim/.../worlds/sentai_whycon.sdf`
so the CrazySim `sitl_singleagent.sh -w sentai_whycon` launcher
picks it up.

Texture: `materials/textures/whycon_krajnik.png` — single PNG used
by all 4 markers (Krajník is ID-less; constellation pose
disambiguation is W19-T3).  Geometry: outer dark annulus 232 px /
inner white disc 139 px / centre dark dot 46 px on 512 px image,
matching the W3 sample geometry constants in `sentai_aruco.cc`.

### 2. Marker layout (asymmetric cross)

```
            N (0, +0.20)
             |
W (-0.08,0)──┼──E (+0.16, 0)
             |
            S (0, -0.20)
```

Operator-noted: 4 perfectly symmetric markers would be yaw-ambiguous.
E is shifted outward by 4 cm and W pulled in by 4 cm so the cross
has NO 4-fold rotational symmetry.  Single-marker WhyCon can't
recover yaw anyway (W17 §6.2), but the asymmetric layout is the
right shape for a multi-marker constellation pose solver (W19-T3).

Marker physical outer-ring diameter on the 0.12 m box face:
`(232/256) × 0.06 m × 2 = 0.1088 m` (PNG margin offsets reduce the
on-face circle radius).  Pass `sentai.markers.set_marker_size(0.1088)`
to match.

### 3. Mission

`mission_s182.py` runs INSIDE `sentai_sim` (per
`[[missions-run-in-sentai-only]]`) — host launches, sentai_sim flies:

  1. `sentai.markers.init('whycon')` + intrinsics + diameter.
  2. `sentai.crazy.init` + arm + takeoff to 0.6 m.
  3. Altitude sweep: hover at z ∈ {0.4, 0.5, 0.6, 0.7, 0.8, 1.0} m.
     For each altitude, dwell 30 ticks @ 100 ms; per-tick log
     `cf2 pose (CRTP), n_dets, per-marker tvec_cam / rvec_cam`.
  4. Land + disarm.

### 4. GT recording (HOST-SIDE ONLY, anti-cheat)

`sim/scripts/gt_recorder.py` runs OUTSIDE sentai_sim; subscribes
to Gazebo `/world/sentai_whycon/dynamic_pose/info`, writes
`gt/cf2_gt.jsonl`.  This file feeds the post-mortem verdict — it
is NEVER read by sentai_sim.

### 5. Verdict

`verdict.py` (host-side):
  - Parses `mission_s182_journal.txt` for per-tick WhyCon detections.
  - Parses `cf2_gt.jsonl` for Gazebo drone pose at the same wall-clock.
  - Per detected marker: `drone_world_est = marker_world + tvec_cam`
    (the assumption noted in `estimate_drone_world()` re. cam-to-body
    rotation under cf2 yaw=0; deviation shows up as a constant flip
    in the residual plot).
  - Plots `s182_xyz_world.png` — 3 panels, drone X / Y / Z est vs GT.
  - Writes `s182_paired.csv` raw paired samples.

## How to run

Prerequisites: `crazysim-garden` distrobox installed (per Sim.md §3
prep), `venv/` with `cflib` + matplotlib, Gazebo Garden 7.9 inside
the distrobox.

```bash
cd examples/sentai_runtime/experiments/s182_whycon_gazebo_eval
bash run.sh
```

`run.sh` does:
  1. Cleanup any prior SITL processes + UDS sockets.
  2. Launch `sentai_whycon` world inside distrobox via `launch_hybrid_cf2.sh`.
  3. Start `gz_to_camera_bridge.py` (host) → feeds `/tmp/sentai_cam.sock`.
  4. Start `gt_recorder` (host) → writes `cf2_gt.jsonl`.
  5. Copy `mission_s182.py` into `build-sim/sentai_fs_root/` and
     pipe `import mission_s182; mission_s182.run()` into
     `build-sim/sim/sentai_sim`.
  6. Stop recorders.
  7. Run `verdict.py` → plots + CSV.

Outputs land in this folder:
  - `journal.txt`           — sentai_sim mission log
  - `summary.json`          — mission summary (phases, errors)
  - `cf2_gt.jsonl`          — Gazebo GT pose stream
  - `s182_paired.csv`       — paired (est, gt) samples
  - `s182_xyz_world.png`    — thesis figure

## Caveats

  - **Yaw is indeterminate for single-marker WhyCon** (W17 §6.2).
    For multi-marker constellation pose, see W19-T3.
  - **Cam-to-body convention** — `estimate_drone_world` in
    `verdict.py` assumes simple `drone_world = marker_world + tvec_cam`,
    which is correct for cf2 yaw=0 + cam straight down + the SIM
    body-frame transform (`body_xform = (-1, 0, 0, +1)` per Sim.md
    §10b).  A sign mismatch shows up as a constant flip in X or Y —
    visible in the plot.  Real fix: apply the documented R_cam_to_body
    rotation.
  - **WhyCon detection rate scales with R_px**.  At z=1.0 m the
    marker projects to `r_px ≈ fx · 0.0544 / 1.0 = 13 px` — just above
    the s181-measured minimum of 6 px.  At z=1.5 m we'd hit `r_px ≈ 8.7`
    which is borderline; the altitude sweep doesn't go that high.

## Cross-refs

  - `[[sentai-sim-air-gapped-from-truth]]` — anti-cheat hard rule.
  - `[[missions-run-in-sentai-only]]` — mission orchestration rule.
  - `[[experiments-in-own-folder-log-dead-ends]]` — preserve runs.
  - `[[gt-recorder-tool]]` — canonical GT recorder.
  - W17 §6.2 — yaw indeterminacy of single circular marker.
  - W19-T3 — multi-marker constellation pose (resolves yaw + ID).
  - Sim.md §10b — body-frame mapping.
  - s127 / s167 — comparable ArUco SIM evals (different world,
    same launch pattern).
