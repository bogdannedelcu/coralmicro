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

### 2. Marker layout — H-pattern (Kim 2013)

```
NW (-0.16, +0.20) ──────── NE (+0.16, +0.20)
        ║                          ║
        ║                          ║
        ║                          ║
 W (-0.16,  0.00) ─────────  E (+0.16, 0.00)   ← horizontal bar
        ║                          ║
        ║                          ║
 SW (-0.16, -0.14) ────────  SE (+0.16, -0.14)
```

6 markers in the shape of the letter **H**, asymmetric on Y (top arm
20 cm, bottom arm 14 cm), symmetric on X.  Operator request 2026-05-20:
"obiectiv: takeoff/landing markers area pe care sa verificam flow,
calibrare, stabilizare pe X, Y si Z".

SOTA basis: Kim, Yang & Kim 2013 (IROS) "A new approach to drone-
based fast localization for landing using only landmark pattern
recognition" — 6-marker H pattern minimises pose ambiguity for downward
VTOL landing.  Equivalent geometry to the ICAO Annex 14 helipad **H**.

Iter-3 (5-marker rotated cross) was abandoned because the 5 markers
were collinear on the X and Y cardinal axes — drift on one axis was
constrained only by 2 markers, the other axis by 3 (uneven lever
arms).  The H pattern has 4 outer corners + 2 mid-bar markers, all 6
contributing roughly equal lever to X and Y — well-conditioned Kabsch
fit (s182 iter-3 numbers reflect the BAD layout; iter-5+ use H).

Marker physical outer-ring diameter on the 0.12 m box face:
`(232/256) × 0.06 m × 2 = 0.1088 m` (PNG margin offsets reduce the
on-face circle radius).  Pass `sentai.markers.set_marker_size(0.1088)`
to match.

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

## Status (2026-05-20 EOD)

Scaffolding COMPLETE, end-to-end run NOT yet validated.  Known
issues from the first launch attempt:

  - **launch_hybrid_cf2.sh tears down when its foreground `gz sim -g`
    closes.**  Backgrounding the script via `&` followed by `disown`
    leaves the GUI in the orphaned script's foreground; when the
    parent shell exits, the GUI dies, the trap fires, and the whole
    stack goes down.  Mitigation: run `bash run.sh` from a terminal
    that stays open, OR add `wait` at the end of launch_hybrid +
    drop the EXIT trap.
  - **150 s deadline in run.sh** may be too short on first-build
    machines (cf2 SITL pre-task-loop init can take that long).
    Bump to 240 s if first launch times out.
  - **World name inside SDF** was originally `sentai_crazysim`
    (carried from the copy source); now corrected to `sentai_whycon`
    so gz topics route correctly to `/world/sentai_whycon/dynamic_pose/info`.
    gt_recorder env `GT_RECORDER_WORLD=sentai_whycon` is set in
    run.sh.
  - **White-marker rendering** — operator reported on first attempt
    that markers appeared as white squares (no Krajník circles
    visible).  Three working hypotheses:
      1. PBR texture cache lag in Gazebo — relaunch from scratch
         after a full `pkill gz` should fix.
      2. The texture file wasn't yet in CrazySim's
         `materials/textures/` when the launch ran — now it IS
         (verified: 3616 bytes at the canonical path).
      3. Hidden material mismatch between PBR `<diffuse>1 1 1 1</diffuse>`
         + `albedo_map` — ArUco markers use the same config and
         render fine, so probably not it.
    Next operator interaction confirms which.

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
