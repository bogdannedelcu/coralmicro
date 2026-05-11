# s092 — Camera ↔ body-frame axis calibration

Controlled-motion identification of the cf2 SITL downward-camera
orientation in body frame.  No-wind, MotionCommander-driven X/Y
translations; per-frame L0 phase-corr output recorded; bias-corrected
differential gives the camera-to-body rotation matrix.

## Why

Passive observation under wind contaminates the flow signal with DC
bias (drone tilts to fight wind → camera tilts → optical flow integrates
that as translation).  Pulling the rotation matrix out of biased data
gives garbage.  This experiment isolates motion-induced flow from bias
by symmetric +X / -X / +Y / -Y motion + middle-50%-of-phase sampling.

## Run

Wind is auto-disabled by `/tmp/run_axis_calib.sh` (edits the SDF, restored
on exit).

```bash
bash /tmp/run_axis_calib.sh
# or manually after disabling wind:
python3 axis_calib.py
```

Artifacts in `$SENTAI_DUMP_FRAMES_DIR/`:
- `axis_calib_raw.csv` — per-flow record (seq, t, L0_dx/dy/conf, best fields)
- `axis_calib_summary.csv` — per-phase mean/std/conf
- `calib.log` — stdout, including the derived BODY_XFORM

## Result (2026-05-11, build trail)

Bias-corrected differential per 1.2 m of body motion:

|  | mean L0_dx | mean L0_dy |
|---|---:|---:|
| +body_X | -114 | **-732** |
| -body_X | +234 | **+676** |
| +body_Y | **-957** | +131 |
| -body_Y | **+653** | -184 |

Image-Y axis tracks body-X (sign inverted); image-X tracks body-Y (sign
inverted).  Confirms `BODY_XFORM = (0, -1, -1, 0)` in s091 was correct
all along.  See `Sim.md §10l` for the full write-up.
