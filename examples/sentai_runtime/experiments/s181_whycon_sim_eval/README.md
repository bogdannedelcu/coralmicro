# s181 — WhyCon SIM pose-estimation eval

WBS: **OP-S10-W19-T4**.  First end-to-end exercise of the unified
`sentai.markers` namespace with the WhyCon backend on the SIM build.
Generates the thesis-grade X/Y/Z estimated-vs-ground-truth plots
the operator asked for ("vreau să ajungem să desenăm pe niște
grafice fine pe care să le folosim la teză: poziție estimată vs
poziție reală").

## What's measured

For a controlled sweep of known world poses `(X, Y, Z)` over a
pinhole camera (`fx = fy = 240`, `cx = 160`, `cy = 120`, frame
320×240), the eval driver:

  1. Forward-projects the marker centre + outer-ring radius:
     `cx_px = fx · X / Z + cx`, `cy_px = fy · Y / Z + cy`,
     `r_px = fx · (d/2) / Z` with `d = 0.08 m`.
  2. Synth-draws a Krajník-pattern marker at the predicted pixel
     coordinates (3 concentric zones — outer dark annulus 0.6R..R,
     white inner disc 0.2R..0.6R, dark centre dot ≤ 0.2R).
  3. Runs the WhyCon backend through the unified `sentai.markers`
     dispatcher: adaptive threshold → 8-conn flood-fill → W1+W2
     filter + axes → W3 concentric validation → closed-form PnP.
  4. Reads back `tvec_cam = (X_est, Y_est, Z_est)` from the
     marker struct via `sentai.markers.get_pose_tuple(0)`.

Sweeps:

  - **Z sweep** — `X = 0, Y = 0, Z ∈ {0.20, 0.30, …, 2.00} m`
  - **X sweep** — `Y = 0, Z = 0.50 m, X ∈ {-0.20, …, +0.20} m`
  - **Y sweep** — `X = 0, Z = 0.50 m, Y ∈ {-0.15, …, +0.15} m`

## Findings

### 1. Systematic Z bias from annulus geometry (BUG → fix)

The first iteration (uncorrected) revealed a constant ~14 %
under-estimation of Z across the entire altitude band — `Z_est ≈
0.857 · Z_gt`.  Root cause: the closed-form PnP formula

```
z = fx · diameter / (2 · axis_a)
```

assumes `axis_a` equals the projected outer radius `R`.  That holds
for a **solid disc** (the lite-bench synth), where the 2nd-order
moment along the major axis is `mu20 = R² / 4` so
`axis_a = 2·√(mu20) = R`.  For a **Krajník annulus** (outer R,
inner r₁ = 0.6 R), the moment integrates only over the annular mass:

```
mu20_annulus = (R² + r₁²) / 4 = R² · (1 + (r₁/R)²) / 4
axis_a       = R · √(1 + (r₁/R)²)
             = R · √(1.36)
             = R · 1.166
```

So the eigenvalue-derived `axis_a` over-estimates `R` by a factor
of 1.166, and the un-corrected PnP under-estimates Z by exactly
the inverse `1/1.166 = 0.857`.

**Fix shipped** (`whycon_pnp_inplace_` + `WHYCON_PNP_ANNULUS_FACTOR
= 1.16619f`): multiply the closed-form Z by the annulus factor.
After the fix:

  - Z bias collapses from −14 % to under ±3 % (≈ ±3 cm absolute
    in the 0.3–1.2 m band).
  - X and Y biases (which were derived from the same Z and
    inherited the same multiplicative error) also collapse.

See `s181_z_uncorrected_vs_corrected.png` for the side-by-side.

### 2. Operating envelope (320×240 frame, d = 8 cm marker)

Detection succeeds for `r_px ≥ 6` and the marker fully inside the
frame.  At `fx = 240`:

  - `Z_max` for `r_px = 6` is `fx · 0.04 / 6 = 1.6 m`
  - `Z_min` for the marker to fit in the frame (R < frame/2) is
    `fx · 0.04 / 120 ≈ 0.08 m`

The sweep gates out Z ≥ 1.3 m (r_px < 7 → `NO_DETECT`) and Z ≤
0.20 m (border-touch — marker bigger than the frame's central
region, W3 sample radii fall outside).  Useful range is therefore
**0.3 m – 1.2 m** at this marker size + intrinsics.

### 3. Headline plots

`s181_xyz_compare.png` — three panels showing `X_est`, `Y_est`,
`Z_est` plotted against their respective ground-truth values, all
hugging the y = x identity line within ~1 cm on X/Y and ~3 cm on Z
after the annulus correction.  Thesis-ready.

## Folder layout

  - `README.md`              — this file
  - `s181_eval.py`           — MicroPython-side driver (lives in
                                 `build-sim/sentai_fs_root/` for `import`).
  - `s181_uncorrected.csv`   — first pass, no annulus correction.
                                 Kept per [[experiments-in-own-folder-log-dead-ends]]
                                 — shows the engineering process.
  - `s181_corrected.csv`     — after `WHYCON_PNP_ANNULUS_FACTOR`
                                 shipped.
  - `plot_results.py`        — host-side matplotlib script.
  - `s181_xyz_compare.png`   — headline figure for the thesis.
  - `s181_z_uncorrected_vs_corrected.png` — side-by-side bias plot.

## How to run

```
# Build SIM
cmake -B build-sim -S . -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake \
       -DSENTAI_SIM=ON
cmake --build build-sim --target sentai_sim -j

# Drop the eval script into the SIM filesystem
cp examples/sentai_runtime/experiments/s181_whycon_sim_eval/s181_eval.py \
   build-sim/sentai_fs_root/

# Run + capture
(echo "import s181_eval" | timeout 10 build-sim/sim/sentai_sim 2>&1) \
    | grep "^S181:" \
    > examples/sentai_runtime/experiments/s181_whycon_sim_eval/s181_corrected.csv

# Plot
python3 examples/sentai_runtime/experiments/s181_whycon_sim_eval/plot_results.py
```

## Caveats / follow-ups

  - **Closed-form PnP only**.  We do NOT iteratively refine; the
    1.166 correction factor assumes the standard Krajník 0.6
    inner/outer ratio.  Markers with a different ratio need a
    different factor (or, better, generalise PnP to read the ratio
    from a calibration field).
  - **Single marker, single frame**.  No motion, no real camera
    noise, no Gazebo physics.  This eval validates the WhyCon
    closed-form math under ideal conditions.
  - **Yaw indeterminate** — see W17 §6.2; resolved by multi-marker
    constellation in W19-T3 (TODO).
  - **Real-world delta** — the next iter (s182, planned) drives
    cf2 SITL through a Gazebo scene with rendered Krajník markers
    on the floor, runs the same WhyCon backend on the streamed
    camera frames, and compares against `gt_recorder` GT.  That's
    where real-camera noise + motion blur + tilt errors enter the
    picture.

## Cross-refs

  - WP: `ideas/objects_plan/OP-S10-W19_markers_unified.md`
  - W17 §6.2 (yaw indeterminacy of single circular marker)
  - W17 §4 / §4.1-§4.2 (apples-to-apples HW bench)
  - `[[experiments-in-own-folder-log-dead-ends]]` — s181_uncorrected.csv
    stays on disk as engineering-process evidence.
  - `[[no-heavy-data-through-mp]]` — `get_pose_tuple` returns a small
    Python tuple, not a heavy buffer.
