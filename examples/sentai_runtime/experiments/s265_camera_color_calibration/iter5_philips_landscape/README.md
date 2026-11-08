# s265 iter5 — Philips 234E5 in landscape, board above the monitor (2026-11-08)

**Hypothesis**: the Kamvas-tuned production sets do not fit this display (one monitor, landscape, both
cameras looking down at it); a centre-mode re-tune (expo, tone_centre, awb on top of the kamvas sets)
gives a usable set for the monitor rig that Unreal (Bologna / Paris) is shown on.
**Change vs iter4**: new display + geometry; stages run per camera with `--centre --avg 2`.

## Run

```bash
# board reset + camera.init() (production sets applied) + locate.py  (as in camera_modules/measure_position.sh)
python3 tune_colors.py --out iter5_philips_landscape --cams 0 --stages expo,tone_centre,awb --centre --avg 2
python3 tune_colors.py --out iter5_philips_landscape --cams 1 --stages expo,tone_centre,awb --centre --avg 2
cd ../../camera_modules && python3 merge_set.py --tuned ../experiments/s265_camera_color_calibration/iter5_philips_landscape \
    --name philipsL --date 2026-11-08 --reference "Philips 234E5 landscape ..." --mode centre
python3 make_regs.py --upload --keep-exposure && bash measure_position.sh philipsL_frozen 1.0
```

## Result

| | cam0 ΔE76 centre (greys) | cam1 ΔE76 centre (greys) | note |
|---|---|---|---|
| before (kamvas sets, auto exposure) | 11.2 | 15.1 | neutral 8 clipped (G 253–255 at the auto AGC 3.3–3.5x) |
| after tune (frozen tuning exposure) | **8.3 (3.8)** | **13.5 (3.9)** | expo: AGC 26 (1.6x) / 41 (2.6x); tone_centre rms 10.8 → 3.8 / 6.4 → 3.8; AWB R/B 1539/1307 / 1280/1215 |
| production upload, default (AEC freeze dropped), after reset | 24.0 | 15.8 | **auto exposure re-clips the bright patches on this monitor** (orange-yellow 255/245/112) |
| production upload `--keep-exposure`, after reset | **7.0** | **12.2** | grid mode ΔE00 7.6 / 10.1 (`camera_modules/out/compare_philipsL_frozen.png`) |

Decisions: for the monitor rig the production files keep the tuning exposure (`make_regs.py --upload
--keep-exposure`); the AEC-target alternative (lower 0x3A0F/0x3A10 so auto exposure lands near the
tuned level) is a W24-T23 item.  Worst patches: cam0 yellow (ΔE00 19), cam1 red / yellow-green /
magenta (module chroma deficit, as before).

Bug fixed on the way: `stage_tone_centre` reused cached ramp PNGs from the Kamvas session, drawn at
the old centre, so the grey patch appeared at the frame edge and the visibility guard aborted; the
ramp PNGs are now regenerated every run.

Files: `locate.json`, `chain_cam0.out`, `chain_cam1.out`, `tuned_regs_cam{0,1}.json`,
`tune_after_patches_cam{0,1}.json`, `measure_centre_frozen_cam{0,1}.json`, `tune_cam*_*.json`.
