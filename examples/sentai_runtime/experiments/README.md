# Experiments — camera-speed story (E15-E18)

Downloaded 2026-04-20 from the SentAI board's `/diags/` folder via
`curl http://10.0.0.1/api/raw/...`.  Every session is self-contained:
- `001_*.csv` … `00N_*.csv`  — per-experiment sample data
- `001_*.txt` … `00N_*.txt`  — human-readable column docs + parameters
- `manifest.csv`             — one row per saved experiment
- `summary.txt`              — session uptime, heap delta, experiment count
- `scene_camX_{before,after}_WxH.jpg` — scene JPEGs from each camera
  captured at the start / end of the session (for offline diff)

Total size ~3.3 MB across 25 sessions.  Each session's `manifest.csv` is
the authoritative record of what ran inside it.

## Narrative index — which session matters and why

The full story is in [../paper/cam_switch.md](../paper/cam_switch.md).
Below is the short form, session → chapter.

### E15 parallel pipeline development (fixed-camera baseline, 15 fps)

| Session | Reps | Note |
|---|---:|---|
| `s016_e15_512` – `s020_e15_512` | 1 each | initial single-shot calibration of the 512×512 1-class model through the parallel pipeline |
| `s024_e15_x20`, `s026_e15_x20` | 20×20 | 20-run sustained test showing the 13.4 → 15.4 FPS step (the eDMA memcpy optimisation documented in [../paper/memcpy.md](../paper/memcpy.md)) |
| `s027_e15_512` – `s030_e15_512` | 1 each | post-optimisation single-shots confirming 15 FPS is now reproducible |
| **`s032_e15_x20`** | 20×20 | **last E15-only baseline before E16 was written — cleanest reference for "sustained fixed-camera FPS"** |

### E16 first camera-switch measurements (cam0 ↔ cam1 every frame)

| Session | Reps | Note |
|---|---:|---|
| `s033_e16_camswitch_512` | 1 | first E16 probe, single alternating run |
| **`s034_e15_vs_e16_x40`** | 40 | **pre-Fix A baseline: E15 @ 15 fps + E16 @ 15 fps in the same session; shows the 211 ms alternating frame and the +64 ms cam1 asymmetry** |
| `s035_e15_vs_e16_x40` | 40 | post-Fix A (atomic snapshot) — identical numbers, confirming the race was not the root cause of the asymmetry |

### E17 per-switch JPEG captures (visual inspection)

| Session | Reps | Note |
|---|---:|---|
| `s036_e17_drain_ab` – `s037_e17_drain_ab` | 0 / 0 | empty shell sessions (REPL driver bug discovered, frames never captured) |
| **`s038_e17_drain_ab`** | 16×2 | **E17 at drain=1 vs drain=2 on pre-flip-on-EOF firmware — this is the session that exposed the mid-buffer seam (half-frame from cam0, half from cam1)** |
| `s039_e17_drain_ab` | 16 | re-run with `ratio(0,0)` explicit, drain=1 only |
| `s040_e17_drain_ab` | 0 | empty (REPL dead) |
| **`s041_e17_eof_check`** | 8×2 | **E17 at drain=1 vs drain=2 on post-flip-on-EOF firmware — user-confirmed visually identical, no seam** |

### E18 head-to-tail three-sweep benchmark (fixed A, fixed B, alternating)

| Session | Note |
|---|---|
| `s042_e16_eof_30fps_x40` | 40-rep E16 at 30 fps + flip-on-EOF (Fix B); 6.87 FPS alternating |
| `s043_e18_headtail_drain2` | first E18 proper: fixed cam0 16.0 fps, alternating 6.86 fps, per-switch overhead 82.5 ms |
| `s044_e18_headtail_drain2` | E18 reproducibility confirmation — identical to `s043` within 1 ms |
| **`s045_e18_post_refactor`** | **E18 after the NASA-JPL review refactor (A1-A7, B1-B4): 62 ms fixed-cam, 145.8 ms alternating, 6.86 fps — zero regression.  `scene_cam1_after_512x512.jpg` is 0 bytes (REPL stdout-dead state prevented the after-snapshot; the before snapshot is intact).** |

## Reading the CSVs

All E16/E17/E18 sweeps share the column layout:
```
run_index, cam_id, select_ms, to_tensor_ms, invoke_ms, detect_ms,
num_detections, total_frame_ms
```
The `.txt` beside each CSV explains which sweep it is and what the
parameters were.  The E15 CSV has a different schema documented in its
own `.txt` (columns about parallel-pipeline `frame_interval_ms`,
`prep_stall_ms`, `infer_stall_ms`).

## Reproducing

From the sentai_runtime root on the host:
```bash
# Upload the latest diag/ package to the board
python3 diag/_host_upload_repl.py --file e_pipeline.py --file _util.py

# Run the head-to-tail benchmark (takes ~10 s on-device)
python3 diag/drivers/_e18_post_refactor.py

# Or drive from a shell one-liner via repl_run.py (for short commands)
python3 repl_run.py --line "import diag" \
                    --line "diag.begin('foo')" \
                    --line "diag.e18_camera_switch_headtail(repetitions=40)" \
                    --line "diag.end()"
```

Data is persisted under `/diags/sNNN_<name>/` on the device and survives
reboot.  Re-downloading is a one-command script documented in
[../agent/agent.md](../agent/agent.md) §3.
