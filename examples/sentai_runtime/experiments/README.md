# Experiments — board diagnostic sessions

Downloaded 2026-04-20 from the SentAI board's `/diags/` folder via
`curl http://10.0.0.1/api/raw/...`.  All 45 sessions on the device
are mirrored here.  The narrative arc is the camera-speed story
(E15-E18) — the earlier sessions are pre-camera-switch groundwork
(memory, CPU, filesystem, TPU, individual-subsystem probes).
Every session is self-contained:
- `001_*.csv` … `00N_*.csv`  — per-experiment sample data
- `001_*.txt` … `00N_*.txt`  — human-readable column docs + parameters
- `manifest.csv`             — one row per saved experiment
- `summary.txt`              — session uptime, heap delta, experiment count
- `scene_camX_{before,after}_WxH.jpg` — scene JPEGs from each camera
  captured at the start / end of the session (for offline diff)

Total size ~4.6 MB across 45 sessions.  Each session's `manifest.csv` is
the authoritative record of what ran inside it.

**Methodology** — how these sessions are produced, downloaded, and
statistically summarised is documented in
[`methodology.md`](methodology.md).  That file describes the warm-up
protocol, the warm-up-sample drop convention, noise budget, scene
snapshot discipline, the REPL-chunked upload path, and the
reproducibility criteria.  Read it before interpreting any of the
appendix numbers.

## Pre-camera-switch sessions (E1-E14 — groundwork)

These are the early single-subsystem probes and the build-up to the
parallel vision pipeline.  Included for completeness; they are the
baseline work that the camera-switch story stands on.

| Session | What it probed |
|---|---|
| [`s001_memcheck`](s001_memcheck/) | E10 — idle heap + `pvPortMalloc`/`vPortFree` characterisation |
| [`s002_cam_speed`](s002_cam_speed/) | E3 — camera → RGB tensor at 320×320, raw frame-grab timing |
| [`s003_jpeg_test`](s003_jpeg_test/) | E4 — JPEG encode cost at various qualities |
| [`s004_t1_simple`](s004_t1_simple/) | first-ever scripted TPU probe via `diag.begin`/`end` |
| [`s005_basics`](s005_basics/) | E10 memory + E11 CPU-usage + E7 FS write 4096 B (the first integrated run) |
| [`s006_cam_basic`](s006_cam_basic/) | E5 — camera switch characterisation (early, pre-EOF-ISR) |
| [`s007_tpu_basic`](s007_tpu_basic/) | E1 — `tpu.invoke()` + `tpu.load()` baseline timing |
| [`s008_live_loop`](s008_live_loop/) – [`s011_loop_fix`](s011_loop_fix/) | E12 — live detection loop iteration, bug-fix cycles on the loop exit path |
| [`s012_final`](s012_final/) | consolidated "run everything once" smoke test |
| [`s013_e13`](s013_e13/) – [`s014_e13`](s014_e13/) | E13 first runs — sequential full pipeline at 320×320, per-stage timing introduced |
| [`s015_e14`](s015_e14/) | E14 first run — parallel pipeline (`sentai.pipeline.start/get/stop`) at 320×320 |
| [`s021_e14_x20`](s021_e14_x20/) – [`s023_e14_x20`](s023_e14_x20/) | E14 x20 repetitions — sustained FPS validation for the parallel pipeline, establishing the 13–15 FPS baseline that the camera-switch work builds on |
| [`s025_e14_x20`](s025_e14_x20/), [`s031_e14_x20`](s031_e14_x20/) | further E14 x20 replays at different commit points, confirming reproducibility |

These sessions do NOT use the 512×512 1-class model — they run at
320×320 on the 80-class `yolo26n.edgetpu_1.tflite`.  That model was
later swapped for the faster 1-class 512×512 model, and the
sessions starting at `s016_e15_512` pick up the new model.

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

---


---

# Appendix — per-experiment tables and descriptive text

The tables below are generated from every CSV in this folder and are
structured so sections can be pasted directly into the paper's
appendices.  Each section summarises its experiment class (what it
measures, which model, what parameters) and then shows the
per-session aggregate statistics with a link back to the raw CSV.

## Appendix A — Pre-E15 groundwork sessions (subsystem probes)

Early probes on individual subsystems before the parallel vision
pipeline was established.  All sessions that involve a model use the
80-class `yolo26n.edgetpu_1.tflite`.  Each table row is a single
experiment inside that session; the `Summary` column is verbatim from
the `manifest.csv` on the device.

### `s001_memcheck`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e10_memory` | idle rtos_free=16678652 | [`001_e10_mem_idle.csv`](s001_memcheck/001_e10_mem_idle.csv) |

### `s002_cam_speed`

*Empty session (no experiments recorded).*

### `s003_jpeg_test`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e4_jpeg` | cam0 q=75 jpeg=156.8 save=244.8 ms frames=5 | [`001_e4_jpeg_cam0_q75.csv`](s003_jpeg_test/001_e4_jpeg_cam0_q75.csv) |

### `s004_t1_simple`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e10_memory` | after_begin rtos_free=16686648 | [`001_e10_mem_after_begin.csv`](s004_t1_simple/001_e10_mem_after_begin.csv) |

### `s005_basics`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e10_memory` | idle rtos_free=16686656 | [`001_e10_mem_idle.csv`](s005_basics/001_e10_mem_idle.csv) |
| 3 | `e7_fs_write` | 4096B 55.7 KB/s mean=71.8 ms | [`003_e7_fs_write_4096b.csv`](s005_basics/003_e7_fs_write_4096b.csv) |
| 4 | `e6_fs_read` | 0.2 KB/s mean=157.4 ms | [`004_e6_fs_read.csv`](s005_basics/004_e6_fs_read.csv) |
| 5 | `e8_imu` | read=0.0 deg=0.1 rad=0.1 ms | [`005_e8_imu.csv`](s005_basics/005_e8_imu.csv) |

### `s006_cam_basic`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e4_jpeg` | cam0 q=75 jpeg=106.6 save=360.2 ms frames=5 | [`001_e4_jpeg_cam0_q75.csv`](s006_cam_basic/001_e4_jpeg_cam0_q75.csv) |
| 2 | `e5_camera_switch` | 0->1 switch=0.0 roundtrip=348.7 ms | [`002_e5_switch_0to1.csv`](s006_cam_basic/002_e5_switch_0to1.csv) |

### `s007_tpu_basic`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e2_tpu_load` | mean=1480.0 ms | [`001_e2_tpu_load.csv`](s007_tpu_basic/001_e2_tpu_load.csv) |
| 2 | `e1_tpu_invoke` | mean=122.7 p95=130.0 ms | [`002_e1_tpu_invoke.csv`](s007_tpu_basic/002_e1_tpu_invoke.csv) |
| 3 | `e1_tpu_invoke` | mean=123.8 p95=131.6 ms | [`003_e1_tpu_invoke.csv`](s007_tpu_basic/003_e1_tpu_invoke.csv) |

### `s008_live_loop`

*Empty session (no experiments recorded).*

### `s009_loop2`

*Empty session (no experiments recorded).*

### `s010_loop3`

*Empty session (no experiments recorded).*

### `s011_loop_fix`

*Empty session (no experiments recorded).*

### `s012_final`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e10_memory` | startup rtos_free=16686656 | [`001_e10_mem_startup.csv`](s012_final/001_e10_mem_startup.csv) |
| 2 | `e12_live_loop` | cam0 320x320 loop=167.1 ms fps=6.0 | [`002_e12_loop_cam0_320x320.csv`](s012_final/002_e12_loop_cam0_320x320.csv) |
| 3 | `e10_memory` | after_loop rtos_free=12093612 | [`003_e10_mem_after_loop.csv`](s012_final/003_e10_mem_after_loop.csv) |

### `s013_e13`

*Empty session (no experiments recorded).*

### `s014_e13`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e13_pipeline_full` | cam0 320x320 fps=5.0 frame=5 tensor=54 inv=128 out=9 det=6 ms  dets=20 | [`001_e13_pipeline_cam0_320x320.csv`](s014_e13/001_e13_pipeline_cam0_320x320.csv) |

### `s015_e14`

*Empty session (no experiments recorded).*

### `s021_e14_x20`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`001_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) |
| 2 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`002_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) |
| 3 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`003_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) |
| 4 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`004_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) |
| 5 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=1 | [`005_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) |
| 6 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`006_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) |
| 7 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=2 | [`007_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) |
| 8 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`008_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) |

### `s022_e14_x20`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`001_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) |
| 2 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`002_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) |
| 3 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`003_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) |
| 4 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`004_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) |
| 5 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`005_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) |
| 6 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`006_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) |
| 7 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`007_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) |
| 8 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`008_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) |
| 9 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`009_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) |
| 10 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`010_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) |
| 11 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`011_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) |
| 12 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`012_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) |
| 13 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`013_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) |
| 14 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`014_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) |
| 15 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`015_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) |
| 16 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`016_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) |
| 17 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=1 | [`017_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) |
| 18 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=3 | [`018_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) |
| 19 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`019_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) |
| 20 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`020_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) |

### `s023_e14_x20`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`001_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) |
| 2 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`002_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) |
| 3 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.3 fw_fps=6.1 dropped=0 dets=0 | [`003_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) |
| 4 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`004_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) |
| 5 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`005_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) |
| 6 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`006_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) |
| 7 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`007_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) |
| 8 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`008_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) |
| 9 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`009_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) |
| 10 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`010_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) |
| 11 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`011_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) |
| 12 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`012_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) |
| 13 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`013_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) |
| 14 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`014_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) |
| 15 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`015_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) |
| 16 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`016_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) |
| 17 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`017_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) |
| 18 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`018_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) |
| 19 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.0 dropped=0 dets=0 | [`019_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) |
| 20 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`020_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) |

### `s025_e14_x20`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`001_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) |
| 2 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`002_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) |
| 3 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`003_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) |
| 4 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`004_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) |
| 5 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=2 | [`005_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) |
| 6 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=2 | [`006_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) |
| 7 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`007_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) |
| 8 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`008_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) |
| 9 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.0 dropped=0 dets=0 | [`009_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) |
| 10 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`010_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) |
| 11 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`011_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) |
| 12 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`012_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) |
| 13 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`013_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) |
| 14 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`014_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) |
| 15 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`015_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) |
| 16 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`016_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) |
| 17 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`017_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) |
| 18 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.0 dropped=0 dets=0 | [`018_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) |
| 19 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`019_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) |
| 20 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`020_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) |

### `s031_e14_x20`

| # | Experiment | Summary | CSV |
|---|---|---|---|
| 1 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.1 dropped=0 dets=0 | [`001_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) |
| 2 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`002_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) |
| 3 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`003_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) |
| 4 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`004_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) |
| 5 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`005_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) |
| 6 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`006_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) |
| 7 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`007_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) |
| 8 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=1 | [`008_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) |
| 9 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`009_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) |
| 10 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`010_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) |
| 11 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.3 fw_fps=6.2 dropped=0 dets=0 | [`011_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) |
| 12 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.3 fw_fps=6.2 dropped=0 dets=0 | [`012_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) |
| 13 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`013_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) |
| 14 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.3 fw_fps=6.2 dropped=0 dets=0 | [`014_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) |
| 15 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`015_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) |
| 16 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`016_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) |
| 17 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`017_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) |
| 18 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`018_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) |
| 19 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`019_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) |
| 20 | `e14_pipeline_parallel` | cam0 320x320 fps_wall=6.4 fw_fps=6.2 dropped=0 dets=0 | [`020_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) |


## Appendix B — E14 parallel-pipeline at 320×320, yolo26n 80-class

E14 runs `sentai.pipeline.start/get/stop` with the firmware
`PrepTask + InferTask` double-buffer.  Measurement is the wall-clock
interval between successive `pipeline.get()` returns.  All sessions
below use `yolo26n.edgetpu_1.tflite` at 320×320, 20 repetitions each
inside a run, and multiple runs per session.  The 80-class model is
slower than the 1-class 512×512 model that later replaces it in E15 —
these runs consistently land around **6.4 FPS** because the 80-class
output head does not fit entirely on the EdgeTPU and falls back to
CPU execution for several layers.  That was the motivation for the
model swap in E15.

### `s021_e14_x20` — 8 runs

| Run file | Mean frame_interval (ms) | FPS |
|---|---:|---:|
| [`001_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) | 157.00 | 6.37 |
| [`002_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`003_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) | 156.00 | 6.41 |
| [`004_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) | 156.61 | 6.39 |
| [`005_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) | 157.06 | 6.37 |
| [`006_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`007_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) | 156.22 | 6.40 |
| [`008_e14_pipeline_par_cam0_320x320.csv`](s021_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |

### `s022_e14_x20` — 20 runs

| Run file | Mean frame_interval (ms) | FPS |
|---|---:|---:|
| [`001_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) | 157.17 | 6.36 |
| [`002_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) | 156.22 | 6.40 |
| [`003_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) | 156.67 | 6.38 |
| [`004_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) | 156.94 | 6.37 |
| [`005_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) | 157.17 | 6.36 |
| [`006_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`007_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) | 157.00 | 6.37 |
| [`008_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`009_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) | 156.94 | 6.37 |
| [`010_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |
| [`011_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) | 157.28 | 6.36 |
| [`012_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) | 157.17 | 6.36 |
| [`013_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) | 156.33 | 6.40 |
| [`014_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) | 157.17 | 6.36 |
| [`015_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`016_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) | 156.83 | 6.38 |
| [`017_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) | 157.28 | 6.36 |
| [`018_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) | 156.50 | 6.39 |
| [`019_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) | 156.61 | 6.39 |
| [`020_e14_pipeline_par_cam0_320x320.csv`](s022_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) | 156.44 | 6.39 |

### `s023_e14_x20` — 20 runs

| Run file | Mean frame_interval (ms) | FPS |
|---|---:|---:|
| [`001_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) | 157.28 | 6.36 |
| [`002_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`003_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) | 157.61 | 6.34 |
| [`004_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |
| [`005_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) | 157.06 | 6.37 |
| [`006_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) | 156.44 | 6.39 |
| [`007_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) | 156.94 | 6.37 |
| [`008_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) | 156.22 | 6.40 |
| [`009_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) | 156.83 | 6.38 |
| [`010_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) | 157.06 | 6.37 |
| [`011_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |
| [`012_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) | 157.00 | 6.37 |
| [`013_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) | 156.33 | 6.40 |
| [`014_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) | 157.06 | 6.37 |
| [`015_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`016_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`017_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |
| [`018_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) | 156.67 | 6.38 |
| [`019_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) | 156.61 | 6.39 |
| [`020_e14_pipeline_par_cam0_320x320.csv`](s023_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) | 156.61 | 6.39 |

### `s025_e14_x20` — 20 runs

| Run file | Mean frame_interval (ms) | FPS |
|---|---:|---:|
| [`001_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) | 157.11 | 6.36 |
| [`002_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) | 155.89 | 6.41 |
| [`003_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`004_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) | 156.50 | 6.39 |
| [`005_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`006_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) | 157.00 | 6.37 |
| [`007_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) | 156.78 | 6.38 |
| [`008_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) | 156.67 | 6.38 |
| [`009_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) | 155.89 | 6.41 |
| [`010_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) | 156.39 | 6.39 |
| [`011_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`012_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) | 156.78 | 6.38 |
| [`013_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) | 156.33 | 6.40 |
| [`014_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) | 156.56 | 6.39 |
| [`015_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) | 156.67 | 6.38 |
| [`016_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) | 156.06 | 6.41 |
| [`017_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) | 156.61 | 6.39 |
| [`018_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) | 156.72 | 6.38 |
| [`019_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) | 156.11 | 6.41 |
| [`020_e14_pipeline_par_cam0_320x320.csv`](s025_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) | 156.89 | 6.37 |

### `s031_e14_x20` — 20 runs

| Run file | Mean frame_interval (ms) | FPS |
|---|---:|---:|
| [`001_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/001_e14_pipeline_par_cam0_320x320.csv) | 157.39 | 6.35 |
| [`002_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/002_e14_pipeline_par_cam0_320x320.csv) | 156.56 | 6.39 |
| [`003_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/003_e14_pipeline_par_cam0_320x320.csv) | 156.56 | 6.39 |
| [`004_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/004_e14_pipeline_par_cam0_320x320.csv) | 156.50 | 6.39 |
| [`005_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/005_e14_pipeline_par_cam0_320x320.csv) | 156.50 | 6.39 |
| [`006_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/006_e14_pipeline_par_cam0_320x320.csv) | 157.11 | 6.36 |
| [`007_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/007_e14_pipeline_par_cam0_320x320.csv) | 156.28 | 6.40 |
| [`008_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/008_e14_pipeline_par_cam0_320x320.csv) | 156.67 | 6.38 |
| [`009_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/009_e14_pipeline_par_cam0_320x320.csv) | 156.83 | 6.38 |
| [`010_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/010_e14_pipeline_par_cam0_320x320.csv) | 157.00 | 6.37 |
| [`011_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/011_e14_pipeline_par_cam0_320x320.csv) | 157.83 | 6.34 |
| [`012_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/012_e14_pipeline_par_cam0_320x320.csv) | 157.28 | 6.36 |
| [`013_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/013_e14_pipeline_par_cam0_320x320.csv) | 157.39 | 6.35 |
| [`014_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/014_e14_pipeline_par_cam0_320x320.csv) | 157.78 | 6.34 |
| [`015_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/015_e14_pipeline_par_cam0_320x320.csv) | 157.11 | 6.36 |
| [`016_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/016_e14_pipeline_par_cam0_320x320.csv) | 157.39 | 6.35 |
| [`017_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/017_e14_pipeline_par_cam0_320x320.csv) | 156.50 | 6.39 |
| [`018_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/018_e14_pipeline_par_cam0_320x320.csv) | 156.83 | 6.38 |
| [`019_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/019_e14_pipeline_par_cam0_320x320.csv) | 157.28 | 6.36 |
| [`020_e14_pipeline_par_cam0_320x320.csv`](s031_e14_x20/020_e14_pipeline_par_cam0_320x320.csv) | 157.22 | 6.36 |


## Appendix C — E15 parallel pipeline at 512×512, 1-class model

E15 replays E14 on the faster single-class model
`yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`.
This model is fully edgetpu-compiled — no CPU fallback — so the
bottleneck shifts from inference to the SDRAM→tensor memcpy.

**Two eras in the data below:**

| Era | Sessions | Typical FPS | Why |
|---|---|---:|---|
| Pre-eDMA (CPU memcpy 24 ms) | `s016`-`s020` (single-shot), `s024`, `s026` (x20) | ~6.4 | Invoke is fast but the 786 KB `memcpy(tensor, staging)` on CPU through the D-cache burns ~24 ms per frame |
| Post-eDMA (32-byte AXI bursts) | `s027`-`s030` (single-shot), `s032` (x20) | ~14–15 | 786 KB copy drops to ~14.6 ms; pipeline hits sensor rate ([paper/memcpy.md](../paper/memcpy.md) § "Optimised") |


### `s019_e15_512`

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 152.00 ± 0.00 | 152 / 152 | 1 |
| prep_stall_ms | 0.00 ± 0.00 | 0 / 0 | 1 |
| infer_stall_ms | 35.00 ± 0.00 | 35 / 35 | 1 |
| **FPS** (1000/mean_interval) | **6.58** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s019_e15_512/001_e15_pipeline_par_cam0_512x512.csv)

### `s020_e15_512`

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 150.00 ± 0.00 | 150 / 150 | 1 |
| prep_stall_ms | 0.00 ± 0.00 | 0 / 0 | 1 |
| infer_stall_ms | 35.00 ± 0.00 | 35 / 35 | 1 |
| **FPS** (1000/mean_interval) | **6.67** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s020_e15_512/001_e15_pipeline_par_cam0_512x512.csv)

### `s024_e15_x20`

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 157.06 ± 2.86 | 153 / 165 | 18 |
| prep_stall_ms | 0.50 ± 0.51 | 0 / 1 | 18 |
| infer_stall_ms | 38.83 ± 1.38 | 37 / 41 | 18 |
| **FPS** (1000/mean_interval) | **6.37** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s024_e15_x20/001_e15_pipeline_par_cam0_512x512.csv)

### `s026_e15_x20`

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 156.33 ± 2.22 | 152 / 159 | 18 |
| prep_stall_ms | 0.17 ± 0.38 | 0 / 1 | 18 |
| infer_stall_ms | 38.83 ± 1.38 | 37 / 41 | 18 |
| **FPS** (1000/mean_interval) | **6.40** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s026_e15_x20/001_e15_pipeline_par_cam0_512x512.csv)

### `s032_e15_x20`

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 64.33 ± 2.14 | 62 / 70 | 18 |
| prep_stall_ms | 0.11 ± 0.32 | 0 / 1 | 18 |
| infer_stall_ms | 41.61 ± 1.29 | 41 / 45 | 18 |
| **FPS** (1000/mean_interval) | **15.54** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s032_e15_x20/001_e15_pipeline_par_cam0_512x512.csv)


## Appendix D — E15 + E16 head-to-head sessions (pre/post Fix A)

Both E15 (fixed camera) and E16 (alternating cam0↔cam1) run in the
same session so the numbers are measured under identical scene,
thermal state, and firmware.  `s034` is pre-Fix A, `s035` is post.
The ~65 ms directional asymmetry in cam1 − cam0 was discovered here.

### `s034_e15_vs_e16_x40`

**E15 — fixed cam0 parallel pipeline**

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 64.47 ± 1.78 | 61 / 68 | 38 |
| prep_stall_ms | 0.00 ± 0.00 | 0 / 0 | 38 |
| infer_stall_ms | 41.53 ± 1.22 | 40 / 45 | 38 |
| **FPS** | **15.51** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s034_e15_vs_e16_x40/001_e15_pipeline_par_cam0_512x512.csv)

**E16 — alternating cam0↔cam1 sequential**

| Stage | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| select_ms | 0.00 ± 0.00 | 0 / 0 | 39 |
| to_tensor_ms | 182.85 ± 33.07 | 148 / 216 | 39 |
| invoke_ms | 28.15 ± 2.17 | 26 / 36 | 39 |
| detect_ms | 0.38 ± 0.49 | 0 / 1 | 39 |
| total_frame_ms | 211.44 ± 32.55 | 176 / 250 | 39 |
| **FPS** | **4.73** | — | — |

**Directional split:**

| Direction | Mean total ms | n |
|---|---:|---:|
| → cam0 | 178.53 | 19 |
| → cam1 | 242.70 | 20 |
| **asymmetry (cam1 − cam0)** | **+64.17 ms** | — |

Raw: [`002_e16_camswitch_cam0_cam1.csv`](s034_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv)

### `s035_e15_vs_e16_x40`

**E15 — fixed cam0 parallel pipeline**

| Metric | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| frame_interval_ms | 64.66 ± 2.27 | 62 / 72 | 38 |
| prep_stall_ms | 0.05 ± 0.23 | 0 / 1 | 38 |
| infer_stall_ms | 41.66 ± 1.12 | 40 / 44 | 38 |
| **FPS** | **15.47** | — | — |

Raw: [`001_e15_pipeline_par_cam0_512x512.csv`](s035_e15_vs_e16_x40/001_e15_pipeline_par_cam0_512x512.csv)

**E16 — alternating cam0↔cam1 sequential**

| Stage | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| select_ms | 0.00 ± 0.00 | 0 / 0 | 39 |
| to_tensor_ms | 183.21 ± 33.53 | 148 / 217 | 39 |
| invoke_ms | 28.97 ± 2.31 | 26 / 35 | 39 |
| detect_ms | 0.21 ± 0.41 | 0 / 1 | 39 |
| total_frame_ms | 212.41 ± 33.48 | 175 / 250 | 39 |
| **FPS** | **4.71** | — | — |

**Directional split:**

| Direction | Mean total ms | n |
|---|---:|---:|
| → cam0 | 178.58 | 19 |
| → cam1 | 244.55 | 20 |
| **asymmetry (cam1 − cam0)** | **+65.97 ms** | — |

Raw: [`002_e16_camswitch_cam0_cam1.csv`](s035_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv)


## Appendix E — E16 alternating on Fix B (30 fps + flip-on-EOF)

`s042_e16_eof_30fps_x40` — E16 alternating cam0↔cam1 at 30 fps with
flip-on-EOF active.  First session where the directional asymmetry
collapsed to ≤ 1 ms.

| Stage | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| select_ms | 17.72 ± 0.56 | 17 / 19 | 39 |
| to_tensor_ms | 96.03 ± 0.71 | 95 / 97 | 39 |
| invoke_ms | 31.21 ± 2.43 | 28 / 36 | 39 |
| detect_ms | 0.59 ± 0.50 | 0 / 1 | 39 |
| total_frame_ms | 145.56 ± 2.52 | 141 / 150 | 39 |
| **FPS** | **6.87** | — | — |

| Direction | Mean total ms | n |
|---|---:|---:|
| → cam0 | 146.11 | 19 |
| → cam1 | 145.05 | 20 |
| **asymmetry (cam1 − cam0)** | **-1.06 ms** | — |

Raw: [`001_e16_camswitch_cam0_cam1.csv`](s042_e16_eof_30fps_x40/001_e16_camswitch_cam0_cam1.csv)


## Appendix F — E17 per-switch JPEGs (visual inspection)

E17 captures alternating cam0↔cam1 JPEGs into MicroPython heap
during the timing loop, then writes them to LFS after the loop.
Two threshold values are tested per session (`switch_drain` 2 and 1).

### `s036_e17_drain_ab`

*Empty session (driver failed to record frames).*

### `s037_e17_drain_ab`

*Empty session (driver failed to record frames).*

### `s038_e17_drain_ab`

| Threshold | elapsed_ms mean ± σ | JPEG bytes mean | Frames saved | CSV |
|---:|---:|---:|---:|---|
| 2 | 237.6 ± 33.6 | 24870 | 16 | [`001_e17_switch_drain_t2.csv`](s038_e17_drain_ab/001_e17_switch_drain_t2.csv) |
| 1 | 173.0 ± 37.2 | 24732 | 16 | [`002_e17_switch_drain_t1.csv`](s038_e17_drain_ab/002_e17_switch_drain_t1.csv) |

### `s039_e17_drain_ab`

| Threshold | elapsed_ms mean ± σ | JPEG bytes mean | Frames saved | CSV |
|---:|---:|---:|---:|---|
| 1 | 201.4 ± 2.3 | 21641 | 16 | [`001_e17_switch_drain_t1.csv`](s039_e17_drain_ab/001_e17_switch_drain_t1.csv) |

### `s040_e17_drain_ab`

*Empty session (driver failed to record frames).*

### `s041_e17_eof_check`

| Threshold | elapsed_ms mean ± σ | JPEG bytes mean | Frames saved | CSV |
|---:|---:|---:|---:|---|
| 2 | 201.6 ± 2.4 | 21164 | 8 | [`001_e17_switch_drain_t2.csv`](s041_e17_eof_check/001_e17_switch_drain_t2.csv) |
| 1 | 202.0 ± 3.3 | 21086 | 8 | [`002_e17_switch_drain_t1.csv`](s041_e17_eof_check/002_e17_switch_drain_t1.csv) |


## Appendix G — E18 head-to-tail (three-sweep benchmark)

E18 runs three back-to-back sweeps in one session at identical
firmware + scene + model: (A) fixed cam0, (B) fixed cam1,
(C) alternating.  Per-switch overhead = C_total − max(A_total, B_total).

### `s043_e18_headtail_drain2`

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | `total` | FPS |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed cam0 | 0.0 | 31.6 | 30.7 | 0.4 | **62.7** | 15.94 |
| B — fixed cam1 | 0.0 | 31.7 | 30.5 | 0.3 | **62.5** | 16.00 |
| C — alternating | 17.7 | 96.0 | 30.8 | 0.7 | **145.3** | 6.88 |

**Per-switch overhead** = C − max(A,B) = **82.5 ms** (131.6 % of baseline)

CSVs: [`001_e18_A_fixed_cam0.csv`](s043_e18_headtail_drain2/001_e18_A_fixed_cam0.csv) · [`002_e18_B_fixed_cam1.csv`](s043_e18_headtail_drain2/002_e18_B_fixed_cam1.csv) · [`003_e18_C_alt_cam0_cam1.csv`](s043_e18_headtail_drain2/003_e18_C_alt_cam0_cam1.csv)

### `s044_e18_headtail_drain2`

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | `total` | FPS |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed cam0 | 0.0 | 31.7 | 30.5 | 0.4 | **62.6** | 15.97 |
| B — fixed cam1 | 0.0 | 31.7 | 30.0 | 0.3 | **62.1** | 16.12 |
| C — alternating | 18.0 | 95.9 | 31.4 | 0.5 | **145.7** | 6.86 |

**Per-switch overhead** = C − max(A,B) = **83.1 ms** (132.7 % of baseline)

CSVs: [`001_e18_A_fixed_cam0.csv`](s044_e18_headtail_drain2/001_e18_A_fixed_cam0.csv) · [`002_e18_B_fixed_cam1.csv`](s044_e18_headtail_drain2/002_e18_B_fixed_cam1.csv) · [`003_e18_C_alt_cam0_cam1.csv`](s044_e18_headtail_drain2/003_e18_C_alt_cam0_cam1.csv)

### `s045_e18_post_refactor`

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | `total` | FPS |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed cam0 | 0.0 | 31.6 | 29.8 | 0.4 | **61.8** | 16.18 |
| B — fixed cam1 | 0.0 | 31.6 | 30.5 | 0.4 | **62.5** | 16.00 |
| C — alternating | 17.6 | 96.0 | 31.5 | 0.7 | **145.8** | 6.86 |

**Per-switch overhead** = C − max(A,B) = **83.3 ms** (133.4 % of baseline)

CSVs: [`001_e18_A_fixed_cam0.csv`](s045_e18_post_refactor/001_e18_A_fixed_cam0.csv) · [`002_e18_B_fixed_cam1.csv`](s045_e18_post_refactor/002_e18_B_fixed_cam1.csv) · [`003_e18_C_alt_cam0_cam1.csv`](s045_e18_post_refactor/003_e18_C_alt_cam0_cam1.csv)
