# s239 — ReLU detectors: full detection-pipeline FPS on board (all core tasks live)

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s239 · follows [s238](../s238_relu_v1_board_speed_sweep/)
(isolated invoke) — this measures the **end-to-end pipeline** throughput with the
concurrent tasks running.

## What this is

The user asked for the realistic on-board FPS of the full camera→detection
pipeline for the 8 ReLU (`relu_v1`) models, with the important tasks running
simultaneously (contention on the CSI camera, PXP, USB, and SEMC bus) — even with
an empty scene (zero detections is fine; we measure throughput, not accuracy).

Concurrent tasks during the measurement (the SentAI detection pipeline):
- **CameraTask** — OV5640 CSI capture (cam0, 640×480, ~30 fps sensor)
- **PrepTask** — PXP scale + int8 quant of the camera frame into the input tensor
- **InferTask** — EdgeTPU `invoke()` over USB2

Methodology from [s235/iter06](../s235_revalidate_paper_performance/iter06_pipeline_wall_clock_fresh_boot/):
`sentai.pipeline.start()` runs the pipeline; **FPS = observed_frames /
`sentai.rtos.micros()`** elapsed across `sentai.pipeline.frame_count(last, tmo)`
events (5 warmup + 80 measured). `direct_tensor(1)`, `invokes_per_frame(1)`,
`target_fps`/`prep_fps` caps off, `desc_cache(0)`. Board build 1560.

## Results (cam0 640×480, all core tasks live)

| model | **pipeline FPS** | infer fails | invoke_ms (in-pipeline) |
|---|--:|--:|--:|
| msblock    | 27.6 | 0 | 11.1 |
| c3         | 27.6 | 0 | 11.0 |
| c2f        | 27.4 | 0 | 11.0 |
| c2f_pan2   | 26.9 | 0 | 10.9 |
| gelan      | 26.7 | 0 | 10.8 |
| c2f_thick  | 26.6 | 0 | 10.8 |
| gelan_pan2 | 26.0 | 0 | 10.9 |
| c2f_deep   | 25.9 | 0 | 10.7 |

- **All 8 sustain ~26–28 FPS, zero infer failures**, including c2f_thick (crashed
  as SiLU pre-fix).
- **Camera-bound, not compute-bound.** The in-pipeline `invoke_ms` is only ~11 ms —
  far below the ~33–35 ms of the *isolated* invoke in s238 — because
  `direct_tensor(1)` + the two-buffer pipeline overlap the ~26 ms input USB upload
  with the previous frame's compute. What's left gates on the ~30 fps OV5640 sensor
  period + PrepTask, so all models converge to ~27 FPS regardless of graph size.
- This is why the model choice barely moves pipeline FPS (25.9–27.6): the TPU has
  spare headroom behind the camera. Picking a heavier/more-accurate ReLU model
  (c2f_thick, mAP 0.83) costs ~1 FPS vs the lightest (msblock, mAP 0.68).

## VGA45 vs VGA30 — the sensor rate is NOT the bottleneck

Re-ran the sweep with the OV5640 at **45 fps** (`sentai.camera.init(1, 45)`;
switching fps needs a fresh boot — `init` a 2nd time with a different fps returns
-11; `fs.sync()` before `sys.reset()` to avoid the FileX corruption seen earlier).

| model | VGA30 FPS | VGA45 FPS |
|---|--:|--:|
| msblock    | 27.6 | 27.1 |
| c2f        | 27.4 | 26.8 |
| c3         | 27.6 | 26.9 |
| c2f_pan2   | 26.9 | 26.3 |
| gelan      | 26.7 | 26.1 |
| gelan_pan2 | 26.0 | 25.4 |
| c2f_deep   | 25.9 | 25.4 |
| c2f_thick  | 26.6 | 26.0 |

**Bumping the sensor 30→45 fps did NOT raise pipeline FPS — it's flat (marginally
lower).** So the pipeline is **PrepTask-bound (PXP scale + int8 quant) + pipeline
machinery (buffer swaps, IPC), not sensor-bound.** The 45 fps sensor produces
frames the pipeline can't consume (~18 fps dropped), adding a hair of overhead.
`invoke_ms` stays ~10–11 ms, so the TPU isn't the limit either. To exceed ~27 fps
you'd optimize PrepTask/PXP, not the camera frame rate.

Results: `results.csv` (VGA30), `results_vga45.csv` (VGA45).

## Flow (optical flow) — NOT co-run: a build-1560 limitation

The request included "all important tasks", which in SentAI also means the optical-
flow task. On build 1560 **flow and the detection pipeline do not cleanly coexist**:
- `flow.start(-1)` (the s209 "share the stream" mode) returns -2 — this build's
  binding only accepts cam0/cam1, not the shared tap.
- `flow.start(0)` grabs cam0 exclusively → conflicts with the pipeline on cam0;
  starting it around a running pipeline wedged the camera path (a failed
  `edgetpu-custom-op invoke`, then a transient USB drop).
- `flow.start(1)` (separate cam1) starts, but then `tpu.load()` fails while flow
  holds a camera/MUX resource, and starting flow after the pipeline left flow
  not-running + wedged.

So flow co-running needs a firmware fix (camera-MUX quota arbitration between
FlowTask and the detection pipeline, cf. `detection_task.cc` cam0/cam1 quotas).
Measured here: the detection pipeline (camera+prep+TPU), which is the dominant
concurrent workload. Operator directed "without flow" once this surfaced.

## Files

- `run.log` — full board session log
- `results.csv` — the FPS table
- driver is inline in the session (uses `../s236.../board_serial.py`)

## Gotchas

- Load-vs-parse: `board_serial.cmd()` returns lines with a trailing `\r`; compare
  with `"L 0" in rc`, not `rc == "L 0"` (an early version false-reported LOAD FAIL).
- After `pipeline.stop()`, **wait for `pipeline.running()==False` before the next
  `tpu.load()`** — the InferTask must release the shared interpreter/arena first.
- `sys.reset()` mid-FileX-op corrupted the volume this session (garbage dir, files
  gone) → had to `sentai.fs.format()` + re-upload via MSC. Sync before any reset.
