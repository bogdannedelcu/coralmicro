# s235 / iter05 - B10.3 pipeline wall-clock revalidation

Date: 2026-06-24

Purpose: remeasure the Section 4.2.3 single-camera end-to-end pipeline
throughput with board wall-clock timing.

Primary timing source:

- `sentai.rtos.micros()` around detection-event deltas from
  `sentai.pipeline.frame_count(after, timeout_ms)`.

Diagnostic timing only:

- `sentai.pipeline.prep_stats()`
- `sentai.pipeline.infer_stats()`
- `sentai.pipeline.stats()`
- `sentai.pipeline.get_ex(0, 0)` latest-frame fields

These diagnostic values are retained for debugging but are not the primary
paper FPS source because their internal timing is tick-based.

Planned run configuration:

- model:
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
- camera: `0`
- resolution: `512 x 512`
- `sentai.tpu.chunk_size(128 * 1024)`
- `sentai.tpu.desc_cache(0)`
- `sentai.pipeline.direct_tensor(1)`
- `sentai.pipeline.target_fps(0)`
- `sentai.pipeline.prep_fps(0)`
- `sentai.pipeline.invokes_per_frame(1)`
- repeated runs: `5`
- measured frames per run: `100`
- warmup detection events per run: `5`

Artifacts:

- host script source: `pipeline_wall_clock_probe.py`
- board script path: `/lib/diag/pipeline_wall_clock_probe.py`
- board CSV path:
  `/diags/s235_b10_pipeline_wall/pipeline_wall_clock.csv`
- host stdout capture:
  `run_pipeline_wall_clock_build1544.log`

Status: completed as an invalid-protocol probe.

Observed result:

- run 0 succeeded: `100` events in `4484868 us`, `22.297201 FPS`.
- runs 1-4 failed after repeated `pipeline.stop()` / `pipeline.start()` in the
  same boot, producing continuous `0B62` TPU invoke failures and zero measured
  events.
- the board CSV append file was partially corrupted/truncated during the error
  storm, so the host stdout log is the authoritative iter05 artifact.

Conclusion: repeated stop/start inside one boot is not a valid way to collect
independent pipeline repeats.  The corrected protocol is one measured run per
fresh firmware boot; see `../iter06_pipeline_wall_clock_fresh_boot`.
