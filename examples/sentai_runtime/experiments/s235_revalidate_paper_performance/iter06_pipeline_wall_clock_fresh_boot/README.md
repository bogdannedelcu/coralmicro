# s235 / iter06 - B10.3 pipeline wall-clock fresh-boot runs

Date: 2026-06-24

Purpose: rerun the single-camera pipeline throughput as one measured run per
fresh firmware boot, after iter05 showed that repeated
`pipeline.stop()`/`pipeline.start()` in one boot drives the TPU into repeated
`0B62` invoke failures.

Measurement source:

- Primary FPS: `observed_frames / sentai.rtos.micros()` elapsed around
  `sentai.pipeline.frame_count(after, timeout_ms)` events.
- Diagnostic fields: `prep_stats`, `infer_stats`, `pipeline.stats`,
  `get_ex(0, 0)`.

Configuration:

- model:
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
- build: `SentAI v1.0 build 1544`
- camera: `0`
- resolution: `512 x 512`
- chunk size: `128 KB`
- descriptor cache: disabled
- direct tensor: enabled
- target/prep FPS caps: disabled
- invokes per frame: `1`
- warmup events: `5`
- measured events: `100`

Artifacts:

- host script source: `pipeline_wall_clock_once.py`
- board script path: `/lib/diag/pipeline_wall_clock_once.py`
- board CSV path per run: `/diags/s235_b10_pipeline_once/run_NN.csv`
- host stdout capture per run: `run_once_NN_build1544.log`

Status: completed.

Corrected result:

| run | wall_us | FPS | timeouts | infer_fail |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 4390419 | 22.776869 | 0 | 0 |
| 1 | 4235336 | 23.610878 | 0 | 0 |
| 2 | 4235444 | 23.610275 | 0 | 0 |
| 3 | 4235558 | 23.609640 | 0 | 0 |
| 4 | 4234778 | 23.613989 | 0 | 0 |

Mean FPS: `23.444330`

Sample sigma: `0.373126 FPS`

All five runs produced `100/100` measured events with zero timeouts and zero
`infer_fail`.

Interpretation:

- The old `41-42 FPS` single-camera 512x512 pipeline claim is superseded by
  the fresh-boot wall-clock result above.
- Per-stage values in the CSV are diagnostic only.  They come from existing
  pipeline stats and must not be used as primary wall-clock timing in the
  rewritten paper table.
