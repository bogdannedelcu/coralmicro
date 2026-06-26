# s235 / iter07 - B10.4 multi-patch wall-clock fresh-boot matrix

Date: 2026-06-24

Purpose: remeasure the Section 4.2.4 multi-patch throughput table with valid
board wall-clock timing and with frame FPS separated from TPU invoke FPS.

Measurement source:

- Primary frame FPS: observed `sentai.pipeline.frame_count()` events divided
  by `sentai.rtos.micros()` elapsed time.
- Primary invoke FPS: `infer_stats().ok` delta divided by the same elapsed
  wall time.
- Existing `infer_stats().ms_sum`, `prep_stats()`, and `pipeline.stats()` are
  retained as diagnostics only.

Protocol:

- one matrix row per fresh firmware boot/reflash;
- no repeated `pipeline.stop()`/`pipeline.start()` in a single boot;
- 5 warmup frame events;
- 100 measured frame events per row.

Model and firmware:

- model:
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
- build: `SentAI v1.0 build 1544`
- chunk size: `128 KB`
- descriptor cache: disabled
- direct tensor: enabled
- target FPS cap: disabled

Matrix:

| run | prep_fps | invokes_per_frame | multi_invoke_mode |
| ---: | ---: | ---: | ---: |
| 0 | 0 | 1 | 0 |
| 1 | 30 | 1 | 0 |
| 2 | 30 | 2 | 1 |
| 3 | 20 | 3 | 1 |
| 4 | 15 | 4 | 1 |

`multi_invoke_mode=1` is DEFER, the race-free multi-invoke mode documented in
the old experiment notes for `invokes_per_frame > 1`.

Artifacts:

- host script source: `pipeline_multi_patch_once.py`
- board script path: `/lib/diag/pipeline_multi_patch_once.py`
- board CSV path per row:
  `/diags/s235_b10_multi_patch/run_NN_pfP_ipfI_modeM.csv`
- host stdout capture per row: `run_multi_NN_*.log`

Status: completed.

Corrected result:

| run | prep_fps | ipf | mode | frame FPS | invoke FPS | invokes/frame | fail | timeouts |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 1 | 0 | 23.610554 | 23.610554 | 1.000 | 0 | 0 |
| 1 | 30 | 1 | 0 | 23.614239 | 23.614239 | 1.000 | 0 | 0 |
| 2 | 30 | 2 | 1 | 12.008011 | 24.016022 | 2.000 | 0 | 0 |
| 3 | 20 | 3 | 1 | 8.058186 | 24.174555 | 3.000 | 0 | 0 |
| 4 | 15 | 4 | 1 | 6.062988 | 24.251953 | 4.000 | 0 | 0 |

Interpretation:

- The old `48.5`, `54.8`, and `56.0` TPU-FPS multi-patch claims are
  superseded by this wall-clock rerun.
- Multi-invoke does preserve the requested invokes-per-frame ratio, but total
  invoke throughput remains near `24 FPS`, close to the corrected pure TPU and
  single-camera pipeline ceiling.
- The throttled `prep_fps=30, ipf=1` row no longer reproduces the old `29.8`
  FPS value; it remains at about `23.61` invoke FPS.
- All rows completed with zero invoke failures and zero timeouts.
