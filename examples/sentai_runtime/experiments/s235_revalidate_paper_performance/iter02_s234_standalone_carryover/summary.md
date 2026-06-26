# s235 / iter02 - S234 Standalone TPU Carry-Over

Date: 2026-06-24

Purpose: import the already completed S234 wall-clock TPU timing result into
the B10 performance-chapter revalidation mission.

## Source

Copied from:

`examples/sentai_runtime/experiments/s234_tpu512_timing_recheck/iter4_urb_instrumentation/`

Copied artifacts:

- `README.md`
- `run_urb_wall_100invokes_1542.log`
- `urb_wall_100invokes.py`
- `build_sentai_runtime_retry12_urb_wall_us.log`
- `flash_retry10_urb_wall_us.log`

## Firmware And Model

- Firmware: `SentAI v1.0 build 1542 (2026-06-24 10:23:48)`
- Model:
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
- Input tensor: `uint8[1,512,512,3]`, 786432 bytes.
- Model SHA256:
  `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`

## Corrected Standalone Result

Run: 100 invokes, after warm-up.

| Metric | Value |
| --- | ---: |
| total wall time | 4,151,435 us |
| wall time per invoke | 41.514351 ms |
| throughput | 24.09 FPS |
| tick-derived time per invoke | 14.580000 ms |

The old tick/DWT-derived timing under-counts this path by about 2.85x.

## URB Wall-Time Breakdown

| Phase | Calls | Bytes requested | Wait per invoke |
| --- | ---: | ---: | ---: |
| instructions | 1300 | 37,168,000 | 14.578890 ms |
| input | 2500 | 81,102,400 | 24.527719 ms |
| parameters | 200 | 276,000 | 0.137420 ms |
| output | 100 | 1,075,200 | 1.704270 ms |
| event | 100 | 1,600 | 0.037780 ms |

URB wait sum per invoke: about `40.986 ms`.

## Paper Impact

This result supersedes the draft standalone invoke breakdown:

- old input time `4.40 ms` is invalid as physical USB transfer time;
- old invoke total `14.19 ms` is invalid as board wall-clock invoke time;
- corrected standalone wall throughput for this 512x512 model is currently
  `24.09 FPS`, pending B10 rerun inside the consolidated `s235` mission.

