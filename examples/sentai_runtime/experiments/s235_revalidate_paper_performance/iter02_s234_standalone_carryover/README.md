# iter4_urb_instrumentation

Goal: recheck the contested 512x512 EdgeTPU timing on the physical board with
per-URB counters, while preserving the project rule that experiment artifacts
and logs stay inside the mission iteration folder.

## Firmware

Initial instrumentation firmware:

- `SentAI v1.0 build 1540 (2026-06-24 10:05:33)`
- QSTR regenerated for `sentai.tpu.urb_stats()`.
- Storage mode changed to MSC + REPL. Exit is explicit:
  `sentai.usb.drive(0)` after host unmount/eject.
- Storage boot latch accepts either DTC-RAM/noinit magic or SRC GPR magic.
  On this PC, DTC-RAM/noinit was zero after reset while GPRs preserved
  `0x57500001`, so GPR fallback was required for MSC entry.

Final wall-clock measurement firmware:

- `SentAI v1.0 build 1542 (2026-06-24 10:23:48)`
- Added `sentai.rtos.micros()` backed by GPT1 `TimerMicros()`.
- Added `sentai.rtos.cycles()` for DWT sanity checks.
- Extended `sentai.tpu.urb_stats()` rows from 10 to 13 fields by appending
  `submit_us`, `callback_us`, and `wait_us`.

## Staging

Model and script were copied through the mounted MSC drive:

- Device: `/dev/sdc`
- Model: `FLASH STORAGE`
- Mount: `/run/media/bogdan/0000-0001`

Copied artifacts:

- `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
  - size: `5591680`
  - SHA256: `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`
- `/lib/diag/_t_timing_urb.py`
  - size: `2297`
  - SHA256: `f8935884c5e0ae9f8f078b6554a718d72ac14599cf8926e5f01aa0629be7fb67`

## Run

Initial DWT run log:

- `run_t_timing_urb_1540_retry1.log`

Initial DWT summary:

| Metric | Avg per invoke |
|---|---:|
| invoke wall time from `ticks_ms` | 14.813 ms |
| input bytes requested | 811024 B |
| input submit time | 0.308 ms |
| input callback span | 4.752 ms |
| input wait span | 4.815 ms |
| instruction bytes requested | 371680 B |
| instruction wait span | 3.417 ms |
| output bytes requested | 10752 B |
| output wait span | 0.622 ms |
| event wait span | 0.025 ms |

Interpretation: the old small `input_ms` result is reproducible, and the new
URB counters show the same scale. This is a firmware/driver-visible completion
metric, not yet a validated USB bus wire-time measurement.

Clock sanity logs:

- `run_clock_calib_100invokes_1541_retry1.log`
- `run_clock_calib_300invokes_hostwall_1541.log`

These showed that DWT/tick-based time under-counts the USB blocking path:
300 invokes took `12416603 us` by GPT wall-clock and about `12.43 s` by host
wall-clock, but only `4731 ms` by `ticks_ms`.

Final wall-clock run log:

- `run_urb_wall_100invokes_1542.log`

Final GPT wall-clock summary:

| Metric | Avg per invoke |
|---|---:|
| total wall time | 41.514 ms |
| host wall sanity check | ~42.10 ms |
| input bytes requested | 811024 B |
| input wait span | 24.528 ms |
| instruction bytes requested | 371680 B |
| instruction wait span | 14.579 ms |
| parameter wait span | 0.137 ms |
| output wait span | 1.704 ms |
| event wait span | 0.038 ms |
| URB wait sum | 40.986 ms |
| residual non-URB time | ~0.528 ms |

Conclusion: the paper should use the GPT wall-clock URB fields, not the
DWT/tick-only timings. The contested `4.40 ms` input number was a counter
artifact, not a physical USB transfer time.
