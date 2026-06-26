# Iter11 - Reality-To-Detection Latency Recompute

This iteration does not run new board code.  It recomputes latency-facing
quantities from the corrected B10 wall-clock runs.

## Rules

- Values marked measured come directly from board wall-clock experiments.
- Values marked derived are computed from measured FPS as `1000 / FPS`.
- This is not a full photon-to-result timestamp trace.  The current firmware
  does not yet stamp the same frame through capture, prep, invoke, and result
  publication with one wall-clock domain.
- Therefore the paper should report update periods and conservative latency
  bounds, not sub-stage millisecond precision from old tick/DWT tables.

## Corrected Values

| Quantity | Value | Status | Source |
| --- | ---: | --- | --- |
| Nominal sensor period | 33.333 ms | derived | camera init logs, 30 FPS target |
| Single-camera pipeline output period | 42.654 ms | derived | iter06 |
| Single-camera pipeline throughput | 23.444330 FPS | measured | iter06 |
| Standalone TPU invoke | 40.813641 ms | measured | iter03, 128 KB row |
| Standalone TPU throughput | 24.501614 FPS | measured | iter03 |
| Continuous 1:1 alternation combined period | 69.479 ms | derived | iter09, drain=1 |
| Continuous 1:1 alternation per-camera period | 138.959 ms | derived | iter09, drain=1 |
| Continuous 1:1 alternation combined throughput | 14.392763 FPS | measured | iter09, drain=1 |
| Manual switch `select -> to_tensor`, drain=1 | 93.891 ms | measured | iter08 |
| Manual switch `select -> to_tensor`, drain=2 | 131.085 ms | measured | iter08 |
| Multi-patch 2 invokes/frame frame period | 83.278 ms | derived | iter07 |
| Multi-patch 2 invokes/frame invoke period | 41.639 ms | derived | iter07 |

## Paper Guidance

For the corrected performance chapter:

- Single-camera 512x512 detection should be described as approximately
  `23.44 FPS`, or one detection output every `42.65 ms`.
- Continuous dual-camera `ratio(1,1)` with the valid drain setting should be
  described as approximately `14.39 FPS` combined, or one output every
  `69.48 ms`; each camera receives an update about every `138.96 ms`.
- `drain=2` must not be used for continuous `ratio(1,1)` latency claims on
  build 1544, because B10.5/B10.6 show tag/content phase errors.
- Manual/batched switching can cite the measured `select -> to_tensor` cost,
  but it is a different operating mode from continuous ISR-ratio alternation.
- Any old claims derived from `invoke_ms_sum`, `prep_stats()` tick fields, or
  DWT USB timing should be replaced by these update-period values unless a new
  wall-clock frame-stamping trace is added.

## Next Instrumentation If Needed

To produce true reality-to-result latency rather than update-period bounds,
add a monotonic microsecond timestamp at:

- camera buffer completion / source tag assignment;
- PrepTask staging handoff;
- InferTask invoke start/end;
- DetectionFrame publication;
- MicroPython `get_ex` receive.

Then report per-frame deltas from the same frame sequence ID.
