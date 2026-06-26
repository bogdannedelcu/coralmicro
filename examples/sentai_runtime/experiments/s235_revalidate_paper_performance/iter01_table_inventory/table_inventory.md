# s235 / iter01 - Section 4.2 Table Inventory

Date: 2026-06-24

Purpose: classify every performance claim in the draft Section 4.2 before
rerunning B10 experiments.

## Timing Source Verdicts

- `DWT` / `sentai.diag.tpu_perf`: not valid as USB EdgeTPU wall time.
- `sentai.rtos.ticks_ms` / `xTaskGetTickCount`: not valid as primary TPU
  invoke wall time.
- `pipeline.calibrate().wall_ms` / `fps_x100`: currently tick-derived, so not
  authoritative for final paper throughput.
- `pipeline.prep_stats()` / `infer_stats()`: currently tick-derived for
  per-stage timing; useful for old debugging, not final paper timing.
- camera ISR/frame counters: potentially valid for camera-only production
  rates, but rerun or trace before final use.
- visual seam/content checks: independent from TPU timing, but should be
  artifact-backed.

## Table Inventory

| Paper section | Draft table/title | Key claims | Old evidence anchor | Timing dependency | B10 action |
| --- | --- | --- | --- | --- | --- |
| 4.2.1 | Overall progression | Pure TPU `32 -> 73-75 FPS`; pipeline `1.8 -> 41-42 FPS`; multi-patch `48.5/56 FPS`; switch `~14 ms`; alternation `8.7 -> 19.5 FPS` | `experiment.md` V22/V25 and paper draft | mixed DWT/ticks/calibrate/camera counters | rerun as consolidated summary after all subtables |
| 4.2.2 | TPU optimization tweaks | chunk `64 KB=32 FPS`, `128 KB=27 FPS`, `33 KB=73 FPS`, zero-copy/persistent sema `75.9 FPS` | `experiment.md` TPU throughput sprint; old E20/V13 notes | affected TPU invoke timing | rerun-required |
| 4.2.2 | Average standalone invoke breakdown | input `4.40 ms`, instructions `2.90 ms`, total `14.19 ms` | `experiment.md` detailed timing breakdown | invalid DWT-as-wall-time | superseded by S234; rerun/copy into B10 |
| 4.2.3 | Stable end-to-end pipeline throughput | `42.5 FPS`, `1.2 FPS` SDRAM baseline, `100% success` | `experiment.md` V22 notes and pipeline timing table | pipeline stats/calibrate tick-derived | rerun-required |
| 4.2.3 | Statistical pipeline timing | infer `40.74 FPS`, prep `41.14 FPS`, `invoke 23.06 ms`, `PXP 9.42 ms` | `experiment.md` detailed timing section | tick-derived `prep_stats` and `invoke_ms_sum` | rerun-required with micros instrumentation |
| 4.2.4 | Multi-patch throughput | `44.0`, `29.8`, `48.5`, `54.8`, `56.0` infer FPS | `experiment.md` multi-patch viability | likely tick-derived invoke/pipeline stats | rerun-required |
| 4.2.5 | Camera-switch latency | cold `11/14/18 ms`; running pipeline `5/12/21 ms` | `experiment.md` camera switch sections | source must be audited; may be camera/tick based | camera-counter-rerun |
| 4.2.5 | Continuous 1:1 alternation | single cam `40.9 FPS`; old drain `8.7 FPS`; new drain `19.5 FPS` | `experiment.md` V22++ alternation | mixed camera counters and pipeline timing | rerun-required |
| 4.2.5 | Visual seam validation | 22 frames, 0 mixed artifacts | `experiments/s082_e39_cam_switch_visual/` referenced in old notes | content-based | recover or rerun content validation |
| 4.2.6 | Reality-to-detection latency | single-camera `24..46 ms`, switching `51..73 ms`, expected `35/62 ms` | derived from pipeline FPS/stage table | derived from contaminated timings | recompute after reruns |

## Immediate Experiment Order

1. Reuse S234 as the first corrected standalone TPU baseline and archive the
   relevant result summary into this mission.
2. Patch or wrap the pipeline timing path so `pipeline` FPS is computed from
   `sentai.rtos.micros()` / `TimerMicros`.
3. Rerun 512x512 single-camera pipeline with the same model and current board.
4. Rerun multi-patch matrix.
5. Rerun camera switch and alternation.
6. Recover or rerun visual seam validation.
7. Rewrite the Section 4.2 tables with corrected values.

## Notes For Paper Rewrite

- Avoid claiming that the board reaches `73-75 FPS` for the 512x512 yolo_1
  model unless a wall-clock rerun proves it.  S234 currently says the corrected
  standalone wall rate is about `24.09 FPS`.
- The old reviewer objection is valid for Table 9: `4.40 ms` cannot represent
  the full 786 KiB USB input transfer.
- Preserve relative engineering conclusions only after rerun.  For example,
  chunk-size sweeps may still show a best region, but the absolute FPS values
  must change.

