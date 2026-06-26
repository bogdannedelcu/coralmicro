# TD-S10-B10 - Revalidate Performance Chapter Experiments

## Objective

Implement A10 by rerunning the paper's Section 4.2 performance experiments
table by table, using board-valid wall-clock timing and preserving all
artifacts under the B10 mission folder.

Primary artifact root:

`examples/sentai_runtime/experiments/s235_revalidate_paper_performance/`

## Rules

- Run performance experiments on the physical board unless the row is explicitly
  host-pycoral or emulator-only.
- Do not use DWT or `xTaskGetTickCount()` as the primary wall-clock source for
  TPU USB timing.
- Prefer `sentai.rtos.micros()` / `TimerMicros()` for board wall time.
- Use URB wall stats for TPU transfer-phase breakdown.
- Use camera frame counters only for camera-produced-frame rates, not TPU
  invoke duration.
- Save logs, scripts, source diffs, firmware versions, model hashes, board FS
  artifacts, and summaries in `s235/iterNNN_*`.
- Use `sentai.fr` / `sentai.fs` for board-side mission logs where possible.
- Use MSC/mounted-drive transfer for large models/assets.
- Keep old values visible as historical context, but label them
  `superseded` once corrected.

## Current Baseline From S234

The S234 recheck already invalidated the paper's DWT-based 512x512 standalone
invoke breakdown.

Authoritative result to carry into B10 until rerun in the consolidated folder:

- model:
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`
- input tensor:
  `uint8[1,512,512,3]`, 786432 bytes
- model SHA256:
  `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`
- firmware:
  `SentAI v1.0 build 1542`
- 100 invokes wall time:
  `4151435 us`
- standalone invoke wall time:
  `41.514 ms`
- standalone throughput:
  `24.09 FPS`
- URB wall-time per invoke:
  input `24.528 ms`, instructions `14.579 ms`, parameters `0.137 ms`,
  output `1.704 ms`, event `0.038 ms`
- conclusion:
  the old `14.19 ms / 73-75 FPS` standalone claim for this 512x512 model is
  not valid as wall-clock board throughput.

Original S234 artifacts live in:

`examples/sentai_runtime/experiments/s234_tpu512_timing_recheck/iter4_urb_instrumentation/`

B10 carry-over copy:

`examples/sentai_runtime/experiments/s235_revalidate_paper_performance/iter02_s234_standalone_carryover/`

## Table Workplan

### B10.1 - Inventory Paper Tables And Old Evidence

Status: completed

Tasks:

- [x] map every Section 4.2 table row to the old script/log family that
  produced it;
- [x] mark each row as one of:
  `rerun-required`, `camera-counter-rerun`, `content-validation-rerun`,
  `host-only`, or `remove/defer`;
- [x] record whether the old value depends on DWT, `ticks_ms`,
  `pipeline.calibrate`, `pipeline.prep_stats`, `pipeline.infer_stats`,
  camera frame counters, or external host timers.
- [x] refine old evidence anchors down to exact script/log paths where needed
  before each rerun.

Output:

- `s235/iter01_table_inventory/table_inventory.md`
- `s235/iter01_table_inventory/table_inventory.csv`

### B10.2 - Pure TPU Optimization Matrix

Status: completed

Target paper rows:

- "Overall progression" pure TPU start/final;
- "TPU optimization tweaks";
- standalone invoke breakdown.

Tasks:

- [x] import the corrected S234 standalone TPU wall-clock result into the
  consolidated B10 artifact folder;
- [x] create the first one-condition pure TPU wall-clock driver in
  `s235/iter03_pure_tpu_wall_matrix`;
- [x] rerun the current-firmware chunk-size matrix for 33 KB, 36 KB, 64 KB,
  and 128 KB;
- [x] use board wall-clock microseconds for total invoke FPS;
- [x] use URB wall stats for phase breakdown;
- [x] preserve model hash, firmware build, chunk size, descriptor/cache settings,
  and transfer byte counts.
- [x] decide whether to reconstruct old intermediate firmware states
  (pre-zero-copy, pre-persistent-semaphore, etc.) or replace Table 8 with a
  corrected current-firmware chunk-size table.
- [x] test the experimental descriptor/instruction cache idea and reject it
  if it fails correctness/stability validation.

Output:

- corrected pure TPU chunk-size table in
  `s235/iter03_pure_tpu_wall_matrix/pure_tpu_wall_matrix_summary.csv`;
- corrected standalone invoke breakdown;
- old DWT table marked superseded.

Current build 1543 result:

| Chunk | Wall ms/invoke | FPS | Input wait ms/invoke | Instructions wait ms/invoke |
| ---: | ---: | ---: | ---: | ---: |
| 33 KB | 41.547539 | 24.068813 | 24.603979 | 14.598960 |
| 36 KB | 41.428139 | 24.138182 | 24.508600 | 14.572780 |
| 64 KB | 41.001480 | 24.389364 | 24.244680 | 14.435160 |
| 128 KB | 40.813641 | 24.501614 | 24.151621 | 14.361740 |

Conclusion: the old 73-75 FPS standalone 512x512 values are superseded by
valid board wall-clock results around 24 FPS.  The old "33 KB best" conclusion
is not reproduced on build 1543; 128 KB is slightly faster in this current
matrix, though all tested chunk sizes are close.

Descriptor-cache validation (`s235/iter04_desc_cache_validation`) failed:

- OFF baseline: 20/20 invokes succeeded, output hash `0b4b28d1`, input wait
  `24.149300 ms`, instruction wait `14.359500 ms`.
- ON after prime: 19/20 invokes failed with `E:0B61:0` / `SendInputs failed`.
- `skip_ins=19` proves the skip path activated, but URB stats then showed
  input timeout/submit failures.

Conclusion: `desc_cache` is not a valid optimization for the paper.  Any FPS
computed from the ON row is a failure artifact, not throughput.

Table 8 policy: replace the old historical optimization-step table with the
corrected current-firmware chunk-size table.  The old intermediate rows were
mixed with invalid timing domains and should not be reconstructed for the paper
unless each historical firmware state is rerun with board wall-clock timing.

### B10.3 - Stable End-To-End Pipeline

Status: completed

Target paper rows:

- overall progression single-patch pipeline;
- statistical pipeline timing table;
- latency inputs for the single-camera case.

Tasks:

- [x] migrate or wrap pipeline measurement so FPS is frames over measured wall
  microseconds;
- [x] re-instrument or re-label per-stage timing fields;
- [x] run at least five repeated runs matching the paper structure;
- [x] record failures, frame counts, invoke counts, and pipeline state.

Output:

- corrected pipeline FPS;
- corrected/stated-validity per-stage table;
- stability/failure-rate evidence.

Artifacts:

- invalid repeated stop/start probe:
  `s235/iter05_pipeline_wall_clock`;
- valid fresh-boot repeated-run probe:
  `s235/iter06_pipeline_wall_clock_fresh_boot`;
- summary CSV:
  `s235/iter06_pipeline_wall_clock_fresh_boot/pipeline_wall_clock_summary.csv`;
- board-side CSV copies:
  `s235/iter06_pipeline_wall_clock_fresh_boot/board_run_00.csv` through
  `board_run_04.csv`.

Corrected build 1544 result, one measured run per fresh firmware boot:

| run | Wall us / 100 events | FPS | timeouts | infer_fail |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 4390419 | 22.776869 | 0 | 0 |
| 1 | 4235336 | 23.610878 | 0 | 0 |
| 2 | 4235444 | 23.610275 | 0 | 0 |
| 3 | 4235558 | 23.609640 | 0 | 0 |
| 4 | 4234778 | 23.613989 | 0 | 0 |

Mean FPS: `23.444330`, sample sigma `0.373126 FPS`.

Conclusion: the old `41-42 FPS` single-camera 512x512 pipeline value is not
valid under board wall-clock revalidation on build 1544.  The corrected
fresh-boot pipeline rate is about `23.44 FPS` for the same model and camera
configuration, close to the corrected pure TPU standalone ceiling.  Iter05
also showed that repeated `pipeline.stop()`/`pipeline.start()` in one boot is
not a valid repeated-run protocol: run 0 succeeded, then subsequent runs
entered repeated `0B62` TPU invoke failures.  Independent pipeline repeats
must use fresh firmware boot/reflash until the stop/start recovery path is
fixed.

### B10.4 - Multi-Patch Throughput

Status: completed

Target paper rows:

- multi-patch throughput matrix;
- any derived "higher total TPU throughput" statements.

Tasks:

- [x] rerun prep-rate and invokes-per-frame combinations;
- [x] compute both frame FPS and TPU invoke FPS from wall microseconds;
- [x] log backpressure and failures.

Output:

- corrected multi-patch throughput table.

Artifacts:

- fresh-boot matrix:
  `s235/iter07_multi_patch_wall_clock_fresh_boot`;
- summary CSV:
  `s235/iter07_multi_patch_wall_clock_fresh_boot/multi_patch_wall_clock_summary.csv`;
- board-side CSV copies:
  `s235/iter07_multi_patch_wall_clock_fresh_boot/board_run_*.csv`.

Corrected build 1544 result:

| prep_fps | invokes/frame | mode | Frame FPS | Invoke FPS | failures |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 1 | 0 | 23.610554 | 23.610554 | 0 |
| 30 | 1 | 0 | 23.614239 | 23.614239 | 0 |
| 30 | 2 | 1 | 12.008011 | 24.016022 | 0 |
| 20 | 3 | 1 | 8.058186 | 24.174555 | 0 |
| 15 | 4 | 1 | 6.062988 | 24.251953 | 0 |

Conclusion: the old multi-patch claims of `48.5`, `54.8`, and `56.0` TPU
FPS are not reproduced under board wall-clock timing on build 1544.
Multi-invoke correctly preserves the requested invokes/frame ratio, but total
TPU invoke throughput remains near `24 FPS`, close to the corrected pure TPU
and single-camera pipeline ceiling.  The old claim that PrepTask throttling
raises `ipf=1` throughput to about `29.8 FPS` is also not reproduced; the
corrected `prep_fps=30, ipf=1` value is `23.614239 FPS`.

### B10.5 - Dual-Camera Switching And Alternation

Status: completed / follow-up required for visual drain policy

Target paper rows:

- camera switch latency;
- continuous 1:1 alternation throughput;
- camera-related rows in overall progression.

Tasks:

- [x] rerun sequential switch timing as `select() -> to_tensor()` with
  `sentai.rtos.micros()`;
- [x] rerun 1:1 alternation with drain=1 and drain=2 through the parallel
  pipeline;
- [x] compute camera and infer FPS from counts over measured wall time;
- [x] recover the older correctness strategy: OV5640 test patterns plus
  `peek5_b40()` single-buffer content/tag sampling;
- [x] follow up the unexpected `drain=2` pattern mismatch before presenting it
  as a final visual-correctness conclusion.

Output:

- corrected switch latency table;
- corrected alternation throughput table.

Artifacts:

- switch timing and pattern checks:
  `s235/iter08_camera_switch_latency`;
- pipeline alternation:
  `s235/iter09_dual_camera_pipeline_alternation`.

Corrected build 1544 switch timing, first same-camera row dropped:

| drain | select mean | to_tensor mean | total mean | effective FPS |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 13.907 ms | 79.984 ms | 93.891 ms | 10.651 |
| 2 | 13.498 ms | 117.587 ms | 131.085 ms | 7.629 |

Corrected build 1544 parallel alternation:

| ratio | drain | frame FPS | cam0 FPS | cam1 FPS | invoke FPS | failures |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1:1 | 1 | 14.392763 | 7.196382 | 7.196382 | 14.392763 | 0 |
| 1:1 | 2 | 4.981067 | 2.490534 | 2.490534 | 4.981067 | 0 |

Pattern-integrity check using synthetic sensor patterns:

| drain | ratio | frames | correct | scrambled | wrong tag | note |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 1 | 1:1 | 100 | 100 | 0 | 0 | passed synthetic tag/content check |
| 2 | 1:1 | 100 | 46 | 1 | 53 | unexpected mismatch; follow-up needed |

Conclusion so far: the old ~21 FPS alternating-camera claim is not reproduced
with the current 512x512 TPU pipeline. Drain=1 gives about 14.39 combined FPS
and balanced 50/50 camera outputs; drain=2 is much slower at about 4.98
combined FPS. Correctness must remain tied to content validation, not just
throughput.

Focused drain follow-up (`camera_drain_probe_once.py`, no per-frame FS writes)
confirmed the mismatch:

| drain | ratio | frames | correct | scrambled | wrong tag | cam0 | cam1 | verdict |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 1:1 | 50 | 50 | 0 | 0 | 26 | 24 | valid |
| 2 | 1:1 | 50 | 0 | 25 | 25 | 50 | 0 | invalid for continuous 1:1 |

Interpretation: under continuous `ratio(1,1)`, `drain=2` is not a safer
version of `drain=1`. The ISR ratio scheduler can flip the MUX again before
the consumer's two-frame drain finishes, so tag/content phase can slip.
`drain=1` matches the current FB2-gated/dirty-buffer semantics for continuous
alternation. Manual/batched camera switching should be treated separately.

### B10.6 - Visual Seam Validation

Status: completed for continuous ratio(1,1); manual/batched switching remains separate

Target paper rows:

- visual seam validation table.

Tasks:

- [x] rerun a small capture scenario for continuous `ratio(1,1)`;
- [x] validate mixed-frame artifacts by content inspection / script, not
  DWT/ticks;
- [x] record images and verdict JSON;
- [ ] keep manual/batched switching validation separate from continuous
  alternation policy.

Output:

- seam validation artifacts and corrected table.

Artifacts:

- `s235/iter10_visual_seam_validation`

Corrected continuous `ratio(1,1)` visual result, build 1544:

| drain | frames | correct | mismatch | mixed | verdict |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 2 | 2 | 0 | 0 | valid for continuous alternation |
| 2 | 2 | 1 | 1 | 0 | invalid for continuous alternation |

Method: first vertical bar / left-column signature.  Cam0 BARS has a white
left bar; the cam1 alternate pattern has a dark left bar.  The host classifier
samples the same left column from top to bottom; mixed white/dark means visible
seam, and tag/content disagreement means phase error.

Important caveat: JPEG capture/save time is not used as performance timing.
`save_jpeg` FileX writes took hundreds of milliseconds to seconds.  These
JPEGs are visual artifacts only.

### B10.7 - Reality-To-Detection Latency

Status: completed as corrected update-period bounds

Target paper rows:

- latency decomposition and derived bounds.

Tasks:

- [x] recompute using corrected camera period, corrected output period, corrected
  stage timings, and corrected switch/alternation period;
- [x] separate measured stages from derived bounds;
- [x] remove unsupported sub-microsecond precision unless directly measured by a
  reliable hardware timer.

Output:

- corrected latency decomposition.

Artifacts:

- `s235/iter11_latency_recompute`

Corrected latency-facing quantities:

| Quantity | Value | Status |
| --- | ---: | --- |
| Single-camera pipeline output period | 42.654 ms | derived from measured 23.444330 FPS |
| Standalone TPU invoke | 40.813641 ms | measured wall-clock |
| Continuous 1:1 alternation combined period | 69.479 ms | derived from measured 14.392763 FPS |
| Continuous 1:1 alternation per-camera period | 138.959 ms | derived from measured 7.196382 FPS/cam |
| Manual switch `select -> to_tensor`, drain=1 | 93.891 ms | measured |
| Manual switch `select -> to_tensor`, drain=2 | 131.085 ms | measured, not valid for continuous 1:1 policy |

Conclusion: until a same-frame wall-clock timestamp trace is added through
capture, prep, invoke, and publication, the paper should report corrected
update periods and conservative bounds, not old tick/DWT-derived sub-stage
latency numbers.

## Initial Table Classification

| Paper table/claim | Current status | Reason |
| --- | --- | --- |
| Overall pure TPU 32 -> 73-75 FPS | rerun-required | TPU FPS derived from affected timing path. |
| Overall pipeline 1.8 -> 41-42 FPS | rerun-required | Pipeline `fps_x100`/sleep-window measurements used tick-derived wall time. |
| Overall multi-patch 48.5/56 FPS | rerun-required | Invoke FPS and per-invoke ms depend on old timing. |
| Camera switch latency ~14 ms | camera-counter-rerun | May be valid if frame/timer based, but must be traced and rerun. |
| Continuous 1:1 alternation 8.7 -> 19.5 FPS | rerun-required | Mixes camera/pipeline counts and tick-based pipeline timing. |
| TPU optimization tweaks | rerun-required | Standalone invoke FPS invalid until wall-clock rerun. |
| Standalone invoke breakdown | superseded by S234 | DWT-as-wall-time invalid; S234 gives corrected wall/URB numbers. |
| Stable pipeline statistical table | rerun-required | `prep_stats`, `infer_stats`, and script duration use tick-based fields. |
| Multi-patch table | rerun-required | Needs frame FPS vs invoke FPS split over wall microseconds. |
| Switch latency table | camera-counter-rerun | Must confirm timing source. |
| Visual seam validation | content-validation-rerun | Not a TPU timing claim, but should be artifact-backed. |
| Reality-to-detection latency | derived-rerun | Must be recomputed after corrected FPS/stage timings. |

## Final B10 Consolidation

Status: completed

Paper-ready replacement artifacts:

- `s235/iter12_corrected_paper_tables/README.md`
- `s235/iter12_corrected_paper_tables/superseded_claims.csv`
- `s235/iter12_corrected_paper_tables/manuscript_rewrite_notes.md`

Final headline replacements:

| Claim family | Corrected value | Source |
| --- | ---: | --- |
| Standalone 512x512 TPU | 24.50 FPS | iter03 |
| Single-camera 512x512 pipeline | 23.44 FPS | iter06 |
| Multi-patch total invoke throughput | ~24 invokes/s | iter07 |
| Continuous dual-camera `ratio(1,1)` | 14.39 combined FPS, 7.20 FPS/cam | iter09 |
| Continuous `ratio(1,1)` drain policy | `drain=1` valid, `drain=2` invalid | iter08, iter10 |
| Single-camera output period | 42.654 ms | iter11 |
| Continuous 1:1 per-camera update period | 138.959 ms | iter11 |

B10 is now sufficient to rewrite Section 4.2 without relying on DWT-as-wall or
tick-derived TPU timing.  Remaining optional work is a separate manual/batched
camera-switch visual validation if the manuscript wants to discuss that mode
independently from continuous ISR-ratio alternation.

## Open Implementation Notes

- `sentai.rtos.micros()` and `sentai.rtos.cycles()` were added during S234.
- `sentai.tpu.urb_stats()` includes wall-microsecond fields in build 1542.
- `sentai.pipeline.calibrate()` still uses `sentai_ticks_ms()` for `wall_ms`;
  B10 must either patch it to `TimerMicros()` or avoid it for final timing.
- `sentai_tpu_invoke_internal`, `sentai_tpu_invoke_with_input`, and
  `sentai_tpu_invoke_slot_with_input` currently return tick-derived
  milliseconds; B10 must not use those return values as authoritative
  wall-clock invoke duration.
