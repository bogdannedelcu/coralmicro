# s246 — dual-camera alternation at FULL sensor rate, no loss, no mixing

Anchor: `TD-S10-A10` (paper §4.2) · EXP-s246 · follows
[s245](../s245_camera_trigger_hw_probe/) (which closed the trigger route) and
[s243](../s243_camera_alternation_yuv/) (which measured the −44 % cost).

**WBS**: TD-S10-A10
**Started**: 2026-09-04
**Status**: 🟢 sweep COMPLETE — objective met on speed and integrity, blocked on labelling

## Objective

Capture frames alternately from both cameras at the **full sensor rate**,
without losing a frame and without any frame mixed from both cameras.

## Oracle (house method — operator-specified)

The sensors generate their own synthetic patterns, so the expected signature is
known in advance and does not depend on the scene:

- cam0 ← **BARS** (`0x503D=0x80`): col-40 B byte ≈ 0xFF (white bar)
- cam1 ← **HBAND** (`0x503D=0x88`): col-40 B byte ≈ 0x1F (dark band)

`peek5_b40()` does a **single dequeue** and samples the B channel at column 40
from rows {0, H/4, H/2, 3H/4, H−1} of that one buffer, returning the samples
plus the buffer's camera tag. Sampling several rows of ONE buffer is what
detects a horizontal seam — a frame cut across two cameras. (Calling
`peek_row_at` five times instead dequeues five *different* buffers and
manufactures false SCRAMBLED — the documented bug `_t_pattern_31.py` fixed.)

This gives, per frame: which camera by **content**, which camera by **tag**, and
whether the frame is internally consistent.

## Findings so far

### 1. `frame_count()` ticks at HALF the sensor rate — confirmed, and it matters

Measured over a 2 s window with the sensor at 45 fps:

| counter | rate |
|---|--:|
| `sentai.camera.frame_count()` | **22.5 Hz** |
| `sentai.camera.sensor_frames()` (new) | **45.5 Hz** |

The comment in `modsentai_camera.c` claimed "one tick = one actual sensor frame
captured". It is wrong; `g_camera_frame_seq` is gated on `fb2_done` alone
(`camera_support.c:432`). Auto-memory `[[csi-isr-is-timing-sensitive]]` was
right. Comment corrected and `sensor_frames()` added in build 1597.

**Why it matters:** the ratio-alternate scheduler keys its modulo off
`g_camera_frame_seq`, so **`ratio(1,1)` flips the MUX once per TWO sensor
frames**, not once per frame.

### 2. The oracle itself is sound (PHASE A)

| camera | n | fps | correct class | scrambled | mismatch |
|---|--:|--:|--:|--:|--:|
| cam0 (BARS) | 40 | 44.1 | 39 | 1 | 0 |
| cam1 (HBAND) | 40 | 44.0 | 40 | 0 | 0 |

Static single-camera capture runs at the full sensor rate and classifies
essentially perfectly, so what it reports about alternation can be trusted.

### 3. The central result: full speed is reachable, but the LABEL breaks

Full sweep, `ratio(1,1)`, 120 frames per config, from
[iter3_power_trace/results.csv](iter3_power_trace/results.csv):

| drain | dirty | fps | frames dropped | SCRAMBLED | tag↔content mismatch |
|--:|--:|--:|--:|--:|--:|
| 0 | 0 | **44.8** | **0** | 1 / 120 | **35 / 120 (29 %)** |
| 0 | 1 | 22.6 | 115 | 0 | **0** |
| 0 | 2 | 0.7 | — | — | 21 grab failures |
| 1 | 0 | 22.3 | 116 | 0 | 10 |
| 1 | 1 | 22.3 | 116 | 0 | **0** |
| 1 | 2 | 0.0 | — | — | 21 grab failures |
| 2 | 0 | 11.3 | 173 | **28** | 30 |
| 2 | 1 | 9.0 | 138 | **22** | 25 |
| 2 | 2 | 0.0 | — | — | 21 grab failures |

**`drain=0, dirty=0` meets the objective except for the label.** 120 frames out
of 120 sensor frames — **zero dropped** — at 44.8 fps, with one internally-mixed
frame in 120. Tags alternate perfectly (60/60) while content splits 52/67, so
29 % of frames are attributed to the wrong camera. Throughput and frame
integrity are solved; the tag↔camera association at MUX flip is not.

**The two perfectly clean configs pay exactly 2×.** `(0,1)` and `(1,1)` give 0
scrambled and 0 mismatch but 22.3–22.6 fps with ~115 frames discarded — half,
exactly as flipping once per two sensor frames predicts.

**`drain=2` is actively harmful, not merely conservative** — 28 and 22 mixed
frames, and the tag distribution collapses entirely (tag0=59/tag1=0, then
tag0=0/tag1=47). It is a regression, not a safe fallback.

**`dirty=2` stops capture** — every queued buffer is marked dirty, the recovery
loop runs out, 21 grab failures, zero frames.

So the 2× is real and reachable, and the blocker is purely the tag association
at flip — the same "dynamic tag association at MUX flip" question
`_t_pattern_31.py` was originally written to investigate.

And this explains s243's 21.6 fps exactly: flip every two sensor frames,
`dirty_skip_n=1` discards the first of each pair, half the frames survive.

### 4. It is NOT a power problem (FNIRSI FNB48S trace)

The board is powered through an FNIRSI FNB48S, and `tools/fnirsi_power.py` from
`../ch569w` has been logging it at 100 SPS into `/tmp/power_log.jsonl`
continuously since 2026-08-18. We marked the run window and sliced that log
rather than starting a second logger.

Over the whole 47.9 s sweep (5141 samples):

| | value |
|---|--:|
| V min / mean / max | **4.988** / 5.039 / 5.059 V |
| P mean / max | 1.834 / 1.915 W |
| V min in the last 1.0 s | 5.016 V |

**The rail held.** Earlier mid-sweep resets were therefore a software fault, not
brownout. The one we could attribute was the uncaught `peek5_b40 failed`
exception at `dirty=2`, now handled. Two changes landed together (removing all
NAND writes from the driver, and making grab failures non-fatal), so we cannot
fully separate their contributions — but the power route is excluded on data.

## Pass criteria (unchanged)

- aggregate fps ≥ 0.9 × sensor rate at `ratio(1,1)`
- SCRAMBLED = 0
- tag↔content mismatch = 0
- cam0/cam1 share within 45–55 %

`drain=0, dirty=0` meets criteria 1, 2 and 4 (44.8 fps, 1/120 scrambled, 60/60
tag balance, zero dropped) and fails **only** criterion 3, the tag↔content
match, at 29 %.

## Files

- `../../diag/_t_s246_alt.py` — the on-board driver (whole sweep lives there per
  `[[no-complex-serial-orchestration]]`; host only pushes, execs once, downloads)
- `iter1_embedded_frame_counter/` — abandoned approach, kept as a record
- `iter2_pattern_row_oracle/` — pattern characterisation helper
- `iter3_power_trace/` — the completed sweep + supply-rail trace
  (`run_with_power.py`, `results.csv`, `power_window.csv`, `verdict.json`)

## Method notes worth keeping

Three self-inflicted measurement bugs, all of which produced confident wrong
output before being caught:

1. **Iteration-count guard instead of a time budget.** Polling a counter costs
   microseconds, so a few thousand spins elapse long before one 22 ms frame
   does. Every config returned `n=0` — and the code then labelled zero frames
   "CLEAN", because zero frames trivially have zero scrambled. A phase that
   observed nothing must report INVALID, never CLEAN.
2. **Busy-wait without yielding.** A bare `continue` in the poll loop starved
   the other tasks, including the watchdog kicker and the USB stack, and reset
   the board mid-run. `sleep_ms(1)` against a 22 ms frame period costs no
   resolution.
3. **Dedup on the wrong counter.** Deduping on `frame_count()` can only ever see
   half the frames, so it cannot distinguish a dropped frame from an unobserved
   one — which is precisely what this experiment must measure.
