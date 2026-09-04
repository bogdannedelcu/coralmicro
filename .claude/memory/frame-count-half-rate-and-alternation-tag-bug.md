---
name: frame-count-half-rate-and-alternation-tag-bug
description: "2026-09-04 (EXP-s246). frame_count() ticks at HALF sensor rate; ratio(1,1) therefore flips per TWO frames. Full-speed alternation is blocked by frame LABELLING, not image integrity."
metadata: 
  node_type: memory
  type: project
  originSessionId: d5a3f91c-f734-4950-82fe-b796eef65638
  modified: 2026-09-04T08:18:23.575Z
---

Two measured facts from EXP-s246 (`experiments/s246_alternation_full_speed/`,
build 1597) that change how dual-camera alternation should be reasoned about.

## 1. `frame_count()` is HALF the sensor rate — measured, not inferred

Over a 2 s window with the sensor at 45 fps:

- `sentai.camera.frame_count()` → **22.5 Hz**
- `sentai.camera.sensor_frames()` → **45.5 Hz**  ← added build 1597

`g_camera_frame_seq` is gated on `fb2_done` alone (`camera_support.c:432`). The
binding comment claiming "one tick = one actual sensor frame captured" was
wrong and has been corrected in the source. `[[csi-isr-is-timing-sensitive]]`
was right all along.

**Consequence:** the ratio-alternate scheduler keys its modulo off that counter,
so **`ratio(1,1)` flips the MUX once per TWO sensor frames**, not once per
frame. Use `sensor_frames()` whenever every frame matters — it is the only way
to tell a DROPPED frame from a merely unobserved one.

## 2. Full-speed alternation is a LABELLING problem, not a throughput one

`ratio(1,1)`, 120 frames per config, pattern oracle (cam0=BARS, cam1=HBAND,
`peek5_b40()` single-dequeue 5-row sampling):

| drain | dirty_skip_n | fps | SCRAMBLED | tag↔content mismatch |
|--:|--:|--:|--:|--:|
| 0 | 0 | **44.5** | 1/120 | **28/120 (23 %)** |
| 0 | 1 | 22.5 | 0 | 0 |

At full rate the **images are clean** — 1 internally-mixed frame in 120. What
breaks is **provenance**: 23 % of frames get attributed to the wrong camera.
So 44.5 fps alternation is achievable today; the blocker is the tag association
at MUX flip, which is a software problem (the same one `_t_pattern_31.py` was
written to investigate), not a hardware limit.

This also explains s243's 21.6 fps exactly: flip every two sensor frames,
`dirty_skip_n=1` discards the first of each pair → half the frames survive.

**Open:** the sweep beyond `drain=0,dirty=1` is unmeasured — the board resets
under sustained 45 fps alternation plus NAND writes (stable when idle, so it is
load-induced). `dirty_skip_n=2` makes `peek5_b40()` raise: every queued buffer
is marked dirty and the recovery loop runs out.

## Measurement traps hit here (all produced confident wrong output first)

- **Iteration-count guard instead of a time budget** — polling a counter costs
  µs, so thousands of spins elapse before one 22 ms frame. Every config returned
  `n=0`, and the code called zero frames "CLEAN" because zero frames trivially
  have zero scrambled. A phase that observed nothing must report INVALID.
- **Busy-wait without yielding** — a bare `continue` in the poll loop starved
  the watchdog kicker and the USB stack and reset the board mid-run. Yield with
  `sleep_ms(1)`; against a 22 ms frame period it costs no resolution.
- **Deduping on `frame_count()`** — can only ever see half the frames, so it
  cannot detect a drop, which is the whole point of the experiment.

See also [[camera-trigger-mode-does-not-exist]] (why the hardware-trigger route
is closed) and [[no-complex-serial-orchestration]] (the driver shape used here).
