# EXP-s264 — SXGA camera at the pipeline's own rate: does the Edge TPU see every frame? (OP-S10-W24-T17)

**Board** 58eb, **builds** 1754–1755, **date** 2026-11-01, one camera, packed YUV420 capture
(s259) + OCRAM input ring (s262), SXGA YUV420 model.

**Question** (operator): the SXGA pipeline reaches ~15.6 fps with the camera at a true 30 fps,
i.e. every other frame is dropped.  If the camera itself runs at SXGA 15 fps, does the TPU
process every frame (lockstep)?  Frame slip matters because a dropped frame desynchronises the
two-camera alternation at a camera switch.

## Method

`run_lockstep.py`: per fps key a fresh boot, `camera.packed420(1)`, `camera.init(1, key)`, AEC
night mode off, CSI frame counter (`CR3.FRMCNT`) rate idle, SXGA model, `pipeline.ocram_ring(1)`,
a discarded warm-up run, then a **steady-state window**: with the pipeline running, the CSI frame
counter and the InferTask ok count are read in one REPL statement at t0 and again at t1.
processed / captured over that window is the lockstep ratio (the earlier start..stop window
under-counted by ~8 % because the start/stop transients and REPL round-trips sat inside it).
`trace_probe.py` dumps the pipeline trace ring in steady state (grab times, camera frame_seq,
invoke spans).

## Findings

**The stock "SXGA 15" sensor row is 11.2 fps** (sysdiv 4 on the 22.6 fps PLL), the same way
the "SXGA 30" row is 22.6 fps (s260).  A true-15 row was added: fps key **18** (= 15 × 1.2,
the same ratio as 36/30), PLL 0x70 with `pllCtrl1` 0x41 (sysdiv 4), `pclkPeriod` 0x10, D-PHY
settle 0x13 — SDK patch `0008-ov5640-sxga-true15-row.patch`, build 1755.  Measured 14.98 frames/s.

### Steady state, 30 s per key (`run_b1755_k15_18_36.out`)

| fps key | camera (CSI FRMCNT) | processed by the TPU | processed / captured | invoke ms | PrepTask idle (grab wait) ms | stale skips | fail |
|---|---|---|---|---|---|---|---|
| 15 (stock) | 11.25 /s | 11.25 /s (341 / 341) | **1.000** | 60.3 | 47.9 | 0 | 0 |
| **18 (true 15)** | 14.98 /s | 14.98 /s (454 / 454) | **1.000** | 60.0 | 25.5 | 0 | 0 |
| 36 (true 30) | 30.00 /s | 16.30 /s (494 / 909) | 0.543 | 61.0 | 1.0 | 0 | 0 |

### Soak and chase at key 18 (`soak_k18.out`, `chase_k18.out`)

| mode | window | camera | processed | ratio | invoke ms | fail / desync / usb_detach |
|---|---|---|---|---|---|---|
| ring, 120 s soak | 120.3 s | 1806 (15.01 /s) | 1805 (15.00 /s) | **0.999** (1 frame = the read skew between the two counters) | 60.0 | 0 / 0 / 0 |
| ring + `chase_prep(1)` | 30.3 s | 456 (15.05 /s) | 222 (7.33 /s) | 0.487 | 66.1 | **10 / 10 / 2**, ring consumer timeouts 3, producer timeouts 12 |

* The chase producer does NOT work with the slow sensor row: at 15 fps the CSI writes a frame in
  ~44 ms, so the ring slices arrive slower than the USB consumer's per-slice timeout and the
  driver's URB path (`take_failed` 5, `csr_timeouts` 1) breaks into recovery — half the frames
  lost.  `chase_prep` stays a 30 fps (key 36) knob; combining it with key 18 would need the
  ring consumer timeout scaled to the frame period.  Not pursued: at key 18 the lockstep ring
  already has the frame on the wire within one period.

* At key 18 the pipeline is in lockstep with the sensor: 454 frames captured, 454 processed,
  zero stale skips, zero failures (and 1805 of 1806 over the 120 s soak).  Margin per frame: 66.7 ms period − 60.0 ms invoke ≈ 6.7 ms
  on the InferTask side; PrepTask finishes producing at ~41 ms and idles ~25 ms waiting for the
  next frame.
* At key 36 the same pipeline processes 54 % of the frames (16.3 of 30): the grab path keeps the
  newest queued frame and drops the older one, so the drop pattern is "every other frame plus a
  slip" (s260: 28.6 vs 30 analysis).
* The trace at key 15 (`trace_probe_k15.out`) shows grabs alternating 86.7 / 91.7 ms (= the
  11.2 fps period), every frame followed by its invoke; the camera `frame_seq` advances once
  per two frames, the known half-rate counter, not a drop.

## Re-keying (build 1756, SDK patch 0007 rewritten)

Operator, after the first report: a key that does not mean the frame rate is confusing.  The
SXGA rows are now keyed by the measured rate: **15 = 15.0 fps** (was key 18), **30 = 30.0 fps**
(was key 36), **22 = 22.6 fps** (the old "30" row, kept only for the legacy 4 B/px capture;
`init(1, 30)` without `packed420(1)` returns -12 because that mode hung the board in s260); the
11.2 fps row is gone.  Everything above this section was measured with the old keys.

`rekey_check.py` (`rekey_check.out`): legacy init(1, 30) → -12, init(1, 36) → -10, legacy
init(1, 22) → 0 at 22.3 frames/s, packed init(1, 30) → 0 at 29.99 frames/s.
`run_b1756_rekey.out` (20 s steady state each, option B + ring):

| key | camera | processed | ratio | invoke ms |
|---|---|---|---|---|
| 15 | 15.02 /s | 14.97 /s | 0.997 (1 frame = read skew) | 60.0 |
| 22 | 22.51 /s | 16.11 /s | 0.716 | 62.0 |
| 30 | 30.00 /s | 16.30 /s | 0.544 | 61.0 |

## Verdict

Yes: with the camera at a true 15 fps (key 18) the SXGA option B + ring pipeline processes
**every** camera frame, 60 ms invoke inside a 66.7 ms period, 0 drops in the measured windows.
The stock key 15 (11.2 fps) is also lockstep but wastes 25 % of the TPU.  Key 36 (30 fps) gives
+9 % throughput (16.3 vs 15.0 fps) at the cost of non-deterministic frame slip.

## Files

`run_lockstep.py`, `trace_probe.py`, `results_b1754_k15_36.txt` (start..stop window, superseded),
`run_b1755_k15_18_36.out`, `soak_k18.out`, `chase_k18.out`, `trace_probe_k15.out`,
`results_*.txt`, `run_*.log`.  Firmware:
SDK patch 0008 (fsl_ov5640.c row key 18), settle row in `libs/camera/camera_support.c`, fps
allowlist in `sentai_cam_init_full` (`sentai_runtime.cc`).
