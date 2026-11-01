# EXP-s263 — PrepTask repacks the frame while the CSI writes it (OP-S10-W24-T15 step 2)

**Board** 58eb, **builds** 1753–1754, **date** 2026-11-01, one camera, SXGA true-30 row
(fps key 36), packed YUV420 capture (s259), OCRAM input ring (s262).

**Question** (operator): start the repack when the frame is only partly written instead
of waiting for the complete frame, so the TPU input is on the wire sooner.

## Method

`sentai.pipeline.chase_prep(1)`: with the ring on, PrepTask hands the completed frame
from `GetRawFrame()` straight back, identifies the buffer the CSI is writing right now
(s261 mechanics: DMASA_FB1/FB2 candidates, padding-byte markers with D-cache
maintenance; `sentai_cam_chase_begin/wait_slice/end` in `sentai_runtime.cc`), and
produces each 128-row slice into the ring as soon as the CSI has written it.  The
last slice is partial (960 = 7.5 × 128).  Latencies from the pipeline trace ring
(`run_chase_prep.py`): kind 7 = chase start, 8 = slice written, 3 = complete frame
grabbed, 13 = TPU input fully transferred.

## Results (`run_b1754b.log`, 15 s each)

| mode | pipeline fps | invoke ms | last line written → TPU input done | notes |
|---|---|---|---|---|
| ring, normal (complete frame) | 15.9 | 61.0 | **75.4 ms** (71.6–76.4) | consumer waited 1 ms total |
| **ring + chase** | 13.2 | 66.0 | **20.4 ms** (slice 0 written → done 51.3 ms) | chase ok 350/350 frames, 0 slice timeouts; consumer waited 954 ms total = 4.4 ms per frame |

* Latency: the TPU has the whole frame **55 ms earlier** (−73 %) relative to the last
  line of the frame.  In normal mode the frame waits for the previous frame's stream
  (one frame in flight), then 8 PXP slices + USB; in chase mode slices stream while the
  frame is being written and only the last slice's PXP + USB remain after the last line.
* Throughput: fps 15.9 → 13.2 (−17 %), invoke 61 → 66 ms.  The pipeline is USB-bound,
  and in chase mode the USB consumer is paced by the camera for the first slices (the
  producer cannot be ahead of the CSI), so the TPU intake idles ~4.4 ms per frame.  With
  the single-frame ring the two goals conflict: low latency (chase) or maximum fps
  (stream complete frames).  A two-frame ring (or chasing frame N+1 while frame N
  finishes its transfer) would recover the fps; OCRAM for 2 × 737 KB is not available.
* Zero failures, zero chase timeouts; the in-flight buffer was identified on every
  frame right after returning the completed one (we are inside the vertical blank).

## Verdict

Step 2 works and is a knob (`pipeline.chase_prep(1)`, default off): −55 ms frame-to-TPU
latency for −17 % fps at SXGA.  For the detection pipeline (fps-bound) keep it off;
for latency-critical control loops (servo on a single camera) turn it on.

## Files

`run_chase_prep.py`, `run_b1753.log`, `run_b1754.log`, `run_b1754b.log`.  Firmware:
`sentai_cam_chase_begin/wait_slice/end/stats` (`sentai_runtime.cc`), PrepTask chase
branch (`detection_task.cc`), bindings `pipeline.chase_prep`, `pipeline.chase_stats`.
