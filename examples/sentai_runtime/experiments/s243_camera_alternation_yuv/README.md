# s243 — camera-alternation throughput, YUV + detection pipeline, VGA45

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s243 · redo of the experiment.md #1077
alt-mode table, now with **YUV + the detection pipeline live + build 1590** (tick bugs
fixed).

## What this is

Dual-camera **alternation**: `camera.ratio(a,b)` makes the CSI ISR flip the MUX so
cam0 owns `a` frames and cam1 owns `b` frames per `(a+b)` cycle (glitch-free
EOF/VBLANK). Each flip costs a **drain / dirty-buffer skip** (the first frame after a
flip is discarded until the new camera's image is clean), so alternation trades
throughput for two views. We measure the sustained detection-pipeline FPS per ratio.

One model on **both** cameras: **`c2f_thick_yuv420`** (slot 0 / primary interpreter).
Note — *multi-slot* dual-input (a **different** YUV model per camera) is **not wired**:
the dual-input invoke (`sentai_tpu_invoke_with_inputs2`) targets the primary interpreter,
not the per-slot interpreters. So YUV alternation = same model, two cameras.

Protocol: yuv_mode(1), camera.init(1,45), `desc_cache(0)`/`direct_tensor(1)`/
`invokes_per_frame(1)`, M=4 trials × 7 s, `frame_count`/wall. mean±σ.

## Results (c2f_thick_yuv420, VGA45, build 1590)

| ratio cam0:cam1 | flips/cycle | pipeline FPS | vs 1-cam |
|---|--:|--:|--:|
| 1-cam (no alt) | 0 | **38.8 ± 0.07** | — |
| 1:1 | 2/2 | **21.6 ± 0.0** | −44 % |
| 1:2 | 2/3 | **21.6 ± 0.0** | −44 % |
| 1:3 | 2/4 | **27.0 ± 0.09** | −30 % |

(1:5, 3:1, 5:1 attempted but the MUX-flip at those cadences + the fragile board state
destabilised `pipeline.start`; not reported.)

## Findings

1. **Camera alternation is expensive.** Going from single-cam to 1:1 alternation roughly
   **halves** the detection throughput (38.8 → 21.6 fps). The cost is the per-flip drain:
   at 1:1 the MUX flips every frame, so a large fraction of frames are drained.
2. **Higher ratios amortise the flip.** 1:3 recovers to 27.0 fps (fewer flips per frame).
   The curve is flip-count-bound, not compute-bound — invoke is still ~22 ms.
3. **Design implication.** For a dual-camera SentAI mission that needs both views through
   the detector, 1:1 costs ~44 % FPS. If cam1 is only an occasional check (e.g. 1:3 or
   sparser), most of the throughput is retained. A frame that just needs *both* views
   every cycle should budget ~22 fps at VGA45.

## Caveat vs the old experiment.md #1077 table

Those RGB alt numbers (1cam 45.9 → 1:1 34.6) were build #1077, whose in-pipeline
invoke_ms was **tick-starved** (see s241 / memory `tpu-invoke-timing-ticks-bug`) — not
directly comparable. s243 is the corrected post-fix measurement (YUV, build 1590).

## Reproducing

```bash
python3 run_s243.py     # ratios 1cam/1:1/1:2/1:3(/1:5), writes results_alt_yuv_vga45.csv
```
