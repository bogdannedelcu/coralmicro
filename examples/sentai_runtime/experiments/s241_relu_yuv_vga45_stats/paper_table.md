# s241 — ReLU (RGB) vs YUV420 detector performance on VGA45 (build 1590)

On-board i.MX RT1176, single-EP EdgeTPU/USB2, camera 640×480@45fps (verified).
Isolated invoke: N=30 GPT1-µs-timed invokes. Pipeline FPS: M=3 trials × 6 s,
frame_count/wall, full camera→PXP→TPU pipeline live. mean±σ.

| arch | fmt | mAP@.5 | mAP@.5:.95 | invoke ms | pipeline FPS |
|---|---|--:|--:|--:|--:|
| msblock | RGB | 0.645 | 0.228 | 32.0±0.0 | 27.2±0.09 |
| c3 | RGB | 0.683 | 0.261 | 32.4±0.5 | 27.0±0.00 |
| c2f | RGB | 0.702 | 0.278 | 33.0±0.0 | 26.9±0.09 |
| c2f_pan2 | RGB | 0.707 | 0.276 | 33.7±0.5 | 26.4±0.00 |
| gelan | RGB | 0.690 | 0.266 | 34.0±0.0 | 26.4±0.00 |
| c2f_thick | RGB | 0.803 | 0.360 | 35.0±0.0 | 26.2±0.00 |
| gelan_pan2 | RGB | 0.661 | 0.255 | 35.0±0.0 | 25.4±0.00 |
| c2f_deep | RGB | 0.705 | 0.283 | 35.0±0.0 | unstable* |
| msblock | YUV | 0.522 | 0.158 | 19.0±0.0 | 42.8±0.09 |
| c3 | YUV | 0.700 | 0.258 | 19.0±0.0 | 42.6±0.09 |
| c2f | YUV | 0.720 | 0.275 | 19.4±0.5 | 42.3±0.09 |
| c2f_pan2 | YUV | 0.707 | 0.269 | 20.0±0.0 | 41.0±0.09 |
| gelan | YUV | 0.734 | 0.285 | 20.9±0.3 | 40.6±0.00 |
| c2f_thick | YUV | 0.823 | 0.368 | 21.9±0.3 | 40.5±0.00 |
| gelan_pan2 | YUV | 0.712 | 0.261 | 22.0±0.0 | 38.8±0.09 |
| c2f_deep | YUV | 0.736 | 0.282 | 22.0±0.0 | 38.9±0.09 |

\* c2f_deep RGB: pipeline unstable (scratch-spill 153600 B wedges the pipeline under camera+prep contention); isolated invoke 35.0±0.0 ms. Its YUV counterpart (smaller, no spill) runs fine at 38.9 fps.

## Per-architecture RGB → YUV (same weights, half the input transfer)

| arch | ΔmAP@.5 | RGB FPS | YUV FPS | FPS speedup |
|---|--:|--:|--:|--:|
| msblock | -0.123 | 27.2 | 42.8 | 1.57× |
| c3 | +0.017 | 27.0 | 42.6 | 1.58× |
| c2f | +0.018 | 26.9 | 42.3 | 1.57× |
| c2f_pan2 | +0.000 | 26.4 | 41.0 | 1.56× |
| gelan | +0.044 | 26.4 | 40.6 | 1.54× |
| c2f_thick | +0.020 | 26.2 | 40.5 | 1.55× |
| gelan_pan2 | +0.051 | 25.4 | 38.8 | 1.53× |
| c2f_deep | +0.031 | n/a | 38.9 | — |
