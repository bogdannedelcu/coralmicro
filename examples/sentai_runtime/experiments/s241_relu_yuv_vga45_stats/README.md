# s241 — ReLU (RGB) vs YUV420 detector performance on VGA45, with statistics

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s241 · supersedes [s239](../s239_relu_full_pipeline_fps/)
(build 1560, camera-regressed) — the **definitive** on-board pipeline benchmark after
the two speed-measurement bugs were fixed.

## What this is

On-board (i.MX RT1176, single-EP EdgeTPU over USB2) full camera→PXP→TPU detection
pipeline throughput for **16 detectors**, with proper statistics (mean ± σ) for a
paper table:

- **8 ReLU RGB detectors** — MLflow `sentai_revision_relu_v1` (exp 8), single-input
  480×640×3 uint8, `<arch>.tflite`.
- **8 YUV420 dual-input detectors** — MLflow `sentai_yuv420_v1` (exp 13), inputs
  Y[480,640,1] + UV[240,320,2] uint8, `<arch>_y.tflite`. Native YUV capture halves
  the per-frame EdgeTPU input transfer (921,600 → 460,800 B).

Firmware **build 1590**, camera **640×480 @ 45 fps** (registers + `camera.fps()`
verified — see [[camera-yuv420-native-capture]] / s240).

## Why it supersedes s239 (two tick-starvation bugs, now fixed)

The TPU invoke's USB transfers starve the FreeRTOS SysTick ISR, so `xTaskGetTickCount`
**under-counts during the invoke**. Two sites used it and were bogus before build 1590:

1. **Invoke timing** — reported ~11 ms for a real ~32 ms invoke (s239's "camera-bound
   ~27 fps" was an artifact; the pipeline is actually *invoke*-bound). Fixed → GPT1 µs.
2. **InferTask FPS throttle** (`s_target_fps=45`) — its tick-based elapsed read short,
   so it injected a spurious ~9 ms `vTaskDelay` every frame, capping everything at
   ~30 fps. Fixed → GPT1 µs. This is what unlocked YUV 30 → 40 fps (+35%).

(Both are the same class of bug; the DWT-cycle-based `tpu_cyc` per-phase counters were
*not* affected. See memory `tpu-invoke-timing-ticks-bug`.)

## Method (honest, GPT1 µs, statistical)

Per model: `desc_cache(0)`, `direct_tensor(1)`, `invokes_per_frame(1)`, `target_fps=45`.
- **Isolated invoke**: 5 warmup + **N=30** invokes; each `sentai.tpu.invoke()` returns
  real GPT1-µs ms. → mean / σ / min / max / median.
- **Pipeline FPS**: **M=3** trials × 6 s, `frame_count`/wall-clock with the full
  pipeline live (CameraTask + PrepTask + InferTask, CSI/PXP/USB/SEMC contention). → mean / σ.

Driver: `run_s241_all.py` (incremental CSV, retry on transient USB re-enum). Raw:
`results_all_vga45.csv`. Table: `paper_table.md` / `paper_table.tex`. mAP: pulled from
MLflow (float model, person/1-class) by `build_table.py`.

## Results (mean ± σ)

| arch | fmt | mAP@.5 | mAP@.5:.95 | invoke ms | pipeline FPS |
|---|---|--:|--:|--:|--:|
| msblock | RGB | 0.645 | 0.228 | 32.0±0.0 | 27.2±0.09 |
| c3 | RGB | 0.683 | 0.261 | 32.4±0.5 | 27.0±0.00 |
| c2f | RGB | 0.702 | 0.278 | 33.0±0.0 | 26.9±0.09 |
| c2f_pan2 | RGB | 0.707 | 0.276 | 33.7±0.5 | 26.4±0.00 |
| gelan | RGB | 0.690 | 0.266 | 34.0±0.0 | 26.4±0.00 |
| c2f_thick | RGB | 0.803 | 0.360 | 35.0±0.0 | 26.2±0.00 |
| gelan_pan2 | RGB | 0.661 | 0.255 | 35.0±0.0 | 25.4±0.00 |
| c2f_deep | RGB | 0.705 | 0.283 | 35.0±0.0 | unstable\* |
| msblock | YUV | 0.522 | 0.158 | 19.0±0.0 | 42.8±0.09 |
| c3 | YUV | 0.700 | 0.258 | 19.0±0.0 | 42.6±0.09 |
| c2f | YUV | 0.720 | 0.275 | 19.4±0.5 | 42.3±0.09 |
| c2f_pan2 | YUV | 0.707 | 0.269 | 20.0±0.0 | 41.0±0.09 |
| gelan | YUV | 0.734 | 0.285 | 20.9±0.3 | 40.6±0.00 |
| c2f_thick | YUV | 0.823 | 0.368 | 21.9±0.3 | 40.5±0.00 |
| gelan_pan2 | YUV | 0.712 | 0.261 | 22.0±0.0 | 38.8±0.09 |
| c2f_deep | YUV | 0.736 | 0.282 | 22.0±0.0 | 38.9±0.09 |

\* c2f_deep RGB: pipeline **unstable** — its 153,600-B host-scratch spill wedges the
pipeline under camera+prep bus contention (isolated invoke is fine: 35.0±0.0 ms). Its
YUV counterpart (smaller graph, no spill) runs stably at 38.9 fps.

## Per-architecture RGB → YUV (same weights, half the input)

| arch | ΔmAP@.5 | RGB FPS | YUV FPS | speedup |
|---|--:|--:|--:|--:|
| msblock | −0.123 | 27.2 | 42.8 | 1.57× |
| c3 | +0.017 | 27.0 | 42.6 | 1.58× |
| c2f | +0.018 | 26.9 | 42.3 | 1.57× |
| c2f_pan2 | +0.000 | 26.4 | 41.0 | 1.56× |
| gelan | +0.044 | 26.4 | 40.6 | 1.54× |
| c2f_thick | +0.020 | 26.2 | 40.5 | 1.55× |
| gelan_pan2 | +0.051 | 25.4 | 38.8 | 1.53× |
| c2f_deep | +0.031 | n/a | 38.9 | — |

## Findings

1. **YUV420 native capture is a ~1.5–1.6× on-board FPS win across every architecture**
   — the EdgeTPU invoke is **input-transfer-bound** over USB2, and YUV halves the input
   (460,800 vs 921,600 B). Invoke drops from ~32–35 ms (RGB) to ~19–22 ms (YUV);
   the model graph barely moves the invoke time (transfer dominates).
2. **YUV mostly preserves or improves mAP** (ΔmAP@.5 ≥ 0 for 7/8 archs), so the speedup
   is nearly free. **Exception: msblock** (−0.123) — the smallest backbone loses
   accuracy in the 2-plane input; not a good YUV candidate.
3. **Champion `c2f_thick_yuv420` wins on both axes**: best mAP of the set (0.823 vs the
   RGB champion's 0.803) **and** +55 % FPS (40.5 vs 26.2). This is the recommended
   production detector.
4. σ is tiny throughout (FPS ±0.0–0.09, invoke ±0–0.5 ms at 1-ms quantization) — the
   pipeline is highly deterministic once the throttle bug is removed.

## Reproducing

```bash
# board: build 1590, models uploaded via USB-MSC (<arch>.tflite + <arch>_y.tflite)
python3 run_s241_all.py           # 8 RGB + 8 YUV, writes results_all_vga45.csv
python3 build_table.py            # merges MLflow mAP -> paper_table.{md,tex}
```

Models downloaded from MLflow `https://mlflow.autonomous.ro:5443`
(`sentai_revision_relu_v1` exp 8, `sentai_yuv420_v1` exp 13); the edgetpu artifacts
are also cached under `models/` (c2f_thick RGB) and `models_yuv/` (all 8 YUV).
