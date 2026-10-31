# s242 — detection pipeline + optical flow CONCURRENT (1 model), VGA45

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s242 · follows [s241](../s241_relu_yuv_vga45_stats/).

## What this is

The [s241](../s241_relu_yuv_vga45_stats/) table is detection-only. Here we add the
**optical-flow stack running concurrently** with detection, on the single champion
model **`c2f_thick_yuv420`** (best mAP + fastest of the sweep), VGA45, build 1590.

Flow consumes PrepTask's shared **FLOW_GRAY_80×60 Y slot** (`use_prep_slot=1`): ONE
camera read feeds BOTH detection and flow, and — since this is the YUV path — the Y
plane flow needs is already extracted. `slot_div=2` → flow processes every 2nd frame.

Question for the paper: **how much does concurrent flow cost detection throughput, is
it stable, and does flow stay valid?** (Prior concern: flow+detection can saturate the
M7 SEMC/OCRAM buses and reset the core — mem `flow-detection-semc-contention`.)

Conditions (M=4 trials × 8 s, mean±σ): detection-only · +flow(M7) · +flow(M4 offload).

## Results (c2f_thick_yuv420, VGA45, build 1590)

| mode | detection FPS | flow valid? | resets | notes |
|---|--:|:--:|--:|---|
| detect-only | **38.9 ± 0.0** | — | 0/4 | baseline |
| **detect + flow (M7)** | **31.7 ± 0.0** | ✅ conf=255 | **0/4** | 81 % of baseline (−19 %), stable |
| detect + flow (M4 offload) | — | — | — | `m4_start` fails to boot (not usable in 1590) |

- **Concurrent M7 flow is STABLE and cheap**: detection 38.9 → 31.7 fps (**−19 %**), **0
  resets across 4 trials**, and flow produces valid vectors every trial (confidence=255,
  dx/dy non-zero, e.g. (61,66), (78,−73)). One camera read feeds both.
- Flow runs at the design rate ≈ `det_fps / slot_div` ≈ **16 flow-fields/s** (slot_div=2);
  output validity confirmed (the `frame_seq` delta metric in the driver misparsed, but
  `flow.read()` returns live confidence=255 + moving dx/dy).
- **M4 offload not usable in build 1590** — `sentai.flow.m4_start()` fails to boot the M4
  worker (transient USB re-enum, board recovers). The M4 flow-offload (EXP-s240) remains
  experimental / OCRAM-handoff-limited; M7-local flow is the working path.

## Finding — updates the old "flow+detection resets M7" concern

On the **YUV path** with the shared 80×60 gray slot + `slot_div=2`, concurrent flow is
**stable** (0/4 resets) — contrary to the earlier RGB-era observation. Why: (i) the YUV
PrepTask already produces Y (no extra full-frame conversion), (ii) flow works on a tiny
80×60 slot, (iii) `slot_div=2` halves the flow cadence. The net M7/SEMC contention stays
within budget, so detection keeps ~32 fps + full-rate flow with zero instability. For a
navigation stack that needs both detection *and* ego-motion flow, this is the recommended
configuration; M4 offload is not required (and not currently working).

## Reproducing

```bash
# board: build 1590, c2f_thick_y.tflite (= c2f_thick_yuv420) present, camera VGA45
python3 run_s242.py     # detect-only | +flow(M7) | +flow(M4); writes results_flow_vga45.csv
```
