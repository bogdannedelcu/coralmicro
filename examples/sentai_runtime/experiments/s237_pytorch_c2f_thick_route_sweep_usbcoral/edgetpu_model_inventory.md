# EdgeTPU model inventory — MLflow `sentai_revision_cover_v1` (exp 5)

All runs that ship an **EdgeTPU-compiled** `.tflite` (9 of 17 runs; the other 8 are
training-only). Combines MLflow-logged graph metrics with measured throughput
(s236 = on-board i.MX RT1176 USB2; s237 = host Coral USB stick, USB3).

## 1. cover_v1 architecture sweep — TF-native, headless raw-map, SiLU (s236)

8 architectures, 1-class (person), 480×640×3 uint8 in, two raw heads out. Ranked by size.

| # | arch | int8 mAP@.5 | edgetpu KB | instr KB | ops | scratch | on-board FPS |
|--:|---|--:|--:|--:|--:|--:|--:|
| 1 | msblock | 0.589 | 265 | 139 | 125 | 0 | 22.9 |
| 2 | c2f | 0.678 | 329 | 152 | 134 | 0 | 26.2 |
| 3 | c3 | 0.668 | 325 | 155 | 144 | 0 | 26.2 |
| 4 | c2f_pan2 | 0.515 | 413 | 194 | 176 | 0 | 24.9 |
| 5 | gelan | 0.652 | 445 | 210 | 188 | 0 | 24.4 |
| 6 | c2f_deep | 0.565 | 585 | 277 | 242 | **153600** | 22.6 |
| 7 | gelan_pan2 | 0.480 | 561 | 272 | 246 | 0 | 22.8 |
| 8 | c2f_thick | 0.606 | 1077 | 273 | 188 | **153600** | crash* |

\* `c2f_thick` crashed the board pre-fix (the scratch spill, EXP-s236); runs after commit `8510b1b7`.

## 2. c2f_thick champion — PyTorch export, 3 routes (s237)

Same weights (751,260 params, **int8 mAP@.5 = 0.805** — trained full 100 ep, best of the sweep),
3 EdgeTPU export routes. Measured on the Coral USB stick.

| route | activation | edgetpu KB | instr KB | ops | scratch | stick FPS |
|---|---|--:|--:|--:|--:|--:|
| **B / relu_experiment** | **ReLU (fused)** | 1057 | **151** | **124** | **0** | **28.3** |
| A / export.py (main)    | SiLU              | 1217 | 310 | 224 | 153600 | 14.5 |
| C / onnx2tf             | SiLU + int8       | 1418 | 364 | 221 | 153600 | 12.5 |

ReLU cuts instructions **51%** (310→151 KB) and removes the scratch spill entirely.
(Ref: `native` TF-native route = 280 KB instr, matching s236's c2f_thick.)

## 3. Runs WITHOUT an EdgeTPU export (training-only, 8)

| run | date | note |
|---|---|---|
| cover c2f_p2       | 06-26 | new arch, 1-epoch smoke (e1) |
| cover c2f_spd      | 06-26 | new arch, 1-epoch smoke (e1) |
| cover c2f_thick ×2 | 06-25 | retraining runs |
| cover yolov5n_vanilla | 06-25 | P5 vanilla baseline |
| visdrone c2f_thick | 06-25 | different dataset |
| visdrone c2f_thick finetune | 06-25 | different dataset |
| visdrone yolov5n_vanilla | 06-25 | different dataset |

These have weights + saved_model but no `edgetpu/` compilation yet.
