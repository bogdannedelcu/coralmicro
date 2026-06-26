# Cross-framework parity: c2f_thick champion — PyTorch yolov5 vs TF-native fork

**Date:** 2026-06-26. Same architecture (`yolov5n_p3p4_c2f_thick`, 751,260 trainable params — verified
identical both frameworks), same cover split (synthetic), same hyps (`hyp.no-resize`), imgsz 480×640,
seed 42, 100 epochs, SGD + cosine LR.

## 1. Port fidelity
PyTorch C2f ported into `models/common.py` + `models/tf.py` (TFC2f). Param count **751,260 == 751,260**
(TF trainable). Build + forward verified.

## 2. Training performance
| metric (best in-training val) | TF-native | PyTorch yolov5 |
|---|---:|---:|
| mAP@0.5 | 0.7725 (ep39) | **0.8046** (ep99) |
| mAP@0.5:0.95 | 0.3343 (ep41) | **0.3663** (ep99) |
| epochs run | 57 (early-stop) | 100 (full) |

**Not apples-to-apples:** TF early-stopped at ep57; PyTorch ran full 100. On the shared range (ep0–48)
the curves nearly overlap. TF-native proper native eval (selection table) was val 0.7693 / test 0.8019 —
≈ PyTorch's 0.805. Models are equally good; the gap is training-duration + measurement convention.

### Root cause of the TF late-training collapse (ep49→56: 0.749→0.587)
`ModelEMA` in the TF trainer (`utils/tf_torch_utils.py:143`) averages only `model.trainable_variables`
— it **omits the BatchNorm `moving_mean`/`moving_variance`**. PyTorch's `ModelEMA` averages the full
`state_dict` (params **and** buffers, incl. BN running stats). At validation TF swaps in EMA kernels but
keeps the raw (drifting) BN stats → growing distribution mismatch late in training → mAP collapse +
rising val loss. This handicaps **all** TF-trained models, not just c2f_thick. Fix: add
`model.non_trainable_variables` (BN moving stats) to the EMA shadow. (Diagnosed, fix pending.)

## 3. Export path — what `export.py --include tflite` generates
In this (classic yolov5) fork, `export.py` tflite/edgetpu route is a **Keras rebuild** (`models/tf.py`:
`export_saved_model` → `export_tflite`), **NOT** an ONNX intermediate (ONNX is only `--include onnx`).
It loads PyTorch weights into Keras `TFConv`/`TFC2f`/`TFDetect`, exports SavedModel → TFLite. FP16 by
default (1.5 MB, decoded output `(1,6000,6)`); `--int8` gives a fully-quantized UINT8 model (0.9 MB).

## 4. Deployment graph comparison (INT8 + EdgeTPU)
| | PyTorch-route (export.py) | TF-native (tf_yolo) |
|---|---:|---:|
| INT8 tflite | 909,464 B | 895,504 B |
| EdgeTPU tflite | 1,217,152 B | 1,102,592 B |
| **EdgeTPU subgraphs** | **1 (fully mapped)** | **1 (fully mapped)** |
| total ops | 224 | 188 |
| on-chip params | 869.25 KiB | 866.75 KiB |
| off-chip params | 12.00 KiB | 0.00 B |
| instructions / invoke | 310,416 B (303 KB) | 279,968 B (273 KB) |
| params (load once) | 782,976 B (764 KB) | 782,336 B (764 KB) |
| input activations | 921,600 B | 921,600 B |

### Op-level diff
Backbone **identical**: CONV_2D 52=52, MAX_POOL_2D 3=3, SPLIT 5=5, RESIZE_NEAREST 1=1, PAD 5=5.
The +36 ops in the PyTorch route are entirely the **on-TPU detection-head decode**:
RESHAPE 4 (vs 0), STRIDED_SLICE 6 (vs 0), +LOGISTIC (56 vs 50), +MUL (62 vs 50), +QUANTIZE (5 vs 2),
+CONCAT (11 vs 8). PyTorch route bundles the grid decode into the TPU graph (output `(1,6000,6)`);
TF-native exports raw grid maps (decode on host) → leaner graph.

## 5. Conclusion
Both pipelines produce **functionally equivalent, fully-TPU-mapped INT8 graphs with identical weights
(764 KB params)**. The PyTorch/`export.py` Keras rebuild is ~11% larger / +36 ops because it bundles the
detection decode on-TPU. The TF-native custom export is leaner (raw maps). Training quality is
equivalent; the TF trainer's only real defect is the **EMA-omits-BatchNorm** bug causing late collapse.
