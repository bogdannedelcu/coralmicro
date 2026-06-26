# s236 - Per-Model Speed Sweep (640x480 cover_v1 detectors)

Follow-up to `TD-S10-A10/B10` (paper Section 4.2 performance revalidation).
Where s235 revalidated the single 512x512 `yolo_1_class` detector, s236
benchmarks the **8 cover_v1 architecture-sweep detectors** trained for the
MDPI *Drones* revision (MLflow experiment `sentai_revision_cover_v1`,
exp_id=5), at the deployed input size **480x640 (HxW) = 640x480 image**.

## Models (MLflow exp 5, "cover" sweep, seed 42)

YOLOv5n p3p4, 1 class (person), input `uint8 [1,480,640,3]`, **headless**
export: two raw heads `[1,30,40,6]` (stride 16) + `[1,60,80,6]` (stride 8),
anchors P3=[16,16] / P4=[32,32]. EdgeTPU-compiled (single subgraph, fully
on-chip).

| arch | MLflow run_id | edgetpu .tflite |
|---|---|---|
| c2f_thick (champion) | 68e39c7696cd49bfbcf76476d71dffc6 | 1102592 B |
| c2f | 6107b4801a6a40bcbb6b718fb17a539f | 336640 B |
| c3 | 927d8206d4824bb5b63f81fac5601dd6 | 332544 B |
| gelan | b7b8b9e629f549e2a7842ea4429c95d2 | 455424 B |
| gelan_pan2 | f90c207695b34a6496365e9c1ee455a0 | 574208 B |
| c2f_pan2 | b8359134949b4022a768263efdac60af | 422656 B |
| c2f_deep | 9331ca3045214684ba474d858c0eb435 | 598784 B |
| msblock | 1788a6b70e654a40808489e82009a157 | 271104 B |

`vanilla` (ec7c2229...) has no INT8/EdgeTPU export (FP32-only off-the-shelf
baseline, retrain.md T4) -> excluded from the board sweep.

## Goal

Per-model wall-clock speed on the physical board, using the s235/B10
methodology (NOT DWT, NOT ticks): `sentai.rtos.micros()` for invoke wall
time + `sentai.tpu.urb_stats()` for USB transfer-phase breakdown
(instructions / input image / output readback). Reports, per model:

- standalone invoke wall time + throughput (invokes/s)
- USB phase breakdown: instruction transfer, input-image transfer, output
- pipeline frame throughput (camera -> tensor -> invoke)

## Decode policy (decided 2026-06-26)

The board's box decoder (`sentai_tpu_detect`, kV5Like/kV8) only handles a
single decoded output tensor; it returns 0 detections for these headless
2-head models. Decision: **board measures speed only**; bounding-box
correctness is validated **host-side** on the Coral USB Accelerator
(`edgetpu.tflite` + Python 2-head decode). On-board decode-to-boxes timing
is out of scope for this sweep (would need a with-head re-export or a new C
decoder; see AskUserQuestion log).

## Host environment

- **`venv-coral39`** (standalone CPython 3.9.18 + `pycoral` + `tflite_runtime
  2.5.0.post1` + libedgetpu 16.0) — the ONLY working host EdgeTPU stack on
  this box. The system python is now 3.12/3.14; both `ai-edge-litert` (invoke
  fails: unresolved custom op) and `tensorflow-cpu 2.17` (segfault) are
  ABI-incompatible with the apt `libedgetpu 16.0`. Fix: fetch a relocatable
  python3.9 (astral python-build-standalone) and install the legacy coral
  stack from `https://google-coral.github.io/py-repo/`. The Coral USB
  Accelerator can wedge in runtime state (`18d1:9302`, invoke fails); recover
  by physical replug (drops to DFU `1a6e:089a`, firmware re-uploads on open).
- `venv-coral312` (ai-edge-litert) is kept only for fast CPU-int8 model I/O
  introspection — it cannot drive the EdgeTPU.

## Layout

- `models/<arch>/{edgetpu,int8,config,architecture.json}` - MLflow artifacts
- `models/runs.tsv` - arch -> run_id map
- `host_decode.py` - YOLOv5 p3p4 2-head decoder (shared)
- `iterNNN_*/` - per-iteration scripts, logs, board CSVs, summaries

## Results (build 1546) - see iter03

Board standalone EdgeTPU invoke, 640x480, N=50, micros + URB phase breakdown:

| model | ms/invoke | fps | instr ms | input ms | output ms |
|---|--:|--:|--:|--:|--:|
| c3 | 38.1 | 26.2 | 4.5 | 25.6 | 6.1 |
| c2f | 38.1 | 26.2 | 4.4 | 25.6 | 6.2 |
| c2f_pan2 | 40.2 | 24.9 | 5.6 | 25.6 | 7.0 |
| gelan | 41.0 | 24.4 | 6.1 | 25.6 | 7.3 |
| gelan_pan2 | 43.8 | 22.8 | 7.8 | 25.7 | 8.4 |
| c2f_deep | 44.2 | 22.6 | 8.0 | 25.6 | 8.6 |
| msblock | 43.7* | 22.9* | 4.0 | 32.4* | 5.3 |
| **c2f_thick** | **FAIL (0B62)** | - | - | - | - |

\* msblock input is a first-boot warmup outlier; steady ~37 ms / ~27 fps.

7/8 run at 22.6-26.2 fps, USB-input-bound (921 KB image = ~63% of each invoke).

**c2f_thick (champion) LIMITATION (documented, operator decision 2026-06-26):**
fails to invoke on the board's single-EP EdgeTPU firmware (`0B62
SendInstructions`) despite running correctly on host pycoral (7/7 GT). Not an
on-chip overflow (866 KiB used / 6.25 MiB free). Treated as a board-firmware
incompatibility for this model; board sweep covers the other 7 architectures.
Firmware-side fix (multi-EP / transfer-buffer / recompile) deferred.

## Anti-cheat

Host-only inference on a synthetic test-split image; no Gazebo ground truth
involved. Board sweep uses fixed dataset image / camera tensor. N/A to
sentai_sim air-gap rule.
