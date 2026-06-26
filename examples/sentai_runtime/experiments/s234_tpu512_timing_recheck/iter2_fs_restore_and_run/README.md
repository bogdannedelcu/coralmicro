# s234 iter2 - restore FS and re-run unchanged `_t_timing.py`

**Hypothesis**: After an operator-approved user-FS format and restaging of the
original model/script, the unchanged `_t_timing.py` run can reproduce or refute
the historical Table 9 timing values.

**Change vs prior iter**: restore board user filesystem. The board-side
experiment driver itself remains unchanged.

**Date**: 2026-06-24

## Files Captured

- `lsblk_before.txt` - host block devices before MSC mode.
- `format_fs.log` - REPL transcript for `sentai.fs.format()`.
- `msc_enter.log` - REPL transcript for `sentai.usb.drive(1)`.
- `lsblk_msc.txt` - host block devices after MSC mode.
- `msc_copy.log` - mount/copy/unmount transcript.
- `msc_exit.log` - transcript for exiting MSC mode.
- `board_probe_after_restore.log` - REPL probe after FS restore.
- `run_t_timing.log` - unchanged `_t_timing.py` experiment transcript.
- `summary.json` - parsed headline values.

## Result

Completed.

Board/user-FS restore:

- `sentai.fs.format()` succeeded after operator confirmation.
- `/lib`, `/lib/diag`, and `/diags` were recreated.
- The board was switched to MSC mode and mounted by the host as
  `/run/media/bogdan/0000-0001`.
- The exact original model was copied to
  `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`.
- The exact original script was copied to `/lib/diag/_t_timing.py`.

Artifact identity:

- Board version: `SentAI v1.0 build 1375 (2026-05-19 11:56:14)`
- Script SHA256: `5bc6f29ae715675a6de3bd8110ee083d6d7b469a8efeddbe4bad2af5c04d03d7`
- Model SHA256: `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`
- Model size on board: `5591680` bytes
- Runtime input tensor: `uint8[1,512,512,3] (786432 bytes)`
- Bytes per invoke reported by `_t_timing.py`: input `811008` B,
  instructions `371664` B, output `10752` B

Pure TPU averages, 5 runs x 30 invokes:

| Metric | Avg | Stdev |
|---|---:|---:|
| total_ms | 14.980 | 0.258 |
| input_ms | 4.924 | 0.283 |
| params_ms | 0.090 | 0.000 |
| instructions_ms | 3.364 | 0.303 |
| output_ms | 0.628 | 0.076 |
| event_ms | 0.0228 | 0.0008 |

Pipeline averages, 5 runs x 5 s:

| Metric | Avg | Stdev |
|---|---:|---:|
| prep_fps | 32.36 | 5.36 |
| infer_fps | 32.12 | 5.38 |
| cam_ms | 0.50 | 0.07 |
| pxp_ms | 8.14 | 0.11 |
| wait_ms | 22.58 | 5.64 |
| total_prep_ms | 31.66 | 5.42 |
| invoke_ms | 31.56 | 5.63 |

Interpretation note for the reviewer thread:

This run confirms that the model/input are not smaller than 512x512x3. The
reported `input_ms` value is reproducible, but because it is below the simple
USB 2.0 full-payload lower-bound calculation, the firmware metric label needs
an implementation audit. Treat `input_ms` as a reproduced board-side DWT stage
measurement until we prove whether it is full bus wall-clock transfer time,
queued driver time, DMA-visible time, or a partially overlapped interval.
