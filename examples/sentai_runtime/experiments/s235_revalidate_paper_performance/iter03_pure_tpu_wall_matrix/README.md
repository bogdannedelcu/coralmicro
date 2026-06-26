# s235 / iter03 - Pure TPU Wall-Time Matrix

Goal: rerun the paper's pure TPU optimization/chunk-size table with valid
board wall-clock timing.

The driver `pure_tpu_wall_one_chunk.py` measures exactly one chunk size per
board run.  For final paper rows, each chunk/optimization condition should be
run from a fresh flash or known-clean boot to avoid TPU cross-test
contamination.

Default first sanity condition:

- chunk: `36 KB`
- invokes: `50`
- warmup: `5`
- model: `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`

Board-side CSV:

`/diags/s235_b10_pure_tpu/pure_tpu_wall.csv`

Host-side logs and pulled CSV must stay in this iteration folder.

## Result

Firmware:

`SentAI v1.0 build 1543 (2026-06-24 10:48:29)`

Model:

`/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`

Input tensor reported by firmware:

`uint8[1,512,512,3]`, 786432 bytes

All rows used 50 measured invokes after 5 warmup invokes.  Each chunk
condition was run after a fresh persistent firmware flash of the same build.
The descriptor/instruction cache remained at the default OFF state.

| Chunk | Wall ms/invoke | FPS | Input wait ms/invoke | Instructions wait ms/invoke | Failures |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 33 KB | 41.547539 | 24.068813 | 24.603979 | 14.598960 | 0 |
| 36 KB | 41.428139 | 24.138182 | 24.508600 | 14.572780 | 0 |
| 64 KB | 41.001480 | 24.389364 | 24.244680 | 14.435160 | 0 |
| 128 KB | 40.813641 | 24.501614 | 24.151621 | 14.361740 | 0 |

Clean summary:

`pure_tpu_wall_matrix_summary.csv`

Board CSV pulled through REPL:

`board_csv_pure_tpu_wall.log`

## Interpretation

The original paper's standalone 512x512 TPU values around 73-75 FPS are
superseded.  With valid board wall-clock timing, this firmware/model
combination runs at about 24.1-24.5 FPS.  The URB wait breakdown explains why:
the input tensor transfer alone is about 24 ms/invoke, and instruction upload
adds about 14.4-14.6 ms/invoke when descriptor cache is OFF.

The old "33 KB is best" conclusion is not supported by this wall-clock rerun
on build 1543.  In the tested range, larger chunks reduce the number of URB
submissions and are slightly faster, but the difference is small compared with
the total transfer budget.

## Artifact Index

- `qstr_regen_chunk_timeout.log` - regenerated QSTRs for
  `sentai.tpu.chunk_size()` and `sentai.tpu.urb_timeout_ms()`.
- `build_chunk_timeout_binding.log` - ARM firmware build log.
- `readelf_ramfunc_itcm.log` - `.ramfunc` section check using local
  `readelf` because `arm-none-eabi-objdump` was not installed on this PC.
- `flash_build1543_venv.log` - persistent flash log for build 1543.
- `repl_verify_build1543_tpu_api_retry2.log` - REPL verification that the new
  TPU API exists and defaults to 36 KB / 200 ms.
- `msc_repair_script_copy.log` - mounted-drive copy of the board driver after
  REPL upload proved unreliable for this file.
- `run_pure_tpu_wall_chunk36_build1543.log` - 36 KB run.
- `run_pure_tpu_wall_chunk64_build1543_retry1.log` - 64 KB run.
- `run_pure_tpu_wall_chunk128_build1543_retry1.log` - 128 KB run.
- `run_pure_tpu_wall_chunk33_build1543_retry1.log` - 33 KB run.
