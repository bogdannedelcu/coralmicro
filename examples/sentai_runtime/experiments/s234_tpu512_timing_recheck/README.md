# s234 - TPU 512 timing recheck

**WBS**: OP-S10-W11
**Started**: 2026-06-24
**Status**: active - reproduced original timing and added URB-level recheck

## Claim

Re-run the original `_t_timing.py` experiment unchanged on the connected
SentAI board to verify whether the previously reported 512x512 YOLO TPU
timing numbers reproduce on the new PC / current hardware setup.

This experiment intentionally does not modify the board-side timing driver.
The artifact under test is:

`examples/sentai_runtime/diag/_t_timing.py`

## Pass Criteria

- Board enumerates as NXP `1fc9:c0a1`.
- `sentai.version()` is captured before the run.
- `_t_timing.py` is staged unchanged and executed from `/lib/diag/_t_timing.py`.
- Host captures the complete REPL transcript through the final `=== done ===`.
- Output contains the original sections:
  - `=== PURE TPU variance`
  - `bytes/invoke:`
  - `=== PIPELINE variance`

## How To Run

`iter1` used the host-side upload runner and stopped before execution because
the board user FS was unavailable. `iter2_fs_restore_and_run` was restored by
operator-approved `sentai.fs.format()`, then the exact script/model were staged
through MSC and executed from the board FS.

## Files

- `run.sh` - wrapper that creates an iteration directory.
- `run_s234.py` - host-side runner used by iter1; upload was blocked by FS state.
- `iter1/` - captured outputs for the first re-run.
- `iter2_fs_restore_and_run/` - FS restore, MSC staging, and completed re-run.
- `iter3_source_audit/` - source-level audit of the contested `input_ms` metric.
- `iter4_urb_instrumentation/` - board firmware instrumentation for per-URB
  submit/callback/wait timing and MSC+REPL staging on the new PC.

## Iter Results

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | Re-run original `_t_timing.py` unchanged on board build captured at runtime | blocked before run: board FS unavailable, upload/model staging impossible | no |
| iter2_fs_restore_and_run | Restore FS, stage exact original model/script, run unchanged `_t_timing.py` | completed: pure TPU total 14.98 ms avg, input stage 4.924 ms avg, pipeline infer 32.12 fps avg | yes |
| iter3_source_audit | Inspect source path for input cache / measurement semantics | no input-tensor cache found; `input_ms` is driver-stage DWT and should not be called full USB bus wall-clock time until lower-level URB validation | yes |
| iter4_urb_instrumentation | Add per-URB stats and re-run board-side timing with same 512 model | completed on build #1542 with GPT wall-clock URB timing: total 41.514 ms/invoke, input wait 24.528 ms/invoke | yes |

## Result

`iter2_fs_restore_and_run` completed the re-run after restoring the board FS.
The connected board enumerated as NXP `1fc9:c0a1`, and `sentai.version()`
reported `SentAI v1.0 build 1375 (2026-05-19 11:56:14)`.

The exact staged artifacts were:

- Script: `/lib/diag/_t_timing.py`, SHA256 `5bc6f29ae715675a6de3bd8110ee083d6d7b469a8efeddbe4bad2af5c04d03d7`
- Model: `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`, SHA256 `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`
- Model size on board: `5591680` bytes
- Input tensor reported by the runtime: `uint8[1,512,512,3] (786432 bytes)`

Pure TPU averages from `run_t_timing.log`:

| Metric | Avg | Stdev |
|---|---:|---:|
| total_ms | 14.980 | 0.258 |
| input_ms | 4.924 | 0.283 |
| params_ms | 0.090 | 0.000 |
| instructions_ms | 3.364 | 0.303 |
| output_ms | 0.628 | 0.076 |
| event_ms | 0.0228 | 0.0008 |

Pipeline averages:

| Metric | Avg | Stdev |
|---|---:|---:|
| prep_fps | 32.36 | 5.36 |
| infer_fps | 32.12 | 5.38 |
| pxp_ms | 8.14 | 0.11 |
| total_prep_ms | 31.66 | 5.42 |
| invoke_ms | 31.56 | 5.63 |

The re-run confirms that the contested value is reproducible with this
firmware/script/model combination. It also confirms the reviewer-relevant
detail that the input tensor is indeed 512x512x3 uint8. Therefore the next
paper-side action is not to reinterpret the experiment as a smaller input; it
is to audit what the firmware labels as `input_ms` and whether that field is
device-side driver/queue/DWT time rather than full USB bus wall-clock transfer.

The source audit in `iter3_source_audit/` rules out a simple repeated-input
cache explanation in the visible SentAI path: `SendInputs(...)` is still called
for `BASE_ADDRESS_INPUT_ACTIVATION`; the explicit caches apply to parameters
and instructions, not input activations. The measured rate implied by
`811008` bytes in `4.924 ms` is about `1318 Mbps`, which is physically above
USB 2.0 high-speed. Treat the value as a reproduced driver-stage metric until
per-URB submit/callback timestamps or external USB capture validate the exact
semantics.

## Iter4 Result

`iter4_urb_instrumentation` added `sentai.tpu.urb_stats()` on the board and
re-ran the TPU-only 512x512 path from the board REPL. The host only launched
the board script and captured serial output; timing and inference ran on the
board.

Firmware / staging notes:

- Valid run firmware: `SentAI v1.0 build 1540 (2026-06-24 10:05:33)`.
- The new PC did not preserve the DTC-RAM/noinit storage latch across the
  `sentai.usb.drive(1)` reset, but the SRC GPR copies did survive. The boot
  latch now accepts either SRAM/noinit or GPR magic for storage entry.
- Storage mode now keeps MSC and REPL active simultaneously. Exit is explicit:
  unmount/eject on the host, then run `sentai.usb.drive(0)` from REPL. There
  is no `q`/arbitrary-byte CDC exit path.
- The model was staged through the mounted MSC drive, not HTTP:
  `/run/media/bogdan/0000-0001` -> `/dev/sdc`, `FLASH STORAGE`.

Staged artifacts:

- Model: `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`,
  size `5591680`, SHA256
  `49942d36335f1e09ee10d29b967bf413c8d449a3261d9d7bb9cb1d51a8a73e4f`.
- Script: `/lib/diag/_t_timing_urb.py`, size `2297`, SHA256
  `f8935884c5e0ae9f8f078b6554a718d72ac14599cf8926e5f01aa0629be7fb67`.

Initial board-side DWT run summary from `run_t_timing_urb_1540_retry1.log`:

| Metric | Avg per invoke |
|---|---:|
| invoke wall time from `ticks_ms` | 14.813 ms |
| input bytes requested | 811024 B |
| input submit time | 0.308 ms |
| input callback span | 4.752 ms |
| input wait span | 4.815 ms |
| instruction bytes requested | 371680 B |
| instruction wait span | 3.417 ms |
| output bytes requested | 10752 B |
| output wait span | 0.622 ms |
| event wait span | 0.025 ms |

This reproduced the contested small `input` duration with DWT-based URB
counters, but a follow-up calibration showed why this was still not a valid
USB wall-clock measurement. `DWT->CYCCNT` and `sentai.rtos.ticks_ms()` can
under-count while the core blocks/idles in the USB path. Build #1542 added
GPT1 `TimerMicros()` wall-clock timestamps directly to each URB submit,
callback, and wait interval.

Final board-side wall-clock run from `run_urb_wall_100invokes_1542.log`:

| Metric | Avg per invoke |
|---|---:|
| invoke wall time from GPT micros | 41.514 ms |
| host wall sanity check | ~42.10 ms |
| input bytes requested | 811024 B |
| input wait span | 24.528 ms |
| instruction bytes requested | 371680 B |
| instruction wait span | 14.579 ms |
| parameter wait span | 0.137 ms |
| output wait span | 1.704 ms |
| event wait span | 0.038 ms |
| URB wait sum | 40.986 ms |
| residual non-URB time | ~0.528 ms |

This resolves the reviewer concern: the earlier `4.40-4.92 ms` input value
was not a physical USB 2.0 transfer time. The corrected wall-clock input
transfer/wait is about `24.5 ms` for the 512x512 tensor path. The paper tables
and any throughput claims derived from the DWT/tick-only standalone benchmark
must be updated to the GPT wall-clock measurements.

## Iter1 Blocker

Before the restore, the board user filesystem was not usable:

- `sentai.fs.ls("/")` raised `OSError: dir not found`
- `sentai.fs.mkdir("/lib")` returned `False`
- `sentai.fs.write(...)` / `sentai.fs.append(...)` returned `False`

This was fixed in iter2 using operator-approved `sentai.fs.format()` followed
by MSC staging of the exact model and script.
