# s235 / iter04 - Descriptor Cache Validation

Goal: test the experimental TPU descriptor/instruction cache with valid
wall-clock timing and output validation.

The cache is OFF by default.  This iteration exposes a narrow runtime control
under `sentai.tpu`:

- `sentai.tpu.desc_cache([enabled])`
- `sentai.tpu.desc_cache_stats([reset])`

The board driver is:

`examples/sentai_runtime/diag/desc_cache_probe.py`

Validation requirements:

- run on a freshly flashed board;
- first measure `desc_cache=OFF`;
- enable cache, perform one prime invoke, then measure cache-hit invokes;
- compare output hash for every invoke against the OFF baseline;
- record URB bytes and descriptor-cache stats.

Expected useful result, if the assumption is valid: instruction bytes should
drop to zero in the measured ON phase while output hashes remain identical.
If hashes mismatch, failures occur, or the TPU wedges, the optimization is not
acceptable.

## Result

Firmware:

`SentAI v1.0 build 1544 (2026-06-24 11:07:59)`

Model:

`/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`

The OFF baseline was valid:

- 20/20 invokes succeeded.
- Output hash was stable: `0b4b28d1`.
- Descriptor-cache stats: `(enabled=0, sent_params=20, sent_ins=40,
  skip_params=0, skip_ins=0)`.
- URB wait per invoke: input `24.149300 ms`, instructions `14.359500 ms`.

The ON phase is invalid as a performance result:

- Prime invoke succeeded and matched the baseline hash.
- Measured cache-hit phase had `19` failures out of `20` invokes.
- Repeated firmware error: `E:0B61:0`, which maps to `SendInputs failed`.
- Descriptor-cache stats showed the intended skip path was active:
  `(enabled=1, sent_params=1, sent_ins=2, skip_params=0, skip_ins=19)`.
- URB stats showed broken traffic after the skip path:
  input had `1` timeout and `18` submit failures.

Therefore `desc_cache` is not acceptable.  It appears to skip instruction
uploads, but the TPU/protocol state is not valid for subsequent input sends.
The apparent `71.855858 FPS` in the ON row is a failure artifact caused by
fast failed invokes, not a throughput improvement.

## Artifacts

- `qstr_regen_desc_cache.log` - QSTR regeneration.
- `build_desc_cache.log` - build log for build 1544.
- `flash_build1544.log` - persistent flash log.
- `repl_verify_build1544_desc_cache_api_retry1.log` - REPL API check.
- `msc_copy_desc_cache_probe.log` - mounted-drive copy of the driver.
- `run_desc_cache_probe_build1544_retry1.log` - full run log.
- `board_csv_desc_cache_probe.log` - CSV pulled from board FileX.
