# Iter08 - Camera Switch Latency And Pattern Integrity

Build: `SentAI v1.0 build 1544 (2026-06-24 11:07:59)`

Scope:

- sequential `select() -> to_tensor()` switch timing at 512x512;
- synthetic-pattern integrity check for `ratio(1,1)` using
  `sentai.camera.test_pattern()` plus `peek5_b40()`.

Timing result, first same-camera row dropped:

| drain | select mean | to_tensor mean | total mean | effective FPS |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 13.907 ms | 79.984 ms | 93.891 ms | 10.651 |
| 2 | 13.498 ms | 117.587 ms | 131.085 ms | 7.629 |

Pattern integrity result:

| drain | ratio | frames | correct | scrambled | wrong-tag | note |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 1 | 1:1 | 100 | 100 | 0 | 0 | valid synthetic-pattern run; timing perturbed by per-frame FS detail logging |
| 2 | 1:1 | 100 | 46 | 1 | 53 | valid synthetic-pattern run; unexpected mismatch, needs follow-up before using as a correctness claim |

Follow-up focused drain probe, no per-frame FS writes:

| drain | ratio | frames | correct | scrambled | wrong-tag | cam0 | cam1 | note |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 1:1 | 50 | 50 | 0 | 0 | 26 | 24 | clean |
| 2 | 1:1 | 50 | 0 | 25 | 25 | 50 | 0 | invalid for continuous 1:1 alternation |

The older E17 strategy was confirmed: correctness is checked with camera
content, not just timing. The relevant primitives are OV5640 test pattern
register `0x503D` via `sentai.camera.test_pattern(cam, mode)` and
single-buffer sampling via `sentai.camera.peek5_b40()`.

Interpretation: `drain=2` is not simply a slower, safer setting under
continuous `ratio(1,1)`. Since the ISR ratio scheduler flips the MUX every
frame, waiting for two post-switch frame-sequence ticks can cross another MUX
flip. The consumer then samples a buffer whose content no longer matches the
assumed tag/phase. `drain=1` matches the current FB2-gated/dirty-buffer
semantics for continuous alternation.

`camera_pattern_integrity_once.py` was later changed to RAM-buffer details, but
that run reset the board and produced only a header. It is recorded as invalid
in `pattern_integrity_summary.csv`.

Artifacts:

- `board_run_00_drain1.csv`
- `board_run_01_drain2.csv`
- `camera_switch_latency_summary.csv`
- `pattern_integrity_summary.csv`
- `run_drain1.log`
- `run_drain2.log`
- `pattern_ratio1_1_drain1.log`
- `pattern_ratio1_1_drain2.log`
- `drain_probe_ratio1_1_rerun.log`
- `pattern_ratio1_1_drain1_ram.log` (invalid reset/header-only follow-up)
