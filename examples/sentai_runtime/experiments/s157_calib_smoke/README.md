# s157 — sentai.calib smoke test (OP-S6-W1-T5)

WBS: `OP-S6-W1` Stage 6 sub-WP 1 — `sentai.calib` MicroPython binding.
Validates the Kabsch 3D Procrustes implementation
(`sentai_calib.{h,cc}` + `bindings/modsentai_calib.c`) end-to-end inside
`sentai_sim` over the same MP API the on-board firmware will expose.

## What this proves

Pure-synthetic, no Gazebo, no flight, no camera — six unit-style gates
exercising the production code path:

| Gate | Behaviour | Pass criterion |
|---|---|---|
| T1 | Identity recovery (R_true = I3, n=8, noise-free) | `det_R > 0.999`, `mean_res < 0.01°`, `accepted = True` |
| T2 | ±5° tilt recovery (R_true = `Rz(5°)`, persisted = I3) | `drift_from_persisted ∈ [4.5°, 5.5°]`, `mean_res < 0.5°`, `accepted = True` |
| T3 | Drift gate, identical R as persisted | `drift_from_persisted < 0.01°` |
| T4 | Too-few-samples reject (n=2 < `SAMPLES_MIN=3`) | `accepted = False`, `reject_code = 5` (`REJ_TOO_FEW`) |
| T5 | Bad-input reject (NaN in `tvec_cam`) | `accepted = False`, `reject_code = 6` (`REJ_BAD_INPUT`) |
| T6 | Save / load round-trip on the SIM stdio fallback | `rc_save = True`, `rc_load = True`, `err_R < 1e-6`, `err_off < 1e-6` |

The driver lives at `build-sim/sentai_fs_root/test_calib_s157.py`
(per the Sim.md §10w "drop in `sentai_fs_root/`, import from REPL"
convention).  It composes its own samples with `math.sin/cos` — no
host-side numpy dependency — and prints one `[s157] Tn PASS/FAIL …`
line per gate plus a final `[s157] OVERALL PASS/FAIL`.

## How to run

```bash
bash examples/sentai_runtime/experiments/s157_calib_smoke/run.sh
```

The script:
1. Forces a SIM rebuild (`make sentai_sim` is idempotent + fast).
2. Cleans any stale `./cam_calib.json` (host stdio fallback path).
3. Pipes `import test_calib_s157` to `sentai_sim` on stdin.
4. Hands the captured log to `verdict.py` for the PASS gate.

PASS iff `[s157] OVERALL PASS` appears AND every `[s157] Tn` line
reports `PASS`.

## What this does NOT prove

- The runtime `sentai_calib_init()` boot path with FxUser (FileX) —
  the SIM falls back to host stdio.  ARM-side persistence is exercised
  separately at `OP-S6-W1-T6` (ARM build + size delta).
- End-to-end with real ArUco samples in flight — that is `EXP-s158`
  (`OP-S6-W1-T7`, deferred until T1-T6 lock).
- Noise robustness — only the noise-free path is tested here.  The
  host Python reference (`_shared/test_camera_calibration.py`) already
  validates a 5 mm noise budget; mirroring it on-board is part of
  `EXP-s158`.

## Cross-references

- `ideas/objects_plan/11_camera_calib.md` — §21 design + fault model.
- `ideas/wbs.md` — WBS scheme.
- `[[short-term-plan-2026-05-17]]` — calendar W1 = OP-S6-W1.
- `examples/sentai_runtime/sentai_calib.{h,cc}` — production module.
- `examples/sentai_runtime/_shared/camera_calibration.py` — host Python
  reference + golden unit tests.
