# s193 iter1 — first full calib bringup post-T10 + 7-marker

**Hypothesis**: With (a) the T10 WhyCon detector and (b) the 7-marker
asymmetric pad, `sentai.calib.run_bringup()` reaches `DONE_OK` and
all acceptance thresholds (R<1°, offset<5mm, kp ∈ [0.30, 0.50],
hold rms<30mm, max<80mm).

**Change vs prior iter**: baseline (no prior iter in this experiment).
Predecessor: s187 (was blocked by detector recall + by 6-marker
symmetric PnP ambiguity).

**Date**: 2026-05-23

## Files captured in this folder

- `mission_s193_journal.txt` — per-tick mission log
- `mission_s193_summary.json` — pass/fail + bringup result fields
- `sentai_repl.log` — sentai_sim REPL output
- `fr_current/gt.jsonl` — host-side GT recorder JSONL
- `verdict.log` — human-readable verdict
- `verdict_s193.json` — structured verdict

## Result

(Filled in after running.)
