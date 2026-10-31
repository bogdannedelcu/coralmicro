# s244 — on-board manuscript-gap measurements (build 1590, VGA native)

EXP-s244. Covers 3 manuscript gaps, re-measured on build 1590 (GPT1-µs), native VGA
640×480@45. Full write-up with the 3 tables + honest failure notes:

**→ `review/reports/onboard_gaps_build1590.md`**

## Files
- `run_task1_multipatch.py` + `task1_multipatch.csv` — Task 1, multi-patch
  (`invokes_per_frame` N=1..4), RGB+YUV → Table 12. COMPLETE.
- `run_task2_dualcam.py` + `task2_throughput.csv` / `task2_periods.csv` /
  `task2_validation.csv` — Task 2, 1:1 dual-camera alternation drain=1 vs 2 →
  Tables 13/15 COMPLETE, Table 14 PARTIAL (per-frame tag breakdown not measurable).
- `cam0_pattern.png`, `cam1_pattern.png` — Task 3, distinct per-camera OV5640 patterns
  (cam0 BLACK/dark scene vs cam1 grayscale color-bar). COMPLETE.
- `seam_artifact.png`, `drain1_clean.png` — Task 3 seam A/B: NOT capturable from REPL
  (sub-frame-period artifact; latency > 1 frame). See report.

## Headline numbers (measured, mean±σ)
- Multi-patch: invoke throughput constant (RGB ~26.5 inv/s, YUV ~41 inv/s); frame_tp = invoke_tp/N; 0 timeouts/fails.
- Dual-cam 1:1 drain=1: **22.5 fps combined** (~11.3/cam), RGB & YUV alike (flip-bound); drain=2 halves it.
- Manual switch (select→to_tensor, drain=1): **~75 ms**.
