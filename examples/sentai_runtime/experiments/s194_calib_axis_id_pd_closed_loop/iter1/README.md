# s194 iter1 — closed-loop PD axis ID baseline

**Hypothesis**: Replacing s193 iter17/18's open-loop pulse+settle with
a closed-loop PD park (with symmetric ±δ targets and 150 ms zero-
attitude level-settle before each capture) yields:
- pitch_disp and roll_disp near-orthogonal (dot < 0.3)
- mag_p, mag_r ≥ 2 cm
- R_committed orthogonal → calib SAMPLE phase progresses past iter17/18

**Change vs s193 iter18 (open-loop)**:
- Discovery: exploratory 4°×250 ms pulse → measure Δp_pad → d_axis
- Per axis: PD-park at ±δ·d_axis (δ=6 cm) instead of fixed-time settle
- PD gains: Kp=10°/m, Kd=14°/(m/s) (ω_n ≈ 1.3 rad/s, ζ ≈ 0.95)
- 150 ms zero-attitude stream before each capture (level drone)
- Capture median over 0.4 s
- disp = (p_+ - p_-) / 2  (drift cancellation by symmetric diff)

**Date**: 2026-05-24

## Files captured in this folder

- `mission_s194_journal.txt` — mission FSM events + per-tick PnP/EKF
- `mission_s194_summary.json` — phase_reached + R + axis_id_ok + status
- `cf2_gt.jsonl` — host-side GT recorder (post-mortem only)
- `gz_to_uds_bridge.log`, `launch_hybrid.log`, `launch_sim.pids` — runtime
- `fr_current/scalars.csv`, `fr_current/events.csv` — FR per-tick logs
- `sentai_repl.log`, `verdict.log`, `verdict_s194.json`

## Result

(Filled after running iter1.)
