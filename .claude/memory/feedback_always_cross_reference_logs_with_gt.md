---
name: always-cross-reference-logs-with-gt
description: "HARD RULE — when analyzing a SIM mission log/journal, ALWAYS cross-reference with cf2_gt.jsonl to determine the drone's real physical position vs what the code thinks."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d28bcfbd-5126-440f-a985-6fa6451f2400
---

**Operator-stated 2026-05-23 during s193 iter3 post-mortem**:
> "intodeauna cand analizam log comparam si cu GT sa vedem in
> realitate care era pozitia/altitudinea dronei."

**Why**: a flight log shows what the mission code BELIEVED + what cf2
EKF reported.  Both can be wildly wrong (PnP miscalculation, EKF
divergence, stale frame buffer, mirror-branch correspondence).  The
ONLY authoritative answer to "where was the drone REALLY" is the GT
recorded by host-side `gt_recorder.py` (anti-cheat exception — host
only, never injected into sentai_sim).

**Failure mode this prevents**: jumping to "camera bridge frozen"
conclusion from repeated identical frame CRCs, when actually the
drone had crashed and was stationary so the camera legitimately saw
the same scene.  Without GT, the symptom (identical frames) looks
identical to the cause (bridge starvation).  s193 iter3 burned 10
min of analysis on this misdiagnosis before operator corrected.

**How to apply** — when analyzing a SIM flight log:

1. Load `<iter_dir>/fr_current/gt.jsonl` first.
2. Align timestamps: bridge log uses `gz_sec`, mission journal uses
   host wall_t (e.g. `463649640`), FR events use sentai_sim-relative
   ts_ms.  Bridge sim_t ↔ GT gz_sec align directly.  Mission/FR
   require offset arithmetic.
3. For EACH suspected anomaly tick (sudden PnP value change, ekf
   freeze, status flip), look up the GT pose at the same sim_t.
4. State the real physical position in the analysis output.
5. THEN compare against what code believed.  Discrepancy → bug.

**Tool note**: GT is JSONL `{t_wall, t_unix, gz_sec, gz_nsec, x, y,
z, qx, qy, qz, qw, yaw_deg}` at typically 50-100 Hz.  Pre-built
analysis script lives at `sim/scripts/gt_recorder.py`; correlator
scripts should be added under `sim/scripts/correlate_log_gt.py`
when this becomes a regular workflow.
