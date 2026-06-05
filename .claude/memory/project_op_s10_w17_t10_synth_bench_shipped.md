---
name: op-s10-w17-t10-synth-bench-shipped
description: OP-S10-W17-T10 SHIPPED 2026-05-23 — air-gapped Gazebo synth WhyCon ablation bench + OpenCV-parity port of sentai_aruco.cc.  Commit b4eb37ce.
metadata: 
  node_type: memory
  type: project
  originSessionId: d28bcfbd-5126-440f-a985-6fa6451f2400
---

OP-S10-W17-T10 SHIPPED 2026-05-23 (commit `b4eb37ce`).

**What landed:**

- `todo/TD-S10-A1/A2` stable specs (dataset format + validation
  methodology) + `B1/B2` implementation logs with iter-by-iter
  dead-ends.
- `sim/scripts/generate_whycon_synthetic_dataset.py` — Gazebo as
  renderer only (no cf2, no flight), 320×240 P5 PGM, manifest with
  GT projected centers + bbox + visibility classes, 7-marker
  asymmetric pad injected only in run-local world copy.
- `sim/scripts/validate_whycon_synthetic_dataset.py` — 2 OpenCV
  variants (`paper`, `edge_partial`), reproj-gated pose, IPPE +
  correspondence permutation ambiguity diagnostic, 16 report tables.
- `sim/scripts/run_sentai_sim_whycon_dataset.py` — drives
  `sentai.markers` through `sentai_sim` REPL + `sentai.fs`, no
  flight sim.
- `sentai_aruco.cc` rewrite: OpenCV-parity WhyCon (threshold sweep
  100/130/150, 8-CC, bbox/moment-centered dot pairing, dedupe, W3
  retired, component cap 96→240).
- New `SentaiMarkersDetection` ABI in `sentai_markers.{cc,h}` +
  bindings `sentai.markers.detect_pgm` / `get_detection_tuple` /
  `set_marker_world` (`bindings/modsentai_markers.c`).  QSTRs
  regenerated.

**Canonical 368-frame ablation** (sentai_sim build #558+):

```text
OpenCV  recall 1.0000, FP/frame 0.005, complete 0.995, centroid p95 1.51 px
sentai  recall 0.9982, FP/frame 0.052, complete 0.938, centroid p95 1.51 px
centroid Δ p95 0.36 px (sub-pixel parity)
sentai pose tr RMSE 16 mm, p95 29 mm, yaw p95 0.22°
```

OpenCV pose RMSE 0.128 m is inflated by ~6 mirrored-branch outliers
on 4-marker weak-geometry frames at z=1.0 m; reprojection gate
cannot reject all of them (some bad branches reproj < 0.5 px).
sentai_sim production drone-pose is more robust because
`get_drone_pose_tuple` uses prior-guided correspondence (perm +
Kabsch + yaw-anchor mirror picker per
[[feedback-yaw-anchor-mirror-picker]]).

Reference dataset folder kept in git (smaller smoke runs gitignored):

- `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/`
  (PGMs + manifest + config + render_world.sdf)
- `dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/
  validation_20260523_141003/` (summary.json + results.jsonl +
  failures.csv + report_tables/*.csv,md)

**Closes:** W17-T9 (BUG #74 synth detect returns 0 post-T18).
Parity port fixed the underlying CC pairing semantic.

**Drives next:**

1. Rerun s191 (`OP-S10-W21-T12/T13`) against committed detector.
   Open Q: does T13 iter-21d "33/44 match (75%)" go to ~98% now?
2. ARM build verification per [[sim-arm-parity-check]] — SIM build
   at #558+ uncommitted; arm-builder should confirm no m_text /
   ITCM regression.
3. Integrate prior-guided assignment into `get_drone_pose_tuple`
   as a regression test against this same bench (pose-side blocker
   is now the real bottleneck, not detection recall).
4. Optionally port `edge_partial` cropped-marker variant to
   sentai_sim if anchor-forward (heavy FOV crop) needs it.

**Anti-cheat status:** clean per
[[feedback-sentai-sim-air-gapped-from-truth]].  Frames are
camera-only renders; manifest GT only host-side; upstream
`sentai_whycon_small.sdf` unmodified.

Related: [[op-s10-w17-whycon]], [[op-s10-w21-t4-session2-open]],
[[feedback-experiments-in-own-folder-log-dead-ends]].
