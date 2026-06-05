---
name: sentai.flow.mode("anchor") — platform-abstracted ArUco anchor (s112)
description: Same MP API on ARM + x86 SIM. ARM stub today (real M7 detector deferred), SIM via Python sidecar + UDS. Wire test PASS 2026-05-12.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
`sentai.flow.mode("anchor")` is alive on both targets with identical
dict fields.  Mirrors the existing `sentai_pxp_shim`/`sentai_fft_shim`
pattern.

**Contract**: `examples/sentai_runtime/sentai_aruco_shim.h`
(`sentai_aruco_init/detect/get_latest/shutdown` + `sentai_aruco_pose_t`).

**Impls**:
- ARM: `sentai_aruco_shim_arm.cc` — STUB (returns detected=0).  Real
  M7 detector deferred to s113+ using s111 building blocks
  (1.1 ms threshold, 3.0 ms edge, 37 µs rectify).  Placed in
  `.sdram_text` to avoid ITCM overflow.
- SIM: `sim/sentai_aruco_shim_sim.c` — `SOCK_DGRAM` UDS reader bound to
  `/tmp/sentai_aruco_pose_recv.sock`, drains-to-latest on every read.
- SIM publisher: `sim/scripts/aruco_pose_publisher.py` — Python sidecar
  using cv2.aruco + solvePnP (reuses `experiments/s090_hover_over_cat/
  aruco_detector.py`).

**Wire**: `struct.pack("<II BB H ffff II", ...)` = 36 B, magic
0x41524332 ('ARC2'), little-endian.

**Why**: drivers should be portable byte-for-byte between SIM
(rapid iteration, no hardware) and ARM (production target).

**How to apply**:
- Same pattern when adding a future cross-platform primitive: header
  contract, ARM impl + SIM impl + shared MP binding hand-ported into
  both `modsentai_*.c` files (don't `#include` between them — the
  state backends differ).  Regenerate QSTRs (`agent.md §6`) for any
  new `MP_QSTR_*`.

**Gotchas captured the hard way**:
- `get_latest()` MUST drain the UDS, not only `detect()` — first cut
  only drained in detect(), polls returned stale zeros despite live
  publisher.
- Any new ARM-side function not in an ISR-hot path should go into
  `.sdram_text` — adding stub code to ITCM overflows m_text.
- `.ocram_bss` section name is misleading codebase-wide — orphan-lands
  in DTCM not OCRAM.

**Smoke test pass**: `experiments/s112_x86_anchor_shim/test_anchor_wire.py`
sent (det=1, n=3, x=1.25, y=-0.5, z=1.75, seq=42) over UDS;
`sentai.flow.anchor_pose()` returned the identical dict.  SIM build
#107, 2026-05-12.

**Synth-frame test pass**: `test_anchor_synth_frame.py` synthesizes a
640×480 PPM with 4 cv2.aruco markers, feeds publisher in
`--frames-dir` mode, sim reads detected=True, n=4 via REPL.  Full
cv2.aruco detection path validated.

**Live Gazebo pass — BOTH paths, 2026-05-12**:
- `run_px4_live.sh` (PX4 SITL + Garden + x500_sentai): 45/60 anchor
  polls detected during OFFBOARD hover; peak z 2.45 m (target 1.5 m);
  bridge logged 399 ArUco detections.
- `run_cf2_live.sh` (CrazySim + Garden + crazyflie): 23/40 anchor
  polls detected during cflib MotionCommander hover; peak z 1.19 m
  (target 1.0 m); bridge logged 318 detections.

**Live-path key insight**: don't write a third gz consumer — extend
the proven `sim/scripts/aruco_to_vision_estimate.py` (already used by
s108/s109) with `--anchor-pub-uds` flag for dual-publish.  ~20 LoC
patch; backwards-compatible.
