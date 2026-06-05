---
name: sentai-sim-air-gapped-from-truth
description: sentai_sim is AIR-GAPPED from Gazebo ground truth. SentAI sensors are fed only by camera frames (via gz_to_uds_bridge) + drone telemetry (CRTP LOG). Ground truth may only be used host-side for post-mortem comparison in verdict scripts. Enforced via bridge topic allowlist + audit_anti_cheat.sh.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Rule** (operator-stated 2026-05-17): `sentai_sim` must NEVER consume
Gazebo ground-truth state.  Tests that pass because SentAI quietly reads
`/world/.../dynamic_pose/info` or files containing simulator state are
worthless — they don't mirror real HW, which is the whole point of the
SIM.

**Why**: real hardware has no ground truth.  If we accept truth-leaks
into sentai_sim, our PASS verdicts give us false confidence; HW bring-up
will surface bugs we should have caught in SIM.  Operator pushback when
I claimed s153/s154/s155 closures (2-6 cm) were good — those numbers
reflect cf2 SITL's overly-optimistic pose feedback, NOT SentAI flow
quality.

**What's allowed**:
- `sentai.camera.grab_gray()` — frames via `gz_to_uds_bridge` (allowlist
  enforces camera-class topics only: `/image` or `_cam` substring).
- `sentai.servo.pose()` / `sentai.crazy.pose()` — drone telemetry via
  CRTP LOG.  Same wire format on HW and SIM.
- Ground truth ON THE HOST in verdict scripts for post-mortem comparison
  (`|sentai_pose - gz_truth|` error metric) — observation only,
  never re-enters sentai_sim.

**What's forbidden inside sentai_sim / examples/sentai_runtime / libs**:
- `gz::transport::Node::Subscribe` for non-camera topics
- `gz topic -e -t /world/.../dynamic_pose/info` piped into mission code
- Pre-populating MP state (`sentai.objects.add`, etc.) with values
  copied from world files

**Enforcement**:
- `sim/gazebo/gz_to_uds_bridge.cc` allowlists topics by substring; hard
  reject + exit 5 otherwise
- `sim/scripts/audit_anti_cheat.sh` greps the tree for forbidden
  patterns; run before commits touching sensor plumbing
- `sim/ANTI_CHEAT.md` documents the rule + exception process
- `CLAUDE.md` carries a short summary near the top

**Exception process**: explicit operator approval recorded in the
test's README, and the test must be clearly labeled legacy/mock and
kept out of the SentAI flight-validation path.

**Known legacy exemptions** (NOT new patterns): PX4 mock experiments
`s100-s109` used `gz topic` → VPE forwarders during PX4 bring-up.
Documented in `sim/ANTI_CHEAT.md` as exempted; do not pattern after.

**Separate concern — cf2 SITL fidelity**: cf2 SITL's own EKF inside the
simulated firmware uses Gazebo truth as a sensor shortcut (its problem,
not SentAI's).  This means `sentai.servo.pose()` is overly accurate in
SIM compared to real HW.  Mitigations are FutureWork F-AC-1 (force cf2
SITL into flow+IMU-only mode) and F-AC-2 (parallel "honesty gate"
comparing SentAI anchor_pose vs cf2 pose, large divergence flags
suspicious cf2 cheating).

Related:
[[experiments-start-from-origin]] — physical respawn, also anti-cheat
[[sim-test-must-return-home]] — closure vs world origin
[[flowbaseline-canonical-config]] — separate gate for camera+flow pipeline
