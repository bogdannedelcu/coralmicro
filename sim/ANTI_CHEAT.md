# Anti-cheat rule — SentAI sensors must NOT consume Gazebo ground truth

Operator-stated 2026-05-17.

## The rule

> **`sentai_sim` is AIR-GAPPED from simulator ground truth.**
>
> SentAI senzorii pot fi alimentați DOAR din componente dedicate SentAI:
> imagini de la camera + telemetrie drona (CRTP LOG / radio).  NU din
> ground truth oferit de simulator.

Real hardware doesn't have ground truth.  If our SIM tests pass because
SentAI quietly reads `/world/.../dynamic_pose/info` (Gazebo truth) or
files containing simulator state, those tests don't mirror real HW.
The whole value of the SIM is to surface bugs that would show up on
hardware — that property is destroyed by truth leaks.

## What CAN use ground truth

Ground truth is allowed **on the host side, for post-mortem comparison
between SentAI's estimate and the simulator's reality** — e.g., a
verdict script computing `|sentai_pose - gz_truth_pose|` to measure
estimator error.  This is observation-only, downstream of the mission;
it never re-enters `sentai_sim`.

The boundary is:

```
   Gazebo truth → host verdict.py / logger                ✓ allowed
   Gazebo truth → sentai_sim (any path)                   ✗ FORBIDDEN
```

## Exception process

If a test genuinely cannot be designed without truth (e.g., a perception
algorithm under development needs a trusted pose label for a few
iterations), it requires **explicit operator approval** captured in
the test's README, and the test must be clearly marked legacy/mock
(not part of the SentAI flight-validation path).

## What's allowed

- `sentai.camera.grab_gray()` — reads frames from `gz_to_uds_bridge`,
  which subscribes to a Gazebo *camera topic* (`/downward_cam/image`
  by default).  This is the simulated equivalent of the OV5640 CSI.
- `sentai.servo.pose()` / `sentai.crazy.pose()` — reads drone telemetry
  via CRTP LOG (same wire format as real radio link).  The drone's
  internal EKF state is "drone telemetry" — what cf2/PX4 uses
  internally is the drone's problem, not SentAI's.
- `sentai.imu.*` / `sentai.flow.*` — derived from the camera + drone
  telemetry; never from Gazebo state.

## What's forbidden inside SentAI code

- Any `gz::transport::Node::Subscribe` for a non-camera topic in
  `sim/` or `examples/sentai_runtime/`.
- Reading `gz topic -e -t /world/...` output in mission scripts /
  diag drivers / verdict scripts that gate test pass.
- Pre-populating MP globals (e.g. `sentai.objects.add(...)` before
  takeoff with positions copied from the world file).
- Any file/socket whose payload is "simulator state by reference".

## What's enforced

### Architectural air-gap

`sentai_sim` (the SIM binary) is built WITHOUT linking any `gz-transport`
/ `gz-msgs` library and includes ZERO `<gz/...>` headers.  By
construction it cannot subscribe to Gazebo, period.  The only inbound
data paths to `sentai_sim` are:

- UDS `/tmp/sentai_cam.sock` ← `gz_to_uds_bridge` (camera frames, with
  SCM1 wire-format magic)
- UDP `127.0.0.1:19850` ← `cf2 SITL` (CRTP wire format, drone telemetry)

Architecturally, the air-gap is enforced by what is NOT linked in.

### Code-level seal in the two bridge binaries

The bridges (`sim/gazebo/gz_to_uds_bridge.cc` and the older
`sim/scripts/gz_to_camera_bridge.py`) are the only Gazebo subscribers
in our codebase.  Both allowlist camera topics by substring match
(`/image` or `_cam`) and hard-reject any other topic with a clear
error message + exit code 5:

```
[bridge] REJECTED topic 'X' — anti-cheat rule: SentAI sensors
         can only be fed by camera-class topics...
```

### Audit script `sim/scripts/audit_anti_cheat.sh`

Greps the working tree for forbidden patterns.  Run before any commit
that touches sensor plumbing.  CI gate (TODO: wire into
`build.sh` / pre-commit).

```bash
bash sim/scripts/audit_anti_cheat.sh
```

Exits 0 if clean; non-zero with location list if any leak found.

## Known legacy exceptions (NOT new cheats)

These are pre-existing PX4 mock experiments that explicitly used
ground truth to bootstrap PX4 sensor bring-up.  They are *labeled
legacy* and not part of the SentAI flight-validation path:

| Folder | What it does | Why it's exempted |
|---|---|---|
| `s101-s109_*` | gz topic → VPE/POSE forward to PX4 | PX4 sensor mock during s100-series bring-up; superseded by sentai.link.flow C forwarder |
| `sim/scripts/gz_pose_to_vision_estimate.py` | same | same |
| `sim/scripts/gz_pose_logger.py` | dumps ground truth as reference for offline analysis | observation only, not fed back to SentAI |

These scripts must NOT be invoked from any new test.  If you find
yourself reaching for `gz topic -e dynamic_pose`, that's a code-smell
that the test is going to cheat — design around it.

## Simulator-fidelity gap (separate from cheat)

Even with this rule enforced, cf2 SITL has its OWN ground-truth
shortcut inside its firmware (its EKF in SIM uses Gazebo truth +
sensor noise instead of relying purely on flow+IMU).  That makes
`sentai.servo.pose()` overly accurate in SIM compared to real HW.

This is a **simulator fidelity** issue, not a SentAI cheat.  SentAI
reads CRTP LOG faithfully; the drone happens to be too smart in SIM.

Mitigations (FutureWork):
- F-AC-1: cf2 SITL config to disable Gazebo-truth EKF shortcut, force
  flow+IMU-only mode.  This would make closure tests honest.
- F-AC-2: a parallel "honesty gate" that compares SentAI's anchor_pose
  (from camera flow) against cf2's reported pose; large divergence =
  suspicious cf2 cheating.
