---
name: PnP marker size + axis bugs FIXED (2026-05-11)
description: Two PnP bugs in s091 hover that masked algorithm correctness — marker size 28% off, X/Y centroid bias when markers leave FOV
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
s091 hover tests showed huge "drift" (40cm) under wind and EKF/PnP disagreement of 40cm.
Diagnosis revealed two PnP bugs (not flow algorithm bugs).

**Bug 1 — MARKER_SIZE_M = 0.08 should be 0.0625** (`s090_hover_over_cat/aruco_detector.py:46`)
ArUco texture aruco_4x4_50_idN.png is 512×512 with the actual black-border
pattern only 400×400 px (78.1% coverage). On a 0.08m box face, the marker
that solvePnP detects (outer black border) is only 0.08 × 0.781 = 0.0625m.
With MARKER_SIZE_M=0.08, solvePnP returns tvec[2] inflated by 1.28× →
PnP-z over-estimate of 25cm at z=1m.
Empirical verify: with fix, PnP-z error 25cm → 3-5cm.

**Bug 2 — PnP X/Y centroid bias and axis assumption** (`s091_aruco_lowalt/aruco_hover.py:355-361`)
Old code: `xs = mean([-m.tvec[0] for m in dets.values()])` — assumed
camera_X axis = body_X axis (wrong; camera is mounted pitch+π/2 yaw+π
per drone model.sdf.jinja), AND broke when some markers left FOV (the
"visible centroid" shifts, biasing the average).
Fix in `aruco_detector.py::estimate_drone_world_pose()` using:
- Per-marker tvec + KNOWN_POSITIONS_M[mid] independently
- R_camera_to_world from yaw_EKF + fixed _R_CAM_TO_BODY = [[0,-1,0],[-1,0,0],[0,0,-1]]
- Camera-CoM offset (-4cm, 0, -2cm) compensation
Empirical _R_CAM_TO_BODY verified via s092_axis_calib (controlled motion test).

**Why:** s091 test results were unreliable. With wind, EKF said drone at -0.42m X
but PnP said +0.01m X. Gz ground truth confirmed EKF was right (drone really did
drift -X). The whole "drift" diagnosis was bogus because PnP labels were wrong.

**How to apply:** Trust the fixed code. With no wind: 100% all-4, 9.6cm dist (was
8.6cm pre-fix). With wind: drift still ~40cm but it's REAL physical drift, not
PnP confusion. Future improvements should focus on gyro de-rotation or ArUco
PnP feedback to EKF, NOT on tuning flow fusion thresholds.
