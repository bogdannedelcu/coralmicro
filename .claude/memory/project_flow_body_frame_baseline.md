---
name: Flow body-frame baseline (cam0 + vflip=1)
description: Hardware mount and image-axis convention for sentai.flow → drone EKF. Cam0 = USB-side, forward = cam0→cam1. With vflip=1 the cam0 buffer maps image LEFT→fwd, BOTTOM→left; body_xform[0] = (+1,0,0,-1).
type: project
originSessionId: c7a210f2-a5f7-4a53-86b4-b15f5f9be074
---
**Established 2026-05-07 by photo verification** (cam0 with `vflip=0` vs
`vflip=1`, calibration target visible in
`diag/_orientation_cam0_default.jpg` and `diag/_orientation_cam0_vflip.jpg`).

**Why:** Until this baseline existed, the body-frame mapping in
`_t_flow_to_drone.py` was only a guess from `paper/flow_body_frame.md`
(unverified on hardware). Drift on first flight would have been
unrecoverable without this calibration.

**How to apply:** Treat the mapping below as authoritative for cam0.
Cam1 is a placeholder copy — re-run `verify_orientation(cam_id=1)`
before trusting cam1 in flight. If the SentAI board is ever physically
re-mounted on a different drone or rotated 90° / 180° on the same
drone, this whole baseline must be re-derived (re-photo + re-verify).

**Hardware mount (board-on-Crazyflie):**
- cam0 = camera physically NEAREST the SentAI board's USB-C port
- cam1 = camera at the FAR end of the board
- Drone **forward (+body_x)** axis = direction from cam0 → cam1
- Body +y = LEFT, +z = UP (standard right-hand frame)

**Camera orientation chosen for FLOW:**
- `cam_hflip = 0` (no horizontal mirror)
- `cam_vflip = 1` (vertical flip ON; reg20=0x47, reg21=0x01 on both cams)

**With those settings, image-axis ↔ body-axis mapping (cam0):**
| Image position | Body direction |
|---|---|
| LEFT   | FORWARD  (+x) |
| RIGHT  | BACKWARD (-x) |
| TOP    | RIGHT    (-y) |
| BOTTOM | LEFT     (+y) |

**Phase-correlation sign convention** (sentai.flow.read()):
- `+dx` = features moved RIGHT in buffer
- `+dy` = features moved DOWN  in buffer

**Drone physical motion → flow output sign:**
- FORWARD  (+body_x) → features go to image RIGHT → `dx > 0`
- BACKWARD (-body_x) → `dx < 0`
- LEFT     (+body_y) → features go to image TOP   → `dy < 0`
- RIGHT    (-body_y) → `dy > 0`

**Body_xform encoded in driver** (`diag/_t_flow_to_drone.py::DEFAULTS`):
```python
body_xform = {
    0: (-1.0, 0.0, 0.0, +1.0),   # cam0 EMPIRICAL 2026-05-07: body_fw=-dx, body_left=+dy
    1: (-1.0, 0.0, 0.0, +1.0),   # PLACEHOLDER — verify cam1 before flight
}
```

**Why the sign flip (2026-05-07 controlled-translation test)**:
sentai's phase-correlation reports peak SHIFT in the OPPOSITE
direction of feature motion.  Theoretical model assumed +dx for
forward; measured -222 mgp dx during forward translation.  Trust
the data.  Per-phase displacements (180 grid-px integrated):

  drone FORWARD (+body_x) → flow dx ≈ -222 mgp
  drone BACK    (-body_x) → flow dx ≈ +364 mgp
  drone LEFT    (+body_y) → flow dy ≈ +233 mgp

Cross-talk negligible (other-axis component < 90 mgp), so axes are
correctly assigned per `paper/flow_altitude.md`; only signs were
flipped.  Validated visually with `diag/_flow_trajectory.png`
two-panel render (image-frame raw + body-frame corrected).

**FOV / focal length** (NOT readable from camera registers — lens
property): `fov_h_deg=58, fov_v_deg=45, focal_px_grid≈72.3` (from
`paper/flow_altitude.md`, empirical 2026-04-21).

All scale factors (grid-px → drone EKF dpixel units, accounting for
PMW3901 vs SentAI lens geometry) are derived in
`_t_flow_to_drone.py::_scale_to_drone_units()` — typical values are
`(scale_x, scale_y) ≈ (6.18, 6.39)`.

**Re-derivation procedure** if hardware changes:
1. `f.verify_orientation(cam_id=N, secs=20)`
2. Slide board over textured paper in 4 known directions
3. Build the (fw_from_dx, fw_from_dy, left_from_dx, left_from_dy)
   tuple that makes body_fw POSITIVE on physical FORWARD motion and
   body_left POSITIVE on physical LEFT motion
4. Pass to `f.run(..., body_xform={N: (...)})`
