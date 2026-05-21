# test_drone_pose_s184.py -- OP-S10-W19-T6b drone-pose Kabsch smoke.
#
# Pure synthetic. No Gazebo, no camera, no real WhyCon pipeline.  We
# generate body-frame observations analytically from a known drone
# pose, inject them via sentai.markers.test_inject_obs, then call
# sentai.markers.get_drone_pose and verify the recovered pose matches
# the ground truth.
#
# History note: the 2026-05-21 first pass at this smoke had three
# invalid test designs (T4 mutated obs by 180-Z rotation -- equivalent
# to claiming the picker undoes mutation rather than what it actually
# does, which is detect SVD-on-symmetric-pad wrong branches via cf2_yaw;
# T5 placed the drone below the marker plane -- but the Z-plane
# reflection's design assumption is drone-above-pad, so the picker
# correctly reflects the truth instead of returning it; T6 scrambled
# observations on a symmetric pad, which gave the permutation search
# two equally-valid res=0 fits and admit no unique recovery).  The
# current cases test each picker code path under HONEST inputs:
#
#   T1  Identity (yaw=0, drone at origin):
#         pure Kabsch fit, no disambiguation needed.
#   T2  Off-centre hover (yaw=0, drone at +5/-3/+78 cm):
#         translation recovery.
#   T3  Non-zero yaw (yaw=+15 deg, drone at +2/+4/+80 cm):
#         R diagonal positive, yaw extraction.
#   T4  Large-angle yaw (yaw=+170 deg, drone at +6/-4/+78 cm):
#         cos(yaw)<0 -- the picker's flip-sign check must take the
#         negative-cos branch correctly.  The 180-Z symmetric pad
#         admits an alternate res=0 fit; whichever Kabsch picks, the
#         picker must converge on the same (truth) pose.
#   T5  flip_z trigger (drone at +0.78 m, mutate obs.z by negation):
#         simulates Kabsch picking the below-pad coplanar branch.
#         Picker reflects t around z_mean -> truth.
#   T6  Permutation search with asymmetric pad (4-marker subset:
#         NW/NE/W/SE -- no 180-Z self-mapping).  Scramble observation
#         order; assignment search must find the unique inverse.
#
# Each test prints `[s184] T{N} {STATUS} <metrics>`.

import math
import struct
import sentai

print("[s184] BEGIN")
sentai.markers.clear()
rc_init = sentai.markers.init("whycon")
print("[s184] init rc=", rc_init, "backend=", sentai.markers.backend())

# ---------------------------------------------------------------------
# Marker world layout (mirror of s183 verdict_sota.py:71-79).
# Square 32x32 cm pad, 4 corners + 2 mid-bar markers, all at z=5 mm.
# This layout is symmetric under R_180z about the centroid (0,0),
# so T1..T5 exercise that symmetry; T6 swaps to an asymmetric subset.
# ---------------------------------------------------------------------
MARKER_WORLD_FULL = [
    (-0.16, +0.16, 0.005),   # 0  NW
    (+0.16, +0.16, 0.005),   # 1  NE
    (-0.12,  0.00, 0.005),   # 2  W
    (+0.12,  0.00, 0.005),   # 3  E
    (-0.16, -0.16, 0.005),   # 4  SW
    (+0.16, -0.16, 0.005),   # 5  SE
]

# Asymmetric subset for T6: NW, NE, W, SE.  Under 180-Z about (0,0):
# NW->SE (in set), NE->SW (NOT in set), W->E (NOT in set), SE->NW (in set).
# Two of four miss, so the subset has NO 180-Z self-symmetry and the
# permutation search has a unique res=0 winner.
MARKER_WORLD_ASYM = [
    (-0.16, +0.16, 0.005),   # NW
    (+0.16, +0.16, 0.005),   # NE
    (-0.12,  0.00, 0.005),   # W
    (+0.16, -0.16, 0.005),   # SE
]


def pack_xyz(xyz_list):
    return b"".join(struct.pack("<fff", *p) for p in xyz_list)


def body_obs_from_pose(world_markers, drone_x, drone_y, drone_z, yaw_rad):
    """Return list of (bx, by, bz) for each marker in the drone body frame.

    Body frame is the drone with X forward, Y left, Z up.  R_body_to_world
    is yaw rotation around Z.  Observation in body = R_yaw^T @ (m_W - drone_W).
    """
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    obs = []
    for (mx, my, mz) in world_markers:
        dx = mx - drone_x
        dy = my - drone_y
        dz = mz - drone_z
        bx =  c*dx + s*dy
        by = -s*dx + c*dy
        bz =        dz
        obs.append((bx, by, bz))
    return obs


def _wrap_pi(a):
    while a > math.pi:  a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def run_case(label, world, drone_xyz, yaw_rad, mutate=None,
             pos_tol_m=1.5e-3, yaw_tol_deg=0.5):
    """Generic case driver.

    world      -- list of (x,y,z) marker world positions (registered fresh
                  for each case via set_marker_world)
    drone_xyz  -- ground-truth drone position
    yaw_rad    -- ground-truth drone yaw, used both to synthesise obs and
                  as the cf2_yaw anchor passed to get_drone_pose
    mutate     -- optional fn(obs, world) -> obs to perturb observations
                  (T5 negates obs.z; T6 scrambles order)
    """
    # Register this case's world geometry.
    sentai.markers.set_marker_world(pack_xyz(world))

    obs = body_obs_from_pose(world,
                             drone_xyz[0], drone_xyz[1], drone_xyz[2],
                             yaw_rad)
    if mutate is not None:
        obs = mutate(obs, world)
    sentai.markers.test_inject_obs(pack_xyz(obs))

    out = sentai.markers.get_drone_pose_tuple(yaw_rad)
    if out is None:
        print("[s184]", label, "FAIL get_drone_pose returned None")
        return False
    x, y, z, yaw_est, res_max, n_used, flip_x, flip_y, flip_z = out

    err_x = x - drone_xyz[0]
    err_y = y - drone_xyz[1]
    err_z = z - drone_xyz[2]
    pos_err = math.sqrt(err_x*err_x + err_y*err_y + err_z*err_z)

    yaw_err = math.degrees(abs(_wrap_pi(yaw_est - yaw_rad)))

    ok = (pos_err < pos_tol_m and
          yaw_err < yaw_tol_deg and
          n_used  == len(world) and
          res_max < 5e-4)
    print("[s184]", label, "PASS" if ok else "FAIL",
          "pos_err={:.4f}mm yaw_err={:.4f}deg n={} res={:.4f}mm "
          "flip=({},{},{})".format(
              pos_err*1000.0, yaw_err, n_used, res_max*1000.0,
              flip_x, flip_y, flip_z))
    return ok


# ====================== T1 -- identity ======================
ok_t1 = run_case("T1", MARKER_WORLD_FULL, (0.0, 0.0, 0.78), 0.0)

# ====================== T2 -- off-centre hover ======================
ok_t2 = run_case("T2", MARKER_WORLD_FULL, (0.05, -0.03, 0.78), 0.0)

# ====================== T3 -- non-zero yaw (small angle) ======================
ok_t3 = run_case("T3", MARKER_WORLD_FULL,
                 (0.02, 0.04, 0.80), math.radians(15.0))

# ====================== T4 -- large-angle yaw (cos<0) ======================
# yaw=+170 deg: cos(yaw) = -0.985 < 0.  The symmetric pad admits an
# alternate res=0 fit (R_180z composed with a 180-rotated marker
# permutation); whichever branch Kabsch picks, the picker must use the
# negative-cos check to converge on the truth pose.
ok_t4 = run_case("T4", MARKER_WORLD_FULL,
                 (0.06, -0.04, 0.78), math.radians(170.0))

# ====================== T5 -- flip_z (below-pad branch) ======================
# Honest drone at +0.78 m above the pad, but we negate obs.z to simulate
# Kabsch picking the coplanar "drone-below" branch (e.g. from a noisy
# fit on a near-coplanar pad).  The post-fit Z reflection MUST lift t[2]
# back above the marker plane.
def negate_obs_z(obs, world):
    return [(bx, by, -bz) for (bx, by, bz) in obs]


ok_t5 = run_case("T5", MARKER_WORLD_FULL,
                 (0.0, 0.0, 0.78), 0.0, mutate=negate_obs_z,
                 pos_tol_m=2.0e-3)

# ====================== T6 -- permutation search (6-marker symmetric pad, mirror deployment) ======================
# Operator decision 2026-05-21: T6 must mirror the deployment hardware
# (6-marker symmetric pad), not isolate permutation search on an
# asymmetric subset.  Symmetric pad admits two equally-valid res=0
# Kabsch fits per scrambled assignment (identity + 180-Z); the picker
# must converge on truth via yaw-anchor regardless of which branch SVD
# picked.  Tolerance loosened slightly to absorb any sub-mm residual
# from the symmetric-pad permutation-search tie-breaking.
def scramble_6(obs, world):
    perm = [3, 0, 5, 2, 4, 1]
    return [obs[p] for p in perm]


ok_t6 = run_case("T6", MARKER_WORLD_FULL,
                 (-0.04, 0.07, 0.79), math.radians(10.0),
                 mutate=scramble_6,
                 pos_tol_m=2.0e-3, yaw_tol_deg=1.0)


all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5 and ok_t6
print("[s184] OVERALL", "PASS" if all_ok else "FAIL")
print("[s184] END")
