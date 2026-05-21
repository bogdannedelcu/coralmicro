# test_calib_s186.py — OP-S10-W19-T8 sentai.calib smoke on WhyCon
# square pad.
#
# Operator request 2026-05-21: confirm `sentai.calib.run_kabsch` numerics
# survive the smaller-baseline 32x32 cm 6-marker WhyCon pad before
# committing the mission-level s185 yaw smoke to the new geometry.
#
# Pure synthetic — no Gazebo.  Each sample is a (tvec_cam, marker_W,
# drone_W, yaw) tuple computed analytically from a known ground-truth
# R_cam_to_body.  Kabsch is supposed to recover R_cam_to_body to within
# a small residual.
#
# Tests T1-T6 mirror s157 but use the s183/s184 marker geometry:
#   6 markers at corners + mid-bars of a 32x32 cm square pad, all
#   coplanar at z = 5 mm.  4 hover poses at different yaws give
#   6 * 4 = 24 samples per calibration.

import math
import sentai

print("[s186] BEGIN")
sentai.calib.clear()
print("[s186] is_calibrated_initial =", sentai.calib.is_calibrated())

I3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

# Same square pad as s183 / s184 / s185.  Coplanar at z = 0.005 m.
MARKER_WORLD = (
    (-0.16, +0.16, 0.005),
    (+0.16, +0.16, 0.005),
    (-0.12,  0.00, 0.005),
    (+0.12,  0.00, 0.005),
    (-0.16, -0.16, 0.005),
    (+0.16, -0.16, 0.005),
)

# 4 hover poses across a meaningful yaw + position spread.  Drone hovers
# at z = 0.78 m (s183 baseline) with small XY offset; yaw cycles to keep
# Kabsch conditioning balanced.
HOVER_POSES = (
    ( 0.00,  0.00, 0.78,  0.00),
    ( 0.05, -0.03, 0.78,  math.radians(+15.0)),
    (-0.04,  0.07, 0.78,  math.radians(-25.0)),
    ( 0.02,  0.02, 0.78,  math.radians(+40.0)),
)


def mat_vec(R, v):
    return (
        R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
        R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
        R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
    )


def mat_transpose_vec(R, v):
    return (
        R[0] * v[0] + R[3] * v[1] + R[6] * v[2],
        R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
        R[2] * v[0] + R[5] * v[1] + R[8] * v[2],
    )


def Rz(theta_rad):
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return (c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)


def make_whycon_samples(R_true_cam_to_body):
    """Generate 6 markers × 4 hover poses = 24 samples.

    For each (marker, drone, yaw) triple:
        delta_W   = marker_W - drone_W
        body_vec  = R_yaw^T @ delta_W                  (world -> body)
        tvec_cam  = R_cam_to_body^T @ body_vec         (body  -> cam)
    """
    samples = []
    for (dx, dy, dz, yaw) in HOVER_POSES:
        R_world_to_body = Rz(yaw)
        for (mx, my, mz) in MARKER_WORLD:
            delta_W = (mx - dx, my - dy, mz - dz)
            body = mat_transpose_vec(R_world_to_body, delta_W)
            tvec_cam = mat_transpose_vec(R_true_cam_to_body, body)
            samples.append((tvec_cam, (mx, my, mz), (dx, dy, dz), yaw))
    return samples


# ===================== T1 — identity recovery =====================
# Threshold note: s157 uses < 0.01 deg, calibrated for an 8-marker arc at
# radius 0.30 m.  WhyCon's 32 cm square pad has a smaller baseline and
# fewer well-distributed eigen-directions, so the float32 Kabsch residual
# floor rises to ~0.013 deg (0.2 mrad ≈ 80 µm absolute on a 0.4 m
# baseline).  Empirically 0.05 deg comfortably absorbs this; raising it
# higher would mask a real regression.
samples_t1 = make_whycon_samples(I3)
print("[s186] T1 n_samples=", len(samples_t1))
R_t1, q_t1 = sentai.calib.run_kabsch(samples_t1)
ok_t1 = (q_t1["accepted"] and
         q_t1["det_R"] > 0.999 and
         q_t1["mean_residual_deg"] < 0.05)
print("[s186] T1 {} det_R={:.6f} mean_res={:.6f} max_res={:.6f}".format(
    "PASS" if ok_t1 else "FAIL",
    q_t1["det_R"], q_t1["mean_residual_deg"], q_t1["max_residual_deg"]))

# ===================== T2 — ±5° tilt recovery =====================
R_true_t2 = Rz(math.radians(5.0))
samples_t2 = make_whycon_samples(R_true_t2)
R_t2, q_t2 = sentai.calib.run_kabsch(samples_t2, I3)
drift_t2 = q_t2["drift_from_persisted_deg"]
ok_t2 = (q_t2["accepted"] and
         q_t2["mean_residual_deg"] < 0.5 and
         4.5 < drift_t2 < 5.5)
print("[s186] T2 {} drift={:.4f} mean_res={:.6f}".format(
    "PASS" if ok_t2 else "FAIL",
    drift_t2, q_t2["mean_residual_deg"]))

# ===================== T3 — drift gate (no perturb) =====================
samples_t3 = make_whycon_samples(I3)
R_t3, q_t3 = sentai.calib.run_kabsch(samples_t3, I3)
ok_t3 = q_t3["drift_from_persisted_deg"] < 0.01
print("[s186] T3 {} drift={:.6f}".format(
    "PASS" if ok_t3 else "FAIL", q_t3["drift_from_persisted_deg"]))

# ===================== T4 — too-few-samples reject =====================
R_t4, q_t4 = sentai.calib.run_kabsch(samples_t3[:2])
ok_t4 = (not q_t4["accepted"]) and q_t4["reject_code"] == 5
print("[s186] T4 {} accepted={} reject_code={}".format(
    "PASS" if ok_t4 else "FAIL", q_t4["accepted"], q_t4["reject_code"]))

# ===================== T5 — bad-input reject =====================
nan = float("nan")
samples_t5 = [((nan, 0.0, 0.0), (0.0, 0.0, 0.005), (0.0, 0.0, 0.78), 0.0)]
samples_t5 = samples_t5 + samples_t3[:3]
R_t5, q_t5 = sentai.calib.run_kabsch(samples_t5)
ok_t5 = (not q_t5["accepted"]) and q_t5["reject_code"] == 6
print("[s186] T5 {} accepted={} reject_code={}".format(
    "PASS" if ok_t5 else "FAIL", q_t5["accepted"], q_t5["reject_code"]))

# ===================== T6 — save/load round-trip =====================
R_target = R_true_t2
off_target = (-0.04, 0.0, -0.02)   # cf2 SDF cam mount offset
rc_commit = sentai.calib.commit_R(R_target, off_target)
rc_save = sentai.calib.save()
sentai.calib.clear()
is_zero = not sentai.calib.is_calibrated()
rc_load = sentai.calib.load()
R_loaded = sentai.calib.get_R_cam_to_body()
off_loaded = sentai.calib.get_cam_offset_B()
err_R = max(abs(R_target[i] - R_loaded[i]) for i in range(9))
err_off = max(abs(off_target[i] - off_loaded[i]) for i in range(3))
ok_t6 = (rc_commit == 0 and rc_save and is_zero and rc_load and
         err_R < 1e-6 and err_off < 1e-6 and sentai.calib.is_calibrated())
print("[s186] T6 {} rc_commit={} rc_save={} rc_load={} err_R={:.2e} err_off={:.2e}".format(
    "PASS" if ok_t6 else "FAIL",
    rc_commit, rc_save, rc_load, err_R, err_off))

all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5 and ok_t6
print("[s186] OVERALL {}".format("PASS" if all_ok else "FAIL"))
print("[s186] END")
