# test_calib_s157.py — OP-S6-W1 (sentai.calib) smoke test driver.
# Pure MicroPython; runs end-to-end inside sentai_sim via REPL `import`.
#
# Pass criteria (gated by verdict.py on the host side):
#   T1  identity recovery (R_true = identity, n=8, no noise):
#         mean_residual_deg < 0.01, det_R > 0.999, accepted = True
#   T2  ±5° tilt recovery (R_true = identity rotated by 5° around Z):
#         drift_from_persisted_deg(R_est, identity) within [4.5, 5.5]
#         mean_residual_deg < 0.5, accepted = True
#   T3  drift gate (R_est vs persisted identity, no perturb):
#         drift_from_persisted_deg < 0.01
#   T4  too-few-samples reject (n=2): accepted = False, reject_code = 5
#   T5  bad-input reject (NaN tvec): accepted = False, reject_code = 6
#   T6  save/load round-trip: bit-identical R after load
#
# Each T prints `[s157] T{N} {STATUS} <metrics>`.  verdict.py greps the log.

import math
import sentai

print("[s157] BEGIN")
sentai.calib.clear()
print("[s157] is_calibrated_initial =", sentai.calib.is_calibrated())

I3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def mat_vec(R, v):
    return (
        R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
        R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
        R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
    )


def mat_transpose_vec(R, v):
    # Returns R^T * v.
    return (
        R[0] * v[0] + R[3] * v[1] + R[6] * v[2],
        R[1] * v[0] + R[4] * v[1] + R[7] * v[2],
        R[2] * v[0] + R[5] * v[1] + R[8] * v[2],
    )


def Rz(theta_rad):
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return (c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)


def make_samples(R_true, n=8, yaw_spread_rad=0.5):
    """Generate n synthetic samples for Kabsch — deterministic, no RNG."""
    radius = 0.30
    z_pad = 0.20
    drone_z = 1.0
    samples = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        marker_W = (radius * math.cos(theta), radius * math.sin(theta), z_pad)
        # Drone hovers slightly off-axis so samples are not degenerate.
        drone_W = (0.02 * math.cos(theta + 1.0),
                   0.02 * math.sin(theta + 1.0),
                   drone_z)
        # Yaw cycles around 0 to span yaw_spread.
        yaw = yaw_spread_rad * math.sin(theta)
        Rwb = Rz(yaw)
        delta_W = (marker_W[0] - drone_W[0],
                   marker_W[1] - drone_W[1],
                   marker_W[2] - drone_W[2])
        body = mat_transpose_vec(Rwb, delta_W)
        tvec_cam = mat_transpose_vec(R_true, body)
        samples.append((tvec_cam, marker_W, drone_W, yaw))
    return samples


# ===================== T1 — identity recovery =====================
samples_t1 = make_samples(I3, n=8)
R_t1, q_t1 = sentai.calib.run_kabsch(samples_t1)
ok_t1 = (q_t1["accepted"] and
         q_t1["det_R"] > 0.999 and
         q_t1["mean_residual_deg"] < 0.01)
print("[s157] T1 {} det_R={:.6f} mean_res={:.6f} max_res={:.6f}".format(
    "PASS" if ok_t1 else "FAIL",
    q_t1["det_R"], q_t1["mean_residual_deg"], q_t1["max_residual_deg"]))

# ===================== T2 — ±5° tilt recovery =====================
R_true_t2 = Rz(math.radians(5.0))
samples_t2 = make_samples(R_true_t2, n=8)
R_t2, q_t2 = sentai.calib.run_kabsch(samples_t2, I3)
drift_t2 = q_t2["drift_from_persisted_deg"]
ok_t2 = (q_t2["accepted"] and
         q_t2["mean_residual_deg"] < 0.5 and
         4.5 < drift_t2 < 5.5)
print("[s157] T2 {} drift={:.4f} mean_res={:.6f}".format(
    "PASS" if ok_t2 else "FAIL",
    drift_t2, q_t2["mean_residual_deg"]))

# ===================== T3 — drift gate (no perturb) =====================
samples_t3 = make_samples(I3, n=8)
R_t3, q_t3 = sentai.calib.run_kabsch(samples_t3, I3)
ok_t3 = q_t3["drift_from_persisted_deg"] < 0.01
print("[s157] T3 {} drift={:.6f}".format(
    "PASS" if ok_t3 else "FAIL", q_t3["drift_from_persisted_deg"]))

# ===================== T4 — too-few-samples reject =====================
R_t4, q_t4 = sentai.calib.run_kabsch(samples_t3[:2])
ok_t4 = (not q_t4["accepted"]) and q_t4["reject_code"] == 5
print("[s157] T4 {} accepted={} reject_code={}".format(
    "PASS" if ok_t4 else "FAIL", q_t4["accepted"], q_t4["reject_code"]))

# ===================== T5 — bad-input reject =====================
nan = float("nan")
samples_t5 = [((nan, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)]
# Pad to >= SAMPLES_MIN so we hit the NaN check, not the too-few gate.
samples_t5 = samples_t5 + samples_t3[:3]
R_t5, q_t5 = sentai.calib.run_kabsch(samples_t5)
ok_t5 = (not q_t5["accepted"]) and q_t5["reject_code"] == 6
print("[s157] T5 {} accepted={} reject_code={}".format(
    "PASS" if ok_t5 else "FAIL", q_t5["accepted"], q_t5["reject_code"]))

# ===================== T6 — save/load round-trip =====================
R_target = R_true_t2  # the 5° rotation, deterministic
off_target = (0.01, 0.02, 0.03)
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
print("[s157] T6 {} rc_commit={} rc_save={} rc_load={} err_R={:.2e} err_off={:.2e}".format(
    "PASS" if ok_t6 else "FAIL",
    rc_commit, rc_save, rc_load, err_R, err_off))

all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5 and ok_t6
print("[s157] OVERALL {}".format("PASS" if all_ok else "FAIL"))
print("[s157] END")
