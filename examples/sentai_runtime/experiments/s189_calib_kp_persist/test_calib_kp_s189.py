# test_calib_kp_s189.py -- OP-S10-W21-T3 Kp persist round-trip.
#
# Pure MicroPython smoke; exercises sentai.calib.commit_kp /
# get_persisted_kp + the calib.ini schema v2 round-trip.

import sentai

print("[s189] BEGIN")

CALIB_FILE = "calib.ini"
KP_X       = 0.390
KP_Y       = 0.425
KP_YAW     = 0.150


def _read(path):
    try:
        return sentai.fs.read(path)
    except (AttributeError, OSError):
        return None


def _isclose(a, b, tol=1e-5):
    return abs(a - b) < tol


# ===================== T1 -- round-trip ============================
sentai.calib.clear()
# Need a valid R + offset before save (commit_R asserts non-default
# state for is_calibrated transition; here we use identity + sdf
# offset just to satisfy save path).
I3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
sentai.calib.commit_R(I3, (-0.04, 0.0, -0.02))
rc_x   = sentai.calib.commit_kp("x",   KP_X)
rc_y   = sentai.calib.commit_kp("y",   KP_Y)
rc_yaw = sentai.calib.commit_kp("yaw", KP_YAW)
rc_save = sentai.calib.save()

sentai.calib.clear()
# After clear, all 3 axes must return -1.
clear_ok = (_isclose(sentai.calib.get_persisted_kp("x"),   -1.0) and
             _isclose(sentai.calib.get_persisted_kp("y"),   -1.0) and
             _isclose(sentai.calib.get_persisted_kp("yaw"), -1.0))

rc_load = sentai.calib.load()
x_back   = sentai.calib.get_persisted_kp("x")
y_back   = sentai.calib.get_persisted_kp("y")
yaw_back = sentai.calib.get_persisted_kp("yaw")
ok_t1 = (rc_x == 0 and rc_y == 0 and rc_yaw == 0 and
         rc_save and clear_ok and rc_load == 1 and
         _isclose(x_back,   KP_X)   and
         _isclose(y_back,   KP_Y)   and
         _isclose(yaw_back, KP_YAW))
print("[s189] T1 {} x={} y={} yaw={} clear_ok={} rc_save={} rc_load={}".format(
    "PASS" if ok_t1 else "FAIL",
    x_back, y_back, yaw_back, clear_ok, rc_save, rc_load))


# ===================== T2 -- default uncalibrated ==================
sentai.calib.clear()
ok_t2 = (_isclose(sentai.calib.get_persisted_kp("x"),   -1.0) and
         _isclose(sentai.calib.get_persisted_kp("y"),   -1.0) and
         _isclose(sentai.calib.get_persisted_kp("yaw"), -1.0))
print("[s189] T2 {} all_neg1={}".format("PASS" if ok_t2 else "FAIL", ok_t2))


# ===================== T3 -- invalid input rejected ===============
# Negative non-sentinel -> reject.
rc_bad_neg = sentai.calib.commit_kp("x", -0.5)
# Zero -> reject (per spec).
rc_bad_zero = sentai.calib.commit_kp("x", 0.0)
# NaN -> reject.
rc_bad_nan = sentai.calib.commit_kp("x", float("nan"))
# After the failed commits, x should still be -1 (default).
x_still = sentai.calib.get_persisted_kp("x")
ok_t3 = (rc_bad_neg == -1 and
         rc_bad_zero == -1 and
         rc_bad_nan  == -1 and
         _isclose(x_still, -1.0))
print("[s189] T3 {} neg={} zero={} nan={} x_unchanged={}".format(
    "PASS" if ok_t3 else "FAIL",
    rc_bad_neg, rc_bad_zero, rc_bad_nan, _isclose(x_still, -1.0)))


# ===================== T4 -- explicit -1 sentinel accepted =========
sentai.calib.commit_kp("x", 0.5)            # set something real
rc_clear = sentai.calib.commit_kp("x", -1.0)
x_after = sentai.calib.get_persisted_kp("x")
ok_t4 = (rc_clear == 0 and _isclose(x_after, -1.0))
print("[s189] T4 {} rc_clear={} x_after={}".format(
    "PASS" if ok_t4 else "FAIL", rc_clear, x_after))


# ===================== T5 -- on-disk format ========================
# Re-prime so save writes real values; then inspect the INI.
sentai.calib.clear()
sentai.calib.commit_R(I3, (-0.04, 0.0, -0.02))
sentai.calib.commit_kp("x",   0.31)
sentai.calib.commit_kp("y",   0.32)
sentai.calib.commit_kp("yaw", 0.33)
sentai.calib.save()
disk = _read(CALIB_FILE)
ok_t5 = (disk is not None and
         "kp_x=0.310000" in disk and
         "kp_y=0.320000" in disk and
         "kp_yaw=0.330000" in disk and
         "schema=2" in disk)
print("[s189] T5 {} has_kp_x={} has_kp_y={} has_kp_yaw={} has_schema2={}".format(
    "PASS" if ok_t5 else "FAIL",
    "kp_x=0.310000" in disk if disk else False,
    "kp_y=0.320000" in disk if disk else False,
    "kp_yaw=0.330000" in disk if disk else False,
    "schema=2" in disk if disk else False))


all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5
print("[s189] OVERALL", "PASS" if all_ok else "FAIL")
print("[s189] END")
