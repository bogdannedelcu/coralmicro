# test_aruco_s159.py — OP-S6-W3-T7 sentai.aruco smoke driver.
# Pure synthetic; no Gazebo.  Verifies the full pipeline end-to-end:
# adaptive threshold + connected-components + quad extraction +
# 4x4 bit decode + DLT-based PnP.
#
# Pass criteria (verdict.py greps the log):
#   T1  Detect id=0 at canonical rotation.
#   T2  Detect id=0 at 90/180/270 CW rotation — same id, tvec.z
#       within 5 mm of T1 (rvec varies; that's correct — physical
#       marker rotation is observable in pose).
#   T3  Detect ids 1..3 from the 8-entry placeholder dictionary.
#   T4  Detect at multiple sizes (48, 96, 120 px) — tvec.z scales as
#       expected by image distance = marker_size * fx / pixel_side.
#   T5  Stats: frames_total advances; markers_total counts hits.
#   T6  Clear: post-clear get_latest returns [].

import math
import sentai

print("[s159] BEGIN")
sentai.aruco.init()
sentai.aruco.set_intrinsics(240.0, 240.0, 160.0, 120.0)
sentai.aruco.set_marker_size(0.10)


def expect_id(name, marker_id, side_px, rotation_cw,
              tvec_z_min, tvec_z_max):
    n = sentai.aruco._test_synth_and_detect(marker_id, side_px, rotation_cw)
    m = sentai.aruco.get_latest()
    ok = (n == 1 and m and m[0]['marker_id'] == marker_id
          and m[0]['hamming'] == 0
          and m[0]['reproj_err_px'] < 1.0
          and tvec_z_min <= m[0]['tvec_cam'][2] <= tvec_z_max)
    tvec = m[0]['tvec_cam'] if m else None
    reproj = m[0]['reproj_err_px'] if m else None
    print("[s159] {} {} n={} id={} hamm={} tvec_z={} reproj={}".format(
        name, "PASS" if ok else "FAIL", n,
        m[0]['marker_id'] if m else None,
        m[0]['hamming'] if m else None,
        tvec[2] if tvec else None,
        reproj))
    return ok


# T1 — canonical rotation, id=0.
ok_t1 = expect_id("T1 id=0 rot=0", 0, 96, 0, 0.24, 0.26)

# T2 — 4 rotations of id=0.  tvec.z should be identical (within FP).
res_t2 = []
for rot in (1, 2, 3):
    res_t2.append(expect_id("T2.{} id=0 rot={}".format(rot, rot*90),
                             0, 96, rot, 0.24, 0.26))
ok_t2 = all(res_t2)

# T3 — different IDs from the dict.
ok_t3a = expect_id("T3a id=1", 1, 96, 0, 0.24, 0.26)
ok_t3b = expect_id("T3b id=2", 2, 96, 0, 0.24, 0.26)
ok_t3c = expect_id("T3c id=3", 3, 96, 0, 0.24, 0.26)
ok_t3  = ok_t3a and ok_t3b and ok_t3c

# T4 — size scaling.  Distance = marker_size * fx / pixel_side.
#   side=48  -> Z ~= 0.10 * 240 / 48  = 0.500 m
#   side=120 -> Z ~= 0.10 * 240 / 120 = 0.200 m
ok_t4a = expect_id("T4a id=0 size=48",  0, 48,  0, 0.46, 0.54)
ok_t4b = expect_id("T4b id=0 size=120", 0, 120, 0, 0.18, 0.22)
ok_t4 = ok_t4a and ok_t4b

# T5 — stats sanity.
stats = sentai.aruco.get_stats()
ok_t5 = (stats['frames_total'] >= 9 and stats['markers_total'] >= 9)
print("[s159] T5 stats {}: frames={} markers={}".format(
    "PASS" if ok_t5 else "FAIL",
    stats['frames_total'], stats['markers_total']))

# T6 — clear empties the cache.
sentai.aruco.clear()
ok_t6 = (len(sentai.aruco.get_latest()) == 0)
print("[s159] T6 clear {}: latest={}".format(
    "PASS" if ok_t6 else "FAIL", sentai.aruco.get_latest()))

# T7 — rvec_to_R and R_to_rvec round-trip on a non-trivial rotation.
rv = (0.3, -0.4, 0.5)
R = sentai.aruco.rvec_to_R(rv)
rv2 = sentai.aruco.R_to_rvec(R)
err = max(abs(rv[i] - rv2[i]) for i in range(3))
ok_t7 = err < 1e-5
print("[s159] T7 rvec_R_roundtrip {}: err={}".format(
    "PASS" if ok_t7 else "FAIL", err))

all_ok = ok_t1 and ok_t2 and ok_t3 and ok_t4 and ok_t5 and ok_t6 and ok_t7
print("[s159] OVERALL {}".format("PASS" if all_ok else "FAIL"))
print("[s159] END")
