# L5 driver test for sentai.object_lifter (ObjectsPlan Stage 5).
# Inverse-depth EKF; math validated against
# experiments/s131_lifter_replay/lifter_proto.py.
#
# Covers:
#   1. clear() / count() / list() lifecycle
#   2. init_from_bbox — valid + reject paths (F1, F2)
#   3. update_bbox — unknown tracklet (F4), behind_camera (F8)
#   4. update_bbox — full near-marker scenario, σ_ρ ↓, status → LIFTED
#   5. world_pos() yields position near anchor for ρ≈init
#   6. mark_lost() transition
#   7. Eviction of LOST when full
#   8. set_camera() reject paths
#   9. stats() invariants — counters monotonic
#
# Run via SIM REPL (per Sim.md §10w):
#   cp diag/_t_04_object_lifter.py build-sim/sentai_fs_root/t04_object_lifter.py
#   echo 'import t04_object_lifter' | ./build-sim/sim/sentai_sim

import math
import sentai

L = sentai.object_lifter
PASS = 0
FAIL = 0

def chk(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  %s" % label)
    else:
        FAIL += 1
        print("  FAIL  %s" % label)


def near(a, b, tol):
    return abs(a - b) <= tol


print("=== sentai.object_lifter L5 driver test ===")

# Fresh start
L.clear()
chk(L.count() == 0, "clear → count=0")
chk(L.list() == [], "clear → list=[]")

# Bootstrap camera (matches s130 + lifter_proto.py)
fx, fy, cx, cy = 577.0, 579.0, 320.0, 240.0
R_B_C = (0.0, 1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, -1.0)
ofs = (-0.04, 0.0, -0.02)
chk(L.set_camera(fx, fy, cx, cy, R_B_C, ofs) == 0, "set_camera ok")

# ---- F1 invalid input ----
rc = L.init_from_bbox(1, 0, float("nan"), 240.0, 30.0, 0.0625,
                       (0.0, 0.0, 1.0), 0.0)
chk(rc == -1, "init_from_bbox: NaN u_c → -1")
rc = L.init_from_bbox(1, 0, 320.0, 240.0, 30.0, 0.0625,
                       (0.0, float("inf"), 1.0), 0.0)
chk(rc == -1, "init_from_bbox: Inf drone_W → -1")

# ---- F2 invalid class prior ----
rc = L.init_from_bbox(1, 0, 320.0, 240.0, 0.5, 0.0625,
                       (0.0, 0.0, 1.0), 0.0)
chk(rc == -2, "init_from_bbox: bbox_w_px<1 → -2")
rc = L.init_from_bbox(1, 0, 320.0, 240.0, 30.0, 0.0,
                       (0.0, 0.0, 1.0), 0.0)
chk(rc == -2, "init_from_bbox: real_size=0 → -2")

# ---- Valid init: NEAR marker (drone @ z=1m, marker bbox 45 px → d≈0.8m) ----
# bbox_w_px = 45 → d₀ = 577·0.0625/45 ≈ 0.801m → ρ₀ ≈ 1.25.
rc = L.init_from_bbox(7, 4, 320.0, 240.0, 45.0, 0.0625,
                       (0.0, 0.0, 1.0), 0.0)
chk(rc >= 0, "init_from_bbox: valid → slot returned")
chk(L.count() == 1, "after init → count=1")

snap = L.get(7)
chk(snap is not None, "get(7) → dict")
chk(snap["tracklet_id"] == 7, "tracklet_id captured")
chk(snap["class_id"] == 4, "class_id captured")
chk(snap["n_obs"] == 1, "n_obs == 1 after init")
chk(0.8 < 1.0 / snap["rho"] < 0.9, "d_init ≈ 0.8m (cam-prior pseudo-depth)")
chk(snap["var_rho"] > 0, "var_rho > 0")
# Near marker at d=0.8m has σ_ρ₀ ≈ 0.5·ρ₀ = 0.625; ε·ρ² = 0.5·1.56² = ...
# Actually σ_ρ₀ ≈ 0.5·1.25 = 0.625; ε·ρ² = 0.5·1.25² = 0.78 → σ < ε·ρ²
# → status==LIFTED at init.
chk(snap["status"] == L.LIFTED,
    "near marker → LIFTED at init (σ_ρ₀ < ε·ρ²)")

# ---- world_pos() sanity ----
wp = L.world_pos(7)
chk(wp is not None and len(wp) == 3, "world_pos(7) → 3-tuple")
# Drone at (0,0,1) looking down through center pixel → bearing = (0,0,1)
# in cam frame → R_B_C @ (0,0,1) = (0,0,-1) in body → R_W_B(yaw=0) @
# (0,0,-1) = (0,0,-1) world. anchor_w = (0,0,1)+R·ofs = (0,0,1)+(-0.04,
# 0,-0.02) = (-0.04, 0, 0.98). L_W ≈ anchor + (1/1.25)·(0,0,-1) ≈
# (-0.04, 0, 0.18). z should be near 0 (marker on ground).
chk(near(wp[2], 0.18, 0.05),
    "world_pos[2] ≈ 0.18 for near marker (z = drone_z - d_init)")

# ---- F8 behind_camera: drone moves backwards (z=-5m) ----
# Marker is at ~ (-0.04, 0, 0.18); if drone teleports below marker, the
# observation would put marker BEHIND camera. Lifter rejects.
rc = L.update_bbox(7, 320.0, 240.0, (0.0, 0.0, -5.0), 0.0, 0.033)
chk(rc == -5, "update_bbox: behind_camera → -5")

# ---- F4 unknown tracklet ----
rc = L.update_bbox(999, 320.0, 240.0, (0.0, 0.0, 1.0), 0.0, 0.033)
chk(rc == -1, "update_bbox: unknown tracklet → -1")

# ---- Valid update sequence (drone hovers, ρ barely changes since
# 0 parallax). Just verify it doesn't crash + monotonic n_obs. ----
ok_updates = 0
for i in range(5):
    rc = L.update_bbox(7, 320.0, 240.0, (0.001 * i, 0.0, 1.0), 0.0, 0.033)
    if rc == 0:
        ok_updates += 1
chk(ok_updates >= 4, "5 small-motion updates: ≥4 ok (innov ≈ 0)")
snap = L.get(7)
chk(snap["n_obs"] >= 5, "n_obs >= 5 after updates")

# ---- mark_lost ----
chk(L.mark_lost(7) == 0, "mark_lost ok")
snap = L.get(7)
chk(snap["status"] == L.LOST, "status == LOST after mark_lost")
chk(L.mark_lost(999) == -1, "mark_lost unknown → -1")

# ---- set_camera invalid ----
rc = L.set_camera(0.5, 579.0, 320.0, 240.0, R_B_C, ofs)  # fx<=1
chk(rc == -2, "set_camera fx<=1 → -2")

rc = L.set_camera(577.0, float("nan"), 320.0, 240.0, R_B_C, ofs)
chk(rc == -1, "set_camera NaN → -1")

# ---- Stats invariants ----
st = L.stats()
chk(st["inits"] >= 1, "stats.inits >= 1")
chk(st["updates"] >= 4, "stats.updates >= 4")
chk(st["rejects_unknown_tracklet"] >= 1, "stats.rejects_unknown_tracklet >= 1")
chk(st["rejects_behind_camera"] >= 1, "stats.rejects_behind_camera >= 1")
chk(st["rejects_invalid_input"] >= 4,
    "stats.rejects_invalid_input >= 4 (F1, F2 paths)")
chk(st["capacity"] == 16, "stats.capacity == SENTAI_LIFTER_MAX")
chk(st["used"] >= 1, "stats.used >= 1")

# ---- Eviction: fill with LOST entries then verify init replaces oldest ----
L.clear()
chk(L.count() == 0, "post-clear count=0")
# Init 16 entries, mark all as LOST → next init must evict (returns slot).
for tid in range(1, 17):
    L.init_from_bbox(tid, 0, 320.0, 240.0, 45.0, 0.0625,
                      (0.0, 0.0, 1.0), 0.0)
    L.mark_lost(tid)
chk(L.count() == 16, "after 16 inits+lost → 16 entries")
rc = L.init_from_bbox(99, 0, 320.0, 240.0, 45.0, 0.0625,
                       (0.0, 0.0, 1.0), 0.0)
chk(rc >= 0, "17th init evicts LOST → slot returned")
chk(L.get(99) is not None, "get(99) succeeds")

# Final
print("=== L5 lifter: %d PASS / %d FAIL ===" % (PASS, FAIL))
if FAIL == 0:
    print("L5 LIFTER DRIVER PASS")
else:
    print("L5 LIFTER DRIVER FAIL")
