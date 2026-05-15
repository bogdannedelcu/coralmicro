# L6 driver test for sentai.explore (ObjectsPlan L6 skeleton, s133).
#
# Covers (per s133 README):
#   1. Module surface + state ids + action ids
#   2. init(backend) — default sim, bad ids rejected
#   3. State-machine wrong-state gates (start/takeoff/goto/return/land)
#   4. Argument OOB rejection (alt, stop_dist)
#   5. start() → ARMING happy path
#   6. takeoff(alt) → TAKEOFF; pose@alt → HOVERING via tick()
#   7. Negative: goto(unknown_id) → -4; goto() before takeoff → -1
#   8. Bootstrap 2 LIFTED tracklets via L5 init_from_bbox (LIFTED at first call,
#      per s131/t04 math: near marker bbox=45 px → σ_ρ₀ < ε·ρ²)
#   9. goto(obj) → APPROACH; set_pose at target → INSPECT (after toler)
#  10. INSPECT dwell → HOVERING (real-time wait > INSPECT_DUR_MS)
#  11. return_home() → RETURNING; set_pose@home → HOVERING
#  12. land() → LANDING; dwell > LAND_DUR_MS → DONE; servo disarmed
#  13. abort() from any state → ABORT (terminal)
#  14. Trace ring shape + state_name() / action_name() helpers
#  15. Metrics dict contains expected keys + monotonic counters
#
# Run via SIM REPL (per Sim.md §10w):
#   cp examples/sentai_runtime/experiments/s133_explore_skeleton/_t_06_explore.py \
#      build-sim/sentai_fs_root/t06_explore.py
#   echo 'import t06_explore' | ./build-sim/sim/sentai_sim
#
# Pass criterion: ≥ 30 chk() PASS, 0 FAIL.

import sentai

E = sentai.explore
L = sentai.object_lifter
S = sentai.servo
RT = sentai.rtos  # sleep_ms

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

print("=== sentai.explore L6 driver test ===")

# ---------- 1. Module surface ----------
print("[1] Module surface")
chk(hasattr(E, "init"), "explore.init present")
chk(hasattr(E, "start"), "explore.start present")
chk(hasattr(E, "takeoff"), "explore.takeoff present")
chk(hasattr(E, "set_pose"), "explore.set_pose present")
chk(hasattr(E, "goto"), "explore.goto present")
chk(hasattr(E, "return_home"), "explore.return_home present")
chk(hasattr(E, "land"), "explore.land present")
chk(hasattr(E, "stop"), "explore.stop present")
chk(hasattr(E, "abort"), "explore.abort present")
chk(hasattr(E, "tick"), "explore.tick present")
chk(hasattr(E, "state"), "explore.state present")
chk(hasattr(E, "metrics"), "explore.metrics present")
chk(hasattr(E, "trace"), "explore.trace present")
chk(E.IDLE == 0 and E.ABORT == 9, "state ids span 0..9")
chk(E.HOVERING == 3 and E.APPROACH == 4, "HOVERING=3, APPROACH=4")

# ---------- 2. init() and bad ids ----------
print("[2] init() and bad backend ids")
chk(E.init("nonsense") == -2, "init('nonsense') → -2")
chk(E.init("sim") == 0, "init('sim') → 0")
chk(E.state() == "IDLE", "state == 'IDLE' post-init")

# ---------- 3. Wrong-state gates (pre-start) ----------
print("[3] Wrong-state gates from IDLE")
chk(E.takeoff(1.5) == -1, "takeoff() from IDLE → -1")
chk(E.goto(0, 0.3) == -1, "goto() from IDLE → -1")
chk(E.return_home() == -1, "return_home() from IDLE → -1")
chk(E.land() == -1, "land() from IDLE → -1")

# ---------- 4. start() → ARMING ----------
print("[4] start() → ARMING")
chk(E.start() == 0, "start() rc=0")
chk(E.state() == "ARMING", "state == ARMING")
chk(E.start() == -1, "start() from ARMING → -1 (wrong state)")

# ---------- 5. takeoff() arg gates + happy path ----------
print("[5] takeoff() arg + state gates")
chk(E.takeoff(0.0) == -2, "takeoff(0.0) → -2 (alt OOB)")
chk(E.takeoff(-1.0) == -2, "takeoff(-1.0) → -2")
chk(E.takeoff(100.0) == -2, "takeoff(100.0) → -2 (alt > 30)")
chk(E.takeoff(float("nan")) == -2, "takeoff(NaN) → -2")
chk(E.takeoff(1.5) == 0, "takeoff(1.5) rc=0")
chk(E.state() == "TAKEOFF", "state == TAKEOFF")

# ---------- 6. set_pose@alt → tick → HOVERING ----------
print("[6] pose@alt → HOVERING")
chk(E.set_pose(0.0, 0.0, 1.5, 0.0) == 0, "set_pose(0,0,1.5,0) rc=0")
# set_pose internally ticks; should already be HOVERING.
chk(E.state() == "HOVERING", "post-pose@alt → HOVERING")
m = E.metrics()
chk(m["home_set"] == 1, "home_set=1")
chk(near(m["home_x"], 0.0, 0.01), "home_x=0")
chk(near(m["home_y"], 0.0, 0.01), "home_y=0")
chk(near(m["home_z"], 1.5, 0.01), "home_z=alt")

# ---------- 7. Negative goto paths from HOVERING ----------
print("[7] goto() negative paths")
chk(E.goto(99, 0.3) == -4, "goto(unknown_id) → -4")
chk(E.goto(0, -1.0) == -2, "goto(_, -1.0) → -2 (stop_dist OOB)")
chk(E.goto(0, 10.0) == -2, "goto(_, 10.0) → -2 (stop_dist > 5)")
chk(E.state() == "HOVERING", "still HOVERING after rejects")

# ---------- 8. Bootstrap 2 LIFTED tracklets ----------
print("[8] Bootstrap 2 LIFTED tracklets via L5")
L.clear()
fx, fy, cx, cy = 577.0, 579.0, 320.0, 240.0
R_B_C = (0.0, 1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, -1.0)
ofs = (-0.04, 0.0, -0.02)
L.set_camera(fx, fy, cx, cy, R_B_C, ofs)

# Drone at (1.0, 0.0, 1.5) yaw=0 sees marker through center pixel.
# Per L5/s131: bbox=45 px on 0.0625m physical → ρ₀≈1.25, σ_ρ₀≈0.625,
# ε·ρ²≈0.78 → LIFTED at init.  Marker world position: anchor +
# (1/ρ)·r_w ≈ pose + R_W_B(yaw=0)·R_B_C·(0,0,1)·(1/ρ) ≈ pose+(0,0,-0.8).
rc = L.init_from_bbox(7, 4, 320.0, 240.0, 45.0, 0.0625,
                      (1.0, 0.0, 1.5), 0.0)
chk(rc >= 0, "L.init_from_bbox(7,...) ok")
e7 = L.get(7)
chk(e7 is not None and e7["status"] == L.LIFTED, "tracklet 7 LIFTED")
wp7 = L.world_pos(7)
chk(wp7 is not None and len(wp7) == 3, "world_pos(7) 3-tuple")
print("     tracklet 7 world: %.3f %.3f %.3f" % wp7)

rc = L.init_from_bbox(8, 4, 320.0, 240.0, 45.0, 0.0625,
                      (-1.0, 1.0, 1.5), 0.0)
chk(rc >= 0, "L.init_from_bbox(8,...) ok")
wp8 = L.world_pos(8)
chk(wp8 is not None, "world_pos(8) ok")
print("     tracklet 8 world: %.3f %.3f %.3f" % wp8)

# ---------- 9. goto(7) → APPROACH; arrival → INSPECT ----------
print("[9] goto(7) round-trip")
# Drone currently at home (0,0,1.5).  Target wp7 ≈ (~1, 0, 0.7).
# We're testing FSM logic, not landing on a marker — stop_dist=0.5
# keeps motion in horizontal plane reasonable.
chk(E.goto(7, 0.5) == 0, "goto(7, 0.5) rc=0")
chk(E.state() == "APPROACH", "state == APPROACH")

# Simulate drone arriving near target xy.  L6 internal stop-test uses
# 0.4 m radius (skeleton default).  Move pose to target xy.
E.set_pose(wp7[0], wp7[1], 1.5, 0.0)
chk(E.state() == "INSPECT", "pose@target → INSPECT")

# ---------- 10. INSPECT dwell → HOVERING ----------
print("[10] INSPECT dwell")
RT.sleep_ms(1700)  # > 1500 ms inspect dur
E.tick()
chk(E.state() == "HOVERING", "after INSPECT dwell → HOVERING")
m = E.metrics()
chk(m["gotos_completed"] >= 1, "gotos_completed advanced")

# ---------- 11. return_home() ----------
print("[11] return_home() round-trip")
chk(E.return_home() == 0, "return_home() rc=0")
chk(E.state() == "RETURNING", "state == RETURNING")
E.set_pose(0.0, 0.0, 1.5, 0.0)
chk(E.state() == "HOVERING", "pose@home → HOVERING")

# ---------- 12. land() → LANDING → DONE ----------
print("[12] land() → LANDING → DONE")
chk(E.land() == 0, "land() rc=0")
chk(E.state() == "LANDING", "state == LANDING")
RT.sleep_ms(3100)
E.tick()
chk(E.state() == "DONE", "after LAND dwell → DONE")
servo_st = S.status()
chk(servo_st["armed"] == 0, "servo disarmed post-DONE")

# ---------- 13. abort() terminal ----------
print("[13] abort() terminal")
E.init("sim")
E.start()
E.takeoff(1.5)
E.set_pose(0.0, 0.0, 1.5, 0.0)
chk(E.state() == "HOVERING", "fresh mission: HOVERING")
chk(E.abort() == 0, "abort() rc=0")
chk(E.state() == "ABORT", "state == ABORT after abort()")
chk(E.start() == -1, "start() from ABORT → -1 (terminal)")

# ---------- 14. Trace ring shape ----------
print("[14] Trace ring shape")
tr = E.trace()
chk(isinstance(tr, list), "trace() returns list")
chk(len(tr) > 0, "trace has entries")
last = tr[-1]
for k in ("seq", "t_ms", "action", "result", "state_before", "state_after", "p0", "p1", "p2", "p3"):
    chk(k in last, "trace entry has '%s'" % k)
# ABORT trace entry should be present
abort_actions = [t for t in tr if t["action"] == E.ACT_ABORT]
chk(len(abort_actions) >= 1, "trace contains ABORT action")

# ---------- 15. Metrics shape + counters ----------
print("[15] Metrics shape + counters")
m = E.metrics()
required = ("state", "backend", "seq", "trace_count",
            "pose_x", "pose_y", "pose_z", "pose_yaw",
            "home_x", "home_y", "home_z", "home_set",
            "target_tracklet_id",
            "actions_ok", "faults_wrong_state", "faults_oob",
            "faults_servo", "faults_lifter", "faults_pose_stale",
            "transitions", "gotos_completed", "aborts")
for k in required:
    chk(k in m, "metrics has '%s'" % k)
chk(m["aborts"] >= 1, "aborts counter ≥ 1")
chk(m["transitions"] >= 5, "transitions counter advanced (≥ 5)")
chk(m["faults_wrong_state"] >= 5, "faults_wrong_state captured negatives")

# ---------- Summary ----------
print("=== L6 explore driver: %d PASS / %d FAIL ===" % (PASS, FAIL))
if FAIL == 0:
    print("VERDICT: PASS")
else:
    print("VERDICT: FAIL")
