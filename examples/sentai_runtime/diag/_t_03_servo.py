# L4 driver test for sentai.servo (ObjectsPlan Stage 4 skeleton).
# Fresh — does NOT reuse anything from feature/ov5640-camera-support
# tests (per [[no-broken-branch-test-reuse]]).
#
# Covers:
#   1. No-backend faults (fresh-boot only — auto-skipped on reruns)
#   2. init(backend) — str and int forms; bad ids rejected
#   3. Arm/disarm idempotency
#   4. Not-armed gates: takeoff/move/hover/land all reject when armed=0
#   5. Takeoff range gate: alt in (0, 30]
#   6. Flight-phase gate: move/hover/land require AIRBORNE
#   7. Move range gate: |dxyz| ≤ 5 m, |dyaw| ≤ π/2, all finite
#   8. Happy-path FSM: init → arm → takeoff → move → hover → land
#   9. Trace ring FIFO + overwrite semantics (16-entry depth)
#  10. status() and clear_trace() truth check (state preserved)
#
# Run via SIM REPL (per Sim.md §10w):
#   cp diag/_t_03_servo.py build-sim/sentai_fs_root/t03_servo.py
#   echo 'import t03_servo' | ./build-sim/sim/sentai_sim
#
# Run on board (REPL):
#   exec(open('/lib/diag/_t_03_servo.py').read())

import sentai

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

print("=== sentai.servo L4 driver test ===")

S = sentai.servo
DEPTH = 16   # SENTAI_SERVO_TRACE_DEPTH

# ---------- 1. No-backend (fresh-boot only) ----------
print("[1] No-backend faults (fresh-boot only)")
st = S.status()
if st['backend'] == S.NONE:
    chk(S.arm()              == -1, "arm() before init → -1")
    chk(S.disarm()           == -1, "disarm() before init → -1")
    chk(S.takeoff(2.0)       == -1, "takeoff() before init → -1")
    chk(S.move(0.1, 0.0, 0.0) == -1, "move() before init → -1")
    chk(S.hover()            == -1, "hover() before init → -1")
    chk(S.land()             == -1, "land() before init → -1")
    chk(S.status()['faults_no_backend'] >= 6,
        "faults_no_backend counter advanced (got %d)" % S.status()['faults_no_backend'])
else:
    print("  SKIP  fresh-boot test (backend already %s)" % st['backend'])

# Baseline: init to SIM and wipe trace ring for the rest of the suite.
chk(S.init("sim")            == 0, "init('sim') rc=0")
S.clear_trace()

# ---------- 2. init() ----------
print("[2] init() forms")
chk(S.init(S.SIM)            == 0, "init(int SIM) rc=0")
chk(S.init("cf2")            == 0, "init('cf2') rc=0")
chk(S.status()['backend']    == S.CF2, "backend now CF2 (got %d)" % S.status()['backend'])
chk(S.init(99)               == -1, "init(99) rejected → -1")
chk(S.init("xyz")            == -1, "init('xyz') rejected → -1")
# After bad init, previous backend MUST be untouched.
chk(S.status()['backend']    == S.CF2, "backend preserved after bad init")
chk(S.init("sim")            == 0, "reset to SIM for remaining tests")
S.clear_trace()

# ---------- 3. Arm/disarm idempotency ----------
print("[3] Arm/disarm idempotency")
chk(S.arm()                  == 0, "arm() rc=0")
chk(S.status()['armed']      == 1, "armed=1 after arm()")
chk(S.arm()                  == 0, "arm() idempotent rc=0")
chk(S.status()['armed']      == 1, "still armed after 2nd arm()")
chk(S.disarm()               == 0, "disarm() rc=0")
chk(S.status()['armed']      == 0, "armed=0 after disarm()")
chk(S.disarm()               == 0, "disarm() idempotent")

# ---------- 4. Not-armed gates ----------
print("[4] Not-armed gates")
# Pre-condition: backend=SIM, armed=0.  Each of these MUST return -2.
chk(S.takeoff(2.0)           == -2, "takeoff while disarmed → -2")
chk(S.move(0.1, 0.0, 0.0)    == -2, "move while disarmed → -2")
chk(S.hover()                == -2, "hover while disarmed → -2")
chk(S.land()                 == -2, "land while disarmed → -2")
fn = S.status()['faults_not_armed']
chk(fn >= 4, "faults_not_armed advanced (got %d)" % fn)

# ---------- 5. Takeoff range gate ----------
print("[5] Takeoff range gate: alt ∈ (0, 30]")
S.arm()
chk(S.takeoff(0.0)           == -3, "takeoff(0) → -3 (open interval)")
chk(S.takeoff(-1.0)          == -3, "takeoff(-1) → -3")
chk(S.takeoff(30.0001)       == -3, "takeoff(30.0001) → -3")
chk(S.takeoff(float('nan'))  == -3, "takeoff(NaN) → -3")
chk(S.status()['flight']     == S.GROUND, "still GROUND after all failed takeoffs")
chk(S.takeoff(30.0)          ==  0, "takeoff(30.0) ok (closed upper)")
chk(S.status()['flight']     == S.AIRBORNE, "flight=AIRBORNE after success")
chk(S.takeoff(2.0)           == -3, "takeoff() while AIRBORNE → -3")

# ---------- 6. Flight-phase gate for move/hover/land ----------
# (Already AIRBORNE from previous step.)
print("[6] Flight-phase gate")
chk(S.land()                 == 0, "land() AIRBORNE→GROUND ok")
chk(S.status()['flight']     == S.GROUND, "flight=GROUND after land")
chk(S.land()                 == -3, "land() while GROUND → -3")
chk(S.move(0.1, 0.0, 0.0)    == -3, "move() while GROUND → -3")
chk(S.hover()                == -3, "hover() while GROUND → -3")

# ---------- 7. Move range gate (need AIRBORNE again) ----------
print("[7] Move range gate")
S.takeoff(1.0)
chk(S.status()['flight'] == S.AIRBORNE, "back AIRBORNE for move tests")
chk(S.move(5.0,  0.0, 0.0, 0.0)        ==  0, "move(5,0,0,0) ok (boundary)")
chk(S.move(5.0001, 0.0, 0.0, 0.0)      == -3, "|dx|>5 → -3")
chk(S.move(0.0, -5.01, 0.0, 0.0)       == -3, "|dy|>5 → -3")
chk(S.move(0.0,  0.0, 5.01, 0.0)       == -3, "|dz|>5 → -3")
# dyaw boundary test uses a value well below π/2 to avoid float32-rounding
# ambiguity at the cap (1.5707963705f).  1.5 << π/2 → must accept.
chk(S.move(0.0,  0.0, 0.0, 1.5)        ==  0, "dyaw=1.5 rad (< π/2) ok")
chk(S.move(0.0,  0.0, 0.0, 1.58)       == -3, "|dyaw|>π/2 → -3")
chk(S.move(float('inf'), 0.0, 0.0, 0.0) == -3, "Inf dx → -3")
chk(S.move(0.0, float('nan'), 0.0, 0.0) == -3, "NaN dy → -3")
fo = S.status()['faults_oob']
chk(fo >= 8, "faults_oob counter advanced (got %d)" % fo)

# Sanity: hover and land both work from AIRBORNE.
chk(S.hover()                == 0, "hover() AIRBORNE ok")
chk(S.land()                 == 0, "land() AIRBORNE→GROUND ok")

# ---------- 8. Happy-path FSM end-to-end (fresh trace) ----------
print("[8] Happy-path FSM walk")
S.init("sim")
S.clear_trace()
chk(S.arm()                  ==  0, "arm")
chk(S.takeoff(1.5)           ==  0, "takeoff(1.5)")
chk(S.move(0.5,  0.0, 0.0)   ==  0, "move forward")
chk(S.move(0.0,  0.5, 0.0)   ==  0, "strafe right")
chk(S.hover()                ==  0, "hover")
chk(S.land()                 ==  0, "land")
chk(S.disarm()               ==  0, "disarm")

# clear_trace() ran AFTER init(), so INIT trace is gone.  7 successful
# actions: ARM, TAKEOFF, MOVE, MOVE, HOVER, LAND, DISARM.
st = S.status()
chk(st['trace_count']        == 7, "trace_count == 7 after walk (got %d)" % st['trace_count'])
chk(st['last_action']        == S.ACT_DISARM,
    "last_action == ACT_DISARM (got %d)" % st['last_action'])
chk(st['last_result']        == 0, "last_result == 0")
chk(st['actions_ok']         >= 7, "actions_ok ≥ 7 (got %d)" % st['actions_ok'])

# Trace dump truth check.
tr = S.trace()
chk(len(tr) == 7, "trace() returns 7 entries")
chk(tr[0]['action']  == S.ACT_ARM,     "trace[0] = ARM")
chk(tr[1]['action']  == S.ACT_TAKEOFF, "trace[1] = TAKEOFF")
chk(abs(tr[1]['p0'] - 1.5) < 1e-5,     "trace[1].p0 == 1.5 (alt)")
chk(tr[2]['action']  == S.ACT_MOVE,    "trace[2] = MOVE")
chk(abs(tr[2]['p0'] - 0.5) < 1e-5,     "trace[2].p0 == 0.5 (dx)")
chk(tr[3]['action']  == S.ACT_MOVE,    "trace[3] = MOVE (second)")
chk(tr[4]['action']  == S.ACT_HOVER,   "trace[4] = HOVER")
chk(tr[5]['action']  == S.ACT_LAND,    "trace[5] = LAND")
chk(tr[6]['action']  == S.ACT_DISARM,  "trace[6] = DISARM")
# Sequence numbers strictly increasing.
seqs = [e['seq'] for e in tr]
chk(seqs == sorted(seqs) and len(set(seqs)) == len(seqs),
    "trace seqs strictly increasing (got %s)" % seqs)

# ---------- 9. Ring overwrite semantics ----------
print("[9] Trace ring overwrite (depth=16)")
S.init("sim")
S.clear_trace()
S.arm()
S.takeoff(1.0)
# Push lots of MOVE intents so the ring rolls over.  20 entries beyond
# the 2 setup actions → expect overwrites and a count clamped at DEPTH.
N_PUSH = 20
for i in range(N_PUSH):
    S.move(0.1, 0.0, 0.0)
st = S.status()
chk(st['trace_count']        == DEPTH, "trace_count clamps at DEPTH (got %d)" % st['trace_count'])
# Pushes since clear_trace: ARM(1) + TAKEOFF(1) + MOVE*20 = 22 entries.
# DEPTH=16 means overwrites since clear_trace ≥ 22 - 16 = 6.  Counter is
# cumulative across clear_trace, so the absolute value may be higher.
chk(st['trace_overwrites']  >= 6,
    "trace_overwrites ≥ 6 (got %d)" % st['trace_overwrites'])
tr = S.trace()
chk(len(tr)                  == DEPTH, "trace() returns DEPTH entries")
# Oldest survivor in the ring is the (22 - 16) = 6th action since
# clear_trace.  Indices 1..2 were ARM + TAKEOFF; index 3 was MOVE #1;
# overwritten = first 6 entries (ARM, TAKEOFF, MOVE#1..4) → oldest
# survivor is MOVE #5 (and remaining are MOVE #5..#20).
chk(all(e['action'] == S.ACT_MOVE for e in tr),
    "every retained entry is MOVE (ARM/TAKEOFF rolled off)")
# Seq numbers still strictly increasing.
seqs = [e['seq'] for e in tr]
chk(seqs == sorted(seqs) and len(set(seqs)) == len(seqs),
    "ring trace seqs strictly increasing")

# ---------- 10. clear_trace() preserves FSM + counters ----------
print("[10] clear_trace() preserves FSM state and counters")
before = S.status()
n_cleared = S.clear_trace()
after  = S.status()
chk(n_cleared              == before['trace_count'], "clear_trace returned trace_count")
chk(after['trace_count']   == 0,                     "trace_count == 0 after clear_trace")
chk(after['backend']       == before['backend'],     "backend preserved")
chk(after['armed']         == before['armed'],       "armed preserved")
chk(after['flight']        == before['flight'],      "flight preserved")
chk(after['seq']           == before['seq'],         "seq preserved")
chk(after['actions_ok']    == before['actions_ok'],  "actions_ok preserved")
chk(after['faults_oob']    == before['faults_oob'],  "faults_oob preserved")
chk(after['trace_overwrites'] == before['trace_overwrites'],
    "trace_overwrites preserved")

# Land safely so the test ends in a sane FSM state.
S.land()
S.disarm()

print("=== L4 RESULT: %d PASS / %d FAIL ===" % (PASS, FAIL))
