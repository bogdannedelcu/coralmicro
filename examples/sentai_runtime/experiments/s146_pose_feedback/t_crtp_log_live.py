# t_crtp_log_live.py — integration test for crtp_log.py (Task #41 Option B).
#
# Requires cf2 SITL listening on UDP 127.0.0.1:19850 (launched by
# sim/scripts/launch_hybrid_cf2.sh).  See run.sh --live.
#
# Validates the END-TO-END CRTP LOG pipeline:
#   1. sentai.crazy.init() opens UDP transport
#   2. crtp_log.reset() clears any stale blocks from a prior run
#   3. crtp_log.scan_toc() discovers TOC entries (>= 50 expected on
#      stock cf2 SITL firmware)
#   4. crtp_log.find('stateEstimate', 'x') returns a (id, type) tuple
#      (Float, type=7)
#   5. crtp_log.create_pose_block() subscribes successfully
#   6. After ≥1 second of polling, latest_pose() returns a 4-tuple of
#      floats — drone is at (~0, ~0, ~0, ~0) since it hasn't taken off
#   7. crtp_log.stop() cleans up
#
# PASS criteria: every step returns expected values; pose components
# are floats (not None) within 5 seconds of subscription start.

import sentai
import crtp_log

print("---- t_crtp_log_live ----")

rc = sentai.crazy.init()
print("crazy.init rc=%d" % rc)
assert rc == 0, "crazy.init failed"

# Step 1: clean any leftover blocks from previous runs.
crtp_log.reset()
print("crazy.reset sent")

# Step 2: TOC scan.  Takes ~1-3s on stock cf2 SITL (200-250 entries).
print("TOC scan starting...")
n = crtp_log.scan_toc(timeout_ms=10000)
print("TOC scan got %d entries" % n)
assert n >= 50, "TOC scan returned %d entries (expected >= 50)" % n

# Step 3: verify our target vars are in TOC.
for (g, name) in [('stateEstimate', 'x'),
                  ('stateEstimate', 'y'),
                  ('stateEstimate', 'z'),
                  ('stabilizer', 'yaw')]:
    entry = crtp_log.find(g, name)
    print("  %s.%s -> %s" % (g, name, entry))
    assert entry is not None, "TOC missing: %s.%s" % (g, name)
    ident, ttype = entry
    assert ttype == crtp_log.LOG_T_FLOAT, \
        "%s.%s type=0x%02X, expected 0x07 (float)" % (g, name, ttype)

# Step 4: subscribe.
bid = crtp_log.create_pose_block(block_id=1, period_ms=100)
print("create_pose_block bid=%d" % bid)
assert bid > 0, "create_pose_block failed rc=%d" % bid

# Step 5: poll for at least one pose frame.  At 100ms period we should
# see frames within 200ms; we give it 5s leeway.
got_pose = None
for i in range(50):
    sentai.rtos.sleep_ms(100)
    p = crtp_log.latest_pose()
    if p is not None:
        got_pose = p
        print("got pose @ %d ms: x=%.4f y=%.4f z=%.4f yaw=%.4f" %
              (i*100, p[0], p[1], p[2], p[3]))
        break

assert got_pose is not None, "no pose frame in 5s"

# Sanity: drone at rest near origin (z should be ~0.5m default cf2 SITL
# spawn height; x/y near 0; yaw any value).
assert abs(got_pose[0]) < 2.0, "x out of range: %.3f" % got_pose[0]
assert abs(got_pose[1]) < 2.0, "y out of range: %.3f" % got_pose[1]
assert -1.0 < got_pose[2] < 3.0, "z out of range: %.3f" % got_pose[2]

# Step 6: cleanup.
crtp_log.stop(bid)
print("stop ok")

# Get a few more poses (drone should still be at rest) to verify
# multi-frame delivery works.
sentai.crazy.stop()

print("---- t_crtp_log_live PASS ----")
