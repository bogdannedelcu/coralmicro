# L2 driver test for sentai.objects (ObjectsPlan Stage 1).
# Fresh — does NOT reuse anything from feature/ov5640-camera-support
# tests (per [[no-broken-branch-test-reuse]]).
#
# Covers:
#   1. CRUD round-trip: add → get → list → mark_visited → set_status → remove
#   2. NaN/Inf reject (oob_rejected counter, return -1)
#   3. Class out-of-range reject (return -2)
#   4. Eviction: fill 32 + 1 → STALE oldest reclaimed
#   5. Eviction starvation: fill 32 (no STALE) → return -3
#   6. clear() returns the right count and zeroes the map
#   7. stats() truth check (used / hwm / counters)
#
# Run via REPL:  exec(open('/lib/diag/_t_01_objects.py').read())
# Or in SIM:     <stdin from sentai_sim>  exec(open('.../_t_01_objects.py').read())

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

print("=== sentai.objects L2 driver test ===")
sentai.objects.clear()

# ---------- 1. CRUD round-trip ----------
print("[1] CRUD round-trip")
id1 = sentai.objects.add(3, 1.0, 2.0, 0.5)
chk(id1 > 0, "add returns id>0 (got %s)" % id1)

obj = sentai.objects.get(id1)
chk(obj is not None, "get(id1) non-None")
chk(obj['class_id'] == 3, "get class_id matches")
chk(abs(obj['x'] - 1.0) < 1e-6 and abs(obj['y'] - 2.0) < 1e-6 and abs(obj['z'] - 0.5) < 1e-6,
    "get x/y/z match")
chk(obj['status'] == sentai.objects.TENTATIVE, "default status TENTATIVE")
chk(obj['observations'] == 1, "observations == 1")
chk(len(obj['cov']) == 6, "cov has 6 entries")

lst = sentai.objects.list()
chk(len(lst) == 1, "list len == 1")
chk(lst[0]['id'] == id1, "list[0].id == id1")

rc = sentai.objects.mark_visited(id1)
chk(rc == 0, "mark_visited rc=0")
obj = sentai.objects.get(id1)
chk(obj['visited'] == 1, "visited flag set")

rc = sentai.objects.set_status(id1, sentai.objects.CONFIRMED)
chk(rc == 0, "set_status CONFIRMED rc=0")
obj = sentai.objects.get(id1)
chk(obj['status'] == sentai.objects.CONFIRMED, "status == CONFIRMED")

rc = sentai.objects.remove(id1)
chk(rc == 0, "remove rc=0")
chk(sentai.objects.get(id1) is None, "get after remove is None")
chk(sentai.objects.count() == 0, "count == 0 after remove")

# ---------- 2. NaN / Inf reject ----------
print("[2] NaN / Inf reject")
nan = float('nan')
inf = float('inf')
chk(sentai.objects.add(0, nan, 0.0, 0.0) == -1, "NaN x → -1")
chk(sentai.objects.add(0, 0.0, inf, 0.0) == -1, "Inf y → -1")
chk(sentai.objects.add(0, 0.0, 0.0, -inf) == -1, "-Inf z → -1")
cov_bad = [nan, 0, 0, 1, 0, 1]
chk(sentai.objects.add(0, 0.0, 0.0, 0.0, cov_bad) == -1, "NaN cov → -1")

# ---------- 3. Class out of range ----------
print("[3] Class out of range")
chk(sentai.objects.add(80, 0.0, 0.0, 0.0) == -2, "class=80 (DICT_MAX) → -2")
chk(sentai.objects.add(255, 0.0, 0.0, 0.0) == -2, "class=255 → -2")

# ---------- 4. Fill + eviction ----------
print("[4] Fill 32 then trigger STALE eviction")
sentai.objects.clear()
ids = []
for i in range(32):
    rid = sentai.objects.add(i % 10, float(i), 0.0, 0.0)
    if rid > 0: ids.append(rid)
chk(len(ids) == 32, "32 adds succeeded")
chk(sentai.objects.count() == 32, "count == 32")

# Mark first half STALE so eviction has candidates.
for rid in ids[:16]:
    sentai.objects.set_status(rid, sentai.objects.STALE)

evict_before = sentai.objects.stats()['evictions']
new_id = sentai.objects.add(5, 99.0, 99.0, 0.0)
chk(new_id > 0, "33rd add succeeded via eviction (got %s)" % new_id)
evict_after = sentai.objects.stats()['evictions']
chk(evict_after == evict_before + 1, "evictions counter advanced (%d→%d)"
    % (evict_before, evict_after))
chk(sentai.objects.count() == 32, "count still == 32 after eviction")

# ---------- 5. Eviction starvation (no STALE) ----------
print("[5] Fill 32, no STALE → return -3")
sentai.objects.clear()
for i in range(32):
    sentai.objects.add(0, float(i), 0.0, 0.0)
rc = sentai.objects.add(0, 1000.0, 0.0, 0.0)
chk(rc == -3, "33rd add with no STALE → -3 (got %s)" % rc)

# ---------- 6. clear() ----------
print("[6] clear() returns count and zeroes")
cleared = sentai.objects.clear()
chk(cleared == 32, "clear returned 32 (got %s)" % cleared)
chk(sentai.objects.count() == 0, "count == 0 after clear")
chk(len(sentai.objects.list()) == 0, "list empty after clear")

# ---------- 7. stats() truth check ----------
print("[7] stats truth check")
sentai.objects.clear()
a = sentai.objects.add(1, 0.0, 0.0, 0.0)
b = sentai.objects.add(2, 0.0, 0.0, 0.0)
sentai.objects.set_status(a, sentai.objects.CONFIRMED)
sentai.objects.set_status(b, sentai.objects.COASTING)
sentai.objects.add(0, float('nan'), 0.0, 0.0)  # rejected
sentai.objects.get(255)                         # bad id
s = sentai.objects.stats()
chk(s['used'] == 2, "stats.used == 2 (got %s)" % s['used'])
chk(s['confirmed'] == 1, "stats.confirmed == 1")
chk(s['coasting'] == 1, "stats.coasting == 1")
chk(s['hwm'] >= 32, "hwm preserved across clear() (got %s)" % s['hwm'])
chk(s['oob_rejected'] >= 1, "oob_rejected advanced (got %s)" % s['oob_rejected'])
chk(s['bad_ids'] >= 1, "bad_ids advanced (got %s)" % s['bad_ids'])
chk(s['capacity'] == 32, "capacity == SENTAI_OBJECTS_MAX")

print("=== L2 RESULT: %d PASS / %d FAIL ===" % (PASS, FAIL))
