# L3 driver test for sentai.places (ObjectsPlan Stage 11.B + 11.D).
# Fresh — does NOT reuse anything from feature/ov5640-camera-support
# tests (per [[no-broken-branch-test-reuse]]).
#
# Covers:
#   1. CRUD round-trip: add → get → list → observe → set_status → remove
#   2. Bad input reject:
#       - h3_cell == 0 AND desc == None  → -1
#       - desc length wrong              → -2
#   3. H3 helpers: cell_at round-trip, cell_to_latlng inverse, neighbors
#   4. Eviction: fill 64 + 1 → LRU non-CONFIRMED reclaimed; CONFIRMED
#      slots are protected
#   5. Eviction starvation: fill 64 all CONFIRMED → return -3
#   6. observe() bumps visits, auto-promotes TENTATIVE → CONFIRMED at 2
#   7. query() finds best L1 match; spatial prefilter (h3 ring) shrinks
#      candidate set; thresh_pct gates `hit` flag
#   8. clear() returns the right count and zeroes the gallery
#   9. stats() truth check (used / hwm / counters / capacity / desc_dim)
#
# Run via SIM REPL (per Sim.md §10w):
#   cp diag/_t_02_places.py build-sim/sentai_fs_root/t02_places.py
#   echo 'import t02_places' | ./build-sim/sim/sentai_sim
#
# Run on board (REPL):
#   exec(open('/lib/diag/_t_02_places.py').read())

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

print("=== sentai.places L3 driver test ===")
sentai.places.clear()

P = sentai.places
DD = P.stats()['desc_dim']        # discover descriptor size at runtime

def desc_solid(byte):
    """Return DD-byte descriptor filled with a constant byte."""
    return bytes([byte & 0xFF] * DD)

def desc_ramp(start):
    """Return DD-byte descriptor with values (start+i) mod 256."""
    return bytes([(start + i) & 0xFF for i in range(DD)])

# ---------- 1. CRUD round-trip ----------
print("[1] CRUD round-trip")
cell0 = P.cell_at(44.4268, 26.1025, 8)             # Bucharest, res 8
chk(cell0 != 0, "cell_at returned non-zero (got 0x%x)" % cell0)

id1 = P.add(cell0, desc_solid(0x40), 1.0, 2.0, 0.5)
chk(id1 > 0, "add returns id>0 (got %s)" % id1)

p = P.get(id1)
chk(p is not None, "get(id1) non-None")
chk(p['h3_cell'] == cell0, "get h3_cell matches input")
chk(abs(p['x'] - 1.0) < 1e-6 and abs(p['y'] - 2.0) < 1e-6 and abs(p['z'] - 0.5) < 1e-6,
    "get x/y/z match")
chk(p['status'] == P.TENTATIVE, "default status TENTATIVE")
chk(p['visits'] == 1, "visits == 1 after first add")
chk(p['desc_set'] == 1, "desc_set flag set")

lst = P.list()
chk(len(lst) == 1, "list len == 1")
chk(lst[0]['id'] == id1, "list[0].id == id1")

rc = P.observe(id1)
chk(rc == 0, "observe rc=0")
p = P.get(id1)
chk(p['visits'] == 2, "visits == 2 after observe (got %s)" % p['visits'])
chk(p['status'] == P.CONFIRMED, "auto-promoted to CONFIRMED at visits==2")

rc = P.set_status(id1, P.TENTATIVE)
chk(rc == 0, "set_status TENTATIVE rc=0")
chk(P.get(id1)['status'] == P.TENTATIVE, "status reverted to TENTATIVE")

rc = P.remove(id1)
chk(rc == 0, "remove rc=0")
chk(P.get(id1) is None, "get after remove is None")
chk(P.count() == 0, "count == 0 after remove")

# ---------- 2. Bad input reject ----------
print("[2] Bad input reject")
chk(P.add(0, None, 0.0, 0.0, 0.0) == -1, "no cell + no desc → -1")
chk(P.add(cell0, b"too_short") == -2, "desc len=9 → -2")
chk(P.add(cell0, b"\x00" * (DD + 1)) == -2, "desc len=DD+1 → -2")
nan = float('nan')
chk(P.add(cell0, desc_solid(0), nan, 0.0, 0.0) == -1, "NaN x → -1")

# ---------- 3. H3 helpers ----------
print("[3] H3 helpers")
lat, lng = P.cell_to_latlng(cell0)
# Round-trip drift bound: res-8 cell ≈ 461 m radius; centre drift < 1 km.
chk(abs(lat - 44.4268) < 0.01, "cell_to_latlng lat within 0.01° (got %.4f)" % lat)
chk(abs(lng - 26.1025) < 0.01, "cell_to_latlng lng within 0.01° (got %.4f)" % lng)

ring1 = P.neighbors(cell0, 1)
chk(len(ring1) == 7, "neighbors(k=1) → 7 cells (got %d)" % len(ring1))
chk(cell0 in ring1, "neighbors(k=1) contains origin")

ring2 = P.neighbors(cell0, 2)
chk(len(ring2) == 19, "neighbors(k=2) → 19 cells (got %d)" % len(ring2))

ring0 = P.neighbors(cell0, 0)
chk(len(ring0) == 1 and ring0[0] == cell0, "neighbors(k=0) → just origin")

# ---------- 4. Eviction (LRU non-CONFIRMED) ----------
print("[4] Fill 64, then trigger LRU eviction")
P.clear()
ids = []
for i in range(64):
    rid = P.add(cell0, desc_ramp(i), float(i), 0.0, 0.0)
    if rid > 0: ids.append(rid)
chk(len(ids) == 64, "64 adds succeeded")
chk(P.count() == 64, "count == 64")

# Confirm the LAST 32; oldest 32 stay TENTATIVE so they evict first.
for rid in ids[32:]:
    P.set_status(rid, P.CONFIRMED)

evict_before = P.stats()['evictions']
new_id = P.add(cell0, desc_solid(0xFF), 99.0, 99.0, 0.0)
chk(new_id > 0, "65th add succeeded via eviction (got %s)" % new_id)
evict_after = P.stats()['evictions']
chk(evict_after == evict_before + 1,
    "evictions counter advanced (%d→%d)" % (evict_before, evict_after))
chk(P.count() == 64, "count still == 64 after eviction")

# Pick any of the original CONFIRMED ids — must still be there.
survived = sum(1 for rid in ids[32:] if P.get(rid) is not None)
chk(survived == 32, "all 32 CONFIRMED slots survived (got %d)" % survived)

# ---------- 5. Eviction starvation (all CONFIRMED) ----------
print("[5] Fill 64 all CONFIRMED → return -3")
P.clear()
for i in range(64):
    rid = P.add(cell0, desc_ramp(i), float(i), 0.0, 0.0)
    P.set_status(rid, P.CONFIRMED)
rc = P.add(cell0, desc_solid(0x55), 0.0, 0.0, 0.0)
chk(rc == -3, "65th add with all CONFIRMED → -3 (got %s)" % rc)

# ---------- 6. observe() promotion + visits saturation ----------
print("[6] observe() semantics")
P.clear()
rid = P.add(cell0, desc_solid(0x10), 0.0, 0.0, 0.0)
chk(P.get(rid)['status'] == P.TENTATIVE, "fresh add is TENTATIVE")
P.observe(rid)
chk(P.get(rid)['status'] == P.CONFIRMED, "promoted at visits==2")
# Visits saturate at 255; bump ~ 260 times and ensure no overflow.
for _ in range(260):
    P.observe(rid)
chk(P.get(rid)['visits'] == 255, "visits saturates at 255")

# ---------- 7. query() ----------
print("[7] query() — best L1 match + spatial prefilter")
P.clear()
# Two cells far apart so the H3 ring prefilter actually filters.
cell_buc = P.cell_at(44.4268, 26.1025, 8)
cell_par = P.cell_at(48.8566,  2.3522, 8)
chk(cell_buc != cell_par, "Bucharest and Paris cells distinct")

id_buc1 = P.add(cell_buc, desc_solid(0x10), 0.0, 0.0, 0.0)
id_buc2 = P.add(cell_buc, desc_solid(0x80), 0.0, 0.0, 0.0)
id_par1 = P.add(cell_par, desc_solid(0x10), 0.0, 0.0, 0.0)

# Global query (cell=0) — should hit Bucharest #1 OR Paris #1 (both
# match exactly, score 100).  Tie-break is "first one found"; verify
# only that the score is 100 and the match is one of the two.
r = P.query(desc_solid(0x10), 0, 0, 90)
chk(r['score_pct'] == 100, "global query exact match → score 100 (got %d)" % r['score_pct'])
chk(r['l1_dist'] == 0, "exact match → l1_dist 0 (got %d)" % r['l1_dist'])
chk(r['hit'] == 1, "global query hit flag set")
chk(r['id'] in (id_buc1, id_par1), "global match id in {buc1,par1}")

# Spatial prefilter: restrict to Paris ring → must match id_par1.
r = P.query(desc_solid(0x10), cell_par, 1, 90)
chk(r['id'] == id_par1, "Paris-ring query → id_par1 (got %s)" % r['id'])

# Spatial prefilter: restrict to Bucharest ring → never matches Paris.
r = P.query(desc_solid(0x10), cell_buc, 1, 90)
chk(r['id'] == id_buc1, "Bucharest-ring query → id_buc1 (got %s)" % r['id'])

# No match in the ring (descriptor doesn't match Bucharest #2's 0x80
# strongly) — but the closest in-ring slot still wins.  Pick a desc
# that's CLOSEST to id_buc2 to verify ranking inside the ring.
r = P.query(desc_solid(0x80), cell_buc, 1, 0)
chk(r['id'] == id_buc2, "ring query ranks by L1 inside ring (got %s)" % r['id'])

# thresh gating: solid 0x40 vs 0x10 (L1 = (0x40-0x10)*DD = 48*64 = 3072
# out of 16320 max → score ≈ 81%).  thresh=90 → hit=0; thresh=50 → hit=1.
r_low = P.query(desc_solid(0x40), 0, 0, 50)
r_hi  = P.query(desc_solid(0x40), 0, 0, 99)
chk(r_low['hit'] == 1 and r_hi['hit'] == 0,
    "thresh gates hit flag (low=%d, hi=%d)" % (r_low['hit'], r_hi['hit']))

# ---------- 8. clear() ----------
print("[8] clear() returns count and zeroes")
n = P.count()
cleared = P.clear()
chk(cleared == n, "clear returned %d (got %s)" % (n, cleared))
chk(P.count() == 0, "count == 0 after clear")
chk(len(P.list()) == 0, "list empty after clear")

# ---------- 9. stats() truth check ----------
print("[9] stats truth check")
P.clear()
a = P.add(cell0, desc_solid(0), 0.0, 0.0, 0.0)
b = P.add(cell0, desc_solid(1), 0.0, 0.0, 0.0)
P.observe(b)                                           # b now CONFIRMED
P.add(0, None)                                          # rejected (oob)
P.get(255)                                              # bad id
P.query(desc_solid(0), 0, 0, 0)                         # bumps queries
s = P.stats()
chk(s['used'] == 2, "stats.used == 2 (got %s)" % s['used'])
chk(s['tentative'] == 1, "stats.tentative == 1 (got %s)" % s['tentative'])
chk(s['confirmed'] == 1, "stats.confirmed == 1 (got %s)" % s['confirmed'])
chk(s['hwm'] >= 64, "hwm preserved across clear() (got %s)" % s['hwm'])
chk(s['oob_rejected'] >= 1, "oob_rejected advanced (got %s)" % s['oob_rejected'])
chk(s['queries'] >= 1, "queries advanced (got %s)" % s['queries'])
chk(s['capacity'] == 64, "capacity == SENTAI_PLACES_MAX")
chk(s['desc_dim'] == 64, "desc_dim == SENTAI_PLACES_DESC_DIM")

print("=== L3 RESULT: %d PASS / %d FAIL ===" % (PASS, FAIL))
