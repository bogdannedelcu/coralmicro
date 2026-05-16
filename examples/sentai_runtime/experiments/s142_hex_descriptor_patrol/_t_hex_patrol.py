# s142 — HexPatrol SIM smoke (without Gazebo).
#
# Verifies the descriptor → H3 cell → L3 places pipeline:
#   1. compute_phog + compute_gist on synthetic image
#   2. quantize 168+64 floats → 64 uint8 bytes
#   3. xy_to_h3 → unique H3 cell per location
#   4. places.add stores it
#   5. places.query finds it back (self-match)
#   6. Distinct seeds → distinct descriptors → distinct cells
#
# Mirrors what s142's full Gazebo mission would do, minus the cf2 flight.
# Full Gazebo mission shipped as mission_explore.py with the same flow.

import sentai
import hex_helpers as H

P = sentai.places
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

print("=== s142 HexPatrol SIM smoke ===")

# Fresh state
P.clear()
chk(P.count() == 0, "clear → count=0")

print("[1] Store home @ origin (seed=0)")
pid_home = H.capture_and_store(seed=0, x=0.0, y=0.0, z=0.0)
chk(pid_home > 0, "home place_id > 0 (got %d)" % pid_home)
print("[2] Store @ (+1.5, 0) (seed=1)")
pid_t1 = H.capture_and_store(seed=1, x=1.5, y=0.0, z=0.0)
chk(pid_t1 > 0, "target1 place_id > 0 (got %d)" % pid_t1)
print("[3] Store @ (-0.5, +1.0) (seed=2)")
pid_t2 = H.capture_and_store(seed=2, x=-0.5, y=1.0, z=0.0)
chk(pid_t2 > 0, "target2 place_id > 0 (got %d)" % pid_t2)

print("[4] Gallery state")
chk(P.count() == 3, "count == 3 (got %d)" % P.count())
chk(pid_home != pid_t1 != pid_t2, "place ids are distinct")

print("[5] Each place has descriptor + position set")
for pid, label in [(pid_home, "home"), (pid_t1, "t1"), (pid_t2, "t2")]:
    p = P.get(pid)
    chk(p is not None, "places.get(%s) returns dict" % label)
    chk(p['desc_set'] == 1, "%s desc_set == 1" % label)
    chk(p['h3_cell'] != 0, "%s has non-zero H3 cell" % label)

print("[6] H3 cells are distinct (spatial separation)")
cells = set()
for pid in (pid_home, pid_t1, pid_t2):
    p = P.get(pid)
    cells.add(p['h3_cell'])
chk(len(cells) == 3, "3 distinct H3 cells (got %d unique)" % len(cells))

print("[7] Descriptors are distinct")
descs = []
for pid in (pid_home, pid_t1, pid_t2):
    d = P.get_desc(pid)
    chk(d is not None and len(d) == 64, "get_desc(%d) returns 64 bytes" % pid)
    descs.append(d)
def l1_dist(a, b):
    return sum(abs(int(x) - int(y)) for x, y in zip(a, b))
d01 = l1_dist(descs[0], descs[1])
d02 = l1_dist(descs[0], descs[2])
d12 = l1_dist(descs[1], descs[2])
print("     L1 dist: home-t1=%d, home-t2=%d, t1-t2=%d" % (d01, d02, d12))
chk(d01 > 0, "home vs t1 descriptor distance > 0")
chk(d02 > 0, "home vs t2 descriptor distance > 0")
chk(d12 > 0, "t1 vs t2 descriptor distance > 0")
chk(min(d01, d02, d12) > 100, "min pairwise distance > 100 (clear separation)")

print("[8] Self-match: query(home_desc, home_cell) → home_id")
m_home = H.self_query(pid_home)
chk(m_home == pid_home, "self_query(home) returns home id (got %d, expected %d)" % (m_home, pid_home))
m_t1 = H.self_query(pid_t1)
chk(m_t1 == pid_t1, "self_query(t1) returns t1 id (got %d)" % m_t1)
m_t2 = H.self_query(pid_t2)
chk(m_t2 == pid_t2, "self_query(t2) returns t2 id (got %d)" % m_t2)

print("[9] Cross-match: query(home_desc) finds home even with no h3 prefilter")
r = P.query(descs[0], 0, 0, 0)
chk(r is not None and r['id'] == pid_home,
    "global query(home_desc) → home_id (got %s)" % (r['id'] if r else None))

print("[10] Repeat capture from same seed → identical descriptor")
# Determinism: re-capture seed=1 (without storing) and check same desc.
img_again = H.hex_image(1)
phog_again = sentai.places.compute_phog(img_again, H.W, H.H)
gist_again = sentai.places.compute_gist(img_again, H.W, H.H)
desc_again = H.quantize(phog_again, gist_again)
det = all(x == y for x, y in zip(desc_again, descs[1]))
chk(det, "re-capturing seed=1 produces identical descriptor")

print("[11] Determinism on places.cell_at (deterministic for fixed inputs)")
cell_a = H.xy_to_h3(0.0, 0.0)
cell_b = H.xy_to_h3(0.0, 0.0)
chk(cell_a == cell_b, "xy_to_h3 deterministic (same input → same cell)")
cell_c = H.xy_to_h3(1.5, 0.0)
chk(cell_c != cell_a, "different xy → different cell (1.5 vs 0)")

print("=== s142 HexPatrol smoke: %d PASS / %d FAIL ===" % (PASS, FAIL))
print("VERDICT: %s" % ("PASS" if FAIL == 0 else "FAIL"))
