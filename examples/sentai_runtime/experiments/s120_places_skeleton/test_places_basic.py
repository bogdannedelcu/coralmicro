#!/usr/bin/env python3
"""Stage 11.B — verify sentai.places skeleton works on SIM.

Round-trip test:
  1. fresh state: info().n == 0, initialized == 0
  2. init(40.689167, -74.044444, scale=1.0, default_res=9)
  3. cell(0, 0) at origin → some H3 index
  4. observe(cell, 56)  → visits=1; classes(cell) = [(56, 1)]
  5. observe(cell, 56) ×2 + observe(cell, 17) → visits=4, hist=[(56,3),(17,1)]
  6. neighbors(cell, 1) → 7 cells (origin + 6 ring)
  7. clear() wipes table; observe new cell after clear works
  8. cell(100, 0, res=9) far enough to land in a different H3 cell than (0,0)
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai
print("=INFO_INIT", sentai.places.info())

# init at Empire State Building, default res 9
rc = sentai.places.init(40.689167, -74.044444, 1.0, 9)
print("=INIT_RC", rc)

# cell at origin
c0 = sentai.places.cell(0.0, 0.0)
print("=CELL0", hex(c0))

# observe
v1 = sentai.places.observe(c0, 56)
print("=OBSERVE1", v1)

# observe more
sentai.places.observe(c0, 56)
sentai.places.observe(c0, 56)
v4 = sentai.places.observe(c0, 17)
print("=OBSERVE4", v4)

print("=CLASSES", sentai.places.classes(c0))
print("=VISITS", sentai.places.visits(c0))

# neighbors
neighbors = sentai.places.neighbors(c0, 1)
print("=NEIGHBORS_LEN", len(neighbors))
print("=NEIGHBORS_INCLUDES_ORIGIN", c0 in neighbors)

# cells list
cells = sentai.places.cells()
print("=CELLS_LEN", len(cells))

# cell at (500m, 0) — at res=9 (cell edge ~174m) 500m east must land elsewhere
c_far = sentai.places.cell(500.0, 0.0)
print("=CELL_FAR_DIFFERS", c_far != c0)

# clear
sentai.places.clear()
print("=POST_CLEAR_VISITS", sentai.places.visits(c0))
print("=POST_CLEAR_INFO_N", sentai.places.info()["n"])

# observe after clear should still work
v_post = sentai.places.observe(c0, 99)
print("=POST_CLEAR_OBSERVE", v_post)

print("=DONE")
'''

proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
proc.stdin.write(DRIVER + "\nexit\n")
proc.stdin.flush()
out, _ = proc.communicate(timeout=10)

def parse(prefix):
    for ln in out.splitlines():
        i = ln.find(prefix)
        if i >= 0:
            return ln[i + len(prefix):].strip()
    return None

# diagnostic dump
for ln in out.splitlines():
    if "=" in ln:
        print(ln)

ok = True
def check(name, prefix, expected):
    global ok
    got = parse(prefix)
    if got != expected:
        print(f"FAIL: {name}: expected {expected!r}, got {got!r}")
        ok = False
    else:
        print(f"  OK: {name}")

print("\n--- Verification ---")
check("init returns 0",                 "=INIT_RC",                "0")
check("first observe returns 1",        "=OBSERVE1",               "1")
check("fourth observe returns 4",       "=OBSERVE4",               "4")
check("classes = [(56,3),(17,1)]",      "=CLASSES",                "[(56, 3), (17, 1)]")
check("visits returns 4",               "=VISITS",                 "4")
check("neighbors length 7",             "=NEIGHBORS_LEN",          "7")
check("neighbors include origin",       "=NEIGHBORS_INCLUDES_ORIGIN", "True")
check("cells list length 1",            "=CELLS_LEN",              "1")
check("far cell differs from origin",   "=CELL_FAR_DIFFERS",       "True")
check("post-clear visits is 0",         "=POST_CLEAR_VISITS",      "0")
check("post-clear info.n is 0",         "=POST_CLEAR_INFO_N",      "0")
check("post-clear observe returns 1",   "=POST_CLEAR_OBSERVE",     "1")

if ok:
    print("\n[test] PASS — sentai.places skeleton works")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
