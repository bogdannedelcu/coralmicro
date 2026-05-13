#!/usr/bin/env python3
"""Stage 11.D — verify sentai.places HSV embedding round-trip + match.

Driver:
  1. init places at origin
  2. observe 3 distinct cells (offsets 0, 500m, 1000m east)
  3. set_embedding on each with synthetic 128-byte histograms:
       cell A: bin 0 of every spatial cell = 255   (all "red")
       cell B: bin 4 of every spatial cell = 255   (all "cyan")
       cell C: bin 2 of every spatial cell = 255   (mid hue)
  4. embedding(cellA) returns 128 bytes
  5. embedding(cellA) == the bytes we wrote
  6. match(emb_A) returns cell A with 100% similarity
  7. match(emb_A, k=3) returns 3 cells ordered by similarity
  8. match(emb_A_noisy) (one bin off) returns cell A with ≤100% but >50%
  9. set_embedding rejects wrong-length input (-2)
 10. set_embedding rejects invalid cell (-1)
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai

sentai.places.init(40.689167, -74.044444, 1.0, 9)

# 3 distinct cells via increasing easting (res=9 cell edge ~174m → 500m steps)
cA = sentai.places.cell(0.0, 0.0)
cB = sentai.places.cell(500.0, 0.0)
cC = sentai.places.cell(1000.0, 0.0)
print("=DISTINCT_CELLS", cA != cB and cB != cC and cA != cC)

# observe to register the cells in the gallery
sentai.places.observe(cA, 1)
sentai.places.observe(cB, 2)
sentai.places.observe(cC, 3)

# Build synthetic histograms (16 spatial cells × 8 bins = 128 bytes).
# Each spatial cell has all weight in one hue bin.  Use bytes() with
# a generator expression — single line, REPL-safe.
embA = bytes(255 if (i % 8) == 0 else 0 for i in range(128))
embB = bytes(255 if (i % 8) == 4 else 0 for i in range(128))
embC = bytes(255 if (i % 8) == 2 else 0 for i in range(128))

print("=SET_A", sentai.places.set_embedding(cA, embA))
print("=SET_B", sentai.places.set_embedding(cB, embB))
print("=SET_C", sentai.places.set_embedding(cC, embC))

# read back
got = sentai.places.embedding(cA)
print("=GOT_LEN", len(got))
print("=GOT_EQ_SET", got == embA)

# match emb_A — should rank cA first at 100%
m = sentai.places.match(embA, 3)
print("=MATCH_LEN", len(m))
print("=MATCH_TOP_CELL_IS_A", m[0][0] == cA)
print("=MATCH_TOP_SIM", m[0][1])

# match a noisy version (one bin smeared per spatial cell) — A still wins.
# Each spatial cell still has dominant energy in bin 0 (240/255) plus a tail
# in bin 1 (60/255).  Bhattacharyya with the clean A should be high.
print("=PROBE_NOISY_BUILD", "start")
noisy = bytes(240 if (i % 8) == 0 else (60 if (i % 8) == 1 else 0) for i in range(128))
print("=PROBE_NOISY_LEN", len(noisy))
m2 = sentai.places.match(noisy, 3)
print("=PROBE_M2", m2)
print("=NOISY_TOP_CELL_IS_A", m2[0][0] == cA)
print("=NOISY_TOP_SIM_RANGE", 50 < m2[0][1] <= 100)

# fault gates
print("=BAD_LEN", sentai.places.set_embedding(cA, b"\\x00" * 16))   # wrong len
print("=BAD_CELL", sentai.places.set_embedding(0, embA))            # invalid cell

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
check("3 cells distinct",                  "=DISTINCT_CELLS",           "True")
check("set_embedding A returns 0",         "=SET_A",                    "0")
check("set_embedding B returns 0",         "=SET_B",                    "0")
check("set_embedding C returns 0",         "=SET_C",                    "0")
check("embedding len is 128",              "=GOT_LEN",                  "128")
check("embedding roundtrip exact",         "=GOT_EQ_SET",               "True")
check("match returned 3 rows",             "=MATCH_LEN",                "3")
check("match top is cell A",               "=MATCH_TOP_CELL_IS_A",      "True")
check("match A self-similarity 100%",      "=MATCH_TOP_SIM",            "100")
check("noisy match still ranks A first",   "=NOISY_TOP_CELL_IS_A",      "True")
check("noisy match sim within 50-100",     "=NOISY_TOP_SIM_RANGE",      "True")
check("wrong-length rejected (-2)",        "=BAD_LEN",                  "-2")
check("invalid cell rejected (-1)",        "=BAD_CELL",                 "-1")

if ok:
    print("\n[test] PASS — places embedding + match work")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
