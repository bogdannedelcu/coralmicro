#!/usr/bin/env python3
"""Dry-run check: exercise the SentaiSim REPL pipe + sentai.places lookup
across a synthetic square trajectory, with no Gazebo / no drone.

Pass criteria:
  - sentai_sim subprocess spawns OK
  - places.init(scale=10) returns 0
  - 16 synthetic poses on a 1.5 m square produce ≥4 distinct H3 cells
    (at res=13, ~3.5 m edge after the ×10 scale lift)
  - places.cells() at the end matches the distinct-set count
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"
assert SIM_BIN.exists(), f"missing {SIM_BIN}"

# Simulate 4 corners of a 1.5 m square, 4 samples each → 16 poses
TRAJECTORY = []
for corner in [(0.0, 0.0), (1.5, 0.0), (1.5, 1.5), (0.0, 1.5)]:
    for k in range(4):
        TRAJECTORY.append((corner[0] + 0.1 * k, corner[1] + 0.1 * k))

DRIVER = ['import sentai',
          'print("=INIT", sentai.places.init(40.689167, -74.044444, 10.0, 13))']

for i, (x, y) in enumerate(TRAJECTORY):
    DRIVER.append(f'c = sentai.places.cell({x:.3f},{y:.3f})')
    DRIVER.append(f'print("=C{i:02d}", hex(c))')
    DRIVER.append(f'sentai.places.observe(c, 0)')

DRIVER.append('print("=CELLS_TOTAL", len(sentai.places.cells()))')
DRIVER.append('print("=DONE")')

proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
proc.stdin.write("\n".join(DRIVER) + "\nexit\n")
proc.stdin.flush()
out, _ = proc.communicate(timeout=10)

def parse(prefix):
    for ln in out.splitlines():
        i = ln.find(prefix)
        if i >= 0:
            return ln[i + len(prefix):].strip()
    return None

cells = set()
for i in range(len(TRAJECTORY)):
    c = parse(f"=C{i:02d}")
    if c:
        cells.add(c)

total = parse("=CELLS_TOTAL")
print(f"[dry_run] init_rc = {parse('=INIT')}")
print(f"[dry_run] {len(TRAJECTORY)} poses → {len(cells)} distinct cells")
print(f"[dry_run] places.cells() total = {total}")
print(f"[dry_run] cell ids: {sorted(cells)}")

ok = (parse("=INIT") == "0"
      and len(cells) >= 4
      and total == str(len(cells)))
print(f"\n[dry_run] {'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
