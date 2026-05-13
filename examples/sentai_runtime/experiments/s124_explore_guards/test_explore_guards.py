#!/usr/bin/env python3
"""Stage 3.B — verify sentai.explore guard conditions (no auto-tick budgets).

Driver:
  1. start() → ARM_AT_MARKER
  2. tick() with no inputs set → HOLDS in ARM_AT_MARKER (guards default no-go)
  3. set_arm_ack(1) + tick() → still holds (no marker yet)
  4. set_marker(1) + tick() → TAKEOFF
  5. tick() with alt unknown → holds in TAKEOFF
  6. set_alt(0.5) → still holds (< target 1.0)
  7. set_alt(1.0) + tick() → ESTABLISH_BASELINE
  8. 3 ticks → EXPLORE
  9. set_cells_visited(8) + tick() → RETURN_HOME
 10. set_dist_home(0.5) → holds (> tol 0.30)
 11. set_dist_home(0.1) + tick() → PRECISION_LAND
 12. set_alt(0.15) + tick() → COAST_LAND
 13. set_alt(0.02) + tick() → DONE
 14. Restart, drive to EXPLORE, force timeout via tiny budget → RETURN_HOME
 15. set_thresholds() changes target_alt and re-test transition
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai
e = sentai.explore

# 1. start
e.start("guards")
print("=S1", e.state())

# 2. holds without inputs
print("=S2", e.tick())

# 3. arm only
e.set_arm_ack(1)
print("=S3", e.tick())

# 4. marker too — should transition to TAKEOFF
e.set_marker(1)
print("=S4", e.tick())

# 5. takeoff without alt — holds
print("=S5", e.tick())

# 6. alt below target — still holds
e.set_alt(0.5)
print("=S6", e.tick())

# 7. alt at target — advance
e.set_alt(1.0)
print("=S7", e.tick())

# 8. three ticks for baseline → EXPLORE
e.tick(); e.tick()
print("=S8", e.tick())

# 9. cells_visited reaches budget → RTH
e.set_cells_visited(8)
print("=S9", e.tick())

# 10. dist_home too large — holds
e.set_dist_home(0.5)
print("=S10", e.tick())

# 11. dist_home below tol → PRECISION_LAND
e.set_dist_home(0.1)
print("=S11", e.tick())

# 12. alt drops below safe-land → COAST_LAND
e.set_alt(0.15)
print("=S12", e.tick())

# 13. alt below done threshold → DONE
e.set_alt(0.02)
print("=S13", e.tick())

# 14. EXPLORE timeout — set cells below budget, walk past timeout ticks
e.start("timeout_test")
e.set_arm_ack(1); e.set_marker(1); e.set_alt(1.0)
e.tick(); e.tick(); e.tick(); e.tick(); e.tick()  # 5 ticks land in EXPLORE
print("=TO_PRE", e.state())
# Make EXPLORE timeout immediately by setting timeout to 1 tick
e.set_thresholds(0, 0, 0, 0, 0, 1)   # explore_timeout_ticks=1
print("=TO_AFTER", e.tick())          # state_ticks ≥ 1 → RETURN_HOME
print("=TO_REASON", e.metrics()["abort_reason"])   # 1 = TIMEOUT

# 15. set_thresholds — raise target_alt to 2.0 m, verify TAKEOFF holds at 1.5m
# Move FSM to ABORT first since RETURN_HOME isn't a restartable terminal state
e.abort()
e.start("th_test")
e.set_thresholds(2.0)   # target_alt = 2.0
e.set_arm_ack(1); e.set_marker(1)
e.tick()                # → TAKEOFF
e.set_alt(1.5)          # below new target
print("=TH_HOLD", e.tick())   # holds in TAKEOFF
e.set_alt(2.0)
print("=TH_GO", e.tick())     # advances

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
check("post-start: ARM_AT_MARKER",          "=S1",  "ARM_AT_MARKER")
check("tick with no inputs holds",          "=S2",  "ARM_AT_MARKER")
check("arm_ack only still holds",           "=S3",  "ARM_AT_MARKER")
check("arm+marker → TAKEOFF",               "=S4",  "TAKEOFF")
check("takeoff w/o alt holds",              "=S5",  "TAKEOFF")
check("alt below target holds",             "=S6",  "TAKEOFF")
check("alt ≥ target → ESTABLISH_BASELINE",  "=S7",  "ESTABLISH_BASELINE")
check("baseline budget → EXPLORE",          "=S8",  "EXPLORE")
check("cells_visited ≥ budget → RTH",       "=S9",  "RETURN_HOME")
check("dist_home large holds",              "=S10", "RETURN_HOME")
check("dist_home small → PRECISION_LAND",   "=S11", "PRECISION_LAND")
check("alt below safe_land → COAST_LAND",   "=S12", "COAST_LAND")
check("alt below done_alt → DONE",          "=S13", "DONE")
check("pre-timeout state EXPLORE",          "=TO_PRE",    "EXPLORE")
check("explore timeout → RETURN_HOME",      "=TO_AFTER",  "RETURN_HOME")
check("abort_reason = 1 (TIMEOUT)",         "=TO_REASON", "1")
check("threshold change: 1.5m holds",       "=TH_HOLD",   "TAKEOFF")
check("threshold change: 2.0m goes",        "=TH_GO",     "ESTABLISH_BASELINE")

if ok:
    print("\n[test] PASS — explore guard conditions work")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
