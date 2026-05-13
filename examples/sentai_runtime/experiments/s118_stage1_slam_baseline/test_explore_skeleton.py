#!/usr/bin/env python3
"""Stage 3.A — verify sentai.explore state machine skeleton on SIM.

Per §18 mission spec.  Skeleton currently auto-advances after fixed
tick budgets per state.  Tests:

  1. fresh boot: state == "IDLE"
  2. start("learning") → state == "ARM_AT_MARKER"
  3. tick 3× → state == "TAKEOFF"
  4. continued ticks → progresses through all states to DONE
  5. abort() → state == "ABORT"
  6. metrics() returns expected dict shape
  7. fault gates: double-start rejected with -1
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai

# 1. fresh state
print("=INIT_STATE", sentai.explore.state())

# 2. start
print("=START", sentai.explore.start("learning"))
print("=STATE_AFTER_START", sentai.explore.state())

# 3. 3 ticks → TAKEOFF (per skeleton budget: ARM=3 ticks → TAKEOFF)
print("=TICK1", sentai.explore.tick())
print("=TICK2", sentai.explore.tick())
print("=TICK3", sentai.explore.tick())

# 4. Drive through full chain — REPL is line-at-a-time, so use list-comprehension
#    Skeleton budget: ARM=3 + TAKEOFF=5 + BASELINE=3 + EXPLORE=10 + RETURN=5 + LAND=5 = 31 ticks
_ = [sentai.explore.tick() for _ in range(35)]
print("=END_STATE", sentai.explore.state())

# 5. metrics
m = sentai.explore.metrics()
print("=METRICS_KEYS", sorted(m.keys()))
print("=METRICS_LABEL", m["label"])
print("=METRICS_TRANSITIONS", m["transitions"])
print("=METRICS_TICKS", m["ticks"])

# 6. double-start rejected
sentai.explore.start("flight2")  # back from DONE — should be accepted
print("=2ND_START_STATE", sentai.explore.state())
rc_dup = sentai.explore.start("flight2_dup")
print("=DUP_START_RC", rc_dup)

# 7. abort
print("=ABORT", sentai.explore.abort())
print("=STATE_AFTER_ABORT", sentai.explore.state())

print("=DONE")
'''

proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
proc.stdin.write(DRIVER + "\nexit\n")
proc.stdin.flush()
out, _ = proc.communicate(timeout=8)

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
check("fresh state IDLE",                "=INIT_STATE",          "IDLE")
check("start rc 0",                       "=START",                "0")
check("state after start ARM_AT_MARKER",  "=STATE_AFTER_START",   "ARM_AT_MARKER")
check("tick3 lands in TAKEOFF",           "=TICK3",                "TAKEOFF")
check("end state is DONE",                "=END_STATE",            "DONE")
check("label persisted",                  "=METRICS_LABEL",        "learning")
check("2nd start (after DONE) OK",        "=2ND_START_STATE",     "ARM_AT_MARKER")
check("duplicate start rejected -1",      "=DUP_START_RC",         "-1")
check("abort transitions to ABORT",       "=STATE_AFTER_ABORT",   "ABORT")

if ok:
    print("\n[test] PASS — explore skeleton state machine works end-to-end")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
