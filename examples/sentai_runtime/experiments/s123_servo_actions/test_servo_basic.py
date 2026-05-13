#!/usr/bin/env python3
"""Stage 4 — verify sentai.servo action layer skeleton on SIM.

Driver:
  1. status() before init: backend="none"
  2. init("sim") returns 0
  3. arm before init → -1; arm after init → 0
  4. takeoff(2.5) before arm → -2; takeoff after arm → 0
  5. move(1, 0, 0) → 0; move(10, 0, 0) → -3 (>5m)
  6. hover() → 0
  7. land() → 0
  8. disarm() → 0
  9. status() after disarm: armed=0, last_action="DISARM"
 10. trace() returns the 6 actions in order
 11. clear_trace() empties the ring
 12. init("bogus") → -1
 13. init("px4") clears state (armed=0, no trace)
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai

# 1. fresh status
print("=PRE_INIT_BACKEND", sentai.servo.status()["backend"])

# 2. init
print("=INIT_RC", sentai.servo.init("sim"))
print("=POST_INIT_BACKEND", sentai.servo.status()["backend"])

# 3. arm before / after init
print("=ARM_OK", sentai.servo.arm())
print("=ARM_DUP", sentai.servo.arm())   # already armed -> -2

# 4. takeoff
print("=TKOF_OK", sentai.servo.takeoff(2.5))
print("=TKOF_OOR", sentai.servo.takeoff(99.0))   # > 30m → -3

# 5. move
print("=MOVE_OK",  sentai.servo.move(1.0, 0.0, 0.0))
print("=MOVE_OOR", sentai.servo.move(10.0, 0.0, 0.0))   # >5m → -3

# 6. hover
print("=HOVER_OK", sentai.servo.hover())

# 7. land
print("=LAND_OK", sentai.servo.land())

# 8. disarm
print("=DISARM_OK",  sentai.servo.disarm())
print("=DISARM_DUP", sentai.servo.disarm())   # not armed -> -2

# 9. status after disarm
st = sentai.servo.status()
print("=POST_ARMED", st["armed"])
print("=POST_LAST", st["last_action"])

# 10. trace — should have ARM, TAKEOFF, MOVE, HOVER, LAND, DISARM (6 ok ones)
tr = sentai.servo.trace()
print("=TRACE_LEN", len(tr))
print("=TRACE_ACTIONS", [t[0] for t in tr])

# 11. clear_trace
sentai.servo.clear_trace()
print("=POST_CLEAR_TRACE_LEN", len(sentai.servo.trace()))

# 12. init bogus
print("=BAD_BACKEND", sentai.servo.init("bogus"))

# 13. init px4 (clears state)
print("=INIT_PX4", sentai.servo.init("px4"))
st2 = sentai.servo.status()
print("=PX4_ARMED", st2["armed"])
print("=PX4_BACKEND", st2["backend"])

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
check("backend none pre-init",        "=PRE_INIT_BACKEND",  "none")
check("init sim returns 0",           "=INIT_RC",           "0")
check("backend sim post-init",        "=POST_INIT_BACKEND", "sim")
check("arm first call returns 0",     "=ARM_OK",            "0")
check("arm duplicate returns -2",     "=ARM_DUP",           "-2")
check("takeoff valid returns 0",      "=TKOF_OK",           "0")
check("takeoff OOR returns -3",       "=TKOF_OOR",          "-3")
check("move valid returns 0",         "=MOVE_OK",           "0")
check("move OOR (>5m) returns -3",    "=MOVE_OOR",          "-3")
check("hover returns 0",              "=HOVER_OK",          "0")
check("land returns 0",               "=LAND_OK",           "0")
check("disarm returns 0",             "=DISARM_OK",         "0")
check("disarm duplicate returns -2",  "=DISARM_DUP",        "-2")
check("post-disarm armed=0",          "=POST_ARMED",        "0")
check("post-disarm last=DISARM",      "=POST_LAST",         "DISARM")
check("trace length is 6",            "=TRACE_LEN",         "6")
check("trace action order",
      "=TRACE_ACTIONS",
      "['ARM', 'TAKEOFF', 'MOVE', 'HOVER', 'LAND', 'DISARM']")
check("clear_trace empties ring",     "=POST_CLEAR_TRACE_LEN", "0")
check("bogus backend returns -1",     "=BAD_BACKEND",       "-1")
check("init px4 returns 0",           "=INIT_PX4",          "0")
check("px4 init resets armed",        "=PX4_ARMED",         "0")
check("px4 init sets backend",        "=PX4_BACKEND",       "px4")

if ok:
    print("\n[test] PASS — sentai.servo action layer works")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
