#!/usr/bin/env python3
"""Stage 1.B — verify sentai.slam.set_class_prior / class_prior on SIM.

Per §18.10 (canonical scenario) + §19 namespace audit, class priors
are the table that drives pseudo-depth in upcoming update_3d.

Test:
  - set valid (class, size) → 0
  - read back via class_prior(class) → matches
  - set 0.0 explicitly → 0 (clears)
  - OOB class_id → -1, no state change
  - NaN/Inf size → -2, no state change
  - negative size → -2
"""
import math, struct, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai

# Init slam (needed for any slam call).
sentai.slam.init(70.0, 320, 240, 16)

# 1. Set valid priors.
print("=SET_RED_CUBE", sentai.slam.set_class_prior(1, 0.30))
print("=SET_BLUE_CYL", sentai.slam.set_class_prior(2, 0.25))
print("=SET_GREEN_BOX", sentai.slam.set_class_prior(3, 0.40))

# 2. Read back.
print("=GET_1", sentai.slam.class_prior(1))
print("=GET_2", sentai.slam.class_prior(2))
print("=GET_3", sentai.slam.class_prior(3))
print("=GET_UNSET", sentai.slam.class_prior(5))  # never set, should be 0.0

# 3. Clear via explicit 0.0
print("=CLEAR_1", sentai.slam.set_class_prior(1, 0.0))
print("=GET_AFTER_CLEAR", sentai.slam.class_prior(1))

# 4. OOB class_id → -1
print("=OOB_NEG", sentai.slam.set_class_prior(-1, 0.30))
print("=OOB_HIGH", sentai.slam.set_class_prior(64, 0.30))
print("=OOB_HIGH2", sentai.slam.set_class_prior(999, 0.30))

# 5. NaN / negative size → -2
print("=NAN", sentai.slam.set_class_prior(2, float("nan")))
print("=INF", sentai.slam.set_class_prior(2, float("inf")))
print("=NEG", sentai.slam.set_class_prior(2, -0.5))

# Verify class 2 unchanged after attempted bad sets.
print("=POST_BAD_GET_2", sentai.slam.class_prior(2))

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

ok = True
def check(name, prefix, expected):
    global ok
    got = parse(prefix)
    if got != expected:
        print(f"FAIL: {name}: expected {expected!r}, got {got!r}")
        ok = False
    else:
        print(f"  OK: {name} = {got}")

# Show full output for diagnostics
for ln in out.splitlines():
    if "=" in ln:
        print(ln)

print()
print("--- Verification ---")
check("set red cube ok",       "=SET_RED_CUBE",     "0")
check("get class 1 = 0.30",    "=GET_1",            "0.3")
check("get class 2 = 0.25",    "=GET_2",            "0.25")
check("get class 3 = 0.40",    "=GET_3",            "0.4")
check("unset class returns 0", "=GET_UNSET",        "0.0")
check("clear via 0.0 OK",      "=CLEAR_1",          "0")
check("after clear is 0",      "=GET_AFTER_CLEAR",  "0.0")
check("OOB negative rejected", "=OOB_NEG",          "-1")
check("OOB high rejected",     "=OOB_HIGH",         "-1")
check("OOB very high rejected","=OOB_HIGH2",        "-1")
check("NaN rejected",          "=NAN",              "-2")
check("Inf rejected",          "=INF",              "-2")
check("Negative rejected",     "=NEG",              "-2")
check("class 2 still 0.25 after bad sets", "=POST_BAD_GET_2", "0.25")

if ok:
    print("\n[test] PASS — class prior set/get + all 5 fault gates fire correctly")
    sys.exit(0)
else:
    print("\n[test] FAIL")
    sys.exit(1)
