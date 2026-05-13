#!/usr/bin/env python3
"""s113 P2 fault-model gate verification.

Per embeded.md, the auto-forwarder must reject:
  F1  NaN/Inf in any pose float          -> dropped_nonfinite++
  F2  Out-of-bounds coords (>50 m XY,
      z<-2 or z>+20, |yaw|>3.2)          -> dropped_oob++
  F3  Stale pose (src_ts_ms older than
      500 ms ago)                        -> dropped_stale++

This test spawns sentai_sim, enables auto-forward, then sends three
crafted packets (one per fault class) and verifies the corresponding
counter increments without the forwarder crashing.
"""
import math
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"
RECV_UDS = "/tmp/sentai_aruco_pose_recv.sock"
PUB_MAGIC = 0x41524332
PUB_FMT   = "<II BB H ffff II"
assert SIM_BIN.exists()

def pkt(seq, x, y, z, yaw, src_ts_ms):
    return struct.pack(PUB_FMT, PUB_MAGIC, seq, 1, 1, 0,
                       float(x), float(y), float(z), float(yaw),
                       0, src_ts_ms & 0xFFFFFFFF)

# Fresh state.
try: os.unlink(RECV_UDS)
except FileNotFoundError: pass

sim = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)

def send_cmd(line):
    sim.stdin.write(line + "\n")
    sim.stdin.flush()

send_cmd("import sentai")
send_cmd('print("=BOOT_OK")')
send_cmd('sentai.flow.mode("anchor")')
# Use a tight 5 Hz so each test datagram has plenty of dispatch ticks.
send_cmd('sentai.flow.anchor_forward(5, "px4")')

# Wait for UDS bind.
for _ in range(60):
    if Path(RECV_UDS).exists(): break
    time.sleep(0.05)
else:
    print("FAIL: SIM never bound UDS"); sim.kill(); sys.exit(1)

sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

# F1 — NaN.
print("[test] F1 sending NaN x_m"); sys.stdout.flush()
sender.sendto(pkt(seq=1, x=float('nan'), y=0, z=1.5, yaw=0,
                   src_ts_ms=int(time.monotonic()*1000)), RECV_UDS)
time.sleep(0.5)

# F2 — OOB (x = 999 m).
print("[test] F2 sending OOB x_m=999"); sys.stdout.flush()
sender.sendto(pkt(seq=2, x=999, y=0, z=1.5, yaw=0,
                   src_ts_ms=int(time.monotonic()*1000)), RECV_UDS)
time.sleep(0.5)

# F3 — stale (src_ts_ms way in the past).
print("[test] F3 sending stale pose"); sys.stdout.flush()
sender.sendto(pkt(seq=3, x=0, y=0, z=1.5, yaw=0,
                   src_ts_ms=int(time.monotonic()*1000) - 5000), RECV_UDS)
time.sleep(0.5)

# Normal pose — should be sent.
print("[test] OK sending valid pose"); sys.stdout.flush()
sender.sendto(pkt(seq=4, x=0.5, y=-0.3, z=1.2, yaw=0,
                   src_ts_ms=int(time.monotonic()*1000)), RECV_UDS)
time.sleep(0.5)

send_cmd('print("=STATS", sentai.flow.anchor_forward_stats())')
send_cmd("exit")

out, _ = sim.communicate(timeout=4)
print(out)

# Parse + verify.
import re
for line in out.splitlines():
    if "=STATS" in line:
        # Find the 4-tuple `'health': (a, b, c, d)`.
        m = re.search(r"'health':\s*\(([\d, ]+)\)", line)
        if not m:
            print("FAIL: no health tuple in stats"); sys.exit(2)
        h = tuple(int(x.strip()) for x in m.group(1).split(","))
        nonfinite, oob, stale, hwm = h
        sent_match = re.search(r"'sent_px4':\s*(\d+)", line)
        sent = int(sent_match.group(1)) if sent_match else -1
        print()
        print(f"[test] dropped_nonfinite={nonfinite}  expected >=1")
        print(f"[test] dropped_oob      ={oob}  expected >=1")
        print(f"[test] dropped_stale    ={stale}  expected >=1")
        print(f"[test] sent_px4         ={sent}  expected >=0 (PX4 not running)")
        print(f"[test] stack_hwm_words  ={hwm}")
        if nonfinite >= 1 and oob >= 1 and stale >= 1:
            print("[test] PASS — all 3 fault gates fired")
            sys.exit(0)
        print("[test] FAIL — at least one fault gate did not trip")
        sys.exit(1)

print("FAIL: no =STATS line")
sys.exit(3)
