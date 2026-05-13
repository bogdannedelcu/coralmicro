#!/usr/bin/env python3
"""End-to-end wire-format test for the anchor shim.

Spawns sentai_sim, sends a fake pose packet over UDS, asks the SIM to
read sentai.flow.anchor_pose(), checks the values match what was sent.

Independent of cv2 / Gazebo — pure struct.pack test of the shim.
"""
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"
RECV_UDS = "/tmp/sentai_aruco_pose_recv.sock"

PUB_MAGIC = 0x41524332
PUB_FMT   = "<II BB H ffff II"
PUB_SIZE  = struct.calcsize(PUB_FMT)

assert SIM_BIN.exists(), f"build SIM first: {SIM_BIN}"

# Wait for any stale UDS to clear.
try: os.unlink(RECV_UDS)
except FileNotFoundError: pass

print(f"[test] spawning {SIM_BIN}")
proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, bufsize=1)

def send_cmd(line):
    proc.stdin.write(line + "\n")
    proc.stdin.flush()

# Boot through REPL banner.
send_cmd("import sentai")
send_cmd('print("=BOOT_OK")')
# Enable anchor mode — this binds the UDS.
send_cmd('print(sentai.flow.mode("anchor"))')

# Wait for the SIM to bind the UDS.
for _ in range(50):
    if Path(RECV_UDS).exists():
        break
    time.sleep(0.05)
else:
    print(f"[test] FAIL: SIM never bound {RECV_UDS}", file=sys.stderr)
    proc.kill()
    sys.exit(1)
print(f"[test] SIM bound {RECV_UDS}")

# Craft + send a fake pose packet.
expected = dict(seq=42, detected=1, n=3, x=1.25, y=-0.5, z=1.75, yaw=0.12,
                detect_us=1234, src_ts=999)
pkt = struct.pack(PUB_FMT, PUB_MAGIC, expected["seq"],
                  expected["detected"], expected["n"], 0,
                  expected["x"], expected["y"], expected["z"], expected["yaw"],
                  expected["detect_us"], expected["src_ts"])
assert len(pkt) == PUB_SIZE
sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
sender.sendto(pkt, RECV_UDS)
print(f"[test] sent pose packet ({PUB_SIZE} B) to SIM")
time.sleep(0.2)

send_cmd('p=sentai.flow.anchor_pose(); print("=POSE", p["detected"], p["num_markers"], p["x"], p["y"], p["z"], p["frame_seq"])')
send_cmd("exit")

# Drain the SIM output.
out, _ = proc.communicate(timeout=5)
print("=== SIM stdout ===")
print(out)

# Verify.
for line in out.splitlines():
    if line.startswith("=POSE"):
        parts = line.split()
        det, n = int(parts[1] == "True"), int(parts[2])
        x, y, z, seq = float(parts[3]), float(parts[4]), float(parts[5]), int(parts[6])
        ok = (det == expected["detected"]
              and n == expected["n"]
              and abs(x - expected["x"]) < 1e-6
              and abs(y - expected["y"]) < 1e-6
              and abs(z - expected["z"]) < 1e-6
              and seq == expected["seq"])
        if ok:
            print("[test] PASS — wire format matches end-to-end")
            sys.exit(0)
        print(f"[test] FAIL: got det={det} n={n} x={x} y={y} z={z} seq={seq}")
        print(f"       expected det={expected['detected']} n={expected['n']} "
              f"x={expected['x']} y={expected['y']} z={expected['z']} seq={expected['seq']}")
        sys.exit(1)

print("[test] FAIL — no =POSE line in SIM output")
sys.exit(2)
