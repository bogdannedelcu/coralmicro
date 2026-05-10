#!/usr/bin/env python3
"""
test_uds_loopback.py — push synthetic 640x480 RGB frames into sentai_sim
                       via /tmp/sentai_cam.sock, verify flow reply parses.

Run sentai_sim in another terminal first:
    ./build-sim/sim/sentai_sim

Then:
    python3 test_uds_loopback.py

Expected: 30 frames sent, 30 replies received with REPLY_MAGIC = 0x46524C31,
last reply has dx_q1000/dy_q1000 != 0 (random shift between consecutive
synthetic frames produces a non-zero phase-corr peak).
"""
import socket
import struct
import sys
import time

MAGIC       = 0x53434D31
REPLY_MAGIC = 0x46524C31
HEADER_FMT  = "<IIIIII"
REPLY_FMT   = "<IIiiIQ"
REPLY_LEN   = struct.calcsize(REPLY_FMT)
W, H        = 640, 480

def _read_exact(sock, n):
    buf = b""
    while len(buf) < n:
        c = sock.recv(n - len(buf))
        if not c: return None
        buf += c
    return buf

def make_frame(seq):
    """Generate a synthetic frame with a clear feature pattern that
    shifts by 1 pixel right per increment of seq."""
    import os
    # 8x8 black/white tiles, shifted by `seq` pixels horizontally.
    rows = []
    shift = seq % 16
    for y in range(H):
        row = bytearray(W * 3)
        for x in range(W):
            tile = ((x + shift) // 8 + y // 8) & 1
            v = 0xFF if tile else 0x10
            row[x*3+0] = v
            row[x*3+1] = v
            row[x*3+2] = v
        rows.append(bytes(row))
    return b"".join(rows)

def main():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect("/tmp/sentai_cam.sock")
    print("[loopback] connected to /tmp/sentai_cam.sock")

    n_frames = 30
    last_reply = None
    t_start = time.monotonic()
    for seq in range(1, n_frames + 1):
        payload = make_frame(seq)
        hdr = struct.pack(HEADER_FMT, MAGIC, seq, W, H, 0, len(payload))
        s.sendall(hdr + payload)

        raw = _read_exact(s, REPLY_LEN)
        if raw is None:
            print(f"[loopback] EOF at seq={seq}")
            return 1
        rmagic, rseq, dx, dy, conf, lat = struct.unpack(REPLY_FMT, raw)
        if rmagic != REPLY_MAGIC:
            print(f"[loopback] bad magic 0x{rmagic:08x}")
            return 1
        last_reply = (rseq, dx, dy, conf, lat)
        if seq % 5 == 0:
            print(f"  seq={rseq:3d}  dx={dx:+5d} dy={dy:+5d} conf={conf:3d} lat={lat:5d} us")

    elapsed = time.monotonic() - t_start
    fps = n_frames / elapsed
    print(f"\n[loopback] sent/recv {n_frames} frames in {elapsed*1000:.1f} ms = {fps:.1f} fps")
    print(f"[loopback] last reply: {last_reply}")

    if last_reply and last_reply[1] == 0 and last_reply[2] == 0 and last_reply[0] > 1:
        print("[loopback] WARN: dx=dy=0 after multiple frames — phase-corr may not be working")
        return 2
    print("[loopback] PASS")
    return 0

if __name__ == "__main__":
    sys.exit(main())
