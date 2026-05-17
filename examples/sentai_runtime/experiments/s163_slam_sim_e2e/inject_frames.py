#!/usr/bin/env python3
"""s163 — minimal SCM1 frame injector for the SIM camera bridge.

Connects to /tmp/sentai_cam.sock and pushes N synthetic 640x480 RGB
frames at the requested cadence.  Used by t_slam_e2e.py to drive the
sentai_prep SLOT_RGB_64 producer from outside sentai_sim — no Gazebo
required.
"""
import os
import socket
import struct
import sys
import time

W, H = 640, 480
MAGIC = 0x53434D31  # 'SCM1'
HDR = struct.Struct('<IIIIII')   # magic, seq, w, h, pix_fmt, payload_bytes


def synth_frame(seq: int) -> bytes:
    # Slowly-varying gradient so consecutive frames differ in bytes —
    # otherwise the bridge's duplicate-frame detection drops them.
    band = bytes(((i + seq) & 0xFF) for i in range(W))
    row_r = band
    row_g = bytes((b ^ 0x55) for b in band)
    row_b = bytes((b ^ 0xAA) for b in band)
    # 1 row = 3 channels interleaved
    row = bytes(c for triple in zip(row_r, row_g, row_b) for c in triple)
    return row * H


def main():
    sock_path = os.environ.get('SENTAI_CAM_SOCK', '/tmp/sentai_cam.sock')
    n = int(os.environ.get('S163_FRAMES', '15'))
    period_s = float(os.environ.get('S163_PERIOD_S', '0.05'))   # 20 Hz default

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(sock_path)
    payload_bytes = W * H * 3
    for i in range(n):
        rgb = synth_frame(i)
        hdr = HDR.pack(MAGIC, i, W, H, 0, payload_bytes)
        s.sendall(hdr + rgb)
        # The bridge sends a REPLY_MAGIC flow snapshot back per frame
        # (28 bytes).  Drain it so the socket doesn't backpressure.
        _ = s.recv(28, socket.MSG_WAITALL)
        time.sleep(period_s)
    s.close()
    print(f"[inject] sent {n} frames @ {1.0/period_s:.1f} Hz")


if __name__ == '__main__':
    sys.exit(main())
