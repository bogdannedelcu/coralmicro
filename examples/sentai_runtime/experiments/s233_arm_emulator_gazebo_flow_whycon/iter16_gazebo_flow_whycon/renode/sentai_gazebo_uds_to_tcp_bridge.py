#!/usr/bin/env python3
"""Expose latest Gazebo SCM1 UDS camera frame to Renode over TCP.

Gazebo/cf2 produces 640x480 RGB888 frames continuously through
gz_to_uds_bridge.  Renode must not receive every payload eagerly: moving
921 KiB per frame through IronPython starves emulation.  This host helper
keeps the latest accepted SCM1 frame in host memory and replies to Gazebo
immediately; Renode pulls the latest frame only when the guest camera task
requests it through its MMIO doorbell.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import threading
import time


SCM_MAGIC = 0x53434D31
GET_MAGIC = 0x47455431
REPLY_MAGIC = 0x46524C31
SCM_HDR = "<IIIIII"
GET_REQ = "<II"
REPLY_FMT = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_BYTES = struct.calcsize(REPLY_FMT)


def recv_full(conn: socket.socket, n: int) -> bytes | None:
    out = bytearray()
    while len(out) < n:
        chunk = conn.recv(n - len(out))
        if not chunk:
            return None
        out.extend(chunk)
    return bytes(out)


def dummy_reply(seq: int) -> bytes:
    return struct.pack(
        REPLY_FMT,
        REPLY_MAGIC, seq, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0)


class LatestFrame:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.seq = 0
        self.header = b""
        self.payload = b""
        self.frames = 0
        self.kept = 0
        self.dropped = 0
        self.bad = 0

    def store(self, header: bytes, payload: bytes, seq: int) -> None:
        with self.lock:
            self.seq = seq
            self.header = header
            self.payload = payload
            self.frames += 1
            self.kept += 1

    def drop(self) -> None:
        with self.lock:
            self.frames += 1
            self.dropped += 1

    def mark_bad(self) -> None:
        with self.lock:
            self.bad += 1

    def get_if_newer(self, served_seq: int) -> tuple[bytes, bytes, int]:
        with self.lock:
            if self.seq == 0 or self.seq == served_seq:
                return b"", b"", 0
            return self.header, self.payload, self.seq

    def snapshot(self) -> tuple[int, int, int, int, int]:
        with self.lock:
            return self.frames, self.kept, self.dropped, self.bad, self.seq


def downsample_rgb888(payload: bytes,
                      src_w: int,
                      src_h: int,
                      dst_w: int,
                      dst_h: int) -> bytes:
    if src_w == dst_w and src_h == dst_h:
        return payload
    out = bytearray(dst_w * dst_h * 3)
    src = memoryview(payload)
    for y in range(dst_h):
        sy = (y * src_h) // dst_h
        src_row = sy * src_w * 3
        dst_row = y * dst_w * 3
        for x in range(dst_w):
            sx = (x * src_w) // dst_w
            si = src_row + sx * 3
            di = dst_row + x * 3
            out[di:di + 3] = src[si:si + 3]
    return bytes(out)


def tcp_server(latest: LatestFrame, host: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"[uds-tcp] pull server {host}:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        print("[uds-tcp] renode client connected", flush=True)
        with conn:
            while True:
                req = recv_full(conn, struct.calcsize(GET_REQ))
                if req is None:
                    break
                magic, served_seq = struct.unpack(GET_REQ, req)
                if magic != GET_MAGIC:
                    print(f"[uds-tcp] bad pull magic 0x{magic:08x}",
                          flush=True)
                    break
                header, payload, seq = latest.get_if_newer(served_seq)
                if seq == 0:
                    conn.sendall(struct.pack(SCM_HDR, SCM_MAGIC, 0, 0, 0, 0, 0))
                    continue
                conn.sendall(header)
                conn.sendall(payload)
        print("[uds-tcp] renode client disconnected", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--uds", default="/tmp/sentai_emu_cam.sock")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=30233)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--max-fps", type=float, default=15.0,
        help="Maximum Gazebo frames per second retained for Renode; 0 keeps all.")
    parser.add_argument("--out-width", type=int, default=320)
    parser.add_argument("--out-height", type=int, default=240)
    args = parser.parse_args()

    try:
        os.unlink(args.uds)
    except FileNotFoundError:
        pass

    latest = LatestFrame()
    threading.Thread(target=tcp_server,
                     args=(latest, args.tcp_host, args.tcp_port),
                     daemon=True).start()

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.uds)
    os.chmod(args.uds, 0o666)
    srv.listen(1)
    print(f"[uds-tcp] listening {args.uds}", flush=True)

    last_keep_s = 0.0
    min_period_s = 0.0 if args.max_fps <= 0 else 1.0 / args.max_fps

    while True:
        uds_conn, _ = srv.accept()
        print("[uds-tcp] gz client connected", flush=True)
        with uds_conn:
            while True:
                hdr = recv_full(uds_conn, struct.calcsize(SCM_HDR))
                if hdr is None:
                    break
                magic, seq, w, h, fmt, nbytes = struct.unpack(SCM_HDR, hdr)
                if magic != SCM_MAGIC:
                    latest.mark_bad()
                    print(f"[uds-tcp] bad magic seq={seq}", flush=True)
                    break
                payload = recv_full(uds_conn, nbytes)
                if payload is None:
                    break
                now_s = time.monotonic()
                if min_period_s <= 0 or (now_s - last_keep_s) >= min_period_s:
                    out_payload = downsample_rgb888(
                        payload, w, h, args.out_width, args.out_height)
                    out_hdr = struct.pack(
                        SCM_HDR, SCM_MAGIC, seq, args.out_width,
                        args.out_height, fmt, len(out_payload))
                    latest.store(out_hdr, out_payload, seq)
                    last_keep_s = now_s
                else:
                    latest.drop()
                uds_conn.sendall(dummy_reply(seq))
                frames, kept, dropped, bad, last_seq = latest.snapshot()
                if frames % 30 == 0:
                    print(
                        f"[uds-tcp] frames={frames} seq={seq} "
                        f"kept={kept} dropped={dropped} bad={bad} "
                        f"latest={last_seq}",
                        flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
