#!/usr/bin/env python3
"""Forward Gazebo SCM1 UDS camera frames to Renode's TCP camera peripheral."""

from __future__ import annotations

import argparse
import os
import socket
import struct
import time


MAGIC = 0x53434D31
REPLY_BYTES = struct.calcsize("<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x")


def recv_full(conn: socket.socket, n: int) -> bytes | None:
    out = bytearray()
    while len(out) < n:
        chunk = conn.recv(n - len(out))
        if not chunk:
            return None
        out.extend(chunk)
    return bytes(out)


def connect_tcp(host: str, port: int, timeout_s: float) -> socket.socket:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return socket.create_connection((host, port), timeout=1.0)
        except OSError as exc:
            last_error = exc
            time.sleep(0.1)
    raise RuntimeError(f"could not connect to {host}:{port}: {last_error}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--uds", default="/tmp/sentai_emu_cam.sock")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=30233)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args()

    try:
        os.unlink(args.uds)
    except FileNotFoundError:
        pass

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.uds)
    os.chmod(args.uds, 0o666)
    srv.listen(1)
    print(f"[uds-tcp] listening {args.uds}", flush=True)

    frames = 0
    while True:
        uds_conn, _ = srv.accept()
        print("[uds-tcp] gz client connected", flush=True)
        try:
            tcp = connect_tcp(args.tcp_host, args.tcp_port, args.timeout_s)
            print(f"[uds-tcp] tcp connected {args.tcp_host}:{args.tcp_port}",
                  flush=True)
            with uds_conn, tcp:
                while True:
                    hdr = recv_full(uds_conn, 24)
                    if hdr is None:
                        break
                    magic, seq, _w, _h, _fmt, nbytes = struct.unpack(
                        "<IIIIII", hdr)
                    if magic != MAGIC:
                        print(f"[uds-tcp] bad magic seq={seq}", flush=True)
                        break
                    payload = recv_full(uds_conn, nbytes)
                    if payload is None:
                        break
                    tcp.sendall(hdr)
                    tcp.sendall(payload)
                    reply = recv_full(tcp, REPLY_BYTES)
                    if reply is None:
                        break
                    uds_conn.sendall(reply)
                    frames += 1
                    if frames % 30 == 0:
                        print(f"[uds-tcp] frames={frames} seq={seq}",
                              flush=True)
        except Exception as exc:
            print(f"[uds-tcp] client error: {exc}", flush=True)
            try:
                uds_conn.close()
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
