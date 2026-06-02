#!/usr/bin/env python3
"""Persistent PyCoral TPU helper for SentAI SIM.

The SIM process keeps PrepTask/InferTask and all mission-facing APIs in C++.
This helper owns the host TensorFlow Lite interpreter plus EdgeTPU delegate, so
custom ops and Detection_PostProcess remain handled by TFLite/libedgetpu.
"""

from __future__ import annotations

import os
import socket
import sys
import time
from pathlib import Path

import numpy as np
from pycoral.utils.edgetpu import make_interpreter


_INTERP = {}


def _tf_type(dtype) -> int:
    if dtype == np.float32:
        return 1
    if dtype == np.int32:
        return 2
    if dtype == np.uint8:
        return 3
    if dtype == np.int64:
        return 4
    if dtype == np.int8:
        return 9
    if dtype == np.int16:
        return 7
    return 0


def _tensor_meta(detail) -> tuple[int, int, list[int], float, int]:
    shape = [int(x) for x in detail["shape"]]
    q = detail.get("quantization", (0.0, 0))
    return (_tf_type(detail["dtype"]), int(np.prod(shape)) * np.dtype(detail["dtype"]).itemsize,
            shape, float(q[0]), int(q[1]))


def _out_line(idx: int, detail) -> str:
    t, nbytes, shape, scale, zp = _tensor_meta(detail)
    dims = shape + [0] * (4 - len(shape))
    return ("OUT %d %d %d %d %d %d %d %d %.9g %d\n" %
            (idx, t, nbytes, len(shape), dims[0], dims[1], dims[2], dims[3],
             scale, zp))


def _read_exact(f, n: int) -> bytes:
    chunks = []
    left = n
    while left:
        b = f.read(left)
        if not b:
            raise EOFError("short read from SIM")
        chunks.append(b)
        left -= len(b)
    return b"".join(chunks)


def _load(slot: int, path: str, w) -> None:
    print("CMD LOAD slot=%d path=%s" % (slot, path), flush=True)
    interp = make_interpreter(path)
    print("CMD LOAD made_interpreter slot=%d" % slot, flush=True)
    interp.allocate_tensors()
    print("CMD LOAD allocated slot=%d" % slot, flush=True)
    _INTERP[slot] = interp
    inp = interp.get_input_details()[0]
    t, nbytes, shape, scale, zp = _tensor_meta(inp)
    dims = shape + [0] * (4 - len(shape))
    outs = interp.get_output_details()
    w.write(("OK LOAD %d %d %d %d %d %d %d %d %.9g %d %d\n" %
             (slot, t, nbytes, len(shape), dims[0], dims[1], dims[2], dims[3],
              scale, zp, len(outs))).encode())
    for i, out in enumerate(outs):
        w.write(_out_line(i, out).encode())
    w.flush()
    print("CMD LOAD done slot=%d outputs=%d" % (slot, len(outs)), flush=True)


def _invoke(slot: int, nbytes: int, r, w) -> None:
    print("CMD INVOKE slot=%d nbytes=%d" % (slot, nbytes), flush=True)
    interp = _INTERP.get(slot)
    if interp is None:
        _read_exact(r, nbytes)
        w.write(b"ERR no_interpreter\n")
        w.flush()
        return
    inp = interp.get_input_details()[0]
    in_shape = inp["shape"]
    in_dtype = inp["dtype"]
    data = _read_exact(r, nbytes)
    if nbytes != int(np.prod(in_shape)) * np.dtype(in_dtype).itemsize:
        w.write(b"ERR bad_input_size\n")
        w.flush()
        return
    arr = np.frombuffer(data, dtype=in_dtype).reshape(in_shape)
    interp.set_tensor(inp["index"], arr)
    t0 = time.monotonic()
    interp.invoke()
    ms = int((time.monotonic() - t0) * 1000.0 + 0.5)
    outs = interp.get_output_details()
    blobs = []
    for out in outs:
        blobs.append(interp.get_tensor(out["index"]).tobytes())
    total = sum(len(b) for b in blobs)
    w.write(("OK INVOKE %d %d %d\n" % (ms, len(outs), total)).encode())
    for i, out in enumerate(outs):
        w.write(_out_line(i, out).encode())
    for b in blobs:
        w.write(b)
    w.flush()
    print("CMD INVOKE done slot=%d ms=%d outputs=%d total=%d" %
          (slot, ms, len(outs), total), flush=True)


def serve(sock_path: str) -> int:
    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    print("READY", flush=True)
    conn, _ = srv.accept()
    with conn, conn.makefile("rb", buffering=0) as r, conn.makefile("wb", buffering=0) as w:
        while True:
            line = r.readline()
            if not line:
                break
            parts = line.decode().rstrip("\n").split(" ", 2)
            try:
                if parts[0] == "LOAD" and len(parts) == 3:
                    _load(int(parts[1]), parts[2], w)
                elif parts[0] == "INVOKE" and len(parts) == 3:
                    _invoke(int(parts[1]), int(parts[2]), r, w)
                elif parts[0] == "QUIT":
                    w.write(b"OK QUIT\n")
                    w.flush()
                    break
                else:
                    w.write(b"ERR bad_command\n")
                    w.flush()
            except Exception as exc:  # Keep helper alive and report to SIM.
                w.write(("ERR %s\n" % str(exc).replace("\n", " ")).encode())
                w.flush()
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: sentai_tpu_pycoral_server.py SOCKET", file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(serve(sys.argv[1]))
