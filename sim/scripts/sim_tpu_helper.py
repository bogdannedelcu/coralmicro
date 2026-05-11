#!/usr/bin/env python3
"""
sim_tpu_helper.py — host-side TPU daemon for sentai_sim (Phase 5).

Architecture:
  sentai_sim (FreeRTOS POSIX) C task → /tmp/sentai_tpu.sock (UDS)
       ↑ length-prefixed framing
       ↓
  THIS SCRIPT (long-running) — uses pycoral or tflite-runtime to:
     - load a .tflite model (Edge TPU compiled or plain quant)
     - invoke inference
     - return raw output bytes + tensor metadata

Why subprocess instead of linking libedgetpu directly into sentai_sim:
  - libedgetpu1-std (apt) ships only the .so, no headers — no easy C link
  - pycoral wraps it cleanly + handles delegate creation + USB hot-plug
  - no per-pixel work in Python (per realtime rule); just marshalling
  - daemon stays warm so we don't pay model-load cost per inference

Wire protocol — request frame:
  uint32_t magic = 0x53545055   ('STPU')
  uint32_t opcode
    1 = LOAD          payload: utf-8 path
    2 = INVOKE        no payload
    3 = GET_OUTPUT    payload: uint32_t idx
    4 = GET_INFO      no payload (returns input dims/dtype/zp/scale)
    5 = SET_INPUT     payload: uint32_t bytes_len, then raw bytes
    6 = OUTPUT_DIMS   payload: uint32_t idx (returns 4-tuple int32 dims)
    7 = OUTPUT_QUANT  payload: uint32_t idx (returns float scale + int32 zp)
    8 = NUM_OUTPUTS   no payload
    9 = OUTPUT_TYPE   payload: uint32_t idx (returns int32 dtype tag)
   10 = OUTPUT_HASH   no payload (uint32_t hash of all outputs)
  uint32_t payload_bytes
  uint8_t  payload[payload_bytes]

Reply frame:
  uint32_t reply_magic = 0x52545055   ('RTPU')
  int32_t  status   (0 = OK, negative = error)
  uint32_t result_bytes
  uint8_t  result[result_bytes]

All multi-byte fields little-endian.

Run:
  source ~/work/coralmicro/venv-coral/bin/activate
  python3 sim/scripts/sim_tpu_helper.py
"""
from __future__ import annotations

import os
import socket
import struct
import sys
import threading
import time
import zlib

SOCK_PATH    = os.environ.get("SENTAI_TPU_SOCK", "/tmp/sentai_tpu.sock")
REQ_MAGIC    = 0x53545055   # 'STPU'
REPLY_MAGIC  = 0x52545055   # 'RTPU'
# Request:  magic uint32, opcode uint32, payload_bytes uint32
# Reply:    magic uint32, status int32 (negative on error), bytes uint32
REQ_FMT      = "<III"
REPLY_FMT    = "<IiI"
HDR_LEN      = struct.calcsize(REQ_FMT)

OP_LOAD          = 1
OP_INVOKE        = 2
OP_GET_OUTPUT    = 3
OP_GET_INFO      = 4
OP_SET_INPUT     = 5
OP_OUTPUT_DIMS   = 6
OP_OUTPUT_QUANT  = 7
OP_NUM_OUTPUTS   = 8
OP_OUTPUT_TYPE   = 9
OP_OUTPUT_HASH   = 10


class TPUEngine:
    """Single-model TPU engine — pycoral or tflite-runtime."""

    def __init__(self):
        self._interpreter = None
        self._delegate    = None
        self._model_path  = None

    def load(self, path: str) -> int:
        """Load a .tflite (Edge TPU compiled) model.  Returns 0 on OK."""
        if not os.path.isfile(path):
            print(f"[tpu] load: file not found: {path}", file=sys.stderr)
            return -1
        try:
            # Lazy import — pycoral / tflite_runtime only available in venv-coral.
            try:
                from pycoral.utils.edgetpu import make_interpreter
                self._interpreter = make_interpreter(path)
                self._mode = "pycoral"
            except Exception as e:
                # Fallback to plain tflite_runtime CPU.
                print(f"[tpu] pycoral unavailable ({e}); falling back to CPU tflite",
                      file=sys.stderr)
                import tflite_runtime.interpreter as tflite
                self._interpreter = tflite.Interpreter(model_path=path)
                self._mode = "tflite-cpu"
            self._interpreter.allocate_tensors()
            self._model_path = path
            i = self._interpreter.get_input_details()[0]
            print(f"[tpu] LOADED {path}", file=sys.stderr)
            print(f"[tpu] mode={self._mode}  input shape={i['shape'].tolist()} "
                  f"dtype={i['dtype'].__name__}", file=sys.stderr)
            return 0
        except Exception as e:
            import traceback; traceback.print_exc()
            print(f"[tpu] load failed: {e}", file=sys.stderr)
            self._interpreter = None
            return -2

    def invoke(self) -> int:
        if not self._interpreter: return -1
        try:
            self._interpreter.invoke()
            return 0
        except Exception as e:
            print(f"[tpu] invoke err: {e}", file=sys.stderr)
            return -3

    def num_outputs(self) -> int:
        if not self._interpreter: return -1
        return len(self._interpreter.get_output_details())

    def output_data(self, idx: int) -> bytes | None:
        if not self._interpreter: return None
        outs = self._interpreter.get_output_details()
        if idx < 0 or idx >= len(outs): return None
        t = self._interpreter.get_tensor(outs[idx]["index"])
        return t.tobytes()

    def output_dims(self, idx: int) -> tuple | None:
        if not self._interpreter: return None
        outs = self._interpreter.get_output_details()
        if idx < 0 or idx >= len(outs): return None
        sh = outs[idx]["shape"].tolist()
        # Pad/truncate to 4 dims (TFLite convention) for stable wire format.
        sh = (sh + [1, 1, 1, 1])[:4]
        return tuple(sh)

    def output_quant(self, idx: int) -> tuple[float, int] | None:
        if not self._interpreter: return None
        outs = self._interpreter.get_output_details()
        if idx < 0 or idx >= len(outs): return None
        q = outs[idx].get("quantization", (0.0, 0))
        # Some pycoral versions return (scale, zp) as a 2-tuple of floats/ints.
        return (float(q[0]), int(q[1]))

    def output_type(self, idx: int) -> int:
        """Return TFLite type enum: 1=float32, 9=int8, 3=uint8, 7=int32."""
        if not self._interpreter: return -1
        import numpy as np
        outs = self._interpreter.get_output_details()
        if idx < 0 or idx >= len(outs): return -1
        d = outs[idx]["dtype"]
        # Map numpy dtype -> TFLite TensorType enum (matches firmware).
        return {
            np.dtype("float32"): 1,
            np.dtype("int32"):   3,   # not standard but consistent w/ firmware
            np.dtype("uint8"):   3,
            np.dtype("int8"):    9,
        }.get(np.dtype(d), 0)

    def set_input(self, data: bytes) -> int:
        if not self._interpreter: return -1
        ins = self._interpreter.get_input_details()
        if not ins: return -1
        import numpy as np
        shape = ins[0]["shape"]
        dtype = ins[0]["dtype"]
        expect = int(np.prod(shape) * np.dtype(dtype).itemsize)
        if len(data) != expect:
            print(f"[tpu] set_input size {len(data)} != expected {expect}",
                  file=sys.stderr)
            return -2
        arr = np.frombuffer(data, dtype=dtype).reshape(shape)
        self._interpreter.set_tensor(ins[0]["index"], arr)
        return 0

    def info(self) -> bytes | None:
        if not self._interpreter: return None
        i = self._interpreter.get_input_details()[0]
        sh = (list(i["shape"]) + [1, 1, 1, 1])[:4]
        # Pack as: int32×4 dims, float scale, int32 zp, int32 dtype-enum
        import numpy as np
        dtype_enum = {
            np.dtype("float32"): 1, np.dtype("uint8"): 3,
            np.dtype("int8"):    9, np.dtype("int32"): 7,
        }.get(np.dtype(i["dtype"]), 0)
        q = i.get("quantization", (0.0, 0))
        return struct.pack("<iiii fii",
                           int(sh[0]), int(sh[1]), int(sh[2]), int(sh[3]),
                           float(q[0]), int(q[1]), int(dtype_enum))

    def output_hash(self) -> int:
        if not self._interpreter: return 0
        h = 0
        for o in self._interpreter.get_output_details():
            t = self._interpreter.get_tensor(o["index"])
            h = zlib.crc32(t.tobytes(), h) & 0xFFFFFFFF
        return h


def read_full(sock, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        c = sock.recv(n - len(buf))
        if not c: return None
        buf += c
    return buf


def reply(sock, status: int, payload: bytes = b"") -> bool:
    hdr = struct.pack(REPLY_FMT, REPLY_MAGIC, status, len(payload))
    try:
        sock.sendall(hdr + payload)
        return True
    except OSError:
        return False


def handle_client(sock, eng: TPUEngine):
    print(f"[tpu] client connected", file=sys.stderr)
    while True:
        h = read_full(sock, HDR_LEN)
        if h is None: break
        magic, opcode, plen = struct.unpack(REQ_FMT, h)
        if magic != REQ_MAGIC:
            print(f"[tpu] bad magic 0x{magic:08x}", file=sys.stderr)
            break
        payload = b""
        if plen > 0:
            payload = read_full(sock, plen)
            if payload is None: break

        if opcode == OP_LOAD:
            path = payload.decode("utf-8", errors="replace")
            if not reply(sock, eng.load(path)): break
        elif opcode == OP_INVOKE:
            if not reply(sock, eng.invoke()): break
        elif opcode == OP_NUM_OUTPUTS:
            n = eng.num_outputs()
            if not reply(sock, 0 if n >= 0 else -1, struct.pack("<i", n)): break
        elif opcode == OP_GET_OUTPUT:
            idx, = struct.unpack("<I", payload[:4])
            d = eng.output_data(idx)
            if d is None:
                if not reply(sock, -1): break
            else:
                if not reply(sock, 0, d): break
        elif opcode == OP_OUTPUT_DIMS:
            idx, = struct.unpack("<I", payload[:4])
            dims = eng.output_dims(idx)
            if dims is None:
                if not reply(sock, -1): break
            else:
                if not reply(sock, 0, struct.pack("<iiii", *dims)): break
        elif opcode == OP_OUTPUT_QUANT:
            idx, = struct.unpack("<I", payload[:4])
            q = eng.output_quant(idx)
            if q is None:
                if not reply(sock, -1): break
            else:
                if not reply(sock, 0, struct.pack("<fi", q[0], q[1])): break
        elif opcode == OP_OUTPUT_TYPE:
            idx, = struct.unpack("<I", payload[:4])
            t = eng.output_type(idx)
            if not reply(sock, 0 if t >= 0 else -1, struct.pack("<i", t)): break
        elif opcode == OP_GET_INFO:
            info = eng.info()
            if info is None:
                if not reply(sock, -1): break
            else:
                if not reply(sock, 0, info): break
        elif opcode == OP_SET_INPUT:
            if not reply(sock, eng.set_input(payload)): break
        elif opcode == OP_OUTPUT_HASH:
            h = eng.output_hash()
            if not reply(sock, 0, struct.pack("<I", h)): break
        else:
            print(f"[tpu] unknown opcode {opcode}", file=sys.stderr)
            if not reply(sock, -99): break
    try: sock.close()
    except Exception: pass
    print(f"[tpu] client disconnected", file=sys.stderr)


def main() -> int:
    try: os.unlink(SOCK_PATH)
    except FileNotFoundError: pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    os.chmod(SOCK_PATH, 0o666)
    srv.listen(4)   # let multiple sentai_sim connect attempts queue
    print(f"[tpu] listening on {SOCK_PATH}", file=sys.stderr)
    eng = TPUEngine()
    while True:
        try:
            cl, _ = srv.accept()
        except Exception as e:
            print(f"[tpu] accept err: {e}", file=sys.stderr)
            time.sleep(0.5)
            continue
        # Single-client at a time (sentai_sim is the only writer).
        try:
            handle_client(cl, eng)
        except Exception as e:
            import traceback; traceback.print_exc(file=sys.stderr)
            print(f"[tpu] handler crashed: {e}; resuming accept loop",
                  file=sys.stderr)
            try: cl.close()
            except Exception: pass


if __name__ == "__main__":
    sys.exit(main())
