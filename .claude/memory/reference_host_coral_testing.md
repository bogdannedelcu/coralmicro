---
name: Host-side Coral USB Accelerator testing (Linux x86)
description: Python 3.9 venv + pycoral + libedgetpu.so can run any .tflite on a separately-connected Coral USB Accelerator — faster iteration than RT1176 round-trips for protocol-level investigation
type: reference
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
Iterate ~100× faster than RT1176 reflash+test when investigating TPU protocol /
USB / libedgetpu behaviour.  Saved us days on the instruction-caching dead-end
investigation.

## Setup (one-shot)

UPDATE 2026-06-26: the box was upgraded — system python is now **3.12 / 3.14**,
`python3.9` is GONE from apt. The legacy coral stack (pycoral + tflite_runtime
2.5 + libedgetpu 16.0) needs py3.6–3.9. Do NOT try `ai-edge-litert` (invoke
fails: "unresolved custom op EdgeTpuDelegateForCustomOp") or `tensorflow-cpu
2.17` (hard segfault) — both are ABI-incompatible with apt libedgetpu 16.0.
Working recipe = fetch a relocatable CPython 3.9 then install the legacy stack:

```bash
# relocatable python 3.9 (no apt, no conda/pyenv on this box)
curl -skL https://github.com/astral-sh/python-build-standalone/releases/download/20240224/cpython-3.9.18+20240224-x86_64-unknown-linux-gnu-install_only.tar.gz | tar xz   # -> ./python/bin/python3.9
./python/bin/python3.9 -m venv /home/bogdan/work/coralmicro/venv-coral39
venv-coral39/bin/pip install --extra-index-url \
    https://google-coral.github.io/py-repo/ pycoral tflite-runtime "numpy<2" pillow
# verify: venv-coral39/bin/python -c "from pycoral.utils.edgetpu import list_edge_tpus; print(list_edge_tpus())"
```

The Coral USB Accelerator can wedge in runtime state (stuck at `18d1:9302`,
invoke fails even with a fresh interpreter); a USBDEVFS_RESET does NOT clear it
— **physical replug** is the fix (drops to DFU `1a6e:089a`, firmware
re-uploads cleanly on next pycoral open). Verified 2026-06-26 running the
headless YOLOv5n p3p4 cover_v1 detectors (s236).

Legacy (pre-2026-06): `python3.9` used to come from apt; venv was `venv-coral`.

Device enumerates as `1a6e:089a` (DFU) pre-pycoral, `18d1:9302` (Google EdgeTPU)
after pycoral auto-uploads apex.bin.  On this Ubuntu host it runs at **USB 3.0
SuperSpeed** (5 Gbit/s via xhci_hcd) — 2.4× faster than RT1176 USB 2.0 HS for
yolo26-class.

## Max-verbose USB/CSR trace

```python
import ctypes
lib = ctypes.CDLL('libedgetpu.so.1.0')
lib.edgetpu_verbosity.argtypes = [ctypes.c_int]
lib.edgetpu_verbosity(10)   # must be called BEFORE make_interpreter

from pycoral.utils.edgetpu import make_interpreter
# ... every AsyncBulkOutTransfer, WriteRegister64, etc. logged to stderr
```

Or via env: `GLOG_v=10 GLOG_logtostderr=1` (apt-built libedgetpu 16.0 supports both).

## Baseline measurements captured 2026-04-24

| Model | Ins/invoke | RT1176 USB2 | x86 USB3 |
|---|---:|---:|---:|
| yolo_1 512² | 362 KB | 13 ms (76 FPS) | 20 ms (51 FPS) |
| yolo26 768×512 | 1228 KB | 90 ms (11 FPS) | 37 ms (27 FPS) |
| MobileNet v1 224 | 220 KB | — | 4 ms (255 FPS) |

Curiosity: for small ins (yolo_1), RT1176 is FASTER than USB3 — SuperSpeed per-packet
overhead hurts small transfers.  For large ins (yolo26), USB3 wins 2.4×.

## Key finding validated here only (not easily on board)

Verbose trace proves **instructions are re-uploaded every invoke** even when
the bitstream pointer host-side is identical (0x1e680000 both invokes for
mobilenet).  Only PARAMETER_CACHING skips re-upload.  See
`paper/coral_hostside.md` for full trace analysis.

## Gotcha

pycoral wheels are Python 3.9 cp39 only on the py-repo; won't work on our
default venv (Python 3.12).  Always use `venv-coral/` (3.9) for Coral work.
`numpy>=2` breaks pybind — pin `numpy<2`.
