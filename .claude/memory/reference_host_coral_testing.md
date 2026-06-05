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

```bash
# python3.9 is available on this box (Ubuntu 24.04 apt has libpython3.9)
python3.9 -m venv /home/bogdan/work/coralmicro/venv-coral
/home/bogdan/work/coralmicro/venv-coral/bin/pip install --upgrade pip
/home/bogdan/work/coralmicro/venv-coral/bin/pip install --extra-index-url \
    https://google-coral.github.io/py-repo/ pycoral
/home/bogdan/work/coralmicro/venv-coral/bin/pip install "numpy<2"  # pybind built against numpy 1.x
```

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
