---
name: REPL upload via sentai.fs.append (build #986)
description: Chunked upload uses sentai.fs.append per line, NOT old _d=_d+chunk accumulator. Old pattern broke after MP interpreter moved from ITCM to SDRAM.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
`diag/_host_upload_repl.py` (build #986+) uses `sentai.fs.append(path, chunk)`
per line.  Defaults: CHUNK=192 raw bytes, REPL_LINE_MAX=1024
(`micropython_task.c`).  Companion runner: `diag/_host_run_with_var.py`
seeds REPL globals (`--set _target_fps=30`) before exec.

**Why:** Old protocol (`_d = _d + b'...'` × N lines, then single
`sentai.fs.write(_d)`) broke after the MP interpreter moved out of
ITCM into `.micropython` (SDRAM).  Per-line GC pauses stretched into
hundreds of ms under SEMC contention; USB CDC RX FIFO overflowed;
pyserial saw "device reports readiness to read but returned no
data".  Fragmented heap + slower instruction fetch = cliff at ~50
chunks.

**How to apply:**
- When chunked upload fails with phantom disconnect, verify
  `_host_upload_repl.py` is the new (`fs.append`-based) version.
- CHUNK and REPL_LINE_MAX must stay paired:
  `CHUNK*4 + ~50 < REPL_LINE_MAX`.  Raise REPL_LINE_MAX FIRST
  before raising CHUNK; rebuild firmware.
- Don't revert to the accumulator pattern unless the MP interpreter
  is moved back to ITCM.

`LfsUserAppendFile(O_APPEND|O_CREAT)` shipped in same build to
`libs/base/filesystem.cc`.  Performance: ~3.5 KB driver in ~11 s
(LFS open/close per chunk dominates).  Acceptable for diag drivers,
NOT for large blobs — use MSC (`sentai.usb.drive(1)`) or HTTP
`/api/raw/...` for models / JPEG dumps.

**Inherited rules from previous version (still hold):**
- `_host_*.py` files in `diag/` are host-only; both uploaders skip
  them automatically.
- Firmware emits async `E:0500:NNN` watchdog log lines that scroll
  past the prompt — terminator match must scan the full buffer,
  not the tail.
