#!/usr/bin/env python3
"""_host_upload_repl.py — REPL-based chunked fs.write uploader for diag/."""

# ═══════════════════════════════════════════════════════════════════════════
# ⚠  THIS SCRIPT RUNS ON THE LINUX HOST, NOT ON THE SENTAI BOARD.  ⚠
#
# It drives the board's MicroPython REPL over /dev/ttyACM0 (CDC-ACM) from
# your workstation.  It imports `pyserial`, opens a TTY, and does NOT use
# any `sentai.*` module — those only exist in the firmware's embedded
# MicroPython interpreter, never at host Python import time.
#
# The only reason this file lives inside the `diag/` package directory is
# discoverability: it is the tool that pushes every *other* file in this
# directory onto the board.  By convention every file in `diag/*.py` is
# upload target material — this one is the exception.  The `_host_` prefix
# marks it as host-only, and the uploader itself filters files starting
# with `_host_` out of its glob so it never tries to push itself to the
# board (see `_is_host_only` below).
#
# Do NOT add `import _host_upload_repl` or similar in any on-device diag
# module — it will fail to import on the board because `serial`, `argparse`
# and `pathlib` are not available in the firmware's stripped-down
# MicroPython.
# ═══════════════════════════════════════════════════════════════════════════
#
# Why REPL and not HTTP?  The firmware's `/api/write` endpoint is known to
# hang on this board (see `agent/embeded.md`:
#   *"We do not upload files on the board via HTTP, it just doesn't work
#    well.  We use USB or REPL fs.write in chunks."*).  MSC mode (via
# `sentai.usb.drive(1)`) is a second option but requires warm reset +
# littlefs-fuse mount — heavier than this script.
#
# Protocol on the wire (one REPL line at a time):
#   >>> _d = b""
#   >>> _d = _d + b"<chunk 1>"
#   >>> _d = _d + b"<chunk 2>"
#   ...
#   >>> sentai.fs.write("/lib/diag/<name>", _d); print("OK:<name>:%d" % len(_d))
#
# Usage (from examples/sentai_runtime/):
#   python3 diag/_host_upload_repl.py                            # push diag/*.py
#   python3 diag/_host_upload_repl.py --file e_pipeline.py
#   python3 diag/_host_upload_repl.py --file e_pipeline.py --file __init__.py
#   python3 diag/_host_upload_repl.py --port /dev/ttyACM0

import argparse
import pathlib
import sys
import time

# pyserial is a HOST-only dependency.  On-board MicroPython does not have
# this module and should never import it — see banner above.
import serial

# This script sits inside the package dir, so DIAG_DIR is this file's
# parent, not `__file__.parent / "diag"` as in the HTTP uploader at
# `../upload_diag.py`.
DIAG_DIR = pathlib.Path(__file__).parent
REMOTE_BASE = "/lib/diag"
DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200
CHUNK = 48  # raw bytes per append line.  Worst-case `repr(bytes)` escapes a
            # non-printable byte to \xNN (4 chars).  With 48 raw bytes the
            # encoded line tops out at 48*4 + 15 prefix = 207 chars — safely
            # below REPL_LINE_MAX=256 (micropython_task.c:28).


def _is_host_only(path: pathlib.Path) -> bool:
    """True for files that live in diag/ but must NOT be pushed to the board.

    Right now that is only this uploader itself (`_host_upload_repl.py`)
    and anything else a human places under a `_host_` prefix.  Keeping the
    rule simple makes it easy to audit what ends up in `/lib/diag/` on the
    device.
    """
    return path.name.startswith("_host_")


def _recv_until(ser, terminator, timeout, echo=False):
    """Read from serial until `terminator` bytes appear anywhere in buf.

    We can't tail-check: the firmware emits asynchronous log lines
    (e.g. `E:0500:NNN` watchdog kicks) at any point, including right after
    the REPL's `\\r\\n>>> ` reply.  A tail-only check would miss a prompt
    that had scrolled out.  Full-buffer check is safe because none of the
    lines we send contain `\\r\\n>>> ` as a substring.
    """
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
            if echo:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
            if terminator in bytes(buf):
                return bytes(buf)
        else:
            time.sleep(0.01)
    raise TimeoutError(
        "timeout waiting for %r (last 200 bytes: %r)"
        % (terminator, bytes(buf[-200:]))
    )


def _send_line(ser, line, timeout=5.0):
    """Send one REPL line and wait for next '>>> ' prompt.

    The SentAI REPL tokenises on \\r\\n and processes each line synchronously,
    but at 115200 baud and repeated `_d = _d + b'...'` (which allocates a new
    bytes object each time) we sometimes outrun its input buffer and lines
    get dropped silently.  Drain any prior stale bytes, then send, then
    consume the echo + next prompt.
    """
    # Drain anything left over from a prior command's trailing output.
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    return _recv_until(ser, b"\r\n>>> ", timeout=timeout)


def _open_repl(port):
    ser = serial.Serial(port, BAUD, timeout=0.1,
                        rtscts=False, xonxoff=False, dsrdtr=False)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"\x03\x03")
    time.sleep(0.2)
    ser.write(b"\r\n")
    _recv_until(ser, b"\r\n>>> ", timeout=3.0)
    _send_line(ser, "import sentai")
    return ser


def _upload_one(ser, local: pathlib.Path, remote: str):
    data = local.read_bytes()
    n = len(data)
    print("  %s → %s  (%d bytes)" % (local.name, remote, n))
    _send_line(ser, "_d = b''")
    # Verify _d starts empty — otherwise a prior session left state and our
    # size check will look fine but the content will be garbage.
    resp = _send_line(ser, "print('LEN:%d' % len(_d))", timeout=3.0).decode(
        "utf-8", errors="replace"
    )
    if "LEN:0" not in resp:
        raise RuntimeError("_d did not reset: %r" % resp[-200:])
    t0 = time.time()
    for i in range(0, n, CHUNK):
        piece = data[i : i + CHUNK]
        # repr(bytes) gives a MicroPython-parseable literal like b'\x00\x01foo'
        resp = _send_line(ser, "_d = _d + " + repr(piece), timeout=3.0).decode(
            "utf-8", errors="replace"
        )
        # Every ~5 chunks, re-check the running total to catch truncation
        # early rather than at the end.
        if (i // CHUNK) % 5 == 4:
            chk = _send_line(
                ser, "print('LEN:%d' % len(_d))", timeout=3.0
            ).decode("utf-8", errors="replace")
            expected = min(i + CHUNK, n)
            if ("LEN:%d" % expected) not in chk:
                raise RuntimeError(
                    "chunk drift after %d bytes; REPL says: %s"
                    % (expected, chk[-300:])
                )
            print("    … %d/%d bytes OK" % (expected, n))
    # NOTE: we assume /lib/diag exists (this firmware ships it at boot and
    # previous uploads have created it).  Multi-line try/except over the
    # line-at-a-time REPL is fragile — a stray `...` continuation prompt
    # silently consumes the next chunk as part of the block.
    # Write the file and verify size.
    resp = _send_line(
        ser,
        "sentai.fs.write('%s', _d); print('OK:%s:%%d' %% len(_d))"
        % (remote, local.name),
        timeout=15.0,
    ).decode("utf-8", errors="replace")
    marker = "OK:%s:%d" % (local.name, n)
    dt = time.time() - t0
    if marker in resp:
        print("    ✓ wrote %d bytes in %.1fs" % (n, dt))
        return True
    print("    ✗ missing '%s' in reply; last 300 chars:\n%s" % (marker, resp[-300:]))
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument(
        "--file",
        action="append",
        default=[],
        help="upload only these files (repeatable)",
    )
    args = ap.parse_args()

    if not DIAG_DIR.is_dir():
        print("ERROR: %s does not exist" % DIAG_DIR)
        sys.exit(1)

    if args.file:
        files = [DIAG_DIR / f for f in args.file]
        for f in files:
            if not f.exists():
                print("ERROR: %s not found" % f)
                sys.exit(1)
            if _is_host_only(f):
                print("ERROR: %s is a host-side tool and cannot be uploaded"
                      % f.name)
                sys.exit(1)
    else:
        # Default: every *.py in diag/ except host-only tooling (this file
        # and anything else tagged `_host_*`).  Without this filter the
        # uploader would blindly push itself onto the board.
        files = sorted(f for f in DIAG_DIR.glob("*.py") if not _is_host_only(f))
    if not files:
        print("ERROR: no .py files to upload")
        sys.exit(1)

    print("Opening REPL on %s …" % args.port)
    ser = _open_repl(args.port)
    print("REPL ready.  Uploading %d file(s):" % len(files))
    errors = 0
    try:
        for f in files:
            remote = "%s/%s" % (REMOTE_BASE, f.name)
            try:
                if not _upload_one(ser, f, remote):
                    errors += 1
            except Exception as e:
                print("    ✗ %s: %s" % (f.name, e))
                errors += 1
    finally:
        ser.close()

    if errors:
        print("%d file(s) failed." % errors)
        sys.exit(1)
    print("Done.  On device:  import diag")


if __name__ == "__main__":
    main()
