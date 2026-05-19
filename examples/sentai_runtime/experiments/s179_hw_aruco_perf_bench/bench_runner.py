#!/usr/bin/env python3
"""s179 — HW ArUco detect perf bench on the Coral Dev Board Micro.

Host driver: pushes a 320x240 grayscale PGM to the FxUser partition
via REPL-chunked `sentai.fs.append`, then drives the REPL to run
`sentai.aruco._test_pgm()` ten times and collect `last_detect_us`
from each pass.

Outputs: min/median/mean/max microseconds per detect pass.  Also
runs a Lane-A T2 sanity check (sentai.version + module
availability for W12 safety / W13 fr / W14 calib).

USB MSC was attempted first (operator preference for speed) but
falls back to REPL chunked because the FxUser partition is empty
of FAT signature on a freshly-flashed board (first 64 bytes of
/dev/sda are zeros).  Once the firmware writes any file via the
FAT layer, MSC mount becomes viable for future iterations.

WBS: OP-S10-W11 (no algorithm change) + companion to OP-S9-W4
(DWT instrumentation preview).
"""

# This script runs on the Linux host.  It uses pyserial to talk
# REPL over /dev/ttyACM0.  The board's MicroPython does NOT have
# pyserial or argparse.

import argparse
import pathlib
import statistics
import sys
import time

import serial


# Per-chunk size for `sentai.fs.append`.  Same value as the diag
# uploader: 192 raw bytes fit in REPL_LINE_MAX=1024 after the
# `repr(bytes)` worst-case expansion (4 chars/byte) plus the call
# overhead.
APPEND_CHUNK = 192


HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_PGM = HERE / "frame.pgm"
DEFAULT_RESULTS = HERE / "results.txt"
BAUD = 115200


def _recv_until(ser, terminator, timeout, echo=False):
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
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    return _recv_until(ser, b"\r\n>>> ", timeout=timeout).decode(
        "utf-8", errors="replace"
    )


def _open_repl(port, timeout=5.0):
    ser = serial.Serial(port, BAUD, timeout=0.1,
                        rtscts=False, xonxoff=False, dsrdtr=False)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"\x03\x03")
    time.sleep(0.2)
    ser.write(b"\r\n")
    _recv_until(ser, b"\r\n>>> ", timeout=timeout)
    _send_line(ser, "import sentai")
    return ser


def _repl_upload(ser, local_pgm, remote_path):
    """Chunked `sentai.fs.append` upload, validated by size-poll checkpoints.

    Mirrors `diag/_host_upload_repl.py:_upload_one` but writes to an
    arbitrary path (not /lib/diag) so we can drop test fixtures on
    the FxUser root.
    """
    data = local_pgm.read_bytes()
    n = len(data)
    print("[s179] uploading %s → %s (%d bytes, chunk=%d)" % (
        local_pgm.name, remote_path, n, APPEND_CHUNK,
    ))
    # Drop any prior copy so size checks are exact.
    _send_line(ser, "_ = sentai.fs.remove(%r)" % remote_path, timeout=3.0)
    t0 = time.time()
    for i in range(0, n, APPEND_CHUNK):
        piece = data[i : i + APPEND_CHUNK]
        resp = _send_line(
            ser,
            "sentai.fs.append(%r, %s)" % (remote_path, repr(piece)),
            timeout=15.0,
        )
        if "True" not in resp:
            raise RuntimeError(
                "append at offset %d did not return True; resp=%r"
                % (i, resp[-300:])
            )
        # Size check every 10 chunks (~2 KB) — earlier than the
        # diag uploader because we send a single large fixture so a
        # lost chunk in the middle would be invisible until the end.
        if (i // APPEND_CHUNK) % 10 == 9:
            chk = _send_line(
                ser,
                "print('LEN:%%d' %% sentai.fs.size(%r))" % remote_path,
                timeout=3.0,
            )
            expected = min(i + APPEND_CHUNK, n)
            if ("LEN:%d" % expected) not in chk:
                raise RuntimeError(
                    "size drift after %d bytes; REPL says: %s"
                    % (expected, chk[-300:])
                )
            sys.stdout.write(
                "\r  ... %d/%d bytes (%.0f%%)" % (
                    expected, n, 100.0 * expected / n,
                )
            )
            sys.stdout.flush()
    # Final size + sync.
    print("")
    final = _send_line(
        ser, "print('FINAL:%%d' %% sentai.fs.size(%r))" % remote_path,
        timeout=5.0,
    )
    line = _parse_print(final, "FINAL:")
    sz = int(line.split(":", 1)[1]) if line else -1
    sync_resp = _send_line(
        ser,
        "print('SYNC:' + repr(sentai.fs.sync())) if "
        "'sync' in dir(sentai.fs) else print('SYNC:NOAPI')",
        timeout=5.0,
    )
    sync_line = _parse_print(sync_resp, "SYNC:") or "SYNC:?"
    dt = time.time() - t0
    print("[s179] upload done: %d bytes in %.1fs (%.1f KB/s); %s" % (
        sz, dt, (sz / 1024.0) / dt if dt > 0 else 0, sync_line,
    ))
    if sz != n:
        raise RuntimeError(
            "size mismatch after upload: board=%d local=%d" % (sz, n)
        )


def _parse_print(resp, prefix):
    """Pull the line that starts with prefix out of a REPL response."""
    for line in resp.splitlines():
        s = line.strip()
        if s.startswith(prefix):
            return s
    return None


def run_kernel_bench(ser):
    """Pull the existing aruco_bench kernel breakdown via diag binding.

    Returns the parsed dict (or None on parse failure).  sentai.diag.
    aruco_bench() runs `aruco_bench_run()` — kernel-level timing for
    pure scan, Bradley adaptive threshold, separable threshold, PXP-
    accelerated threshold, and 3x3 edge — useful for the embedded
    chapter as a sub-stage decomposition.

    PXP path requires BOARD_InitPxp() to have run; on this firmware
    that only happens inside sentai.camera.init() (see memory entry
    [[pxp-init-required]]).  We kick camera.init first so the PXP
    measurement reflects a hot PXP, not the 100M-cycle DWT timeout.
    """
    print("\n[s179] preparing PXP — sentai.camera.init() (needed for PXP path)")
    cam_resp = _send_line(
        ser, "print('CAMINIT:' + repr(sentai.camera.init()))", timeout=10.0,
    )
    cam_line = _parse_print(cam_resp, "CAMINIT:")
    print("  %s" % (cam_line or "CAMINIT: <no reply, PXP may be cold>"))

    print("\n[s179] kernel-level aruco_bench via sentai.diag.aruco_bench()")
    r = _send_line(
        ser,
        "_b = sentai.diag.aruco_bench(); "
        "print('AB:%d:%d:%d:%d:%d:%d:%d' % (_b['w'], _b['h'], _b['scan_us'], "
        "_b['thresh_us'], _b['thresh_bradley_us'], "
        "_b['thresh_pxp_us'], _b['edge_us']))",
        timeout=20.0,
    )
    line = _parse_print(r, "AB:")
    if not line:
        print("  no AB line; raw last 300: %r" % r[-300:])
        return None
    parts = line.split(":")
    try:
        out = {
            "w": int(parts[1]), "h": int(parts[2]),
            "scan_us": int(parts[3]),
            "thresh_naive_us": int(parts[4]),
            "thresh_bradley_us": int(parts[5]),
            "thresh_pxp_us": int(parts[6]),
            "edge_us": int(parts[7]),
        }
    except (IndexError, ValueError):
        print("  malformed AB line %r" % line)
        return None
    print("  %dx%d  scan=%d us  thresh(naive)=%d us  "
          "thresh(bradley)=%d us  thresh(pxp)=%d us  edge=%d us"
          % (out["w"], out["h"], out["scan_us"],
             out["thresh_naive_us"], out["thresh_bradley_us"],
             out["thresh_pxp_us"], out["edge_us"]))
    return out


def t2_sanity(ser):
    print("\n[s179] Lane A T2 — REPL sanity")
    v_resp = _send_line(ser, "print('VERSION:%s' % sentai.version())", timeout=4.0)
    v = _parse_print(v_resp, "VERSION:")
    print("  %s" % (v or "VERSION: <no reply>"))
    if v is None:
        print("  (raw last 300 bytes: %r)" % v_resp[-300:])
    mod_resp = _send_line(
        ser,
        "print('MODS:' + ','.join('%s=%d' % (n, int(hasattr(sentai, n))) "
        "for n in ('safety','fr','calib','aruco','fs','camera','crazy',"
        "'flow','prep','places','servo','health','diag')))",
        timeout=4.0,
    )
    m = _parse_print(mod_resp, "MODS:")
    print("  %s" % (m or "MODS: <no reply>"))
    if m is None:
        print("  (raw last 300 bytes: %r)" % mod_resp[-300:])
    return v, m


def _check_remote_file(ser, remote_path, expect_bytes=None, retries=4):
    """Verify the PGM is visible to sentai.fs after MSC unmount.

    FxUser can need a beat after warm-reset to mount the volume.  We
    poll a few times before giving up.  Returns the reported size, or
    -1 if absent.  Falls back to listdir parsing if sentai.fs.exists
    is not available in this build.
    """
    for attempt in range(retries):
        size_resp = _send_line(
            ser,
            "print('SIZE:' + repr(sentai.fs.size(%r)) if "
            "hasattr(sentai.fs, 'size') else 'SIZE:NOAPI')" % remote_path,
            timeout=3.0,
        )
        line = _parse_print(size_resp, "SIZE:")
        if line is None:
            time.sleep(0.5)
            continue
        body = line.split(":", 1)[1]
        try:
            sz = int(body)
        except ValueError:
            # Could be "NOAPI" or a Python repr() like "False"
            print("  fs.size returned %r (attempt %d/%d)" % (
                body, attempt + 1, retries,
            ))
            time.sleep(0.5)
            continue
        if sz >= 0:
            if expect_bytes is None or sz == expect_bytes:
                print("  fs.size(%r) = %d bytes — OK" % (remote_path, sz))
                return sz
            print("  fs.size(%r) = %d, expected %d — retrying" % (
                remote_path, sz, expect_bytes,
            ))
        else:
            print("  fs.size(%r) = %d (absent, attempt %d/%d)" % (
                remote_path, sz, attempt + 1, retries,
            ))
        time.sleep(0.5)
    return -1


def t4_perf(ser, n_iters=10, marker_id=0, side_px=96):
    """Perf bench using sentai.aruco._test_synth_and_detect.

    Times each call with sentai.rtos.ticks_ms() — millisecond precision,
    adequate for a 20-30 ms per-frame budget.  `sentai_aruco_detect()`
    does not currently update `s_stats.last_detect_us`, so we can't
    use the stats path; ticks_ms wrapping is the closest thing without
    a firmware change (proper sub-ms timing would need a new DWT
    binding, deferred to OP-S9-W4).

    Note: synthetic frame is sharper than a real OV5640 / SIM capture
    (no gradient or noise) and current detector returns 0 markers on
    this geometry post the T18 cv2-parity changes — so detect cost
    here is the "full multi-scale threshold + contour search, find-
    nothing" upper-floor.  Real-camera frames with detected markers
    add the cost of corner refine + IPPE PnP per marker (~1-2 ms each).
    """
    print("\n[s179] Lane A T4 — ArUco detect perf bench (%d iterations,"
          " synth marker_id=%d side_px=%d)" % (n_iters, marker_id, side_px))
    init_resp = _send_line(
        ser, "print('INIT:%d' % sentai.aruco.init())", timeout=5.0
    )
    print("  %s" % (_parse_print(init_resp, "INIT:") or "INIT: <no reply>"))
    _send_line(ser, "sentai.aruco.set_intrinsics(170.0, 170.0, 160.0, 120.0)",
               timeout=3.0)
    _send_line(ser, "sentai.aruco.set_marker_size(0.094)", timeout=3.0)
    # Warmup (also confirms the binding works and detector doesn't crash).
    w_resp = _send_line(
        ser,
        "a=sentai.rtos.ticks_ms(); "
        "n=sentai.aruco._test_synth_and_detect(%d, %d, 0); "
        "b=sentai.rtos.ticks_ms(); "
        "print('WARMUP:%%d:%%d' %% (n, b-a))" % (marker_id, side_px),
        timeout=15.0,
    )
    warmup = _parse_print(w_resp, "WARMUP:")
    print("  %s" % (warmup or "WARMUP: <no reply>"))
    if warmup is None:
        print("  ABORT: no warmup reply.")
        print("  Raw last 300 bytes: %r" % w_resp[-300:])
        return [], []
    parts = warmup.split(":")
    try:
        warmup_n = int(parts[1])
    except (IndexError, ValueError):
        print("  ABORT: malformed warmup line %r" % warmup)
        return [], []
    if warmup_n < 1:
        print("  CAVEAT: warmup detected %d markers (expected >=1)." % warmup_n)
        print("  Detector regression on synth pattern post T18 (separate from")
        print("  timing).  Timing below is upper-floor: full multi-scale")
        print("  threshold + contour search, no PnP cost included.")
    times = []
    dets = []
    for i in range(n_iters):
        _send_line(ser, "sentai.aruco.clear()", timeout=3.0)
        r = _send_line(
            ser,
            "a=sentai.rtos.ticks_ms(); "
            "n=sentai.aruco._test_synth_and_detect(%d, %d, 0); "
            "b=sentai.rtos.ticks_ms(); "
            "print('IT:%d:%%d:%%d' %% (n, b-a))" % (marker_id, side_px, i),
            timeout=15.0,
        )
        line = _parse_print(r, "IT:%d:" % i)
        if not line:
            print("  iter %d: no IT line in response, raw=%r" % (i, r[-300:]))
            continue
        # "IT:<i>:<n_dets>:<ms>"
        parts = line.split(":")
        try:
            n = int(parts[2])
            ms = int(parts[3])
        except (IndexError, ValueError):
            print("  iter %d: malformed line %r" % (i, line))
            continue
        times.append(ms)
        dets.append(n)
        print("  iter %d: n_dets=%d, detect_ms=%d" % (i, n, ms))
    return times, dets


def summarize(times, dets, kernel_bench=None):
    out = []
    if kernel_bench:
        out.append("# kernel-bench (sentai.diag.aruco_bench) — single 320x240 Y8 frame")
        out.append("# sub-stage cycle counts converted via M7 @ 800 MHz (800 cyc / us)")
        out.append("kernel.scan_us           = %d   # raw memory scan (BW floor)" % kernel_bench["scan_us"])
        out.append("kernel.thresh_naive_us   = %d   # scalar 7x7 box mean" % kernel_bench["thresh_naive_us"])
        out.append("kernel.thresh_bradley_us = %d   # integral-image (Bradley-Roth)" % kernel_bench["thresh_bradley_us"])
        out.append("kernel.thresh_pxp_us     = %d   # PXP HW scale + CPU compare" % kernel_bench["thresh_pxp_us"])
        out.append("kernel.edge_us           = %d   # Sobel-like 3x3 |dx|+|dy|" % kernel_bench["edge_us"])
        if kernel_bench["thresh_pxp_us"] > 0 and kernel_bench["thresh_naive_us"] > 0:
            speedup = kernel_bench["thresh_naive_us"] / kernel_bench["thresh_pxp_us"]
            out.append("# PXP vs naive speedup       = %.1fx" % speedup)
        out.append("")
    if not times:
        out.append("# end-to-end detect (sentai_aruco_detect, synth 320x240 frame)")
        out.append("FAIL: no iterations completed")
        return "\n".join(out)
    out.append("# end-to-end sentai_aruco_detect (synth marker → full multi-scale")
    out.append("# threshold + contour search; PnP cost NOT included because synth")
    out.append("# detection currently returns 0 markers post T18 — see CAVEAT.")
    out.append("# Timing source: sentai.rtos.ticks_ms() (millisecond precision).")
    out.append("n_iters         = %d" % len(times))
    out.append("times_ms        = %s" % times)
    out.append("dets            = %s" % dets)
    out.append("min_ms          = %d" % min(times))
    out.append("median_ms       = %d" % int(statistics.median(times)))
    out.append("mean_ms         = %.1f" % statistics.mean(times))
    out.append("max_ms          = %d" % max(times))
    out.append("stdev_ms        = %.1f" % (statistics.stdev(times) if len(times) > 1 else 0.0))
    fps = 1000.0 / statistics.median(times) if statistics.median(times) > 0 else 0
    budget_30fps = (statistics.median(times) / 33.3) * 100.0
    out.append("equivalent_fps  = %.1f (median)" % fps)
    out.append("30fps_budget    = %.0f%% of 33.3 ms frame slot" % budget_30fps)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--pgm", default=str(DEFAULT_PGM),
                     help="local PGM to upload (320x240 grayscale)")
    ap.add_argument("--remote-name", default="aruco_test.pgm",
                     help="filename written on FxUser partition")
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--results", default=str(DEFAULT_RESULTS))
    ap.add_argument("--upload-pgm", action="store_true",
                     help="also upload the PGM (default: skip; bench uses "
                          "synth frames so no FS dependency).")
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--side-px", type=int, default=80)
    args = ap.parse_args()

    local_pgm = pathlib.Path(args.pgm)
    if not local_pgm.exists():
        print("ERROR: %s not found" % local_pgm, file=sys.stderr)
        sys.exit(1)

    # sentai.fs.* writes via FxUser FAT; remote path is the volume root.
    remote_path = "/%s" % args.remote_name

    print("[s179] opening REPL for sanity + bench")
    ser = _open_repl(args.port, timeout=5.0)

    # File upload is no longer required for the perf bench: we drive
    # `sentai.aruco._test_synth_and_detect` which builds the frame
    # in C SRAM, so no FS dependency.  --upload-pgm still works for
    # offline inspection (verifies the chunked upload path itself).
    if args.upload_pgm:
        _repl_upload(ser, local_pgm, remote_path)
    try:
        version, mods = t2_sanity(ser)
        kernel_bench = run_kernel_bench(ser)
        times, dets = t4_perf(
            ser, args.iters,
            marker_id=args.marker_id, side_px=args.side_px,
        )
        summary = summarize(times, dets, kernel_bench)
        print("\n=== s179 SUMMARY ===")
        print(summary)
        with open(args.results, "w") as f:
            f.write("# s179 HW ArUco detect perf bench (real M7 @ 800 MHz)\n")
            f.write("# Generated: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            f.write("# Board: 1fc9:c0a1 (Coral Dev Board Micro)\n")
            f.write("# %s\n" % version)
            f.write("# %s\n" % mods)
            f.write("# synth frame: marker_id=%d side_px=%d (no FS dep)\n" % (
                args.marker_id, args.side_px,
            ))
            f.write("\n")
            f.write(summary + "\n")
        print("\n[s179] results saved to %s" % args.results)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
