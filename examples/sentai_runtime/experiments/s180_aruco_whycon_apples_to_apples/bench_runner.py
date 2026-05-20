#!/usr/bin/env python3
"""s180 — ArUco vs WhyCon apples-to-apples per-stage bench.

Drives the real Coral M7 over /dev/ttyACM0 to call:

  sentai.aruco._test_synth_and_detect(...) + ._stage_cyc()
  sentai.whycon._test_synth(...)            + ._stage_cyc()      (lite, solid disks)
  sentai.whycon._test_synth_krajnik(...)    + ._stage_cyc5()     (W3 off, PnP off)
  sentai.whycon._test_synth_krajnik(...)    + ._stage_cyc5()     (W3 ON, PnP ON)

Per-stage cycles are converted to milliseconds at 800 MHz and
written to `results.txt`.

WBS: OP-S10-W17-T7.
"""

import argparse
import json
import pathlib
import statistics
import sys
import time

import serial


HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_RESULTS = HERE / "results.txt"
DEFAULT_PGM = HERE / "aruco_frame.pgm"
BAUD = 115200
APPEND_CHUNK = 192
REMOTE_PGM = "/aruco_frame.pgm"

# Camera intrinsics for the synthetic 320x240 frame (matches the bench
# defaults already used elsewhere; not load-bearing — only PnP-z
# absolute value depends on these, ratios are intrinsic-invariant).
FX = 240.0
FY = 240.0
CX = 160.0
CY = 120.0

# Marker physical size (metres).  ArUco square side = 0.08; WhyCon
# circle diameter = 0.08 (same physical card mounted on the same
# landing pad in production — picked symmetric here for fair PnP).
ARUCO_MARKER_M = 0.08
WHYCON_DIAMETER_M = 0.08

# Synth frame counts.
N_MARKERS = 4
SYNTH_RADIUS_PX = 18   # WhyCon synth radius (~36 px diameter on 320x240)
ARUCO_SIDE_PX   = 60   # ArUco synth side length

N_ITERS = 7


def _recv_until(ser, terminator, timeout):
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
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


def _parse_print(resp, prefix):
    for line in resp.splitlines():
        s = line.strip()
        if s.startswith(prefix):
            return s
    return None


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
    """Chunked sentai.fs.append upload — same pattern as s179."""
    data = local_pgm.read_bytes()
    n = len(data)
    print("[s180] upload %s -> %s (%d bytes, chunk=%d)" % (
        local_pgm.name, remote_path, n, APPEND_CHUNK))
    _send_line(ser, "_ = sentai.fs.remove(%r)" % remote_path, timeout=3.0)
    t0 = time.time()
    for i in range(0, n, APPEND_CHUNK):
        piece = data[i:i + APPEND_CHUNK]
        resp = _send_line(
            ser, "sentai.fs.append(%r, %s)" % (remote_path, repr(piece)),
            timeout=15.0)
        if "True" not in resp:
            raise RuntimeError("append failed at %d; resp=%r"
                                % (i, resp[-200:]))
        if (i // APPEND_CHUNK) % 25 == 24:
            sys.stdout.write("\r  %d/%d bytes (%.0f%%)"
                              % (i + APPEND_CHUNK, n,
                                 100.0 * (i + APPEND_CHUNK) / n))
            sys.stdout.flush()
    sys.stdout.write("\n")
    dt = time.time() - t0
    r = _send_line(ser,
                    "print('SIZE:%%d' %% sentai.fs.size(%r))" % remote_path,
                    timeout=4.0)
    line = _parse_print(r, "SIZE:")
    if not line or int(line.split(":")[1]) != n:
        raise RuntimeError("upload size mismatch: %r" % r[-200:])
    print("[s180] upload OK %.1f KB/s" % ((n / 1024.0) / dt if dt > 0 else 0))


def setup_detectors(ser):
    print("[s180] sentai.version() + module sanity")
    v = _send_line(ser,
                    "print('VER:' + sentai.version())", timeout=4.0)
    print("  " + (_parse_print(v, "VER:") or "VER: <missing>"))
    for mod in ("aruco", "whycon"):
        h = _send_line(ser,
                        "print('HAS_%s:%%d' %% int(hasattr(sentai, %r)))"
                        % (mod, mod), timeout=3.0)
        print("  " + (_parse_print(h, "HAS_%s:" % mod) or "<missing>"))

    print("[s180] init ArUco")
    _send_line(ser, "sentai.aruco.init()", timeout=4.0)
    _send_line(ser,
                "sentai.aruco.set_intrinsics(%.3f, %.3f, %.3f, %.3f)"
                % (FX, FY, CX, CY), timeout=3.0)
    _send_line(ser,
                "sentai.aruco.set_marker_size(%.4f)" % ARUCO_MARKER_M,
                timeout=3.0)

    print("[s180] WhyCon — set diameter for PnP (intrinsics shared with ArUco)")
    _send_line(ser,
                "sentai.whycon._set_diameter(%.4f)" % WHYCON_DIAMETER_M,
                timeout=3.0)


def bench_aruco_pgm(ser, n_iters):
    """ArUco bench on a REAL Gazebo-rendered PGM with 4 markers.

    Workaround for BUG #74 (synth detection broken post-T18).  The
    PGM was captured in s175 (SIM Gazebo, clean render) and is
    known-good for ArUco detection (s175 + s176 successfully ran
    full PnP on it).  This gives us per-stage timing that includes
    realistic decode + PnP costs.
    """
    print("\n[s180] ArUco — _test_pgm(%s), %d iters" % (REMOTE_PGM, n_iters))
    stages = []
    n_dets = []
    for i in range(n_iters):
        _send_line(ser, "sentai.aruco.clear()", timeout=3.0)
        r = _send_line(
            ser,
            "n = sentai.aruco._test_pgm(%r); "
            "s = sentai.aruco._stage_cyc(); "
            "print('AR:%d:%%d:%%d:%%d:%%d:%%d:%%d' %% (n, s[0], s[1], s[2], s[3], s[4]))"
            % (REMOTE_PGM, i),
            timeout=15.0,
        )
        line = _parse_print(r, "AR:%d:" % i)
        if not line:
            print("  iter %d: no AR line; raw=%r" % (i, r[-200:]))
            continue
        parts = line.split(":")
        try:
            n = int(parts[2])
            t = [int(parts[3+k]) for k in range(5)]
        except (IndexError, ValueError):
            print("  iter %d: malformed %r" % (i, line))
            continue
        print("  iter %d: n=%d  thresh=%d  flood=%d  quad=%d  decode=%d  pnp=%d"
              % (i, n, t[0], t[1], t[2], t[3], t[4]))
        stages.append(t)
        n_dets.append(n)
    return stages, n_dets


def bench_whycon(ser, synth_fn, w3, n_iters, label):
    print("\n[s180] WhyCon — %s, %d iters" % (label, n_iters))
    _send_line(ser, "sentai.whycon._set_concentric(%d)" % (1 if w3 else 0),
                timeout=3.0)
    stages = []
    n_dets = []
    for i in range(n_iters):
        r = _send_line(
            ser,
            "n = sentai.whycon.%s(%d, %d); "
            "s = sentai.whycon._stage_cyc5(); "
            "print('WC:%d:%%d:%%d:%%d:%%d:%%d:%%d' %% (n, s[0], s[1], s[2], s[3], s[4]))"
            % (synth_fn, N_MARKERS, SYNTH_RADIUS_PX, i),
            timeout=15.0,
        )
        line = _parse_print(r, "WC:%d:" % i)
        if not line:
            print("  iter %d: no WC line; raw=%r" % (i, r[-200:]))
            continue
        parts = line.split(":")
        try:
            n = int(parts[2])
            t = [int(parts[3+k]) for k in range(5)]
        except (IndexError, ValueError):
            print("  iter %d: malformed %r" % (i, line))
            continue
        print("  iter %d: n=%d  A=%d  B=%d  W=%d  W3=%d  PnP=%d"
              % (i, n, t[0], t[1], t[2], t[3], t[4]))
        stages.append(t)
        n_dets.append(n)
    return stages, n_dets


def median_stages(stages):
    if not stages:
        return None
    n = len(stages[0])
    return [int(statistics.median([s[k] for s in stages])) for k in range(n)]


def cyc_to_ms(c):
    return c / 800000.0  # M7 @ 800 MHz


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--iters", type=int, default=N_ITERS)
    ap.add_argument("--results", default=str(DEFAULT_RESULTS))
    ap.add_argument("--pgm", default=str(DEFAULT_PGM),
                     help="Local PGM (320x240) — uploaded once to FxUser "
                          "and used for the ArUco real-frame bench.")
    args = ap.parse_args()

    print("[s180] open REPL @ %s" % args.port)
    ser = _open_repl(args.port, timeout=5.0)
    try:
        setup_detectors(ser)

        # ArUco bench needs a real frame because BUG #74 makes the
        # synth path return 0 markers post-T18.  Upload the s175
        # Gazebo capture once; reuse for every detect call.
        pgm_path = pathlib.Path(args.pgm)
        if not pgm_path.exists():
            raise FileNotFoundError("missing %s" % pgm_path)
        _repl_upload(ser, pgm_path, REMOTE_PGM)

        ar_st, ar_n = bench_aruco_pgm(ser, args.iters)
        wc_lite_st, wc_lite_n = bench_whycon(
            ser, "_test_synth", w3=False, n_iters=args.iters,
            label="lite (solid disks, no W3, no PnP)")
        wc_kraj_st, wc_kraj_n = bench_whycon(
            ser, "_test_synth_krajnik", w3=False, n_iters=args.iters,
            label="Krajnik pattern (no W3, no PnP)")
        wc_full_st, wc_full_n = bench_whycon(
            ser, "_test_synth_krajnik", w3=True, n_iters=args.iters,
            label="Krajnik pattern + W3 + PnP (production)")

        ar_med   = median_stages(ar_st)
        wcl_med  = median_stages(wc_lite_st)
        wck_med  = median_stages(wc_kraj_st)
        wcf_med  = median_stages(wc_full_st)

        report = []
        report.append("# s180 — ArUco vs WhyCon apples-to-apples per-stage bench")
        report.append("# Build: real M7 @ 800 MHz, sentai_runtime")
        report.append("# Generated: %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
        report.append("# Frame: 320x240 synth, %d markers per frame" % N_MARKERS)
        report.append("# Iters: %d  (median reported)" % args.iters)
        report.append("")
        report.append("## Per-stage medians (cycles @ 800 MHz → ms)")
        report.append("")
        report.append("### ArUco production")
        if ar_med:
            tags = ("thresh", "flood", "quad", "decode", "pnp")
            for tag, c in zip(tags, ar_med):
                report.append("aruco.%-7s = %10d cyc  = %.3f ms"
                              % (tag, c, cyc_to_ms(c)))
            total = sum(ar_med)
            report.append("aruco.TOTAL  = %10d cyc  = %.3f ms"
                          % (total, cyc_to_ms(total)))
        report.append("aruco.n_dets_median = %d (iters: %s)"
                      % (int(statistics.median(ar_n)) if ar_n else -1,
                         ar_n))
        report.append("")
        for label, med, n_list in (
            ("WhyCon-lite  (solid disks, no W3, no PnP)",
                wcl_med, wc_lite_n),
            ("WhyCon       (Krajnik pattern, no W3, no PnP)",
                wck_med, wc_kraj_n),
            ("WhyCon       (Krajnik pattern + W3 + PnP) [PRODUCTION]",
                wcf_med, wc_full_n),
        ):
            report.append("### %s" % label)
            if med:
                tags = ("A", "B", "W", "W3", "PnP")
                for tag, c in zip(tags, med):
                    report.append("  whycon.%-3s = %10d cyc  = %.3f ms"
                                  % (tag, c, cyc_to_ms(c)))
                total = sum(med)
                report.append("  whycon.TOT = %10d cyc  = %.3f ms"
                              % (total, cyc_to_ms(total)))
            report.append("  whycon.n_dets_median = %d (iters: %s)"
                          % (int(statistics.median(n_list)) if n_list else -1,
                             n_list))
            report.append("")

        # JSON sidecar for downstream tooling.
        json_blob = {
            "aruco": {"median": ar_med, "n_dets": ar_n,
                       "all": ar_st},
            "whycon_lite": {"median": wcl_med, "n_dets": wc_lite_n,
                             "all": wc_lite_st},
            "whycon_krajnik_no_w3_no_pnp": {"median": wck_med,
                                              "n_dets": wc_kraj_n,
                                              "all": wc_kraj_st},
            "whycon_production": {"median": wcf_med, "n_dets": wc_full_n,
                                   "all": wc_full_st},
        }
        json_path = HERE / "results.json"
        with json_path.open("w") as f:
            json.dump(json_blob, f, indent=2)

        text = "\n".join(report) + "\n"
        print("\n=== s180 SUMMARY ===\n" + text)
        with open(args.results, "w") as f:
            f.write(text)
        print("[s180] wrote %s + %s" % (args.results, json_path))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
