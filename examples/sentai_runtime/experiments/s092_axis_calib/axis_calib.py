"""
s092 — controlled axis calibration for sentai.flow vs cf2 body frame.

Goal: determine the mapping (L0_dx, L0_dy) → (fw_body, lf_body) EMPIRICALLY
by commanding known unit motions and recording flow output.

Protocol:
  1. Takeoff to z=1.0m, hover 3s to settle.
  2. PHASE A — +body_X (forward) at 0.20 m/s for 2.0s.   Expected Δx = +40cm.
  3. Hover 2s.
  4. PHASE B — -body_X back to origin at 0.20 m/s for 2.0s.
  5. Hover 2s.
  6. PHASE C — +body_Y (left) at 0.20 m/s for 2.0s.       Expected Δy = +40cm.
  7. Hover 2s.
  8. PHASE D — -body_Y back to origin at 0.20 m/s for 2.0s.
  9. Land.

During each phase, record (L0_dx, L0_dy) per frame.  Mean over the motion
window gives the per-axis flow signature.  The 4 means (A/B/C/D) form a
linear system that uniquely identifies BODY_XFORM.

Critical assumptions (per drone model.sdf.jinja):
  - Camera offset from CoM = (-0.04, 0, -0.02) — 4cm back, 2cm down.
  - At z=1m above ground, camera lever-arm has negligible effect on
    apparent ground translation (only matters for ROTATIONAL flow).
  - Use yaw=0 throughout (no rotation, isolates translation).

WIND MUST BE DISABLED for this experiment.  Set SENTAI_AXIS_CALIB_NOWIND=1
to skip Gazebo wind effects via SDF override (or manually edit the SDF).
"""
import sys, os, time, struct, threading, socket, math, csv, json
from pathlib import Path

# Reuse REPLY_FMT / FLOW_OUT_SOCK from s091 to avoid drift
sys.path.insert(0, str(Path(__file__).parent.parent / "s091_aruco_lowalt"))

FLOW_OUT_SOCK = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC   = 0x46524C31
REPLY_FMT     = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_SZ      = struct.calcsize(REPLY_FMT)
assert REPLY_SZ == 124

# Test parameters
TARGET_Z      = 1.0
SETTLE_S      = 5.0      # long hover after takeoff — measure bias
MOVE_S        = 4.0      # long motion phase — signal > tilt-transient
HOVER_BETWEEN = 3.0      # let drone settle between phases (no PID tilt)
VEL_MPS       = 0.30     # higher vel → more signal vs DC bias

# Phases — (label, vx_body, vy_body) per MotionCommander commands.
# MotionCommander's start_linear_motion(x, y, z) uses BODY frame (x=forward, y=left).
PHASES = [
    ("A_+X_fw",  +VEL_MPS, 0.0),
    ("B_-X_fw",  -VEL_MPS, 0.0),
    ("C_+Y_lf",  0.0, +VEL_MPS),
    ("D_-Y_lf",  0.0, -VEL_MPS),
]


def flow_reader(stop_evt, records):
    """Background thread reading flow packets — append to records list."""
    sock = None
    for _ in range(20):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(FLOW_OUT_SOCK)
            sock = s
            break
        except Exception:
            time.sleep(0.25)
    if sock is None:
        print(f"[calib] FAIL connect {FLOW_OUT_SOCK}", file=sys.stderr)
        return
    print(f"[calib] flow socket connected", file=sys.stderr)

    buf = b""
    while not stop_evt.is_set():
        try:
            chunk = sock.recv(4096)
            if not chunk:
                time.sleep(0.01); continue
            buf += chunk
            while len(buf) >= REPLY_SZ:
                rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
                fields = struct.unpack(REPLY_FMT, rec)
                magic, seq, dx, dy, conf = fields[:5]
                if magic != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                # We only need timestamps + L0 dx/dy/conf for axis calib
                records.append({
                    "t": time.monotonic(),
                    "seq": seq,
                    "L0_dx": dx, "L0_dy": dy, "L0_conf": conf,
                    # Carry the fused output too in case caller wants it
                    "best_dx": fields[23], "best_dy": fields[24],
                    "best_conf": fields[25], "best_source": fields[26],
                })
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.02)


def main():
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    cflib.crtp.init_drivers()
    uri = "udp://0.0.0.0:19850"  # CrazySim default
    cf = Crazyflie(rw_cache="/tmp/cfcache_calib")
    sync = SyncCrazyflie(uri, cf=cf)
    print("[calib] connecting...", file=sys.stderr)
    sync.open_link()
    time.sleep(2.0)

    # EKF reset (per s090/s091 convention)
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)

    stop_evt = threading.Event()
    records = []
    th = threading.Thread(target=flow_reader, args=(stop_evt, records), daemon=True)
    th.start()
    time.sleep(0.5)

    phase_log = []
    mc = MotionCommander(sync, default_height=TARGET_Z)
    print(f"[calib] takeoff to {TARGET_Z}m", file=sys.stderr)
    mc.take_off(height=TARGET_Z, velocity=0.5)
    time.sleep(SETTLE_S)
    print(f"[calib] settled @ {TARGET_Z}m — bias measurement period", file=sys.stderr)
    # Pre-motion bias: hover STILL for additional SETTLE_S to characterize bias
    t_bias_start = time.monotonic()
    time.sleep(SETTLE_S)
    t_bias_end = time.monotonic()
    phase_log.append({"label": "BIAS_hover", "vx": 0.0, "vy": 0.0,
                      "t_start": t_bias_start, "t_end": t_bias_end})

    for label, vx, vy in PHASES:
        t_start = time.monotonic()
        print(f"[calib] PHASE {label}: vx={vx:+.2f} vy={vy:+.2f} for {MOVE_S}s",
              file=sys.stderr)
        mc.start_linear_motion(vx, vy, 0)
        time.sleep(MOVE_S)
        mc.start_linear_motion(0, 0, 0)
        t_end = time.monotonic()
        phase_log.append({"label": label, "vx": vx, "vy": vy,
                          "t_start": t_start, "t_end": t_end})
        print(f"[calib]    hover gap {HOVER_BETWEEN}s", file=sys.stderr)
        time.sleep(HOVER_BETWEEN)

    mc.land(velocity=0.3)
    time.sleep(2.0)
    stop_evt.set()
    th.join(timeout=1.0)
    sync.close_link()

    # Analysis
    print(f"\n[calib] captured {len(records)} flow records", file=sys.stderr)
    # Per-phase mean L0_dx, L0_dy
    print(f"\n{'phase':<10} {'n':>4} {'mean_dx':>10} {'mean_dy':>10} {'std_dx':>8} {'std_dy':>8} {'med_conf':>8}")
    rows = []
    for p in phase_log:
        # MIDDLE 50% of phase only — drops accel/decel transients where
        # drone is tilting (=> rotational flow contaminates measurement).
        dur = p["t_end"] - p["t_start"]
        t_lo = p["t_start"] + dur * 0.25
        t_hi = p["t_end"] - dur * 0.10
        sub = [r for r in records if t_lo <= r["t"] <= t_hi]
        if not sub:
            print(f"{p['label']:<10}  no data", file=sys.stderr)
            continue
        import statistics as st
        dxs = [r["L0_dx"] for r in sub]
        dys = [r["L0_dy"] for r in sub]
        cfs = [r["L0_conf"] for r in sub]
        mdx = st.mean(dxs); mdy = st.mean(dys)
        sdx = st.stdev(dxs) if len(dxs)>1 else 0
        sdy = st.stdev(dys) if len(dys)>1 else 0
        mcf = st.median(cfs)
        print(f"{p['label']:<10} {len(sub):>4} {mdx:>+10.0f} {mdy:>+10.0f} {sdx:>8.0f} {sdy:>8.0f} {mcf:>8.0f}")
        rows.append({"phase": p["label"], "vx": p["vx"], "vy": p["vy"],
                     "n": len(sub), "mean_L0_dx": mdx, "mean_L0_dy": mdy,
                     "std_L0_dx": sdx, "std_L0_dy": sdy, "med_L0_conf": mcf})

    # Save raw + summary CSV
    out_dir = Path(os.environ.get("SENTAI_DUMP_FRAMES_DIR", "."))
    raw_csv = out_dir / "axis_calib_raw.csv"
    with raw_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(records[0].keys()) if records else ["t"])
        w.writeheader()
        for r in records:
            w.writerow(r)
    print(f"[calib] raw → {raw_csv}", file=sys.stderr)

    sum_csv = out_dir / "axis_calib_summary.csv"
    with sum_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["phase"])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[calib] summary → {sum_csv}", file=sys.stderr)

    # Interpretation — with BIAS_hover at rows[0] and A/B/C/D after:
    # Subtract BIAS row from each motion row to get the pure motion signal.
    if len(rows) == 5:
        bias, A, B, C, D = rows  # BIAS, +X, -X, +Y, -Y
        print(f"\n=== BIAS-corrected per-phase signal ===", file=sys.stderr)
        print(f"  BIAS (hover):   mean_dx={bias['mean_L0_dx']:+.0f}  mean_dy={bias['mean_L0_dy']:+.0f}", file=sys.stderr)
        for r in (A, B, C, D):
            r['signal_dx'] = r['mean_L0_dx'] - bias['mean_L0_dx']
            r['signal_dy'] = r['mean_L0_dy'] - bias['mean_L0_dy']
            print(f"  {r['phase']:<10}: signal_dx={r['signal_dx']:+.0f}  signal_dy={r['signal_dy']:+.0f}",
                  file=sys.stderr)
        print(f"\n=== AXIS MAP INTERPRETATION (bias-corrected) ===", file=sys.stderr)
        print(f"  +body_X forward  →  signal (dx={A['signal_dx']:+.0f}, dy={A['signal_dy']:+.0f})",
              file=sys.stderr)
        print(f"  -body_X back     →  signal (dx={B['signal_dx']:+.0f}, dy={B['signal_dy']:+.0f})",
              file=sys.stderr)
        print(f"  +body_Y left     →  signal (dx={C['signal_dx']:+.0f}, dy={C['signal_dy']:+.0f})",
              file=sys.stderr)
        print(f"  -body_Y right    →  signal (dx={D['signal_dx']:+.0f}, dy={D['signal_dy']:+.0f})",
              file=sys.stderr)
        # Differential per axis (double-cancels DC bias):
        dx_per_X = (A['signal_dx'] - B['signal_dx']) / 2
        dy_per_X = (A['signal_dy'] - B['signal_dy']) / 2
        dx_per_Y = (C['signal_dx'] - D['signal_dx']) / 2
        dy_per_Y = (C['signal_dy'] - D['signal_dy']) / 2
        print(f"\n  Differential (cancels bias):", file=sys.stderr)
        print(f"    body+X induces L0_dx={dx_per_X:+.0f}, L0_dy={dy_per_X:+.0f} (per +0.20m/s × MOVE_S)",
              file=sys.stderr)
        print(f"    body+Y induces L0_dx={dx_per_Y:+.0f}, L0_dy={dy_per_Y:+.0f}",
              file=sys.stderr)
        # The 2×2 matrix [[dx_per_X, dx_per_Y],[dy_per_X, dy_per_Y]] is
        # the flow-image-axis → body-axis transform.  Invert it for the
        # correct BODY_XFORM.
        det = dx_per_X*dy_per_Y - dx_per_Y*dy_per_X
        if abs(det) > 1:
            inv = [dy_per_Y/det, -dx_per_Y/det, -dy_per_X/det, dx_per_X/det]
            print(f"\n  CORRECT BODY_XFORM (fw_dx, fw_dy, lf_dx, lf_dy) =", file=sys.stderr)
            print(f"    ({inv[0]:+.4f}, {inv[1]:+.4f}, {inv[2]:+.4f}, {inv[3]:+.4f})",
                  file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
