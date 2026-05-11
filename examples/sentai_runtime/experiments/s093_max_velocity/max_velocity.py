"""
s093 — Discover the drone's maximum achievable velocity in no-wind SIL.

Protocol:
  1. Takeoff to z=1.0m, hover 3s to settle.
  2. For each commanded velocity in [0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0] m/s:
       a. Hover briefly (1.5s) → record baseline.
       b. Command +body_X at vel for MOVE_S seconds.
       c. Stop (zero velocity).
       d. Return-trip: command -body_X for slightly longer (to recover).
       e. Hover 2s to settle for next phase.
  3. Land.

For each phase, log:
  - Commanded velocity
  - EKF velocity (derived from positions at start/end of phase)
  - Gz ground truth velocity (from gz model -p before/after)
  - Peak flow magnitude (saturated?)

Pass criterion: identify max velocity at which:
  - EKF velocity ≈ commanded (drone reached target speed)
  - Flow didn't saturate (no peak above ±28000 mgrid)
  - Drone returned to origin successfully

WIND MUST BE DISABLED (the test commands -X to return, but wind would
add disturbance).  Use /tmp/run_max_velocity.sh which disables wind.
"""
import sys, os, time, struct, threading, socket, math, csv, json, subprocess
from pathlib import Path

# Reuse same flow protocol as s091
FLOW_OUT_SOCK = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC   = 0x46524C31
REPLY_FMT     = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_SZ      = struct.calcsize(REPLY_FMT)
assert REPLY_SZ == 124

TARGET_Z       = 3.0    # higher altitude → more room + larger flow saturation budget
                        # (at z=3m, 1 L0-px ≈ 34mm ground → flow saturates at ~27 m/s
                        #  vs ~9 m/s at z=1m)
SETTLE_S       = 4.0
HOVER_GAP_S    = 2.5
MOVE_S         = 3.0
RETURN_VEL_FAC = 1.0
RETURN_S_FAC   = 1.2

VELOCITIES = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]   # m/s commanded


def gz_drone_pose():
    """Query gz for ground-truth drone pose. Returns (x,y,z) or None."""
    try:
        r = subprocess.run(
            ["distrobox", "enter", "crazysim-garden", "--",
             "gz", "model", "-m", "crazyflie_0", "-p"],
            capture_output=True, text=True, timeout=2)
        # Parse "    [X Y Z]" line from output
        for ln in r.stdout.splitlines():
            ln = ln.strip()
            if ln.startswith("[") and "]" in ln:
                parts = ln.strip("[]").split()
                if len(parts) >= 3:
                    return (float(parts[0]), float(parts[1]), float(parts[2]))
    except Exception:
        pass
    return None


def flow_reader(stop_evt, records):
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
        print(f"[maxv] FAIL flow socket", file=sys.stderr)
        return
    print(f"[maxv] flow socket connected", file=sys.stderr)
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
                if fields[0] != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                records.append({
                    "t": time.monotonic(),
                    "seq": fields[1],
                    "L0_dx": fields[2], "L0_dy": fields[3], "L0_conf": fields[4],
                })
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.02)


_lock = threading.Lock()
_ekf = [0.0, 0.0, 0.0]
def _att_cb(_ts, data, _lc):
    with _lock:
        _ekf[0] = data["stateEstimate.x"]
        _ekf[1] = data["stateEstimate.y"]
        _ekf[2] = data["stateEstimate.z"]
def get_ekf():
    with _lock:
        return tuple(_ekf)


def main():
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="/tmp/cfcache_maxv")
    sync = SyncCrazyflie("udp://0.0.0.0:19850", cf=cf)
    print("[maxv] connecting...", file=sys.stderr)
    sync.open_link()
    time.sleep(2.0)
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)

    lc = LogConfig(name="state", period_in_ms=20)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    stop_evt = threading.Event()
    flow_records = []
    th = threading.Thread(target=flow_reader, args=(stop_evt, flow_records),
                           daemon=True)
    th.start()
    time.sleep(0.5)

    mc = MotionCommander(sync, default_height=TARGET_Z)
    print(f"[maxv] takeoff to {TARGET_Z}m", file=sys.stderr)
    mc.take_off(height=TARGET_Z, velocity=0.5)
    time.sleep(SETTLE_S)
    print(f"[maxv] settled", file=sys.stderr)

    results = []
    for vel in VELOCITIES:
        # Quiet baseline before phase
        time.sleep(HOVER_GAP_S)
        print(f"\n[maxv] PHASE +X at {vel:.2f} m/s for {MOVE_S}s", file=sys.stderr)
        # Capture state RIGHT BEFORE motion command (no slow subprocess here)
        ekf_start = get_ekf()
        t_motion_start = time.monotonic()
        mc.start_linear_motion(vel, 0, 0)
        # Mid-phase peak velocity sampling — record EKF at the middle of
        # the motion to estimate steady-state speed (after the accel ramp).
        time.sleep(MOVE_S * 0.5)
        ekf_mid = get_ekf()
        t_mid = time.monotonic()
        time.sleep(MOVE_S * 0.5)
        # Stop motion ASAP
        t_motion_end = time.monotonic()
        mc.start_linear_motion(0, 0, 0)
        ekf_end = get_ekf()
        # Velocity from second half (drone should be near steady state)
        dt_motion = t_motion_end - t_motion_start
        dt_second_half = t_motion_end - t_mid
        dx_full   = ekf_end[0] - ekf_start[0]
        dx_second = ekf_end[0] - ekf_mid[0]
        v_ekf_full = dx_full / dt_motion
        v_ekf_ss   = dx_second / dt_second_half   # steady-state estimate
        # Settle then query gz ground truth (post-motion only, no pre)
        time.sleep(0.5)
        gz_end = gz_drone_pose()
        # We don't have gz_start (would have been before motion) — skip gz delta
        dx_gz = v_gz = None
        # alias for compatibility with CSV
        dt = dt_motion
        v_ekf = v_ekf_ss
        dx_ekf = dx_full
        # Flow magnitudes during the SECOND HALF of motion (steady-state)
        phase_flow = [r for r in flow_records if t_mid <= r["t"] <= t_motion_end]
        if phase_flow:
            mags = [max(abs(r["L0_dx"]), abs(r["L0_dy"])) for r in phase_flow]
            sat_count = sum(1 for m in mags if m >= 28000)
            peak = max(mags) if mags else 0
        else:
            sat_count = 0; peak = 0
        print(f"       end EKF=({ekf_end[0]:+.3f},{ekf_end[1]:+.3f},{ekf_end[2]:.3f})  "
              f"gz={gz_end}", file=sys.stderr)
        print(f"       Δt_motion={dt_motion:.2f}s  ΔX_full={dx_full:+.3f}m  v_full={v_ekf_full:+.2f} m/s",
              file=sys.stderr)
        print(f"       second-half: Δt={dt_second_half:.2f}s  ΔX={dx_second:+.3f}m  v_ss={v_ekf_ss:+.2f} m/s",
              file=sys.stderr)
        print(f"       flow peak={peak} mgrid, saturated frames={sat_count}/{len(phase_flow)}",
              file=sys.stderr)
        results.append({
            "v_cmd": vel,
            "dt_motion_s": dt_motion,
            "dx_full": dx_full,
            "v_full": v_ekf_full,
            "dt_ss_s": dt_second_half,
            "dx_ss": dx_second,
            "v_ss": v_ekf_ss,
            "gz_end_x": gz_end[0] if gz_end else None,
            "flow_peak": peak,
            "flow_sat_n": sat_count,
            "flow_n": len(phase_flow),
        })

        # Return trip to roughly recover to origin
        return_vel = -vel * RETURN_VEL_FAC
        return_s   = MOVE_S * RETURN_S_FAC
        # Cap to avoid hitting cf2 internal limits
        return_vel = max(-3.0, min(3.0, return_vel))
        print(f"[maxv] return: vx={return_vel:+.2f} for {return_s:.2f}s", file=sys.stderr)
        mc.start_linear_motion(return_vel, 0, 0)
        time.sleep(return_s)
        mc.start_linear_motion(0, 0, 0)

    time.sleep(1.0)
    print("\n[maxv] landing", file=sys.stderr)
    mc.land(velocity=0.3)
    time.sleep(2.0)
    stop_evt.set()
    th.join(timeout=1.0)
    sync.close_link()

    # Save & summary
    out_dir = Path(os.environ.get("SENTAI_DUMP_FRAMES_DIR", "."))
    csv_path = out_dir / "max_velocity.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        for r in results:
            w.writerow(r)
    print(f"\n[maxv] CSV → {csv_path}", file=sys.stderr)

    print("\n=== MAX VELOCITY SUMMARY ===", file=sys.stderr)
    print(f"{'v_cmd':>6} {'v_ss':>7} {'v_full':>7} {'track_err':>10} {'flow_peak':>10} {'sat%':>5}",
          file=sys.stderr)
    for r in results:
        err = abs(r['v_cmd'] - r['v_ss'])
        err_pct = err / max(0.01, r['v_cmd']) * 100
        sat_pct = r['flow_sat_n'] / max(1, r['flow_n']) * 100
        print(f"{r['v_cmd']:>6.2f} {r['v_ss']:>+7.2f} {r['v_full']:>+7.2f} "
              f"{err_pct:>8.0f}% {r['flow_peak']:>10} {sat_pct:>4.0f}%",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
