"""
s093 v2 — Max velocity via DIRECT position setpoint (bypass MotionCommander).

MotionCommander caps actual speed at ~30-45% of commanded.  Direct
position setpoint lets the cf2 position-PID fly to a target as fast
as physically possible.  Useful both to find max velocity AND because
it's the right architecture for closed-loop hover with PnP feedback.

Protocol:
  1. Takeoff to z=TARGET_Z (3m) via MotionCommander.
  2. For each target offset D in [0.5, 1.0, 2.0, 3.0, 5.0]:
       a. Hover stable for 2s.
       b. Send position setpoint (D, 0, z) at 100 Hz for D/0.5 seconds
          (allow time = D / 0.5m/s estimated cruise).
       c. Record EKF position throughout. Compute peak velocity.
       d. Send position setpoint (0, 0, z) to come back.
  3. Land.

Direct position cmd uses cf2 internal position PID — no high-level
caps. Will reveal true vmax of the platform under flow-only EKF.
"""
import sys, os, time, struct, threading, socket, math, csv
from pathlib import Path

FLOW_OUT_SOCK = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC   = 0x46524C31
REPLY_FMT     = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_SZ      = struct.calcsize(REPLY_FMT)

TARGET_Z      = 3.0
SETTLE_S      = 4.0
HOVER_GAP_S   = 3.0
SETPOINT_HZ   = 100   # cf2 expects steady setpoint stream
RECOVERY_S    = 4.0   # time at origin between phases

# Test targets: drone is told to fly to (Δx, 0, z).  Recording window
# is 3s — enough time at low speed (1.5m) but truncated by recovery
# for high speeds.
TARGETS = [
    ("0.5m", +0.5, 3.0),
    ("1.0m", +1.0, 3.0),
    ("2.0m", +2.0, 3.0),
    ("3.0m", +3.0, 4.0),
    ("5.0m", +5.0, 5.0),
    ("8.0m", +8.0, 6.0),
]


_lock = threading.Lock()
_ekf = [0.0, 0.0, 0.0]
_ekf_log = []  # timestamped EKF history (when actively recording)
_recording = [False]
def _att_cb(_ts, data, _lc):
    with _lock:
        _ekf[0] = data["stateEstimate.x"]
        _ekf[1] = data["stateEstimate.y"]
        _ekf[2] = data["stateEstimate.z"]
        if _recording[0]:
            _ekf_log.append((time.monotonic(), _ekf[0], _ekf[1], _ekf[2]))
def get_ekf():
    with _lock:
        return tuple(_ekf)


def flow_reader(stop_evt, records):
    sock = None
    for _ in range(20):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(FLOW_OUT_SOCK)
            sock = s; break
        except Exception:
            time.sleep(0.25)
    if sock is None:
        print(f"[posmaxv] FAIL flow socket", file=sys.stderr); return
    buf = b""
    while not stop_evt.is_set():
        try:
            chunk = sock.recv(4096)
            if not chunk: time.sleep(0.01); continue
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
                    "L0_dx": fields[2], "L0_dy": fields[3], "L0_conf": fields[4],
                })
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.02)


def stream_setpoint(cf, x, y, z, yaw, duration_s, hz=SETPOINT_HZ):
    """Spam send_position_setpoint at hz for duration_s.

    cf2 firmware requires a continuous setpoint stream — if no command
    arrives for ~1 second, controller falls back to hover/land mode.
    """
    period = 1.0 / hz
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        cf.commander.send_position_setpoint(x, y, z, yaw)
        time.sleep(period)


def main():
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="/tmp/cfcache_pos")
    sync = SyncCrazyflie("udp://0.0.0.0:19850", cf=cf)
    print("[posmaxv] connecting...", file=sys.stderr)
    sync.open_link()
    time.sleep(2.0)
    # RAISE cf2 position-PID velocity cap.  Default PID_POS_VEL_*_MAX = 1.0
    # in platform_defaults_sitl.h → drone never exceeds 1 m/s.  Bump to
    # 3 m/s (real cf2 platform handles this fine).
    print("[posmaxv] raising posCtlPid.xVelMax / yVelMax to 3.0 m/s", file=sys.stderr)
    cf.param.set_value("posCtlPid.xVelMax", "3.0")
    cf.param.set_value("posCtlPid.yVelMax", "3.0")
    time.sleep(0.3)
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)

    lc = LogConfig(name="state", period_in_ms=10)   # 100 Hz for sharp velocity calc
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

    # Phase 1: takeoff via MotionCommander, then HAND OFF to position setpoints
    mc = MotionCommander(sync, default_height=TARGET_Z)
    print(f"[posmaxv] takeoff to {TARGET_Z}m", file=sys.stderr)
    mc.take_off(height=TARGET_Z, velocity=0.5)
    time.sleep(SETTLE_S)
    print(f"[posmaxv] settled — switching to position setpoint mode",
          file=sys.stderr)

    results = []
    for label, dx, duration in TARGETS:
        # Recovery: go to origin first
        print(f"\n[posmaxv] recovery to (0, 0, {TARGET_Z})", file=sys.stderr)
        stream_setpoint(cf, 0.0, 0.0, TARGET_Z, 0.0, RECOVERY_S)
        ekf_start = get_ekf()
        print(f"          start EKF=({ekf_start[0]:+.3f},{ekf_start[1]:+.3f},{ekf_start[2]:.3f})",
              file=sys.stderr)
        # Snapshot phase
        _ekf_log.clear()
        _recording[0] = True
        t_phase_start = time.monotonic()
        print(f"[posmaxv] PHASE → target ({dx:+.1f}m, 0, {TARGET_Z}) for {duration:.1f}s",
              file=sys.stderr)
        stream_setpoint(cf, dx, 0.0, TARGET_Z, 0.0, duration)
        _recording[0] = False
        t_phase_end = time.monotonic()
        ekf_end = get_ekf()
        print(f"          end EKF=({ekf_end[0]:+.3f},{ekf_end[1]:+.3f},{ekf_end[2]:.3f})",
              file=sys.stderr)
        # Compute peak velocity from EKF log (use 0.2s sliding window)
        ekf_pts = list(_ekf_log)
        peak_v_x = 0.0
        v_at_50pct = 0.0
        if len(ekf_pts) > 20:
            window_s = 0.2
            for i in range(len(ekf_pts)):
                t_i = ekf_pts[i][0]
                # Find j such that t_j ~ t_i + window_s
                j = i
                while j < len(ekf_pts)-1 and (ekf_pts[j][0]-t_i) < window_s:
                    j += 1
                if j > i and (ekf_pts[j][0]-t_i) > 0:
                    v = (ekf_pts[j][1] - ekf_pts[i][1]) / (ekf_pts[j][0] - ekf_pts[i][0])
                    if v > peak_v_x:
                        peak_v_x = v
            # Velocity at the midpoint of the phase
            mid_idx = len(ekf_pts) // 2
            if mid_idx > 5:
                a = ekf_pts[mid_idx-5]; b = ekf_pts[mid_idx+5]
                v_at_50pct = (b[1]-a[1]) / (b[0]-a[0])
        # Flow saturation during the phase
        phase_flow = [r for r in flow_records if t_phase_start <= r["t"] <= t_phase_end]
        if phase_flow:
            mags = [max(abs(r["L0_dx"]), abs(r["L0_dy"])) for r in phase_flow]
            sat = sum(1 for m in mags if m >= 28000)
            peak_flow = max(mags) if mags else 0
        else:
            sat = 0; peak_flow = 0
        achieved_dist = ekf_end[0] - ekf_start[0]
        print(f"          peak_v_x={peak_v_x:.2f} m/s  v_mid={v_at_50pct:.2f} m/s  "
              f"achieved={achieved_dist:.2f}m  flow_peak={peak_flow}  sat={sat}",
              file=sys.stderr)
        results.append({
            "target": label, "dx_cmd": dx, "duration_s": duration,
            "peak_v_x": peak_v_x, "v_mid": v_at_50pct,
            "achieved_dx": achieved_dist,
            "flow_peak": peak_flow, "flow_sat_n": sat, "flow_n": len(phase_flow),
        })

    # Final recovery + landing
    print("\n[posmaxv] final recovery to origin then land", file=sys.stderr)
    stream_setpoint(cf, 0.0, 0.0, TARGET_Z, 0.0, RECOVERY_S)
    stream_setpoint(cf, 0.0, 0.0, 0.5, 0.0, 2.0)   # descend
    mc.land(velocity=0.3)
    time.sleep(2.0)
    stop_evt.set()
    th.join(timeout=1.0)
    sync.close_link()

    # Save & summary
    out_dir = Path(os.environ.get("SENTAI_DUMP_FRAMES_DIR", "."))
    csv_path = out_dir / "max_velocity_pos.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        for r in results: w.writerow(r)
    print(f"\n[posmaxv] CSV → {csv_path}", file=sys.stderr)

    print("\n=== MAX VELOCITY via POSITION SETPOINT ===", file=sys.stderr)
    print(f"{'target':>8} {'peak_v_x':>9} {'v_mid':>7} {'achieved':>9} {'flow_peak':>10} {'sat%':>5}",
          file=sys.stderr)
    for r in results:
        sat_pct = r['flow_sat_n']/max(1,r['flow_n'])*100
        print(f"{r['target']:>8} {r['peak_v_x']:>+8.2f}  {r['v_mid']:>+6.2f} "
              f"{r['achieved_dx']:>+8.2f} {r['flow_peak']:>10} {sat_pct:>4.0f}%",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
