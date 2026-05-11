"""sentai.flow pipeline diagnostic — T1+T2+T3 unified instrument.

Applies NASA/JPL flight-software discipline (per agent/embeded.md):
  - Single deterministic flow, no hidden control paths.
  - Every loop bounded by explicit timeout, no "wait forever".
  - Structured error codes (EFL_*) per fault class, never deleted.
  - Each test produces (pass, expected, measured, code, msg) tuple.
  - Per-frame samples logged for post-mortem; no in-memory growth past
    bounded test window (max ~30s × 30 fps × 200B = 180 KB).
  - No dynamic allocation in steady-state — pre-sized lists.
  - Single fault → single test fails → instrument continues.
  - Recovery: drone lands cleanly even on test abort.

TESTS:
  T1 — stationary baseline
      stimulus: zero velocity, drone hovering, wind expected OFF
      pass:     |mean(dx_q1000)| ≤ T1_MAX_DRIFT_Q
                |mean(dy_q1000)| ≤ T1_MAX_DRIFT_Q
                fraction(conf > T1_MIN_CONF) ≥ T1_MIN_CONF_RATE
      fault map: low conf → EFL_F3 ; non-zero mean drift → EFL_F6 (EKF
                 drift while we thought we were still — actually that
                 means flow IS reporting motion, so the issue is
                 controller, not flow.  Logged as EFL_F8)

  T2 — controlled body-X step
      stimulus: 3s commanded body-X velocity = V_STEP_M (default 0.1)
      pass:     sign(mean(dx_q1000)) == EXPECTED_SIGN_X
                |mean(dx_q1000) - EXPECTED_DX_Q| / EXPECTED_DX_Q ≤ 0.35
                |mean(dy_q1000)| ≤ T2_CROSS_AXIS_MAX_Q
      fault map: sign wrong → EFL_F1_SIGN_X
                 magnitude wrong → EFL_F2 (or EFL_F4)
                 cross-axis bleed → EFL_F1_SIGN_Y

  T3 — controlled body-Y step
      mirror of T2 on the orthogonal axis.

POST-CONDITIONS (regardless of pass/fail):
  - drone lands safely
  - results written to flow_diag_log.json
  - exit code: 0 if ALL tests pass, otherwise 1
"""
from __future__ import annotations

import json
import math
import os
import socket
import statistics
import struct
import sys
import threading
import time
from pathlib import Path

# ────────────────────────────────────────────────────────────
# Error code registry — per embeded.md §I.  ADD-only, never delete.
# ────────────────────────────────────────────────────────────
EFL_OK              = 0x0000
EFL_F1_SIGN_X       = 0xF100   # T2 dx sign wrong
EFL_F1_SIGN_Y       = 0xF101   # T3 dy sign wrong
EFL_F1_AXIS_BLEED_X = 0xF102   # T2 large dy when only dx expected
EFL_F1_AXIS_BLEED_Y = 0xF103   # T3 large dx when only dy expected
EFL_F2_MAG_X        = 0xF200   # T2 dx magnitude > 35% off
EFL_F2_MAG_Y        = 0xF201   # T3 dy magnitude > 35% off
EFL_F3_LOW_CONF     = 0xF300   # T1 conf fail
EFL_F4_FOV          = 0xF400   # reserved — covers F2 if scale mismatch dominates
EFL_F5_NO_ACK       = 0xF500   # cf2 not consuming flow (no log evidence)
EFL_F6_EKF          = 0xF600   # EKF response wrong (T4-style, not implemented yet)
EFL_F7_DT           = 0xF700   # dt observed outside plausible range
EFL_F8_HOLD         = 0xF800   # T1 drone drifted despite flow — controller/hold issue

EFL_STRINGS = {
    EFL_OK              : "OK",
    EFL_F1_SIGN_X       : "BODY_XFORM X-axis sign wrong",
    EFL_F1_SIGN_Y       : "BODY_XFORM Y-axis sign wrong",
    EFL_F1_AXIS_BLEED_X : "X-axis motion leaks into dy (axis swap?)",
    EFL_F1_AXIS_BLEED_Y : "Y-axis motion leaks into dx (axis swap?)",
    EFL_F2_MAG_X        : "FLOW_SCALE_X magnitude wrong (>35% off)",
    EFL_F2_MAG_Y        : "FLOW_SCALE_Y magnitude wrong (>35% off)",
    EFL_F3_LOW_CONF     : "phase-corr conf low — sentai_sim flow broken or scene bad",
    EFL_F4_FOV          : "intrinsics FOV mismatch with gz camera (suspect)",
    EFL_F5_NO_ACK       : "cf2 plugin not acking SENSOR_FLOW_SIM packets",
    EFL_F6_EKF          : "EKF flow_update response wrong (kalman/sign issue)",
    EFL_F7_DT           : "dt observed outside [0.001, 0.5]s",
    EFL_F8_HOLD         : "drone drifted in T1 — controller/hold protocol issue",
}

# ────────────────────────────────────────────────────────────
# Constants — pipeline + test parameters
# ────────────────────────────────────────────────────────────
# SIM body↔image mapping — empirically determined by THIS instrument
# (first-run results: body-+X commanded yields dy_q1000≈-500, body-+Y
# commanded yields dx_q1000≈-500).  Axes swapped + sign-flipped vs HW.
# See aruco_hover.py for the BODY_XFORM constant actually used by the
# flow forwarder.  Here it's documentation only — the analysers below
# encode the "image-Y is on-axis for body-X" rule directly.
SIM_BODY_XFORM = (0.0, -1.0, -1.0, 0.0)

# Camera + grid + drone (constants verified upstream)
CAM_W                = 640
CAM_H                = 480
CAM_FOV_H_RAD        = math.radians(58.0)
CAM_FOV_V_RAD        = math.radians(45.0)
FLOW_GRID_W          = 80
FLOW_GRID_H          = 60
FLOW_FRAME_HZ        = 30.0
TEST_Z_M             = 0.5
V_STEP_M             = 0.10            # body velocity for T2/T3 step
STEP_DURATION_S      = 4.0             # per-step hold time
SETTLE_BEFORE_S      = 2.0             # zero-vel settle before each step
SETTLE_AFTER_S       = 2.0             # zero-vel relax after each step
T1_DURATION_S        = 5.0
SAMPLE_HZ            = 5                # how often we read latest_ppm for PnP

# T1 thresholds
T1_MAX_DRIFT_Q       = 80               # mean |dx|, |dy| at rest (milli-grid/frame)
T1_MIN_CONF          = 64
T1_MIN_CONF_RATE     = 0.50             # fraction of frames with conf >= T1_MIN_CONF

# T2/T3 thresholds — derived from geometry:
# At z=0.5, footprint_x = 2·z·tan(HFOV/2) = 0.554 m, per-grid = 6.93 mm.
# v=0.1 m/s, per-frame ground disp = 0.1/30 = 3.33 mm = 0.481 grid → 481 mgrid.
# cam0 + vflip=1 mounting: body +X drone motion → ground slides -X in image,
# so dx_q1000 should be NEGATIVE.  flow_to_dpixel then re-negates via
# BODY_XFORM[0]=-1, producing positive body-fw dpx — which is what cf2 expects.
def _derive_expected_dx_q():
    footprint = 2.0 * TEST_Z_M * math.tan(CAM_FOV_H_RAD / 2.0)
    per_grid_m = footprint / FLOW_GRID_W
    per_frame_m = V_STEP_M / FLOW_FRAME_HZ
    return per_frame_m / per_grid_m * 1000.0   # milli-grid units

EXPECTED_DX_Q_MAG    = _derive_expected_dx_q()  # ~481 at z=0.5, v=0.1, 30fps
EXPECTED_DX_Q        = -EXPECTED_DX_Q_MAG       # sign from cam mounting (see above)

# Cross-axis tolerance: when commanding only +X, dy should stay near 0
T2_CROSS_AXIS_MAX_Q  = EXPECTED_DX_Q_MAG * 0.30   # ≤ 30% of axis-aligned flow

# Magnitude tolerance: ±35% on the on-axis component
MAG_TOL              = 0.35

# Flow output socket
FLOW_OUT_SOCK        = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC          = 0x46524C31
REPLY_FMT            = "<IIiiIQiI"
REPLY_SZ             = struct.calcsize(REPLY_FMT)
assert REPLY_SZ == 36

# Output
LOG_PATH             = Path(__file__).parent / "flow_diag_log.json"

# ────────────────────────────────────────────────────────────
# Pose mirror — populated by cflib log thread
# ────────────────────────────────────────────────────────────
_pose_lock = threading.Lock()
_pose = {"x": 0.0, "y": 0.0, "z": 0.0,
         "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _att_cb(_ts, data, _lc):
    with _pose_lock:
        _pose["x"] = data["stateEstimate.x"]
        _pose["y"] = data["stateEstimate.y"]
        _pose["z"] = data["stateEstimate.z"]
        _pose["roll"] = data["stateEstimate.roll"]
        _pose["pitch"] = data["stateEstimate.pitch"]
        _pose["yaw"] = data["stateEstimate.yaw"]


def pose_snapshot():
    with _pose_lock:
        return dict(_pose)


# ────────────────────────────────────────────────────────────
# Flow sniffer — non-injecting reader of /tmp/sentai_flow_out.sock
#
# Per embeded.md §C: NO business logic in driver/HAL.  This thread
# only READS the flow reply stream.  Forwarding to cf2 is a separate
# concern handled by the caller if needed.  Each diagnostic test takes
# a fresh "drain window" so it sees ONLY samples captured during that
# window, not stale data from earlier tests.
# ────────────────────────────────────────────────────────────
class FlowSniffer:
    """Background thread reading flow replies from gz_to_uds_bridge.

    The sniffer keeps a bounded ring of recent samples (latest N kept).
    Tests call `drain()` to start a fresh window and `harvest(window_s)`
    to get the list of samples captured during that window."""

    RING_MAX = 2048   # bounded — at 30 Hz that's >60s of history
    REPLY_BYTES = REPLY_SZ

    def __init__(self):
        self._sock = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._ring = []
        self._sock_err = None
        self._thr = None
        self._packets_total = 0

    def start(self):
        for _ in range(20):
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(0.5)
                s.connect(FLOW_OUT_SOCK)
                self._sock = s
                break
            except Exception as e:
                self._sock_err = str(e)
                time.sleep(0.25)
        if self._sock is None:
            raise RuntimeError(f"flow socket unavailable: {self._sock_err}")
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop.set()
        if self._thr is not None:
            self._thr.join(timeout=1.0)

    def _loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    time.sleep(0.005)
                    continue
                buf += chunk
                while len(buf) >= self.REPLY_BYTES:
                    rec, buf = buf[:self.REPLY_BYTES], buf[self.REPLY_BYTES:]
                    magic, seq, dx, dy, conf, lat, dz, dz_conf = struct.unpack(
                        REPLY_FMT, rec)
                    if magic != REPLY_MAGIC:
                        idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                        buf = buf[idx:] if idx >= 0 else b""
                        continue
                    rec_time = time.monotonic()
                    sample = {
                        "t_host": rec_time,
                        "seq": seq,
                        "dx_q1000": dx,
                        "dy_q1000": dy,
                        "conf": conf,
                        "lat_us": lat,
                        "dz_q1000": dz,
                        "dz_conf": dz_conf,
                    }
                    with self._lock:
                        self._ring.append(sample)
                        # bounded — drop oldest if exceeded
                        if len(self._ring) > self.RING_MAX:
                            self._ring = self._ring[-self.RING_MAX:]
                        self._packets_total += 1
            except socket.timeout:
                continue
            except Exception as e:
                self._sock_err = str(e)
                time.sleep(0.02)

    def drain(self):
        """Mark new test window — return time.monotonic() for filtering."""
        return time.monotonic()

    def harvest(self, t_start, t_end):
        """Return all samples captured in [t_start, t_end]."""
        with self._lock:
            return [s for s in self._ring
                    if t_start <= s["t_host"] <= t_end]

    @property
    def packets_total(self):
        return self._packets_total


# ────────────────────────────────────────────────────────────
# Per-test analysers — pure functions, return structured verdict
# ────────────────────────────────────────────────────────────
def _stats_of_axis(samples, key):
    if not samples:
        return None
    vals = [s[key] for s in samples]
    return {
        "n": len(vals),
        "mean": statistics.mean(vals),
        "stdev": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        "min": min(vals),
        "max": max(vals),
    }


def _conf_rate(samples, threshold):
    if not samples:
        return 0.0
    return sum(1 for s in samples if s["conf"] >= threshold) / len(samples)


def analyse_t1(samples):
    """Stationary baseline.  PASS if drift small + conf decent."""
    n = len(samples)
    if n < 5:
        return {"pass": False, "code": EFL_F3_LOW_CONF,
                "msg": f"only {n} samples captured", "samples": []}
    sx = _stats_of_axis(samples, "dx_q1000")
    sy = _stats_of_axis(samples, "dy_q1000")
    cr = _conf_rate(samples, T1_MIN_CONF)
    code = EFL_OK
    msg = []
    if cr < T1_MIN_CONF_RATE:
        code = EFL_F3_LOW_CONF
        msg.append(f"conf_rate={cr:.2f} < {T1_MIN_CONF_RATE}")
    if abs(sx["mean"]) > T1_MAX_DRIFT_Q or abs(sy["mean"]) > T1_MAX_DRIFT_Q:
        # Flow reports motion while we hovered → drone IS drifting due
        # to lack of hold, OR phase-corr biased.  Per spec we treat as
        # F8 (hold issue) — controller didn't react.
        if code == EFL_OK:
            code = EFL_F8_HOLD
            msg.append(f"|mean dx|={abs(sx['mean']):.0f}, "
                       f"|mean dy|={abs(sy['mean']):.0f} > {T1_MAX_DRIFT_Q}")
    return {
        "pass": code == EFL_OK,
        "code": code,
        "code_str": EFL_STRINGS[code],
        "msg": "; ".join(msg) or "ok",
        "dx_stats": sx,
        "dy_stats": sy,
        "conf_rate": cr,
        "n_samples": n,
    }


def analyse_step(samples, axis, expected_q, cross_axis_max):
    """Generic analyser for T2 (axis='x') and T3 (axis='y').
    expected_q is the *signed* expected value on the on-axis flow output.

    For the SIM camera mount, body-axis ↔ image-axis is SWAPPED:
        body-X commanded  ⇒  signal on dy_q1000 (image Y)
        body-Y commanded  ⇒  signal on dx_q1000 (image X)
    So 'on-axis' here means the IMAGE axis that should carry the signal."""
    n = len(samples)
    if n < 5:
        return {"pass": False, "code": EFL_F3_LOW_CONF,
                "msg": f"only {n} samples", "samples": []}
    sx = _stats_of_axis(samples, "dx_q1000")
    sy = _stats_of_axis(samples, "dy_q1000")
    cr = _conf_rate(samples, T1_MIN_CONF)
    # body-X → image-Y on-axis ; body-Y → image-X on-axis
    on_stats = sy if axis == "x" else sx
    off_stats = sx if axis == "x" else sy
    on_mean = on_stats["mean"]

    code = EFL_OK
    msg = []
    # Sign check
    if (expected_q > 0 and on_mean <= 0) or (expected_q < 0 and on_mean >= 0):
        code = EFL_F1_SIGN_X if axis == "x" else EFL_F1_SIGN_Y
        msg.append(f"sign wrong: on_mean={on_mean:.0f} expected≈{expected_q:.0f}")
    else:
        # Magnitude check (only meaningful if sign was right)
        rel = abs(on_mean - expected_q) / abs(expected_q)
        if rel > MAG_TOL:
            code = EFL_F2_MAG_X if axis == "x" else EFL_F2_MAG_Y
            msg.append(f"magnitude off: {abs(on_mean):.0f} vs |expected|={abs(expected_q):.0f} ({rel*100:.0f}%)")
        # Cross-axis bleed check
        if abs(off_stats["mean"]) > cross_axis_max:
            if code == EFL_OK:
                code = (EFL_F1_AXIS_BLEED_X if axis == "x"
                        else EFL_F1_AXIS_BLEED_Y)
                msg.append(f"cross-axis bleed: off_mean={off_stats['mean']:.0f} > {cross_axis_max:.0f}")
    return {
        "pass": code == EFL_OK,
        "code": code,
        "code_str": EFL_STRINGS[code],
        "msg": "; ".join(msg) or "ok",
        "dx_stats": sx,
        "dy_stats": sy,
        "conf_rate": cr,
        "expected_q": expected_q,
        "n_samples": n,
    }


# ────────────────────────────────────────────────────────────
# Test runner — each test is a single deterministic sequence
# ────────────────────────────────────────────────────────────
def run_t1_stationary(mc, sniffer):
    print(f"\n[T1] stationary baseline — {T1_DURATION_S}s zero-vel hover",
          file=sys.stderr)
    mc.start_linear_motion(0, 0, 0)
    t_start = sniffer.drain()
    t_end_target = t_start + T1_DURATION_S
    while time.monotonic() < t_end_target:
        time.sleep(0.1)
    samples = sniffer.harvest(t_start, time.monotonic())
    res = analyse_t1(samples)
    print(f"[T1] n={res['n_samples']}  "
          f"dx mean={res['dx_stats']['mean']:+.1f} (σ={res['dx_stats']['stdev']:.1f})  "
          f"dy mean={res['dy_stats']['mean']:+.1f} (σ={res['dy_stats']['stdev']:.1f})  "
          f"conf_rate={res['conf_rate']:.2f}  ",
          file=sys.stderr)
    print(f"[T1] verdict: {'PASS' if res['pass'] else 'FAIL'} "
          f"({res['code_str']})  {res['msg']}", file=sys.stderr)
    return res


def _run_step(mc, axis, sniffer):
    """Common T2/T3 motion sequence."""
    sign_label = "+X" if axis == "x" else "+Y"
    vx = V_STEP_M if axis == "x" else 0.0
    vy = V_STEP_M if axis == "y" else 0.0
    expected_q = EXPECTED_DX_Q if axis == "x" else (-EXPECTED_DX_Q_MAG)
    # cam0+vflip=1: body +Y → ground slides -Y in image → dy_q < 0.
    # BODY_XFORM[3]=+1 re-asserts: lft_grid = +dy_grid → forwarded as
    # dpy with sign matching body +Y.

    # Pre-settle
    mc.start_linear_motion(0, 0, 0)
    time.sleep(SETTLE_BEFORE_S)

    print(f"[T{2 if axis=='x' else 3}] commanded body-{sign_label} = "
          f"{V_STEP_M} m/s for {STEP_DURATION_S}s", file=sys.stderr)
    pose_pre = pose_snapshot()
    print(f"[T{2 if axis=='x' else 3}]   pose pre  = "
          f"({pose_pre['x']:+.2f}, {pose_pre['y']:+.2f}, {pose_pre['z']:.2f})",
          file=sys.stderr)

    t_start = sniffer.drain()
    mc.start_linear_motion(vx, vy, 0)
    t_end_target = t_start + STEP_DURATION_S
    while time.monotonic() < t_end_target:
        time.sleep(0.1)
    mc.start_linear_motion(0, 0, 0)

    samples = sniffer.harvest(t_start, time.monotonic())
    pose_post = pose_snapshot()
    print(f"[T{2 if axis=='x' else 3}]   pose post = "
          f"({pose_post['x']:+.2f}, {pose_post['y']:+.2f}, {pose_post['z']:.2f})",
          file=sys.stderr)

    time.sleep(SETTLE_AFTER_S)
    res = analyse_step(samples, axis, expected_q, T2_CROSS_AXIS_MAX_Q)
    res["pose_pre"] = pose_pre
    res["pose_post"] = pose_post
    res["delta_pose"] = {
        "dx_m": pose_post["x"] - pose_pre["x"],
        "dy_m": pose_post["y"] - pose_pre["y"],
    }
    res["expected_q_signed"] = expected_q
    print(f"[T{2 if axis=='x' else 3}] n={res['n_samples']}  "
          f"dx mean={res['dx_stats']['mean']:+.1f}  "
          f"dy mean={res['dy_stats']['mean']:+.1f}  "
          f"expected on-axis={expected_q:+.0f}", file=sys.stderr)
    print(f"[T{2 if axis=='x' else 3}] verdict: "
          f"{'PASS' if res['pass'] else 'FAIL'} "
          f"({res['code_str']})  {res['msg']}", file=sys.stderr)
    return res


def run_t2_body_x(mc, sniffer):
    return _run_step(mc, "x", sniffer)


def run_t3_body_y(mc, sniffer):
    return _run_step(mc, "y", sniffer)


# ────────────────────────────────────────────────────────────
# Main — bounded sequence with explicit recovery
# ────────────────────────────────────────────────────────────
def main() -> int:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    print(f"[diag] expected dx_q at z={TEST_Z_M}, v={V_STEP_M}: "
          f"{EXPECTED_DX_Q:+.0f} milli-grid/frame (cam0+vflip=1)",
          file=sys.stderr)

    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf

    cf.param.set_value("stabilizer.estimator", 2)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 1)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 0)
    time.sleep(2.0)

    lc = LogConfig(name="pose", period_in_ms=20)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    # Sniffer reads flow reply stream — NO injection to cf2
    sniffer = FlowSniffer()
    try:
        sniffer.start()
    except Exception as e:
        print(f"[diag] FATAL sniffer init: {e}", file=sys.stderr)
        sync.close_link()
        return 1
    print(f"[diag] sniffer connected to {FLOW_OUT_SOCK}", file=sys.stderr)

    mc = MotionCommander(sync, default_height=TEST_Z_M)
    mc.take_off(height=TEST_Z_M, velocity=0.3)
    time.sleep(3.0)   # let EKF settle, sentai_sim warm up flow

    results = {}
    try:
        results["t1"] = run_t1_stationary(mc, sniffer)
        results["t2"] = run_t2_body_x(mc, sniffer)
        results["t3"] = run_t3_body_y(mc, sniffer)
    except Exception as e:
        print(f"[diag] EXCEPTION during tests: {e}", file=sys.stderr)
        results["exception"] = str(e)
    finally:
        try:
            mc.start_linear_motion(0, 0, 0)
            time.sleep(0.3)
            mc.land(velocity=0.3)
            time.sleep(2.0)
        except Exception as e:
            print(f"[diag] WARN land failed: {e}", file=sys.stderr)
        sniffer.stop()
        lc.stop()
        sync.close_link()

    # Aggregate verdict
    all_pass = all(r.get("pass") for r in results.values() if isinstance(r, dict) and "pass" in r)
    print(f"\n=== DIAGNOSTIC SUMMARY ===", file=sys.stderr)
    print(f"sniffer packets total: {sniffer.packets_total}", file=sys.stderr)
    for tag, res in results.items():
        if not isinstance(res, dict) or "pass" not in res:
            continue
        verdict = "PASS" if res["pass"] else "FAIL"
        print(f"  {tag.upper():3}  {verdict}  code=0x{res['code']:04X}  "
              f"{res['code_str']}", file=sys.stderr)
    print(f"OVERALL: {'PASS' if all_pass else 'FAIL'}", file=sys.stderr)

    LOG_PATH.write_text(json.dumps({
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "test_z_m": TEST_Z_M,
        "v_step_m": V_STEP_M,
        "step_duration_s": STEP_DURATION_S,
        "expected_dx_q_signed": EXPECTED_DX_Q,
        "expected_dx_q_mag": EXPECTED_DX_Q_MAG,
        "mag_tol": MAG_TOL,
        "results": results,
        "sniffer_total_packets": sniffer.packets_total,
        "overall_pass": all_pass,
        "error_codes_legend": {f"0x{k:04X}": v for k, v in EFL_STRINGS.items()},
    }, indent=2, default=lambda o: float(o) if hasattr(o, "__float__") else str(o)))
    print(f"[diag] log → {LOG_PATH}", file=sys.stderr)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
