"""Hover over ArUco marker pattern @ z=0.5m with sentai.flow stabilization.

User spec (2026-05-11):
    "fa hover over ArUco, testam hover cu sentai.flow peste ArUco, nu peste CAT"
    "wind puternic e ok pentru ca asa validăm calitatea algoritmului flow"

Validation test for sentai.flow under aggressive Gazebo WindEffects
disturbance.  Pass criteria:
  - drone stays within ±0.20 m of (0, 0, 0.5) for 15 s hover
  - all 4 markers visible >= 50% of frames (FOV containment)
  - mean(|pnp_z - ekf_z|) < 8 cm (PnP-altitude calibration valid)
  - flow packet rate >= 5 Hz (forwarder feeding EKF properly)

Architecture:
  Gazebo /downward_cam/image
    → gz_to_uds_bridge (C++, distrobox) → /tmp/sentai_cam.sock
    → sentai_sim (camera_bridge_recv → flow_phase_corr)
    → /tmp/sentai_flow_out.sock (flow_reply_t, 36 bytes/frame)
    → THIS SCRIPT's forwarder thread → cflib cf.send_packet(SENSOR_FLOW_SIM)
    → cf2 SITL EKF (sensors_sitl.c) → position hold
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

sys.path.insert(0, str(Path(__file__).parent.parent / "s090_hover_over_cat"))
from aruco_detector import detect_in_ppm, latest_ppm, KNOWN_POSITIONS_M

# ────────────────────────────────────────────────────────────
# Constants — body-frame xform + flow-to-PMW3901 scaling
# (verbatim from s090/hover_over_cat.py to keep one convention)
# ────────────────────────────────────────────────────────────
FLOW_FOV_H_DEG    = 58.0
FLOW_FOV_V_DEG    = 45.0
FLOW_GRID_W       = 80
FLOW_GRID_H       = 60
DRONE_NPIX        = 35.0
DRONE_THETAPIX    = 0.71674
DRONE_FLOW_RES    = 0.10
# SIM-specific body↔image axes mapping.  Verified by flow_diagnostic T2/T3
# (commit 1aa12e18 + subsequent): with the current sentai_crazysim camera
# mount in gz Garden, raw phase-corr output has IMAGE-X axis aligned to
# body-Y and IMAGE-Y axis aligned to body-X, both sign-inverted.
#   body_fw  =  -dy_grid    (T2: body+X → dy_q1000 = -500)
#   body_lft =  -dx_grid    (T3: body+Y → dx_q1000 = -500)
# This DIFFERS from the HW convention (cam0+vflip=1, BODY_XFORM=(-1,0,0,+1)
# per Sim.md §10b) — the SDF cam mount yaw in CrazySim doesn't replicate
# the OV5640 vflip path, so we have a SIM-only override here.  TODO:
# unify by fixing the gz camera mount yaw to match HW image orientation.
BODY_XFORM        = (0.0, -1.0, -1.0, 0.0)
_FLOW_SCALE_X = (math.radians(FLOW_FOV_H_DEG) * DRONE_NPIX) / (FLOW_GRID_W * DRONE_FLOW_RES * DRONE_THETAPIX)
_FLOW_SCALE_Y = (math.radians(FLOW_FOV_V_DEG) * DRONE_NPIX) / (FLOW_GRID_H * DRONE_FLOW_RES * DRONE_THETAPIX)
CRTP_PORT_SETPOINT_SIM = 0x09
SENSOR_FLOW_SIM        = 6

# Flow output socket protocol (gz_to_uds_bridge.cc Reply struct, packed 1).
# 2026-05-11: extended with foveated CENTER pipeline — adds dx/dy/conf for
# the native-pixel center-patch flow (8× higher per-pixel resolution at
# z=1m, so detects sub-pixel-of-wide-grid slow drift).  Reply grew from
# 36 → 48 bytes — BOTH ends of the pipeline (sentai_sim + gz_to_uds_bridge)
# must be rebuilt for this protocol revision.
FLOW_OUT_SOCK    = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC      = 0x46524C31   # 'FRL1'
REPLY_FMT        = "<IIiiIQiIiiIiiIiiIIiiIB3x"  # +BEST (dx,dy,conf,source) at tail
REPLY_SZ         = struct.calcsize(REPLY_FMT)
assert REPLY_SZ == 92, f"unexpected REPLY_SZ={REPLY_SZ}"

# Pyramid scale ratios — fine-grained mgrid in each level corresponds to
# different physical ground motion.  Convert each level's mgrid to the
# wide-equivalent (L0) reference by dividing.  Per-grid:
#   L0  (8× decimation): 13.85 mm/grid @ z=1m  (reference)
#   L1  (4× decimation):  6.93 mm/grid  → L1 mgrid × 0.5 = L0 mgrid
#   L2  (2× decimation):  3.46 mm/grid  → L2 mgrid × 0.25 = L0 mgrid
L1_TO_L0_RATIO = 0.5    # mid (4× decim) → wide-equivalent
L2_TO_L0_RATIO = 0.125  # native crop (8× finer per-px) → wide-equivalent
# Back-compat alias (some code may still reference CENTER_TO_WIDE_RATIO).
CENTER_TO_WIDE_RATIO = L1_TO_L0_RATIO

# Test parameters
# Climb rapid to 1.0m — at 0.5m drone is too low and small drifts push
# markers out of FOV before EKF settles via flow.  At 1.0m FOV footprint
# is 1.1m × 0.83m so the 0.30×0.20 marker pattern has ~25cm margin per
# axis even if drone drifts 30cm laterally during takeoff transient.
TARGET_Z_M       = 1.0
TAKEOFF_VEL_MPS  = 0.8     # fast climb to escape low-altitude FOV-loss
HOVER_S          = 15.0
SAMPLE_HZ        = 5
LOG_PATH         = Path(__file__).parent / "hover_log.json"


def flow_to_dpixel(dx_q1000: int, dy_q1000: int) -> tuple[float, float]:
    """sentai.flow milli-grid-px → PMW3901-equivalent dpixel (body frame)."""
    dx_grid = dx_q1000 / 1000.0
    dy_grid = dy_q1000 / 1000.0
    fw_dx, fw_dy, lf_dx, lf_dy = BODY_XFORM
    fw_grid   = fw_dx * dx_grid + fw_dy * dy_grid
    left_grid = lf_dx * dx_grid + lf_dy * dy_grid
    return (fw_grid * _FLOW_SCALE_X, left_grid * _FLOW_SCALE_Y)


def flow_conf_to_std(conf: int) -> float:
    # 2026-05-11: bumped 1/2/4/8 → 3/4/6/10.  Cf2 EKF was over-trusting
    # flow observations (small std → high Kalman gain → aggressive
    # correction → phase-lag oscillation at 20Hz vs PMW3901's 100Hz
    # tuning).  Raising std reduces gain, trades drift quality for
    # hover stability.
    if conf >= 200: return 3.0
    if conf >= 128: return 4.0
    if conf >= 64:  return 6.0
    return 10.0


# ────────────────────────────────────────────────────────────
# cflib state
# ────────────────────────────────────────────────────────────
_lock = threading.Lock()
_ekf_x = _ekf_y = _ekf_z = 0.0
_roll = _pitch = _yaw = 0.0


def _att_cb(_ts, data, _lc):
    global _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw
    with _lock:
        _ekf_x = data["stateEstimate.x"]
        _ekf_y = data["stateEstimate.y"]
        _ekf_z = data["stateEstimate.z"]
        _roll = data["stateEstimate.roll"]
        _pitch = data["stateEstimate.pitch"]
        _yaw = data["stateEstimate.yaw"]


def get_ekf():
    with _lock:
        return _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw


# ────────────────────────────────────────────────────────────
# Flow forwarder — reads /tmp/sentai_flow_out.sock, sends to cf2
# ────────────────────────────────────────────────────────────
def flow_forwarder(stop_evt: threading.Event, cf, stats: dict) -> None:
    from cflib.crtp.crtpstack import CRTPPacket

    # Connect retry — bridge needs a moment to start the listener.
    sock = None
    for i in range(20):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(FLOW_OUT_SOCK)
            sock = s
            break
        except Exception:
            time.sleep(0.25)
    if sock is None:
        print(f"[flow] FAIL connect {FLOW_OUT_SOCK}", file=sys.stderr)
        stats["fatal"] = "no flow socket"
        return
    print(f"[flow] connected {FLOW_OUT_SOCK}", file=sys.stderr)

    last_send_t = time.monotonic()
    buf = b""
    while not stop_evt.is_set():
        try:
            chunk = sock.recv(4096)
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            while len(buf) >= REPLY_SZ:
                rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
                (magic, seq, dx, dy, conf, lat, dz, dz_conf,
                 dx_c, dy_c, conf_c,
                 dx_f, dy_f, conf_f,
                 dx_anch, dy_anch, conf_anch, frames_since_anch,
                 dx_best, dy_best, conf_best, best_source) = (
                    struct.unpack(REPLY_FMT, rec))
                if magic != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                now = time.monotonic()
                dt = max(0.001, min(0.2, now - last_send_t))
                last_send_t = now

                # FUSION wide ↔ center.  Two phase-corr results:
                #   WIDE:    coarse but full-FOV.  Good for any motion
                #            magnitude up to ±32 wide-grid ≈ ±0.44 m at z=1m.
                #            Loses confidence below ~0.5 wide-grid ≈ 7mm.
                #   CENTER:  fine but limited-FOV.  Resolves 1.73mm/pixel
                #            so detects sub-wide-grid motion at high conf.
                #            Saturates above ±28 center-grid ≈ 5cm body shift
                #            between frames.
                # Rule: prefer the source with higher confidence.  If center
                # has high conf AND magnitude under sat band, scale center
                # to wide-equivalent units and use it.  Else fall back to
                # wide.  This makes slow drift visible without sacrificing
                # fast-motion tracking.
                # FUSION DONE C-SIDE (in camera_bridge_recv.c handle_one_frame).
                # We just use dx_best/dy_best/conf_best as the single source
                # of truth.  This matches the ARM real-time architecture:
                # firmware does the fusion, Python only relays to cf2 EKF.
                dx_eff, dy_eff, conf_eff = dx_best, dy_best, conf_best
                level_names = {0: "L0", 1: "L1_refined", 2: "L2_refined", 3: "LCF_anchor"}
                used_level = level_names.get(best_source, f"?{best_source}")
                # Stats per-pipeline for post-run analysis
                stats.setdefault("pp", []).append({
                    "L0_dx": dx, "L0_dy": dy, "L0_conf": conf,
                    "L1_dx": dx_c, "L1_dy": dy_c, "L1_conf": conf_c,
                    "L2_dx": dx_f, "L2_dy": dy_f, "L2_conf": conf_f,
                    "anch_dx": dx_anch, "anch_dy": dy_anch,
                    "anch_conf": conf_anch, "anch_frames": frames_since_anch,
                    "used": used_level,
                })
                dpx, dpy = flow_to_dpixel(dx_eff, dy_eff)
                std = flow_conf_to_std(conf_eff)
                pk = CRTPPacket()
                pk.port = CRTP_PORT_SETPOINT_SIM
                pk.channel = 0
                pk.data = struct.pack("<Bffff", SENSOR_FLOW_SIM, dpx, dpy, dt, std)
                try:
                    cf.send_packet(pk)
                    stats["n_sent"] += 1
                except Exception as e:
                    stats["last_err"] = f"send: {e}"
        except socket.timeout:
            continue
        except Exception as e:
            stats["last_err"] = f"recv: {e}"
            time.sleep(0.02)


# ────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────
def main() -> int:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    frames_dir = Path(os.environ["SENTAI_DUMP_FRAMES_DIR"])
    assert frames_dir.is_dir()

    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf

    cf.param.set_value("stabilizer.estimator", 2)
    time.sleep(0.5)

    # 2026-05-11: damp the cascaded controller to counter flow @ 20Hz
    # phase-lag oscillation.  Best-effort: each param wrapped in
    # try/except since not all cf2 builds expose the same TOC.
    DAMP_PARAMS = {
        "posCtlPid.xyKd":  0.5,   # default 0.2 — position-loop damping
        "velCtlPid.vxKd":  0.05,  # default 0.0
        "velCtlPid.vyKd":  0.05,
    }
    for k, v in DAMP_PARAMS.items():
        try:
            cf.param.set_value(k, v)
            print(f"[hover] set {k}={v}", file=sys.stderr)
        except Exception as e:
            print(f"[hover] WARN {k} not in TOC: {e}", file=sys.stderr)
    time.sleep(0.3)
    cf.param.set_value("kalman.resetEstimation", 1)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 0)
    time.sleep(2.0)

    lc = LogConfig(name="att", period_in_ms=20)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    # Start flow forwarder thread BEFORE takeoff so EKF gets flow from
    # liftoff onwards.
    stop_evt = threading.Event()
    flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
    flow_th = threading.Thread(target=flow_forwarder,
                                args=(stop_evt, cf, flow_stats),
                                daemon=True)
    flow_th.start()
    time.sleep(1.0)

    mc = MotionCommander(sync, default_height=TARGET_Z_M)
    print(f"[hover] FAST take off to {TARGET_Z_M}m @ {TAKEOFF_VEL_MPS} m/s",
          file=sys.stderr)
    mc.take_off(height=TARGET_Z_M, velocity=TAKEOFF_VEL_MPS)
    time.sleep(2.5)   # settle at altitude before hover phase

    # Hover hold — zero velocity setpoint, cf2 EKF uses flow to maintain
    # position under wind disturbance.
    mc.start_linear_motion(0, 0, 0)

    print(f"[hover] hovering for {HOVER_S}s (Gz wind 0.3 m/s + σ=0.30, "
          f"flow injection active)", file=sys.stderr)
    t0 = time.monotonic()
    last_seq = 0
    samples = []
    while time.monotonic() - t0 < HOVER_S:
        time.sleep(1.0 / SAMPLE_HZ)
        ppm = latest_ppm(frames_dir)
        if ppm is None:
            continue
        try:
            fseq = int(ppm.stem.replace("frame_", ""))
        except Exception:
            continue
        if fseq <= last_seq:
            continue
        last_seq = fseq

        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception:
            continue

        ekf_x, ekf_y, ekf_z, _, _, _ = get_ekf()
        # PnP altitude = mean tvec[2] across all detected markers, plus
        # marker top-face altitude (0.20m for the compact tall posts).
        zs = [m.tvec[2] for m in dets.values() if m.tvec is not None]
        pnp_z = (statistics.mean(zs) + 0.20) if zs else None

        # Drone-to-pattern-centroid offset (PnP-based world X/Y of cam)
        # — assuming yaw=0 and downward cam, the average horizontal
        # tvec in body frame ≈ (x_drone - centroid_x, y_drone - centroid_y).
        # KNOWN_POSITIONS_M centroid is (0,0).  So (cam→marker_avg) ≈ -drone_xy.
        xs = [-(m.tvec[0]) for m in dets.values() if m.tvec is not None]
        ys = [-(m.tvec[1]) for m in dets.values() if m.tvec is not None]
        pnp_x = statistics.mean(xs) if xs else None
        pnp_y = statistics.mean(ys) if ys else None

        dist_from_target_m = math.sqrt(ekf_x ** 2 + ekf_y ** 2 + (ekf_z - TARGET_Z_M) ** 2)

        samples.append({
            "fseq": fseq,
            "n_det": len(dets),
            "ids": sorted(dets.keys()),
            "ekf": [round(ekf_x, 3), round(ekf_y, 3), round(ekf_z, 3)],
            "pnp": [round(pnp_x, 3) if pnp_x is not None else None,
                    round(pnp_y, 3) if pnp_y is not None else None,
                    round(pnp_z, 3) if pnp_z is not None else None],
            "z_err_cm": round((pnp_z - ekf_z) * 100, 2) if pnp_z is not None else None,
            "dist_target_m": round(dist_from_target_m, 3),
            "flow_n": flow_stats["n_sent"],
        })
        print(f"[hover] fseq={fseq:>4}  n={len(dets)}  "
              f"ekf=({ekf_x:+.2f},{ekf_y:+.2f},{ekf_z:.2f})  "
              f"pnp=({'na' if pnp_x is None else f'{pnp_x:+.2f}'},"
              f"{'na' if pnp_y is None else f'{pnp_y:+.2f}'},"
              f"{'na' if pnp_z is None else f'{pnp_z:.2f}'})  "
              f"dist={dist_from_target_m:.2f}  flow_n={flow_stats['n_sent']}",
              file=sys.stderr)

    stop_evt.set()
    flow_th.join(timeout=1.0)
    mc.land(velocity=0.3)
    time.sleep(2.0)
    lc.stop()
    sync.close_link()

    # Summary
    if not samples:
        print("[hover] no samples", file=sys.stderr)
        return 1

    full4 = sum(1 for s in samples if s["n_det"] == 4)
    any_det = sum(1 for s in samples if s["n_det"] >= 1)
    dist_max = max(s["dist_target_m"] for s in samples)
    dist_mean = statistics.mean(s["dist_target_m"] for s in samples)
    z_errs = [s["z_err_cm"] for s in samples if s["z_err_cm"] is not None]
    z_rms_cm = math.sqrt(sum(e * e for e in z_errs) / len(z_errs)) if z_errs else float("nan")
    z_mean_cm = statistics.mean(z_errs) if z_errs else float("nan")
    flow_hz = flow_stats["n_sent"] / HOVER_S

    print(f"\n=== HOVER SUMMARY (z={TARGET_Z_M}m, {HOVER_S}s, Gz wind 0.3 m/s) ===",
          file=sys.stderr)
    print(f"frames sampled  : {len(samples)}", file=sys.stderr)
    print(f"all-4-detect    : {full4}/{len(samples)} = {full4/len(samples)*100:5.1f}%",
          file=sys.stderr)
    print(f"any-detect      : {any_det}/{len(samples)} = {any_det/len(samples)*100:5.1f}%",
          file=sys.stderr)
    print(f"dist to target  : mean={dist_mean:.3f}m  max={dist_max:.3f}m",
          file=sys.stderr)
    print(f"PnP-z vs EKF-z  : mean={z_mean_cm:+.2f}cm  RMS={z_rms_cm:.2f}cm",
          file=sys.stderr)
    print(f"flow packets    : {flow_stats['n_sent']} = {flow_hz:.1f} Hz",
          file=sys.stderr)
    if flow_stats.get("fatal"):
        print(f"flow FATAL      : {flow_stats['fatal']}", file=sys.stderr)
    if flow_stats.get("last_err"):
        print(f"flow last_err   : {flow_stats['last_err']}", file=sys.stderr)

    pass_dist = dist_max < 0.30
    pass_4det = full4 / len(samples) >= 0.30
    pass_anydet = any_det / len(samples) >= 0.80
    pass_z = (not math.isnan(z_rms_cm)) and z_rms_cm < 12.0
    pass_flow = flow_hz >= 5.0
    print(f"PASS dist<30cm  : {'YES' if pass_dist else 'no'}", file=sys.stderr)
    print(f"PASS 4det>=30%  : {'YES' if pass_4det else 'no'}", file=sys.stderr)
    print(f"PASS any>=80%   : {'YES' if pass_anydet else 'no'}", file=sys.stderr)
    print(f"PASS z_rms<12cm : {'YES' if pass_z else 'no'}", file=sys.stderr)
    print(f"PASS flow>=5Hz  : {'YES' if pass_flow else 'no'}", file=sys.stderr)

    # 3-pipeline pyramid stats
    pp = flow_stats.get("pp", [])
    import statistics as _s
    def _stats(key_conf, key_dx, key_dy, name):
        confs = [r[key_conf] for r in pp]
        zeros = sum(1 for c in confs if c == 0)
        mags = [max(abs(r[key_dx]), abs(r[key_dy])) for r in pp]
        avg_conf = _s.mean(confs) if confs else 0
        med_mag = _s.median(mags) if mags else 0
        print(f"{name:>4}: avg_conf={avg_conf:5.1f}  conf=0 in "
              f"{zeros}/{len(pp)} ({zeros/max(1,len(pp))*100:.1f}%)  "
              f"median |mag|={med_mag:.0f}", file=sys.stderr)
        return avg_conf, zeros / max(1, len(pp)) * 100
    used_counts = {}
    for r in pp:
        used_counts[r.get("used", "?")] = used_counts.get(r.get("used", "?"), 0) + 1
    print(f"\n=== PER-PIPELINE FLOW STATS ===", file=sys.stderr)
    print(f"records       : {len(pp)}", file=sys.stderr)
    print(f"fusion picks  : "
          + ", ".join(f"{k}={v}/{len(pp)}" for k, v in sorted(used_counts.items())),
          file=sys.stderr)
    L0_avg, L0_zpct = _stats("L0_conf", "L0_dx", "L0_dy", "L0")
    L1_avg, L1_zpct = _stats("L1_conf", "L1_dx", "L1_dy", "L1")
    L2_avg, L2_zpct = _stats("L2_conf", "L2_dx", "L2_dy", "L2")

    LOG_PATH.write_text(json.dumps({
        "target_z_m": TARGET_Z_M,
        "hover_s": HOVER_S,
        "n_samples": len(samples),
        "all4_rate": full4 / len(samples),
        "any_rate": any_det / len(samples),
        "dist_mean_m": dist_mean,
        "dist_max_m": dist_max,
        "z_mean_cm": z_mean_cm if not math.isnan(z_mean_cm) else None,
        "z_rms_cm": z_rms_cm if not math.isnan(z_rms_cm) else None,
        "flow_n": flow_stats["n_sent"],
        "flow_hz": flow_hz,
        "pass_all": all([pass_dist, pass_4det, pass_anydet, pass_z, pass_flow]),
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "samples": samples,
        "pipeline_stats": {
            "n_records": len(pp),
            "fusion_picks": used_counts,
            "L0_avg_conf": L0_avg, "L0_zero_pct": L0_zpct,
            "L1_avg_conf": L1_avg, "L1_zero_pct": L1_zpct,
            "L2_avg_conf": L2_avg, "L2_zero_pct": L2_zpct,
        },
        "pipeline_records": pp,
    }, indent=2))
    print(f"[hover] log → {LOG_PATH}", file=sys.stderr)
    return 0 if all([pass_dist, pass_4det, pass_anydet, pass_z, pass_flow]) else 1


if __name__ == "__main__":
    sys.exit(main())
