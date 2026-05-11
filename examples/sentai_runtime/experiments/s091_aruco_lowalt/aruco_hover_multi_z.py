"""Multi-altitude hover with sentai.flow under Gz wind.

User asked to test at 2-3 altitudes to see how hold quality scales with z.
Hypothesis: at higher z the FOV ground footprint widens, so per-pixel
ground motion is smaller for the same body velocity → lower mgrid/frame
flow signal → EKF observes smaller "perceived motion" per unit drone
drift.  Effect on hold:
  - higher z = LESS sensitive to small drifts (flow underreports motion)
  - lower z = MORE sensitive but also more risk of marker leaving FOV

At z=0.5: per-grid ground = 0.554/80 = 6.93 mm
At z=1.0: per-grid ground = 1.109/80 = 13.86 mm  → ½ resolution
At z=1.5: per-grid ground = 1.663/80 = 20.79 mm  → ⅓ resolution

So flow signal-to-motion ratio drops linearly with z.  Validation:
hover under same wind at each altitude, compare drift.
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

# Reuse constants + helpers from aruco_hover
sys.path.insert(0, str(Path(__file__).parent))
from aruco_hover import (
    flow_to_dpixel, flow_conf_to_std,
    BODY_XFORM, FLOW_OUT_SOCK, REPLY_MAGIC, REPLY_FMT, REPLY_SZ,
    CRTP_PORT_SETPOINT_SIM, SENSOR_FLOW_SIM,
    _att_cb, get_ekf,
)

Z_LIST_M       = [0.5, 1.0, 1.5]
HOVER_PER_Z_S  = 12.0
SAMPLE_HZ      = 5
LOG_PATH       = Path(__file__).parent / "multi_z_log.json"


def flow_forwarder(stop_evt, cf, stats):
    from cflib.crtp.crtpstack import CRTPPacket
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
        stats["fatal"] = "no flow socket"
        return
    last_t = time.monotonic()
    buf = b""
    while not stop_evt.is_set():
        try:
            chunk = sock.recv(4096)
            if not chunk:
                time.sleep(0.005); continue
            buf += chunk
            while len(buf) >= REPLY_SZ:
                rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
                magic, seq, dx, dy, conf, lat, dz, dz_conf = struct.unpack(REPLY_FMT, rec)
                if magic != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                now = time.monotonic()
                dt = max(0.001, min(0.2, now - last_t))
                last_t = now
                dpx, dpy = flow_to_dpixel(dx, dy)
                std = flow_conf_to_std(conf)
                pk = CRTPPacket()
                pk.port = CRTP_PORT_SETPOINT_SIM
                pk.channel = 0
                pk.data = struct.pack("<Bffff", SENSOR_FLOW_SIM, dpx, dpy, dt, std)
                try:
                    cf.send_packet(pk); stats["n_sent"] += 1
                except Exception as e:
                    stats["last_err"] = f"send: {e}"
        except socket.timeout:
            continue
        except Exception as e:
            stats["last_err"] = f"recv: {e}"
            time.sleep(0.02)


def hover_at_z(mc, target_z, frames_dir, stats_phase):
    """Climb to target_z, hover HOVER_PER_Z_S seconds, return statistics."""
    print(f"\n[multi_z] >>> climbing to z={target_z:.2f}m", file=sys.stderr)
    # climb open-loop
    KP_Z = 0.6
    t0 = time.monotonic()
    while time.monotonic() - t0 < 12.0:
        ekf_x, ekf_y, ekf_z, _, _, _ = get_ekf()
        if abs(ekf_z - target_z) < 0.10:
            break
        vz = max(-0.4, min(0.4, (target_z - ekf_z) * KP_Z))
        mc.start_linear_motion(0, 0, vz)
        time.sleep(0.1)
    mc.start_linear_motion(0, 0, 0)
    time.sleep(1.5)   # settle

    print(f"[multi_z] hovering at z={target_z:.2f}m for {HOVER_PER_Z_S}s",
          file=sys.stderr)
    t_start = time.monotonic()
    flow_n_start = stats_phase["n_sent"]
    last_seq = 0
    samples = []
    while time.monotonic() - t_start < HOVER_PER_Z_S:
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
        zs = [m.tvec[2] for m in dets.values() if m.tvec is not None]
        pnp_z = statistics.mean(zs) + 0.20 if zs else None
        dist_lateral = math.hypot(ekf_x, ekf_y)
        samples.append({
            "fseq": fseq,
            "t": round(time.monotonic() - t_start, 2),
            "n_det": len(dets),
            "ekf": [round(ekf_x, 3), round(ekf_y, 3), round(ekf_z, 3)],
            "pnp_z": round(pnp_z, 3) if pnp_z is not None else None,
            "dist_lateral_m": round(dist_lateral, 3),
            "dist_z_err_m": round(abs(ekf_z - target_z), 3),
        })

    flow_n_end = stats_phase["n_sent"]
    flow_n_phase = flow_n_end - flow_n_start
    if not samples:
        return {"target_z": target_z, "n_samples": 0}

    lat_dists = [s["dist_lateral_m"] for s in samples]
    z_errs = [s["dist_z_err_m"] for s in samples]
    return {
        "target_z": target_z,
        "n_samples": len(samples),
        "lat_dist_mean_m": round(statistics.mean(lat_dists), 3),
        "lat_dist_max_m": round(max(lat_dists), 3),
        "lat_dist_rms_m": round(math.sqrt(sum(d*d for d in lat_dists)/len(lat_dists)), 3),
        "z_err_mean_m": round(statistics.mean(z_errs), 3),
        "z_err_max_m": round(max(z_errs), 3),
        "n_det_avg": round(statistics.mean(s["n_det"] for s in samples), 2),
        "flow_n_phase": flow_n_phase,
        "flow_hz_phase": round(flow_n_phase / HOVER_PER_Z_S, 2),
        "samples": samples,
    }


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
    # NOTE: motion.disable param is not in cf2 SITL firmware TOC for this
    # build — the flowdeck driver isn't compiled in, so no need to disable.
    # If a future build includes it, add try/except here.
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

    stop_evt = threading.Event()
    flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
    flow_th = threading.Thread(target=flow_forwarder,
                                args=(stop_evt, cf, flow_stats),
                                daemon=True)
    flow_th.start()
    time.sleep(1.0)

    mc = MotionCommander(sync, default_height=Z_LIST_M[0])
    mc.take_off(height=Z_LIST_M[0], velocity=0.3)
    time.sleep(2.5)

    phases = []
    try:
        for tz in Z_LIST_M:
            phase = hover_at_z(mc, tz, frames_dir, flow_stats)
            phases.append(phase)
            if phase.get("n_samples", 0) > 0:
                print(f"[multi_z] z={tz:.2f}m  "
                      f"lat_mean={phase['lat_dist_mean_m']:.3f}m  "
                      f"lat_max={phase['lat_dist_max_m']:.3f}m  "
                      f"z_err_mean={phase['z_err_mean_m']:.3f}m  "
                      f"flow_hz={phase['flow_hz_phase']:.1f}",
                      file=sys.stderr)
    except Exception as e:
        print(f"[multi_z] EXCEPTION: {e}", file=sys.stderr)
    finally:
        try:
            mc.start_linear_motion(0, 0, 0)
            time.sleep(0.3)
            mc.land(velocity=0.3)
            time.sleep(2.0)
        except Exception as e:
            print(f"[multi_z] WARN land: {e}", file=sys.stderr)
        stop_evt.set()
        flow_th.join(timeout=1.0)
        lc.stop()
        sync.close_link()

    print(f"\n=== MULTI-Z HOVER SUMMARY ===", file=sys.stderr)
    print(f"{'z':>5}  {'samples':>7}  {'lat_mean':>9}  {'lat_max':>8}  "
          f"{'z_err_mean':>10}  {'flow_hz':>7}", file=sys.stderr)
    for p in phases:
        if p.get("n_samples", 0) == 0:
            print(f"{p['target_z']:>5.2f}  --no samples--", file=sys.stderr)
            continue
        print(f"{p['target_z']:>5.2f}  {p['n_samples']:>7}  "
              f"{p['lat_dist_mean_m']:>9.3f}  {p['lat_dist_max_m']:>8.3f}  "
              f"{p['z_err_mean_m']:>10.3f}  {p['flow_hz_phase']:>7.1f}",
              file=sys.stderr)

    LOG_PATH.write_text(json.dumps({
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "phases": phases,
        "flow_total": flow_stats["n_sent"],
    }, indent=2))
    print(f"[multi_z] log → {LOG_PATH}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
