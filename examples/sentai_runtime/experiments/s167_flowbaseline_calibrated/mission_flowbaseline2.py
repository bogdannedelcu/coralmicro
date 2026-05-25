"""FlowBaseline2 — 4-phase calibrated hover (OP-S8-W1-T8 path).

Crisis context ([[op-s8-w1-cf2-sim-honest]]): the cheat plugin gone,
s166 trial #1 showed cf2 EKF diverges from GT by 67-92 cm during a 15 s
hover at z=1 m.  Position-PID chases EKF → positive-feedback drift
spirals the drone out.

Calibration-first design (operator spec 2026-05-18):

  F1  takeoff @ 0.6 m         ~3 s   wide-FOV: all 4 markers in view
  F2a static pin @ 0.6 m       2 s   VPE @ 20 Hz conf-max → EKF locked
       + PnP-z baseline averaged → SEED FOR F3
  F2b axes probe @ 0.6 m      ~6 s   4 directional steps ±4 cm,
       measure (flow_integral, ekf_delta) per leg
       → fit BODY_XFORM 2×2 empirically (override SIM hardcode)
  F3  climb to 1.0 m + 10 s measurement hover, VPE @ 5 Hz gentle
       (this is the actual baseline measurement; gated on GT closure)
  F4  land

Outputs: phases.json with per-phase metrics + samples + derived body_xform.
hover_log.json keeps the per-sample + ekf_trace shape from s166's
aruco_hover so the s166 verdict can plot the F3 portion directly.
"""
from __future__ import annotations

import csv
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

# Reuse s090's marker layout + PnP helpers.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "s090_hover_over_cat"))
from aruco_detector import (detect_in_ppm, latest_ppm,
                             KNOWN_POSITIONS_M, estimate_drone_world_pose)

# ────────────────────────────────────────────────────────────
# Constants — same flow math as s091/s166 aruco_hover.
# ────────────────────────────────────────────────────────────
FLOW_FOV_H_DEG    = 58.0
FLOW_FOV_V_DEG    = 45.0
FLOW_GRID_W       = 80
FLOW_GRID_H       = 60
DRONE_NPIX        = 35.0
DRONE_THETAPIX    = 0.71674
DRONE_FLOW_RES    = 0.10
# Default (SIM-empirical from aruco_hover) — may be REPLACED by F2b
# probe-derived matrix.  Stored as (fw_from_dx, fw_from_dy, lf_from_dx, lf_from_dy)
BODY_XFORM_DEFAULT = (0.0, -1.0, -1.0, 0.0)
_FLOW_SCALE_X = (math.radians(FLOW_FOV_H_DEG) * DRONE_NPIX) / \
                (FLOW_GRID_W * DRONE_FLOW_RES * DRONE_THETAPIX)
_FLOW_SCALE_Y = (math.radians(FLOW_FOV_V_DEG) * DRONE_NPIX) / \
                (FLOW_GRID_H * DRONE_FLOW_RES * DRONE_THETAPIX)
CRTP_PORT_SETPOINT_SIM = 0x09
SENSOR_FLOW_SIM        = 6

FLOW_OUT_SOCK    = "/tmp/sentai_flow_out.sock"
REPLY_MAGIC      = 0x46524C31
REPLY_FMT        = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_SZ         = struct.calcsize(REPLY_FMT)
assert REPLY_SZ == 124, f"unexpected REPLY_SZ={REPLY_SZ}"

# Phase timing + setpoints (iter #3 — PnP in-loop closed-loop calibration).
Z_LOW            = 0.75     # F2 altitude — +50% vs iter #2 (operator 2026-05-18)
Z_HIGH           = 0.80     # F3 measurement altitude
TAKEOFF_VEL_MPS  = 0.3
F1_SETTLE_S      = 2.0
F2A_S            = 2.0      # short passive baseline (PnP-z mean) before iter loop

# F2-iter PnP-driven closed-loop calibration knobs.
F2_VS_PROBE_MPS       = 0.06       # body-velocity per impulse leg
F2_IMPULSE_S          = 0.5        # impulse duration → drone accelerates + holds vs
F2_INERTIA_S          = 0.3        # coast post-impulse (drone keeps moving via inertia)
                                    # snap_after happens HERE — captures real response
                                    # before flow damping erases the motion.  Operator
                                    # spec 2026-05-18 ("inertia factor").
F2_SETTLE_S           = 1.2        # post-snap hover settle (was 1.5; 0.3 moved to inertia)
F2_RETURN_VS_MAX_MPS  = 0.08       # cap on auto-return velocity
F2_RETURN_S           = 0.5        # auto-return duration
F2_ANCHOR_DRIFT_M     = 0.05       # drift threshold to trigger auto-return
F2_MAX_ITERS          = 12         # operator-chosen "tipic"
F2_CONV_SIGMA_PCT     = 0.10       # 10% rolling std on BX entries
F2_CONV_WINDOW        = 4          # last-N iterations for convergence check
F2_BX_DET_MIN         = 1e-3       # |raw_det| floor for safe-to-swap (NEW)

F3_CLIMB_S       = 2.0
F3_HOVER_S       = 10.0
SAMPLE_HZ        = 5
LANDING_VEL_MPS  = 0.3
# Crash detector — abort mission if drone "lands itself".
CRASH_Z_M        = 0.20
CRASH_DWELL_S    = 0.6

# Working dir for s167 artefacts.
WORKDIR          = Path("/tmp/s167_flowbaseline_calibrated")
WORKDIR.mkdir(parents=True, exist_ok=True)
PHASES_PATH      = WORKDIR / "phases.json"
# Mirror s166's per-sample log shape so s166 verdict can be reused on F3 alone.
HOVER_LOG_PATH   = Path(__file__).resolve().parents[1] / "s091_aruco_lowalt" / "hover_log.json"


# ────────────────────────────────────────────────────────────
# Globals — cf2 EKF state + high-rate EKF trace + flow stats
# ────────────────────────────────────────────────────────────
_lock = threading.Lock()
_ekf_x = _ekf_y = _ekf_z = 0.0
_roll = _pitch = _yaw = 0.0
_ekf_trace: list[dict] = []

# VPE forwarder rate cap — mutable so the orchestrator can rate-up in F2.
_VPE_RATE_HZ      = 5.0
_VPE_ON           = True
_vpe_last_send_t  = 0.0


def _att_cb(_ts, data, _lc):
    global _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw
    with _lock:
        _ekf_x = data["stateEstimate.x"]
        _ekf_y = data["stateEstimate.y"]
        _ekf_z = data["stateEstimate.z"]
        _roll  = data["stateEstimate.roll"]
        _pitch = data["stateEstimate.pitch"]
        _yaw   = data["stateEstimate.yaw"]
        _ekf_trace.append({
            "t_wall": time.monotonic(),
            "x": _ekf_x, "y": _ekf_y, "z": _ekf_z, "yaw": _yaw,
        })


def get_ekf():
    with _lock:
        return _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw


# ────────────────────────────────────────────────────────────
# Flow forwarder — same protocol as aruco_hover.
# ────────────────────────────────────────────────────────────
def flow_to_dpixel(dx_q1000: int, dy_q1000: int,
                   bx: tuple[float, float, float, float]
                   ) -> tuple[float, float]:
    dx_grid = dx_q1000 / 1000.0
    dy_grid = dy_q1000 / 1000.0
    fw_dx, fw_dy, lf_dx, lf_dy = bx
    fw_grid   = fw_dx * dx_grid + fw_dy * dy_grid
    left_grid = lf_dx * dx_grid + lf_dy * dy_grid
    return (fw_grid * _FLOW_SCALE_X, left_grid * _FLOW_SCALE_Y)


def flow_conf_to_std(conf: int) -> float:
    if conf >= 200: return 3.0
    if conf >= 128: return 4.0
    if conf >= 64:  return 6.0
    return 10.0


def flow_forwarder(stop_evt: threading.Event, cf, stats: dict,
                   body_xform_ref: dict) -> None:
    """Reads /tmp/sentai_flow_out.sock, forwards to cf2 EKF.

    body_xform_ref is a mutable container {'bx': tuple} so the F2b probe
    can swap the matrix mid-mission without restarting the thread.
    """
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
                fields = struct.unpack(REPLY_FMT, rec)
                (magic, seq, dx, dy, conf, lat, dz, dz_conf,
                 dx_c, dy_c, conf_c, dx_f, dy_f, conf_f,
                 dx_anch, dy_anch, conf_anch, frames_since_anch,
                 dx_anch_L2, dy_anch_L2, conf_anch_L2, frames_since_anch_L2,
                 dx_anch_L1, dy_anch_L1, conf_anch_L1, frames_since_anch_L1,
                 dx_best, dy_best, conf_best, best_source) = fields
                if magic != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                now = time.monotonic()
                dt = max(0.001, min(0.2, now - last_send_t))
                last_send_t = now
                bx = body_xform_ref["bx"]
                dpx, dpy = flow_to_dpixel(dx_best, dy_best, bx)
                std = flow_conf_to_std(conf_best)
                stats.setdefault("pp", []).append({
                    "seq": seq, "t": now,
                    "best_dx": dx_best, "best_dy": dy_best,
                    "best_conf": conf_best, "best_source": best_source,
                    "dpx": dpx, "dpy": dpy, "std": std,
                })
                pk = CRTPPacket()
                pk.port = CRTP_PORT_SETPOINT_SIM
                pk.channel = 0
                pk.data = struct.pack("<Bffff",
                                       SENSOR_FLOW_SIM, dpx, dpy, dt, std)
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


def vpe_send_if_due(cf, x, y, z, stats: dict) -> None:
    """Send extpos (PnP-derived) honouring _VPE_ON + _VPE_RATE_HZ."""
    global _vpe_last_send_t
    if not _VPE_ON:
        return
    now = time.monotonic()
    if (now - _vpe_last_send_t) < (1.0 / _VPE_RATE_HZ):
        return
    _vpe_last_send_t = now
    try:
        cf.extpos.send_extpos(float(x), float(y), float(z))
        stats["vpe_n_sent"] = stats.get("vpe_n_sent", 0) + 1
    except Exception as e:
        stats["vpe_last_err"] = f"{type(e).__name__}: {e}"


# ────────────────────────────────────────────────────────────
# PnP capture loop — driven by the most recent dumped frame.
# Yields PnP poses at the cadence of the frame-dump (~33 Hz).
# ────────────────────────────────────────────────────────────
# Module-level inspect-dir + mission start — set by main() if
# SENTAI_INSPECT_DIR env is provided.  Each polled frame gets
# copied with descriptive name "tMMMMMMMM_nX_fNNNNNN.ppm" so the
# operator can visually verify marker visibility post-trial.
_inspect_dir: Path | None = None
_mission_t0_wall: float | None = None


class PnPCollector:
    """Continuous PnP processor.

    Iter #6 change (2026-05-18): runs a BACKGROUND THREAD that processes
    every dumped frame at full camera FPS (~11 Hz at SENTAI_DUMP_FRAMES_EVERY=3).
    Each fresh frame:
      1. Runs ArUco detection + PnP.
      2. Updates SafetyMonitor (so safety triggers see EVERY frame, not
         just frames lucky enough to coincide with a phase poll).
      3. Hardlinks to inspect-dir.
      4. Caches result for phase code to read non-destructively.

    Phase code uses `latest_fresh(seq_seen)` to get the latest cached
    result; it advances its own seq_seen counter to avoid double-counting.
    The old `poll()` semantics are kept as a wrapper for back-compat.

    Operator-noted bug (2026-05-18): in iter #5, `_send_velocity_segment`
    (impulse/inertia/settle/return = ~30s of mission) did NOT call
    `pnp.poll()`, so SafetyMonitor saw zero frames during impulse legs.
    Streak counter only advanced at probe boundaries → max streak 15
    even though offline analysis showed 13+ seconds without n=4.
    Background-thread fix closes the visibility gap.
    """
    def __init__(self, frames_dir: Path):
        self.frames_dir = frames_dir
        self.last_seq = 0
        self.samples: list[dict] = []
        self._lock = threading.Lock()
        self._latest: tuple | None = None        # (pose, dets, fseq) or None
        self._stop_evt: threading.Event | None = None
        self._thread: threading.Thread | None = None

    def start_background(self) -> None:
        """Spawn the continuous-polling thread."""
        if self._thread is not None:
            return
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                          name="PnPCollector.bg")
        self._thread.start()

    def stop_background(self) -> None:
        if self._stop_evt is not None:
            self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _process_one_frame(self, ppm: Path, fseq: int) -> tuple | None:
        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception:
            return None
        _, _, _, _, _, ekf_yaw_deg = get_ekf()
        pose = estimate_drone_world_pose(dets, drone_yaw=math.radians(ekf_yaw_deg))
        # Inspect-dir hardlink (operator visual review).
        if _inspect_dir is not None and _mission_t0_wall is not None:
            try:
                ms = int((time.monotonic() - _mission_t0_wall) * 1000)
                dst = _inspect_dir / f"t{ms:08d}_n{len(dets)}_f{fseq:06d}.ppm"
                try:
                    os.link(ppm, dst)
                except FileExistsError:
                    pass
                except OSError:
                    import shutil
                    shutil.copy(ppm, dst)
            except Exception as e:
                print(f"[inspect] copy failed: {e}", file=sys.stderr)
        # SafetyMonitor — runs at full FPS now (operator fix 2026-05-18).
        if _safety is not None:
            pnp_z = pose[2] if pose is not None else None
            _safety.update_pnp(pnp_z, len(dets))
        return (pose, dets, fseq)

    def _run(self) -> None:
        """Background loop — process every fresh dumped frame.

        CRITICAL: SafetyAbort raised inside `_process_one_frame` (via
        `_safety.update_pnp`) is caught HERE and translated into:
          1. set `_safety.aborted=True` (already done by update_pnp itself)
          2. stop the BG thread cleanly
          3. main thread sees `_safety.aborted` via `_check_crash` and
             raises SafetyAbort in its own context → emergency land.
        Without this catch, SafetyAbort in a daemon thread silently dies
        and the mission keeps flying blind.
        """
        while not self._stop_evt.is_set():
            ppm = latest_ppm(self.frames_dir)
            if ppm is None:
                time.sleep(0.01)
                continue
            try:
                fseq = int(ppm.stem.replace("frame_", ""))
            except Exception:
                time.sleep(0.01)
                continue
            if fseq <= self.last_seq:
                time.sleep(0.01)
                continue
            self.last_seq = fseq
            try:
                result = self._process_one_frame(ppm, fseq)
                if result is not None:
                    with self._lock:
                        self._latest = result
                else:
                    # SafetyMonitor must still see the failed detection
                    # as "no markers" so the lt4 streak grows.
                    if _safety is not None:
                        _safety.update_pnp(None, 0)
            except SafetyAbort:
                # Don't propagate from BG (would silently kill the
                # daemon thread).  Flag is set; main thread picks up
                # via _check_crash.
                print(f"[PnPCollector.bg] SafetyAbort flagged "
                      f"(reason={_safety.reason if _safety else '?'}); "
                      f"BG polling will stop.", file=sys.stderr)
                return
            time.sleep(0.01)

    # ── Phase-code APIs ────────────────────────────────────────────
    def latest_fresh(self, seq_seen: int) -> tuple | None:
        """Return (pose, dets, fseq) if a newer frame exists vs seq_seen.

        Use in phase loops to consume frames without race vs the
        background thread.  Caller persists its own seq_seen counter.
        """
        with self._lock:
            if self._latest is None:
                return None
            if self._latest[2] <= seq_seen:
                return None
            return self._latest

    def latest(self) -> tuple | None:
        """Return the most recently processed (pose, dets, fseq), without
        any freshness check.  Used for snapshot-style reads."""
        with self._lock:
            return self._latest

    def poll(self):
        """Back-compat shim: returns a fresh frame ONCE, then None until
        a newer frame arrives.  Uses internal seq tracking so existing
        phase code keeps working.
        """
        # Mimic the old behaviour: only return when caller hasn't seen
        # this fseq yet.  We track the caller's view via a counter
        # stored on self (legacy single-consumer assumption).
        if not hasattr(self, "_poll_seq_seen"):
            self._poll_seq_seen = 0
        r = self.latest_fresh(self._poll_seq_seen)
        if r is None:
            return None
        self._poll_seq_seen = r[2]
        return r


# ────────────────────────────────────────────────────────────
# Phase orchestration helpers
# ────────────────────────────────────────────────────────────
class CrashAbort(Exception):
    """Raised by phase loops when EKF z dwells below CRASH_Z_M for too long."""
    pass


class SafetyAbort(Exception):
    """Raised by SafetyMonitor on PnP-z floor, marker FOV loss, or EKF ceiling.

    See [[op-s8-w1-mission-safety-triggers]] for spec.  Iter #4
    addition 2026-05-18 — operator-noted after iter #3 crashed
    undetected mid-F2-iter (EKF z is unreliable post-crash).
    """
    pass


class SafetyMonitor:
    """Triggers a SafetyAbort on any of three orthogonal conditions.

    1. PnP-z floor: pnp_z < z_min_safe for >= dwell_s seconds
    2. Marker FOV loss: n_dets < 4 sustained for >= marker_loss_max_s seconds
       (iter #6 change 2026-05-18 — time-based instead of frame-count;
       operator: "asteptarea mea e ca dupa 1 second sa abortam daca nu
       avem markeri").  Rate-independent — works regardless of camera
       FPS / dump rate.
    3. EKF ceiling (catches post-crash integrator runaway): ekf_z > z_ceiling

    Caller invokes update_pnp(pz, n_dets) on every PnP frame (background
    thread runs at full camera FPS as of iter #6) and update_ekf(ekf_z)
    on every EKF read inside a phase loop.  Both raise SafetyAbort on
    trigger; main() catches and lands.
    """
    def __init__(self, z_min_safe: float = 0.20,
                  dwell_s: float = 0.5,
                  marker_loss_max_s: float = 1.0,
                  z_ceiling: float | None = None):
        self.z_min_safe = z_min_safe
        self.dwell_s = dwell_s
        self.marker_loss_max_s = marker_loss_max_s
        self.z_ceiling = z_ceiling
        self.last_pnp_below_floor_t: float | None = None
        self.last_n4_t: float | None = None    # time of most recent n_dets==4
        self.lt4_streak_start_t: float | None = None
        self.aborted: bool = False
        self.reason: str | None = None
        # Lightweight event log for forensics in phases.json.
        self.events: list[dict] = []

    def _log(self, kind: str, detail: dict) -> None:
        self.events.append({"t_wall": time.monotonic(),
                              "kind": kind, **detail})

    def update_pnp(self, pnp_z: float | None, n_dets: int) -> None:
        now = time.monotonic()
        # Trigger 1 — PnP-z floor.
        if pnp_z is not None:
            if pnp_z < self.z_min_safe:
                if self.last_pnp_below_floor_t is None:
                    self.last_pnp_below_floor_t = now
                    self._log("pnp_z_below_floor_start",
                              {"z": pnp_z, "limit": self.z_min_safe})
                elif now - self.last_pnp_below_floor_t >= self.dwell_s:
                    self.aborted = True
                    self.reason = (f"PnP-z={pnp_z:.3f}m < {self.z_min_safe}m "
                                    f"for >={self.dwell_s}s")
                    self._log("ABORT_pnp_z_floor",
                              {"z": pnp_z, "limit": self.z_min_safe,
                               "dwell_s": self.dwell_s})
                    raise SafetyAbort(self.reason)
            else:
                if self.last_pnp_below_floor_t is not None:
                    self._log("pnp_z_below_floor_recover", {"z": pnp_z})
                self.last_pnp_below_floor_t = None
        # Trigger 2 — "lost the 4-marker set" per HARD RULE
        # [[flowbaseline2-4markers-abort]] (operator 2026-05-18).
        # TIME-BASED (iter #6 fix): abort if n_dets<4 sustained for
        # >= marker_loss_max_s seconds.  Rate-independent.  Fluke n=4
        # detection resets the streak — that's INTENTIONAL per operator:
        # "e ok ca fluke sa salveze misiunia, nu e grav".
        if n_dets < 4:
            if self.lt4_streak_start_t is None:
                self.lt4_streak_start_t = now
            elapsed = now - self.lt4_streak_start_t
            if elapsed >= self.marker_loss_max_s:
                self.aborted = True
                self.reason = (f"<4 markers (last n_dets={n_dets}) for "
                                f"{elapsed:.2f}s >= {self.marker_loss_max_s}s")
                self._log("ABORT_lt4_markers",
                          {"elapsed_s": elapsed,
                           "limit_s": self.marker_loss_max_s,
                           "last_n_dets": n_dets})
                raise SafetyAbort(self.reason)
        else:
            # n_dets == 4 → reset the streak.
            if self.lt4_streak_start_t is not None:
                duration = now - self.lt4_streak_start_t
                self._log("4markers_recover",
                          {"prev_streak_s": round(duration, 3)})
            self.lt4_streak_start_t = None
            self.last_n4_t = now

    def update_ekf(self, ekf_z: float) -> None:
        # Trigger 3 — EKF ceiling (catches post-crash integrator runaway).
        if self.z_ceiling is not None and ekf_z > self.z_ceiling:
            self.aborted = True
            self.reason = f"EKF z={ekf_z:.2f}m > ceiling {self.z_ceiling:.2f}m"
            self._log("ABORT_ekf_ceiling",
                      {"z": ekf_z, "limit": self.z_ceiling})
            raise SafetyAbort(self.reason)

    def to_dict(self) -> dict:
        return {
            "z_min_safe": self.z_min_safe,
            "dwell_s": self.dwell_s,
            "marker_loss_max_s": self.marker_loss_max_s,
            "z_ceiling": self.z_ceiling,
            "aborted": self.aborted,
            "reason": self.reason,
            "events": self.events,
        }


# Module-level singleton — phase functions call via _safety, set in main().
_safety: SafetyMonitor | None = None


def _check_crash(crash_state: dict, ekf_z: float, label: str) -> None:
    """Update crash dwell tracker; raise CrashAbort if exceeded.

    Also forwards to the SafetyMonitor for EKF-ceiling check AND picks
    up any BG-thread-flagged abort (iter #6 fix — without this, a
    SafetyAbort raised inside the PnPCollector BG thread is silently
    swallowed and the main mission keeps flying).
    """
    now = time.monotonic()
    # ── Iter #6 fix: pick up BG-flagged SafetyAbort ─────────────────
    if _safety is not None and _safety.aborted:
        raise SafetyAbort(_safety.reason or "background SafetyAbort")
    # ── SafetyMonitor EKF-ceiling (independent of CRASH_Z_M) ────────
    if _safety is not None:
        _safety.update_ekf(ekf_z)
    if ekf_z < CRASH_Z_M:
        if crash_state.get("low_since") is None:
            crash_state["low_since"] = now
        elif now - crash_state["low_since"] >= CRASH_DWELL_S:
            crash_state["aborted"] = True
            raise CrashAbort(
                f"crash detector: EKF z={ekf_z:.3f}m < {CRASH_Z_M}m for "
                f">{CRASH_DWELL_S}s in {label}")
    else:
        crash_state["low_since"] = None


def phase_static_pin(cf, pnp: PnPCollector, flow_stats: dict,
                     setpoint_xyz: tuple[float, float, float],
                     duration_s: float,
                     vpe_rate_hz: float, phase_name: str,
                     crash_state: dict) -> dict:
    """F2a — PASSIVE hover observation via send_hover_setpoint(0,0,0,z).

    No position-PID drive; cf2 only holds altitude + zeroes body
    velocity from flow.  Operator note 2026-05-18: position_setpoint
    is closed-loop on a possibly-wrong EKF; until we have a real
    stabiliser, only velocity-/altitude-bounded commands are safe.

    Collects PnP captures for Z-baseline averaging.  Raises CrashAbort
    if EKF z drops below CRASH_Z_M for >CRASH_DWELL_S.
    """
    global _VPE_RATE_HZ
    _VPE_RATE_HZ = vpe_rate_hz
    t0 = time.monotonic()
    _, _, sz = setpoint_xyz
    last_sp_t = 0.0
    captures = []
    print(f"[{phase_name}] passive hover_setpoint(0,0,0,z={sz:.2f}) "
          f"for {duration_s}s, VPE@{vpe_rate_hz}Hz", file=sys.stderr)
    while time.monotonic() - t0 < duration_s:
        now = time.monotonic()
        if (now - last_sp_t) >= 0.05:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, sz)
            last_sp_t = now
        r = pnp.poll()
        if r is not None:
            pose, dets, fseq = r
            ekf_x, ekf_y, ekf_z, _, _, ekf_yaw_deg = get_ekf()
            _check_crash(crash_state, ekf_z, phase_name)
            if pose is not None:
                px, py, pz = pose
                vpe_send_if_due(cf, px, py, pz, flow_stats)
                captures.append({
                    "t": now - t0, "t_wall": now, "fseq": fseq,
                    "n_det": len(dets),
                    "pnp": [px, py, pz],
                    "ekf": [ekf_x, ekf_y, ekf_z],
                    "yaw_deg": ekf_yaw_deg,
                })
        time.sleep(0.02)
    # Summary numbers for the phase.
    pnp_z_list = [c["pnp"][2] for c in captures]
    pnp_xy_list = [(c["pnp"][0], c["pnp"][1]) for c in captures]
    ekf_xy_list = [(c["ekf"][0], c["ekf"][1]) for c in captures]
    summary = {
        "name": phase_name,
        "t0_wall": t0, "duration_s": duration_s,
        "setpoint": list(setpoint_xyz),
        "vpe_rate_hz": vpe_rate_hz,
        "n_captures": len(captures),
        "pnp_z_mean": statistics.mean(pnp_z_list) if pnp_z_list else None,
        "pnp_z_std":  statistics.pstdev(pnp_z_list) if len(pnp_z_list) > 1 else None,
        "pnp_xy_mean": [
            statistics.mean(c[0] for c in pnp_xy_list) if pnp_xy_list else None,
            statistics.mean(c[1] for c in pnp_xy_list) if pnp_xy_list else None,
        ] if pnp_xy_list else [None, None],
        "ekf_xy_mean": [
            statistics.mean(c[0] for c in ekf_xy_list) if ekf_xy_list else None,
            statistics.mean(c[1] for c in ekf_xy_list) if ekf_xy_list else None,
        ] if ekf_xy_list else [None, None],
        "captures": captures,
    }
    print(f"[{phase_name}] captures={len(captures)} "
          f"pnp_z_mean={summary['pnp_z_mean']!r} "
          f"ekf_xy=({summary['ekf_xy_mean'][0]!r},{summary['ekf_xy_mean'][1]!r})",
          file=sys.stderr)
    return summary


def _run_impulse_leg(cf, pnp: PnPCollector, flow_stats: dict,
                      vx_body: float, vy_body: float, z_hold: float,
                      impulse_s: float, settle_s: float,
                      label: str, crash_state: dict) -> dict:
    """One impulse leg: command body velocity for impulse_s, then hover_setpoint(0)
    for settle_s.  Returns the (ekf delta, flow integral, n_pp) for THIS leg.

    cf2 receives `send_hover_setpoint(vx, vy, yaw_rate, z)` — body-velocity
    + Z hold.  Open-loop in XY (no position positive-feedback), bounded by
    `vs × dt` regardless of EKF correctness.
    """
    # Snap baseline.
    t_start = time.monotonic()
    ekf_x0, ekf_y0, ekf_z0, _, _, ekf_yaw0 = get_ekf()
    pp_start = len(flow_stats.get("pp", []))
    # ── Impulse: drive body velocity ────────────────────────────────
    last_sp_t = 0.0
    while time.monotonic() - t_start < impulse_s:
        now = time.monotonic()
        if (now - last_sp_t) >= 0.05:
            cf.commander.send_hover_setpoint(vx_body, vy_body, 0.0, z_hold)
            last_sp_t = now
        _, _, ekf_z, _, _, _ = get_ekf()
        _check_crash(crash_state, ekf_z, f"F2b/{label}/impulse")
        time.sleep(0.02)
    # ── Settle: hover_setpoint(0,0,0,z) for settle_s ────────────────
    t_settle = time.monotonic()
    while time.monotonic() - t_settle < settle_s:
        now = time.monotonic()
        if (now - last_sp_t) >= 0.05:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_hold)
            last_sp_t = now
        _, _, ekf_z, _, _, _ = get_ekf()
        _check_crash(crash_state, ekf_z, f"F2b/{label}/settle")
        r = pnp.poll()
        if r is not None and r[0] is not None:
            vpe_send_if_due(cf, *r[0], flow_stats)
        time.sleep(0.02)
    # ── Snap end (after settle, so we measure NET displacement) ─────
    ekf_x1, ekf_y1, ekf_z1, _, _, ekf_yaw1 = get_ekf()
    pp_end = len(flow_stats.get("pp", []))
    leg_pp = flow_stats["pp"][pp_start:pp_end]
    flow_int_dx = sum(p["best_dx"] for p in leg_pp) / 1000.0
    flow_int_dy = sum(p["best_dy"] for p in leg_pp) / 1000.0
    # Body-frame EKF delta (rotate world delta by -yaw_mean).
    yaw_mean = math.radians((ekf_yaw0 + ekf_yaw1) / 2.0)
    cy_, sy_ = math.cos(-yaw_mean), math.sin(-yaw_mean)
    ekf_dx_w = ekf_x1 - ekf_x0
    ekf_dy_w = ekf_y1 - ekf_y0
    ekf_dx_body = cy_ * ekf_dx_w - sy_ * ekf_dy_w
    ekf_dy_body = sy_ * ekf_dx_w + cy_ * ekf_dy_w
    leg = {
        "label": label, "vx_body_cmd": vx_body, "vy_body_cmd": vy_body,
        "impulse_s": impulse_s, "settle_s": settle_s,
        "ekf_dx_body": ekf_dx_body, "ekf_dy_body": ekf_dy_body,
        "ekf_z_start": ekf_z0, "ekf_z_end": ekf_z1,
        "flow_int_dx_grid": flow_int_dx, "flow_int_dy_grid": flow_int_dy,
        "n_pp": len(leg_pp),
    }
    print(f"[F2b] {label:<14s} v_cmd=({vx_body:+.3f},{vy_body:+.3f}) m/s  "
          f"ekf_body=({ekf_dx_body:+.3f},{ekf_dy_body:+.3f}) m  "
          f"flow_int=({flow_int_dx:+.1f},{flow_int_dy:+.1f}) grid  "
          f"ekf_z=({ekf_z0:.2f}->{ekf_z1:.2f}) pp_n={len(leg_pp)}",
          file=sys.stderr)
    return leg


def _capture_pnp_with_retry(pnp: PnPCollector, n_tries: int = 25,
                              dt_s: float = 0.05):
    """Poll for a fresh PnP fix up to n_tries × dt_s seconds.

    Returns (px, py, pz, yaw_deg) or None.  PnP source is the latest
    dumped frame; under 6.6 Hz frame dump the loop usually finds one
    within 2 polls.
    """
    for _ in range(n_tries):
        r = pnp.poll()
        if r is not None and r[0] is not None:
            (px, py, pz), _dets, _fseq = r
            _, _, _, _, _, ekf_yaw_deg = get_ekf()
            return (px, py, pz, ekf_yaw_deg)
        time.sleep(dt_s)
    return None


def _send_velocity_segment(cf, vx_body: float, vy_body: float, z_hold: float,
                            duration_s: float, label: str,
                            crash_state: dict) -> None:
    """Drive body velocity (vx, vy) at z_hold for duration_s via hover_setpoint."""
    t0 = time.monotonic()
    last_sp_t = 0.0
    while time.monotonic() - t0 < duration_s:
        now = time.monotonic()
        if (now - last_sp_t) >= 0.05:
            cf.commander.send_hover_setpoint(vx_body, vy_body, 0.0, z_hold)
            last_sp_t = now
        _, _, ekf_z, _, _, _ = get_ekf()
        _check_crash(crash_state, ekf_z, label)
        time.sleep(0.02)


def _rotate_world_to_body(dx_w: float, dy_w: float, yaw_rad: float
                            ) -> tuple[float, float]:
    """Rotate a world-frame delta by -yaw → body-frame delta."""
    cy_, sy_ = math.cos(-yaw_rad), math.sin(-yaw_rad)
    return (cy_ * dx_w - sy_ * dy_w, sy_ * dx_w + cy_ * dy_w)


def _rotate_body_to_world(dx_b: float, dy_b: float, yaw_rad: float
                            ) -> tuple[float, float]:
    """Rotate a body-frame delta by +yaw → world-frame delta."""
    cy_, sy_ = math.cos(yaw_rad), math.sin(yaw_rad)
    return (cy_ * dx_b - sy_ * dy_b, sy_ * dx_b + cy_ * dy_b)


def phase_calibrate_pnp_iterative(cf, pnp: PnPCollector, flow_stats: dict,
                                    body_xform_ref: dict,
                                    z_hold: float,
                                    vpe_rate_hz: float,
                                    crash_state: dict) -> dict:
    """F2-iter — PnP-in-loop closed-loop calibration of BODY_XFORM.

    Operator design 2026-05-18.  Replaces iter #2's open-loop F2b which
    derived BX from EKF (circular: flow→EKF→BX→flow).  PnP gives an
    independent world-frame measurement of drone motion, breaking the
    loop.  After each probe, drift from the initial anchor is detected
    via PnP and corrected with an open-loop return impulse — drone
    stays over markers throughout the calibration.

    Algorithm per iteration i (i = 0 .. F2_MAX_ITERS-1):
        dir = round-robin from {(+vs,0), (-vs,0), (0,+vs), (0,-vs)}
        snap PnP@before, flow_idx@before
        send hover_setpoint(*dir) for F2_IMPULSE_S
        send hover_setpoint(0, 0, 0, z) for F2_SETTLE_S
        snap PnP@after, flow_idx@after
        pnp_delta_world = PnP@after - PnP@before
        pnp_delta_body  = R(-yaw_mean) @ pnp_delta_world
        flow_int_grid   = sum best_dx, best_dy over [idx0:idx1]
        history.append(...)
        BX = LSQ_fit(F, P)  on accumulated history
        drift_world = PnP@after - anchor_world
        if |drift_world| > F2_ANCHOR_DRIFT_M:
            vs_return_body = clip(-drift_body / F2_RETURN_S, F2_RETURN_VS_MAX_MPS)
            send hover_setpoint(vs_return_body, ...) for F2_RETURN_S
            send hover_setpoint(0, 0, 0, z) for F2_SETTLE_S
        if rolling_std(BX, F2_CONV_WINDOW) / |mean| < F2_CONV_SIGMA_PCT for all entries:
            break
    """
    import numpy as np
    global _VPE_RATE_HZ
    _VPE_RATE_HZ = vpe_rate_hz
    print(f"[F2-iter] PnP-in-loop calibration z={z_hold}m  vs={F2_VS_PROBE_MPS}m/s  "
          f"max_iters={F2_MAX_ITERS}  conv_sigma={F2_CONV_SIGMA_PCT*100:.0f}%  "
          f"VPE@{vpe_rate_hz}Hz", file=sys.stderr)
    # Anchor: PnP fix at start of phase.
    anchor = _capture_pnp_with_retry(pnp)
    if anchor is None:
        print(f"[F2-iter] FAIL — no PnP visible at phase start", file=sys.stderr)
        return {"name": "F2_iter_calib", "status": "no_anchor",
                "history": [], "BX_final": list(body_xform_ref["bx"])}
    anchor_x, anchor_y, anchor_z, anchor_yaw = anchor
    print(f"[F2-iter] anchor PnP = ({anchor_x:+.3f},{anchor_y:+.3f},{anchor_z:.3f}) "
          f"yaw={anchor_yaw:+.1f}°", file=sys.stderr)

    directions = [(+1.0, 0.0), (-1.0, 0.0), (0.0, +1.0), (0.0, -1.0)]
    dir_labels = ["+x", "-x", "+y", "-y"]
    history: list[dict] = []
    bx_history: list[tuple] = []   # for convergence check
    BX_current = None
    BX_raw_det = None
    converged_at = None
    aborted_reason = None

    for i in range(F2_MAX_ITERS):
        dx_unit, dy_unit = directions[i % 4]
        label = dir_labels[i % 4]
        vx_body = F2_VS_PROBE_MPS * dx_unit
        vy_body = F2_VS_PROBE_MPS * dy_unit
        # ── Snap before ──
        before = _capture_pnp_with_retry(pnp, n_tries=15)
        if before is None:
            print(f"[F2-iter] iter#{i:02d} {label}: no PnP before — skip",
                  file=sys.stderr)
            continue
        bx_w, by_w, bz_w, byaw = before
        flow_idx_before = len(flow_stats.get("pp", []))
        ekf_x0, ekf_y0, _, _, _, _ = get_ekf()
        # ── Probe (3 sub-segments: impulse → inertia coast → snap → settle) ──
        try:
            _send_velocity_segment(cf, vx_body, vy_body, z_hold,
                                     F2_IMPULSE_S, f"F2-iter/{label}/impulse",
                                     crash_state)
            # Inertia coast — drone continues moving from inertia while we
            # release the velocity command.  Capture response peak HERE.
            _send_velocity_segment(cf, 0.0, 0.0, z_hold,
                                     F2_INERTIA_S, f"F2-iter/{label}/inertia",
                                     crash_state)
        except CrashAbort:
            aborted_reason = f"crash during iter#{i:02d} {label}"
            print(f"[F2-iter] {aborted_reason}", file=sys.stderr)
            break
        # ── Snap after — at peak response, BEFORE settle damping ──
        after = _capture_pnp_with_retry(pnp, n_tries=15)
        # ── Settle (remainder) — let drone fully stop before next leg ──
        try:
            _send_velocity_segment(cf, 0.0, 0.0, z_hold,
                                     F2_SETTLE_S, f"F2-iter/{label}/settle",
                                     crash_state)
        except CrashAbort:
            aborted_reason = f"crash during settle iter#{i:02d}"
            print(f"[F2-iter] {aborted_reason}", file=sys.stderr)
            break
        # Re-fetch a refined after, in case the inertia snap was old
        # (the snap above was the canonical one for the response math).
        after_settled = _capture_pnp_with_retry(pnp, n_tries=15)
        if after is None:
            print(f"[F2-iter] iter#{i:02d} {label}: no PnP after — skip metrics",
                  file=sys.stderr)
            continue
        ax_w, ay_w, az_w, ayaw = after
        flow_idx_after = len(flow_stats.get("pp", []))
        ekf_x1, ekf_y1, ekf_z1, _, _, _ = get_ekf()
        leg_pp = flow_stats["pp"][flow_idx_before:flow_idx_after]
        flow_int_dx = sum(p["best_dx"] for p in leg_pp) / 1000.0
        flow_int_dy = sum(p["best_dy"] for p in leg_pp) / 1000.0
        # ── Body-frame PnP delta ──
        yaw_mean_rad = math.radians((byaw + ayaw) / 2.0)
        pnp_dx_b, pnp_dy_b = _rotate_world_to_body(
            ax_w - bx_w, ay_w - by_w, yaw_mean_rad)
        # ── Drift from anchor + auto-return decision ──
        drift_dx_w = ax_w - anchor_x
        drift_dy_w = ay_w - anchor_y
        drift_mag_w = math.hypot(drift_dx_w, drift_dy_w)
        did_return = False
        return_vs = (0.0, 0.0)
        if drift_mag_w > F2_ANCHOR_DRIFT_M:
            # Open-loop corrective return — derived purely from PnP delta,
            # no BX dependence so safe even if BX is still wrong.
            drift_dx_b, drift_dy_b = _rotate_world_to_body(
                drift_dx_w, drift_dy_w, math.radians(ayaw))
            target_vx_b = -drift_dx_b / F2_RETURN_S
            target_vy_b = -drift_dy_b / F2_RETURN_S
            # Clip per axis.
            m = max(abs(target_vx_b), abs(target_vy_b), 1e-9)
            if m > F2_RETURN_VS_MAX_MPS:
                k = F2_RETURN_VS_MAX_MPS / m
                target_vx_b *= k
                target_vy_b *= k
            return_vs = (target_vx_b, target_vy_b)
            print(f"[F2-iter] iter#{i:02d} drift={drift_mag_w*100:.2f}cm  "
                  f"return v_body=({target_vx_b:+.3f},{target_vy_b:+.3f}) m/s",
                  file=sys.stderr)
            try:
                _send_velocity_segment(cf, target_vx_b, target_vy_b, z_hold,
                                         F2_RETURN_S, f"F2-iter/{label}/return",
                                         crash_state)
                _send_velocity_segment(cf, 0.0, 0.0, z_hold,
                                         F2_SETTLE_S,
                                         f"F2-iter/{label}/return_settle",
                                         crash_state)
                did_return = True
            except CrashAbort:
                aborted_reason = f"crash during return iter#{i:02d}"
                print(f"[F2-iter] {aborted_reason}", file=sys.stderr)
                break
        # ── LSQ fit on history (independent of EKF — uses PnP) ──
        history.append({
            "iter": i, "label": label,
            "vx_body_cmd": vx_body, "vy_body_cmd": vy_body,
            "pnp_before":   [bx_w, by_w, bz_w],
            "pnp_after":    [ax_w, ay_w, az_w],          # @ inertia-end (peak response)
            "pnp_after_settled": (list(after_settled[:3])
                                    if after_settled is not None else None),
            "yaw_mean_deg": (byaw + ayaw) / 2.0,
            "pnp_dx_body": pnp_dx_b, "pnp_dy_body": pnp_dy_b,
            "flow_int_dx_grid": flow_int_dx, "flow_int_dy_grid": flow_int_dy,
            "ekf_dx_body": (ekf_x1 - ekf_x0), "ekf_dy_body": (ekf_y1 - ekf_y0),
            "n_pp": len(leg_pp),
            "drift_from_anchor_m": drift_mag_w,
            "did_return": did_return, "return_vs_body": list(return_vs),
        })
        Fm = np.array([[h["flow_int_dx_grid"], h["flow_int_dy_grid"]] for h in history])
        Pm = np.array([[h["pnp_dx_body"],     h["pnp_dy_body"]]      for h in history])
        try:
            # Solve P = F @ BX^T  →  BX^T = pinv(F) @ P
            BX_T, *_ = np.linalg.lstsq(Fm, Pm, rcond=None)
            BX_current = BX_T.T   # 2x2
            BX_raw_det = float(np.linalg.det(BX_current))
            bx_history.append(tuple(BX_current.flatten().tolist()))
        except Exception as e:
            print(f"[F2-iter] LSQ failed at iter#{i:02d}: {e}", file=sys.stderr)
        print(f"[F2-iter] iter#{i:02d} {label:<3s}  "
              f"pnp_body=({pnp_dx_b:+.3f},{pnp_dy_b:+.3f})m  "
              f"flow=({flow_int_dx:+.1f},{flow_int_dy:+.1f})grid  "
              f"drift={drift_mag_w*100:.1f}cm  ret={did_return}  "
              f"BX=[{','.join(f'{v:+.4f}' for v in BX_current.flatten())}]"
              f"  det={BX_raw_det:+.2e}",
              file=sys.stderr)
        # ── Convergence: rolling stdev on last N BX entries ──
        if len(bx_history) >= F2_CONV_WINDOW:
            window = np.array(bx_history[-F2_CONV_WINDOW:])  # (N, 4)
            means = np.mean(window, axis=0)
            stds  = np.std(window, axis=0)
            # Fractional stdev per entry, but guard against |mean| near 0.
            denoms = np.maximum(np.abs(means), 1e-4)
            frac_sigma = stds / denoms
            max_frac = float(np.max(frac_sigma))
            print(f"[F2-iter] iter#{i:02d} conv max_frac_sigma={max_frac:.3f} "
                  f"(target<{F2_CONV_SIGMA_PCT})", file=sys.stderr)
            if max_frac < F2_CONV_SIGMA_PCT:
                converged_at = i
                print(f"[F2-iter] converged at iter#{i:02d}", file=sys.stderr)
                break

    # ── Final BX decision (rank-check on raw det) ──
    safe_to_swap = False
    bx_normalised = None
    if BX_current is not None and abs(BX_raw_det) > F2_BX_DET_MIN:
        # Sign-snap rows.
        rows = []
        for r in BX_current:
            n = math.hypot(r[0], r[1])
            if n < 1e-9:
                rows.append((0.0, 0.0)); continue
            r0 = r[0] / n; r1 = r[1] / n
            if abs(r0) > abs(r1):
                rows.append((1.0 if r0 > 0 else -1.0, 0.0))
            else:
                rows.append((0.0, 1.0 if r1 > 0 else -1.0))
        bx_normalised = (rows[0][0], rows[0][1], rows[1][0], rows[1][1])
        import numpy as np
        snap_det = float(np.linalg.det(np.array(
            [[bx_normalised[0], bx_normalised[1]],
             [bx_normalised[2], bx_normalised[3]]])))
        safe_to_swap = abs(snap_det) > 0.5
        if not safe_to_swap:
            print(f"[F2-iter] snap_det={snap_det:.3f} — degenerate, no swap",
                  file=sys.stderr)
    else:
        det_disp = "n/a" if BX_raw_det is None else f"{BX_raw_det:.2e}"
        print(f"[F2-iter] |raw_det|={det_disp} < {F2_BX_DET_MIN} — "
              f"no swap (insufficient observability)", file=sys.stderr)

    return {
        "name": "F2_iter_calib",
        "status": "ok" if aborted_reason is None else "aborted",
        "aborted_reason": aborted_reason,
        "z_hold": z_hold,
        "vs_probe_mps": F2_VS_PROBE_MPS,
        "anchor_pnp_world": list(anchor[:3]),
        "n_iterations_run": len(history),
        "converged_at": converged_at,
        "history": history,
        "BX_default": list(BODY_XFORM_DEFAULT),
        "BX_raw_fit": BX_current.tolist() if BX_current is not None else None,
        "BX_raw_det": BX_raw_det,
        "BX_normalised_sign": bx_normalised,
        "BX_safe_to_swap": safe_to_swap,
    }


def phase_axes_probe(cf, pnp: PnPCollector, flow_stats: dict,
                     centre_xyz: tuple[float, float, float],
                     vs_list: tuple[float, ...],
                     impulse_s: float, settle_s: float,
                     vpe_rate_hz: float,
                     crash_state: dict) -> dict:
    """F2b — IMPULSE-BASED axes probe (operator spec 2026-05-18).

    Replaces step position_setpoints with body-velocity impulses.  For
    each vs in vs_list (progressive amplitude ramp), runs 4 legs:
    (+vs, 0), (-vs, 0), (0, +vs), (0, -vs).  Each leg = impulse_s of
    body-velocity drive + settle_s hover_setpoint(0).

    Per leg: collect (ekf_delta_body, flow_int_body, n_pp).
    Global fit: 2x2 BX with rank-check (skip swap if degenerate).

    Returns dict with per-leg measurements and the BX candidates.
    """
    global _VPE_RATE_HZ
    _VPE_RATE_HZ = vpe_rate_hz
    _, _, cz = centre_xyz
    print(f"[F2b] impulse probe vs={vs_list} m/s, impulse={impulse_s}s, "
          f"settle={settle_s}s, z_hold={cz}m", file=sys.stderr)
    legs = []
    for vs in vs_list:
        for label_suffix, (vx, vy) in [("+x", (+vs, 0.0)), ("-x", (-vs, 0.0)),
                                          ("+y", (0.0, +vs)), ("-y", (0.0, -vs))]:
            label = f"vs{vs:.2f}_{label_suffix}"
            leg = _run_impulse_leg(cf, pnp, flow_stats, vx, vy, cz,
                                     impulse_s, settle_s, label, crash_state)
            legs.append(leg)
    # ── Fit BODY_XFORM 2x2: ekf_body = BX @ flow_int  (rank-check) ─
    import numpy as np
    F = np.array([[l["flow_int_dx_grid"], l["flow_int_dy_grid"]] for l in legs])
    E = np.array([[l["ekf_dx_body"],     l["ekf_dy_body"]]      for l in legs])
    BX_fit = None
    BX_det = None
    bx_normalised = None
    safe_to_swap = False
    try:
        BX_T, *_ = np.linalg.lstsq(F, E, rcond=None)
        BX_fit = BX_T.T   # (2,2)
        BX_det = float(np.linalg.det(BX_fit))
        # Sign-snap rows to nearest of {(±1,0), (0,±1)}.
        rows = []
        for r in BX_fit:
            n = math.hypot(r[0], r[1])
            if n < 1e-9:
                rows.append((0.0, 0.0)); continue
            r0 = r[0] / n; r1 = r[1] / n
            if abs(r0) > abs(r1):
                rows.append((1.0 if r0 > 0 else -1.0, 0.0))
            else:
                rows.append((0.0, 1.0 if r1 > 0 else -1.0))
        bx_normalised = (rows[0][0], rows[0][1], rows[1][0], rows[1][1])
        # Rank check on the SIGN-SNAPPED matrix.
        snap = np.array([[bx_normalised[0], bx_normalised[1]],
                          [bx_normalised[2], bx_normalised[3]]])
        snap_det = float(np.linalg.det(snap))
        safe_to_swap = abs(snap_det) > 0.5      # for sign-permutation, det is ±1
        if not safe_to_swap:
            print(f"[F2b] degenerate sign-snap (det={snap_det:.3f}) — "
                  f"NOT swapping BODY_XFORM", file=sys.stderr)
    except Exception as e:
        print(f"[F2b] LSQ failed: {e}", file=sys.stderr)
    summary = {
        "name": "F2b_axes_probe",
        "vpe_rate_hz": vpe_rate_hz,
        "vs_list": list(vs_list),
        "impulse_s": impulse_s, "settle_s": settle_s,
        "legs": legs,
        "BX_raw_fit": BX_fit.tolist() if BX_fit is not None else None,
        "BX_raw_det": BX_det,
        "BX_normalised_sign": bx_normalised,
        "BX_safe_to_swap": safe_to_swap,
        "BX_default": list(BODY_XFORM_DEFAULT),
    }
    print(f"[F2b] BX_raw_fit={summary['BX_raw_fit']}  det={BX_det}",
          file=sys.stderr)
    print(f"[F2b] BX_norm={bx_normalised}  safe_to_swap={safe_to_swap}  "
          f"(default {BODY_XFORM_DEFAULT})", file=sys.stderr)
    return summary


def phase_measurement_hover(cf, pnp: PnPCollector, flow_stats: dict,
                            setpoint_xyz: tuple[float, float, float],
                            duration_s: float, vpe_rate_hz: float,
                            crash_state: dict) -> dict:
    """F3 — measure hover via hover_setpoint(0,0,0,z) (velocity damping only).

    NO position_setpoint here — that closes the loop on EKF position,
    which we know diverges post-cheat ([[cf2-sitl-cheat-odom-gt]]).
    hover_setpoint(0,0,0,z) tells cf2: "hold this altitude, zero body
    velocity".  Drone drifts only as far as flow fails to damp; we
    measure that drift against GT.  This is the honest test of the
    flow stabilisation loop in isolation.
    """
    global _VPE_RATE_HZ
    _VPE_RATE_HZ = vpe_rate_hz
    sx, sy, sz = setpoint_xyz   # sx/sy ignored for hover_setpoint (always 0)
    t0 = time.monotonic()
    last_seq = 0
    samples = []
    last_sp_t = 0.0
    print(f"[F3] hover_setpoint(0,0,0,z={sz:.2f}) for {duration_s}s, "
          f"VPE@{vpe_rate_hz}Hz  (no position drive)", file=sys.stderr)
    while time.monotonic() - t0 < duration_s:
        now = time.monotonic()
        if (now - last_sp_t) >= 0.05:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, sz)
            last_sp_t = now
        time.sleep(1.0 / SAMPLE_HZ)
        r = pnp.poll()
        if r is None:
            ekf_x, ekf_y, ekf_z, _, _, _ = get_ekf()
            _check_crash(crash_state, ekf_z, "F3_measure")
            continue
        pose, dets, fseq = r
        if fseq <= last_seq:
            continue
        last_seq = fseq
        ekf_x, ekf_y, ekf_z, _, _, ekf_yaw_deg = get_ekf()
        _check_crash(crash_state, ekf_z, "F3_measure")
        if pose is not None:
            px, py, pz = pose
            vpe_send_if_due(cf, px, py, pz, flow_stats)
        else:
            px = py = pz = None
        # dist relative to where hover STARTED, not (0,0).
        dist = math.sqrt((ekf_x-sx)**2 + (ekf_y-sy)**2 + (ekf_z-sz)**2)
        samples.append({
            "fseq": fseq,
            "t_wall": time.monotonic(),
            "n_det": len(dets),
            "ids": sorted(dets.keys()),
            "ekf": [round(ekf_x, 3), round(ekf_y, 3), round(ekf_z, 3)],
            "pnp": [None if px is None else round(px, 3),
                    None if py is None else round(py, 3),
                    None if pz is None else round(pz, 3)],
            "z_err_cm": round((pz - ekf_z) * 100, 2) if pz is not None else None,
            "dist_target_m": round(dist, 3),
            "flow_n": flow_stats["n_sent"],
        })
        print(f"[F3] fseq={fseq:>4}  n={len(dets)}  "
              f"ekf=({ekf_x:+.2f},{ekf_y:+.2f},{ekf_z:.2f})  "
              f"pnp=({'na' if px is None else f'{px:+.2f}'},"
              f"{'na' if py is None else f'{py:+.2f}'},"
              f"{'na' if pz is None else f'{pz:.2f}'})  "
              f"dist={dist:.2f}", file=sys.stderr)
    return {"name": "F3_measure", "t0_wall": t0, "duration_s": duration_s,
            "setpoint": list(setpoint_xyz), "vpe_rate_hz": vpe_rate_hz,
            "samples": samples}


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

    # Optional inspect-dir for visual operator review.  Each PnP-polled
    # frame gets hardlinked with name "t<ms>_n<dets>_f<fseq>.ppm".
    global _inspect_dir, _mission_t0_wall
    inspect_env = os.environ.get("SENTAI_INSPECT_DIR")
    if inspect_env:
        _inspect_dir = Path(inspect_env)
        _inspect_dir.mkdir(parents=True, exist_ok=True)
        print(f"[main] inspect-dir: {_inspect_dir}", file=sys.stderr)
    _mission_t0_wall = time.monotonic()

    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf

    cf.param.set_value("stabilizer.estimator", 2)
    time.sleep(0.5)
    # Gentle defaults from aruco_hover (post-cheat).
    for k, v in {
        "posCtlPid.xyKd":  0.5,
        "velCtlPid.vxKd":  0.05,
        "velCtlPid.vyKd":  0.05,
        "posCtlPid.xVelMax": 0.5,
        "posCtlPid.yVelMax": 0.5,
        "posCtlPid.xKp":   1.0,
        "posCtlPid.yKp":   1.0,
    }.items():
        try:
            cf.param.set_value(k, v)
        except Exception as e:
            print(f"[main] WARN {k} not in TOC: {e}", file=sys.stderr)
    time.sleep(0.3)
    cf.param.set_value("kalman.resetEstimation", 1); time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 0); time.sleep(2.0)

    lc = LogConfig(name="att", period_in_ms=20)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc); lc.data_received_cb.add_callback(_att_cb); lc.start()

    body_xform_ref = {"bx": BODY_XFORM_DEFAULT}
    flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
    crash_state = {"low_since": None, "aborted": False, "reason": None}
    # SafetyMonitor — iter #4 ([[op-s8-w1-mission-safety-triggers]]).
    # z_min_safe=0.20m: hard floor.  Marker loss 30 frames (~5s at 6.6Hz
    # frame dump).  Ceiling 3 × Z_HIGH catches EKF post-crash blowup.
    global _safety
    _safety = SafetyMonitor(
        z_min_safe=0.20, dwell_s=0.5,
        marker_loss_max_s=1.0,   # iter #6: time-based (op spec ~1s)
        z_ceiling=3.0 * Z_HIGH)
    print(f"[main] SafetyMonitor armed: z_floor={_safety.z_min_safe}m "
          f"(dwell={_safety.dwell_s}s), marker_loss_max={_safety.marker_loss_max_s}s, "
          f"z_ceiling={_safety.z_ceiling:.2f}m", file=sys.stderr)
    stop_evt = threading.Event()
    flow_th = threading.Thread(target=flow_forwarder,
                                args=(stop_evt, cf, flow_stats, body_xform_ref),
                                daemon=True)
    flow_th.start()
    time.sleep(1.0)

    pnp = PnPCollector(frames_dir)
    pnp.start_background()    # iter #6: bg thread at full camera FPS
    mc = MotionCommander(sync, default_height=Z_LOW)

    phases = []
    overall_t0 = time.monotonic()
    try:
        # ── F1: takeoff to Z_LOW ────────────────────────────────────
        print(f"[F1] takeoff to {Z_LOW}m @ {TAKEOFF_VEL_MPS} m/s", file=sys.stderr)
        mc.take_off(height=Z_LOW, velocity=TAKEOFF_VEL_MPS)
        time.sleep(F1_SETTLE_S)
        phases.append({"name": "F1_takeoff", "duration_s": F1_SETTLE_S,
                        "z_target_m": Z_LOW})

        # ── F2a: PASSIVE hover observation (no position drive) ─────
        f2a = phase_static_pin(cf, pnp, flow_stats,
                                (0.0, 0.0, Z_LOW), F2A_S, 20.0,
                                "F2a_static_pin", crash_state)
        phases.append(f2a)
        # Soft Z re-seed via extpos (PnP-z averaged baseline).
        if f2a["pnp_z_mean"] is not None and f2a["n_captures"] >= 5:
            try:
                cf.extpos.send_extpos(0.0, 0.0, float(f2a["pnp_z_mean"]))
                print(f"[F2a] re-seed extpos z={f2a['pnp_z_mean']:.3f}m",
                      file=sys.stderr)
            except Exception as e:
                print(f"[F2a] reseed extpos FAILED: {e}", file=sys.stderr)
        else:
            print(f"[F2a] skipping z re-seed — only {f2a['n_captures']} captures "
                  f"(need >=5)", file=sys.stderr)

        # ── F2-iter: PnP-in-loop closed-loop BX calibration ────────
        f2b = phase_calibrate_pnp_iterative(
            cf, pnp, flow_stats, body_xform_ref,
            z_hold=Z_LOW, vpe_rate_hz=20.0, crash_state=crash_state)
        phases.append(f2b)
        # Swap body_xform only if rank-check on RAW det passed (iter #3 fix).
        if f2b.get("BX_safe_to_swap") and f2b.get("BX_normalised_sign"):
            body_xform_ref["bx"] = f2b["BX_normalised_sign"]
            print(f"[F2-iter] BODY_XFORM swapped → {f2b['BX_normalised_sign']}",
                  file=sys.stderr)

        # ── F3: climb to Z_HIGH via hover_setpoint z-ramp ─────────
        print(f"[F3] climbing {Z_LOW}m → {Z_HIGH}m via hover_setpoint z-ramp",
              file=sys.stderr)
        f3_climb_t0 = time.monotonic()
        while time.monotonic() - f3_climb_t0 < F3_CLIMB_S:
            frac = (time.monotonic() - f3_climb_t0) / F3_CLIMB_S
            z_target = Z_LOW + frac * (Z_HIGH - Z_LOW)
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
            _, _, ekf_z, _, _, _ = get_ekf()
            _check_crash(crash_state, ekf_z, "F3_climb")
            time.sleep(0.05)
        # ── F3 measure (pure velocity-damping hover, no position drive) ─
        f3 = phase_measurement_hover(cf, pnp, flow_stats,
                                      (0.0, 0.0, Z_HIGH),
                                      F3_HOVER_S, 5.0, crash_state)
        phases.append(f3)

        # ── F4: landing ─────────────────────────────────────────────
        print(f"[F4] landing", file=sys.stderr)
        mc.land(velocity=LANDING_VEL_MPS)
        time.sleep(2.0)
        phases.append({"name": "F4_land", "duration_s": 2.0})
    except CrashAbort as e:
        print(f"[MAIN] CRASH ABORT — {e}", file=sys.stderr)
        crash_state["reason"] = str(e)
        try:
            mc.land(velocity=LANDING_VEL_MPS)
            time.sleep(2.0)
        except Exception as land_e:
            print(f"[MAIN] emergency land FAILED: {land_e}", file=sys.stderr)
        phases.append({"name": "ABORT", "reason_kind": "crash", "reason": str(e)})
    except SafetyAbort as e:
        print(f"[MAIN] SAFETY ABORT — {e}", file=sys.stderr)
        try:
            mc.land(velocity=LANDING_VEL_MPS)
            time.sleep(2.0)
        except Exception as land_e:
            print(f"[MAIN] emergency land FAILED: {land_e}", file=sys.stderr)
        phases.append({"name": "ABORT", "reason_kind": "safety", "reason": str(e)})
    finally:
        stop_evt.set()
        flow_th.join(timeout=1.0)
        pnp.stop_background()   # iter #6
        lc.stop()
        try:
            sync.close_link()
        except Exception:
            pass

    # ── Dump phases.json (sentai_sim-style structured journal) ──────
    with _lock:
        ekf_trace_snap = list(_ekf_trace)
    PHASES_PATH.write_text(json.dumps({
        "overall_t0_wall": overall_t0,
        "body_xform_default": list(BODY_XFORM_DEFAULT),
        "body_xform_final": list(body_xform_ref["bx"]),
        "phases": phases,
        "flow_n_sent": flow_stats["n_sent"],
        "vpe_n_sent": flow_stats.get("vpe_n_sent", 0),
        "ekf_trace_n": len(ekf_trace_snap),
        "crash_state": crash_state,
        "safety_state": _safety.to_dict() if _safety is not None else None,
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }, indent=2, default=str))
    print(f"[main] phases → {PHASES_PATH}", file=sys.stderr)

    # ── Also mirror F3 into s091/hover_log.json shape so s166 verdict
    #    plots the F3 portion directly with no extra glue. ───────────
    f3_samples = next((p for p in phases if p.get("name") == "F3_measure"), None)
    if f3_samples is not None:
        samples = f3_samples["samples"]
        if samples:
            dists = [s["dist_target_m"] for s in samples]
            n_det4 = sum(1 for s in samples if s["n_det"] == 4)
            n_det_any = sum(1 for s in samples if s["n_det"] >= 1)
            z_errs = [s["z_err_cm"] for s in samples if s["z_err_cm"] is not None]
            HOVER_LOG_PATH.write_text(json.dumps({
                "target_z_m": Z_HIGH,
                "hover_s": F3_HOVER_S,
                "hover_t0_wall": f3_samples["t0_wall"],
                "ekf_trace": ekf_trace_snap,
                "n_samples": len(samples),
                "all4_rate": n_det4 / len(samples),
                "any_rate": n_det_any / len(samples),
                "dist_mean_m": statistics.mean(dists),
                "dist_max_m":  max(dists),
                "z_mean_cm":   statistics.mean(z_errs) if z_errs else None,
                "z_rms_cm":    math.sqrt(sum(e*e for e in z_errs)/len(z_errs)) if z_errs else None,
                "flow_n":  flow_stats["n_sent"],
                "flow_hz": flow_stats["n_sent"] / max(0.1, F3_HOVER_S),
                "samples": samples,
                "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "_source": "s167_flowbaseline_calibrated",
            }, indent=2))
            print(f"[main] F3-only mirror → {HOVER_LOG_PATH}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
