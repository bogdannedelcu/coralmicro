"""s128 L4.1Baseline mission — pre-seeded marker, single-waypoint tour.

Hybrid host-side runner: spawns sentai_sim with bidirectional pipes and
drives its REPL while cflib (MotionCommander) flies cf2 SITL to the
same coordinate in lockstep.  See README.md.

SIMPLIFIED (2026-05-14, operator request): tour reduced to ONE marker
to isolate the integration point.  Multi-marker tour will follow once
single-marker is reliably green.

Outputs (under /tmp/s128_l41baseline/):
    mission.log         — step trace
    repl.transcript     — every line sent/received over REPL
    cf2_telemetry.json  — stateEstimate samples
    servo_*.json        — servo.status() / .trace() / objects.list() dumps
    summary.json        — verdict.py reads this
"""
from __future__ import annotations

import ast
import contextlib
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
sys.path.insert(0, str(S091_DIR))
import aruco_hover  # noqa: E402  flow_forwarder + scaling constants

WORKDIR = Path("/tmp/s128_l41baseline")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
TRACE_JSON      = WORKDIR / "servo_trace.json"
STATUS_JSON     = WORKDIR / "servo_status.json"
OBJECTS_JSON    = WORKDIR / "objects_list.json"
SUMMARY_JSON    = WORKDIR / "summary.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

# ─── ONE marker — east of takeoff origin at z=1 m ───────────────────
MARKERS = [
    # (class_id, x_m, y_m, z_m, label)
    (10, +0.50, +0.00, 1.00, "M0_east"),
]
TAKEOFF_Z_M       = 1.00
HOVER_AT_MARKER_S = 3.0      # plenty of dwell so cf2 settles
TAKEOFF_VEL_MPS   = 0.6
MOVE_VEL_MPS      = 0.3      # gentle move for first integration test
WAYPOINT_NEAR_M   = 0.30

# REPL-side journal (sentai.sim.journal_*) — structured append-mode log
# the host parses post-mortem.  Each action writes a line:
#     <t_ms> <label> <repr(servo.status())>
# so a crash mid-flight leaves WHERE+WHEN+WHAT-STATE on disk.  Lives
# under SENTAI_SIM_ROOT (build-sim/sentai_fs_root/).
JOURNAL_NAME      = "s128_journal.txt"
JOURNAL_HOST_PATH = lambda: SENTAI_FS_ROOT / JOURNAL_NAME    # noqa: E731


# ─────────────────────────────────────────────────────────────────
# Mission log
# ─────────────────────────────────────────────────────────────────
class StepLog:
    def __init__(self, path: Path):
        self._fh  = open(path, "w", buffering=1)
        self._n   = 0
        self._fh.write(f"# s128 mission log — {dt.datetime.now().isoformat()}\n")

    def info(self, msg: str) -> None:
        line = f"[{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}\n"
        self._fh.write(line)
        sys.stderr.write(line)

    @contextlib.contextmanager
    def step(self, what: str):
        self._n += 1
        n = self._n
        t0 = time.monotonic()
        self.info(f"step {n:>2}: BEGIN  {what}")
        try:
            yield n
        except BaseException as e:
            t = time.monotonic() - t0
            self.info(f"step {n:>2}: FAIL   {what}  ({t*1000:.0f} ms)  "
                      f"exc={type(e).__name__}: {e}")
            raise
        else:
            t = time.monotonic() - t0
            self.info(f"step {n:>2}: OK     {what}  ({t*1000:.0f} ms)")

    def close(self) -> None:
        self._fh.close()


# ─────────────────────────────────────────────────────────────────
# REPL driver — robust to async stdout noise from camera_bridge etc.
# ─────────────────────────────────────────────────────────────────
class ReplDriver:
    """Spawns sentai_sim with stdin/stdout pipes.

    `exec(cmd)` returns the captured response (raw text, may contain
    async noise from threads inside sentai_sim).  Callers should NOT
    parse raw responses — use `exec_value` / `exec_int` / `exec_repr`,
    which wrap the expression with a unique sentinel and extract just
    the value.  That makes the parser immune to async chatter."""

    def __init__(self, bin_path: Path, fs_root: Path, transcript: Path,
                 startup_timeout_s: float = 8.0):
        self.transcript = open(transcript, "w", buffering=1)
        env = os.environ.copy()
        env["SENTAI_SIM_ROOT"] = str(fs_root)
        fdir = WORKDIR / f"sentai_frames_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        fdir.mkdir(parents=True, exist_ok=True)
        env["SENTAI_DUMP_FRAMES_DIR"]   = str(fdir)
        env["SENTAI_DUMP_FRAMES_EVERY"] = "15"
        env["SENTAI_DUMP_RAW_EVERY"]    = "15"

        self.proc = subprocess.Popen(
            [str(bin_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env, bufsize=0,
        )
        self._buf = b""
        self._wait_prompt(startup_timeout_s)

    def _drain(self, deadline: float) -> bytes:
        while time.monotonic() < deadline:
            chunk = self.proc.stdout.read(1)
            if not chunk:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"sentai_sim exited rc={self.proc.returncode}")
                time.sleep(0.005)
                continue
            self._buf += chunk
            self.transcript.write(chunk.decode(errors="replace"))
            if self._buf.endswith(b">>> "):
                resp = self._buf[:-len(b">>> ")]
                self._buf = b""
                return resp
        raise TimeoutError("REPL prompt timeout")

    def _wait_prompt(self, timeout_s: float) -> bytes:
        return self._drain(time.monotonic() + timeout_s)

    def exec(self, cmd: str, timeout_s: float = 5.0) -> str:
        """Send a single-line statement; return raw response text.
        Does NOT raise on REPL-side exceptions — callers should wrap
        every value-returning call with `exec_value`/`exec_int` which
        relies on a sentinel to confirm success."""
        if "\n" in cmd:
            raise ValueError("ReplDriver.exec: single-line commands only")
        self.transcript.write(f">>> {cmd}\n")
        self.proc.stdin.write((cmd + "\n").encode())
        self.proc.stdin.flush()
        resp = self._drain(time.monotonic() + timeout_s)
        text = resp.decode(errors="replace")
        lines = text.splitlines()
        # strip the echoed command line
        if lines and lines[0].strip() == cmd.strip():
            lines = lines[1:]
        return "\n".join(lines).strip()

    def exec_value(self, expression: str, timeout_s: float = 5.0) -> str:
        """Eval `expression`, return its printed repr as a string.
        Async stdout noise (camera_bridge debug, etc.) is filtered out
        via a unique sentinel marker.  Raises if the sentinel is missing
        (=> the REPL raised, see transcript for details)."""
        marker = f"!RV{int(time.monotonic_ns()) & 0xFFFFFFF}!"
        body = self.exec(f"print('{marker}', {expression})", timeout_s)
        for line in body.splitlines():
            s = line.strip()
            if s.startswith(marker):
                return s[len(marker):].strip()
        # No sentinel → REPL likely raised.  Show transcript tail context.
        raise RuntimeError(f"no sentinel for `{expression}`; body=\n{body}")

    def exec_int(self, expression: str, **kw) -> int:
        s = self.exec_value(expression, **kw)
        try:
            return int(s)
        except ValueError as e:
            raise RuntimeError(f"exec_int: not an int `{s}`: {e}")

    def exec_repr(self, expression: str, **kw):
        """Eval expression, parse its repr with ast.literal_eval.
        For SHORT values only (single line of stdout).  Big structures
        like servo.trace() should go through `exec_via_fs` instead so
        the value travels via a file on disk rather than REPL stdout
        (where async camera_bridge debug spam can intermix and split
        the repr across lines)."""
        s = self.exec_value(expression, **kw)
        try:
            return ast.literal_eval(s)
        except (ValueError, SyntaxError) as e:
            raise RuntimeError(f"exec_repr: bad repr `{s[:120]}…`: {e}")

    def exec_via_fs(self, expression: str, sim_fs_root: Path,
                    timeout_s: float = 5.0):
        """Robust value retrieval: REPL writes `repr(<expr>)` to a file
        under SENTAI_SIM_ROOT, host reads the file directly + parses.

        Works for ANY value size — bypasses REPL stdout entirely so it
        is immune to async chatter from camera_bridge / flow threads.
        """
        # Pick a unique file path so concurrent calls don't collide.
        name = f"_s128_dump_{int(time.monotonic_ns()) & 0xFFFFFFF}.txt"
        # sentai.fs.write returns True/False — use exec_value (not
        # exec_int — `int('True')` raises) and check for the expected
        # truthy string.
        ok_str = self.exec_value(
            f"sentai.fs.write('{name}', repr({expression}))",
            timeout_s=timeout_s,
        )
        if ok_str != "True":
            raise RuntimeError(
                f"sentai.fs.write returned {ok_str!r} for `{expression}`")
        fpath = sim_fs_root / name
        if not fpath.is_file():
            raise RuntimeError(f"exec_via_fs: file missing {fpath}")
        body = fpath.read_text()
        try:
            value = ast.literal_eval(body)
        except (ValueError, SyntaxError) as e:
            raise RuntimeError(f"exec_via_fs: bad repr in {fpath}: {e}\nbody={body[:200]}")
        # Best-effort cleanup; ignore failure (race with concurrent flush).
        with contextlib.suppress(Exception):
            fpath.unlink()
        return value

    def close(self) -> None:
        try:
            self.proc.stdin.write(b"\x04")
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.transcript.close()


# ─────────────────────────────────────────────────────────────────
# cf2 telemetry capture
# ─────────────────────────────────────────────────────────────────
_tel_lock = threading.Lock()
_tel_last = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
_tel_samples: list[dict] = []


def _tel_cb(_ts, data, _lc):
    with _tel_lock:
        _tel_last["x"] = data["stateEstimate.x"]
        _tel_last["y"] = data["stateEstimate.y"]
        _tel_last["z"] = data["stateEstimate.z"]
        _tel_last["yaw_deg"] = data["stateEstimate.yaw"]
        _tel_samples.append({
            "t": time.monotonic(),
            "x": _tel_last["x"],
            "y": _tel_last["y"],
            "z": _tel_last["z"],
            "yaw_deg": _tel_last["yaw_deg"],
        })


def tel_snapshot() -> dict:
    with _tel_lock:
        return dict(_tel_last)


# ─────────────────────────────────────────────────────────────────
# Mission
# ─────────────────────────────────────────────────────────────────
def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {"waypoints": []}

    with log.step("cf2 link + Kalman reset + damp params"):
        sync = SyncCrazyflie("udp://127.0.0.1:19850",
                             cf=Crazyflie(rw_cache=None))
        sync.open_link()
        cf = sync.cf
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.5)
        for k, v in {
            "posCtlPid.xyKd":     0.5,
            "velCtlPid.vxKd":     0.05,
            "velCtlPid.vyKd":     0.05,
            "posCtlPid.xVelMax":  2.5,
            "posCtlPid.yVelMax":  2.5,
            "posCtlPid.xKp":      3.0,
            "posCtlPid.yKp":      3.0,
        }.items():
            try:
                cf.param.set_value(k, v)
            except Exception:
                pass
        time.sleep(0.3)
        cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.0)

    with log.step("telemetry log @ 20 ms"):
        lc = LogConfig(name="att", period_in_ms=20)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                  "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
            lc.add_variable(v, "float")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(_tel_cb)
        lc.start()

    with log.step("flow forwarder thread (give it 2s to connect bridge)"):
        stop_evt = threading.Event()
        flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
        flow_th = threading.Thread(target=aruco_hover.flow_forwarder,
                                    args=(stop_evt, cf, flow_stats),
                                    daemon=True)
        flow_th.start()
        time.sleep(2.0)
        if flow_stats["fatal"]:
            raise RuntimeError(f"flow forwarder fatal: {flow_stats['fatal']}")
        log.info(f"        flow forwarder running (n_sent={flow_stats['n_sent']})")

    # ─── Per-action journal helper (sentai.sim.journal_*) ───
    # Every servo.* call writes one structured line to the journal:
    #     <t_ms> <label> <repr(servo.status())>
    # Open file is owned by the REPL.  If the mission crashes mid-flight
    # the journal on disk shows the LAST completed action + state at
    # that moment (per operator's 2026-05-14 request).
    def j_int(label: str, cmd: str) -> int:
        rc = repl.exec_int(cmd)
        # journal_write returns 0 ok / -1 not open.  We don't care about
        # the journal call's own rc — the test's rc is what counts.
        repl.exec_int(f"sentai.sim.journal_write('{label}', "
                       f"sentai.servo.status())")
        return rc

    with log.step("REPL: import + verbose(0) + sentai.sim.journal_open"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")        # silence [sim]/[LED]/[camera] noise
        rc = repl.exec_int(
            f"sentai.sim.journal_open('{JOURNAL_NAME}')"
        )
        if rc != 0:
            raise RuntimeError(f"journal_open returned {rc}")
        # Header annotation so verdict.py can tell mission start from
        # leftover journal lines (truncate=True default already wipes).
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")

    with log.step("REPL: clear + init + clear_trace"):
        n = repl.exec_int("sentai.objects.clear()")
        log.info(f"        cleared {n} prior objects")
        rc = j_int("servo_init", "sentai.servo.init('sim')")
        if rc != 0:
            raise RuntimeError(f"servo.init rc={rc}")
        cleared_trace = repl.exec_int("sentai.servo.clear_trace()")
        log.info(f"        cleared {cleared_trace} prior trace entries")

    with log.step(f"REPL: seed {len(MARKERS)} marker(s) via objects.add"):
        ids = []
        for cid, x, y, z, label in MARKERS:
            rid = repl.exec_int(f"sentai.objects.add({cid}, {x}, {y}, {z})")
            if rid <= 0:
                raise RuntimeError(f"objects.add({label}) returned {rid}")
            ids.append((rid, label, (x, y, z)))
            log.info(f"        seeded {label} cid={cid} → id={rid}")
        summary["seeded_ids"] = [{"id": r, "label": l, "xyz": list(c)}
                                 for r, l, c in ids]
        n = repl.exec_int("sentai.objects.count()")
        if n != len(MARKERS):
            raise RuntimeError(f"objects.count()={n}, expected {len(MARKERS)}")

    with log.step(f"REPL: servo.arm + servo.takeoff({TAKEOFF_Z_M})"):
        rc = j_int("servo_arm", "sentai.servo.arm()")
        if rc != 0:
            raise RuntimeError(f"servo.arm rc={rc}")
        rc = j_int("servo_takeoff", f"sentai.servo.takeoff({TAKEOFF_Z_M})")
        if rc != 0:
            raise RuntimeError(f"servo.takeoff rc={rc}")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m @ {TAKEOFF_VEL_MPS} m/s"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(2.5)
        # Explicit hover setpoint after takeoff — without it cf2 drifts.
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.2f},{tel['y']:+.2f},"
                 f"{tel['z']:.2f})  flow_n={flow_stats['n_sent']}")

    # ─── Single-waypoint tour ───
    cur_xyz = [0.0, 0.0, TAKEOFF_Z_M]
    for (cid, x, y, z, label) in MARKERS:
        dx, dy, dz = x - cur_xyz[0], y - cur_xyz[1], z - cur_xyz[2]
        with log.step(f"REPL servo.move({dx:+.2f},{dy:+.2f},{dz:+.2f}) + "
                      f"cf2 move_distance"):
            rc = j_int(f"servo_move_{label}", f"sentai.servo.move({dx}, {dy}, {dz})")
            if rc != 0:
                raise RuntimeError(f"servo.move toward {label} rc={rc}")
            mc.move_distance(dx, dy, dz, velocity=MOVE_VEL_MPS)
            cur_xyz = [x, y, z]
        with log.step(f"hover {HOVER_AT_MARKER_S} s over {label}"):
            mc.start_linear_motion(0.0, 0.0, 0.0)
            rc = j_int(f"servo_hover_{label}", "sentai.servo.hover()")
            if rc != 0:
                raise RuntimeError(f"servo.hover at {label} rc={rc}")
            time.sleep(HOVER_AT_MARKER_S)
            tel = tel_snapshot()
            dist = math.sqrt((tel["x"] - x) ** 2 + (tel["y"] - y) ** 2
                              + (tel["z"] - z) ** 2)
            log.info(f"        cf2 @ ({tel['x']:+.2f},{tel['y']:+.2f},"
                     f"{tel['z']:.2f})  dist={dist:.3f} m  "
                     f"flow_n={flow_stats['n_sent']}")
            summary["waypoints"].append({
                "label": label,
                "target": [x, y, z],
                "cf2": [tel["x"], tel["y"], tel["z"]],
                "dist_m": dist,
            })

    with log.step("REPL: servo.land + servo.disarm"):
        rc = j_int("servo_land", "sentai.servo.land()")
        if rc != 0:
            raise RuntimeError(f"servo.land rc={rc}")
        rc = j_int("servo_disarm", "sentai.servo.disarm()")
        if rc != 0:
            raise RuntimeError(f"servo.disarm rc={rc}")

    with log.step("cf2 land"):
        mc.land(velocity=0.3)
        time.sleep(2.0)

    with log.step("teardown telemetry + cf2 link"):
        stop_evt.set()
        flow_th.join(timeout=1.5)
        lc.stop()
        sync.close_link()
        summary["flow_n_sent"] = flow_stats["n_sent"]

    with log.step("REPL dump servo.status / trace / objects.list via fs.write"):
        # Annotate the journal first so a verdict can spot the dump
        # boundary even without timestamp math.
        repl.exec_int("sentai.sim.journal_write('mission_dump_begin', None)")
        status   = repl.exec_via_fs("sentai.servo.status()",  SENTAI_FS_ROOT)
        trace    = repl.exec_via_fs("sentai.servo.trace()",   SENTAI_FS_ROOT)
        objects  = repl.exec_via_fs("sentai.objects.list()",  SENTAI_FS_ROOT)
        STATUS_JSON.write_text(json.dumps(status, indent=2))
        TRACE_JSON.write_text(json.dumps(trace, indent=2))
        OBJECTS_JSON.write_text(json.dumps(objects, indent=2))
        summary["servo_status"]  = status
        summary["servo_trace"]   = trace
        summary["objects_list"]  = objects
        repl.exec_int("sentai.sim.journal_write('mission_end', None)")
        repl.exec_int("sentai.sim.journal_close()")

    # Surface the on-board journal in WORKDIR for easy post-mortem.
    journal = JOURNAL_HOST_PATH()
    if journal.is_file():
        (WORKDIR / "journal.txt").write_text(journal.read_text())
        log.info(f"        journal copied: {(WORKDIR / 'journal.txt')}")

    return summary


def main() -> int:
    log = StepLog(MISSION_LOG)
    log.info(f"REPO_ROOT={REPO_ROOT}")
    log.info(f"SENTAI_SIM_BIN={SENTAI_SIM_BIN}")
    if not SENTAI_SIM_BIN.is_file():
        log.info("FATAL — sentai_sim binary missing; run `cmake --build build-sim`")
        return 1

    repl = ReplDriver(SENTAI_SIM_BIN, SENTAI_FS_ROOT, REPL_TRANSCRIPT)
    try:
        summary = fly(log, repl)
    except BaseException as e:
        log.info(f"MISSION FAILED: {type(e).__name__}: {e}")
        # Best-effort: dump current state via the file-based path so we
        # don't depend on REPL stdout (which may be poisoned by the
        # exception that just happened).  Plus surface the on-board
        # journal so post-mortem shows last completed step.
        with contextlib.suppress(Exception):
            st = repl.exec_via_fs("sentai.servo.status()", SENTAI_FS_ROOT)
            STATUS_JSON.write_text(json.dumps(st, indent=2))
        with contextlib.suppress(Exception):
            tr = repl.exec_via_fs("sentai.servo.trace()", SENTAI_FS_ROOT)
            TRACE_JSON.write_text(json.dumps(tr, indent=2))
        # Even on crash, the journal file is the canonical artifact.
        # Try to close it cleanly so the footer line is written, then
        # copy to WORKDIR for the verdict to inspect.
        with contextlib.suppress(Exception):
            repl.exec_int("sentai.sim.journal_write('mission_crash', None)")
            repl.exec_int("sentai.sim.journal_close()")
        journal = JOURNAL_HOST_PATH()
        if journal.is_file():
            (WORKDIR / "journal.txt").write_text(journal.read_text())
            log.info(f"        journal saved: {(WORKDIR / 'journal.txt')}")
        SUMMARY_JSON.write_text(json.dumps({
            "_status": "FAIL",
            "_exception": f"{type(e).__name__}: {e}",
            "_last_run": dt.datetime.now().isoformat(),
        }, indent=2))
        repl.close()
        log.close()
        return 1

    with _tel_lock:
        TELEMETRY_JSON.write_text(json.dumps(_tel_samples))
    summary["_status"]   = "OK"
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2))
    log.info("MISSION OK")
    repl.close()
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
