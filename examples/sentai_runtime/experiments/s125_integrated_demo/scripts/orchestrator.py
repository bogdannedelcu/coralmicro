#!/usr/bin/env python3
"""s125 — integrated demo orchestrator.

Drives a CrazyFlie cf2 SITL drone through a takeoff / 4-waypoint square /
landing trajectory; in parallel feeds the drone's live pose into a
sentai_sim REPL session so sentai.places builds a hex-cell world model
and sentai.slam ingests synthetic class-0 detections.  Each newly-visited
H3 cell triggers a small cylinder marker spawned in Gazebo so the operator
can SEE the world model materializing on the floor.

Run from inside distrobox crazysim-garden:
    distrobox enter crazysim-garden -- bash examples/sentai_runtime/\\
        experiments/s125_integrated_demo/scripts/run_demo.sh
"""
from __future__ import annotations

import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
URI = "udp://127.0.0.1:19850"

# SFLVP exploration parameters (Stage 3.C — replaces fixed waypoints).
# Per objects_plan §7.5: visit a central cell + its 6 neighbors, then
# pick the next central greedily.  Drone flies at fixed altitude.
WORLD_NAME       = "s125_demo"
ALTITUDE_M       = 1.0
HOVER_TIME_S     = 1.8        # at each cell — short, just enough for EKF to settle
MAX_CELLS        = 13         # safety stop (1 central + 6 ring + 6 next-ring)
MOTION_VELOCITY  = 0.3        # m/s
SENTAI_SIM_BIN   = Path(__file__).resolve().parents[5] / "build-sim/sim/sentai_sim"

# Floor texture for skippable-cell detection — same Harmonic 4K we paint
# on the ground plane in world/s125_demo.sdf.  25 m physical plane,
# 4096 px texture, centred on origin: pixel (px, py) ↔ world (x, y) via
#   px = (x + PLANE_HALF) / PLANE * IMG_W
#   py = (PLANE_HALF - y) / PLANE * IMG_H   (image Y is inverted)
HARMONIC_TILE_PATH = Path(__file__).resolve().parents[5] / "sim/gazebo/worlds/assets/harmonic_tiles/harmonic_alt200_4k.png"
PLANE_HALF_M       = 12.5
PLANE_M            = 25.0
SKIP_VARIANCE_THR  = 5.0     # below this stddev → "uniform / water" → skip
FLOW_OUT_SOCK      = "/tmp/sentai_flow_out.sock"   # set by gz_to_uds_bridge
# Drone EKF flow scaling (DEFAULTS from gz_to_camera_bridge.py).
DRONE_NPIX            = 35.0
DRONE_THETAPIX_RAD    = 0.71674
FOV_H_RAD             = 1.0123          # 58° horizontal
FOV_V_RAD             = 0.7854          # 45° vertical
GRID_W                = 80
GRID_H                = 60
                              # Calibrated on Harmonic 4K: terrain ~25, lake ~1,
                              # marginal/forest ~10.  threshold 5 keeps most of
                              # the visible scene visitable while flagging
                              # the lake patches.

# ---------------------------------------------------------------------------
# sentai_sim REPL pipe — newline-flushed; output is captured into a buffer
# we parse for "=KEY value" lines whenever we need a return value.
# ---------------------------------------------------------------------------

class SentaiSim:
    """Spawn sentai_sim and talk to it over stdin/stdout.

    The REPL prompt ends with ``>>> `` and no trailing newline, so a naive
    ``readline()``-based sync would block forever.  We use a non-blocking
    stdout (``os.set_blocking(fd, False)``) and a small ready-grace
    (``time.sleep``) after spawn — that's enough since the REPL is idle
    until we send a command.
    """

    def __init__(self, sim_bin: Path):
        if not sim_bin.exists():
            sys.exit(f"[s125] sentai_sim not found at {sim_bin}; run cmake --build build-sim")
        # Use BYTE mode (text=False) — non-blocking reads on Python's buffered
        # text stream raise TypeError when no data is available.  We do our
        # own decoding from utf-8 at drain time.
        self.proc = subprocess.Popen(
            [str(sim_bin)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, bufsize=0,
        )
        import os as _os
        _os.set_blocking(self.proc.stdout.fileno(), False)
        self._fd = self.proc.stdout.fileno()
        time.sleep(0.4)
        self._drain()

    def _drain(self) -> str:
        """Read everything currently in the stdout pipe (non-blocking).

        Uses os.read() at the raw fd level so non-blocking semantics are
        precise (EAGAIN → BlockingIOError) — no surprise TypeErrors from
        the buffered reader.
        """
        import os as _os
        chunks = []
        while True:
            try:
                buf = _os.read(self._fd, 65536)
            except BlockingIOError:
                break
            if not buf:
                break
            chunks.append(buf)
        return b"".join(chunks).decode("utf-8", errors="replace")

    def cmd(self, line: str):
        """Send a one-liner Python statement; no return value parsing."""
        if self.proc.poll() is not None:
            raise RuntimeError(f"sentai_sim exited (rc={self.proc.returncode})")
        self.proc.stdin.write((line + "\n").encode("utf-8"))
        self.proc.stdin.flush()

    def query_str(self, py_expr: str, key: str = "Q") -> str | None:
        """Send ``print('=KEY', <expr>)`` and return the matched text."""
        self.cmd(f"print('={key}',{py_expr})")
        prefix = f"={key} "
        # Up to 0.5 s of polling reads
        deadline = time.time() + 0.5
        accum = ""
        while time.time() < deadline:
            chunk = self._drain()
            if chunk:
                accum += chunk
                idx = accum.find(prefix)
                if idx >= 0:
                    rest = accum[idx + len(prefix):]
                    nl = rest.find("\n")
                    if nl >= 0:
                        return rest[:nl].strip()
            time.sleep(0.02)
        return None

    def close(self):
        try:
            self.cmd("exit")
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


# ---------------------------------------------------------------------------
# Gazebo H3-cell overlay — spawn small translucent cylinders via gz service
# ---------------------------------------------------------------------------

def flow_forwarder(stop_evt: threading.Event, cf) -> None:
    """Read flow snapshots from /tmp/sentai_flow_out.sock and forward
    each one to cf2 as CRTP_LOCALIZATION ch=1 packet (s091 pattern).
    Runs as a daemon thread sharing the orchestrator's cflib session."""
    from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

    # Same struct layout as s091/aruco_hover.py — 124-byte FRL1 record.
    REPLY_FMT  = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
    REPLY_SZ   = struct.calcsize(REPLY_FMT)
    REPLY_MAGIC = 0x46524C31  # 'FRL1'

    # Grid-px → drone-pixel scale.
    grid_per_rad_x = GRID_W / FOV_H_RAD
    grid_per_rad_y = GRID_H / FOV_V_RAD
    scale_x = DRONE_NPIX * DRONE_THETAPIX_RAD / grid_per_rad_x
    scale_y = DRONE_NPIX * DRONE_THETAPIX_RAD / grid_per_rad_y

    # Wait for the bridge to come up (up to 15 s).
    sock = None
    for _ in range(30):
        if stop_evt.is_set():
            return
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(FLOW_OUT_SOCK)
            sock = s
            break
        except Exception:
            time.sleep(0.5)
    if sock is None:
        print(f"[flow_fwd] FAIL connect {FLOW_OUT_SOCK} — flow disabled", flush=True)
        return
    print(f"[flow_fwd] connected {FLOW_OUT_SOCK}", flush=True)

    buf = b""
    n_sent = 0
    last_log = time.monotonic()
    last_send = time.monotonic()
    while not stop_evt.is_set():
        try:
            sock.settimeout(0.5)
            chunk = sock.recv(4096)
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            while len(buf) >= REPLY_SZ:
                rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
                fields = struct.unpack(REPLY_FMT, rec)
                if fields[0] != REPLY_MAGIC:
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                dx_q1000 = fields[2]; dy_q1000 = fields[3]; conf = fields[4]
                dpx = (dx_q1000 / 1000.0) * scale_x
                dpy = (dy_q1000 / 1000.0) * scale_y
                now = time.monotonic()
                dt  = max(0.001, min(0.2, now - last_send))
                last_send = now
                std = max(1.0, 8.0 - conf / 32.0)

                pk = CRTPPacket()
                pk.port    = CRTPPort.LOCALIZATION
                pk.channel = 1
                pk.data    = struct.pack("<fhhfHH",
                                          float(dt),
                                          int(round(dpx)),
                                          int(round(dpy)),
                                          float(std),
                                          int(min(conf, 0xFFFF)), 0)
                cf.send_packet(pk)
                n_sent += 1

                if now - last_log > 4.0:
                    print(f"[flow_fwd] sent={n_sent} conf={conf} dpx={dpx:+.1f} dpy={dpy:+.1f}",
                          flush=True)
                    last_log = now
        except socket.timeout:
            continue
        except Exception as e:
            if not stop_evt.is_set():
                print(f"[flow_fwd] err {e}", flush=True)
            time.sleep(0.1)
    try:
        sock.close()
    except Exception:
        pass


class TextureFeatureScorer:
    """Score whether a (x,y) world coordinate has "enough features" to be
    worth visiting.  Loads the Harmonic 4K floor tile once, samples a 16×16
    patch at each query, returns the pixel-intensity std-dev as a score.
    Std-dev < SKIP_VARIANCE_THR → flag as skippable (water / uniform).
    """
    def __init__(self, tile_path: Path):
        self.tile_arr = None
        self.h = self.w = 0
        try:
            from PIL import Image
            import numpy as np
            img = Image.open(tile_path).convert("L")
            self.tile_arr = np.asarray(img)
            self.h, self.w = self.tile_arr.shape
            self._np = np
            print(f"[scorer] loaded {tile_path.name} {self.w}x{self.h} for skippable detection")
        except Exception as e:
            print(f"[scorer] WARN: skippable detection disabled — {e}")

    def score(self, x_m: float, y_m: float) -> float:
        """Return std-dev around the (x,y) coordinate (0 if disabled)."""
        if self.tile_arr is None:
            return 999.0   # never skip if scorer is disabled
        px = int((x_m + PLANE_HALF_M) / PLANE_M * self.w)
        py = int((PLANE_HALF_M - y_m) / PLANE_M * self.h)
        # Clamp
        if px < 8 or px >= self.w - 8 or py < 8 or py >= self.h - 8:
            return 999.0   # off-tile → don't skip
        patch = self.tile_arr[py - 8:py + 8, px - 8:px + 8]
        return float(patch.std())


class CellOverlay:
    """Spawn a small cylinder marker at each newly-visited cell centroid.

    The world uses cf2 physical units (meters from origin).  We accept
    cf2 physical coordinates and spawn at the same (x, y) — but at the
    cell *centroid* (i.e., quantized to the H3 grid lattice as known from
    the orchestrator side).
    """
    # Physical drone scale: cf2 lives in ~3 m of physical space.  H3 cells
    # are ~0.35 m physical diameter at res=13/scale=10, so a 0.15 m radius
    # cylinder lands inside a single cell footprint without overlapping
    # neighbors visually.  Larger length=0.08 makes them noticeable from
    # the operator's bird's-eye GUI camera at z=8 m looking down.
    SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} 0.05 0 0 0</pose>
    <link name="link">
      <visual name="v">
        <geometry><cylinder><radius>0.15</radius><length>0.08</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def __init__(self, world_name: str):
        self.world = world_name
        self.spawned: set[int] = set()
        self.counter = 0

    def add_cell(self, cell_hex: str, x: float, y: float):
        cell_int = int(cell_hex, 16)
        if cell_int in self.spawned:
            return
        self.spawned.add(cell_int)
        # Cycle 4 colors for visual distinction
        palette = [(1.0, 0.3, 0.3), (0.3, 1.0, 0.3),
                   (0.3, 0.3, 1.0), (1.0, 0.8, 0.2)]
        r, g, b = palette[self.counter % 4]
        self.counter += 1
        name = f"hex_cell_{self.counter:03d}"
        # Write SDF to a temp file (the gz service call references it by path).
        tmpf = Path(f"/tmp/{name}.sdf")
        tmpf.write_text(self.SDF_TEMPLATE.format(name=name, x=x, y=y, r=r, g=g, b=b))
        req = (f'sdf_filename: "{tmpf}", '
               f'pose: {{position: {{x: {x}, y: {y}, z: 0.02}}}}, '
               f'name: "{name}", allow_renaming: 1')
        # IMPORTANT: gz service MUST run inside distrobox crazysim-garden
        # so it talks the right Garden 7 protocol (host gz is Harmonic 8
        # and silently fails — that's why hex cylinders were missing in
        # the operator's first watch, 2026-05-13).  /tmp is shared between
        # host and container so the SDF file path resolves on both sides.
        try:
            subprocess.run([
                "distrobox", "enter", "crazysim-garden", "--",
                "gz", "service", "-s", f"/world/{self.world}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "1000",
                "--req", req,
            ], check=False, capture_output=True, timeout=4.0)
        except subprocess.TimeoutExpired:
            print(f"[overlay] spawn timeout for {name}")


# ---------------------------------------------------------------------------
# cflib helper — high-level commander, log pose stream
# ---------------------------------------------------------------------------

def fly_mission(sim: SentaiSim, overlay: "CellOverlay",
                scorer: "TextureFeatureScorer"):
    cflib.crtp.init_drivers()
    print(f"[s125] connecting cflib → {URI}")
    cf = Crazyflie(rw_cache="./cache")
    flow_stop = threading.Event()
    flow_th = None
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Start flow forwarder thread BEFORE we activate sentai.flow inside
        # sentai_sim, so the bridge has someone reading its output socket
        # the moment frames start flowing.
        flow_th = threading.Thread(
            target=flow_forwarder, args=(flow_stop, scf.cf), daemon=True)
        flow_th.start()
        # Full sentai.flow pipeline is wired (run_demo.sh launches
        # gz_to_uds_bridge + this orchestrator runs flow_forwarder
        # thread).  cf2 was teleported to z=1.2 m by run_demo.sh step 6b
        # so the down-cam clears the ground plane and phase-corr can
        # produce real (dx, dy, conf) observations.
        # Kalman estimator (=2) is the proven s091 setup; reset + grace
        # let the EKF lock onto the incoming flow before takeoff.
        scf.cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.3)
        scf.cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        scf.cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.5)
        sim.cmd("sentai.flow.start(0)")
        time.sleep(0.3)

        # sentai_sim setup
        sim.cmd('sentai.places.init(40.689167, -74.044444, 10.0, 13)')
        sim.cmd('sentai.slam.init(70.0, 320, 240)')
        sim.cmd('sentai.slam.set_class_prior(0, 1.7)')
        sim.cmd('sentai.servo.init("cf2")')
        sim.cmd('sentai.servo.arm()')
        sim.cmd('sentai.explore.start("s125")')
        sim.cmd('sentai.explore.set_arm_ack(1)')
        sim.cmd('sentai.explore.set_marker(1)')

        # Pre-takeoff stabilization phase — 4 ArUco panels are visible on
        # the ground around the drone (world/s125_demo.sdf, ids 0..3).
        # The drone holds still while the EKF settles; in a future stage
        # this is where sentai.aruco would lock the home pose.
        print("[s125] pre-takeoff: visual stabilization on ArUco panels (3 s)…")
        for k in range(3):
            print(f"[s125]   stab {k+1}/3 — markers visible at (±0.4, ±0.4)")
            time.sleep(1.0)

        # Pose-logger reads cf2 EKF state in parallel with motion commands.
        lg = LogConfig(name="pose", period_in_ms=200)
        lg.add_variable("stateEstimate.x", "float")
        lg.add_variable("stateEstimate.y", "float")
        lg.add_variable("stateEstimate.z", "float")

        # MotionCommander handles takeoff/move/land via velocity-setpoint
        # CRTP packets — the same path CrazySim's own examples use.  Its
        # context manager auto-takeoffs to default_height on entry and
        # auto-lands on exit, so the structure of the mission is just
        # "move from one waypoint to the next".
        print("[s125] takeoff → 1.0 m via MotionCommander")
        with MotionCommander(scf, default_height=1.0) as mc:
            # MotionCommander.__enter__ already kicked takeoff.  Wait for
            # the climb to settle by reading altitude from the log stream.
            with SyncLogger(scf, lg) as logger:
                print("[s125] stabilizing over ArUco markers (waiting for z ≥ 0.95 m)…")
                t_stab_start = time.time()
                for entry in logger:
                    _ts, data, _logconf = entry
                    z = data["stateEstimate.z"]
                    sim.cmd(f'sentai.explore.set_alt({z:.3f})')
                    if z >= 0.95:
                        elapsed = time.time() - t_stab_start
                        print(f"[s125] hover @ z={z:.2f} m reached (in {elapsed:.1f} s) — markers stable, starting exploration")
                        break
                    if time.time() - t_stab_start > 10.0:
                        print(f"[s125] WARN: altitude never reached 0.95 m (last z={z:.2f}); proceeding anyway")
                        break

                # Brief hover-confirm pause so the operator sees the
                # drone stationary at 1 m over the marker grid.
                print("[s125] hold 1.5 s at 1.0 m (visual confirmation)…")
                time.sleep(1.5)

                # ============================================================
                # SFLVP exploration — "S From Last Visited Place"
                # (objects_plan.md §7.5).  Visit central cell, then its
                # 6 H3 neighbors, then pick the next central; repeat
                # until MAX_CELLS hit.  Cells flagged as skippable=1
                # (no features) are skipped during neighbor traversal.
                # ============================================================
                cur_x, cur_y = 0.0, 0.0
                seq = -1
                visited_cells = set()
                central_queue = []
                # Seed with the current cell as the first central.
                home_cell = sim.query_str(
                    f"hex(sentai.places.cell({cur_x:.3f},{cur_y:.3f}))",
                    key="SFLVP_SEED")
                if home_cell:
                    central_queue.append(home_cell)

                def fly_to_cell(cell_hex):
                    """Resolve cell → (x,y) via places.center, then
                    MotionCommander.move_distance from current pose."""
                    nonlocal cur_x, cur_y
                    coords = sim.query_str(
                        f"sentai.places.center({cell_hex})",
                        key=f"CTR{seq}")
                    if not coords or coords == "None":
                        return False
                    # coords looks like "(0.123, 0.456)"
                    try:
                        s = coords.strip("() ")
                        tx, ty = [float(v) for v in s.split(",")]
                    except Exception as e:
                        print(f"[sflvp] center parse fail: {coords!r} -> {e}")
                        return False
                    dx, dy = tx - cur_x, ty - cur_y
                    if abs(dx) < 0.02 and abs(dy) < 0.02:
                        return True   # already there
                    print(f"[sflvp]   fly_to cell {cell_hex} center=({tx:.2f},{ty:.2f}) Δ=({dx:.2f},{dy:.2f})")
                    mc.move_distance(dx, dy, 0.0, velocity=MOTION_VELOCITY)
                    cur_x, cur_y = tx, ty
                    return True

                while central_queue and len(visited_cells) < MAX_CELLS:
                    central = central_queue.pop(0)
                    if central in visited_cells:
                        continue
                    print(f"[sflvp] central cell {central}  (visited={len(visited_cells)}/{MAX_CELLS})")
                    # Fly to central (no-op for first iteration since we
                    # are already at its centroid)
                    if not fly_to_cell(central):
                        visited_cells.add(central)
                        continue
                    visited_cells.add(central)

                    # Visit the 6 neighbors
                    nbrs_raw = sim.query_str(
                        f"[hex(c) for c in sentai.places.neighbors({central},1)]",
                        key=f"NBR{len(visited_cells)}")
                    if not nbrs_raw:
                        continue
                    # nbrs_raw is like "['0xabc', '0xdef', ...]" — eval safely.
                    try:
                        # ast.literal_eval is safe for python literals.
                        import ast as _ast
                        nbr_list = _ast.literal_eval(nbrs_raw)
                    except Exception:
                        print(f"[sflvp] neighbors parse fail: {nbrs_raw!r}")
                        nbr_list = []
                    # Exclude the central itself; remaining 6 are the ring.
                    nbr_list = [c for c in nbr_list if c != central]

                    for nbr in nbr_list:
                        if nbr in visited_cells:
                            continue
                        # Pre-classify the neighbor: compute its center,
                        # sample the Harmonic floor texture there, decide
                        # whether to skip on low variance (uniform/water).
                        coords_str = sim.query_str(
                            f"sentai.places.center({nbr})",
                            key=f"NCTR{len(visited_cells)}")
                        if coords_str and coords_str != "None":
                            try:
                                s = coords_str.strip("() ")
                                nx, ny = [float(v) for v in s.split(",")]
                            except Exception:
                                nx = ny = 0.0
                            std = scorer.score(nx, ny)
                            if std < SKIP_VARIANCE_THR:
                                # Persist the classification on the cell
                                # so future SFLVP queries can short-circuit.
                                sim.cmd(
                                    f"sentai.places.set_skippable({nbr}, 1)")
                                print(f"[sflvp]   skip {nbr} center=({nx:.2f},{ny:.2f}) std={std:.1f} (no features)")
                                visited_cells.add(nbr)
                                continue
                        # Existing persisted flag also honored.
                        skip = sim.query_str(
                            f"sentai.places.is_skippable({nbr})",
                            key=f"SK{len(visited_cells)}")
                        if skip == "1":
                            print(f"[sflvp]   skip {nbr} (previously flagged)")
                            visited_cells.add(nbr)
                            continue
                        if not fly_to_cell(nbr):
                            continue
                        visited_cells.add(nbr)
                        # Brief settle + observe + FSM update at the new cell
                        t_hover_end = time.time() + HOVER_TIME_S
                        while time.time() < t_hover_end:
                            try:
                                _ts, data, _logconf = next(iter(logger))
                            except Exception:
                                break
                            seq += 1
                            x = data["stateEstimate.x"]
                            y = data["stateEstimate.y"]
                            z = data["stateEstimate.z"]
                            sim.cmd(f'sentai.explore.set_alt({z:.3f})')
                            sim.cmd('sentai.explore.tick()')
                            cur_cell = sim.query_str(
                                f"hex(sentai.places.cell({x:.3f},{y:.3f}))",
                                key=f"OBS{seq}")
                            if cur_cell:
                                sim.cmd(f'sentai.places.observe({cur_cell}, 0)')
                                overlay.add_cell(cur_cell, x, y)
                                sim.cmd(f'sentai.explore.set_cells_visited({len(overlay.spawned)})')
                            if seq % 4 == 0:
                                sim.cmd('sentai.slam.update_3d([(140,100,180,140,0.9,0)])')
                            if seq % 6 == 0:
                                info = sim.query_str(
                                    "sentai.places.info()", key=f"I{seq}")
                                state = sim.query_str(
                                    'sentai.explore.state()', key=f"S{seq}")
                                if info: print(f"[s125] places.info() = {info}")
                                if state: print(f"[s125] explore.state() = {state}")

                    # Frontier-greedy next central — first unvisited neighbor.
                    # Simple heuristic; objects_plan §7.5 calls for the
                    # neighbor whose own ring has the most unvisited cells
                    # but we defer that optimization until we've seen
                    # it run end-to-end.
                    for c in nbr_list:
                        if c not in visited_cells:
                            central_queue.append(c)
                            break

                # Fly back to origin to land at home
                print(f"[sflvp] exploration done — {len(visited_cells)} cells. flying home")
                dx, dy = -cur_x, -cur_y
                if abs(dx) > 0.01 or abs(dy) > 0.01:
                    mc.move_distance(dx, dy, 0.0, velocity=MOTION_VELOCITY)
                    cur_x, cur_y = 0.0, 0.0

                sim.cmd('sentai.explore.set_dist_home(0.0)')
                sim.cmd('sentai.explore.tick()')
                state = sim.query_str('sentai.explore.state()', key="S_END")
                print(f"[s125] explore.state() pre-landing = {state}")

            print("[s125] landing (MotionCommander exit)")
        # MotionCommander.__exit__ already issued land — no manual call needed.
        sim.cmd('sentai.explore.set_alt(0.02)')
        sim.cmd('sentai.explore.tick()')   # PRECISION_LAND → COAST_LAND → DONE
        sim.cmd('sentai.explore.tick()')
        sim.cmd('sentai.explore.tick()')
    # SyncCrazyflie exited; stop the flow thread.
    flow_stop.set()
    if flow_th is not None:
        flow_th.join(timeout=1.5)

    # final dump
    cells = sim.query_str("len(sentai.places.cells())", key="CELLS_TOTAL")
    info  = sim.query_str("sentai.places.info()", key="INFO_FINAL")
    state = sim.query_str("sentai.explore.state()", key="STATE_FINAL")
    print(f"\n[s125] DONE — cells_visited={cells}  info={info}  state={state}")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    print("[s125] starting orchestrator (v4: SFLVP + skippable detection)")
    print(f"[s125] sentai_sim = {SENTAI_SIM_BIN}")
    sim = SentaiSim(SENTAI_SIM_BIN)
    overlay = CellOverlay(WORLD_NAME)
    scorer  = TextureFeatureScorer(HARMONIC_TILE_PATH)
    try:
        fly_mission(sim, overlay, scorer)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
