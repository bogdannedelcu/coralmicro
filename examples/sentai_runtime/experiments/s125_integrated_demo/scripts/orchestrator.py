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
import subprocess
import sys
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
URI = "udp://127.0.0.1:19850"

# Demo mission: 4-corner square at altitude 1.0 m, cf2-natural units (m).
# Per Sim.md §10v, scale=10 maps cf2 → PX4-equiv.  We feed PX4-equiv
# coordinates to sentai.places.cell() via sentai.places.init(scale=10).
WORLD_NAME = "s125_demo"
WAYPOINTS_M = [
    (1.5,  0.0, 1.0),   # east
    (1.5,  1.5, 1.0),   # northeast
    (0.0,  1.5, 1.0),   # north
    (0.0,  0.0, 1.0),   # back to origin
]
HOVER_TIME_S        = 4.0   # at each waypoint
OBSERVE_HZ          = 4     # cell observations per second during cruise
SENTAI_SIM_BIN      = Path(__file__).resolve().parents[5] / "build-sim/sim/sentai_sim"

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

class CellOverlay:
    """Spawn a small cylinder marker at each newly-visited cell centroid.

    The world uses cf2 physical units (meters from origin).  We accept
    cf2 physical coordinates and spawn at the same (x, y) — but at the
    cell *centroid* (i.e., quantized to the H3 grid lattice as known from
    the orchestrator side).
    """
    SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} 0.02 0 0 0</pose>
    <link name="link">
      <visual name="v">
        <geometry><cylinder><radius>0.35</radius><length>0.04</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 0.5</ambient>
          <diffuse>{r} {g} {b} 0.5</diffuse>
        </material>
        <transparency>0.5</transparency>
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
        sdf_txt = self.SDF_TEMPLATE.format(name=name, x=x, y=y, r=r, g=g, b=b).replace("\n", " ")
        # Write SDF to a temp file (multi-line strings in gz service args are tricky)
        tmpf = Path(f"/tmp/{name}.sdf")
        tmpf.write_text(self.SDF_TEMPLATE.format(name=name, x=x, y=y, r=r, g=g, b=b))
        req = (f'sdf_filename: "{tmpf}", '
               f'pose: {{position: {{x: {x}, y: {y}, z: 0.02}}}}, '
               f'name: "{name}", allow_renaming: 1')
        try:
            subprocess.run([
                "gz", "service", "-s", f"/world/{self.world}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "1000",
                "--req", req,
            ], check=False, capture_output=True, timeout=3.0)
        except subprocess.TimeoutExpired:
            print(f"[overlay] spawn timeout for {name}")


# ---------------------------------------------------------------------------
# cflib helper — high-level commander, log pose stream
# ---------------------------------------------------------------------------

def fly_mission(sim: SentaiSim, overlay: CellOverlay):
    cflib.crtp.init_drivers()
    print(f"[s125] connecting cflib → {URI}")
    cf = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(URI, cf=cf) as scf:
        commander = scf.cf.high_level_commander

        # CrazyFlie HighLevelCommander needs the firmware param enabled.
        # Default cf2 firmware ships with enHighLevel=0; we MUST flip this
        # before any takeoff/go_to/land call or the planner ignores them
        # and the drone drifts uncommanded (operator-reported: "crashing
        # chaotically", 2026-05-13).
        scf.cf.param.set_value("commander.enHighLevel", "1")
        # Use the legacy PID controller (1) — Mellinger (2) needs trajectory
        # smoothing we don't provide here.
        scf.cf.param.set_value("stabilizer.controller", "1")
        time.sleep(0.3)

        # Reset Kalman estimator so spawn pose is the origin.
        scf.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.5)
        scf.cf.param.set_value("kalman.resetEstimation", "0")
        time.sleep(1.5)

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

        # Start pose-logger early — covers takeoff + waypoints in one loop.
        lg = LogConfig(name="pose", period_in_ms=200)
        lg.add_variable("stateEstimate.x", "float")
        lg.add_variable("stateEstimate.y", "float")
        lg.add_variable("stateEstimate.z", "float")

        print("[s125] takeoff → 1.0 m (drone holds over ArUco grid)")
        commander.takeoff(1.0, 3.5)
        sim.cmd('sentai.explore.set_alt(0.0)')   # initial value; logger overrides

        with SyncLogger(scf, lg) as logger:
            # Phase 1 — hold over markers until altitude ≥ 0.95 m.
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

            seq = -1
            wp_idx = 0
            # Hover-confirm pause so the drone visibly holds before moving
            print("[s125] hold 1.5 s at 1.0 m (visual confirmation)…")
            time.sleep(1.5)

            # Issue the first waypoint.
            wx0, wy0, wz0 = WAYPOINTS_M[0]
            print(f"[s125] → waypoint 0 ({wx0},{wy0},{wz0})")
            commander.go_to(wx0, wy0, wz0, 0.0, 3.0, relative=False)
            wp_idx = 1
            t_arrive = time.time() + 3.0 + HOVER_TIME_S   # 3s travel + hover

            # Phase 2 — pose loop until all waypoints visited.
            for entry in logger:
                seq += 1
                _ts, data, _logconf = entry
                x = data["stateEstimate.x"]
                y = data["stateEstimate.y"]
                z = data["stateEstimate.z"]

                sim.cmd(f'sentai.explore.set_alt({z:.3f})')

                cell = sim.query_str(f"hex(sentai.places.cell({x:.3f},{y:.3f}))",
                                     key=f"C{seq}")
                if cell:
                    sim.cmd(f'sentai.places.observe({cell}, 0)')
                    overlay.add_cell(cell, x, y)
                if seq % 4 == 0:
                    sim.cmd('sentai.slam.update_3d([(140,100,180,140,0.9,0)])')

                if time.time() >= t_arrive:
                    if wp_idx < len(WAYPOINTS_M):
                        wx, wy, wz = WAYPOINTS_M[wp_idx]
                        print(f"[s125] → waypoint {wp_idx} ({wx},{wy},{wz})")
                        commander.go_to(wx, wy, wz, 0.0, 3.0, relative=False)
                        wp_idx += 1
                        t_arrive = time.time() + 3.0 + HOVER_TIME_S
                    else:
                        break

                if seq % 8 == 0:
                    info = sim.query_str("sentai.places.info()", key=f"I{seq}")
                    if info:
                        print(f"[s125] places.info() = {info}")

            sim.cmd('sentai.explore.set_cells_visited(8)')

        print("[s125] landing")
        commander.land(0.0, 3.0)
        time.sleep(4.0)
        sim.cmd('sentai.explore.set_alt(0.02)')

    # final dump
    cells = sim.query_str("len(sentai.places.cells())", key="CELLS_TOTAL")
    info  = sim.query_str("sentai.places.info()", key="INFO_FINAL")
    state = sim.query_str("sentai.explore.state()", key="STATE_FINAL")
    print(f"\n[s125] DONE — cells_visited={cells}  info={info}  state={state}")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    print("[s125] starting orchestrator (v3: enHighLevel + ArUco hold + altitude gate)")
    print(f"[s125] sentai_sim = {SENTAI_SIM_BIN}")
    sim = SentaiSim(SENTAI_SIM_BIN)
    overlay = CellOverlay(WORLD_NAME)
    try:
        fly_mission(sim, overlay)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
