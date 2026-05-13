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
    def __init__(self, sim_bin: Path):
        if not sim_bin.exists():
            sys.exit(f"[s125] sentai_sim not found at {sim_bin}; run cmake --build build-sim")
        self.proc = subprocess.Popen(
            [str(sim_bin)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        self._buf: list[str] = []
        # Wait for banner so subsequent commands are seen
        self._consume_until(r">>> ", timeout=4.0)

    def _consume_until(self, pattern: str, timeout: float):
        deadline = time.time() + timeout
        rgx = re.compile(pattern)
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                break
            self._buf.append(line.rstrip())
            if rgx.search(line):
                return
        # otherwise — best effort

    def cmd(self, line: str):
        """Send a one-liner Python statement; no return value parsing."""
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def query_str(self, py_expr: str, key: str = "Q") -> str | None:
        """Evaluate `print('=KEY', <expr>)`; return the printed string."""
        # Use a distinct key so we can grep our own response from the noisy buffer.
        self.cmd(f"print('={key}',{py_expr})")
        # Read up to 30 lines or 0.5 s
        deadline = time.time() + 0.5
        prefix = f"={key} "
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                break
            line = line.rstrip()
            self._buf.append(line)
            # REPL echoes ">>> " and possibly "..." prefix
            idx = line.find(prefix)
            if idx >= 0:
                return line[idx + len(prefix):]
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

        # Reset Kalman estimator + arm
        scf.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.5)
        scf.cf.param.set_value("kalman.resetEstimation", "0")
        time.sleep(1.0)

        print("[s125] arming + takeoff")
        sim.cmd('sentai.places.init(40.689167, -74.044444, 10.0, 13)')
        sim.cmd('sentai.slam.init(70.0, 320, 240)')
        sim.cmd('sentai.slam.set_class_prior(0, 1.7)')
        sim.cmd('sentai.servo.init("cf2")')
        sim.cmd('sentai.servo.arm()')
        sim.cmd('sentai.explore.start("s125")')
        sim.cmd('sentai.explore.set_arm_ack(1)')
        sim.cmd('sentai.explore.set_marker(1)')

        # Start pose-logger
        lg = LogConfig(name="pose", period_in_ms=250)
        lg.add_variable("stateEstimate.x", "float")
        lg.add_variable("stateEstimate.y", "float")
        lg.add_variable("stateEstimate.z", "float")
        with SyncLogger(scf, lg) as logger:
            # Takeoff
            commander.takeoff(1.0, 3.0)
            time.sleep(4.0)
            sim.cmd('sentai.explore.set_alt(1.0)')   # → TAKEOFF passes guard

            # Drive through waypoints
            seq = -1
            wp_idx = 0
            t_arrive = time.time() + HOVER_TIME_S
            for entry in logger:
                seq += 1
                _ts, data, _logconf = entry
                x = data["stateEstimate.x"]
                y = data["stateEstimate.y"]
                z = data["stateEstimate.z"]

                # Push state to sentai
                sim.cmd(f'sentai.explore.set_alt({z:.3f})')
                # observe current cell — pose mapped to PX4-equivalent via scale=10
                cell = sim.query_str(f"hex(sentai.places.cell({x:.3f},{y:.3f}))",
                                     key=f"C{seq}")
                if cell:
                    sim.cmd(f'sentai.places.observe({cell}, 0)')
                    overlay.add_cell(cell, x, y)
                # Periodically inject a synthetic detection so slam.update_3d runs
                if seq % 4 == 0:
                    sim.cmd('sentai.slam.update_3d([(140,100,180,140,0.9,0)])')

                # Progress to next waypoint when time elapsed
                if time.time() >= t_arrive:
                    if wp_idx < len(WAYPOINTS_M):
                        wx, wy, wz = WAYPOINTS_M[wp_idx]
                        print(f"[s125] → waypoint {wp_idx} ({wx},{wy},{wz})")
                        commander.go_to(wx, wy, wz, 0.0, 3.0, relative=False)
                        wp_idx += 1
                        t_arrive = time.time() + HOVER_TIME_S
                    else:
                        break

                # Operator visibility every 2s
                if seq % 8 == 0:
                    info = sim.query_str("sentai.places.info()", key=f"I{seq}")
                    if info:
                        print(f"[s125] places.info() = {info}")

            # Tell explore we have enough cells -> RTH
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
    print("[s125] starting orchestrator")
    print(f"[s125] sentai_sim = {SENTAI_SIM_BIN}")
    sim = SentaiSim(SENTAI_SIM_BIN)
    overlay = CellOverlay(WORLD_NAME)
    try:
        fly_mission(sim, overlay)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
