#!/usr/bin/env python3
"""Run S233: ARM emulator + Gazebo camera + PrepTask + Flow + WhyCon."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import shutil
import signal
import socket
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_EMU = ROOT / "build_emu"
TARGET = "sentai_emu_gazebo_flow_whycon_repl"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_gazebo_flow_whycon.resc"
RENODE_UI_SCRIPT = ROOT / "emu/renode/sentai_emu_gazebo_flow_whycon_ui.resc"
UART_LOG = ROOT / "emu/output/sentai_emu_gazebo_flow_whycon.log"
CAM_BRIDGE_LOG = pathlib.Path("/tmp/sentai_emu_gazebo_camera_bridge.log")
CRAZY_BRIDGE_LOG = pathlib.Path("/tmp/sentai_emu_crazy_cpx_udp_bridge.log")
SITL_LOG = pathlib.Path("/tmp/respawn_sitl/sitl.log")
CAM_SOCK = pathlib.Path("/tmp/sentai_emu_cam.sock")
FLOW_OUT_SOCK = pathlib.Path("/tmp/sentai_emu_flow_out.sock")
CAM_TCP_HOST = "127.0.0.1"
CAM_TCP_PORT = 30233
DEFAULT_WORLD = "sentai_whycon_small"
SYMBOL_PREFIX = "sentai_emu_gazebo_flow_whycon"


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_gazebo_flow_whycon"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
    started = time.monotonic()
    with log_path.open("w+", encoding="utf-8") as log:
        log.write("$ " + " ".join(cmd) + "\n")
        proc = subprocess.Popen(
            cmd, cwd=str(cwd), text=True,
            stdout=log, stderr=subprocess.STDOUT)
        timed_out = False
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            proc.terminate()
            try:
                rc = proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                rc = proc.wait()
        elapsed = time.monotonic() - started
        if timed_out:
            log.write(f"\n[TIMEOUT] exceeded {timeout_s}s\n")
        log.write(f"\n[elapsed_s] {elapsed:.3f}\n")
        log.seek(0)
        out = log.read()
    return subprocess.CompletedProcess(cmd, rc, out)


def run_capture(cmd: list[str], cwd: pathlib.Path,
                timeout_s: int = 30) -> str:
    proc = subprocess.run(
        cmd, cwd=str(cwd), text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout_s, check=False)
    return proc.stdout


def start_to_file(cmd: list[str], cwd: pathlib.Path,
                  log_path: pathlib.Path) -> subprocess.Popen[str]:
    log = log_path.open("w", encoding="utf-8")
    log.write("$ " + " ".join(cmd) + "\n")
    log.flush()
    proc = subprocess.Popen(
        cmd, cwd=str(cwd), text=True,
        stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    proc._sentai_log_file = log  # type: ignore[attr-defined]
    proc._sentai_started = time.monotonic()  # type: ignore[attr-defined]
    return proc


def wait_to_completed(proc: subprocess.Popen[str], log_path: pathlib.Path,
                      timeout_s: int | None = None
                      ) -> subprocess.CompletedProcess[str]:
    timed_out = False
    try:
        rc = proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            rc = proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            rc = proc.wait()
    elapsed = time.monotonic() - getattr(proc, "_sentai_started",
                                         time.monotonic())
    log = getattr(proc, "_sentai_log_file", None)
    if log:
        if timed_out:
            log.write(f"\n[TIMEOUT] exceeded {timeout_s}s\n")
        log.write(f"\n[elapsed_s] {elapsed:.3f}\n")
        log.close()
    out = log_path.read_text(encoding="utf-8", errors="replace")
    return subprocess.CompletedProcess(proc.args, rc, out)


def wait_tcp(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.25):
                return True
        except OSError:
            time.sleep(0.1)
    return False


def wait_path_socket(path: pathlib.Path, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            return True
        time.sleep(0.1)
    return False


def cleanup_processes() -> None:
    distrobox_patterns = [
        "gz_to_uds_bridge",
        "gz sim",
        "sitl_make",
        "sentai_emu_gazebo_camera_bridge",
    ]
    for pattern in distrobox_patterns:
        try:
            subprocess.run(
                ["distrobox", "enter", "crazysim-garden", "--", "pkill",
                 "-9", "-f", pattern],
                cwd=str(ROOT), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, check=False, timeout=15)
        except Exception:
            pass
    host_patterns = [
        "gz_to_uds_bridge",
        "sentai_gazebo_uds_to_tcp_bridge.py",
        "sim/scripts/launch_hybrid_cf2.sh",
        "sitl_make/build/cf2",
        "gz sim",
        "renode .*sentai_emu_gazebo_flow_whycon",
    ]
    for pattern in host_patterns:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", pattern],
                cwd=str(ROOT), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, check=False, timeout=10)
        except Exception:
            pass
    for path in (CAM_SOCK, FLOW_OUT_SOCK):
        try:
            if path.exists():
                path.unlink()
        except Exception:
            pass


def maybe_copy(src: pathlib.Path, dst: pathlib.Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    return int(match.group(1), 16) if match else None


def parse_uart(text: str) -> dict[str, object]:
    out: dict[str, object] = {
        "begin": "S233_BEGIN" in text,
        "done": "S233_DONE" in text,
        "samples": len(re.findall(r"^S233_SAMPLE\b", text, re.MULTILINE)),
        "mark_errors": len(re.findall(r"^S233_MARK_ERR\b", text, re.MULTILINE)),
    }
    setup = re.search(
        r"S233_SETUP\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)",
        text)
    if setup:
        keys = [
            "cam_rc", "gray_ref", "flow_ref", "prep_fps",
            "prep_rc", "flow_rc", "markers_rc",
        ]
        out.update({key: int(setup.group(i + 1))
                    for i, key in enumerate(keys)})
    summary = re.search(
        r"S233_SUMMARY\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        text)
    if summary:
        keys = [
            "dt_ms", "prep_frames", "prep_fps_x100",
            "flow_frames", "flow_fps_x100", "marker_samples",
            "marker_hits", "marker_best", "flow_seq_last",
        ]
        out.update({key: int(summary.group(i + 1))
                    for i, key in enumerate(keys)})
    flow_base = re.search(r"S233_FLOW_BASELINE\s+(\d+)\s+(\d+)\s+(\d+)",
                          text)
    if flow_base:
        out["flow_base_frames"] = int(flow_base.group(1))
        out["flow_base_seq_last"] = int(flow_base.group(2))
        out["flow_base_nonzero"] = int(flow_base.group(3))
    crazy_setup = re.search(
        r"S233_CRAZY_SETUP\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)",
        text)
    if crazy_setup:
        keys = [
            "crazy_init", "crazy_ping", "crazy_arm",
            "crazy_takeoff", "crazy_hl_stop",
        ]
        out.update({key: int(crazy_setup.group(i + 1))
                    for i, key in enumerate(keys)})
    crazy_done = re.search(r"S233_CRAZY_DONE\s+(-?\d+)\s+(-?\d+)", text)
    if crazy_done:
        out["crazy_land"] = int(crazy_done.group(1))
        out["crazy_stop"] = int(crazy_done.group(2))
    last_sample = next(
        (line.strip() for line in reversed(text.splitlines())
         if line.startswith("S233_SAMPLE")),
        "")
    if last_sample:
        out["last_sample"] = last_sample
    return out


def parse_bridge_log(text: str) -> dict[str, object]:
    return {
        "connected": "client connected" in text,
        "frames_logged": len(re.findall(r"frame seq=", text)),
        "bad_header": "bad header" in text,
        "exceptions": len(re.findall(r"exception", text)),
    }


def start_gz_bridge(log_path: pathlib.Path) -> subprocess.Popen[str]:
    cmd = [
        "distrobox", "enter", "crazysim-garden", "--",
        "/home/bogdan/work/coralmicro/build-sim/sim/gz_to_uds_bridge",
        "--topic", "/downward_cam/image",
        "--in-sock", str(CAM_SOCK),
        "--out-sock", str(FLOW_OUT_SOCK),
    ]
    return start_to_file(cmd, ROOT, log_path)


def start_host_relay(log_path: pathlib.Path,
                     max_fps: float,
                     out_width: int,
                     out_height: int,
                     out_format: str) -> subprocess.Popen[str]:
    cmd = [
        sys.executable,
        "emu/host/sentai_gazebo_uds_to_tcp_bridge.py",
        "--uds", str(CAM_SOCK),
        "--tcp-host", CAM_TCP_HOST,
        "--tcp-port", str(CAM_TCP_PORT),
        "--max-fps", str(max_fps),
        "--out-width", str(out_width),
        "--out-height", str(out_height),
        "--out-format", out_format,
    ]
    return start_to_file(cmd, ROOT, log_path)


def stop_proc(proc: subprocess.Popen[str] | None) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        log = getattr(proc, "_sentai_log_file", None)
        if log:
            log.close()
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
        try:
            proc.wait(timeout=3)
        except Exception:
            pass
    log = getattr(proc, "_sentai_log_file", None)
    if log:
        try:
            log.close()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--renode-ui", action="store_true")
    parser.add_argument("--no-sitl", action="store_true")
    parser.add_argument("--keep-sitl", action="store_true")
    parser.add_argument(
        "--camera-forward-fps", type=float, default=10.0,
        help="Max Gazebo camera frames/sec forwarded into Renode.")
    parser.add_argument("--camera-out-width", type=int, default=640)
    parser.add_argument("--camera-out-height", type=int, default=480)
    parser.add_argument("--camera-out-format", default="xrgb8888",
                        choices=("rgb888", "xrgb8888"))
    args = parser.parse_args()

    iter_dir = next_iter_dir()
    (iter_dir / "renode").mkdir()
    (iter_dir / "mission").mkdir()

    renode_run_script = RENODE_UI_SCRIPT if args.renode_ui else RENODE_SCRIPT
    commands = {
        "py_compile": [
            sys.executable, "-m", "py_compile",
            "emu/renode/crazy_cpx_udp_mmio_bridge.py",
            "emu/renode/gazebo_camera_mmio_bridge.py",
            "emu/host/sentai_crazy_cpx_udp_bridge.py",
            "emu/host/sentai_gazebo_uds_to_tcp_bridge.py",
        ],
        "configure": [
            "cmake", "-S", ".", "-B", str(BUILD_EMU.relative_to(ROOT)),
            "-DSENTAI_ARM_EMU=ON", "-DSENTAI_SKIP_SDK_PATCHES=ON",
        ],
        "build": [
            "cmake", "--build", str(BUILD_EMU.relative_to(ROOT)),
            "--target", TARGET, f"-j{os.cpu_count() or 1}",
        ],
        "respawn_sitl": [
            "bash", "sim/scripts/respawn_sitl.sh", args.world,
        ],
    }
    if args.renode_ui:
        commands["renode"] = [
            str(RENODE), "--console", str(renode_run_script.relative_to(ROOT))]
    else:
        commands["renode"] = [
            str(RENODE), "--plain", "--console", "--disable-xwt",
            str(renode_run_script.relative_to(ROOT))]

    results: dict[str, object] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "target": TARGET,
        "world": args.world,
        "renode_ui": args.renode_ui,
        "camera_forward_fps": args.camera_forward_fps,
        "camera_out_width": args.camera_out_width,
        "camera_out_height": args.camera_out_height,
        "camera_out_format": args.camera_out_format,
        "renode_script": str(renode_run_script.relative_to(ROOT)),
        "commands": commands,
    }

    gz_bridge_proc: subprocess.Popen[str] | None = None
    relay_proc: subprocess.Popen[str] | None = None
    try:
        cleanup_processes()
        for name in ("py_compile", "configure", "build"):
            proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log",
                               300)
            results[f"{name}_rc"] = proc.returncode
            if proc.returncode != 0:
                (iter_dir / "verdict_s233.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                return proc.returncode

        if not args.no_sitl:
            proc = run_to_file(commands["respawn_sitl"], ROOT,
                               iter_dir / "respawn_sitl.log", 180)
            results["respawn_sitl_rc"] = proc.returncode
            maybe_copy(SITL_LOG, iter_dir / "sitl.log")
            if proc.returncode != 0:
                (iter_dir / "verdict_s233.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                return proc.returncode

        for path in (UART_LOG, CAM_BRIDGE_LOG, CRAZY_BRIDGE_LOG):
            try:
                if path.exists():
                    path.unlink()
            except Exception:
                pass

        (iter_dir / "processes_before_renode.log").write_text(
            run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
            encoding="utf-8")

        if not args.no_sitl:
            relay_proc = start_host_relay(
                iter_dir / "gazebo_uds_tcp_relay.log",
                args.camera_forward_fps,
                args.camera_out_width,
                args.camera_out_height,
                args.camera_out_format)
            results["camera_tcp_ready"] = wait_tcp(
                CAM_TCP_HOST, CAM_TCP_PORT, 10.0)
            if wait_path_socket(CAM_SOCK, 5.0):
                gz_bridge_proc = start_gz_bridge(
                    iter_dir / "gz_to_uds_bridge.log")
            else:
                results["relay_uds_ready"] = False
        else:
            results["camera_tcp_ready"] = False

        renode_popen = start_to_file(commands["renode"], ROOT,
                                     iter_dir / "renode.log")

        renode_proc = wait_to_completed(renode_popen, iter_dir / "renode.log",
                                        300)
        results["renode_rc"] = renode_proc.returncode

        uart_text = ""
        if maybe_copy(UART_LOG, iter_dir / "uart.log"):
            uart_text = (iter_dir / "uart.log").read_text(
                encoding="utf-8", errors="replace")
        else:
            results["uart_log_missing"] = True

        cam_bridge_text = ""
        if maybe_copy(CAM_BRIDGE_LOG,
                      iter_dir / "gazebo_camera_mmio_bridge.log"):
            cam_bridge_text = (
                iter_dir / "gazebo_camera_mmio_bridge.log"
            ).read_text(encoding="utf-8", errors="replace")
        else:
            results["camera_bridge_log_missing"] = True

        if maybe_copy(CRAZY_BRIDGE_LOG,
                      iter_dir / "crazy_cpx_udp_bridge.log"):
            pass

        shutil.copy2(renode_run_script,
                     iter_dir / "renode" / renode_run_script.name)
        shutil.copy2(ROOT / "emu/renode/sentai_rt1176.repl",
                     iter_dir / "renode/sentai_rt1176.repl")
        shutil.copy2(ROOT / "emu/renode/gazebo_camera_mmio_bridge.py",
                     iter_dir / "renode/gazebo_camera_mmio_bridge.py")
        shutil.copy2(ROOT / "emu/host/sentai_gazebo_uds_to_tcp_bridge.py",
                     iter_dir / "renode/sentai_gazebo_uds_to_tcp_bridge.py")
        shutil.copy2(ROOT / "emu/renode/crazy_cpx_udp_mmio_bridge.py",
                     iter_dir / "renode/crazy_cpx_udp_mmio_bridge.py")

        labels = {
            "boot_state": f"{SYMBOL_PREFIX} boot_state",
            "heartbeat": f"{SYMBOL_PREFIX} heartbeat",
            "repl_lines": f"{SYMBOL_PREFIX} repl_lines",
            "last_tick": f"{SYMBOL_PREFIX} last_tick",
            "serial_open_calls": f"{SYMBOL_PREFIX} serial_open_calls",
            "serial_write_calls": f"{SYMBOL_PREFIX} serial_write_calls",
            "serial_read_calls": f"{SYMBOL_PREFIX} serial_read_calls",
            "serial_bytes_tx": f"{SYMBOL_PREFIX} serial_bytes_tx",
            "serial_bytes_rx": f"{SYMBOL_PREFIX} serial_bytes_rx",
            "serial_last_result": f"{SYMBOL_PREFIX} serial_last_result",
            "cam_task_started": f"{SYMBOL_PREFIX} cam_task_started",
            "cam_frames": f"{SYMBOL_PREFIX} cam_frames",
            "cam_publish_fail": f"{SYMBOL_PREFIX} cam_publish_fail",
            "cam_last_seq": f"{SYMBOL_PREFIX} cam_last_seq",
            "cam_last_rc": f"{SYMBOL_PREFIX} cam_last_rc",
            "bridge_seen": f"{SYMBOL_PREFIX} bridge_seen",
            "bridge_served": f"{SYMBOL_PREFIX} bridge_served",
            "bridge_dropped": f"{SYMBOL_PREFIX} bridge_dropped",
            "bridge_bad": f"{SYMBOL_PREFIX} bridge_bad",
            "bridge_read_ms_sum": f"{SYMBOL_PREFIX} bridge_read_ms_sum",
            "bridge_read_ms_max": f"{SYMBOL_PREFIX} bridge_read_ms_max",
            "publish_ms_sum": f"{SYMBOL_PREFIX} publish_ms_sum",
            "publish_ms_max": f"{SYMBOL_PREFIX} publish_ms_max",
            "loop_ms_sum": f"{SYMBOL_PREFIX} loop_ms_sum",
            "loop_ms_max": f"{SYMBOL_PREFIX} loop_ms_max",
        }
        symbols = {
            key: parse_hex(label, renode_proc.stdout)
            for key, label in labels.items()
        }
        uart = parse_uart(uart_text)
        cam_bridge = parse_bridge_log(cam_bridge_text)
        results["symbols"] = symbols
        results["uart"] = uart
        results["gazebo_camera_bridge"] = cam_bridge

        results["pass_flow"] = (
            renode_proc.returncode == 0
            and symbols.get("boot_state") == 0x0500
            and symbols.get("cam_task_started") == 1
            and symbols.get("cam_frames", 0) > 0
            and uart.get("done") is True
            and uart.get("prep_rc") == 0
            and uart.get("flow_rc") == 0
            and uart.get("prep_frames", 0) > 0
            and uart.get("flow_frames", 0) > 0
            and uart.get("flow_seq_last", 0) > 0
        )
        results["pass_whycon"] = (
            results["pass_flow"] is True
            and uart.get("markers_rc") == 0
            and uart.get("marker_samples", 0) > 0
            and uart.get("marker_hits", 0) > 0
            and uart.get("mark_errors", 0) == 0
        )
        results["pass"] = (
            results["pass_flow"] is True and results["pass_whycon"] is True)

        summary = [
            f"pass={results['pass']}",
            f"pass_flow={results['pass_flow']}",
            f"pass_whycon={results['pass_whycon']}",
            f"cam_frames={symbols.get('cam_frames')}",
            f"bridge_seen={symbols.get('bridge_seen')}",
            f"bridge_served={symbols.get('bridge_served')}",
            f"bridge_read_ms_sum={symbols.get('bridge_read_ms_sum')}",
            f"bridge_read_ms_max={symbols.get('bridge_read_ms_max')}",
            f"publish_ms_sum={symbols.get('publish_ms_sum')}",
            f"publish_ms_max={symbols.get('publish_ms_max')}",
            f"loop_ms_sum={symbols.get('loop_ms_sum')}",
            f"loop_ms_max={symbols.get('loop_ms_max')}",
            f"prep_frames={uart.get('prep_frames')}",
            f"prep_fps_x100={uart.get('prep_fps_x100')}",
            f"flow_frames={uart.get('flow_frames')}",
            f"flow_fps_x100={uart.get('flow_fps_x100')}",
            f"flow_base_frames={uart.get('flow_base_frames')}",
            f"flow_base_seq_last={uart.get('flow_base_seq_last')}",
            f"flow_base_nonzero={uart.get('flow_base_nonzero')}",
            f"marker_samples={uart.get('marker_samples')}",
            f"marker_hits={uart.get('marker_hits')}",
            f"marker_best={uart.get('marker_best')}",
            f"crazy_init={uart.get('crazy_init')}",
            f"crazy_ping={uart.get('crazy_ping')}",
            f"crazy_arm={uart.get('crazy_arm')}",
            f"crazy_takeoff={uart.get('crazy_takeoff')}",
            f"crazy_land={uart.get('crazy_land')}",
            f"crazy_stop={uart.get('crazy_stop')}",
            str(uart.get("last_sample", "")),
        ]
        (iter_dir / "summary_s233.txt").write_text(
            "\n".join(x for x in summary if x) + "\n", encoding="utf-8")
        (iter_dir / "verdict_s233.json").write_text(
            json.dumps(results, indent=2) + "\n", encoding="utf-8")
        (iter_dir / "processes_after_renode.log").write_text(
            run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
            encoding="utf-8")

        print(f"S233 iter: {iter_dir.relative_to(ROOT)}")
        print((iter_dir / "summary_s233.txt").read_text(
            encoding="utf-8").strip())
        return 0 if results["pass"] else 1
    finally:
        stop_proc(gz_bridge_proc)
        stop_proc(relay_proc)
        if not args.keep_sitl:
            cleanup_processes()


if __name__ == "__main__":
    raise SystemExit(main())
