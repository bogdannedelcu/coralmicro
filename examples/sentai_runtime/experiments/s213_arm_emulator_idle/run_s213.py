#!/usr/bin/env python3
"""Run B8.1 ARM emulator idle heartbeat experiment."""

from __future__ import annotations

import json
import argparse
import pathlib
import re
import shutil
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_DIR = ROOT / "build_emu"
TARGETS = {
    "idle": {
        "cmake_target": "sentai_emu_idle",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_idle.resc",
        "iter_suffix": "renode_idle_heartbeat",
        "uart_log": None,
    },
    "uart": {
        "cmake_target": "sentai_emu_uart",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_uart.resc",
        "iter_suffix": "renode_uart_heartbeat",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_uart.log",
    },
    "repl": {
        "cmake_target": "sentai_emu_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_repl.resc",
        "iter_suffix": "renode_repl_oneplusone",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_repl.log",
    },
    "mission": {
        "cmake_target": "sentai_emu_mission",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_mission.resc",
        "iter_suffix": "renode_mission_import",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_mission.log",
    },
    "camera": {
        "cmake_target": "sentai_emu_camera",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_camera.resc",
        "iter_suffix": "renode_camera_5frames",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_camera.log",
    },
    "pipeline": {
        "cmake_target": "sentai_emu_pipeline",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_pipeline.resc",
        "iter_suffix": "renode_pipeline_stage1_stage2",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_pipeline.log",
    },
    "fanout": {
        "cmake_target": "sentai_emu_fanout",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_fanout.resc",
        "iter_suffix": "renode_fanout_stage1_2a_2b",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_fanout.log",
    },
    "flowest": {
        "cmake_target": "sentai_emu_flowest",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_flowest.resc",
        "iter_suffix": "renode_flowest_cat_2d_varied",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_flowest.log",
        "pre_hook": "prepare_cat_scenes",
    },
    "usbhost_probe": {
        "cmake_target": "sentai_emu_usbhost_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_usbhost_probe.resc",
        "iter_suffix": "renode_usbhost_probe_ctor",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_usbhost_probe.log",
    },
    "usbhost_task": {
        "cmake_target": "sentai_emu_usbhost_task",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_usbhost_task.resc",
        "iter_suffix": "renode_usbhost_task_scheduler",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_usbhost_task.log",
    },
    "edgetpu_manager_probe": {
        "cmake_target": "sentai_emu_edgetpu_manager_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_manager_probe.resc",
        "iter_suffix": "renode_edgetpu_manager_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_manager_probe.log",
    },
    "edgetpu_task_probe": {
        "cmake_target": "sentai_emu_edgetpu_task_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_task_probe.resc",
        "iter_suffix": "renode_edgetpu_task_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_task_probe.log",
    },
    "edgetpu_opendevice_probe": {
        "cmake_target": "sentai_emu_edgetpu_opendevice_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_opendevice_probe.resc",
        "iter_suffix": "renode_edgetpu_opendevice_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_opendevice_probe.log",
    },
    "edgetpu_synth_enum_probe": {
        "cmake_target": "sentai_emu_edgetpu_synth_enum_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_synth_enum_probe.resc",
        "iter_suffix": "renode_edgetpu_synth_enum_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_synth_enum_probe.log",
    },
    "edgetpu_mmio_enum_probe": {
        "cmake_target": "sentai_emu_edgetpu_mmio_enum_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_mmio_enum_probe.resc",
        "iter_suffix": "renode_edgetpu_mmio_enum_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_mmio_enum_probe.log",
    },
    "edgetpu_mmio_opendevice_probe": {
        "cmake_target": "sentai_emu_edgetpu_mmio_opendevice_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_mmio_opendevice_probe.resc",
        "iter_suffix": "renode_edgetpu_mmio_opendevice_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_mmio_opendevice_probe.log",
    },
    "edgetpu_mmio_send_bridge_probe": {
        "cmake_target": "sentai_emu_edgetpu_mmio_send_bridge_probe",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_edgetpu_mmio_send_bridge_probe.resc",
        "iter_suffix": "renode_edgetpu_mmio_send_bridge_probe",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_edgetpu_mmio_send_bridge_probe.log",
    },
    "fx_storage": {
        "cmake_target": "sentai_emu_fx_storage",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_fx_storage.resc",
        "iter_suffix": "renode_fx_storage_filex_levelx_nand",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_fx_storage.log",
        "pre_hook": "init_emu_nand",
    },
    "fs_repl": {
        "cmake_target": "sentai_emu_fs_repl_smoke",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_fs_repl_smoke.resc",
        "iter_suffix": "renode_fs_repl_filex_levelx_nand",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_fs_repl_smoke.log",
        "pre_hook": "init_emu_nand",
    },
    "fs_stage_assets": {
        "cmake_target": "sentai_emu_fs_stage_assets",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_fs_stage_assets.resc",
        "iter_suffix": "renode_fs_stage_assets_filex_levelx_nand",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_fs_stage_assets.log",
        "pre_hook": "ensure_emu_nand",
    },
    "fs_asset_check": {
        "cmake_target": "sentai_emu_fs_asset_check",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_fs_asset_check.resc",
        "iter_suffix": "renode_fs_asset_check_filex_levelx_nand",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_fs_asset_check.log",
    },
    "tpu_cat_repl": {
        "cmake_target": "sentai_emu_tpu_cat_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_cat_repl.resc",
        "iter_suffix": "renode_tpu_cat_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_cat_repl.log",
    },
    "tpu_fps_repl": {
        "cmake_target": "sentai_emu_tpu_fps_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_fps_repl.resc",
        "iter_suffix": "renode_tpu_fps_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_fps_repl.log",
    },
    "tpu_fps_mem_repl": {
        "cmake_target": "sentai_emu_tpu_fps_mem_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_fps_mem_repl.resc",
        "iter_suffix": "renode_tpu_fps_mem_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_fps_mem_repl.log",
    },
    "tpu_fps_mem_invoke_repl": {
        "cmake_target": "sentai_emu_tpu_fps_mem_invoke_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_fps_mem_invoke_repl.resc",
        "iter_suffix": "renode_tpu_fps_mem_invoke_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_fps_mem_invoke_repl.log",
    },
    "tpu_fps_mem_session_repl": {
        "cmake_target": "sentai_emu_tpu_fps_mem_session_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_fps_mem_session_repl.resc",
        "iter_suffix": "renode_tpu_fps_mem_session_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_fps_mem_session_repl.log",
    },
    "tpu_timing_repl": {
        "cmake_target": "sentai_emu_tpu_timing_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_timing_repl.resc",
        "iter_suffix": "renode_tpu_timing_repl_filex_physical_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_timing_repl.log",
        "renode_timeout_s": 240,
    },
    "tpu_physical_send_smoke": {
        "cmake_target": "sentai_emu_tpu_physical_send_smoke",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_physical_send_smoke.resc",
        "iter_suffix": "renode_tpu_physical_send_smoke_filex_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_physical_send_smoke.log",
        "sim_cmake_target": "tpu_posix_send_server",
        "expected_tpu_physical_completed": 2,
    },
    "tpu_physical_send_fps": {
        "cmake_target": "sentai_emu_tpu_physical_send_fps",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_tpu_physical_send_fps.resc",
        "iter_suffix": "renode_tpu_physical_send_fps_filex_coral",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_tpu_physical_send_fps.log",
        "sim_cmake_target": "tpu_posix_send_server",
        "expected_tpu_physical_completed": 10,
    },
}


def next_iter_dir(suffix: str) -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        last_id = max(int(path.name.split("_", 1)[0].replace("iter", ""))
                      for path in existing)
        next_id = last_id + 1
    out = EXP / f"iter{next_id:02d}_{suffix}"
    out.mkdir(parents=True)
    return out


def run_cmd(cmd: list[str], cwd: pathlib.Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def run_cmd_to_file(cmd: list[str],
                    cwd: pathlib.Path,
                    log_path: pathlib.Path,
                    timeout_s: int) -> subprocess.CompletedProcess[str]:
    with log_path.open("w+", encoding="utf-8") as log:
        proc = subprocess.Popen(
            cmd,
            cwd=str(cwd),
            text=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
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
        if timed_out:
            log.write(f"\n[TIMEOUT] command exceeded {timeout_s}s\n")
        log.seek(0)
        stdout = log.read()
    return subprocess.CompletedProcess(cmd, rc, stdout)


def parse_hex(label: str, text: str) -> int | None:
    pattern = rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)"
    match = re.search(pattern, text)
    if not match:
        return None
    return int(match.group(1), 16)


def parse_physical_send_host_timing(text: str) -> dict:
    """Parse real host wall-clock timing from the Renode TPU Send* bridge log."""
    entries: list[dict] = []
    server_stats: list[dict] = []
    bridge_re = re.compile(
        r"t=(?P<t>\d+)ms bridge cmd=(?P<cmd>\w+) seq=(?P<seq>\d+) "
        r"data_len=(?P<data>\d+) out_len=(?P<out>\d+) ok=(?P<ok>True|False) "
        r"ms=(?P<ms>\d+)"
    )
    stats_re = re.compile(r"server SEND_STATS (?P<body>.*)")
    kv_re = re.compile(r"(\w+)=([0-9]+)")
    for line in text.splitlines():
        m = bridge_re.search(line)
        if m:
            end_ms = int(m.group("t"))
            bridge_ms = int(m.group("ms"))
            entries.append({
                "cmd": m.group("cmd"),
                "seq": int(m.group("seq")),
                "data_len": int(m.group("data")),
                "out_len": int(m.group("out")),
                "ok": m.group("ok") == "True",
                "bridge_ms": bridge_ms,
                "bridge_start_ms": max(0, end_ms - bridge_ms),
                "bridge_end_ms": end_ms,
            })
            continue
        m = stats_re.search(line)
        if m:
            body = m.group("body")
            stat = {k: int(v) for k, v in kv_re.findall(body)}
            cmd_match = re.search(r"cmd=(\w+)", body)
            if cmd_match:
                stat["cmd"] = cmd_match.group(1)
            server_stats.append(stat)

    for entry, stat in zip(entries, server_stats):
        entry["server_ms"] = stat.get("ms")
        entry["usb_out_us"] = stat.get("usb_out_us", 0)
        entry["usb_in_us"] = stat.get("usb_in_us", 0)
        entry["usb_event_us"] = stat.get("usb_event_us", 0)
        entry["usb_failed"] = stat.get("usb_failed", 0)
        entry["usb_timeouts"] = stat.get("usb_timeouts", 0)

    first_invoke_idx = None
    for i, entry in enumerate(entries):
        if entry["cmd"] == "ins" and entry["data_len"] > 100000:
            first_invoke_idx = i
            break
    if first_invoke_idx is None:
        return {
            "command_count": len(entries),
            "invoke_count": 0,
            "error": "no invoke instruction command found",
        }

    setup = entries[:first_invoke_idx]
    invokes: list[list[dict]] = []
    current: list[dict] = []
    for entry in entries[first_invoke_idx:]:
        if entry["cmd"] == "ins" and current:
            invokes.append(current)
            current = []
        current.append(entry)
        if entry["cmd"] == "event":
            invokes.append(current)
            current = []
    if current:
        invokes.append(current)

    def wall_ms(group: list[dict]) -> int:
        if not group:
            return 0
        return max(e["bridge_end_ms"] for e in group) - min(
            e["bridge_start_ms"] for e in group)

    def sum_field(group: list[dict], field: str, cmd: str | None = None) -> int:
        return sum(int(e.get(field) or 0) for e in group
                   if cmd is None or e["cmd"] == cmd)

    def avg_stage(groups: list[list[dict]], field: str, cmd: str) -> float:
        if not groups:
            return 0.0
        return sum(sum_field(g, field, cmd) for g in groups) / len(groups)

    steady = invokes[1:]
    measured_groups = invokes
    measured_start = min(e["bridge_start_ms"] for g in measured_groups for e in g)
    measured_end = max(e["bridge_end_ms"] for g in measured_groups for e in g)
    measured_ms = measured_end - measured_start
    steady_ms = 0
    if steady:
        steady_start = min(e["bridge_start_ms"] for g in steady for e in g)
        steady_end = max(e["bridge_end_ms"] for g in steady for e in g)
        steady_ms = steady_end - steady_start

    setup_wall = wall_ms(setup)
    setup_summary = {
        "wall_ms": setup_wall,
        "bridge_sum_ms": sum_field(setup, "bridge_ms"),
        "server_sum_ms": sum_field(setup, "server_ms"),
        "params_bytes": sum_field(setup, "data_len", "params"),
        "ins_bytes": sum_field(setup, "data_len", "ins"),
        "event_calls": sum(1 for e in setup if e["cmd"] == "event"),
    }
    first = invokes[0] if invokes else []
    first_summary = {
        "wall_ms": wall_ms(first),
        "bridge_sum_ms": sum_field(first, "bridge_ms"),
        "server_sum_ms": sum_field(first, "server_ms"),
        "ins_bridge_ms": sum_field(first, "bridge_ms", "ins"),
        "inputs_bridge_ms": sum_field(first, "bridge_ms", "inputs"),
        "output_bridge_ms": sum_field(first, "bridge_ms", "output"),
        "event_bridge_ms": sum_field(first, "bridge_ms", "event"),
        "ins_server_ms": sum_field(first, "server_ms", "ins"),
        "inputs_server_ms": sum_field(first, "server_ms", "inputs"),
        "output_server_ms": sum_field(first, "server_ms", "output"),
        "event_server_ms": sum_field(first, "server_ms", "event"),
    }
    steady_summary = {
        "count": len(steady),
        "wall_ms": steady_ms,
        "fps": (len(steady) * 1000.0 / steady_ms) if steady_ms else 0.0,
        "avg_wall_ms": (
            sum(wall_ms(g) for g in steady) / len(steady) if steady else 0.0),
        "avg_bridge_sum_ms": (
            sum(sum_field(g, "bridge_ms") for g in steady) / len(steady)
            if steady else 0.0),
        "avg_server_sum_ms": (
            sum(sum_field(g, "server_ms") for g in steady) / len(steady)
            if steady else 0.0),
        "avg_ins_bridge_ms": avg_stage(steady, "bridge_ms", "ins"),
        "avg_inputs_bridge_ms": avg_stage(steady, "bridge_ms", "inputs"),
        "avg_output_bridge_ms": avg_stage(steady, "bridge_ms", "output"),
        "avg_event_bridge_ms": avg_stage(steady, "bridge_ms", "event"),
        "avg_ins_server_ms": avg_stage(steady, "server_ms", "ins"),
        "avg_inputs_server_ms": avg_stage(steady, "server_ms", "inputs"),
        "avg_output_server_ms": avg_stage(steady, "server_ms", "output"),
        "avg_event_server_ms": avg_stage(steady, "server_ms", "event"),
        "avg_usb_out_ms": (
            sum(sum_field(g, "usb_out_us") for g in steady) / len(steady) / 1000.0
            if steady else 0.0),
        "avg_usb_in_ms": (
            sum(sum_field(g, "usb_in_us") for g in steady) / len(steady) / 1000.0
            if steady else 0.0),
        "avg_usb_event_ms": (
            sum(sum_field(g, "usb_event_us") for g in steady) / len(steady) / 1000.0
            if steady else 0.0),
    }

    return {
        "command_count": len(entries),
        "invoke_count": len(invokes),
        "measured_wall_ms": measured_ms,
        "measured_fps_including_first": (
            len(invokes) * 1000.0 / measured_ms if measured_ms else 0.0),
        "setup": setup_summary,
        "first_invoke": first_summary,
        "steady": steady_summary,
        "usb_failed": sum_field(entries, "usb_failed"),
        "usb_timeouts": sum_field(entries, "usb_timeouts"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=sorted(TARGETS), default="idle")
    args = parser.parse_args()
    cfg = TARGETS[args.target]

    iter_dir = next_iter_dir(cfg["iter_suffix"])
    snapshots = iter_dir / "renode"
    snapshots.mkdir()

    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    configure_cmd = [
        "cmake",
        "-S",
        ".",
        "-B",
        str(BUILD_DIR.relative_to(ROOT)),
        "-DSENTAI_ARM_EMU=ON",
        "-DSENTAI_SKIP_SDK_PATCHES=ON",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(BUILD_DIR.relative_to(ROOT)),
        "--target",
        cfg["cmake_target"],
        f"-j{subprocess.os.cpu_count() or 1}",
    ]
    sim_configure_cmd = [
        "cmake",
        "-S",
        ".",
        "-B",
        "build-sim",
        "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake",
        "-DSENTAI_SIM=ON",
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    sim_build_cmd = [
        "cmake",
        "--build",
        "build-sim",
        "--target",
        cfg.get("sim_cmake_target", ""),
        f"-j{subprocess.os.cpu_count() or 1}",
    ]
    renode_cmd = [
        str(RENODE),
        "--plain",
        "--console",
        "--disable-xwt",
        str(cfg["renode_script"].relative_to(ROOT)),
    ]

    logs: dict[str, subprocess.CompletedProcess[str]] = {}

    if cfg.get("pre_hook") == "prepare_cat_scenes":
        # Regenerate emu/output/scenes/*.bin from the B7 reference cat
        # before Renode tries to LoadBinary them.  Deterministic; no
        # randomness, so re-running is a no-op other than file mtimes.
        prep_cmd = [
            sys.executable,
            str((ROOT / "emu" / "scripts" / "prepare_cat_scenes.py").relative_to(ROOT)),
        ]
        prep_result = run_cmd(prep_cmd, ROOT)
        logs["pre_hook"] = prep_result
        (iter_dir / "pre_hook.log").write_text(prep_result.stdout)
        if prep_result.returncode != 0:
            print(prep_result.stdout)
            return 1
    elif cfg.get("pre_hook") in ("init_emu_nand", "ensure_emu_nand"):
        prep_cmd = [
            sys.executable,
            str((ROOT / "emu" / "scripts" / "init_emu_nand.py").relative_to(ROOT)),
        ]
        if cfg.get("pre_hook") == "init_emu_nand":
            prep_cmd.append("--reset")
        prep_result = run_cmd(prep_cmd, ROOT)
        logs["pre_hook"] = prep_result
        (iter_dir / "pre_hook.log").write_text(prep_result.stdout)
        if prep_result.returncode != 0:
            print(prep_result.stdout)
            return 1

    cmd_sequence = [
        ("configure", configure_cmd),
        ("build", build_cmd),
    ]
    if cfg.get("sim_cmake_target"):
        cmd_sequence.extend([
            ("configure_sim", sim_configure_cmd),
            ("build_sim", sim_build_cmd),
        ])
    cmd_sequence.append(("renode", renode_cmd))

    for name, cmd in cmd_sequence:
        if name == "renode":
            result = run_cmd_to_file(
                cmd, ROOT, iter_dir / f"{name}.log",
                int(cfg.get("renode_timeout_s", 120)))
        else:
            result = run_cmd(cmd, ROOT)
        logs[name] = result
        (iter_dir / f"{name}.log").write_text(result.stdout)
        if result.returncode != 0:
            break

    renode_log = logs.get("renode").stdout if "renode" in logs else ""
    renode_label = cfg["cmake_target"]
    boot_state = parse_hex(f"{renode_label} boot_state", renode_log)
    heartbeat = parse_hex(f"{renode_label} heartbeat", renode_log)
    last_tick = parse_hex(f"{renode_label} last_tick", renode_log)
    repl_lines = parse_hex(f"{renode_label} repl_lines", renode_log)
    frames_consumed = parse_hex(f"{renode_label} frames_consumed", renode_log)
    frames_valid = parse_hex(f"{renode_label} frames_valid", renode_log)
    irq_count = parse_hex(f"{renode_label} irq_count", renode_log)
    stage1_processed = parse_hex(f"{renode_label} stage1_processed", renode_log)
    stage2_consumed = parse_hex(f"{renode_label} stage2_consumed", renode_log)
    stage2a_consumed = parse_hex(f"{renode_label} stage2a_consumed", renode_log)
    stage2b_consumed = parse_hex(f"{renode_label} stage2b_consumed", renode_log)
    seqlock_torn_reads = parse_hex(f"{renode_label} seqlock_torn_reads", renode_log)
    pipeline_errors = parse_hex(f"{renode_label} pipeline_errors", renode_log)
    last_sum = parse_hex(f"{renode_label} last_sum", renode_log)
    # B8.7c flowest: signed dx/dy can be negative; Renode echoes them as
    # 32-bit two's complement hex.  parse_hex returns the raw unsigned int;
    # we convert to signed below.
    last_dx_raw = parse_hex(f"{renode_label} last_dx", renode_log)
    last_dy_raw = parse_hex(f"{renode_label} last_dy", renode_log)
    last_sad = parse_hex(f"{renode_label} last_sad", renode_log)
    usbhost_step = parse_hex(f"{renode_label} step", renode_log)
    tpu_bridge_params_calls = parse_hex(f"{renode_label} tpu_bridge_params_calls", renode_log)
    tpu_bridge_input_calls = parse_hex(f"{renode_label} tpu_bridge_input_calls", renode_log)
    tpu_bridge_ins_calls = parse_hex(f"{renode_label} tpu_bridge_ins_calls", renode_log)
    tpu_bridge_output_calls = parse_hex(f"{renode_label} tpu_bridge_output_calls", renode_log)
    tpu_bridge_event_calls = parse_hex(f"{renode_label} tpu_bridge_event_calls", renode_log)
    tpu_bridge_last_result = parse_hex(f"{renode_label} tpu_bridge_last_result", renode_log)
    tpu_bridge_output_sum = parse_hex(f"{renode_label} tpu_bridge_output_sum", renode_log)
    fs_size = parse_hex(f"{renode_label} fs_size", renode_log)
    fs_read_ok = parse_hex(f"{renode_label} fs_read_ok", renode_log)
    fx_reads = parse_hex(f"{renode_label} fx_reads", renode_log)
    fx_writes = parse_hex(f"{renode_label} fx_writes", renode_log)
    fx_erases = parse_hex(f"{renode_label} fx_erases", renode_log)
    fx_errors = parse_hex(f"{renode_label} fx_errors", renode_log)
    fs_smoke_ok = parse_hex(f"{renode_label} fs_smoke_ok", renode_log)
    fs_smoke_size = parse_hex(f"{renode_label} fs_smoke_size", renode_log)
    stage_files = parse_hex(f"{renode_label} stage_files", renode_log)
    stage_bytes = parse_hex(f"{renode_label} stage_bytes", renode_log)
    stage_errors = parse_hex(f"{renode_label} stage_errors", renode_log)
    stage_checksum = parse_hex(f"{renode_label} stage_checksum", renode_log)
    tpu_physical_model_bytes = parse_hex(f"{renode_label} model_bytes", renode_log)
    tpu_physical_image_bytes = parse_hex(f"{renode_label} image_bytes", renode_log)
    tpu_physical_input_bytes = parse_hex(f"{renode_label} input_bytes", renode_log)
    tpu_physical_output_bytes = parse_hex(f"{renode_label} output_bytes", renode_log)
    tpu_physical_completed = parse_hex(f"{renode_label} completed", renode_log)
    tpu_physical_fail_code = parse_hex(f"{renode_label} fail_code", renode_log)
    tpu_physical_first_ms = parse_hex(f"{renode_label} first_ms", renode_log)
    tpu_physical_steady_ms = parse_hex(f"{renode_label} steady_ms", renode_log)
    tpu_physical_output_sum = parse_hex(f"{renode_label} output_sum", renode_log)

    def _u32_to_i32(v):
        if v is None:
            return None
        return v - 0x1_0000_0000 if v >= 0x8000_0000 else v

    last_dx = _u32_to_i32(last_dx_raw)
    last_dy = _u32_to_i32(last_dy_raw)

    if args.target == "repl":
        # REPL target reuses heartbeat as "completed mp_embed_exec_str count";
        # boot_state=0x500 marks the post-banner state.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "mission":
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and repl_lines is not None
            and repl_lines >= 2
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "camera":
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and frames_consumed == 5
            and frames_valid == 5
            and irq_count == 5
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "fanout":
        # B8.7: every IRQ must propagate to Stage1Task and then BOTH parallel
        # consumers (Stage2A + Stage2B).  seqlock_torn_reads is 0 on the
        # happy path; non-zero would indicate the reader retry loop was
        # exercised, which is OK but worth surfacing in the verdict.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0xA00
            and irq_count == 5
            and stage1_processed == 5
            and stage2a_consumed == 5
            and stage2b_consumed == 5
            and pipeline_errors == 0
            and last_sum == 320
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "flowest":
        # B8.7d: Stage1Task block-matcher must recover the per-frame motion
        # delta of a single cat photo panned along a varied 2D trajectory.
        # Frame 1 is prime (no prev) -> (0, 0); frames 2..6 each have a
        # known (dx, dy) computed from the absolute offsets in
        # emu/scripts/prepare_cat_scenes.py.  All deltas have sad=0 because
        # the per-frame shift is a clean translation of the same image.
        expected_dx_per_frame = [0, +2,  0, -3,  0, +4]
        expected_dy_per_frame = [0,  0, +2, -1, -3,  0]
        seen_lines = re.findall(
            rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
            (cfg["uart_log"].read_bytes() if cfg["uart_log"].exists() else b""),
        )
        detected_dx_per_frame = [int(ln[2]) for ln in seen_lines]
        detected_dy_per_frame = [int(ln[3]) for ln in seen_lines]
        detected_sad_per_frame = [int(ln[4]) for ln in seen_lines]
        flowest_dx_ok = detected_dx_per_frame == expected_dx_per_frame
        flowest_dy_ok = detected_dy_per_frame == expected_dy_per_frame
        flowest_sad_ok = all(v == 0 for v in detected_sad_per_frame)
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x900
            and irq_count == 6
            and stage1_processed == 6
            and stage2_consumed == 6
            and pipeline_errors == 0
            and flowest_dx_ok
            and flowest_dy_ok
            and flowest_sad_ok
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "usbhost_probe":
        # B8.8 P1 deliberately does not start the scheduler.  The target calls
        # the production UsbHostTask constructor only; success means the
        # constructor returned and the probe reached the expected spin marker.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0xCAFE
            and usbhost_step == 2
        )
    elif args.target == "usbhost_task":
        # B8.8 P2 starts the production UsbHostTask and the scheduler.  Success
        # means the USB host task did not starve a lower-priority heartbeat
        # while no USB device/model is present.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 5
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_manager_probe":
        # B8.8 P3a adds the production EdgeTpuManager singleton above P2.
        # Success means the manager layer links, constructs, and leaves the
        # scheduler healthy.  OpenDevice()/USB enumeration is still deferred.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 8
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_task_probe":
        # B8.8 P3b starts the production EdgeTpuTask QueueTask and lets it
        # register the Coral VID/PID callback with UsbHostTask.  No USB device
        # is attached; success means the task init path does not starve the
        # scheduler.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 11
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_opendevice_probe":
        # B8.8 P4a calls EdgeTpuManager::OpenDevice() from a FreeRTOS task
        # with the manager pre-marked as USB-error/no-device.  Success means
        # the high-level call returns nullptr and the scheduler stays healthy.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 13
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_synth_enum_probe":
        # B8.8 P4b injects synthetic Coral attach + enumeration-done events
        # through UsbHostTask::HostEvent, using NXP usb_host_* descriptor
        # structs.  This is not an EHCI model yet; it validates the callback
        # chain to EdgeTpuTask.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 16
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_mmio_enum_probe":
        # B8.8 P5a moves the Coral descriptor provider into Renode MMIO.  The
        # firmware-side probe consumes the descriptor block and routes the
        # event through the real UsbHostTask -> EdgeTpuTask callback chain.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 19
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_mmio_opendevice_probe":
        # B8.8 P5b consumes the Renode MMIO descriptor block, lets EdgeTpuTask
        # notify EdgeTpuManager of the connected class instance, then calls
        # OpenDevice() from a FreeRTOS task.  The emu-scoped TpuDriver stub
        # returns success; this validates sequencing, not real TPU transfers.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 22
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "edgetpu_mmio_send_bridge_probe":
        # B8.8 P6a keeps the injection point at the production TpuDriver
        # transfer boundary.  After P5b OpenDevice succeeds, firmware calls
        # SendParameters, SendInputs, SendInstructions, GetOutputs, ReadEvent.
        # Renode's `coral_tpu_bridge` MMIO peripheral acks each request and
        # writes a deterministic output pattern.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and usbhost_step == 30
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
            and tpu_bridge_params_calls == 1
            and tpu_bridge_input_calls == 1
            and tpu_bridge_ins_calls == 1
            and tpu_bridge_output_calls == 1
            and tpu_bridge_event_calls == 1
            and tpu_bridge_last_result == 0
            and tpu_bridge_output_sum == sum(range(0xA0, 0xB0))
        )
    elif args.target == "fx_storage":
        # B8.9: production FxUser* stack must format an emulated raw NAND,
        # write/sync/read a file, and report actual NAND-level traffic.  This
        # validates the filesystem building block without introducing a RAM
        # filesystem or bypassing FileX/LevelX.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x600
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
            and fs_size == 41
            and fs_read_ok == 1
            and fx_reads is not None
            and fx_reads > 0
            and fx_writes is not None
            and fx_writes > 0
            and fx_erases is not None
            and fx_erases > 0
            and fx_errors == 0
        )
    elif args.target == "fs_repl":
        # B8.9b: MicroPython `sentai.fs` must call the same FxUser* stack over
        # FileX/LevelX/NAND.  The smoke is run from inside the VM, matching the
        # eventual REPL path for uploading/loading models and images.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and heartbeat is not None
            and heartbeat >= 1
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
            and fs_smoke_ok == 1
            and fs_smoke_size == 11
        )
    elif args.target == "fs_stage_assets":
        # B8.9c: host assets are copied into the emulated user NAND before
        # the final firmware boot.  The bridge only supplies bytes; file
        # creation and FAT updates are performed by guest FxUser/FileX.
        # Re-runs are idempotent: if a file already exists at the expected
        # size, firmware logs FS_STAGE SKIP and does not rewrite its bytes.
        expected_bytes = (
            (ROOT / "models" / "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite").stat().st_size
            + (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
            + (EXP / "mission_s213_tpu_cat.py").stat().st_size
        )
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x700
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
            and stage_files == 3
            and stage_bytes is not None
            and 0 <= stage_bytes <= expected_bytes
            and stage_errors == 0
            and fx_reads is not None
            and fx_reads > 0
            and fx_writes is not None
            and fx_erases is not None
            and fx_errors == 0
        )
    elif args.target == "fs_asset_check":
        # B8.9d: boot a fresh REPL image over the already-staged NAND and read
        # model/image/mission paths through MicroPython sentai.fs.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and heartbeat is not None
            and heartbeat >= 1
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target in ("tpu_physical_send_smoke", "tpu_physical_send_fps"):
        # B8.11: the emulated guest owns EdgeTpuManager/EdgeTpuExecutable and
        # emits real TpuDriver SendParameters/SendInputs/SendInstructions/
        # GetOutputs/ReadEvent calls.  Renode forwards those calls to a host
        # POSIX/libusb server connected to the physical Coral.
        expected_runs = cfg.get("expected_tpu_physical_completed", 2)
        expected_model = (
            ROOT / "models" / "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
        ).stat().st_size
        expected_image = (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0xB00
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
            and tpu_physical_model_bytes == expected_model
            and tpu_physical_image_bytes == expected_image
            and tpu_physical_input_bytes is not None
            and tpu_physical_input_bytes > 0
            and tpu_physical_output_bytes is not None
            and tpu_physical_output_bytes > 0
            and tpu_physical_completed == expected_runs
            and tpu_physical_fail_code == 0
            and tpu_physical_first_ms is not None
            and tpu_physical_first_ms > 0
            and tpu_physical_steady_ms is not None
            and tpu_physical_steady_ms > 0
            and tpu_bridge_params_calls is not None
            and tpu_bridge_params_calls >= 1
            and tpu_bridge_input_calls is not None
            and tpu_bridge_input_calls >= expected_runs
            and tpu_bridge_ins_calls is not None
            and tpu_bridge_ins_calls >= expected_runs
            and tpu_bridge_output_calls is not None
            and tpu_bridge_output_calls >= expected_runs * 2
            and tpu_bridge_event_calls is not None
            and tpu_bridge_event_calls >= expected_runs
            and tpu_bridge_last_result == 0
            and tpu_physical_output_sum is not None
            and tpu_physical_output_sum != 0
        )
    elif args.target == "tpu_cat_repl":
        # B8.10: the REPL image imports /mission.py from FileX, then
        # sentai.tpu streams the staged model/image bytes to the host bridge
        # and invokes the physical USB Coral through the existing C++ libusb
        # smoke.  The mission is autorun after MP+FS init to avoid brittle
        # UART prompt timing in Renode.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and heartbeat is not None
            and heartbeat >= 1
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "tpu_fps_repl":
        # B8.10b: same guest-FS + physical-Coral path as tpu_cat_repl, but
        # mission.fps() measures steady-state invokes after load() and one
        # warmup run.  Filesystem staging/model/image upload are deliberately
        # outside the FPS window.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and heartbeat is not None
            and heartbeat >= 1
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target in (
        "tpu_fps_mem_repl",
        "tpu_fps_mem_invoke_repl",
        "tpu_fps_mem_session_repl",
        "tpu_timing_repl",
    ):
        # B8.10c: same physical-Coral benchmark, but the image is first read
        # from guest FileX into emulated SDRAM and only then streamed to the
        # bridge.  This mirrors a camera/prep producer that already has a
        # resident frame before inference starts.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and heartbeat is not None
            and heartbeat >= 1
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    elif args.target == "pipeline":
        # B8.6: every IRQ must propagate to Stage1Task and then Stage2Task.
        # last_sum = 5 * 64 = 320 = 0x140 proves the 5th frame body actually
        # reached the reduction stage.  The emu deliberately does NOT call its
        # tasks PrepTask / InferTask / FlowTask — those names belong to the
        # production code in detection_task.cc / flow_task.cc that does the
        # real PXP / quant / TPU / USADA8 work.  The emu only validates the
        # ISR -> task chain topology, not those algorithms.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x900
            and irq_count == 5
            and stage1_processed == 5
            and stage2_consumed == 5
            and pipeline_errors == 0
            and last_sum == 320
            and last_tick is not None
            and last_tick > 0
        )
    else:
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x300
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )

    for src in (
        ROOT / "emu" / "renode" / "sentai_rt1176.repl",
        cfg["renode_script"],
    ):
        shutil.copy2(src, snapshots / src.name)

    uart_log_text = ""
    uart_log_bytes = b""
    if cfg["uart_log"] and cfg["uart_log"].exists():
        uart_log_bytes = cfg["uart_log"].read_bytes()
        uart_log_text = uart_log_bytes.decode("utf-8", errors="replace")
        # Preserve raw CRLF bytes — read_text/write_text would translate them
        # and turn the REPL answer assertion below into a false negative.
        (iter_dir / "uart.log").write_bytes(uart_log_bytes)

    if args.target in (
        "tpu_cat_repl",
        "tpu_fps_repl",
        "tpu_fps_mem_repl",
        "tpu_fps_mem_invoke_repl",
        "tpu_fps_mem_session_repl",
        "tpu_timing_repl",
    ):
        bridge_log = pathlib.Path("/tmp/sentai_emu_tpu_bridge.log")
        if bridge_log.exists():
            shutil.copy2(bridge_log, iter_dir / "tpu_host_bridge.log")
    physical_host_timing = None
    if args.target in ("tpu_physical_send_smoke", "tpu_physical_send_fps"):
        bridge_log = pathlib.Path("/tmp/sentai_emu_tpu_send_bridge.log")
        if bridge_log.exists():
            bridge_dst = iter_dir / "tpu_send_bridge.log"
            shutil.copy2(bridge_log, bridge_dst)
            physical_host_timing = parse_physical_send_host_timing(
                bridge_dst.read_text(errors="replace"))
            (iter_dir / "host_timing_summary.json").write_text(
                json.dumps(physical_host_timing, indent=2) + "\n")

    def parse_uart_int(marker: bytes) -> int | None:
        match = re.search(re.escape(marker) + rb"\s+(\d+)", uart_log_bytes)
        if not match:
            return None
        return int(match.group(1))

    tpu_fps_completed = parse_uart_int(b"TPU_FPS_COMPLETED")
    tpu_fps_warmup = parse_uart_int(b"TPU_FPS_WARMUP")
    tpu_fps_measured_ms = parse_uart_int(b"TPU_FPS_MEASURED_MS")
    tpu_fps_x100 = parse_uart_int(b"TPU_FPS_X100")
    tpu_fps_invoke_ms_sum = parse_uart_int(b"TPU_FPS_INVOKE_MS_SUM")
    tpu_fps_mem_completed = parse_uart_int(b"TPU_FPS_MEM_COMPLETED")
    tpu_fps_mem_warmup = parse_uart_int(b"TPU_FPS_MEM_WARMUP")
    tpu_fps_mem_measured_ms = parse_uart_int(b"TPU_FPS_MEM_MEASURED_MS")
    tpu_fps_mem_x100 = parse_uart_int(b"TPU_FPS_MEM_X100")
    tpu_fps_mem_invoke_ms_sum = parse_uart_int(
        b"TPU_FPS_MEM_INVOKE_MS_SUM")
    tpu_fps_mem_loop_completed = parse_uart_int(b"TPU_FPS_MEM_LOOP_COMPLETED")
    tpu_fps_mem_loop_warmup = parse_uart_int(b"TPU_FPS_MEM_LOOP_WARMUP")
    tpu_fps_mem_loop_measured_ms = parse_uart_int(
        b"TPU_FPS_MEM_LOOP_MEASURED_MS")
    tpu_fps_mem_loop_x100 = parse_uart_int(b"TPU_FPS_MEM_LOOP_X100")
    tpu_fps_mem_loop_invoke_ms_sum = parse_uart_int(
        b"TPU_FPS_MEM_LOOP_INVOKE_MS_SUM")
    tpu_fps_mem_session_completed = parse_uart_int(
        b"TPU_FPS_MEM_SESSION_COMPLETED")
    tpu_fps_mem_session_warmup = parse_uart_int(
        b"TPU_FPS_MEM_SESSION_WARMUP")
    tpu_fps_mem_session_measured_ms = parse_uart_int(
        b"TPU_FPS_MEM_SESSION_MEASURED_MS")
    tpu_fps_mem_session_x100 = parse_uart_int(
        b"TPU_FPS_MEM_SESSION_X100")
    tpu_fps_mem_session_invoke_ms_sum = parse_uart_int(
        b"TPU_FPS_MEM_SESSION_INVOKE_MS_SUM")
    image_mem_size = parse_uart_int(b"IMAGE_MEM_SIZE")
    tpu_timing_image_fs_to_mem_ms = parse_uart_int(
        b"TPU_IMAGE_FS_TO_MEM_MS")
    tpu_timing_host_preload_image_ms = parse_uart_int(
        b"HOST_PRELOAD_IMAGE_MS")
    tpu_timing_start_ms = parse_uart_int(b"TPU_START_MS")
    tpu_timing_first_invoke_ms = parse_uart_int(b"TPU_FIRST_INVOKE")
    tpu_timing_first_invoke_repl_ms = parse_uart_int(
        b"TPU_FIRST_INVOKE_REPL_MS")
    tpu_timing_steady_completed = parse_uart_int(b"TPU_STEADY_COMPLETED")
    tpu_timing_steady_total_ms = parse_uart_int(b"TPU_STEADY_TOTAL_MS")
    tpu_timing_steady_fps_x100 = parse_uart_int(b"TPU_STEADY_FPS_X100")
    tpu_timing_steady_invoke_ms_sum = parse_uart_int(
        b"TPU_STEADY_INVOKE_MS_SUM")
    fr_events_size = parse_uart_int(b"FR_EVENTS_SIZE")
    fr_scalars_size = parse_uart_int(b"FR_SCALARS_SIZE")

    if args.target == "uart":
        passed = passed and b"SentAI EMU UART boot" in uart_log_bytes
        passed = passed and b"SentAI EMU UART heartbeat" in uart_log_bytes

    if args.target == "repl":
        # Inputs injected by the resc are `1+1\r`; the REPL prints the prompt
        # before, echoes the typed bytes, then prints `2` followed by another
        # prompt. Assert the value rather than the prompt to catch silent
        # parsing failures.
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"\r\n2\r\n" in uart_log_bytes

    if args.target == "mission":
        # Verdict for B8.4: mission.run() must reach LPUART6 TX with the marker
        # AND the arithmetic result (proves the function body executed, not
        # just the import-time print path).
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION OK from B8.4 5" in uart_log_bytes

    if args.target == "camera":
        # Verdict for B8.5: every triggered frame must show up in the UART log
        # with ok=1.  The last marker line proves the consumer task processed
        # the 5th frame after the 5th IRQ.
        passed = passed and b"Consumer ready" in uart_log_bytes
        passed = passed and b"FRAME 1 seq=1 byte=0x01 ok=1" in uart_log_bytes
        passed = passed and b"FRAME 5 seq=5 byte=0x05 ok=1" in uart_log_bytes

    if args.target == "fanout":
        # B8.7: assert all three tasks ready, and every frame produces a
        # matching pair of STAGE2A + STAGE2B marker lines in order.  The
        # arithmetic must agree across both consumers (proves the seqlock
        # delivers consistent reads).
        passed = passed and b"Stage1Task ready" in uart_log_bytes
        passed = passed and b"Stage2ATask ready" in uart_log_bytes
        passed = passed and b"Stage2BTask ready" in uart_log_bytes
        passed = passed and b"STAGE2A 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes
        passed = passed and b"STAGE2B 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes
        passed = passed and b"STAGE2A 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes
        passed = passed and b"STAGE2B 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes

    if args.target == "pipeline":
        # B8.6: assert Stage1Task and Stage2Task both reached "ready" state
        # AND the per-frame STAGE2 lines carry the exact sum/avg arithmetic.
        # The last STAGE2 line catches starvation/dropped-frame regressions.
        # Names are deliberately neutral so the verdict does not pretend the
        # emu scaffolding implements production PrepTask / FlowTask.
        passed = passed and b"Stage1Task ready" in uart_log_bytes
        passed = passed and b"Stage2Task ready" in uart_log_bytes
        passed = passed and b"STAGE2 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes
        passed = passed and b"STAGE2 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes

    if args.target == "usbhost_probe":
        passed = passed and b"SentAI EMU USBHOST PROBE B8.8 P1" in uart_log_bytes
        passed = passed and b"step 2: UsbHostTask constructor returned" in uart_log_bytes

    if args.target == "usbhost_task":
        passed = passed and b"SentAI EMU USBHOST PROBE B8.8 P1" in uart_log_bytes
        passed = passed and b"step 4: UsbHostTask::Init returned" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_manager_probe":
        passed = passed and b"step 7: EdgeTpuManager singleton returned" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_task_probe":
        passed = passed and b"step 10: EdgeTpuTask::Init returned" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_opendevice_probe":
        passed = passed and b"step 13: OpenDevice returned nullptr after error" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_synth_enum_probe":
        passed = passed and b"step 16: synthetic EdgeTPU enum done" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_mmio_enum_probe":
        passed = passed and b"step 19: Renode Coral MMIO enum done" in uart_log_bytes

    if args.target == "edgetpu_mmio_opendevice_probe":
        passed = passed and b"step 19: Renode Coral MMIO enum done" in uart_log_bytes
        passed = passed and b"step 22: OpenDevice returned context after MMIO enum" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "edgetpu_mmio_send_bridge_probe":
        passed = passed and b"step 19: Renode Coral MMIO enum done" in uart_log_bytes
        passed = passed and b"step 22: OpenDevice returned context after MMIO enum" in uart_log_bytes
        passed = passed and b"step 30: TpuDriver Send* MMIO bridge smoke passed" in uart_log_bytes
        passed = passed and b"heartbeat task online" in uart_log_bytes

    if args.target == "fx_storage":
        passed = passed and b"SentAI EMU FX storage boot" in uart_log_bytes
        passed = passed and b"FX_STORAGE PASS" in uart_log_bytes

    if args.target == "fs_repl":
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"FS_REPL_BEGIN" in uart_log_bytes
        passed = passed and b"FS_FORMAT True" in uart_log_bytes
        passed = passed and b"FS_MKDIR True" in uart_log_bytes
        passed = passed and b"FS_WRITE True" in uart_log_bytes
        passed = passed and b"FS_APPEND True" in uart_log_bytes
        passed = passed and b"FS_SYNC True" in uart_log_bytes
        passed = passed and b"FS_SIZE 11" in uart_log_bytes
        passed = passed and b"FS_READ hello world" in uart_log_bytes
        passed = passed and b"FS_EXISTS True" in uart_log_bytes
        passed = passed and b"FS_LS" in uart_log_bytes
        passed = passed and (
            b"('A.TXT', 1, 11)" in uart_log_bytes
            or b"('a.txt', 1, 11)" in uart_log_bytes
        )
        passed = passed and b"FS_REPL_DONE" in uart_log_bytes

    if args.target == "fs_stage_assets":
        passed = passed and b"SentAI EMU FS asset staging boot" in uart_log_bytes
        passed = passed and (
            b"FS_STAGE FILE index=0" in uart_log_bytes
            or b"FS_STAGE SKIP index=0" in uart_log_bytes
        )
        passed = passed and (
            b"FS_STAGE FILE index=1" in uart_log_bytes
            or b"FS_STAGE SKIP index=1" in uart_log_bytes
        )
        passed = passed and (
            b"FS_STAGE FILE index=2" in uart_log_bytes
            or b"FS_STAGE SKIP index=2" in uart_log_bytes
        )
        passed = passed and b"FS_STAGE PASS" in uart_log_bytes

    if args.target == "fs_asset_check":
        expected_model = str(
            (ROOT / "models" / "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite").stat().st_size
        ).encode()
        expected_image = str((ROOT / "test_data" / "cat_640x480.bmp").stat().st_size).encode()
        expected_mission = str((EXP / "mission_s213_tpu_cat.py").stat().st_size).encode()
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"FS_ASSET_CHECK_BEGIN" in uart_log_bytes
        passed = passed and b"MODEL_SIZE " + expected_model in uart_log_bytes
        passed = passed and b"IMAGE_SIZE " + expected_image in uart_log_bytes
        passed = passed and b"MISSION_SIZE " + expected_mission in uart_log_bytes
        passed = passed and b"MISSION_HEAD import sentai" in uart_log_bytes
        passed = passed and b"FS_ASSET_CHECK_DONE" in uart_log_bytes

    if args.target in ("tpu_physical_send_smoke", "tpu_physical_send_fps"):
        expected_runs = cfg.get("expected_tpu_physical_completed", 2)
        passed = passed and b"SentAI EMU TPU physical Send* smoke boot" in uart_log_bytes
        passed = passed and b"TPU_PHYSICAL_SEND BEGIN" in uart_log_bytes
        passed = passed and b"INVOKE 1 ms=" in uart_log_bytes
        passed = passed and f"INVOKE {expected_runs} ms=".encode() in uart_log_bytes
        passed = passed and b"BRIDGE_PARAMS_CALLS" in uart_log_bytes
        passed = passed and b"TPU_PHYSICAL_SEND PASS" in uart_log_bytes
        if args.target == "tpu_physical_send_fps":
            passed = (
                passed
                and physical_host_timing is not None
                and physical_host_timing.get("invoke_count") == expected_runs
                and physical_host_timing.get("measured_fps_including_first", 0) > 0
                and physical_host_timing.get("steady", {}).get("fps", 0) > 0
                and physical_host_timing.get("usb_failed") == 0
                and physical_host_timing.get("usb_timeouts") == 0
            )

    if args.target == "tpu_cat_repl":
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_CAT_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_READY True" in uart_log_bytes
        passed = passed and b"TPU_LOAD_IMAGE 0" in uart_log_bytes
        passed = passed and b"TPU_INVOKE " in uart_log_bytes
        passed = passed and b"TPU_OUTPUTS 2" in uart_log_bytes
        passed = passed and b"DETECTIONS_COUNT " in uart_log_bytes
        passed = passed and b"(16," in uart_log_bytes
        passed = passed and b"DETECTIONS_WRITTEN True" in uart_log_bytes
        passed = passed and b"MISSION_TPU_CAT_DONE" in uart_log_bytes

    if args.target == "tpu_fps_repl":
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_READY True" in uart_log_bytes
        passed = passed and b"TPU_LOAD_IMAGE 0" in uart_log_bytes
        passed = passed and b"TPU_FPS_RESULT" in uart_log_bytes
        passed = passed and tpu_fps_completed == 5
        passed = passed and tpu_fps_warmup == 1
        passed = passed and tpu_fps_x100 is not None
        passed = passed and tpu_fps_x100 > 0
        passed = passed and b"DETECTIONS_COUNT " in uart_log_bytes
        passed = passed and b"(16," in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_DONE" in uart_log_bytes

    if args.target == "tpu_fps_mem_repl":
        expected_image = (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_READY True" in uart_log_bytes
        passed = passed and b"TPU_LOAD_IMAGE_MEM 0" in uart_log_bytes
        passed = passed and image_mem_size == expected_image
        passed = passed and b"TPU_FPS_MEM_RESULT" in uart_log_bytes
        passed = passed and tpu_fps_mem_completed == 5
        passed = passed and tpu_fps_mem_warmup == 1
        passed = passed and tpu_fps_mem_x100 is not None
        passed = passed and tpu_fps_mem_x100 > 0
        passed = passed and b"DETECTIONS_COUNT " in uart_log_bytes
        passed = passed and b"(16," in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_DONE" in uart_log_bytes

    if args.target == "tpu_fps_mem_invoke_repl":
        expected_image = (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_LOOP_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_READY True" in uart_log_bytes
        passed = passed and b"TPU_LOAD_IMAGE_MEM 0" in uart_log_bytes
        passed = passed and image_mem_size == expected_image
        passed = passed and b"TPU_FPS_MEM_LOOP_RESULT" in uart_log_bytes
        passed = passed and tpu_fps_mem_loop_completed == 5
        passed = passed and tpu_fps_mem_loop_warmup == 1
        passed = passed and tpu_fps_mem_loop_x100 is not None
        passed = passed and tpu_fps_mem_loop_x100 > 0
        passed = passed and b"DETECTIONS_COUNT " in uart_log_bytes
        passed = passed and b"(16," in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_LOOP_DONE" in uart_log_bytes

    if args.target == "tpu_fps_mem_session_repl":
        expected_image = (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_SESSION_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_READY True" in uart_log_bytes
        passed = passed and b"TPU_LOAD_IMAGE_MEM 0" in uart_log_bytes
        passed = passed and image_mem_size == expected_image
        passed = passed and b"TPU_START 0" in uart_log_bytes
        passed = passed and b"TPU_FPS_MEM_SESSION_RESULT" in uart_log_bytes
        passed = passed and tpu_fps_mem_session_completed == 5
        passed = passed and tpu_fps_mem_session_warmup == 1
        passed = passed and tpu_fps_mem_session_x100 is not None
        passed = passed and tpu_fps_mem_session_x100 > 0
        passed = passed and b"DETECTIONS_COUNT " in uart_log_bytes
        passed = passed and b"(16," in uart_log_bytes
        passed = passed and b"TPU_STOP 0" in uart_log_bytes
        passed = passed and b"MISSION_TPU_FPS_MEM_SESSION_DONE" in uart_log_bytes

    if args.target == "tpu_timing_repl":
        expected_image = (ROOT / "test_data" / "cat_640x480.bmp").stat().st_size
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"MISSION_TPU_TIMING_BEGIN" in uart_log_bytes
        passed = passed and b"TPU_LOAD 0" in uart_log_bytes
        passed = passed and b"TPU_IMAGE_FS_TO_MEM 0" in uart_log_bytes
        passed = passed and image_mem_size == expected_image
        passed = passed and b"HOST_PRELOAD_IMAGE 0" in uart_log_bytes
        passed = passed and b"TPU_START 0" in uart_log_bytes
        passed = passed and b"TPU_FIRST_INVOKE " in uart_log_bytes
        passed = passed and b"TPU_STATS_FIRST params_calls 1" in uart_log_bytes
        passed = passed and b"TPU_STATS_STEADY_0 params_calls 0" in uart_log_bytes
        passed = passed and b"TPU_STEADY_COMPLETED 3" in uart_log_bytes
        passed = passed and tpu_timing_steady_fps_x100 is not None
        passed = passed and tpu_timing_steady_fps_x100 > 0
        passed = passed and b"FR_EVENTS_SIZE " in uart_log_bytes
        passed = passed and b"FR_SCALARS_SIZE " in uart_log_bytes
        passed = passed and fr_events_size is not None and fr_events_size > 0
        passed = passed and fr_scalars_size is not None and fr_scalars_size > 0
        passed = passed and b"FR_EVENTS_PATH /fr/events.csv" in uart_log_bytes
        passed = passed and b"FR_SCALARS_PATH /fr/scalars.csv" in uart_log_bytes
        passed = passed and b"TIMING_SUMMARY_PATH /tpu_timing_summary.txt" in uart_log_bytes
        passed = passed and b"TPU_STOP 0" in uart_log_bytes
        passed = passed and b"MISSION_TPU_TIMING_DONE" in uart_log_bytes

    verdict = {
        "experiment": "s213_arm_emulator_idle",
        "iter": iter_dir.name,
        "target_kind": args.target,
        "started_at": started_at,
        "commands": {
            "configure": configure_cmd,
            "build": build_cmd,
            "configure_sim": sim_configure_cmd if cfg.get("sim_cmake_target") else None,
            "build_sim": sim_build_cmd if cfg.get("sim_cmake_target") else None,
            "renode": renode_cmd,
        },
        "target": str((BUILD_DIR / "emu" / cfg["cmake_target"]).relative_to(ROOT)),
        "renode_script": str(cfg["renode_script"].relative_to(ROOT)),
        "returncodes": {name: result.returncode for name, result in logs.items()},
        "boot_state": boot_state,
        "heartbeat": heartbeat,
        "last_tick": last_tick,
        "repl_lines": repl_lines,
        "uart_log_contains_boot": b"SentAI EMU UART boot" in uart_log_bytes,
        "uart_log_contains_heartbeat": b"SentAI EMU UART heartbeat" in uart_log_bytes,
        "uart_log_contains_repl_banner": b"MicroPython embed ready" in uart_log_bytes,
        "uart_log_contains_repl_answer": b"\r\n2\r\n" in uart_log_bytes,
        "uart_log_contains_mission_marker": b"MISSION OK from B8.4 5" in uart_log_bytes,
        "frames_consumed": frames_consumed,
        "frames_valid": frames_valid,
        "irq_count": irq_count,
        "uart_log_contains_camera_first": b"FRAME 1 seq=1 byte=0x01 ok=1" in uart_log_bytes,
        "uart_log_contains_camera_last": b"FRAME 5 seq=5 byte=0x05 ok=1" in uart_log_bytes,
        "stage1_processed": stage1_processed,
        "stage2_consumed": stage2_consumed,
        "stage2a_consumed": stage2a_consumed,
        "stage2b_consumed": stage2b_consumed,
        "seqlock_torn_reads": seqlock_torn_reads,
        "pipeline_errors": pipeline_errors,
        "last_sum": last_sum,
        "uart_log_contains_pipeline_first": b"STAGE2 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes,
        "uart_log_contains_pipeline_last": b"STAGE2 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes,
        "uart_log_contains_fanout_first_a": b"STAGE2A 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes,
        "uart_log_contains_fanout_first_b": b"STAGE2B 1 frame_seq=1 sum=64 avg=1" in uart_log_bytes,
        "uart_log_contains_fanout_last_a": b"STAGE2A 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes,
        "uart_log_contains_fanout_last_b": b"STAGE2B 5 frame_seq=5 sum=320 avg=5" in uart_log_bytes,
        "last_dx": last_dx,
        "last_dy": last_dy,
        "last_sad": last_sad,
        "usbhost_step": usbhost_step,
        "tpu_bridge_params_calls": tpu_bridge_params_calls,
        "tpu_bridge_input_calls": tpu_bridge_input_calls,
        "tpu_bridge_ins_calls": tpu_bridge_ins_calls,
        "tpu_bridge_output_calls": tpu_bridge_output_calls,
        "tpu_bridge_event_calls": tpu_bridge_event_calls,
        "tpu_bridge_last_result": tpu_bridge_last_result,
        "tpu_bridge_output_sum": tpu_bridge_output_sum,
        "fs_size": fs_size,
        "fs_read_ok": fs_read_ok,
        "fx_reads": fx_reads,
        "fx_writes": fx_writes,
        "fx_erases": fx_erases,
        "fx_errors": fx_errors,
        "fs_smoke_ok": fs_smoke_ok,
        "fs_smoke_size": fs_smoke_size,
        "stage_files": stage_files,
        "stage_bytes": stage_bytes,
        "stage_errors": stage_errors,
        "stage_checksum": stage_checksum,
        "tpu_physical_model_bytes": tpu_physical_model_bytes,
        "tpu_physical_image_bytes": tpu_physical_image_bytes,
        "tpu_physical_input_bytes": tpu_physical_input_bytes,
        "tpu_physical_output_bytes": tpu_physical_output_bytes,
        "tpu_physical_completed": tpu_physical_completed,
        "tpu_physical_fail_code": tpu_physical_fail_code,
        "tpu_physical_first_ms": tpu_physical_first_ms,
        "tpu_physical_steady_ms": tpu_physical_steady_ms,
        "tpu_physical_output_sum": tpu_physical_output_sum,
        "tpu_physical_host_timing": physical_host_timing,
        "uart_log_contains_fx_storage_pass": b"FX_STORAGE PASS" in uart_log_bytes,
        "uart_log_contains_fs_repl_done": b"FS_REPL_DONE" in uart_log_bytes,
        "uart_log_contains_fs_stage_pass": b"FS_STAGE PASS" in uart_log_bytes,
        "uart_log_contains_fs_asset_check_done": b"FS_ASSET_CHECK_DONE" in uart_log_bytes,
        "uart_log_contains_tpu_physical_send_pass": (
            b"TPU_PHYSICAL_SEND PASS" in uart_log_bytes),
        "uart_log_contains_tpu_cat_done": b"MISSION_TPU_CAT_DONE" in uart_log_bytes,
        "uart_log_contains_tpu_fps_done": b"MISSION_TPU_FPS_DONE" in uart_log_bytes,
        "tpu_fps_completed": tpu_fps_completed,
        "tpu_fps_warmup": tpu_fps_warmup,
        "tpu_fps_measured_ms": tpu_fps_measured_ms,
        "tpu_fps_x100": tpu_fps_x100,
        "tpu_fps": (tpu_fps_x100 / 100.0 if tpu_fps_x100 is not None else None),
        "tpu_fps_invoke_ms_sum": tpu_fps_invoke_ms_sum,
        "tpu_fps_mem_completed": tpu_fps_mem_completed,
        "tpu_fps_mem_warmup": tpu_fps_mem_warmup,
        "tpu_fps_mem_measured_ms": tpu_fps_mem_measured_ms,
        "tpu_fps_mem_x100": tpu_fps_mem_x100,
        "tpu_fps_mem": (
            tpu_fps_mem_x100 / 100.0
            if tpu_fps_mem_x100 is not None else None),
        "tpu_fps_mem_invoke_ms_sum": tpu_fps_mem_invoke_ms_sum,
        "tpu_fps_mem_loop_completed": tpu_fps_mem_loop_completed,
        "tpu_fps_mem_loop_warmup": tpu_fps_mem_loop_warmup,
        "tpu_fps_mem_loop_measured_ms": tpu_fps_mem_loop_measured_ms,
        "tpu_fps_mem_loop_x100": tpu_fps_mem_loop_x100,
        "tpu_fps_mem_loop": (
            tpu_fps_mem_loop_x100 / 100.0
            if tpu_fps_mem_loop_x100 is not None else None),
        "tpu_fps_mem_loop_invoke_ms_sum": tpu_fps_mem_loop_invoke_ms_sum,
        "tpu_fps_mem_session_completed": tpu_fps_mem_session_completed,
        "tpu_fps_mem_session_warmup": tpu_fps_mem_session_warmup,
        "tpu_fps_mem_session_measured_ms": tpu_fps_mem_session_measured_ms,
        "tpu_fps_mem_session_x100": tpu_fps_mem_session_x100,
        "tpu_fps_mem_session": (
            tpu_fps_mem_session_x100 / 100.0
            if tpu_fps_mem_session_x100 is not None else None),
        "tpu_fps_mem_session_invoke_ms_sum": (
            tpu_fps_mem_session_invoke_ms_sum),
        "image_mem_size": image_mem_size,
        "tpu_timing_image_fs_to_mem_ms": tpu_timing_image_fs_to_mem_ms,
        "tpu_timing_host_preload_image_ms": (
            tpu_timing_host_preload_image_ms),
        "tpu_timing_start_ms": tpu_timing_start_ms,
        "tpu_timing_first_invoke_ms": tpu_timing_first_invoke_ms,
        "tpu_timing_first_invoke_repl_ms": (
            tpu_timing_first_invoke_repl_ms),
        "tpu_timing_steady_completed": tpu_timing_steady_completed,
        "tpu_timing_steady_total_ms": tpu_timing_steady_total_ms,
        "tpu_timing_steady_fps_x100": tpu_timing_steady_fps_x100,
        "tpu_timing_steady_fps": (
            tpu_timing_steady_fps_x100 / 100.0
            if tpu_timing_steady_fps_x100 is not None else None),
        "tpu_timing_steady_invoke_ms_sum": (
            tpu_timing_steady_invoke_ms_sum),
        "fr_events_size": fr_events_size,
        "fr_scalars_size": fr_scalars_size,
        "uart_log_contains_tpu_timing_done": (
            b"MISSION_TPU_TIMING_DONE" in uart_log_bytes),
        "uart_log_contains_fs_repl_read": (
            b"FS_READ hello world" in uart_log_bytes
        ),
        "uart_log_contains_usbhost_banner": (
            b"SentAI EMU USBHOST PROBE B8.8 P1" in uart_log_bytes
        ),
        "uart_log_contains_usbhost_ctor_returned": (
            b"step 2: UsbHostTask constructor returned" in uart_log_bytes
        ),
        "uart_log_contains_usbhost_init_returned": (
            b"step 4: UsbHostTask::Init returned" in uart_log_bytes
        ),
        "uart_log_contains_usbhost_heartbeat_online": (
            b"heartbeat task online" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_manager_returned": (
            b"step 7: EdgeTpuManager singleton returned" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_task_init_returned": (
            b"step 10: EdgeTpuTask::Init returned" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_opendevice_null": (
            b"step 13: OpenDevice returned nullptr after error" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_synth_enum_done": (
            b"step 16: synthetic EdgeTPU enum done" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_mmio_enum_done": (
            b"step 19: Renode Coral MMIO enum done" in uart_log_bytes
        ),
        "uart_log_contains_edgetpu_mmio_send_bridge_passed": (
            b"step 30: TpuDriver Send* MMIO bridge smoke passed" in uart_log_bytes
        ),
        "flowest_detected_dx_per_frame": (
            [int(ln[2]) for ln in re.findall(
                rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
                uart_log_bytes)] if args.target == "flowest" else None
        ),
        "flowest_detected_dy_per_frame": (
            [int(ln[3]) for ln in re.findall(
                rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
                uart_log_bytes)] if args.target == "flowest" else None
        ),
        "flowest_detected_sad_per_frame": (
            [int(ln[4]) for ln in re.findall(
                rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
                uart_log_bytes)] if args.target == "flowest" else None
        ),
        "flowest_expected_dx_per_frame": (
            [0, +2,  0, -3,  0, +4] if args.target == "flowest" else None
        ),
        "flowest_expected_dy_per_frame": (
            [0,  0, +2, -1, -3,  0] if args.target == "flowest" else None
        ),
        "pass": passed,
    }
    (iter_dir / "verdict_s213.json").write_text(json.dumps(verdict, indent=2) + "\n")

    print(json.dumps(verdict, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
