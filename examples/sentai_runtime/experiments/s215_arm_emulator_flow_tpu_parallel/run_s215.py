#!/usr/bin/env python3
"""Run S215: FlowTask and physical Coral Send* path in parallel in ARM Renode."""

from __future__ import annotations

import json
import pathlib
import re
import shutil
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_EMU = ROOT / "build_emu"
BUILD_SIM = ROOT / "build-sim"
TARGET = "sentai_emu_flow_tpu_parallel"
STAGE_TARGET = "sentai_emu_fs_stage_assets"
SIM_TARGET = "tpu_posix_send_server"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_flow_tpu_parallel.resc"
STAGE_SCRIPT = ROOT / "emu/renode/sentai_emu_fs_stage_assets.resc"
UART_LOG = ROOT / "emu/output/sentai_emu_flow_tpu_parallel.log"
STAGE_UART_LOG = ROOT / "emu/output/sentai_emu_fs_stage_assets.log"
BRIDGE_LOG = pathlib.Path("/tmp/sentai_emu_tpu_send_bridge.log")


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_renode_flow_tpu_parallel_filex_coral"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
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
            log.write(f"\n[TIMEOUT] exceeded {timeout_s}s\n")
        log.seek(0)
        out = log.read()
    return subprocess.CompletedProcess(cmd, rc, out)


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    if not match:
        return None
    return int(match.group(1), 16)


def as_i32(value: int | None) -> int | None:
    if value is None:
        return None
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def parse_uart(text: str) -> dict:
    out: dict[str, object] = {
        "flow_validate_pass": "FLOW_VALIDATE_PASS" in text,
        "flow_done": "FLOW_DONE" in text,
        "tpu_pass": "TPU_PHYSICAL_SEND PASS" in text,
    }
    fps_re = re.search(
        r"FLOW_FPS frames=(?P<frames>\d+) elapsed_ms=(?P<elapsed>\d+) "
        r"fps_x100=(?P<fps>\d+) last_dx_q1000=(?P<dx>-?\d+) "
        r"last_dy_q1000=(?P<dy>-?\d+) conf=(?P<conf>\d+)",
        text,
    )
    if fps_re:
        out["flow_fps"] = {
            "frames": int(fps_re.group("frames")),
            "elapsed_ms": int(fps_re.group("elapsed")),
            "fps_x100": int(fps_re.group("fps")),
            "fps": int(fps_re.group("fps")) / 100.0,
            "last_dx_q1000": int(fps_re.group("dx")),
            "last_dy_q1000": int(fps_re.group("dy")),
            "last_conf": int(fps_re.group("conf")),
        }
    invoke_ms = [int(m.group(2)) for m in re.finditer(r"INVOKE (\d+) ms=(\d+)", text)]
    out["tpu_invoke_ms"] = invoke_ms
    return out


def parse_physical_send_host_timing(text: str) -> dict:
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

    steady = invokes[1:]
    measured_ms = 0
    if invokes:
        measured_ms = max(e["bridge_end_ms"] for g in invokes for e in g) - min(
            e["bridge_start_ms"] for g in invokes for e in g)
    steady_ms = 0
    if steady:
        steady_ms = max(e["bridge_end_ms"] for g in steady for e in g) - min(
            e["bridge_start_ms"] for g in steady for e in g)
    return {
        "command_count": len(entries),
        "invoke_count": len(invokes),
        "measured_wall_ms": measured_ms,
        "measured_fps_including_first": (
            len(invokes) * 1000.0 / measured_ms if measured_ms else 0.0),
        "first_invoke_wall_ms": wall_ms(invokes[0]) if invokes else 0,
        "steady": {
            "count": len(steady),
            "wall_ms": steady_ms,
            "fps": (len(steady) * 1000.0 / steady_ms) if steady_ms else 0.0,
            "avg_wall_ms": (
                sum(wall_ms(g) for g in steady) / len(steady) if steady else 0.0),
            "avg_ins_bridge_ms": (
                sum(sum_field(g, "bridge_ms", "ins") for g in steady) / len(steady)
                if steady else 0.0),
            "avg_inputs_bridge_ms": (
                sum(sum_field(g, "bridge_ms", "inputs") for g in steady) / len(steady)
                if steady else 0.0),
            "avg_output_bridge_ms": (
                sum(sum_field(g, "bridge_ms", "output") for g in steady) / len(steady)
                if steady else 0.0),
            "avg_event_bridge_ms": (
                sum(sum_field(g, "bridge_ms", "event") for g in steady) / len(steady)
                if steady else 0.0),
            "avg_usb_out_ms": (
                sum(sum_field(g, "usb_out_us") for g in steady) / len(steady) / 1000.0
                if steady else 0.0),
            "avg_usb_in_ms": (
                sum(sum_field(g, "usb_in_us") for g in steady) / len(steady) / 1000.0
                if steady else 0.0),
            "avg_usb_event_ms": (
                sum(sum_field(g, "usb_event_us") for g in steady) / len(steady) / 1000.0
                if steady else 0.0),
        },
        "usb_failed": sum_field(entries, "usb_failed"),
        "usb_timeouts": sum_field(entries, "usb_timeouts"),
    }


def maybe_copy(src: pathlib.Path, dst: pathlib.Path) -> bool:
    if not src.exists():
        return False
    shutil.copy2(src, dst)
    return True


def main() -> int:
    iter_dir = next_iter_dir()
    (iter_dir / "renode").mkdir()
    (iter_dir / "assets").mkdir()

    commands = {
        "ensure_nand": [
            sys.executable,
            str((ROOT / "emu/scripts/init_emu_nand.py").relative_to(ROOT)),
        ],
        "configure_emu": [
            "cmake", "-S", ".", "-B", str(BUILD_EMU.relative_to(ROOT)),
            "-DSENTAI_ARM_EMU=ON", "-DSENTAI_SKIP_SDK_PATCHES=ON",
        ],
        "build_stage": [
            "cmake", "--build", str(BUILD_EMU.relative_to(ROOT)),
            "--target", STAGE_TARGET, f"-j{subprocess.os.cpu_count() or 1}",
        ],
        "build_target": [
            "cmake", "--build", str(BUILD_EMU.relative_to(ROOT)),
            "--target", TARGET, f"-j{subprocess.os.cpu_count() or 1}",
        ],
        "configure_sim": [
            "cmake", "-S", ".", "-B", str(BUILD_SIM.relative_to(ROOT)),
            "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake",
            "-DSENTAI_SIM=ON", "-DCMAKE_BUILD_TYPE=Release",
        ],
        "build_sim": [
            "cmake", "--build", str(BUILD_SIM.relative_to(ROOT)),
            "--target", SIM_TARGET, f"-j{subprocess.os.cpu_count() or 1}",
        ],
        "prepare_flow_asset": [
            sys.executable,
            str((ROOT / "examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/prepare_cat80.py").relative_to(ROOT)),
        ],
        "stage_assets": [
            str(RENODE), "--plain", "--console", "--disable-xwt",
            str(STAGE_SCRIPT.relative_to(ROOT)),
        ],
        "renode": [
            str(RENODE), "--plain", "--console", "--disable-xwt",
            str(RENODE_SCRIPT.relative_to(ROOT)),
        ],
    }

    results: dict[str, object] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "target": TARGET,
        "commands": commands,
    }

    for name in ("ensure_nand", "configure_emu", "build_stage", "build_target",
                 "configure_sim", "build_sim"):
        proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log", 300)
        results[f"{name}_rc"] = proc.returncode
        if proc.returncode != 0:
            (iter_dir / "verdict_s215.json").write_text(
                json.dumps(results, indent=2) + "\n")
            print(proc.stdout)
            return proc.returncode

    flow_asset = ROOT / "examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/assets/cat80_base.bin"
    if not flow_asset.exists():
        proc = run_to_file(commands["prepare_flow_asset"], ROOT,
                           iter_dir / "prepare_flow_asset.log", 60)
        results["prepare_flow_asset_rc"] = proc.returncode
        if proc.returncode != 0:
            (iter_dir / "verdict_s215.json").write_text(
                json.dumps(results, indent=2) + "\n")
            print(proc.stdout)
            return proc.returncode

    stage_proc = run_to_file(commands["stage_assets"], ROOT,
                             iter_dir / "stage_assets.log", 240)
    results["stage_assets_rc"] = stage_proc.returncode
    stage_errors = parse_hex("sentai_emu_fs_stage_assets stage_errors",
                             stage_proc.stdout)
    stage_files = parse_hex("sentai_emu_fs_stage_assets stage_files",
                            stage_proc.stdout)
    if stage_proc.returncode != 0 or stage_errors != 0 or stage_files != 3:
        reset_cmd = commands["ensure_nand"] + ["--reset"]
        reset_proc = run_to_file(reset_cmd, ROOT, iter_dir / "reset_nand.log", 120)
        results["reset_nand_rc"] = reset_proc.returncode
        stage_proc = run_to_file(commands["stage_assets"], ROOT,
                                 iter_dir / "stage_assets_retry.log", 240)
        results["stage_assets_retry_rc"] = stage_proc.returncode
        stage_errors = parse_hex("sentai_emu_fs_stage_assets stage_errors",
                                 stage_proc.stdout)
        stage_files = parse_hex("sentai_emu_fs_stage_assets stage_files",
                                stage_proc.stdout)
    results["stage_symbols"] = {
        "stage_files": stage_files,
        "stage_errors": stage_errors,
        "stage_bytes": parse_hex("sentai_emu_fs_stage_assets stage_bytes",
                                 stage_proc.stdout),
        "fx_errors": parse_hex("sentai_emu_fs_stage_assets fx_errors",
                               stage_proc.stdout),
    }

    if maybe_copy(STAGE_UART_LOG, iter_dir / "stage_uart.log"):
        results["stage_uart_copied"] = True

    for path in (UART_LOG, BRIDGE_LOG):
        if path.exists():
            path.unlink()

    env = {
        **subprocess.os.environ,
        "SENTAI_TPU_SEND_PERF": "low",
        "SENTAI_TPU_SEND_CHUNK_SIZE": str(1024 * 1024),
        "SENTAI_TPU_SEND_BULKIN_CHUNK_SIZE": "1024",
        "SENTAI_TPU_SEND_OUTFEED_CHUNK_LENGTH": "0x80",
    }
    with (iter_dir / "renode.log").open("w+", encoding="utf-8") as log:
        proc = subprocess.Popen(
            commands["renode"],
            cwd=str(ROOT),
            env=env,
            text=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            rc = proc.wait(timeout=420)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                rc = proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                rc = proc.wait()
            log.write("\n[TIMEOUT] Renode exceeded 420s\n")
        log.seek(0)
        renode_text = log.read()
    results["renode_rc"] = rc

    uart_text = ""
    if maybe_copy(UART_LOG, iter_dir / "uart.log"):
        uart_text = (iter_dir / "uart.log").read_text(
            encoding="utf-8", errors="replace")
    else:
        results["uart_log_missing"] = True
    if maybe_copy(BRIDGE_LOG, iter_dir / "tpu_send_bridge.log"):
        bridge_text = (iter_dir / "tpu_send_bridge.log").read_text(
            encoding="utf-8", errors="replace")
        results["physical_host_timing"] = parse_physical_send_host_timing(
            bridge_text)
    else:
        results["bridge_log_missing"] = True

    for src in (
        RENODE_SCRIPT,
        STAGE_SCRIPT,
        ROOT / "emu/renode/sentai_rt1176.repl",
        ROOT / "emu/renode/tpu_send_physical_bridge.py",
    ):
        maybe_copy(src, iter_dir / "renode" / src.name)
    for src in (
        ROOT / "examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/assets/cat80_base.bin",
        ROOT / "examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/assets/cat80_base.png",
    ):
        maybe_copy(src, iter_dir / "assets" / src.name)

    label = TARGET
    symbols = {
        "boot_state": parse_hex(f"{label} boot_state", renode_text),
        "heartbeat": parse_hex(f"{label} heartbeat", renode_text),
        "last_tick": parse_hex(f"{label} last_tick", renode_text),
        "parallel_done": parse_hex(f"{label} parallel_done", renode_text),
        "tpu_completed": parse_hex(f"{label} tpu_completed", renode_text),
        "tpu_fail_code": parse_hex(f"{label} tpu_fail_code", renode_text),
        "tpu_first_ms": parse_hex(f"{label} tpu_first_ms", renode_text),
        "tpu_steady_ms": parse_hex(f"{label} tpu_steady_ms", renode_text),
        "tpu_output_sum": parse_hex(f"{label} tpu_output_sum", renode_text),
        "tpu_bridge_last_result": parse_hex(
            f"{label} tpu_bridge_last_result", renode_text),
        "flow_completed": parse_hex(f"{label} flow_completed", renode_text),
        "flow_detect_ok": parse_hex(f"{label} flow_detect_ok", renode_text),
        "flow_fail_code": parse_hex(f"{label} flow_fail_code", renode_text),
        "flow_elapsed_ms": parse_hex(f"{label} flow_elapsed_ms", renode_text),
        "flow_fps_x100": parse_hex(f"{label} flow_fps_x100", renode_text),
        "flow_last_dx_q1000": parse_hex(
            f"{label} flow_last_dx_q1000", renode_text),
        "flow_last_dy_q1000": parse_hex(
            f"{label} flow_last_dy_q1000", renode_text),
        "flow_last_conf": parse_hex(f"{label} flow_last_conf", renode_text),
    }
    symbols["flow_last_dx_q1000_signed"] = as_i32(symbols["flow_last_dx_q1000"])
    symbols["flow_last_dy_q1000_signed"] = as_i32(symbols["flow_last_dy_q1000"])
    results["symbols"] = symbols
    results["uart"] = parse_uart(uart_text)

    host_timing = results.get("physical_host_timing")
    usb_ok = isinstance(host_timing, dict) and host_timing.get("usb_failed") == 0
    usb_ok = usb_ok and isinstance(host_timing, dict) and host_timing.get("usb_timeouts") == 0
    results["pass"] = (
        rc == 0
        and results["stage_symbols"]["stage_errors"] == 0
        and results["stage_symbols"]["stage_files"] == 3
        and symbols["boot_state"] == 0x0B00
        and symbols["parallel_done"] == 1
        and symbols["tpu_completed"] == 5
        and symbols["tpu_fail_code"] == 0
        and symbols["tpu_bridge_last_result"] == 0
        and symbols["tpu_output_sum"] not in (None, 0)
        and symbols["flow_completed"] == 24
        and symbols["flow_detect_ok"] == 1
        and symbols["flow_fail_code"] == 0
        and results["uart"]["flow_validate_pass"] is True
        and results["uart"]["flow_done"] is True
        and results["uart"]["tpu_pass"] is True
        and usb_ok
    )

    (iter_dir / "verdict_s215.json").write_text(
        json.dumps(results, indent=2) + "\n")

    flow_fps = results["uart"].get("flow_fps", {}).get("fps")
    host_fps = None
    if isinstance(host_timing, dict):
        host_fps = host_timing.get("steady", {}).get("fps")
    print(f"S215 iter: {iter_dir.relative_to(ROOT)}")
    print(f"pass={results['pass']} flow_fps={flow_fps} "
          f"tpu_steady_host_fps={host_fps} "
          f"parallel_done={symbols['parallel_done']} "
          f"tpu_completed={symbols['tpu_completed']} "
          f"flow_completed={symbols['flow_completed']}")
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
