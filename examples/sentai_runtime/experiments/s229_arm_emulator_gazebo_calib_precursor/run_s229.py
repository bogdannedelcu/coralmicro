#!/usr/bin/env python3
"""Run S229: ARM emulator + Gazebo WhyCon calib precursor."""

from __future__ import annotations

import argparse
import json
import os
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
TARGET = "sentai_emu_crazy_calib_precursor_repl"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_crazy_calib_precursor.resc"
RENODE_UI_SCRIPT = (
    ROOT / "emu/renode/sentai_emu_crazy_calib_precursor_ui.resc")
UART_LOG = ROOT / "emu/output/sentai_emu_crazy_calib_precursor.log"
BRIDGE_LOG = pathlib.Path("/tmp/sentai_emu_crazy_cpx_udp_bridge.log")
SITL_LOG = pathlib.Path("/tmp/respawn_sitl/sitl.log")
DEFAULT_WORLD = "sentai_whycon_small"
SYMBOL_PREFIX = "sentai_emu_crazy_calib_precursor"


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_gazebo_whycon_calib_precursor"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
    started = time.monotonic()
    with log_path.open("w+", encoding="utf-8") as log:
        log.write("$ " + " ".join(cmd) + "\n")
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
        cmd,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    return proc.stdout


def cleanup_sitl() -> None:
    distrobox_cleanup_cmds = [
        ["distrobox", "enter", "crazysim-garden", "--", "pkill", "-9", "-f",
         "gz_to_uds_bridge"],
        ["distrobox", "enter", "crazysim-garden", "--", "pkill", "-9", "-f",
         "gz sim"],
        ["distrobox", "enter", "crazysim-garden", "--", "pkill", "-9", "-f",
         "sitl_make"],
        ["distrobox", "enter", "crazysim-garden", "--", "pkill", "-9",
         "Xvfb"],
    ]
    host_patterns = [
        "sim/scripts/launch_hybrid_cf2.sh",
        "gz_to_uds_bridge",
        "gz sim",
        "sitl_make/build/cf2",
        "Xvfb :99",
    ]
    for cmd in distrobox_cleanup_cmds:
        try:
            subprocess.run(
                cmd, cwd=str(ROOT), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, check=False, timeout=15)
        except Exception:
            pass
    for pattern in host_patterns:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", pattern], cwd=str(ROOT),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=False, timeout=10)
        except Exception:
            pass


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    if not match:
        return None
    return int(match.group(1), 16)


def parse_uart(text: str) -> dict[str, object]:
    out: dict[str, object] = {
        "done": "CALIB_GZ_DONE" in text,
        "calib_stub_seen": "CALIB_GZ_CALIB_STUB" in text,
        "calib_error_seen": "CALIB_GZ_CALIB_ERR" in text,
    }
    patterns = {
        "init_rc": r"CALIB_GZ_INIT\s+(-?\d+)",
        "ping_ms": r"CALIB_GZ_PING_MS\s+(-?\d+)",
        "fly_rc": r"CALIB_GZ_FLY_RC\s+(-?\d+)",
        "stop_rc": r"CALIB_GZ_STOP\s+(-?\d+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if match:
            out[key] = int(match.group(1))
    match = re.search(r"CALIB_GZ_ALT_AFTER\s+(-?[0-9.]+)", text)
    if match:
        try:
            out["alt_after"] = float(match.group(1))
        except ValueError:
            pass
    return out


def parse_bridge(text: str) -> dict[str, object]:
    return {
        "guest_to_udp": len(re.findall(r"guest->udp crtp_len=", text)),
        "udp_to_guest": len(re.findall(r"udp->guest crtp_len=", text)),
        "system_messages": len(re.findall(r"guest system ", text)),
        "open_seen": " open" in text or text.startswith("open"),
        "close_seen": " close" in text or text.startswith("close"),
    }


def maybe_copy(src: pathlib.Path, dst: pathlib.Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=DEFAULT_WORLD,
                        help=("CrazySim world basename. Defaults to "
                              f"{DEFAULT_WORLD}, the WhyCon/circle-marker "
                              "world used by marker missions."))
    parser.add_argument("--renode-ui", action="store_true",
                        help=("run Renode with UI/analyzers enabled "
                              "(no --plain/--disable-xwt)"))
    parser.add_argument("--no-sitl", action="store_true",
                        help="do not start cf2; useful for failure debugging")
    parser.add_argument("--keep-sitl", action="store_true",
                        help="leave cf2/Gazebo processes running after the test")
    args = parser.parse_args()

    iter_dir = next_iter_dir()
    (iter_dir / "renode").mkdir()
    (iter_dir / "mission").mkdir()
    renode_run_script = RENODE_UI_SCRIPT if args.renode_ui else RENODE_SCRIPT
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    commands = {
        "py_compile": [
            sys.executable, "-m", "py_compile",
            "emu/host/sentai_crazy_cpx_udp_bridge.py",
            "emu/host/test_sentai_crazy_cpx_udp_bridge.py",
            "emu/renode/crazy_cpx_udp_mmio_bridge.py",
        ],
        "protocol_test": [
            sys.executable,
            "emu/host/test_sentai_crazy_cpx_udp_bridge.py",
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
        "started_at": started_at,
        "target": TARGET,
        "world": args.world,
        "renode_ui": args.renode_ui,
        "renode_script": str(renode_run_script.relative_to(ROOT)),
        "commands": commands,
    }

    try:
        for name in ("py_compile", "protocol_test", "configure", "build"):
            proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log",
                               300)
            results[f"{name}_rc"] = proc.returncode
            if proc.returncode != 0:
                (iter_dir / "verdict_s229.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                print(proc.stdout)
                return proc.returncode

        if BRIDGE_LOG.exists():
            BRIDGE_LOG.unlink()
        if UART_LOG.exists():
            UART_LOG.unlink()

        if not args.no_sitl:
            proc = run_to_file(commands["respawn_sitl"], ROOT,
                               iter_dir / "respawn_sitl.log", 180)
            results["respawn_sitl_rc"] = proc.returncode
            maybe_copy(SITL_LOG, iter_dir / "sitl.log")
            if proc.returncode != 0:
                (iter_dir / "verdict_s229.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                print(proc.stdout)
                return proc.returncode

        ss_before = run_capture(["ss", "-lunp"], ROOT)
        (iter_dir / "ss_before_renode.log").write_text(ss_before,
                                                       encoding="utf-8")

        renode_proc = run_to_file(commands["renode"], ROOT,
                                  iter_dir / "renode.log", 180)
        results["renode_rc"] = renode_proc.returncode

        uart_text = ""
        if maybe_copy(UART_LOG, iter_dir / "uart.log"):
            uart_text = (iter_dir / "uart.log").read_text(
                encoding="utf-8", errors="replace")
        else:
            results["uart_log_missing"] = True

        bridge_text = ""
        if maybe_copy(BRIDGE_LOG, iter_dir / "crazy_cpx_udp_bridge.log"):
            bridge_text = (iter_dir / "crazy_cpx_udp_bridge.log").read_text(
                encoding="utf-8", errors="replace")
        else:
            results["bridge_log_missing"] = True

        shutil.copy2(renode_run_script,
                     iter_dir / "renode" / renode_run_script.name)
        shutil.copy2(ROOT / "emu/renode/sentai_rt1176.repl",
                     iter_dir / "renode/sentai_rt1176.repl")
        shutil.copy2(ROOT / "emu/renode/crazy_cpx_udp_mmio_bridge.py",
                     iter_dir / "renode/crazy_cpx_udp_mmio_bridge.py")
        shutil.copy2(EXP / "mission_s229.py",
                     iter_dir / "mission/mission_s229.py")

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
        }
        symbols = {
            key: parse_hex(label, renode_proc.stdout)
            for key, label in labels.items()
        }
        results["symbols"] = symbols
        results["uart"] = parse_uart(uart_text)
        results["bridge"] = parse_bridge(bridge_text)

        uart = results["uart"]
        bridge = results["bridge"]
        results["pass"] = (
            renode_proc.returncode == 0
            and symbols.get("boot_state") == 0x0500
            and symbols.get("repl_lines", 0) >= 1
            and uart.get("init_rc") == 0
            and isinstance(uart.get("ping_ms"), int)
            and uart["ping_ms"] >= 0
            and uart.get("calib_stub_seen") is True
            and uart.get("calib_error_seen") is False
            and uart.get("fly_rc") == 0
            and uart.get("stop_rc") == 0
            and uart.get("done") is True
            and bridge.get("guest_to_udp", 0) >= 3
            and bridge.get("udp_to_guest", 0) >= 1
        )

        (iter_dir / "verdict_s229.json").write_text(
            json.dumps(results, indent=2) + "\n", encoding="utf-8")

        print(f"S229 iter: {iter_dir.relative_to(ROOT)}")
        print(
            f"pass={results['pass']} ping_ms={uart.get('ping_ms')} "
            f"fly_rc={uart.get('fly_rc')} alt_after={uart.get('alt_after')} "
            f"tx={symbols.get('serial_bytes_tx')} "
            f"rx={symbols.get('serial_bytes_rx')}")
        return 0 if results["pass"] else 1
    finally:
        if not args.keep_sitl and not args.no_sitl:
            cleanup_sitl()


if __name__ == "__main__":
    raise SystemExit(main())
