#!/usr/bin/env python3
"""Run S219: ARM-emulated sentai.crazy CPX bridge to CrazySim cf2."""

from __future__ import annotations

import argparse
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
BRIDGE_LOG = pathlib.Path("/tmp/sentai_emu_crazy_cpx_udp_bridge.log")
SITL_LOG = pathlib.Path("/tmp/respawn_sitl/sitl.log")
DEFAULT_WORLD = "sentai_whycon_small"
TARGETS = {
    "cxx": {
        "target": "sentai_emu_crazy_ping_smoke",
        "renode_script": ROOT / "emu/renode/sentai_emu_crazy_ping_smoke.resc",
        "renode_ui_script": (
            ROOT / "emu/renode/sentai_emu_crazy_ping_smoke_ui.resc"),
        "uart_log": ROOT / "emu/output/sentai_emu_crazy_ping_smoke.log",
        "symbol_prefix": "sentai_emu_crazy_ping_smoke",
    },
    "mp": {
        "target": "sentai_emu_crazy_repl",
        "renode_script": ROOT / "emu/renode/sentai_emu_crazy_repl.resc",
        "renode_ui_script": (
            ROOT / "emu/renode/sentai_emu_crazy_repl_ui.resc"),
        "uart_log": ROOT / "emu/output/sentai_emu_crazy_repl.log",
        "symbol_prefix": "sentai_emu_crazy_repl",
    },
}


def next_iter_dir(mode: str) -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    suffix = "renode_crazy_cf2_bridge"
    if mode == "mp":
        suffix = "renode_crazy_mp_cf2_bridge"
    out = EXP / f"iter{next_id:02d}_{suffix}"
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


def run_capture(cmd: list[str], cwd: pathlib.Path, timeout_s: int = 30) -> str:
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


def as_i32(value: int | None) -> int | None:
    if value is None:
        return None
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def parse_uart(text: str) -> dict[str, object]:
    out: dict[str, object] = {}
    for key in (
        "CRAZY_INIT_RC",
        "CRAZY_PING_MS",
        "CRAZY_RUNNING",
        "SERIAL_OPEN_CALLS",
        "SERIAL_WRITE_CALLS",
        "SERIAL_READ_CALLS",
        "SERIAL_BYTES_TX",
        "SERIAL_BYTES_RX",
        "SERIAL_LAST_RESULT",
    ):
        match = re.search(rf"{key}=(-?\d+)", text)
        if match:
            out[key.lower()] = int(match.group(1))
    out["stopped"] = "CRAZY_STOPPED" in text
    for key in ("CRAZY_MP_INIT", "CRAZY_MP_PING_MS", "CRAZY_MP_STOP"):
        match = re.search(rf"{key}\s+(-?\d+)", text)
        if match:
            out[key.lower()] = int(match.group(1))
    out["mp_done"] = "CRAZY_MP_DONE" in text
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
    parser.add_argument("--mode", choices=sorted(TARGETS), default="cxx",
                        help=("cxx = shared sentai_crazy.cc smoke harness; "
                              "mp = MicroPython sentai.crazy namespace "
                              "autorun target"))
    parser.add_argument("--world", default=DEFAULT_WORLD,
                        help=("CrazySim world basename. Defaults to "
                              f"{DEFAULT_WORLD}, the WhyCon/circle-marker "
                              "world used by marker missions."))
    parser.add_argument("--renode-ui", action="store_true",
                        help=("run Renode with its UI/analyzers enabled "
                              "(no --plain/--disable-xwt); useful for "
                              "watching UART live during interactive debug"))
    parser.add_argument("--no-sitl", action="store_true",
                        help="do not start cf2; useful to verify clean timeout")
    parser.add_argument("--keep-sitl", action="store_true",
                        help="leave cf2/Gazebo processes running after the test")
    args = parser.parse_args()
    target_cfg = TARGETS[args.mode]
    target = target_cfg["target"]
    renode_script = target_cfg["renode_script"]
    renode_run_script = (
        target_cfg["renode_ui_script"] if args.renode_ui else renode_script)
    uart_log = target_cfg["uart_log"]
    symbol_prefix = target_cfg["symbol_prefix"]

    iter_dir = next_iter_dir(args.mode)
    (iter_dir / "renode").mkdir()
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
            "--target", target, f"-j{subprocess.os.cpu_count() or 1}",
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
        "target": target,
        "mode": args.mode,
        "renode_script": str(renode_run_script.relative_to(ROOT)),
        "commands": commands,
        "no_sitl": args.no_sitl,
        "world": args.world,
        "renode_ui": args.renode_ui,
    }

    try:
        for name in ("py_compile", "protocol_test", "configure", "build"):
            proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log",
                               300)
            results[f"{name}_rc"] = proc.returncode
            if proc.returncode != 0:
                (iter_dir / "verdict_s219.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                print(proc.stdout)
                return proc.returncode

        if BRIDGE_LOG.exists():
            BRIDGE_LOG.unlink()
        if uart_log.exists():
            uart_log.unlink()

        if not args.no_sitl:
            proc = run_to_file(commands["respawn_sitl"], ROOT,
                               iter_dir / "respawn_sitl.log", 180)
            results["respawn_sitl_rc"] = proc.returncode
            maybe_copy(SITL_LOG, iter_dir / "sitl.log")
            if proc.returncode != 0:
                (iter_dir / "verdict_s219.json").write_text(
                    json.dumps(results, indent=2) + "\n", encoding="utf-8")
                print(proc.stdout)
                return proc.returncode

        ss_before = run_capture(["ss", "-lunp"], ROOT)
        (iter_dir / "ss_before_renode.log").write_text(ss_before,
                                                       encoding="utf-8")

        renode_proc = run_to_file(commands["renode"], ROOT,
                                  iter_dir / "renode.log", 120)
        results["renode_rc"] = renode_proc.returncode

        uart_text = ""
        if maybe_copy(uart_log, iter_dir / "uart.log"):
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

        labels = {
            "boot_state": f"{symbol_prefix} boot_state",
            "heartbeat": f"{symbol_prefix} heartbeat",
            "last_tick": f"{symbol_prefix} last_tick",
            "serial_open_calls": f"{symbol_prefix} serial_open_calls",
            "serial_write_calls": f"{symbol_prefix} serial_write_calls",
            "serial_read_calls": f"{symbol_prefix} serial_read_calls",
            "serial_bytes_tx": f"{symbol_prefix} serial_bytes_tx",
            "serial_bytes_rx": f"{symbol_prefix} serial_bytes_rx",
            "serial_last_result": f"{symbol_prefix} serial_last_result",
        }
        if args.mode == "cxx":
            labels["init_rc"] = f"{symbol_prefix} init_rc"
            labels["ping_ms"] = f"{symbol_prefix} ping_ms"
        else:
            labels["repl_lines"] = f"{symbol_prefix} repl_lines"
        symbols = {
            key: parse_hex(label, renode_proc.stdout)
            for key, label in labels.items()
        }
        if args.mode == "cxx":
            symbols["ping_ms_signed"] = as_i32(symbols["ping_ms"])
        results["symbols"] = symbols
        results["uart"] = parse_uart(uart_text)
        results["bridge"] = parse_bridge(bridge_text)
        if args.mode == "cxx":
            results["pass"] = (
                renode_proc.returncode == 0
                and symbols.get("boot_state") == 0x0B00
                and symbols.get("init_rc") == 0
                and isinstance(symbols.get("ping_ms_signed"), int)
                and symbols["ping_ms_signed"] >= 0
                and results["bridge"].get("guest_to_udp", 0) >= 1
                and results["bridge"].get("udp_to_guest", 0) >= 1
            )
        else:
            results["pass"] = (
                renode_proc.returncode == 0
                and symbols.get("boot_state") == 0x0500
                and symbols.get("repl_lines", 0) >= 1
                and results["uart"].get("crazy_mp_init") == 0
                and isinstance(results["uart"].get("crazy_mp_ping_ms"), int)
                and results["uart"]["crazy_mp_ping_ms"] >= 0
                and results["uart"].get("crazy_mp_stop") == 0
                and results["uart"].get("mp_done") is True
                and results["bridge"].get("guest_to_udp", 0) >= 1
                and results["bridge"].get("udp_to_guest", 0) >= 1
            )

        (iter_dir / "verdict_s219.json").write_text(
            json.dumps(results, indent=2) + "\n", encoding="utf-8")

        print(f"S219 iter: {iter_dir.relative_to(ROOT)}")
        ping_ms = (
            symbols.get("ping_ms_signed") if args.mode == "cxx"
            else results["uart"].get("crazy_mp_ping_ms"))
        print(f"pass={results['pass']} ping_ms={ping_ms} "
              f"tx={symbols.get('serial_bytes_tx')} "
              f"rx={symbols.get('serial_bytes_rx')}")
        return 0 if results["pass"] else 1
    finally:
        if not args.keep_sitl and not args.no_sitl:
            cleanup_sitl()


if __name__ == "__main__":
    raise SystemExit(main())
