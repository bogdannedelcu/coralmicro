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
        "iter_suffix": "renode_flowest_cat_pan_1px",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_flowest.log",
        "pre_hook": "prepare_cat_scenes",
    },
}


def next_iter_dir(suffix: str) -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9][0-9]_*"))
    next_id = 1
    if existing:
        last = existing[-1].name.split("_", 1)[0]
        next_id = int(last.replace("iter", "")) + 1
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


def parse_hex(label: str, text: str) -> int | None:
    pattern = rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)"
    match = re.search(pattern, text)
    if not match:
        return None
    return int(match.group(1), 16)


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

    for name, cmd in (
        ("configure", configure_cmd),
        ("build", build_cmd),
        ("renode", renode_cmd),
    ):
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
        # B8.7c: Stage1Task block-matcher must report the expected per-frame
        # offset sequence for a single cat photo panned by 1 px / frame.
        # Frame 1 is prime (no prev) -> dx=0; frames 2..6 each detect dx=+1
        # with sad=0 (perfect match because the shift is a clean translation).
        expected_dx_per_frame = [0, 1, 1, 1, 1, 1]
        seen_lines = re.findall(
            rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
            (cfg["uart_log"].read_bytes() if cfg["uart_log"].exists() else b""),
        )
        detected_dx_per_frame = [int(ln[2]) for ln in seen_lines]
        detected_dy_per_frame = [int(ln[3]) for ln in seen_lines]
        flowest_dx_ok = detected_dx_per_frame == expected_dx_per_frame
        flowest_dy_ok = all(v == 0 for v in detected_dy_per_frame)
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x900
            and irq_count == 6
            and stage1_processed == 6
            and stage2_consumed == 6
            and pipeline_errors == 0
            and flowest_dx_ok
            and flowest_dy_ok
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

    verdict = {
        "experiment": "s213_arm_emulator_idle",
        "iter": iter_dir.name,
        "target_kind": args.target,
        "started_at": started_at,
        "commands": {
            "configure": configure_cmd,
            "build": build_cmd,
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
        "flowest_detected_dx_per_frame": (
            [int(ln[2]) for ln in re.findall(
                rb"FLOWEST (\d+) frame_seq=(\d+) dx=(-?\d+) dy=(-?\d+) sad=(\d+)",
                uart_log_bytes)] if args.target == "flowest" else None
        ),
        "flowest_expected_dx_per_frame": (
            [0, 1, 1, 1, 1, 1] if args.target == "flowest" else None
        ),
        "pass": passed,
    }
    (iter_dir / "verdict_s213.json").write_text(json.dumps(verdict, indent=2) + "\n")

    print(json.dumps(verdict, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
