#!/usr/bin/env python3
"""Run the sentai_sim WhyCon backend on a synthetic dataset.

This runner keeps the actual detection inside sentai_sim/MicroPython:
it stages a small script into the simulated filesystem, symlinks the
dataset into that filesystem, and asks sentai.markers.detect_buffer()
to process each PGM frame.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import subprocess
import struct
import textwrap


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = REPO_ROOT / "dataset" / "TD-S10-B2"
DEFAULT_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
DEFAULT_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
    return rows


def run_id() -> str:
    return dt.datetime.now().strftime("sentai_sim_%Y%m%d_%H%M%S")


def ensure_symlink(link: Path, target: Path) -> None:
    if link.is_symlink() or link.exists():
        if link.is_dir() and not link.is_symlink():
            raise RuntimeError(f"Refusing to replace real directory: {link}")
        link.unlink()
    link.symlink_to(target)


def make_mp_script(
    frame_paths: list[str],
    out_jsonl: str,
    out_summary: str,
    camera: dict,
    marker_diameter_m: float,
    marker_world: list[dict],
    frame_yaws_rad: list[float],
) -> str:
    # Keep the script plain MicroPython.  No json module, no bytearray
    # dependency, and no multiline REPL blocks.
    frame_repr = repr(frame_paths)
    yaw_repr = repr(frame_yaws_rad)
    marker_world_bytes = b"".join(
        struct.pack("<fff", *[float(v) for v in marker["world_xyz"]])
        for marker in marker_world
    )
    marker_world_repr = repr(marker_world_bytes)
    marker_ids_repr = repr([str(marker.get("marker_id", i)) for i, marker in enumerate(marker_world)])
    return textwrap.dedent(
        f"""
        import sentai

        FRAMES = {frame_repr}
        FRAME_YAWS_RAD = {yaw_repr}
        MARKER_WORLD = {marker_world_repr}
        MARKER_IDS = {marker_ids_repr}
        OUT_JSONL = "{out_jsonl}"
        OUT_SUMMARY = "{out_summary}"

        def _det_json_tuple(t):
            radius_outer = t[17] if len(t) > 17 else 0.0
            return ('{{"id":%d,"center":[%.3f,%.3f],'
                    '"axis_a":%.6f,"axis_b":%.6f,"angle_rad":%.6f,'
                    '"radius_outer":%.6f,'
                    '"comp_id":%d,'
                    '"tvec_cam":[%.6f,%.6f,%.6f],'
                    '"rvec_cam":[%.6f,%.6f,%.6f],'
                    '"reproj_err_px":%.6f,"backend":%d,"pose_valid":%d,'
                    '"geometry_valid":%d}}') % (
                t[0], t[1], t[2],
                t[3], t[4], t[5],
                radius_outer,
                t[6],
                t[7], t[8], t[9],
                t[10], t[11], t[12],
                t[13], t[14], t[15], t[16])

        def main():
            sentai.markers.clear()
            rc = sentai.markers.init("whycon")
            sentai.markers.set_intrinsics({float(camera["fx"])}, {float(camera["fy"])}, {float(camera["cx"])}, {float(camera["cy"])})
            sentai.markers.set_marker_size({float(marker_diameter_m)})
            world_rc = sentai.markers.set_marker_world(MARKER_WORLD)
            world_n = sentai.markers.get_marker_world_count()
            sentai.fs.write(OUT_JSONL, "")
            total = 0
            frames_ok = 0
            pose_ok = 0
            for idx, frame in enumerate(FRAMES):
                n = sentai.markers.detect_pgm(frame)
                dets = []
                for i in range(n):
                    t = sentai.markers.get_detection_tuple(i)
                    if t is not None:
                        dets.append(_det_json_tuple(t))
                dp = sentai.markers.get_drone_pose_tuple(FRAME_YAWS_RAD[idx])
                if dp is None:
                    pose_json = 'null'
                else:
                    pose_ok += 1
                    pose_json = ('{{"x":%.6f,"y":%.6f,"z":%.6f,'
                                 '"yaw_rad":%.6f,"res_max":%.6f,'
                                 '"n_used":%d,"flip_x":%d,"flip_y":%d,'
                                 '"flip_z":%d}}') % (
                        dp[0], dp[1], dp[2], dp[3], dp[4],
                        dp[5], dp[6], dp[7], dp[8])
                line = '{{"frame":"%s","rc":%d,"detections":[%s],"drone_pose_world":%s}}\\n' % (
                    frame, n, ",".join(dets), pose_json)
                sentai.fs.append(OUT_JSONL, line)
                total += n
                if n > 0:
                    frames_ok += 1
            summary = '{{"backend":"sentai_sim.markers.whycon","init_rc":%d,' \
                      '"marker_world_rc":%d,"marker_world_count":%d,' \
                      '"fx":%.6f,"fy":%.6f,"cx":%.6f,"cy":%.6f,' \
                      '"marker_diameter_m":%.6f,' \
                      '"frames":%d,"frames_with_detections":%d,' \
                      '"detections_total":%d,"frames_with_drone_pose":%d,' \
                      '"marker_ids":"%s"}}\\n' % (
                          rc, world_rc, world_n,
                          {float(camera["fx"])}, {float(camera["fy"])}, {float(camera["cx"])}, {float(camera["cy"])},
                          {float(marker_diameter_m)},
                          len(FRAMES), frames_ok, total, pose_ok,
                          ",".join(MARKER_IDS))
            sentai.fs.write(OUT_SUMMARY, summary)
            print("TD-S10-B2_SENTAI_SIM_DONE", len(FRAMES), total)

        main()
        """
    ).strip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--limit", type=int, default=0, help="0 means all frames")
    parser.add_argument("--sim-bin", type=Path, default=DEFAULT_SIM_BIN)
    parser.add_argument("--fs-root", type=Path, default=DEFAULT_FS_ROOT)
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    args = parser.parse_args()

    dataset = args.dataset.resolve()
    manifest_path = dataset / "manifest.jsonl"
    config_path = dataset / "config.json"
    if not manifest_path.exists():
        raise SystemExit(f"Missing manifest: {manifest_path}")
    if not config_path.exists():
        raise SystemExit(f"Missing config: {config_path}")
    if not args.sim_bin.exists():
        raise SystemExit(f"Missing sentai_sim binary: {args.sim_bin}")

    rows = load_jsonl(manifest_path)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if args.limit:
        rows = rows[: args.limit]

    dataset_id = dataset.name
    run_dir = args.results_root / dataset_id / run_id()
    run_dir.mkdir(parents=True, exist_ok=False)

    args.fs_root.mkdir(parents=True, exist_ok=True)
    dataset_link = args.fs_root / "td_s10_b1_dataset"
    out_link = args.fs_root / "td_s10_b2_out"
    ensure_symlink(dataset_link, dataset)
    ensure_symlink(out_link, run_dir)

    frame_paths = ["/td_s10_b1_dataset/" + row["image_path"] for row in rows]
    mp_name = "td_s10_b2_sentai_sim_whycon.py"
    mp_script_host = run_dir / mp_name
    mp_script_fs = args.fs_root / mp_name
    mp_script = make_mp_script(
        frame_paths=frame_paths,
        out_jsonl="/td_s10_b2_out/sentai_sim_results.jsonl",
        out_summary="/td_s10_b2_out/sentai_sim_summary.json",
        camera=config["camera"],
        marker_diameter_m=float(config["marker_diameter_m"]),
        marker_world=config["markers"],
        frame_yaws_rad=[
            float(row["pose_world_camera"]["yaw_deg"]) * 3.141592653589793 / 180.0
            for row in rows
        ],
    )
    mp_script_host.write_text(mp_script, encoding="utf-8")
    shutil.copy2(mp_script_host, mp_script_fs)

    command = f"import {Path(mp_name).stem}\n"
    proc = subprocess.run(
        [str(args.sim_bin)],
        input=command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=REPO_ROOT,
        env={**os.environ, "SENTAI_SIM_ROOT": str(args.fs_root.resolve())},
        timeout=60,
        check=False,
    )
    (run_dir / "sentai_sim_repl.log").write_text(proc.stdout, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed with rc={proc.returncode}; see {run_dir}")

    print(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
