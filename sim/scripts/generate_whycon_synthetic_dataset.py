#!/usr/bin/env python3
"""Generate a synthetic WhyCon dataset from Gazebo renders.

This is a camera-only renderer path: no Crazyflie spawn, no cf2 SITL, no
sentai_sim.  Gazebo is used to render the existing WhyCon small-pad world
through a controllable dataset camera.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Iterable
import xml.etree.ElementTree as ET

try:
    from PIL import Image, ImageDraw
except ImportError:  # pragma: no cover - handled at runtime for preview only.
    Image = None
    ImageDraw = None


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WORLD = Path(
    "/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/"
    "crazyflie-simulation/simulator_files/gazebo/worlds/"
    "sentai_whycon_small.sdf"
)
DEFAULT_GUI_CONFIG = REPO_ROOT / "sim" / "gazebo" / "sentai_gui.config"

WORLD_NAME = "sentai_whycon_small"
DATASET_WORLD_NAME = "sentai_whycon_small_dataset"
CAMERA_MODEL = "dataset_camera"
CAMERA_TOPIC = "/dataset_cam/image"
TASK_DATASET_ROOT = REPO_ROOT / "dataset" / "TD-S10-B1"

WIDTH = 320
HEIGHT = 240
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0
HFOV_RAD = 2.0 * math.atan(WIDTH / (2.0 * FX))
MARKER_DIAMETER_M = 0.0544
REGULAR_MIN_EXPECTED_MARKERS = 4
STRESS_MIN_EXPECTED_MARKERS = 3

MARKERS = [
    ("NW", -0.08, +0.08, 0.005),
    ("NE", +0.08, +0.08, 0.005),
    ("W",  -0.06, +0.00, 0.005),
    ("E",  +0.06, +0.00, 0.005),
    ("SW", -0.08, -0.08, 0.005),
    ("SE", +0.08, -0.08, 0.005),
    # Asymmetric marker: conceptual (0.25, 1.25) with the small-world scale
    # where conceptual +/-1 maps to +/-0.08 m.
    ("N", +0.02, +0.10, 0.005),
]


def run_id() -> str:
    return dt.datetime.now().strftime("whycon_gazebo_synth_%Y%m%d_%H%M%S")


def q_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def rot_x(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rot_y(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_z(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def mat_t_vec(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    )


def camera_rotation_world(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    # Columns are camera OpenCV axes in world coordinates at zero body RPY:
    # image +x -> world +Y, image +y -> world +X, optical +z -> world -Z.
    r0 = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    body = matmul(rot_z(yaw), matmul(rot_y(pitch), rot_x(roll)))
    return matmul(body, r0)


def project_marker(
    pose: dict[str, float], marker: tuple[str, float, float, float]
) -> dict[str, object]:
    name, mx, my, mz = marker
    roll = math.radians(pose["roll_deg"])
    pitch = math.radians(pose["pitch_deg"])
    yaw = math.radians(pose["yaw_deg"])
    r_wc = camera_rotation_world(roll, pitch, yaw)
    dx = mx - pose["x"]
    dy = my - pose["y"]
    dz = mz - pose["z"]
    pc = mat_t_vec(r_wc, (dx, dy, dz))
    visible = False
    evaluation_visible = False
    reason = "behind_camera"
    pixel = None
    radius = None
    bbox = None
    if pc[2] > 1e-6:
        u = FX * pc[0] / pc[2] + CX
        v = FY * pc[1] / pc[2] + CY
        radius = FX * (MARKER_DIAMETER_M * 0.5) / pc[2]
        x0, y0 = u - radius, v - radius
        x1, y1 = u + radius, v + radius
        in_frame = (0.0 <= u < WIDTH) and (0.0 <= v < HEIGHT)
        full = (x0 >= 0.0 and y0 >= 0.0 and x1 < WIDTH and y1 < HEIGHT)
        if full:
            visible, evaluation_visible, reason = True, True, "visible"
        elif in_frame:
            visible, evaluation_visible, reason = False, False, "partial_crop"
        else:
            reason = "out_of_fov"
        pixel = [u, v]
        bbox = [x0, y0, x1, y1]
        if radius < 2.0:
            visible, reason = False, "too_small"
    return {
        "marker_id": name,
        "world_xyz": [mx, my, mz],
        "visible": visible,
        "evaluation_visible": evaluation_visible,
        "visibility_reason": reason,
        "pixel_center": pixel,
        "pixel_radius_outer": radius,
        "projected_bbox": bbox,
    }


def smoke_poses() -> list[dict[str, float]]:
    zs = [0.30, 0.50, 0.75, 1.00]
    poses: list[dict[str, float]] = []
    for z in zs:
        poses.append(dict(x=0.0, y=0.0, z=z, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0))
    variants = [
        (+0.05, +0.00, 0.50, +3.0, -2.0, 0.0),
        (-0.05, +0.05, 0.50, -3.0, +2.0, 45.0),
        (+0.10, -0.10, 0.75, +5.0, +0.0, 90.0),
        (-0.15, +0.10, 0.75, +0.0, -5.0, 135.0),
        (+0.25, +0.00, 1.00, +7.0, -4.0, 180.0),
        (-0.25, +0.00, 1.00, -7.0, +4.0, 225.0),
        (+0.20, +0.20, 1.00, +10.0, -6.0, 270.0),
        (-0.20, -0.20, 1.00, -10.0, +6.0, 315.0),
        (+0.00, +0.00, 0.75, +12.0, +0.0, 30.0),
        (+0.00, +0.00, 0.75, +0.0, -12.0, 300.0),
    ]
    for x, y, z, r, p, yaw in variants:
        poses.append(dict(x=x, y=y, z=z, roll_deg=r, pitch_deg=p, yaw_deg=yaw))
    return poses


def grid_poses() -> Iterable[dict[str, float]]:
    xy = [-0.75, -0.50, -0.25, 0.0, +0.25, +0.50, +0.75]
    zs = [0.30, 0.50, 0.75, 1.00]
    yaws = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    for z in zs:
        for x in xy:
            for y in xy:
                for yaw in yaws:
                    yield dict(x=x, y=y, z=z, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw)
                    yield dict(x=x, y=y, z=z, roll_deg=+4.0, pitch_deg=-3.0, yaw_deg=yaw)


def expected_visible_count(pose: dict[str, float]) -> int:
    return sum(
        1
        for marker in MARKERS
        if project_marker(pose, marker).get("evaluation_visible")
    )


def filter_pose_visibility(poses: Iterable[dict[str, float]]) -> list[dict[str, float]]:
    return [
        pose for pose in poses
        if expected_visible_count(pose) >= REGULAR_MIN_EXPECTED_MARKERS
    ]


def interleave_by_z(poses: Iterable[dict[str, float]]) -> list[dict[str, float]]:
    """Keep small --max-frames subsets representative across altitude strata."""
    buckets: dict[float, list[dict[str, float]]] = {}
    for pose in poses:
        buckets.setdefault(pose["z"], []).append(pose)
    ordered: list[dict[str, float]] = []
    zs = sorted(buckets)
    while any(buckets[z] for z in zs):
        for z in zs:
            if buckets[z]:
                ordered.append(buckets[z].pop(0))
    return ordered


def insert_dataset_camera(src_world: Path, dst_world: Path) -> None:
    text = src_world.read_text()
    text = text.replace(f'<world name="{WORLD_NAME}">', f'<world name="{DATASET_WORLD_NAME}">', 1)
    text = text.replace(f"<world name='{WORLD_NAME}'>", f"<world name='{DATASET_WORLD_NAME}'>", 1)
    camera_model = f"""

    <!-- TD-S10-B1 camera-only dataset capture model. -->
    <model name="{CAMERA_MODEL}">
      <static>false</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <sensor name="dataset_cam" type="camera">
          <always_on>true</always_on>
          <update_rate>30</update_rate>
          <pose>0 0 0 0 1.5707963 3.1415927</pose>
          <camera>
            <horizontal_fov>{HFOV_RAD:.9f}</horizontal_fov>
            <image>
              <width>{WIDTH}</width>
              <height>{HEIGHT}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.01</near>
              <far>20</far>
            </clip>
          </camera>
          <topic>{CAMERA_TOPIC}</topic>
        </sensor>
      </link>
    </model>
"""
    asym_marker_model = """

    <!-- TD-S10-B1 asymmetric WhyCon marker, proportional conceptual (0.25, 1.25). -->
    <model name="whycon_N">
      <static>true</static>
      <pose>+0.02 +0.10 0.005  0 0 0</pose>
      <link name="link"><visual name="v">
        <geometry><box><size>0.06 0.06 0.01</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>materials/textures/whycon_krajnik.png</albedo_map>
            <metalness>0</metalness><roughness>1</roughness>
          </metal></pbr></material>
      </visual></link>
    </model>
"""
    if "</world>" not in text:
        raise RuntimeError(f"cannot find </world> in {src_world}")
    text = text.replace("</world>", asym_marker_model + camera_model + "\n  </world>", 1)
    dst_world.write_text(text)


def stage_world_resources(src_world: Path, work_dir: Path) -> None:
    """Make relative SDF material paths resolve from the generated world dir."""
    candidates = [
        src_world.parent / "materials",
        src_world.parent.parent / "materials",
        REPO_ROOT / "sim" / "gazebo" / "materials",
    ]
    dst = work_dir / "materials"
    if dst.exists():
        return
    for src in candidates:
        if src.exists():
            try:
                dst.symlink_to(src, target_is_directory=True)
            except OSError:
                shutil.copytree(src, dst)
            return
    raise RuntimeError(
        "could not stage Gazebo materials; checked: "
        + ", ".join(str(p) for p in candidates)
    )


def make_dataset_gui_config(src_gui: Path, dst_gui: Path) -> None:
    text = src_gui.read_text()
    text = text.replace("/downward_cam/image", CAMERA_TOPIC)
    text = text.replace(
        "SentAI down-cam (640x480, FOV 58°)",
        "WhyCon dataset camera (320x240)",
    )
    text = text.replace(
        '<property type="string" key="state">docked_collapsed</property>',
        '<property type="string" key="state">docked</property>',
    )
    dst_gui.write_text(text)


def clean_prior_gazebo(distrobox: str) -> None:
    subprocess.run(["pkill", "-9", "-f", "gz sim|gz-tools"], check=False)
    subprocess.run(
        ["distrobox", "enter", distrobox, "--", "pkill", "-9", "-f", "gz sim|gz-tools"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1.0)


def launch_gazebo(distrobox: str, world: Path, gui_config: Path) -> subprocess.Popen:
    resource_path = ":".join([
        str(world.parent),
        str(world.parent.parent),
        str(REPO_ROOT / "sim" / "gazebo"),
        "/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/"
        "crazyflie-simulation/simulator_files/gazebo",
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ])
    cmd = [
        "distrobox", "enter", distrobox, "--",
        "env", f"GZ_SIM_RESOURCE_PATH={resource_path}",
        "gz", "sim", "-r", "-v", "3",
        "--gui-config", str(gui_config),
        str(world),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.DEVNULL)


def wait_for_topic(distrobox: str, topic: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        proc = subprocess.run(
            ["distrobox", "enter", distrobox, "--", "gz", "topic", "-l"],
            capture_output=True,
            text=True,
            check=False,
        )
        if topic in proc.stdout:
            return
        time.sleep(0.5)
    raise TimeoutError(f"Gazebo topic {topic} did not appear within {timeout_s:.1f}s")


def get_model_id(distrobox: str, model_name: str) -> int:
    proc = subprocess.run(
        ["distrobox", "enter", distrobox, "--", "gz", "model", "-m", model_name, "-p"],
        capture_output=True,
        text=True,
        check=False,
    )
    m = re.search(r"Model:\s*\[(\d+)\]", proc.stdout)
    if not m:
        raise RuntimeError(
            f"could not resolve Gazebo model id for {model_name!r}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return int(m.group(1))


def set_camera_pose(
    distrobox: str,
    model_id: int,
    pose: dict[str, float],
    retries: int = 3,
) -> None:
    qx, qy, qz, qw = q_from_rpy(
        math.radians(pose["roll_deg"]),
        math.radians(pose["pitch_deg"]),
        math.radians(pose["yaw_deg"]),
    )
    req = (
        f"id: {model_id} "
        f'position {{x: {pose["x"]:.6f} y: {pose["y"]:.6f} z: {pose["z"]:.6f}}} '
        f'orientation {{x: {qx:.9f} y: {qy:.9f} z: {qz:.9f} w: {qw:.9f}}}'
    )
    last_proc: subprocess.CompletedProcess[str] | None = None
    for attempt in range(1, retries + 1):
        proc = subprocess.run(
            [
                "distrobox", "enter", distrobox, "--",
                "gz", "service",
                "-s", f"/world/{DATASET_WORLD_NAME}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "10000",
                "--req", req,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        last_proc = proc
        if proc.returncode == 0 and "data: true" in (proc.stdout + proc.stderr):
            return
        time.sleep(0.5 * attempt)
    assert last_proc is not None
    raise RuntimeError(
        "failed to set dataset camera pose\n"
        f"request: {req}\nstdout:\n{last_proc.stdout}\nstderr:\n{last_proc.stderr}"
    )


def capture_rgb(distrobox: str, topic: str) -> tuple[int, int, bytes]:
    proc = subprocess.run(
        [
            "distrobox", "enter", distrobox, "--",
            "gz", "topic", "--json-output", "-e", "-t", topic, "-n", "1",
        ],
        capture_output=True,
        timeout=30,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode(errors="replace")[:1000])
    msg = json.loads(proc.stdout.decode())
    w, h = int(msg["width"]), int(msg["height"])
    pixfmt = msg.get("pixelFormatType", "")
    if pixfmt not in ("RGB_INT8", "R8G8B8"):
        raise RuntimeError(f"unsupported Gazebo pixel format {pixfmt!r}")
    data = base64.b64decode(msg["data"])
    expected = w * h * 3
    if len(data) < expected:
        raise RuntimeError(f"short image payload: got {len(data)} expected {expected}")
    return w, h, data[:expected]


def write_pgm(path: Path, w: int, h: int, rgb: bytes) -> None:
    gray = bytearray(w * h)
    for i in range(w * h):
        r = rgb[3 * i]
        g = rgb[3 * i + 1]
        b = rgb[3 * i + 2]
        gray[i] = (77 * r + 150 * g + 29 * b) >> 8
    with path.open("wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(gray)


def write_preview_artifacts(out_dir: Path, max_tiles: int = 120) -> Path | None:
    if Image is None or ImageDraw is None:
        print("[dataset] preview skipped: Pillow is not available", flush=True)
        return None

    frames = sorted((out_dir / "frames").glob("*.pgm"))
    if not frames:
        return None

    preview_dir = out_dir / "preview"
    preview_dir.mkdir(exist_ok=True)
    thumbs = []
    for frame in frames[:max_tiles]:
        img = Image.open(frame).convert("L")
        png = preview_dir / f"{frame.stem}.png"
        img.save(png)
        thumbs.append((frame, img.convert("RGB").resize((160, 120))))

    cols = 5
    gutter = 2
    tile_w = 160
    tile_h = 120
    label_h = 25
    rows = math.ceil(len(thumbs) / cols)
    sheet_w = cols * tile_w + (cols + 1) * gutter
    sheet_h = rows * (tile_h + label_h) + (rows + 1) * gutter
    sheet = Image.new("RGB", (sheet_w, sheet_h), "white")
    draw = ImageDraw.Draw(sheet)
    for i, (frame, thumb) in enumerate(thumbs):
        col = i % cols
        row = i // cols
        x = gutter + col * (tile_w + gutter)
        y = gutter + row * (tile_h + label_h + gutter)
        sheet.paste(thumb, (x, y))
        z = frame.stem.split("_z", 1)[1].split("_", 1)[0]
        n = frame.stem.split("_n")[-1]
        draw.text((x + 4, y + tile_h + 3), f"{i:02d} z{z} n{n}", fill=(0, 0, 0))

    sheet_path = preview_dir / "contact_sheet.png"
    sheet.save(sheet_path)
    return sheet_path


def frame_name(i: int, pose: dict[str, float], n_visible: int) -> str:
    return (
        f"frame_{i:06d}_"
        f"x{pose['x']:+.3f}_y{pose['y']:+.3f}_z{pose['z']:.3f}_"
        f"r{pose['roll_deg']:+.1f}_p{pose['pitch_deg']:+.1f}_"
        f"yaw{int(round(pose['yaw_deg'])) % 360:03d}_n{n_visible}.pgm"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    ap.add_argument("--out-root", type=Path, default=TASK_DATASET_ROOT)
    ap.add_argument("--distrobox", default="crazysim-garden")
    ap.add_argument("--gui-config", type=Path, default=DEFAULT_GUI_CONFIG)
    ap.add_argument("--mode", choices=("smoke", "grid"), default="smoke")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--settle-s", type=float, default=0.25)
    ap.add_argument("--startup-timeout-s", type=float, default=40.0)
    ap.add_argument("--keep-gazebo", action="store_true")
    ap.add_argument("--no-cleanup-first", action="store_true")
    args = ap.parse_args()

    if not args.world.exists():
        raise SystemExit(f"world not found: {args.world}")
    args.out_root.mkdir(parents=True, exist_ok=True)
    out_dir = args.out_root / run_id()
    frames_dir = out_dir / "frames"
    work_dir = out_dir / "_work"
    frames_dir.mkdir(parents=True)
    work_dir.mkdir(parents=True)
    tmp_world = work_dir / "sentai_whycon_small_dataset.sdf"
    tmp_gui = work_dir / "sentai_dataset_gui.config"
    stage_world_resources(args.world, work_dir)
    insert_dataset_camera(args.world, tmp_world)
    make_dataset_gui_config(args.gui_config, tmp_gui)

    poses = (
        smoke_poses()
        if args.mode == "smoke"
        else interleave_by_z(filter_pose_visibility(grid_poses()))
    )
    if args.max_frames > 0:
        poses = poses[: args.max_frames]

    config = {
        "dataset_id": out_dir.name,
        "created_utc": dt.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "source_world": str(args.world),
        "render_world": str(tmp_world),
        "render_gui_config": str(tmp_gui),
        "scene_id": DATASET_WORLD_NAME,
        "camera_topic": CAMERA_TOPIC,
        "camera_model": CAMERA_MODEL,
        "image": {"width": WIDTH, "height": HEIGHT, "format": "P5_PGM"},
        "camera": {
            "fx": FX, "fy": FY, "cx": CX, "cy": CY,
            "horizontal_fov_rad": HFOV_RAD,
            "zero_pose_convention": "image+x->world+Y, image+y->world+X, optical+z->world-Z",
        },
        "marker_diameter_m": MARKER_DIAMETER_M,
        "markers": [
            {"marker_id": name, "world_xyz": [x, y, z]}
            for name, x, y, z in MARKERS
        ],
        "sampling": {"mode": args.mode, "frame_count": len(poses)},
        "visibility_policy": {
            "regular_min_expected_markers": REGULAR_MIN_EXPECTED_MARKERS,
            "stress_min_expected_markers": STRESS_MIN_EXPECTED_MARKERS,
            "markers_expected_definition": "full_projected_outer_circle_inside_image",
            "partial_crop_counts_as_expected": False,
            "edge_cases_with_3_to_5_markers": True,
            "regular_dataset_allows_3_marker_frames": False,
            "stress_dataset_allows_3_marker_frames": True,
            "occlusion_modeled": False,
            "occlusion_note": (
                "Visibility is geometric from projection and image bounds; "
                "Gazebo occlusion is not modeled in manifest classification."
            ),
        },
        "gazebo": {"distrobox": args.distrobox, "gui_enabled": True},
    }
    (out_dir / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    shutil.copyfile(tmp_world, out_dir / "render_world.sdf")

    gz_proc: subprocess.Popen | None = None
    try:
        if not args.no_cleanup_first:
            clean_prior_gazebo(args.distrobox)
        print(f"[dataset] launching Gazebo GUI with {tmp_world}", flush=True)
        gz_proc = launch_gazebo(args.distrobox, tmp_world, tmp_gui)
        wait_for_topic(args.distrobox, CAMERA_TOPIC, args.startup_timeout_s)
        print(f"[dataset] topic ready: {CAMERA_TOPIC}", flush=True)
        camera_model_id = get_model_id(args.distrobox, CAMERA_MODEL)
        print(f"[dataset] camera model id: {camera_model_id}", flush=True)

        manifest_path = out_dir / "manifest.jsonl"
        with manifest_path.open("w", encoding="utf-8") as manifest:
            for i, pose in enumerate(poses):
                markers = [project_marker(pose, marker) for marker in MARKERS]
                n_visible = sum(1 for m in markers if m.get("evaluation_visible"))
                n_partial = sum(
                    1 for m in markers
                    if m.get("visibility_reason") == "partial_crop"
                )
                name = frame_name(i, pose, n_visible)
                rel = Path("frames") / name
                set_camera_pose(args.distrobox, camera_model_id, pose)
                time.sleep(args.settle_s)
                w, h, rgb = capture_rgb(args.distrobox, CAMERA_TOPIC)
                if (w, h) != (WIDTH, HEIGHT):
                    raise RuntimeError(f"unexpected image size {w}x{h}")
                write_pgm(out_dir / rel, w, h, rgb)
                row = {
                    "frame_id": i,
                    "image_path": rel.as_posix(),
                    "scene_id": DATASET_WORLD_NAME,
                    "camera": {
                        "resolution": [WIDTH, HEIGHT],
                        "fx": FX, "fy": FY, "cx": CX, "cy": CY,
                    },
                    "pose_world_camera": pose,
                    "markers_expected": n_visible,
                    "markers_partial_crop": n_partial,
                    "markers": markers,
                }
                manifest.write(json.dumps(row, separators=(",", ":")) + "\n")
                manifest.flush()
                print(f"[dataset] {i + 1:04d}/{len(poses):04d} {name}", flush=True)
        print(f"[dataset] wrote {out_dir}", flush=True)
        preview_path = write_preview_artifacts(out_dir)
        if preview_path is not None:
            print(f"[dataset] wrote preview {preview_path}", flush=True)
        return 0
    finally:
        if gz_proc is not None and not args.keep_gazebo:
            print("[dataset] stopping Gazebo", flush=True)
            gz_proc.send_signal(signal.SIGINT)
            try:
                gz_proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                gz_proc.kill()
        elif gz_proc is not None:
            print("[dataset] leaving Gazebo running (--keep-gazebo)", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
