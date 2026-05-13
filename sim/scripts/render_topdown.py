#!/usr/bin/env python3
"""Render a top-down 4K image of a Gazebo world.

Workflow:
  1. Parse source world SDF
  2. Inject a static nadir camera at high altitude with <save enabled="true">
  3. Run gz sim --headless-rendering --no-server-config + Xvfb display
  4. Camera writes PNG frames to /tmp/topdown_render/
  5. Pick the last saved frame, copy to output path
  6. Show summary

Usage:
  python3 render_topdown.py \
    --src ~/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/"harmonic world"/3/harmonic.sdf \
    --out sim/gazebo/worlds/assets/harmonic_topdown_4k.png \
    --altitude 200 \
    --fov-deg 100
"""
from __future__ import annotations
import argparse, os, shutil, subprocess, sys, time, math
from pathlib import Path

CAMERA_SDF_TEMPLATE = """
    <!-- Injected top-down camera for offline render -->
    <model name="render_cam_topdown">
      <static>true</static>
      <pose>{cx} {cy} {alt} 0 1.5707963 0</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <camera>
            <horizontal_fov>{fov_rad}</horizontal_fov>
            <image>
              <width>{w}</width>
              <height>{h}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>{far}</far>
            </clip>
            <save enabled="true">
              <path>{save_dir}</path>
            </save>
          </camera>
          <always_on>true</always_on>
          <update_rate>1</update_rate>
          <visualize>false</visualize>
        </sensor>
      </link>
    </model>
"""

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, help="source SDF world file path")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--altitude", type=float, default=200.0, help="camera altitude m (default 200)")
    ap.add_argument("--fov-deg", type=float, default=100.0, help="horizontal FOV degrees (default 100)")
    ap.add_argument("--width", type=int, default=4096, help="output width px")
    ap.add_argument("--height", type=int, default=4096, help="output height px")
    ap.add_argument("--center", default="0,0", help="x,y center of camera (default 0,0)")
    ap.add_argument("--run-seconds", type=int, default=15, help="how long to let gz sim run (default 15s)")
    args = ap.parse_args()

    src = Path(args.src).expanduser()
    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    if not src.exists():
        print(f"FATAL: source SDF not found: {src}", file=sys.stderr)
        return 1

    # 1. Inject camera before </world>; also strip <include> blocks for
    # problematic models (SDF v1.11+ that gz-sim7 can't parse, or models
    # we don't have cached).  Whitelist-driven: keep only core scene.
    sdf_text = src.read_text()

    # Strip problematic <include>...</include> blocks.  Match block-level.
    import re
    strip_names = [
        "Fidget", "Pendulum", "Tethys", "CartPole",       # SDF v1.11 / missing
        "Sky",                                              # already <sky> tag
        "wide_angle_camera", "lensflare",                   # ogre2 unsupported
    ]
    def strip_includes(text, names):
        # Match <include> ... </include> blocks with name containing any
        # of the target substrings.
        pattern = re.compile(r"<include>(.*?)</include>", re.DOTALL)
        kept = []
        last = 0
        for m in pattern.finditer(text):
            body = m.group(1)
            drop = any(n.lower() in body.lower() for n in names)
            if drop:
                kept.append(text[last:m.start()])
                last = m.end()
                continue
        kept.append(text[last:])
        return "".join(kept)
    sdf_text_stripped = strip_includes(sdf_text, strip_names)
    print(f"[render] stripped problematic includes: {strip_names}")
    sdf_text = sdf_text_stripped
    cx, cy = [float(v) for v in args.center.split(",")]
    fov_rad = math.radians(args.fov_deg)
    save_dir = "/tmp/topdown_render"
    Path(save_dir).mkdir(exist_ok=True)
    # Wipe previous captures
    for f in Path(save_dir).glob("*"):
        f.unlink()

    cam_block = CAMERA_SDF_TEMPLATE.format(
        cx=cx, cy=cy, alt=args.altitude,
        fov_rad=fov_rad, w=args.width, h=args.height,
        far=args.altitude * 2.5,
        save_dir=save_dir,
    )
    if "</world>" not in sdf_text:
        print("FATAL: source SDF has no </world> tag", file=sys.stderr)
        return 2
    patched = sdf_text.replace("</world>", cam_block + "\n  </world>")

    patched_path = Path("/tmp/render_topdown_world.sdf")
    patched_path.write_text(patched)
    print(f"[render] patched SDF: {patched_path}")
    print(f"[render] camera @ ({cx},{cy},{args.altitude}) nadir, fov={args.fov_deg}°, {args.width}x{args.height}")
    print(f"[render] save_dir = {save_dir}")
    print(f"[render] running gz sim headless for {args.run_seconds}s...")

    # 2. Launch via distrobox — use host display ($DISPLAY=:1 in distrobox)
    #    instead of Xvfb.  gz sim with --headless-rendering still needs OpenGL
    #    context; using the host display works reliably.
    script_path = Path("/tmp/render_launch.sh")
    script_text = f"""#!/bin/bash
pkill -9 "gz sim" 2>/dev/null || true
sleep 1
# Run server-only with display attached for GPU OpenGL.  No GUI client.
gz sim -r --headless-rendering -s {patched_path} > /tmp/gz_render.log 2>&1 &
echo $! > /tmp/gz_render_pid
sleep {args.run_seconds}
GZ_PID=$(cat /tmp/gz_render_pid)
kill -9 $GZ_PID 2>/dev/null || true
pkill -9 "gz sim" 2>/dev/null || true
echo "[done] waited {args.run_seconds}s, killed gz"
"""
    script_path.write_text(script_text)
    script_path.chmod(0o755)
    proc = subprocess.run(
        ["distrobox", "enter", "crazysim-garden", "--", "bash", str(script_path)],
        capture_output=True, text=True, timeout=args.run_seconds + 60,
    )
    print("[render] gz sim stdout last lines:")
    print(proc.stdout[-500:] if proc.stdout else "(empty)")
    if proc.returncode != 0:
        print("[render] gz sim stderr:")
        print(proc.stderr[-500:])

    # 3. Find captured frames
    captured = sorted(Path(save_dir).glob("*.png"))
    if not captured:
        # Some Garden versions save under date subdir
        captured = sorted(Path(save_dir).rglob("*.png"))
    if not captured:
        print(f"[render] FAIL — no PNG captured under {save_dir}")
        print(f"[render] tail of gz sim log:")
        log = Path("/tmp/gz_render.log")
        if log.exists():
            print(log.read_text()[-2000:])
        return 3

    last = captured[-1]
    sz = last.stat().st_size
    print(f"[render] captured {len(captured)} frames; using last: {last} ({sz} B)")

    # 4. Copy to output path
    shutil.copy(last, out)
    print(f"[render] saved → {out}")
    print(f"[render] PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
