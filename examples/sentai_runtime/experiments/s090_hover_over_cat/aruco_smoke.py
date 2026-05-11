"""Smoke test: capture a frame from gz, run ArUco detector, report.
No drone control involved — just validates that:
  1. Markers placed in Gazebo SDF actually render correctly
  2. Detection works on the 640×480 RGB the drone camera publishes

Usage: python3 aruco_smoke.py [<existing_ppm>]
  - if a PPM path is given, detects on that file directly
  - otherwise launches sentai_sim briefly to grab one fresh frame
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from aruco_detector import detect_in_ppm, KNOWN_POSITIONS_M


def find_latest_ppm() -> Path | None:
    dirs = sorted(Path("/tmp").glob("sentai_frames_*"),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    for d in dirs:
        ppms = sorted(d.glob("frame_*.ppm"))
        if ppms:
            return ppms[-1]
    return None


def main():
    if len(sys.argv) > 1:
        ppm = Path(sys.argv[1])
    else:
        ppm = find_latest_ppm()
        if ppm is None:
            print("no PPMs found in /tmp/sentai_frames_*; run hover_over_cat.py first",
                  file=sys.stderr)
            return 1
    print(f"[aruco_smoke] reading {ppm}", file=sys.stderr)
    dets = detect_in_ppm(ppm, estimate_pose=True)
    if not dets:
        print("[aruco_smoke] NO ArUco markers detected.  Check:", file=sys.stderr)
        print("  - drone was looking down at the marker patches", file=sys.stderr)
        print("  - markers are well-lit and in focus", file=sys.stderr)
        print(f"  - PPM content (open {ppm} to verify markers are visible)",
              file=sys.stderr)
        return 1
    print(f"[aruco_smoke] {len(dets)} markers detected:")
    for mid, m in sorted(dets.items()):
        world_pos = KNOWN_POSITIONS_M.get(mid, "unknown")
        print(f"  id={mid:2d}  image cxy=({m.cx:6.1f}, {m.cy:6.1f})")
        print(f"          world pos {world_pos}")
        if m.tvec is not None:
            tx, ty, tz = m.tvec
            print(f"          camera-frame tvec=({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) m  "
                  f"→ distance {(tx**2+ty**2+tz**2)**0.5:.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
