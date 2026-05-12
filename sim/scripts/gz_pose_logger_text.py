#!/usr/bin/env python3
"""Parse `gz topic -e -t /world/.../dynamic_pose/info` text output and
log a target model's pose to CSV.  Used when gz Python bindings are
not available in the runtime env (e.g. crazysim-garden distrobox
ships only C++ libs for transport12/msgs9, no python3-gz-* pkgs).

Reads stdin, writes pose.csv to stdout (or --out FILE).

Each pose block in gz topic -e output looks like:

  pose {
    name: "x500_sentai_0"
    id: 50
    position {
      x: -0.123
      y: 0.456
      z: 1.789
    }
    orientation {
      x: 0.0
      ...
    }
  }
"""
from __future__ import annotations
import argparse
import re
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", default="-",
                    help="CSV out path, or '-' for stdout")
    args = ap.parse_args()

    out = sys.stdout if args.out == "-" else open(args.out, "w")
    out.write("t_s,x_m,y_m,z_m\n")
    out.flush()

    t0 = time.monotonic()
    cur_name: str | None = None
    in_pose = False
    in_pos = False
    px = py = pz = None
    n_written = 0

    pat_name = re.compile(r'^\s*name:\s*"([^"]+)"')
    pat_x = re.compile(r'^\s*x:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_y = re.compile(r'^\s*y:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_z = re.compile(r'^\s*z:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')

    for line in sys.stdin:
        s = line.rstrip("\n")
        if s.startswith("pose {"):
            in_pose = True
            in_pos = False
            cur_name = None
            px = py = pz = None
            continue
        if in_pose:
            m = pat_name.match(s)
            if m and cur_name is None:
                cur_name = m.group(1)
                continue
            if s.lstrip().startswith("position {"):
                in_pos = True
                continue
            if s.lstrip().startswith("orientation {"):
                in_pos = False
                continue
            if in_pos:
                if (m := pat_x.match(s)): px = float(m.group(1))
                elif (m := pat_y.match(s)): py = float(m.group(1))
                elif (m := pat_z.match(s)): pz = float(m.group(1))
                continue
            if s == "}" or s.startswith("}"):
                # End of pose block.
                if cur_name == args.model and None not in (px, py, pz):
                    t = time.monotonic() - t0
                    out.write(f"{t:.4f},{px:.5f},{py:.5f},{pz:.5f}\n")
                    out.flush()
                    n_written += 1
                in_pose = False
                in_pos = False

    print(f"[pose] wrote {n_written} samples", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
