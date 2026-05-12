#!/usr/bin/env python3
"""gz_pose_logger.py — subscribe to a Gazebo world dynamic_pose/info
topic and log a target model's world-frame pose to CSV.

Used by s101+ hover bench to record x500_sentai_0 ground truth while
PX4 EKF runs flow-only nav.  Lives in distrobox (Garden gz bindings).

CSV columns: t_s, x_m, y_m, z_m, qx, qy, qz, qw
t_s is wall-clock seconds since logger start (relative).

Usage:
    python3 sim/scripts/gz_pose_logger.py \
        --topic /world/sentai_crazysim/dynamic_pose/info \
        --model x500_sentai_0 \
        --out /tmp/pose.csv
"""
from __future__ import annotations
import argparse
import csv
import signal
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--model", required=True,
                    help="model name to track (e.g. x500_sentai_0)")
    ap.add_argument("--out", required=True, help="CSV path")
    args = ap.parse_args()

    try:
        from gz.transport12 import Node
        from gz.msgs9.pose_v_pb2 import Pose_V
    except ImportError as e:
        print(f"ERROR: gz Garden python bindings missing: {e}", file=sys.stderr)
        return 2

    out = open(args.out, "w", newline="")
    w = csv.writer(out)
    w.writerow(["t_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])
    out.flush()

    t0 = time.monotonic()
    state = {"n": 0, "last_log": t0}

    def cb(msg: Pose_V) -> None:
        # Find our model in the vector and log its pose.
        for p in msg.pose:
            if p.name == args.model:
                t = time.monotonic() - t0
                w.writerow([f"{t:.4f}",
                            f"{p.position.x:.5f}",
                            f"{p.position.y:.5f}",
                            f"{p.position.z:.5f}",
                            f"{p.orientation.x:.5f}",
                            f"{p.orientation.y:.5f}",
                            f"{p.orientation.z:.5f}",
                            f"{p.orientation.w:.5f}"])
                out.flush()
                state["n"] += 1
                now = time.monotonic()
                if now - state["last_log"] >= 2.0:
                    print(f"[pose] {state['n']} samples, "
                          f"last z={p.position.z:+.3f}m",
                          file=sys.stderr, flush=True)
                    state["last_log"] = now
                break

    node = Node()
    if not node.subscribe(Pose_V, args.topic, cb):
        print(f"[pose] subscribe to {args.topic} FAILED", file=sys.stderr)
        return 3
    print(f"[pose] logging {args.model} on {args.topic} -> {args.out}",
          file=sys.stderr)

    def _stop(_sig, _frm):
        out.close()
        print(f"[pose] stop: {state['n']} samples written", file=sys.stderr)
        sys.exit(0)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    sys.exit(main())
