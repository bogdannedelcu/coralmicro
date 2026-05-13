#!/usr/bin/env python3
"""aruco_pose_publisher.py — SIM sidecar for sentai.flow.mode("anchor").

Same role as sim/scripts/aruco_to_vision_estimate.py (s108) but with
a DIFFERENT consumer: instead of sending VISION_POSITION_ESTIMATE
directly to PX4 over MAVLink, this publishes the latest drone-world
pose to the C-side shim at /tmp/sentai_aruco_pose_recv.sock.  The C
firmware (sim/sentai_aruco_shim_sim.c) reads pose snapshots and
the firmware itself decides what to do with them — forward via
sentai.link MAVLink, sentai.crazy CRTP, or simply expose via
sentai.flow.anchor_pose().  This keeps the architecture identical
to ARM, where the M7 will eventually run the detector itself.

Wire format (matches sim/sentai_aruco_shim_sim.c::aruco_wire_t):

    struct.pack('<II BB H ffff II', magic, frame_seq,
                detected, num_markers, 0,
                x_m, y_m, z_m, yaw_rad,
                detect_us, src_ts_ms)

    magic = 0x41524332  ('ARC2')

Total 36 bytes.  Little-endian.  SOCK_DGRAM Unix socket.

Camera frames come from the EXISTING gz_to_camera_bridge.py UDS at
/tmp/sentai_cam.sock — we tap the same stream the SIM firmware
consumes for flow.  No need for a second gz subscription.

Usage:

    # in a terminal, alongside the SIM:
    python3 sim/scripts/aruco_pose_publisher.py

    # then, in the SIM REPL:
    >>> sentai.flow.mode("anchor")
    >>> sentai.flow.anchor_pose()  # returns the latest published pose
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time
from pathlib import Path

# Reuse the canonical detector — same `cv2.aruco` + solvePnP pipeline
# already proven in s090/s091/s108.
SCRIPT_DIR = Path(__file__).resolve().parent
SDK_ROOT   = SCRIPT_DIR.parent.parent
DETECTOR_DIR = SDK_ROOT / "examples" / "sentai_runtime" / "experiments" / "s090_hover_over_cat"
sys.path.insert(0, str(DETECTOR_DIR))

import numpy as np
try:
    from aruco_detector import (detect_markers, estimate_drone_world_pose,
                                KNOWN_POSITIONS_M)
except ImportError as e:
    print(f"[aruco_pub] FATAL: cannot import aruco_detector from {DETECTOR_DIR}: {e}",
          file=sys.stderr)
    sys.exit(2)

# Camera bridge wire format (must mirror gz_to_camera_bridge.py +
# sim/camera_bridge_recv.c — see camera_bridge_recv.c header comment).
CAM_REQ_MAGIC   = 0x53434D31  # 'SCM1'
CAM_REPLY_MAGIC = 0x46524C31  # 'FRL1'
CAM_HDR_FMT     = "<IIIIII"   # magic seq w h pix_fmt payload_bytes
CAM_HDR_SIZE    = struct.calcsize(CAM_HDR_FMT)
CAM_REPLY_FMT   = "<IIiiIQ"   # magic seq dx dy conf latency_us
CAM_REPLY_SIZE  = struct.calcsize(CAM_REPLY_FMT)

# Our publish wire format.
PUB_MAGIC = 0x41524332        # 'ARC2'
PUB_FMT   = "<II BB H ffff II"
PUB_SIZE  = struct.calcsize(PUB_FMT)
assert PUB_SIZE == 36, f"expected 36-byte struct, got {PUB_SIZE}"

DEFAULT_RECV_UDS = "/tmp/sentai_aruco_pose_recv.sock"
DEFAULT_CAM_UDS  = "/tmp/sentai_cam.sock"


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recv-uds", default=DEFAULT_RECV_UDS,
                    help="destination UDS where the C shim listens "
                         "(default: %(default)s)")
    ap.add_argument("--cam-uds", default=DEFAULT_CAM_UDS,
                    help="camera bridge UDS path (default: %(default)s)")
    ap.add_argument("--frames-dir", default=None,
                    help="optional: replay PPMs from this dir instead "
                         "of subscribing to the camera bridge")
    ap.add_argument("--verbose", "-v", action="store_true",
                    help="print per-frame detection summary")
    return ap.parse_args()


def send_pose(send_sock: socket.socket, dest: str, seq: int,
              detected: bool, num_markers: int,
              x_m: float, y_m: float, z_m: float, yaw_rad: float,
              detect_us: int):
    """Encode and send one pose datagram.  Failures are logged, not fatal —
    the publisher must keep running even if the C side hasn't bound yet."""
    src_ts_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
    pkt = struct.pack(PUB_FMT,
                      PUB_MAGIC, seq & 0xFFFFFFFF,
                      1 if detected else 0, num_markers & 0xFF, 0,
                      float(x_m), float(y_m), float(z_m), float(yaw_rad),
                      int(detect_us) & 0xFFFFFFFF, src_ts_ms)
    try:
        send_sock.sendto(pkt, dest)
    except FileNotFoundError:
        # C shim not bound yet; harmless, retry next frame.
        pass
    except ConnectionRefusedError:
        pass


def detect_and_publish(rgb: np.ndarray, seq: int, send_sock, dest, verbose):
    """Run cv2.aruco + PnP, publish one pose datagram."""
    t0 = time.monotonic()
    dets = detect_markers(rgb, estimate_pose=True)
    detect_us = int((time.monotonic() - t0) * 1e6)

    if dets:
        pose = estimate_drone_world_pose(dets, drone_yaw=0.0)
        if pose is not None:
            x, y, z = pose
            send_pose(send_sock, dest, seq, True, len(dets),
                      x, y, z, 0.0, detect_us)
            if verbose:
                print(f"[aruco_pub] seq={seq} N={len(dets)} "
                      f"pose=({x:+.3f},{y:+.3f},{z:+.3f})m "
                      f"detect={detect_us/1000.0:.1f}ms")
            return
    # No marker — publish detected=0 so the C side knows the detector
    # ran but failed (vs "shim never received any packet").
    send_pose(send_sock, dest, seq, False, 0, 0.0, 0.0, 0.0, 0.0, detect_us)
    if verbose:
        print(f"[aruco_pub] seq={seq} no markers ({detect_us/1000.0:.1f}ms)")


def tap_camera_bridge(uds_path: str, on_frame):
    """Connect to gz_to_camera_bridge.py and call on_frame(rgb, seq) for
    each frame received.  Reuses the same wire format as
    camera_bridge_recv.c.

    NOTE: this is a separate consumer of the bridge, but the bridge is
    request/response — it sends a frame, waits for a reply, sends the
    next.  If we DON'T reply, the bridge waits forever and the firmware
    starves of frames.  So we send a dummy "Flow Reply" the firmware
    will simply ignore (different magic field will mark it as ours).
    Better solution: subscribe directly to gz transport (separate
    consumer).  Implemented below as the default path."""
    raise NotImplementedError(
        "direct camera-bridge tapping not implemented — use "
        "--frames-dir or run subscribed to gz transport (TODO)")


def subscribe_gz(on_frame):
    """Subscribe directly to gz /downward_cam/image and call on_frame(rgb, seq).
    Independent of the camera bridge (no contention with the SIM
    firmware's flow path)."""
    # Lazy import — gz Python bindings may not be on path in CI.
    try:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
    except ImportError as e:
        try:
            from gz.transport14 import Node            # Garden
            from gz.msgs10.image_pb2 import Image
        except ImportError:
            raise RuntimeError(
                "gz.transport not importable.  Either run inside the "
                "crazysim-garden distrobox (where gz Python is set up) "
                "or use --frames-dir to replay captured PPMs.") from e

    node = Node()
    seq = [0]
    last_log = [time.monotonic()]

    def _cb(msg: Image):
        # Image.pixel_format_type: 3 = RGB_INT8, 4 = BGR_INT8
        try:
            data = bytes(msg.data)
            w = int(msg.width)
            h = int(msg.height)
            if msg.pixel_format_type == 3:    # RGB
                rgb = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
            elif msg.pixel_format_type == 4:  # BGR
                bgr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
                rgb = bgr[:, :, ::-1].copy()
            else:
                return  # unsupported
            seq[0] += 1
            on_frame(rgb, seq[0])
            now = time.monotonic()
            if now - last_log[0] >= 5.0:
                print(f"[aruco_pub] {seq[0]} frames processed")
                last_log[0] = now
        except Exception as e:
            print(f"[aruco_pub] frame cb error: {e}", file=sys.stderr)

    topic = "/downward_cam/image"
    if not node.subscribe(Image, topic, _cb):
        raise RuntimeError(f"gz subscribe to {topic} failed")
    print(f"[aruco_pub] subscribed to gz {topic}")
    while True:
        time.sleep(0.5)


def replay_frames_dir(frames_dir: Path, on_frame):
    """Iterate PPMs from frames_dir (oldest first) and call on_frame for
    each.  Useful for unit testing the publisher without running gz."""
    from PIL import Image as PILImage
    ppms = sorted(frames_dir.glob("*.ppm"))
    if not ppms:
        print(f"[aruco_pub] no .ppm under {frames_dir}", file=sys.stderr)
        return
    seq = 0
    for p in ppms:
        rgb = np.array(PILImage.open(p).convert("RGB"))
        seq += 1
        on_frame(rgb, seq)
        time.sleep(1.0 / 30.0)


def main():
    args = parse_args()

    # Caller-side socket: SOCK_DGRAM, doesn't bind (we always sendto).
    send_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    print(f"[aruco_pub] publishing to {args.recv_uds}")
    print(f"[aruco_pub] wire size {PUB_SIZE} bytes, magic 0x{PUB_MAGIC:08X}")
    print(f"[aruco_pub] known markers: {sorted(KNOWN_POSITIONS_M.keys())}")

    def on_frame(rgb, seq):
        detect_and_publish(rgb, seq, send_sock, args.recv_uds, args.verbose)

    if args.frames_dir:
        replay_frames_dir(Path(args.frames_dir), on_frame)
    else:
        subscribe_gz(on_frame)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[aruco_pub] interrupted, exiting cleanly")
        sys.exit(0)
