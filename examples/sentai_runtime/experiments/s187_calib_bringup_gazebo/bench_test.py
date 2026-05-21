"""bench_test — on-the-bench WhyCon detector + PnP sanity check.

Feeds one saved PGM frame to sentai.markers.detect_frame, prints what
the detector reports per marker: axis_a, pixel_cx/cy, tvec_cam, pose_valid.

Run inside sentai_sim REPL (no Gazebo needed).  Lets us check whether
the detector + PnP work end-to-end on a known frame, decoupled from
the live SITL run.
"""

import sentai

FX, FY, CX, CY     = 288.3, 288.3, 160.0, 120.0
MARKER_DIAMETER_M  = 0.1088

# A real frame saved by FR from iter-11, drone at cf2_EKF z=0.79.
FRAME_PATH = "test_frame.pgm"   # bare filename — sim_fs_resolve anchors to FS_ROOT


def main():
    # 1. Init backend + intrinsics + marker size.
    sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(MARKER_DIAMETER_M)
    print("backend:", sentai.markers.backend())

    # 2. Load PGM frame (320x240 P5 binary).  sentai.fs.read returns
    #    raw bytes — strip header (P5 \n WxH \n maxv \n) then 76800 bytes.
    raw = sentai.fs.read(FRAME_PATH)
    if raw is None or len(raw) == 0:
        print("FAIL: could not read", FRAME_PATH); return
    # Find third newline = end of header.
    nl_count = 0
    hdr_end = 0
    for i in range(len(raw)):
        if raw[i] == 0x0A:
            nl_count += 1
            if nl_count == 3:
                hdr_end = i + 1
                break
    gray = raw[hdr_end:]
    print("loaded:", FRAME_PATH.split("/")[-1])
    print("  hdr_bytes={} gray_bytes={}".format(hdr_end, len(gray)))
    if len(gray) != 320 * 240:
        print("FAIL: expected 76800 bytes, got", len(gray))
        return

    # 3. Feed frame to detect_frame via the synthetic-buffer path.
    #    The MP binding for detect_from_camera reads from the camera
    #    ring; for bench we use detect_frame directly with our bytes.
    n = sentai.markers.detect_buffer(gray, 320, 240)
    print("n_detected:", n)
    if n <= 0:
        print("FAIL: no markers detected on this frame")
        return

    # 4. Print per-marker fields.
    print()
    print("idx  pixel_cx/y           tvec_cam (x,y,z)           pose_valid")
    print("-" * 70)
    for i in range(n):
        t = sentai.markers.get_pose_tuple(i)
        if t is None:
            continue
        # (id, pixel_cx, pixel_cy, tx, ty, tz, rx, ry, rz, reproj, backend, valid)
        mid, px, py, tx, ty, tz = t[0], t[1], t[2], t[3], t[4], t[5]
        valid = t[11]
        # Expected axis_a from drone z=0.79 + marker_diam=0.1088:
        #   axis_a_expected = fx * (D/2) / depth = 288.3 * 0.0544 / 0.785 = 19.9 px
        # Expected tvec_cam_z = ~0.78 m
        print("{:3d}  ({:6.1f}, {:6.1f})   ({:+.3f}, {:+.3f}, {:.3f})   {}".format(
            mid, px, py, tx, ty, tz, valid))


main()
