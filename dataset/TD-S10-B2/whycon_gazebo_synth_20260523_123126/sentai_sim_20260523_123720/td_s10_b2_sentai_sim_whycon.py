import sentai

FRAMES = ['/td_s10_b1_dataset/frames/frame_000000_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw000_n7.pgm', '/td_s10_b1_dataset/frames/frame_000001_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw270_n4.pgm', '/td_s10_b1_dataset/frames/frame_000002_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw000_n4.pgm', '/td_s10_b1_dataset/frames/frame_000003_x-0.500_y-0.250_z1.000_r+0.0_p+0.0_yaw090_n4.pgm', '/td_s10_b1_dataset/frames/frame_000004_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw000_n6.pgm', '/td_s10_b1_dataset/frames/frame_000005_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw315_n5.pgm', '/td_s10_b1_dataset/frames/frame_000006_x-0.250_y-0.250_z0.750_r+4.0_p-3.0_yaw000_n7.pgm', '/td_s10_b1_dataset/frames/frame_000007_x-0.500_y-0.250_z1.000_r+0.0_p+0.0_yaw270_n4.pgm', '/td_s10_b1_dataset/frames/frame_000008_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000009_x+0.000_y-0.250_z0.500_r+4.0_p-3.0_yaw000_n4.pgm', '/td_s10_b1_dataset/frames/frame_000010_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw090_n4.pgm', '/td_s10_b1_dataset/frames/frame_000011_x-0.500_y-0.250_z1.000_r+4.0_p-3.0_yaw270_n6.pgm', '/td_s10_b1_dataset/frames/frame_000012_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000013_x+0.000_y-0.250_z0.500_r+4.0_p-3.0_yaw045_n4.pgm', '/td_s10_b1_dataset/frames/frame_000014_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw135_n4.pgm', '/td_s10_b1_dataset/frames/frame_000015_x-0.500_y-0.250_z1.000_r+4.0_p-3.0_yaw315_n6.pgm', '/td_s10_b1_dataset/frames/frame_000016_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw090_n6.pgm', '/td_s10_b1_dataset/frames/frame_000017_x+0.000_y+0.000_z0.500_r+0.0_p+0.0_yaw000_n7.pgm', '/td_s10_b1_dataset/frames/frame_000018_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw180_n4.pgm', '/td_s10_b1_dataset/frames/frame_000019_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw045_n4.pgm', '/td_s10_b1_dataset/frames/frame_000020_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw090_n6.pgm', '/td_s10_b1_dataset/frames/frame_000021_x+0.000_y+0.000_z0.500_r+4.0_p-3.0_yaw000_n7.pgm', '/td_s10_b1_dataset/frames/frame_000022_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw270_n4.pgm', '/td_s10_b1_dataset/frames/frame_000023_x-0.500_y+0.000_z1.000_r+4.0_p-3.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000024_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw135_n5.pgm', '/td_s10_b1_dataset/frames/frame_000025_x+0.000_y+0.000_z0.500_r+0.0_p+0.0_yaw045_n7.pgm', '/td_s10_b1_dataset/frames/frame_000026_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw315_n4.pgm', '/td_s10_b1_dataset/frames/frame_000027_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw090_n4.pgm', '/td_s10_b1_dataset/frames/frame_000028_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw135_n5.pgm', '/td_s10_b1_dataset/frames/frame_000029_x+0.000_y+0.000_z0.500_r+4.0_p-3.0_yaw045_n7.pgm']
FRAME_YAWS_RAD = [0.0, 4.71238898038469, 0.0, 1.5707963267948966, 0.0, 5.497787143782138, 0.0, 4.71238898038469, 0.7853981633974483, 0.0, 1.5707963267948966, 4.71238898038469, 0.7853981633974483, 0.7853981633974483, 2.356194490192345, 5.497787143782138, 1.5707963267948966, 0.0, 3.141592653589793, 0.7853981633974483, 1.5707963267948966, 0.0, 4.71238898038469, 0.7853981633974483, 2.356194490192345, 0.7853981633974483, 5.497787143782138, 1.5707963267948966, 2.356194490192345, 0.7853981633974483]
MARKER_WORLD = b'\n\xd7\xa3\xbd\n\xd7\xa3=\n\xd7\xa3;\n\xd7\xa3=\n\xd7\xa3=\n\xd7\xa3;\x8f\xc2u\xbd\x00\x00\x00\x00\n\xd7\xa3;\x8f\xc2u=\x00\x00\x00\x00\n\xd7\xa3;\n\xd7\xa3\xbd\n\xd7\xa3\xbd\n\xd7\xa3;\n\xd7\xa3=\n\xd7\xa3\xbd\n\xd7\xa3;\n\xd7\xa3<\xcd\xcc\xcc=\n\xd7\xa3;'
MARKER_IDS = ['NW', 'NE', 'W', 'E', 'SW', 'SE', 'N']
OUT_JSONL = "/td_s10_b2_out/sentai_sim_results.jsonl"
OUT_SUMMARY = "/td_s10_b2_out/sentai_sim_summary.json"

def _det_json_tuple(t):
    radius_outer = t[17] if len(t) > 17 else 0.0
    return ('{"id":%d,"center":[%.3f,%.3f],'
            '"axis_a":%.6f,"axis_b":%.6f,"angle_rad":%.6f,'
            '"radius_outer":%.6f,'
            '"comp_id":%d,'
            '"tvec_cam":[%.6f,%.6f,%.6f],'
            '"rvec_cam":[%.6f,%.6f,%.6f],'
            '"reproj_err_px":%.6f,"backend":%d,"pose_valid":%d,'
            '"geometry_valid":%d}') % (
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
    sentai.markers.set_intrinsics(288.3, 288.3, 160.0, 120.0)
    sentai.markers.set_marker_size(0.0544)
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
            pose_json = ('{"x":%.6f,"y":%.6f,"z":%.6f,'
                         '"yaw_rad":%.6f,"res_max":%.6f,'
                         '"n_used":%d,"flip_x":%d,"flip_y":%d,'
                         '"flip_z":%d}') % (
                dp[0], dp[1], dp[2], dp[3], dp[4],
                dp[5], dp[6], dp[7], dp[8])
        line = '{"frame":"%s","rc":%d,"detections":[%s],"drone_pose_world":%s}\n' % (
            frame, n, ",".join(dets), pose_json)
        sentai.fs.append(OUT_JSONL, line)
        total += n
        if n > 0:
            frames_ok += 1
    summary = '{"backend":"sentai_sim.markers.whycon","init_rc":%d,'                       '"marker_world_rc":%d,"marker_world_count":%d,'                       '"fx":%.6f,"fy":%.6f,"cx":%.6f,"cy":%.6f,'                       '"marker_diameter_m":%.6f,'                       '"frames":%d,"frames_with_detections":%d,'                       '"detections_total":%d,"frames_with_drone_pose":%d,'                       '"marker_ids":"%s"}\n' % (
                  rc, world_rc, world_n,
                  288.3, 288.3, 160.0, 120.0,
                  0.0544,
                  len(FRAMES), frames_ok, total, pose_ok,
                  ",".join(MARKER_IDS))
    sentai.fs.write(OUT_SUMMARY, summary)
    print("TD-S10-B2_SENTAI_SIM_DONE", len(FRAMES), total)

main()
