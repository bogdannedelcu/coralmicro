import sentai

FRAMES = ['/td_s10_b1_dataset/frames/frame_000000_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw000_n6.pgm', '/td_s10_b1_dataset/frames/frame_000001_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw045_n4.pgm', '/td_s10_b1_dataset/frames/frame_000002_x-0.500_y+0.000_z0.750_r+4.0_p-3.0_yaw315_n4.pgm']
OUT_JSONL = "/td_s10_b2_out/sentai_sim_results.jsonl"
OUT_SUMMARY = "/td_s10_b2_out/sentai_sim_summary.json"

def _det_json_tuple(t):
    return ('{"id":%d,"center":[%.3f,%.3f],'
            '"axis_a":%.6f,"axis_b":%.6f,"angle_rad":%.6f,'
            '"comp_id":%d,'
            '"tvec_cam":[%.6f,%.6f,%.6f],'
            '"rvec_cam":[%.6f,%.6f,%.6f],'
            '"reproj_err_px":%.6f,"backend":%d,"pose_valid":%d,'
            '"geometry_valid":%d}') % (
        t[0], t[1], t[2],
        t[3], t[4], t[5],
        t[6],
        t[7], t[8], t[9],
        t[10], t[11], t[12],
        t[13], t[14], t[15], t[16])

def main():
    sentai.markers.clear()
    rc = sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(160.0, 160.0, 160.0, 120.0)
    sentai.markers.set_marker_size(0.1088)
    sentai.fs.write(OUT_JSONL, "")
    total = 0
    frames_ok = 0
    for frame in FRAMES:
        n = sentai.markers.detect_pgm(frame)
        dets = []
        for i in range(n):
            t = sentai.markers.get_detection_tuple(i)
            if t is not None:
                dets.append(_det_json_tuple(t))
        line = '{"frame":"%s","rc":%d,"detections":[%s]}\n' % (
            frame, n, ",".join(dets))
        sentai.fs.append(OUT_JSONL, line)
        total += n
        if n > 0:
            frames_ok += 1
    summary = '{"backend":"sentai_sim.markers.whycon","init_rc":%d,'                       '"frames":%d,"frames_with_detections":%d,'                       '"detections_total":%d}\n' % (
                  rc, len(FRAMES), frames_ok, total)
    sentai.fs.write(OUT_SUMMARY, summary)
    print("TD-S10-B2_SENTAI_SIM_DONE", len(FRAMES), total)

main()
