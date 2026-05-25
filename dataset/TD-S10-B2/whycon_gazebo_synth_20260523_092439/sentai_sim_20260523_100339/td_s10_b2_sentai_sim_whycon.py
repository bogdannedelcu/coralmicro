import sentai

FRAMES = ['/td_s10_b1_dataset/frames/frame_000000_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw000_n6.pgm', '/td_s10_b1_dataset/frames/frame_000001_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw045_n4.pgm', '/td_s10_b1_dataset/frames/frame_000002_x-0.500_y+0.000_z0.750_r+4.0_p-3.0_yaw315_n4.pgm', '/td_s10_b1_dataset/frames/frame_000003_x-0.500_y-0.250_z1.000_r+0.0_p+0.0_yaw135_n4.pgm', '/td_s10_b1_dataset/frames/frame_000004_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw000_n6.pgm', '/td_s10_b1_dataset/frames/frame_000005_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw225_n4.pgm', '/td_s10_b1_dataset/frames/frame_000006_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw000_n4.pgm', '/td_s10_b1_dataset/frames/frame_000007_x-0.500_y-0.250_z1.000_r+4.0_p-3.0_yaw270_n6.pgm', '/td_s10_b1_dataset/frames/frame_000008_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw045_n6.pgm', '/td_s10_b1_dataset/frames/frame_000009_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw270_n4.pgm', '/td_s10_b1_dataset/frames/frame_000010_x-0.250_y-0.250_z0.750_r+4.0_p-3.0_yaw000_n6.pgm', '/td_s10_b1_dataset/frames/frame_000011_x-0.500_y-0.250_z1.000_r+0.0_p+0.0_yaw315_n4.pgm', '/td_s10_b1_dataset/frames/frame_000012_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000013_x-0.250_y+0.000_z0.500_r+4.0_p-3.0_yaw315_n5.pgm', '/td_s10_b1_dataset/frames/frame_000014_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw090_n4.pgm', '/td_s10_b1_dataset/frames/frame_000015_x-0.500_y-0.250_z1.000_r+4.0_p-3.0_yaw315_n6.pgm', '/td_s10_b1_dataset/frames/frame_000016_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw090_n6.pgm', '/td_s10_b1_dataset/frames/frame_000017_x+0.000_y-0.250_z0.500_r+0.0_p+0.0_yaw000_n4.pgm', '/td_s10_b1_dataset/frames/frame_000018_x-0.250_y-0.250_z0.750_r+4.0_p-3.0_yaw090_n6.pgm', '/td_s10_b1_dataset/frames/frame_000019_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000020_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw090_n6.pgm', '/td_s10_b1_dataset/frames/frame_000021_x+0.000_y-0.250_z0.500_r+4.0_p-3.0_yaw000_n4.pgm', '/td_s10_b1_dataset/frames/frame_000022_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw135_n5.pgm', '/td_s10_b1_dataset/frames/frame_000023_x-0.500_y+0.000_z1.000_r+4.0_p-3.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000024_x+0.000_y+0.000_z0.300_r+0.0_p+0.0_yaw135_n6.pgm', '/td_s10_b1_dataset/frames/frame_000025_x+0.000_y-0.250_z0.500_r+4.0_p-3.0_yaw045_n5.pgm', '/td_s10_b1_dataset/frames/frame_000026_x-0.250_y-0.250_z0.750_r+0.0_p+0.0_yaw180_n4.pgm', '/td_s10_b1_dataset/frames/frame_000027_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw135_n5.pgm', '/td_s10_b1_dataset/frames/frame_000028_x+0.000_y+0.000_z0.300_r+4.0_p-3.0_yaw135_n5.pgm', '/td_s10_b1_dataset/frames/frame_000029_x+0.000_y-0.250_z0.500_r+4.0_p-3.0_yaw135_n4.pgm']
OUT_JSONL = "/td_s10_b2_out/sentai_sim_results.jsonl"
OUT_SUMMARY = "/td_s10_b2_out/sentai_sim_summary.json"

def _is_ws(c):
    return c == 32 or c == 10 or c == 13 or c == 9

def _next_token(data, idx):
    n = len(data)
    while idx < n:
        c = data[idx]
        if _is_ws(c):
            idx += 1
            continue
        if c == 35:
            while idx < n and data[idx] != 10:
                idx += 1
            continue
        break
    start = idx
    while idx < n and not _is_ws(data[idx]):
        idx += 1
    return data[start:idx], idx

def _pgm_payload(path):
    data = sentai.fs.read(path)
    magic, idx = _next_token(data, 0)
    w_s, idx = _next_token(data, idx)
    h_s, idx = _next_token(data, idx)
    max_s, idx = _next_token(data, idx)
    while idx < len(data) and _is_ws(data[idx]):
        idx += 1
    if magic != b"P5":
        raise ValueError("not P5 PGM")
    w = int(w_s)
    h = int(h_s)
    maxv = int(max_s)
    if maxv != 255:
        raise ValueError("PGM max value must be 255")
    return data[idx:idx + w * h], w, h

def _det_json_tuple(t):
    return ('{"id":%d,"center":[%.3f,%.3f],'
            '"tvec_cam":[%.6f,%.6f,%.6f],'
            '"rvec_cam":[%.6f,%.6f,%.6f],'
            '"reproj_err_px":%.6f,"backend":%d,"pose_valid":%d}') % (
        t[0], t[1], t[2],
        t[3], t[4], t[5],
        t[6], t[7], t[8],
        t[9], t[10], t[11])

def main():
    sentai.markers.clear()
    rc = sentai.markers.init("whycon")
    sentai.markers.set_intrinsics(160.0, 160.0, 160.0, 120.0)
    sentai.markers.set_marker_size(0.1088)
    sentai.fs.write(OUT_JSONL, "")
    total = 0
    frames_ok = 0
    for frame in FRAMES:
        gray, w, h = _pgm_payload(frame)
        n = sentai.markers.detect_buffer(gray, w, h)
        dets = []
        for i in range(n):
            t = sentai.markers.get_pose_tuple(i)
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
