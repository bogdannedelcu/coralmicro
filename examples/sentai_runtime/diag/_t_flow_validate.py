# _t_flow_validate.py -- real-life Flow validation driver (M7-only).
#
# Operator protocol (LED user is the metronome AND phase marker):
#   START : 10 fast blinks         -> "experiment about to start"
#   SIDE i MOVE : LED solid ON     -> walk to corner i (one square side)
#   SIDE i HOLD : LED solid OFF    -> stop on corner i
#   ... NUM_SIDES = 4 cycles total, ending back at the start point
#   END   : 10 fast blinks         -> done
#
# Goal: closing a SQUARE -- after 4 sides cumulative (cum_dx, cum_dy)
# should be small (closure error).
#
# Outputs (under /diags/sNNN_flow_validate/):
#   bulk_gray.bin     -- when BULK_GRAY_CAPTURE=1: header + 4800 B gray
#                          per frame (host parses + replays SAD offline)
#   trace.csv         -- when BULK_GRAY_CAPTURE=0: per-sample CSV
#   params.txt        -- snapshot of run parameters
#   summary.txt       -- high-level pass/fail metrics
#   scene_start.jpg   -- one-shot debug capture before SIDE 1
#   scene_end.jpg     -- one-shot debug capture after SIDE NUM_SIDES

import sentai

# === parameters (edit then re-upload) ===
TARGET_HZ         = 50
CAM_ID            = 0
GRAY_STRETCH      = 0
USE_IMU           = 1
NUM_SIDES         = 4
MOVE_MS           = 2500
HOLD_MS           = 1500
ATTN_BLINKS       = 10
ATTN_BLINK_MS     = 100
SCENE_JPEG_QUAL   = 70
SCENE_CAPTURE     = 1
SCENE_SNAP_COUNT  = 0     # 0 = no mid-run snaps (timing-clean)
ENABLE_HTTP       = 1
BULK_GRAY_CAPTURE = 1     # bulk-capture mode for offline replay


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception:
        sid = 1
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d, sid


def _build_id():
    try:
        return sentai.version().split("build")[1].split()[0]
    except Exception:
        return "?"


def _led_blink(n, on_ms=60, off_ms=60):
    for _ in range(n):
        sentai.io.led_on()
        sentai.rtos.sleep_ms(on_ms)
        sentai.io.led_off()
        sentai.rtos.sleep_ms(off_ms)


def _save_jpeg(path, qual):
    try:
        sentai.camera.save_jpeg(path, qual)
        return 1
    except Exception as e:
        print("[scene-jpeg] skip:", e)
        return 0


GRAY_W = 80
GRAY_H = 60
def _gray_snap_to_pgm(path):
    data = sentai.flow.gray_snap()
    if len(data) != GRAY_W * GRAY_H:
        return 0
    sentai.fs.write(path, b"P5\n80 60\n255\n")
    sentai.fs.append(path, data)
    return 1


def main():
    sess, sid = _session_dir("flow_validate")
    bid = _build_id()
    print("session:", sess, "build:", bid)

    sentai.verbose(1)

    if ENABLE_HTTP:
        try:
            sentai.usb.ip(1)
            print("usb.ip(1) -> HTTP available at http://10.0.0.1/")
        except Exception as e:
            print("usb.ip(1) failed:", e)

    rc = sentai.flow.enable()
    print("flow.enable:", rc)
    if rc < 0:
        print("FAIL flow.enable=%d" % rc)
        print("=== done ==="); return

    rc = sentai.camera.init(1)
    print("camera.init:", rc)
    if rc != 0 and rc != -11:
        print("FAIL camera.init=%d" % rc)
        print("=== done ==="); return
    sentai.camera.select(CAM_ID)

    sentai.flow.gray_stretch(bool(GRAY_STRETCH))

    rc = sentai.flow.start(CAM_ID)
    print("flow.start:", rc)

    if USE_IMU:
        irc = sentai.imu.init()
        print("imu.init:", irc)

    params = (
        "build=%s sid=%d cam=%d sides=%d move_ms=%d hold_ms=%d hz=%d "
        "gray_stretch=%d use_imu=%d scene_capture=%d bulk=%d\n"
    ) % (bid, sid, CAM_ID, NUM_SIDES, MOVE_MS, HOLD_MS, TARGET_HZ,
         GRAY_STRETCH, USE_IMU, SCENE_CAPTURE, BULK_GRAY_CAPTURE)
    sentai.fs.write(sess + "/params.txt", params)
    print("params:", params.strip())

    print("START signal: %d blinks" % ATTN_BLINKS)
    _led_blink(ATTN_BLINKS, on_ms=ATTN_BLINK_MS, off_ms=ATTN_BLINK_MS)
    sentai.rtos.sleep_ms(300)

    if SCENE_CAPTURE:
        _save_jpeg(sess + "/scene_start.jpg", SCENE_JPEG_QUAL)
        _gray_snap_to_pgm(sess + "/gray_start.pgm")

    csv_path  = sess + "/trace.csv"
    bulk_path = sess + "/bulk_gray.bin"
    if BULK_GRAY_CAPTURE:
        sentai.diag.cache_open(bulk_path)
        sentai.diag.cache_write(
            "BULK_GRAY_V1 GW=80 GH=60 record='F idx t_ms seq dx_q1000 "
            "dy_q1000 conf phase side_idx ax ay az detail\\n<4800 raw bytes>'\n")
    else:
        sentai.diag.cache_open(csv_path)
        sentai.diag.cache_write(
            "t_ms,frame_seq,dx_q1000,dy_q1000,sad,conf,"
            "ax_mg,ay_mg,az_mg,detail,phase,side_idx\n")

    period_ms = int(1000 / TARGET_HZ)
    n_rows = 0
    t_loop_start = sentai.rtos.ticks_ms()

    last_logged_seq = [-1]   # dedup: only log when M7 SAD has a new frame
    def sample_block(phase, dur_ms, side_idx, n_rows_in):
        n = n_rows_in
        block_t0  = sentai.rtos.ticks_ms()
        block_end = block_t0 + dur_ms
        next_tick = block_t0
        printed_first = False
        while True:
            now = sentai.rtos.ticks_ms()
            if now >= block_end:
                break
            d = sentai.flow.read()
            if d["frame_seq"] == last_logged_seq[0]:
                next_tick += period_ms
                sleep = next_tick - sentai.rtos.ticks_ms()
                if sleep > 0:
                    sentai.rtos.sleep_ms(sleep)
                else:
                    next_tick = sentai.rtos.ticks_ms()
                continue
            last_logged_seq[0] = d["frame_seq"]
            ax = ay = az = 0.0
            if USE_IMU:
                r = sentai.imu.read()
                if r is not None:
                    ax = r["x"]; ay = r["y"]; az = r["z"]
            detail = sentai.flow.detail_score()

            if BULK_GRAY_CAPTURE:
                ph = "M" if phase == "MOVE" else "H"
                hdr = ("F %d %d %d %d %d %d %s %d %.1f %.1f %.1f %d\n" % (
                    n, now - t_loop_start, d["frame_seq"],
                    d["dx"], d["dy"], d["confidence"],
                    ph, side_idx, ax, ay, az, detail))
                sentai.diag.cache_write(hdr)
                sentai.flow.gray_to_cache()
            else:
                line = "%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%d,%s,%d\n" % (
                    now - t_loop_start, d["frame_seq"], d["dx"], d["dy"],
                    d["sad"], d["confidence"],
                    ax, ay, az, detail, phase, side_idx)
                sentai.diag.cache_write(line)
            n += 1

            if not printed_first:
                printed_first = True
                try: sentai.diag.repl_kick()
                except Exception: pass
                print("[t=%5d ms] side=%d phase=%s seq=%d dx=%d dy=%d conf=%d cache_len=%d" % (
                    now - t_loop_start, side_idx, phase,
                    d["frame_seq"], d["dx"], d["dy"],
                    d["confidence"], sentai.diag.cache_len()))

            next_tick += period_ms
            sleep = next_tick - sentai.rtos.ticks_ms()
            if sleep > 0:
                sentai.rtos.sleep_ms(sleep)
            else:
                next_tick = sentai.rtos.ticks_ms()
        return n

    for side in range(NUM_SIDES):
        sentai.io.led_on()
        n_rows = sample_block("MOVE", MOVE_MS, side, n_rows)
        sentai.io.led_off()
        n_rows = sample_block("HOLD", HOLD_MS, side, n_rows)

    csv_bytes   = sentai.diag.cache_save()
    csv_dropped = sentai.diag.cache_dropped()
    sentai.diag.cache_close()
    sentai.io.led_off()

    print("END signal: %d blinks" % ATTN_BLINKS)
    _led_blink(ATTN_BLINKS, on_ms=ATTN_BLINK_MS, off_ms=ATTN_BLINK_MS)

    if SCENE_CAPTURE:
        _save_jpeg(sess + "/scene_end.jpg", SCENE_JPEG_QUAL)
        _gray_snap_to_pgm(sess + "/gray_end.pgm")

    sentai.flow.stop()

    total_ms = NUM_SIDES * (MOVE_MS + HOLD_MS)
    summary = (
        "build=%s sid=%d rows=%d csv_bytes=%d csv_dropped=%d "
        "sides=%d total_ms=%d target_hz=%d effective_hz=%.2f\n"
    ) % (bid, sid, n_rows, csv_bytes, csv_dropped,
         NUM_SIDES, total_ms, TARGET_HZ,
         (n_rows * 1000.0) / total_ms)
    sentai.fs.write(sess + "/summary.txt", summary)
    print(summary.strip())
    print("session=%s rows=%d" % (sess, n_rows))
    print("=== done ===")


main()
