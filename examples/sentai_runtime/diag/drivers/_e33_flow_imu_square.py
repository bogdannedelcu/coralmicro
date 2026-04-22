# _e33_flow_imu_square.py — E32 square walk + synchronous IMU sampling.
#
# Goal: determine how LIS2DU12 IMU axes (x_mg, y_mg, z_mg) map onto
# the drone body frame (FW, L, UP) established by the cameras
# (paper/flow_body_frame.md).  Each sample records:
#
#   body_fw, body_left  — from M4 optical flow (velocity proxy)
#   imu_x, imu_y, imu_z — raw accel, mg (milli-g)
#
# During SOLID phases (MOVE_FW / MOVE_R / MOVE_BACK / MOVE_L), one
# body axis is being actively accelerated while the other two are
# quiet.  Comparing which IMU axis spikes with which move tells us
# the rotation matrix IMU→body.  Expected outcome (to verify):
#
#   MOVE_FW    → one IMU axis pulses + then − (accel then brake)
#   MOVE_R     → another IMU axis pulses
#   MOVE_BACK  → same axis as FW, opposite sign
#   MOVE_L     → same axis as R, opposite sign
#   Z axis stays ≈ +1000 mg (gravity) throughout (cameras down)
#
# LED protocol identical to E32:  FLICKER = stai, SOLID = misca.

import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)

# IMU init — LIS2DU12 needs I2C handshake before .read() works.
try: sentai.imu.init()
except Exception: pass

sentai.io.led_off()

from diag._session import _session, begin, end, _save_path, _record, _save_desc
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned:
    begin("e33_flow_imu_square")

POLL_MS = 20

rc = sentai.flow.m4_enable()
hb = sentai.flow.m4_heartbeat()
print("[E33] m4_enable rc=%d alive=%s" % (rc, hb["alive"]))
if not hb["alive"]:
    print("[E33] ABORT: M4 not alive")
    if owned: end()
    raise SystemExit

sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)
sentai.flow.m4_start(0)
sentai.rtos.sleep_ms(200)

PHASES = [
    ("WAIT_start", 2000, "flicker"),
    ("MOVE_FW",    4000, "solid"),
    ("WAIT_c1",    2000, "flicker"),
    ("MOVE_R",     4000, "solid"),
    ("WAIT_c2",    2000, "flicker"),
    ("MOVE_BACK",  4000, "solid"),
    ("WAIT_c3",    2000, "flicker"),
    ("MOVE_L",     4000, "solid"),
    ("WAIT_end",   2000, "flicker"),
]

print("[E33] FLICKER=stai, SOLID=misca.  3..2..1..")
for _ in range(3):
    sentai.io.led_on();  sentai.rtos.sleep_ms(100)
    sentai.io.led_off(); sentai.rtos.sleep_ms(900)

samples = []   # (t_ms, phase_idx, phase, seq, dx, dy, bfw, bleft,
               #  pos_fw, pos_left, conf, imu_x, imu_y, imu_z)
pos_fw = pos_left = 0
last_seq = 0
t0 = _ticks()

from diag._session import _photos_dir
shots_dir = _photos_dir("e33_shots") or "/diags/e33_shots"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

def _flick(pstart, now):
    on = (((now - pstart) // 100) & 1) == 0
    if on: sentai.io.led_on()
    else:  sentai.io.led_off()

def _capture_corner(tag):
    """At each WAIT phase, grab one cam0 JPEG + one 40×30 gray snap
    (same buffer the SAD algorithm sees).  Visual inspection on the
    host confirms whether the gray has the spatial structure needed
    for Y-axis matching; if it's vertically uniform, dy stays 0
    regardless of real motion.
    """
    try:
        jpg = sentai.camera.jpeg(80)
        p_jpg = "%s/%s_cam0.jpg" % (shots_dir, tag)
        sentai.fs.write(p_jpg, jpg)
        jpg = None; gc.collect()
    except Exception as e:
        print("  WARN jpeg %s: %s" % (tag, e))
    try:
        gray = sentai.flow.m4_gray_snap()
        p_gray = "%s/%s_gray40x30.pgm" % (shots_dir, tag)
        hdr = b"P5\n40 30\n255\n"
        sentai.fs.write(p_gray, hdr + gray)
    except Exception as e:
        print("  WARN gray %s: %s" % (tag, e))

for phase_idx, (name, dur, mode) in enumerate(PHASES):
    print("[E33] phase %d %s (%d s, %s)" %
          (phase_idx, name, dur // 1000, mode))
    pstart = _ticks()
    if mode == "solid":
        sentai.io.led_on()
    shot_taken = False
    while _ticks() - pstart < dur:
        if mode == "flicker":
            _flick(pstart, _ticks())
            # One visual shot per WAIT phase, ~1s in, when user
            # should be holding still.
            if (not shot_taken) and (_ticks() - pstart > 1000):
                _capture_corner(name)
                shot_taken = True
        r = sentai.flow.m4_body_read()
        if r["alive"] and r["frame_seq"] != last_seq and r["frame_seq"] != 0:
            last_seq = r["frame_seq"]
            dx, dy = r["dx"], r["dy"]
            bfw, bleft = r["body_fw"], r["body_left"]
            pos_fw   += bfw
            pos_left += bleft
            r2 = sentai.flow.m4_read()
            conf = r2["confidence"]
            try:
                im = sentai.imu.read()     # {'x','y','z','temp'} in mg
                imx = int(im["x"])
                imy = int(im["y"])
                imz = int(im["z"])
            except Exception:
                imx = imy = imz = 0
            samples.append((_ticks() - t0, phase_idx, name, last_seq,
                            dx, dy, bfw, bleft, pos_fw, pos_left, conf,
                            imx, imy, imz))
        sentai.rtos.sleep_ms(POLL_MS)

sentai.io.led_off()
sentai.flow.m4_stop()
sentai.pipeline.stop()

n = len(samples)
m4 = sentai.flow.m4_read()
print("[E33] N=%d  frames_processed_m4=%d" % (n, m4["frames_processed"]))
if n:
    fwmax = max(s[8] for s in samples); fwmin = min(s[8] for s in samples)
    lfmax = max(s[9] for s in samples); lfmin = min(s[9] for s in samples)
    print("  pos_fw   range [%d .. %d]" % (fwmin, fwmax))
    print("  pos_left range [%d .. %d]" % (lfmin, lfmax))
    print("  close-loop: pos_fw=%d pos_left=%d"
          % (samples[-1][8], samples[-1][9]))
    # Summary per-axis IMU during each SOLID phase.
    for target in ("MOVE_FW", "MOVE_R", "MOVE_BACK", "MOVE_L"):
        phase_samples = [s for s in samples if s[2] == target]
        if not phase_samples: continue
        xs = [s[11] for s in phase_samples]
        ys = [s[12] for s in phase_samples]
        zs = [s[13] for s in phase_samples]
        def _pp(l): return (max(l) - min(l), sum(l) // len(l))
        dx, mx = _pp(xs); dy, my = _pp(ys); dz, mz = _pp(zs)
        print("  %s: IMU x range=%d mean=%d | y range=%d mean=%d | "
              "z range=%d mean=%d" % (target, dx, mx, dy, my, dz, mz))

csv_path = _save_path("e33_flow_imu_square")
save_csv(csv_path,
    ["t_ms", "phase_idx", "phase", "frame_seq",
     "dx", "dy", "body_fw", "body_left",
     "pos_fw", "pos_left", "confidence",
     "imu_x_mg", "imu_y_mg", "imu_z_mg"],
    samples)
_save_desc(csv_path,
    "E33 - LED-guided square walk with synchronous IMU accel log.\n"
    "FLICKER=stai, SOLID=misca.  Goal: map IMU axes to drone body.\n"
    "During each MOVE_* phase, identify which IMU axis pulses.\n",
    params={"poll_ms": POLL_MS, "samples": n,
            "m4_frames_processed": m4["frames_processed"]})
_record("e33_flow_imu_square", csv_path, "N=%d" % n)

if owned: end()
