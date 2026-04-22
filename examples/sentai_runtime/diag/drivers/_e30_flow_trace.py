# _e30_flow_trace.py — 2D drone-motion trace via M4 optical flow.
#
# Runs the M4 SAD flow + M7 PrepTask on cam0 for ~20 s.  LED ON
# signals "start moving the drone" — fw/back/left/right in arbitrary
# order, trying to draw a recognisable trajectory (letter, square,
# cross, whatever).  Samples (body_fw, body_left) at ~50 Hz, sums
# them into cumulative (pos_fw, pos_left) in grid-pixel units,
# writes a CSV for host-side matplotlib to render.
#
# Output columns (per sample):
#   t_ms            wall time since LED_ON
#   frame_seq       M4 frame seq produced this sample
#   dx, dy          raw image-frame delta (pixels in 40×30 grid)
#   body_fw         +1 = drone moved forward, cam0 convention
#   body_left       +1 = drone moved left
#   pos_fw          cumulative sum of body_fw  — x axis on the plot
#   pos_left        cumulative sum of body_left — y axis
#   confidence      0..255 from SAD match quality

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

sentai.io.led_off()

from diag._session import _session, begin, end, _save_path, _record, _save_desc
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned:
    begin("e30_flow_trace")

DURATION_MS = 20000
POLL_MS     = 20       # 50 Hz target sample rate

# Bring up the M4 and start consuming frames.
rc = sentai.flow.m4_enable()
hb = sentai.flow.m4_heartbeat()
print("[E30] m4_enable rc=%d alive=%s" % (rc, hb["alive"]))
if not hb["alive"]:
    print("[E30] ABORT: M4 not alive")
    if owned: end()
    raise SystemExit

sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)   # warmup
sentai.flow.m4_start()

# Give the M4 a couple of frames to establish a baseline before
# counting towards cumulative motion.
sentai.rtos.sleep_ms(200)

# ── LED flicker cue — tactile signal to the operator ────────────
# Instead of relying on a serial countdown that the user may not
# be watching, do three rapid LED blinks immediately before the
# steady ON that marks "record starts now".  The rhythm (blink-
# blink-blink, then solid ON for 20s, then OFF) is unambiguous
# from across the room.
print("[E30] get ready — LED will flicker 3x then stay solid")
for _ in range(3):
    sentai.io.led_on();  sentai.rtos.sleep_ms(120)
    sentai.io.led_off(); sentai.rtos.sleep_ms(200)
sentai.rtos.sleep_ms(400)

sentai.io.led_on()
print("[E30] LED SOLID — move the drone!  (%d s)" % (DURATION_MS // 1000))

samples = []      # (t, seq, dx, dy, body_fw, body_left, pos_fw, pos_left, conf)
pos_fw   = 0
pos_left = 0
last_seq = 0
t0 = _ticks()

while _ticks() - t0 < DURATION_MS:
    r = sentai.flow.m4_body_read()
    if r["alive"] and r["frame_seq"] != last_seq and r["frame_seq"] != 0:
        last_seq  = r["frame_seq"]
        dx        = r["dx"]
        dy        = r["dy"]
        bfw       = r["body_fw"]
        bleft     = r["body_left"]
        pos_fw   += bfw
        pos_left += bleft
        r2 = sentai.flow.m4_read()
        conf = r2["confidence"]
        samples.append((_ticks() - t0, last_seq,
                        dx, dy, bfw, bleft,
                        pos_fw, pos_left, conf))
    sentai.rtos.sleep_ms(POLL_MS)

sentai.io.led_off()
sentai.flow.m4_stop()
sentai.pipeline.stop()

n = len(samples)
m4 = sentai.flow.m4_read()
print("[E30] N=%d  frames_processed_m4=%d" % (n, m4["frames_processed"]))
if n:
    fwmax = max(s[6] for s in samples); fwmin = min(s[6] for s in samples)
    lfmax = max(s[7] for s in samples); lfmin = min(s[7] for s in samples)
    print("  pos_fw   range [%d .. %d]" % (fwmin, fwmax))
    print("  pos_left range [%d .. %d]" % (lfmin, lfmax))

csv_path = _save_path("e30_flow_trace")
save_csv(csv_path,
    ["t_ms", "frame_seq", "dx", "dy",
     "body_fw", "body_left", "pos_fw", "pos_left", "confidence"],
    samples)
_save_desc(csv_path,
    "E30 - M4 optical-flow 2D trajectory trace.\n"
    "Cam0 body-frame convention per paper/flow_body_frame.md:\n"
    "  body_fw   = -dx    body_left = +dy\n"
    "pos_* are cumulative sums intended for 2D plot on host.\n",
    params={"duration_ms": DURATION_MS, "poll_ms": POLL_MS,
            "samples": n,
            "m4_frames_processed": m4["frames_processed"]})
_record("e30_flow_trace", csv_path,
        "N=%d" % n)

if owned: end()
