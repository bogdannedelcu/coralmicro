# _e32_flow_square_guided.py — LED-guided square-walk optical-flow trace.
#
# Protocol: the LED alternates between FLICKER ("stai pe loc" — don't
# move, I'm at a corner) and SOLID ("mișcă" — translate to the next
# corner).  The phases mark one full square loop of the drone,
# starting and ending at the same point:
#
#     Phase    LED      Duration   Action
#     ─────    ───      ────────   ──────
#     WAIT_0   flicker  2 s        hold at START corner
#     FW       solid    4 s        move FORWARD  ≈15 cm
#     WAIT_1   flicker  2 s        hold at corner 1
#     R        solid    4 s        move RIGHT    ≈15 cm
#     WAIT_2   flicker  2 s        hold at corner 2
#     BACK     solid    4 s        move BACKWARD ≈15 cm
#     WAIT_3   flicker  2 s        hold at corner 3
#     L        solid    4 s        move LEFT     ≈15 cm (→ back to start)
#     WAIT_4   flicker  2 s        hold at START corner (close loop)
#
# Total = 26 s.  Every sample records a `phase` tag so the host-side
# plotter can colour-code the four corners distinctly, showing the
# characteristic "4 clusters + 4 edges" square signature.

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
    begin("e32_flow_square")

POLL_MS = 20

# Bring up M4 flow.
rc = sentai.flow.m4_enable()
hb = sentai.flow.m4_heartbeat()
print("[E32] m4_enable rc=%d alive=%s" % (rc, hb["alive"]))
if not hb["alive"]:
    print("[E32] ABORT: M4 not alive")
    if owned: end()
    raise SystemExit

sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)
sentai.flow.m4_start(0)   # cam0
sentai.rtos.sleep_ms(200)

# Phase schedule: (name, duration_ms, led_mode)
#   led_mode: "solid" or "flicker"
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

print("[E32] protocol: FLICKER = stai pe loc, SOLID = misca")
print("[E32] 3..2..1..")
for n in (3, 2, 1):
    sentai.io.led_on();  sentai.rtos.sleep_ms(100)
    sentai.io.led_off(); sentai.rtos.sleep_ms(900)

# ── main loop — run all phases, tag every sample with current phase ──
samples = []        # (t_ms, phase_idx, phase_name, frame_seq, dx, dy,
                    #  body_fw, body_left, pos_fw, pos_left, conf)
pos_fw   = 0
pos_left = 0
last_seq = 0
t0       = _ticks()

def _flick_step(t_phase_start, now):
    # 5 Hz blink — LED state = (elapsed ms / 100) % 2
    elapsed = now - t_phase_start
    on = ((elapsed // 100) & 1) == 0
    if on: sentai.io.led_on()
    else:  sentai.io.led_off()

for phase_idx, (name, dur_ms, mode) in enumerate(PHASES):
    print("[E32] phase %d  %s  (%d s, %s)" %
          (phase_idx, name, dur_ms // 1000, mode))
    phase_start = _ticks()
    if mode == "solid":
        sentai.io.led_on()
    while _ticks() - phase_start < dur_ms:
        if mode == "flicker":
            _flick_step(phase_start, _ticks())
        r = sentai.flow.m4_body_read()
        if r["alive"] and r["frame_seq"] != last_seq and r["frame_seq"] != 0:
            last_seq = r["frame_seq"]
            dx, dy = r["dx"], r["dy"]
            bfw    = r["body_fw"]
            bleft  = r["body_left"]
            pos_fw   += bfw
            pos_left += bleft
            r2   = sentai.flow.m4_read()
            conf = r2["confidence"]
            samples.append((_ticks() - t0, phase_idx, name, last_seq,
                            dx, dy, bfw, bleft, pos_fw, pos_left, conf))
        sentai.rtos.sleep_ms(POLL_MS)

sentai.io.led_off()
sentai.flow.m4_stop()
sentai.pipeline.stop()

n = len(samples)
m4 = sentai.flow.m4_read()
print("[E32] N=%d  frames_processed_m4=%d" % (n, m4["frames_processed"]))
if n:
    fwmax = max(s[8] for s in samples); fwmin = min(s[8] for s in samples)
    lfmax = max(s[9] for s in samples); lfmin = min(s[9] for s in samples)
    print("  pos_fw   range [%d .. %d]" % (fwmin, fwmax))
    print("  pos_left range [%d .. %d]" % (lfmin, lfmax))
    print("  close-loop error: pos_fw=%d pos_left=%d"
          % (samples[-1][8], samples[-1][9]))

csv_path = _save_path("e32_flow_square")
save_csv(csv_path,
    ["t_ms", "phase_idx", "phase_name", "frame_seq",
     "dx", "dy", "body_fw", "body_left",
     "pos_fw", "pos_left", "confidence"],
    samples)
_save_desc(csv_path,
    "E32 - LED-guided square walk.\n"
    "FLICKER phases = stay still at corner, SOLID phases = move.\n"
    "Phase sequence: WAIT_start, FW, WAIT_c1, R, WAIT_c2, BACK,\n"
    "                WAIT_c3, L, WAIT_end.\n"
    "Ideal output on plot: 4 distinct clusters at corners connected\n"
    "by 4 edges; close-loop error should be small.\n",
    params={"poll_ms": POLL_MS, "samples": n,
            "m4_frames_processed": m4["frames_processed"],
            "phases": len(PHASES)})
_record("e32_flow_square", csv_path, "N=%d" % n)

if owned: end()
