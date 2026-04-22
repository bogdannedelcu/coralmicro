# _e39_gray_stretch_sweep.py — prove that firmware histogram stretch
# actually widens the 80×60 gray buffer's dynamic range, regardless of
# which OV5640 ISP preset the sensor is running.
#
# E38 showed that gamma / SDE / AEC-target presets all produce
# indistinguishable mean brightness because the sensor's AEC feedback
# loop renormalizes the output histogram toward a fixed target.  The
# fix is post-capture auto-level on the decimated frame — this script
# is the ground-truth test.
#
# For each preset we grab TWO 80×60 gray snapshots from the shared
# buffer:
#   (a) stretch OFF — pure decimation output (what E38 looked at)
#   (b) stretch ON  — min-max stretched to [0..255]
# and record {mean, min, max, p05, p95, stddev×100} for each.  The
# progress metric is (max_on - min_on) - (max_off - min_off) — the
# widening of the dynamic range attributable to the stretch.  If that
# delta is consistently > 100 DN, the fix works as designed.
#
# Output:
#   .../NNN_e39_gray_stretch_frames/{preset}_{off,on}_gray80x60.pgm
#   .../NNN_e39_gray_stretch_frames/{preset}_{off,on}_cam0.jpg   (context)
#   CSV row per (preset, mode) with the six per-buffer stats

import sentai, gc

gc.collect()

PRESETS = [
    "nxp_stock",
    "bright_indoor",
    "daylight",
    "low_light",
]

# AEC settle budget after each preset apply.  OV5640 datasheet says
# AEC converges in ~15-30 frames; at VGA/45 fps that's ~350-700 ms.
# 1500 ms leaves margin for the exposure/gain curve to actually land.
AEC_SETTLE_MS = 1500

# Pipeline pause budget around the JPEG grab.  camera.jpeg() needs
# exclusive access; 100 ms gives it a clean frame without the PrepTask
# reclaiming the buffer mid-encode.
PIPELINE_PAUSE_MS = 100

# Post-restart settle before we trust the next frame.  The pipeline
# needs a frame to flow through PrepTask → PXP → publish before the
# shared gray buffer reflects the NEW preset, otherwise we'd sample
# one frame from the previous setup.
RESTART_SETTLE_MS = 400


def _gray_stats(gray):
    """Return (mean, vmin, vmax, p05, p95, stddev_x100) of a bytes-like
    gray buffer.  stddev is computed in integer fixed-point (×100) to
    avoid pulling in the float lib.
    """
    n = len(gray)
    if n == 0:
        return 0, 0, 0, 0, 0, 0
    # One pass for sum, min, max, sum_sq.  256-bin histogram for
    # percentiles.
    hist = [0] * 256
    s = 0
    ss = 0
    vmin = 255
    vmax = 0
    for b in gray:
        hist[b] += 1
        s += b
        ss += b * b
        if b < vmin: vmin = b
        if b > vmax: vmax = b
    mean = s // n
    # stddev² = E[x²] − E[x]²; keep integer by working in (×10000).
    var_x10000 = (ss * 10000 // n) - (mean * mean * 10000)
    if var_x10000 < 0: var_x10000 = 0
    # Integer sqrt via Newton to avoid math.sqrt.
    x = var_x10000
    r = x
    if r > 1:
        # 6 iterations is enough for 32-bit range
        for _ in range(6):
            if r == 0: break
            r = (r + x // r) // 2
    stddev_x100 = r  # sqrt of var_x10000 already scales by 100
    # Percentiles from the histogram.
    p05_target = (5 * n) // 100
    p95_target = (95 * n) // 100
    acc = 0
    p05 = 0
    p95 = 255
    for i in range(256):
        acc += hist[i]
        if acc >= p05_target:
            p05 = i
            break
    acc = 0
    for i in range(256):
        acc += hist[i]
        if acc >= p95_target:
            p95 = i
            break
    return mean, vmin, vmax, p05, p95, stddev_x100


if sentai.pipeline.running():
    sentai.pipeline.stop()
# TPU must be ready before pipeline.start() — the pipeline prereqs a
# loaded model in the M7-side interpreter.
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.select(0)

rc = sentai.flow.m4_enable()
print("[E39] m4_enable rc=%d" % rc)

sentai.pipeline.direct_tensor(1)
sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)
sentai.flow.m4_start(0)
sentai.rtos.sleep_ms(500)

from diag._session import _session, begin, end, _save_path, _record, _save_desc, _photos_dir
from diag._util import save_csv

owned = (_session is None)
if owned: begin("e39_gray_stretch")

shots_dir = _photos_dir("e39_gray_stretch") or "/diags/e39_gray_stretch"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

rows = []   # (preset, mode, mean, vmin, vmax, p05, p95, stddev_x100,
            #  detail_score, jpg_bytes)

# Make sure stretch is OFF at the start of each preset so we always
# measure the unmodified output first.
sentai.flow.gray_stretch(False)

for preset in PRESETS:
    rc_p = sentai.camera.isp_preset(0, preset)
    print("[E39] preset=%s rc=%d" % (preset, rc_p))
    sentai.rtos.sleep_ms(AEC_SETTLE_MS)

    for mode in ("off", "on"):
        sentai.flow.gray_stretch(mode == "on")
        # One frame period at VGA/45 is ~22 ms; wait a few so the
        # stretch flag change (read ACQUIRE inside the publisher) has
        # been observed and the buffer is stretched or not by the
        # intended rule.
        sentai.rtos.sleep_ms(120)

        # Detail-score reads from the same buffer after the publisher's
        # optional stretch pass — so it too reflects the mode.
        score = sentai.flow.detail_score()

        # Pause pipeline to grab a full-resolution JPEG for visual
        # inspection.  The JPEG path bypasses our gray buffer entirely
        # (it goes sensor → PXP → JPEG encoder) so the two JPEGs per
        # preset should be visually identical — they document the raw
        # scene the sensor put on the line.
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(PIPELINE_PAUSE_MS)
        jpg_bytes = 0
        try:
            jpg = sentai.camera.jpeg(80)
            jpg_bytes = len(jpg)
            sentai.fs.write("%s/%s_%s_cam0.jpg" %
                            (shots_dir, preset, mode), jpg)
            jpg = None; gc.collect()
        except Exception as e:
            print("  jpeg fail: %s" % e)

        # 80×60 gray snap AFTER pipeline stop — reads the last published
        # buffer, which still has the mode-appropriate stretch applied.
        gray = sentai.flow.m4_gray_snap()
        sentai.fs.write("%s/%s_%s_gray80x60.pgm" %
                        (shots_dir, preset, mode),
                        b"P5\n80 60\n255\n" + gray)

        mean, vmin, vmax, p05, p95, sd100 = _gray_stats(gray)
        gray = None; gc.collect()

        print("  %-15s %-3s  mean=%3d  min=%3d  max=%3d  p05=%3d  p95=%3d  "
              "sd=%.2f  detail=%d  jpg=%dB" %
              (preset, mode, mean, vmin, vmax, p05, p95,
               sd100 / 100.0, score, jpg_bytes))
        rows.append((preset, mode, mean, vmin, vmax, p05, p95,
                     sd100, score, jpg_bytes))

        # Restart pipeline so the next iteration has a live frame
        # stream.  Keep cam selection + preset unchanged.
        sentai.pipeline.prep_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        _ = sentai.pipeline.get_ex(2000)
        sentai.flow.m4_start(0)
        sentai.rtos.sleep_ms(RESTART_SETTLE_MS)

# Summary: for each preset compute dynamic-range widening.
print()
print("=== E39 SUMMARY (dynamic range off → on) ===")
per_preset = {}
for r in rows:
    per_preset.setdefault(r[0], {})[r[1]] = r
for preset in PRESETS:
    off = per_preset[preset]["off"]
    on  = per_preset[preset]["on"]
    range_off = off[4] - off[3]   # vmax - vmin
    range_on  = on[4]  - on[3]
    widen = range_on - range_off
    print("  %-15s  off:[%3d..%3d] (%3d)  on:[%3d..%3d] (%3d)  "
          "widen=%+4d  mean %3d→%3d" %
          (preset, off[3], off[4], range_off,
           on[3], on[4], range_on, widen,
           off[2], on[2]))

# Restore baseline + stretch off at the end.
sentai.camera.isp_preset(0, "nxp_stock")
sentai.flow.gray_stretch(False)
sentai.flow.m4_stop()
sentai.pipeline.stop()

csv_path = _save_path("e39_gray_stretch")
save_csv(csv_path,
    ["preset", "mode", "mean", "vmin", "vmax", "p05", "p95",
     "stddev_x100", "detail_score", "jpg_bytes"],
    rows)
_save_desc(csv_path,
    "E39 - stretch-off vs stretch-on on the 80x60 gray buffer across\n"
    "the 4 ISP presets.  stretch-on applies min-max linear stretch in\n"
    "the publisher so the decimated frame spans full [0..255].\n"
    "Success = (vmax-vmin) widens consistently in 'on' rows.\n",
    params={"presets": len(PRESETS),
            "aec_settle_ms": AEC_SETTLE_MS,
            "restart_settle_ms": RESTART_SETTLE_MS})
_record("e39_gray_stretch", csv_path,
        "presets=%d rows=%d" % (len(PRESETS), len(rows)))

if owned: end()
