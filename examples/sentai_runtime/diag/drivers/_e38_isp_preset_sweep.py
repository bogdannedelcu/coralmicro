# _e38_isp_preset_sweep.py — sweep the OV5640 ISP presets, capture one
# JPEG + one 40×30 gray snap per preset, score each by gradient energy
# (sentai.flow.detail_score), and pick the preset that maximizes detail.
#
# Each preset writes a bundled set of ISP registers (gamma LUT, SDE
# brightness/contrast, AEC target, gain ceiling).  After apply we wait
# for the AEC loop to converge (~1.5 s), burn a few frames, then
# capture the "settled" frame.
#
# Output:
#   /diags/sNNN_e38/000_e38_preset_frames/{preset}_cam0.jpg
#   /diags/sNNN_e38/000_e38_preset_frames/{preset}_gray40x30.pgm
#   CSV: 001_e38_preset_scores.csv with one row per preset

import sentai, gc

gc.collect()

PRESETS = [
    "nxp_stock",      # baseline — NXP defaults
    "bright_indoor",  # sRGB gamma, +20% contrast, AEC 0x60, 31× gain
    "daylight",       # sRGB gamma, AEC 0x78, 31× gain
    "low_light",      # strong gamma, +30% contrast, 62.9× gain
]

if sentai.pipeline.running():
    sentai.pipeline.stop()
# TPU must be ready before pipeline.start() — it prereqs a loaded model.
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.select(0)

# Flow needs to run so the 40×30 gray buffer is fresh for detail_score.
rc = sentai.flow.m4_enable()
print("[E38] m4_enable rc=%d" % rc)

# Start pipeline + flow once; presets only touch OV5640 ISP regs,
# no pipeline restart needed between presets.
sentai.pipeline.direct_tensor(1)
sentai.pipeline.prep_reset()
sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get_ex(2000)
sentai.flow.m4_start(0)
sentai.rtos.sleep_ms(500)

from diag._session import _session, begin, end, _save_path, _record, _save_desc, _photos_dir
from diag._util import save_csv, _ticks

owned = (_session is None)
if owned: begin("e38_isp_preset")

shots_dir = _photos_dir("e38_preset") or "/diags/e38_preset"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

results = []   # (name, score, gray_mean, jpg_bytes)

for preset in PRESETS:
    rc = sentai.camera.isp_preset(0, preset)
    print("[E38] preset=%s rc=%d" % (preset, rc))
    # AEC/AGC loop needs 15–30 frames to converge to a new setpoint.
    # At VGA/45 fps that's ~350-700 ms; give 1.5 s for margin.
    sentai.rtos.sleep_ms(1500)

    # Read detail score straight from the live 40×30 buffer — flow
    # is still feeding it.
    score = sentai.flow.detail_score()

    # Grab a JPEG for visual inspection (briefly pause pipeline so
    # camera.jpeg has exclusive access).
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(100)
    jpg_bytes = 0
    try:
        jpg = sentai.camera.jpeg(80)
        jpg_bytes = len(jpg)
        sentai.fs.write("%s/%s_cam0.jpg" % (shots_dir, preset), jpg)
        jpg = None; gc.collect()
    except Exception as e:
        print("  jpeg fail: %s" % e)

    # Save the 80×60 gray too, post-stop (last published frame).
    # Header dimensions must match FLOW_GRAY_W/H which is now 80×60.
    gray = sentai.flow.m4_gray_snap()
    sentai.fs.write("%s/%s_gray80x60.pgm" % (shots_dir, preset),
                    b"P5\n80 60\n255\n" + gray)
    # Mean luma on the 40×30 (since we can't easily histogram 640×480).
    s = 0
    for b in gray: s += b
    gray_mean = s // len(gray)
    gray = None

    sentai.pipeline.prep_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    _ = sentai.pipeline.get_ex(2000)
    sentai.flow.m4_start(0)
    sentai.rtos.sleep_ms(300)

    print("  score=%d  gray_mean=%d  jpg=%dB" %
          (score, gray_mean, jpg_bytes))
    results.append((preset, score, gray_mean, jpg_bytes))

# Pick the winner.
best = max(results, key=lambda r: r[1])
print()
print("=== E38 SUMMARY ===")
for name, score, mean, jb in results:
    flag = "  ← BEST" if name == best[0] else ""
    print("  %-15s  score=%5d  mean=%3d  jpg=%dB%s" %
          (name, score, mean, jb, flag))
print("best preset: %s  (detail score %d)" % (best[0], best[1]))

# Restore baseline at the end.
sentai.camera.isp_preset(0, "nxp_stock")
sentai.flow.m4_stop()
sentai.pipeline.stop()

csv_path = _save_path("e38_preset_scores")
save_csv(csv_path,
    ["preset", "detail_score", "gray_mean", "jpg_bytes"],
    results)
_save_desc(csv_path,
    "E38 - OV5640 ISP preset sweep with auto-pick by gradient energy.\n"
    "detail_score = Σ|gradient| / pixels × 100 on the 40x30 gray buffer.\n"
    "best preset = argmax(detail_score).\n",
    params={"presets": len(PRESETS), "best": best[0],
            "best_score": best[1]})
_record("e38_isp_preset", csv_path,
        "best=%s score=%d" % (best[0], best[1]))

if owned: end()
