# _e39_cam_switch_visual.py — visually verify that camera MUX switching
# with the new drain=1 default does NOT mix pixels from one sensor into
# the other's post-switch frame.
#
# Captures 3 scenario passes:
#   A) Baseline — 3 JPEGs per cam (static, 200 ms settle)
#   B) Rapid switch — switch+grab with no settle delay (worst case)
#   C) First-post-switch — the exact first JPEG immediately after flip
#
# Images are saved under /diags/e39_cam_switch_visual/ and the host
# downloader can pull them into experiments/s082_e39_cam_switch_visual/.
import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

out_dir = "/diags/e39_cam_switch_visual"
try: sentai.fs.mkdir("/diags")
except Exception: pass
try: sentai.fs.mkdir(out_dir)
except Exception: pass

QUALITY = 70

def grab(tag):
    t0 = sentai.rtos.ticks_ms()
    jpg = sentai.camera.jpeg(QUALITY)
    dt = sentai.rtos.ticks_ms() - t0
    path = "%s/%s_%dms_%db.jpg" % (out_dir, tag, dt, len(jpg))
    sentai.fs.write(path, jpg)
    print("  %s" % path)
    del jpg
    gc.collect()

# Warm both cameras — first frame after switch sometimes stale.
sentai.camera.select(0)
for _ in range(3): sentai.camera.jpeg(30)
sentai.camera.select(1)
for _ in range(3): sentai.camera.jpeg(30)

print("drain =", sentai.camera.switch_drain())
print("=== A: baseline (200 ms settle) ===")
for i in range(3):
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(200)
    grab("A%d_cam0_baseline" % i)
    sentai.camera.select(1)
    sentai.rtos.sleep_ms(200)
    grab("A%d_cam1_baseline" % i)

print("=== B: rapid switch (no settle) ===")
for i in range(3):
    sentai.camera.select(0)
    grab("B%d_cam0_rapid" % i)      # grab immediately after flip
    sentai.camera.select(1)
    grab("B%d_cam1_rapid" % i)

print("=== C: first-post-switch frame only ===")
# Before flip we're at cam1; flip to cam0 → FIRST grab is the critical
# frame (verifying MUX flip landed in VBLANK, no mixing).
for i in range(5):
    sentai.camera.select(0)
    grab("C%d_cam0_first" % i)
    sentai.camera.select(1)
    grab("C%d_cam1_first" % i)

print("=== done ===")
