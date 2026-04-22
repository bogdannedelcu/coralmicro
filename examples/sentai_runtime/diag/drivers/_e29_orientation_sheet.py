# _e29_orientation_sheet.py — capture both cameras with a user-held
# orientation reference sheet (FW / L / R / BACK).  LED cues when
# to hold the sheet steady for the capture burst.
#
# Protocol:
#   1. LED OFF, 5 s   — user positions the sheet in front of the drone.
#   2. LED ON, captures burst: cam0 then cam1, 3 frames each, VGA.
#   3. LED OFF, done.

import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

sentai.camera.ratio(0, 0)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

from diag._session import _session, begin, end, _photos_dir
from diag._util import _ticks

owned = (_session is None)
if owned:
    begin("e29_orientation")

frames_dir = _photos_dir("e29_orientation") or "/diags/e29_orientation"
try: sentai.fs.mkdir(frames_dir)
except Exception: pass

QUALITY = 80
PAIRS = 3

# Warmup both cams so the first measured frame isn't a cold miss.
def _warm(cam):
    sentai.camera.select(cam)
    for _ in range(5):
        try: sentai.camera.jpeg(60); return True
        except RuntimeError: pass
    return False
_warm(0); _warm(1)

# Phase 1 — wait for the user to position the sheet.
sentai.io.led_off()
print("[E29] Pune foaia cu FW/L/R/BACK in fata dronei.  5s...")
sentai.rtos.sleep_ms(5000)

# Phase 2 — LED on for the capture burst; user must hold the
# sheet still during this window (~1.5 s).
sentai.io.led_on()
print("[E29] LED ON — CAPTUREZ!  Tine foaia fixa ~2s.")

saved = 0
for i in range(PAIRS):
    for cam in (0, 1):
        sentai.camera.select(cam)
        t0 = _ticks()
        jpg = sentai.camera.jpeg(QUALITY)
        dt = _ticks() - t0
        path = "%s/%02d_cam%d_%dms_%db.jpg" % (
            frames_dir, i, cam, dt, len(jpg))
        try:
            sentai.fs.write(path, jpg)
            saved += 1
            print("  saved %s" % path)
        except Exception as e:
            print("  WARN save %s: %s" % (path, e))
        jpg = None
        gc.collect()

sentai.io.led_off()
print("[E29] done saved=%d/%d  dir=%s" % (saved, PAIRS * 2, frames_dir))

if owned:
    end()
