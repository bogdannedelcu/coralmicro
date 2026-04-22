# _e28_both_cams.py — capture a JPEG from each of the two cameras,
# back-to-back, at native VGA.  Purpose: visual sanity check of
# sensor orientation + the post-NXP-init 0x3820/0x3821 register
# values.  No rotation, no mirror — pure sensor output.

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
    begin("e28_both_cams")

frames_dir = _photos_dir("e28_both_cams") or "/diags/e28_both_cams"
try: sentai.fs.mkdir(frames_dir)
except Exception: pass

# Warmup each camera (first frame after switch tends to be stale).
def _warm(cam):
    sentai.camera.select(cam)
    for _ in range(5):
        try: sentai.camera.jpeg(60); return True
        except RuntimeError: pass
    return False
_warm(0); _warm(1)

PAIRS = 3        # 3 shots per camera = 6 JPEGs total
QUALITY = 70

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

print("[E28] saved=%d/%d  dir=%s" % (saved, PAIRS * 2, frames_dir))

if owned:
    end()
