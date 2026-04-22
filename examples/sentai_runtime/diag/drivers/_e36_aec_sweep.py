# _e36_aec_sweep.py — capture 1 JPEG per AEC target setting so we can
# eyeball which value gives acceptable exposure for our scene.
#
# Each pair (high, low) writes OV5640 registers 0x3A0F / 0x3A10 etc.
# via sentai.camera.aec_set(cam_id, high, low).  After setting, we
# wait ~500 ms so the AEC loop converges to the new target, then
# capture a JPEG tagged with the target value.
#
# References from the OmniVision OV5640 application notes:
#   NXP default        : high=0x30 low=0x28  (≈18% luma target)
#   OV AN "daylight"   : high=0x78 low=0x68  (≈45% — common default)
#   Bright / outdoors  : high=0x60 low=0x50  (≈35%)
#   Very dark scene    : high=0x48 low=0x38  (≈25%)
#
# Output:
#   /diags/sNNN_e36/000_e36_aec_frames/aec_HH_LL_cam0.jpg

import sentai, gc

gc.collect()

if sentai.pipeline.running():
    sentai.pipeline.stop()

if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)

from diag._session import _session, begin, end, _photos_dir
from diag._util import _ticks

owned = (_session is None)
if owned:
    begin("e36_aec_sweep")

shots_dir = _photos_dir("e36_aec") or "/diags/e36_aec"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

# Sweep of AEC (high, low) pairs, ordered from darkest to brightest.
PAIRS = [
    (0x30, 0x28),   # NXP stock default
    (0x48, 0x38),
    (0x60, 0x50),
    (0x78, 0x68),   # OV AN "daylight" default
    (0x90, 0x80),
    (0xA8, 0x98),
]

# Warmup with stock settings (so sensor is streaming before we tune).
try: sentai.camera.jpeg(60)
except Exception: pass

for (hi, lo) in PAIRS:
    rc = sentai.camera.aec_set(0, hi, lo)
    print("[E36] aec_set(0, 0x%02X, 0x%02X) rc=%d" % (hi, lo, rc))
    # AEC convergence takes a few frames — give it ~500 ms.
    sentai.rtos.sleep_ms(500)
    # Discard 2 settle frames then capture the measured one.
    for _ in range(2):
        try: sentai.camera.jpeg(50)
        except Exception: pass
    t0 = _ticks()
    try:
        jpg = sentai.camera.jpeg(80)
        path = "%s/aec_%02X_%02X_cam0.jpg" % (shots_dir, hi, lo)
        sentai.fs.write(path, jpg)
        print("  saved %s (%dB, %dms)" %
              (path, len(jpg), _ticks() - t0))
        jpg = None; gc.collect()
    except Exception as e:
        print("  capture FAIL: %s" % e)

# Restore NXP stock as a neutral end state (any subsequent script
# sees the original AEC target, not whatever we last set).
sentai.camera.aec_set(0, 0x30, 0x28)
print("[E36] done. frames at %s" % shots_dir)

if owned: end()
