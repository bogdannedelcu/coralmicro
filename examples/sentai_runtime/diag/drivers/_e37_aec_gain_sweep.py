# _e37_aec_gain_sweep.py — AEC target × gain ceiling sweep.
#
# E36 showed that raising the AEC target alone didn't brighten the
# output because the gain was capped at NXP's 0x7C (15.5×).  This
# experiment sweeps both axes: 3 AEC targets × 3 gain ceilings = 9
# images.  Longer settle time (1.5s) so AEC actually converges.

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
if owned: begin("e37_aec_gain")

shots_dir = _photos_dir("e37") or "/diags/e37"
try: sentai.fs.mkdir(shots_dir)
except Exception: pass

AEC_PAIRS = [(0x30, 0x28), (0x78, 0x68), (0xA8, 0x98)]
GAIN_CEIL = [0x07C, 0x1F0, 0x3FF]   # 15.5×, 31×, 62.9×

try: sentai.camera.jpeg(60)           # warmup
except Exception: pass

for gc_val in GAIN_CEIL:
    rc_g = sentai.camera.gain_ceiling_set(0, gc_val)
    print("[E37] gain_ceiling=0x%03X rc=%d" % (gc_val, rc_g))
    for (hi, lo) in AEC_PAIRS:
        rc_a = sentai.camera.aec_set(0, hi, lo)
        # Generous settle: OV5640 AEC loop typically needs 15-30 frames
        # to reach a new target (~350-700ms at 45fps).
        sentai.rtos.sleep_ms(1500)
        # Burn 3 discard frames before the metered one.
        for _ in range(3):
            try: sentai.camera.jpeg(50)
            except Exception: pass
        try:
            jpg = sentai.camera.jpeg(80)
            path = "%s/g%03X_aec_%02X_%02X.jpg" % (shots_dir, gc_val, hi, lo)
            sentai.fs.write(path, jpg)
            print("  saved %s (%dB)" % (path, len(jpg)))
            jpg = None; gc.collect()
        except Exception as e:
            print("  FAIL: %s" % e)

# Restore NXP defaults at the end.
sentai.camera.aec_set(0, 0x30, 0x28)
sentai.camera.gain_ceiling_set(0, 0x07C)
print("[E37] done.  frames at %s" % shots_dir)
if owned: end()
