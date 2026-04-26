# Ground truth visual A/B:
#   1. select(0) + sleep + capture → cam0 single (no switching)
#   2. select(1) + sleep + capture → cam1 single (no switching)
#   3. enable alt 1:1 + drain=0, capture 4 frames using last_capture_id() label
# Goal: confirm whether last_capture_id correctly tags buffer source.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.camera.init()
sentai.rtos.sleep_ms(500)

try: sentai.fs.mkdir("/diag")
except: pass
try: sentai.fs.mkdir("/diag/gt")
except: pass

# Activate IP
sentai.usb.ip(1)
sentai.rtos.sleep_ms(500)

# Ensure no auto-alt, single-cam stable
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)

# === Ground truth: single capture per camera ===
print("--- ground truth single captures ---")
for target in [0, 1]:
    sentai.camera.select(target)
    sentai.rtos.sleep_ms(300)  # let MUX settle, several sensor frames
    cur = sentai.camera.current_id()
    last = sentai.camera.last_capture_id()
    path = "/diag/gt/single_cam%d_cur%d_last%d.jpg" % (target, cur, last)
    sz = sentai.camera.save_jpeg(path, 50)
    print("  target=%d cur=%d last=%d size=%d -> %s" % (target, cur, last, sz, path))
    sentai.rtos.sleep_ms(500)  # rest before next

# === Switched captures: alt 1:1 drain=0 with proper label ===
print("--- alt 1:1 drain=0 with last_capture_id label ---")
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(0)
sentai.rtos.sleep_ms(500)

for i in range(4):
    last = sentai.camera.last_capture_id()  # use SOURCE id, not MUX state
    path = "/diag/gt/alt_%d_src%d.jpg" % (i, last)
    sz = sentai.camera.save_jpeg(path, 50)
    cur_after = sentai.camera.current_id()
    print("  i=%d src(last)=%d cur_after=%d size=%d -> %s" % (i, last, cur_after, sz, path))
    sentai.rtos.sleep_ms(150)

# Reset
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
print("=== done ===")
