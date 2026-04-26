# Visual A/B verify with PER-BUFFER TAGGING.
# Filename = cam{grabbed}_iter{i}.jpg — grabbed is the ACTUAL buffer source.
# Diagnostics (target/cur/last) printed in log only.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.camera.init()
sentai.rtos.sleep_ms(500)

try: sentai.fs.mkdir("/diag")
except: pass
try: sentai.fs.mkdir("/diag/tg2")
except: pass

sentai.usb.ip(1)
sentai.rtos.sleep_ms(500)

sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)

print("--- single per-cam captures (target alternating) ---")
for i, target in enumerate([0, 1, 0, 1]):
    sentai.camera.select(target)
    sentai.rtos.sleep_ms(500)
    cur  = sentai.camera.current_id()
    last = sentai.camera.last_capture_id()
    tmp = "/diag/tg2/_tmp.jpg"
    sz = sentai.camera.save_jpeg(tmp, 50)
    grabbed = sentai.camera.grabbed_id()
    final = "/diag/tg2/single_iter%d_cam%d.jpg" % (i, grabbed)
    print("  iter%d target=%d cur=%d last=%d -> grabbed=%d sz=%d" %
          (i, target, cur, last, grabbed, sz))
    data = sentai.fs.read_str(tmp)
    sentai.fs.write(final, data)
    sentai.rtos.sleep_ms(400)

print("--- alt 1:1 drain=0 ---")
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(0)
sentai.rtos.sleep_ms(500)

for i in range(6):
    tmp = "/diag/tg2/_tmp.jpg"
    sz = sentai.camera.save_jpeg(tmp, 50)
    grabbed = sentai.camera.grabbed_id()
    cur = sentai.camera.current_id()
    last = sentai.camera.last_capture_id()
    final = "/diag/tg2/alt_iter%d_cam%d.jpg" % (i, grabbed)
    print("  iter%d cur=%d last=%d -> grabbed=%d sz=%d" %
          (i, cur, last, grabbed, sz))
    data = sentai.fs.read_str(tmp)
    sentai.fs.write(final, data)
    sentai.rtos.sleep_ms(150)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
print("=== done ===")
