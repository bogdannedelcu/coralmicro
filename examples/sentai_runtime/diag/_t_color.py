# Quick color test post swapByte=true.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.camera.init()
sentai.rtos.sleep_ms(500)

try: sentai.fs.mkdir("/diag")
except: pass
try: sentai.fs.mkdir("/diag/cs")
except: pass

sentai.usb.ip(1)
sentai.rtos.sleep_ms(500)

sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)

# 4 captures alternating cam, with the new swapByte path
for i, target in enumerate([0, 1, 0, 1]):
    sentai.camera.select(target)
    sentai.rtos.sleep_ms(500)
    grabbed_pre = sentai.camera.grabbed_id()
    sz = sentai.camera.save_jpeg("/diag/cs/_t.jpg", 60)
    grabbed = sentai.camera.grabbed_id()
    final = "/diag/cs/swap_iter%d_cam%d.jpg" % (i, grabbed)
    print("  iter%d target=%d grabbed=%d sz=%d" % (i, target, grabbed, sz))
    data = sentai.fs.read_str("/diag/cs/_t.jpg")
    sentai.fs.write(final, data)
    sentai.rtos.sleep_ms(500)

print("=== done ===")
