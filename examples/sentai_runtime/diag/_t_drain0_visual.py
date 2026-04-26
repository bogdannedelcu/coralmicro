# Visual A/B verify: alt 1:1 + drain=0, capture frames with software cam_id label.
# GENTLE: 4 frames @ q=40, 200 ms between captures (~6 frames/sec, well below LFS throughput).
# No TPU load, no rapid succession.  Captures save asynchronously via lfs_task; we wait.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.camera.init()
sentai.rtos.sleep_ms(500)

# Ensure dirs exist (idempotent)
try: sentai.fs.mkdir("/diag")
except: pass
try: sentai.fs.mkdir("/diag/d0")
except: pass

# Activate IP first (so HTTP stays up if anything goes wrong later)
sentai.usb.ip(1)
sentai.rtos.sleep_ms(800)
print("IP active")

# Alt 1:1, drain=0
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(0)
sentai.camera.select(0)
sentai.rtos.sleep_ms(500)

print("--- capture 4 frames alt 1:1 drain=0 ---")
for i in range(4):
    cid = sentai.camera.current_id()
    path = "/diag/d0/f%d_cam%d.jpg" % (i, cid)
    sz = sentai.camera.save_jpeg(path, 40)
    print("  f=%d sw_cam=%d size=%d path=%s" % (i, cid, sz, path))
    # Long delay so MUX can flip + LFS finishes async write
    sentai.rtos.sleep_ms(200)

# Reset to single-cam stable state
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(200)
print("=== done ===")
