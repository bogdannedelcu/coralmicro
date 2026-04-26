# _t_camid_diag.py — verify the per-buffer cam_id tag write path.
# Build #881 added g_cam_buf_tag_writes / g_cam_buf_tag_idx_miss
# counters in the CSI ISR + sentai.camera.buf_id_dump() to surface them.
import sentai
sentai.verbose(1)

print("=== boot ===")
print("version=", sentai.version())

# Camera up; static MUX (no alt) so any tag changes are unambiguous.
sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(800)

# Snapshot 1: nothing grabbed yet -- but ISR has been running for a while
# so writes counter SHOULD be > 0 and tags SHOULD be 0, not 0xFF.
d = sentai.camera.buf_id_dump()
print("snap1 (post-init, pre-grab):")
print("  buf_id slots = (%d, %d, %d, %d)" % (d[0], d[1], d[2], d[3]))
print("  isr_writes  = %d" % d[4])
print("  idx_misses  = %d" % d[5])

# Load model so to_tensor doesn't bail with -3.
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")

# Single-camera grab loop: should fill buf slots with 0s.
print("--- grab loop, cam0 only ---")
for i in range(8):
    sentai.camera.to_tensor()
    d = sentai.camera.buf_id_dump()
    print("  i=%d slots=(%d,%d,%d,%d) writes=%d miss=%d grabbed=%d" %
          (i, d[0], d[1], d[2], d[3], d[4], d[5],
           sentai.camera.grabbed_id()))
    sentai.rtos.sleep_ms(50)

# Switch to cam1; expect tags to flip to 1s on slots that get filled.
sentai.camera.select(1)
sentai.rtos.sleep_ms(300)
print("--- grab loop, cam1 only ---")
for i in range(8):
    sentai.camera.to_tensor()
    d = sentai.camera.buf_id_dump()
    print("  i=%d slots=(%d,%d,%d,%d) writes=%d miss=%d grabbed=%d" %
          (i, d[0], d[1], d[2], d[3], d[4], d[5],
           sentai.camera.grabbed_id()))
    sentai.rtos.sleep_ms(50)

# Alt 1:1 — both 0 and 1 should appear in slots over time.
sentai.camera.select(0)
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)
print("--- alt 1:1 grab loop ---")
for i in range(10):
    sentai.camera.to_tensor()
    d = sentai.camera.buf_id_dump()
    print("  i=%d slots=(%d,%d,%d,%d) grabbed=%d" %
          (i, d[0], d[1], d[2], d[3], sentai.camera.grabbed_id()))
    sentai.rtos.sleep_ms(50)

print("=== done ===")
