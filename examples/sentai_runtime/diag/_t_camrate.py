# Camera rate probe -- rule out g_camera_frame_seq anomalies
import sentai
sentai.verbose(1)

sentai.camera.init()
sentai.rtos.sleep_ms(1000)  # let CSI stabilize

# 5 samples of 1s each
print("5 samples of 1s camera-only:")
for i in range(5):
    c0 = sentai.camera.frame_count()
    sentai.rtos.sleep_ms(1000)
    c1 = sentai.camera.frame_count()
    print("  sample %d: %d frames -> %d FPS" % (i, c1-c0, c1-c0))

# Now with an active consumer (camera.to_tensor in loop)
print("5 samples with to_tensor() drain:")
for i in range(5):
    c0 = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    while sentai.rtos.ticks_ms() - t0 < 1000:
        sentai.camera.to_tensor()
    c1 = sentai.camera.frame_count()
    print("  sample %d: %d frames -> %d FPS" % (i, c1-c0, c1-c0))

print("=== done ===")
