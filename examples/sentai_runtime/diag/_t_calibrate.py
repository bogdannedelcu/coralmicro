# _t_calibrate.py — exercise sentai.pipeline.calibrate / force_parity / loop_delay.
# First-run with sentai.verbose(1) per agent.md rule §2.7.
import sentai
sentai.verbose(1)

print("=== boot ===")
print("version=", sentai.version())

# Bindings exist?
print("force_parity   :", sentai.pipeline.force_parity())
print("loop_delay     :", sentai.pipeline.loop_delay())
print("calibrate type :", type(sentai.pipeline.calibrate))

# Prepare camera (alt 1:1, drain=1, start from cam0)
print("--- camera setup ---")
sentai.camera.init(1)
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(800)

# Read the per-buffer tag globals — diagnoses propagation
print("current_id      :", sentai.camera.current_id())
print("last_capture_id :", sentai.camera.last_capture_id())
print("grabbed_id (no grab yet):", sentai.camera.grabbed_id())

# Force a sequence of plain grabs via to_tensor (uses the same recovery
# grab path that PrepTask uses) and read the tag after each one.
print("--- to_tensor + grabbed_id loop ---")
for i in range(8):
    sentai.camera.to_tensor()
    print("  i=%d grabbed=%d current=%d last=%d" %
          (i, sentai.camera.grabbed_id(),
              sentai.camera.current_id(),
              sentai.camera.last_capture_id()))
    sentai.rtos.sleep_ms(60)

# Now run calibrate (auto-load model + auto-start pipeline)
print("--- calibrate parity OFF ---")
sentai.pipeline.force_parity(0)
sentai.pipeline.force_parity_reset()
r = sentai.pipeline.calibrate(
    "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
    50)
print("  cam0=%d cam1=%d unk=%d frames=%d wall_ms=%d fps=%d.%02d" %
      (r['cam0'], r['cam1'], r['unknown'], r['frames'],
       r['wall_ms'], r['fps_x100'] // 100, r['fps_x100'] % 100))
print("  invoke_ms min/avg/max = %d / %d / %d" %
      (r['invoke_ms_min'],
       (r['invoke_ms_sum'] // max(1, r['frames'])),
       r['invoke_ms_max']))

# Re-run calibrate WITH parity ON (auto-stop happened → auto-start again)
print("--- calibrate parity ON ---")
sentai.pipeline.force_parity(1)
sentai.pipeline.force_parity_reset()
r = sentai.pipeline.calibrate(None, 50)
ps = sentai.pipeline.force_parity_stats()
print("  cam0=%d cam1=%d unk=%d skipped=%d timeout=%d" %
      (r['cam0'], r['cam1'], r['unknown'],
       ps['skipped'], ps['timeout']))

# loop_delay sweep
print("--- loop_delay sweep (parity OFF) ---")
sentai.pipeline.force_parity(0)
for d in (0, 5, 10, 22, 33):
    r = sentai.pipeline.calibrate(None, 30, 2000, d)
    print("  delay_ms=%d  cam0=%d cam1=%d unk=%d fps=%d.%02d" %
          (d, r['cam0'], r['cam1'], r['unknown'],
           r['fps_x100'] // 100, r['fps_x100'] % 100))

print("=== done ===")
