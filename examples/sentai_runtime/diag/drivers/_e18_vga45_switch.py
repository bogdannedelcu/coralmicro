# _e18_vga45_switch.py — camera-switching benchmark at VGA @ 45 fps.
#
# Runs E18 (head-to-tail A/B/C) twice:
#   drain=1 : minimum drain, relies on the glitch-free MUX flip (Fix B)
#             to guarantee frame N+1 is 100% from the new sensor.
#   drain=2 : conservative, adds one extra frame of wait.
#
# At VGA/45 the cam frame interval is ~22 ms (vs 33 ms @ 30), so each
# drained frame costs proportionally less.  Goal: measure the REAL
# alternating-mode fps at the new camera clock.

import sentai, diag, gc

sentai.verbose(0)

# Make sure pipeline is stopped + model loaded.
if sentai.pipeline.running():
    sentai.pipeline.stop()
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)

results = {}
for drain in (1, 2):
    sentai.camera.switch_drain(drain)
    diag.begin("e18_vga45_d%d" % drain)
    gc.collect()
    r = diag.e18_camera_switch_headtail(repetitions=40, save=True)
    results[drain] = r
    diag.end()
    gc.collect()

sentai.camera.switch_drain(2)  # restore default
sentai.verbose(1)

print("\n=== E18 VGA@45 CAMERA SWITCHING SUMMARY ===")
print("%-12s %-10s %-10s %-10s %-12s" % ("drain", "A_cam0", "B_cam1", "C_alt", "switch_ms"))
for drain in (1, 2):
    s = results[drain]["summary"]
    a = s["A_fixed_cam_a"]["fps"]
    b = s["B_fixed_cam_b"]["fps"]
    c = s["C_alternating"]["fps"]
    ov = s["per_switch_overhead_ms"]
    print("%-12s %-10.2f %-10.2f %-10.2f %-12.2f" % ("drain=%d" % drain, a, b, c, ov))
