# _t_flow_speed.py -- Flow throughput probe (steady state, no operator).
#
# Boots flow, lets it warm up 1 s, samples pub_stats() over a 3 s window
# from C++-side counters (no REPL hot loop in the timing), then dumps
# perf() cycle counts per stage so we can see PXP / SAD breakdown.
#
# Output keeps the "=== done ===" sentinel so host runners stream cleanly.

import sentai
sentai.verbose(1)

print("=== flow speed probe ===")

rc = sentai.flow.enable()
print("flow.enable:", rc)
if rc < 0:
    print("=== done ==="); raise SystemExit

rc = sentai.camera.init(1)
print("camera.init:", rc)
if rc != 0 and rc != -11:
    print("=== done ==="); raise SystemExit

rc = sentai.flow.start(0)
print("flow.start:", rc)

# warm-up so the first PXP / dirty-skip transients drop out
sentai.rtos.sleep_ms(1500)

WINDOW_MS = 3000
s0 = sentai.flow.pub_stats()
t0 = sentai.rtos.ticks_ms()
sentai.rtos.sleep_ms(WINDOW_MS)
s1 = sentai.flow.pub_stats()
t1 = sentai.rtos.ticks_ms()

elapsed_ms = t1 - t0
frames = s1["frames"] - s0["frames"]
fps = (frames * 1000.0) / max(1, elapsed_ms)
print("FLOW frames=%d window_ms=%d -> %.2f fps" % (frames, elapsed_ms, fps))
print("  grab_fail_total=%d  grab_fail_streak=%d  running=%s"
      % (s1["grab_fail_total"], s1["grab_fail_streak"], s1["running"]))

p = sentai.flow.perf()
us = lambda c: c / 800.0  # 800 cyc = 1 us @ 800 MHz M7
print("FLOW perf (last frame, us):  pxp=%.2f rgb2y=%.2f stretch=%.2f sad=%.2f total=%.2f grab=%.2f loop=%.2f"
      % (us(p["pxp_cyc"]), us(p["rgb2y_cyc"]), us(p["stretch_cyc"]),
         us(p["sad_cyc"]), us(p["total_cyc"]),
         us(p["grab_cyc"]), us(p["loop_cyc"])))

sentai.flow.stop()
print("=== done ===")
