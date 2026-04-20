import diag, gc
sentai.verbose(0)
diag.begin("e17_drain_ab")
gc.collect()
sentai.camera.switch_drain(2)
r2 = diag.e17_switch_drain_visual(repetitions=16, quality=70, save=True)
gc.collect()
sentai.camera.switch_drain(1)
r1 = diag.e17_switch_drain_visual(repetitions=16, quality=70, save=True)
gc.collect()
sentai.camera.switch_drain(2)  # restore default before leaving
diag.end()
print("=== E17 SUMMARY ===")
print("t=2: hot_wall=%dms mean=%.1fms" % (
    r2["summary"]["hot_wall_ms"], r2["summary"]["per_frame"]["mean"]))
print("t=1: hot_wall=%dms mean=%.1fms" % (
    r1["summary"]["hot_wall_ms"], r1["summary"]["per_frame"]["mean"]))
