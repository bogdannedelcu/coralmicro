import diag, gc
sentai.verbose(0)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)  # best-case for the fast-wake path

# Run three back-to-back sessions so we can see run-to-run variance.
runs = []
for trial in range(3):
    diag.begin("e19_sync_trial%d" % (trial + 1))
    gc.collect()
    r = diag.e19_switch_sync_check(repetitions=16, quality=70, save=True)
    runs.append(r)
    diag.end()
    gc.collect()

sentai.camera.switch_drain(2)  # restore default
sentai.camera.switch_sync(1)

print("=== E19 SUMMARY (3 trials) ===")
for i, r in enumerate(runs):
    s = r["summary"]
    print("trial%d: sync0=%.1fms (%.2ffps)  sync1=%.1fms (%.2ffps)  x%.2f  %s" % (
        i + 1, s["sync0_mean_ms"], s["sync0_fps"],
        s["sync1_mean_ms"], s["sync1_fps"],
        s["speedup_x"], "OK" if s["meets_target"] else "MISS"))
