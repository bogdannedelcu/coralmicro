# Post-refactor validation run.  Session name records the firmware
# build so paper/cam_switch.md can cross-reference.
import diag, gc
sentai.verbose(0)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(2)
diag.begin("e18_post_refactor")
gc.collect()
r = diag.e18_camera_switch_headtail(repetitions=40, save=True)
# Read fault counters at end — none should be non-zero on nominal run.
print("cam_stats_end:", sentai.diag.cam_stats())
diag.end()
