import diag, gc
sentai.verbose(0)
sentai.camera.ratio(0, 0)      # manual alternation only
sentai.camera.switch_drain(2)  # conservative baseline; re-test at 1 later
diag.begin("e18_headtail_drain2")
gc.collect()
r = diag.e18_camera_switch_headtail(repetitions=40, save=True)
diag.end()
