import diag, gc
sentai.verbose(0)
sentai.camera.ratio(0, 0)         # manual alternation only
sentai.camera.switch_drain(2)     # conservative default
diag.begin("e16_eof_30fps_x40")
gc.collect()
r = diag.e16_camera_switch_512(repetitions=40, save=True)
diag.end()
