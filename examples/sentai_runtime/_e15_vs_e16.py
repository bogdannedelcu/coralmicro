import diag, gc
sentai.verbose(0)
diag.begin("e15_vs_e16_x40")
gc.collect()
r15 = diag.e15_pipeline_parallel_512(repetitions=40, save=True)
gc.collect()
r16 = diag.e16_camera_switch_512(repetitions=40, save=True)
diag.end()
print("=== SUMMARY ===")
print("E15: fps=%.2f mean_frame=%.1fms (cam0 fixed)" % (
    r15["summary"]["fps_wall"], r15["summary"]["frame_interval"]["mean"]))
print("E16: fps=%.2f mean_frame=%.1fms (cam0<->cam1)" % (
    r16["summary"]["fps"], r16["summary"]["total"]["mean"]))
print("E16 cam0 mean=%.1fms  cam1 mean=%.1fms" % (
    r16["summary"]["total_cam_a"]["mean"], r16["summary"]["total_cam_b"]["mean"]))
