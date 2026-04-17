# diag/batch.py — batch runners: run_all_quick, run_ablation_cameras,
#                  run_ablation_resolutions

from diag._session import begin, end
from diag.e_tpu     import e1_tpu_invoke, e2_tpu_load
from diag.e_camera  import e3_camera_tensor, e4_jpeg, e5_camera_switch
from diag.e_sensors import e8_imu
from diag.e_system  import e10_memory, e11_cpu, e12_live_loop


def run_all_quick(model_path, camera_id=0):
    """Run all experiments with minimal repetitions for a quick check."""
    begin("quick_cam%d" % camera_id)

    results = {}
    results["e1"]  = e1_tpu_invoke(model_path, use_camera=True, repetitions=10)
    results["e2"]  = e2_tpu_load(model_path, repetitions=3)
    results["e3"]  = e3_camera_tensor(camera_id, repetitions=10)
    results["e4"]  = e4_jpeg(camera_id, repetitions=10)
    results["e5"]  = e5_camera_switch(repetitions=5)
    results["e8"]  = e8_imu(repetitions=20)
    results["e10"] = e10_memory("quick_all")
    results["e11"] = e11_cpu("quick_all", duration_ms=1000)
    results["e12"] = e12_live_loop(model_path, camera_id, repetitions=10)

    end()
    return results


def run_ablation_cameras(model_path, repetitions=30):
    """Run E3, E4, E12 on both cameras for comparison."""
    begin("ablation_cameras")

    results = {}
    for cam in [0, 1]:
        results["e3_cam%d" % cam] = e3_camera_tensor(cam, repetitions=repetitions)
        results["e4_cam%d" % cam] = e4_jpeg(cam, repetitions=repetitions)
        results["e12_cam%d" % cam] = e12_live_loop(model_path, cam,
                                                     repetitions=repetitions)
    end()
    return results


def run_ablation_resolutions(model_path, camera_id=0, repetitions=20):
    """Run E3 and E12 across multiple resolutions."""
    begin("ablation_res_cam%d" % camera_id)

    resolutions = [(320, 240), (320, 320), (640, 480)]
    results = {}
    for w, h in resolutions:
        tag = "%dx%d" % (w, h)
        results["e3_%s" % tag] = e3_camera_tensor(camera_id, w, h,
                                                    repetitions=repetitions)
        results["e12_%s" % tag] = e12_live_loop(model_path, camera_id, w, h,
                                                  repetitions=repetitions)
    end()
    return results
