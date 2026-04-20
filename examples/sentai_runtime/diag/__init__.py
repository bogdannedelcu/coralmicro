# diag/__init__.py — SentAI diagnostics package
#
# Usage on device:
#   import diag
#   diag.begin("my_test")
#   diag.e1_tpu_invoke("/yolo26n.edgetpu_1.tflite", use_camera=True, repetitions=50)
#   diag.e12_live_loop("/yolo26n.edgetpu_1.tflite")
#   diag.end()
#
# Add new experiments:
#   - Add a function to the appropriate e_*.py (or create a new e_foo.py)
#   - Import it here so it's reachable as diag.e_foo()
#
# Upload to device: python3 upload_diag.py  (from sentai_runtime/)

# ── utilities ────────────────────────────────────────────────────────────────
from diag._util import (
    stats, percentile, time_call,
    ensure_dir, save_csv,
    snapshot_meta, snapshot_heap, snapshot_cpu, snapshot_tasks,
)

# ── session management ────────────────────────────────────────────────────────
from diag._session import Session, begin, end, status, snapshot_scene, snapshot_both_cameras

# ── experiments: TPU ─────────────────────────────────────────────────────────
from diag.e_tpu import e1_tpu_invoke, e2_tpu_load

# ── experiments: camera ───────────────────────────────────────────────────────
from diag.e_camera import (
    e3_camera_tensor,
    e4_jpeg,
    e5_camera_switch,
    e5b_alternating,
    e5c_asymmetric,
    e5c_sweep,
)

# ── experiments: filesystem ───────────────────────────────────────────────────
from diag.e_fs import e6_fs_read, e7_fs_write

# ── experiments: sensors ─────────────────────────────────────────────────────
from diag.e_sensors import e8_imu, e9_mic

# ── experiments: system ───────────────────────────────────────────────────────
from diag.e_system import e10_memory, e11_cpu, e12_live_loop

# ── experiments: full pipeline (per-stage timing) ─────────────────────────────
from diag.e_pipeline import (
    e13_pipeline_full,
    e14_pipeline_parallel,
    e15_pipeline_parallel_512,
    e16_camera_switch_512,
    e17_switch_drain_visual,
    e18_camera_switch_headtail,
)

# ── batch runners ─────────────────────────────────────────────────────────────
from diag.batch import (
    run_all_quick,
    run_ablation_cameras,
    run_ablation_resolutions,
)

print("diag loaded. diag.begin('name') then diag.e1_tpu_invoke(...)")
