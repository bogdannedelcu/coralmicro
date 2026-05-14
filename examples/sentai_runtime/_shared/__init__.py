"""Shared host-side utilities for experiments.

Modules here are imported by `examples/sentai_runtime/experiments/sNNN_*`
scripts running on the host (Linux). They are NEVER pushed to the
board's filesystem (the `_host_` filename rule excludes them from the
chunked-REPL uploader; this directory is excluded by being under
`_shared/` not under `diag/`).

Conventions:
- One module per topic (camera_calibration, marker_world_map, ...).
- Pure-Python + numpy; no SDK imports.
- Mirror C++ APIs where the same operation has a future on-board
  binding (e.g. camera_calibration → future sentai.calib MP module).
"""
