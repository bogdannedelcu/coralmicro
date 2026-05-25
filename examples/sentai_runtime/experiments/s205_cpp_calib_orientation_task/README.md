# s205 - B5 C++ Calibration Migration Harness

`s205` is the B5 workspace for migrating the accepted B3 orientation
calibration from MP-heavy `s197` into C++ runtime services.

Baseline references:

- accepted B3 reference: `../s197_sota_calib_orientation_guarded/iter35_strict_scalar_calib_ini_source/`
- accepted B4 reference: `../s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/`

The rule for this experiment is strict: C++ code must preserve B3 semantics
1-to-1 while progressively taking ownership of per-frame observation,
statistics, scoring, persistence, and eventually the full orientation FSM.
Every moved block must also emit `sentai.fr` evidence so post-mortem debugging
stays at least as good as the original MP journals.

Current migrated pieces:

- `sentai.calib.sample_calib_observation_tuple(...)`: captures the camera frame,
  runs `sentai.markers`, computes the B3 aggregate observation, and logs
  marker-count/centroid/Z scalars to `sentai.fr`.
- `sentai.calib.score_axis_candidate(...)`: scores the discrete
  camera/body orientation candidate in C++.
- `sentai.calib.save_contract(...)`: writes strict scalar
  `/system/calib.ini` from C++.

Run:

```bash
bash examples/sentai_runtime/experiments/s205_cpp_calib_orientation_task/run.sh iter1
```
