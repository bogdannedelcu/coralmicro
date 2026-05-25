// sentai_calib_task.h — LEGACY OP-S10-W14 worker side of the Flow autotuner.
//
// B5 note: this file belongs to the old Flow-autotune/bringup family.  Keep
// it only for old experiment compatibility while s205/s203 are migrated to
// the new sentai.calib + sentai.servo image-frame task split.  Delete after
// the B5 final migration proves the new tasks on SIM and ARM.
//
// Owns the FreeRTOS task that drives sentai_calib_autotune_tick() at
// camera FPS, reads PnP via sentai_markers_get_latest() (active
// backend = whichever sentai.markers.init selected), and writes
// velocity commands via sentai_crazy_hover().  Mirror of
// sentai.safety / sentai_safety_task split — the state machine is in
// sentai_calib_autotune.{h,cc} and pure-compute; this file is the
// only place that touches camera + cf2 telemetry.
//
// Public lifecycle declarations live in sentai_calib.h (operator
// preference: keep all `sentai.calib` API surface in one place).
// This header exists for internal documentation symmetry with the
// safety split, plus to host the period / FOV-bound constants.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Tick period — matches camera FPS (≈ 30 Hz).
#ifndef SENTAI_CALIB_TASK_PERIOD_MS
#define SENTAI_CALIB_TASK_PERIOD_MS   33
#endif

// Stop-event timeout: bounded join.
#ifndef SENTAI_CALIB_TASK_STOP_BUDGET_TICKS
#define SENTAI_CALIB_TASK_STOP_BUDGET_TICKS  30      // × 50 ms = 1.5 s
#endif

// FOV-bound safety: cap v_max so drone displacement during one
// half-period of the slowest expected oscillation stays under
// SENTAI_CALIB_TASK_FOV_FRACTION × min(grid_dx, grid_dy).  At
// vmax=0.10 m/s and Tu/2 ≈ 1.5 s that's 15 cm displacement — well
// under the typical 20 cm marker grid.
#ifndef SENTAI_CALIB_TASK_FOV_FRACTION
#define SENTAI_CALIB_TASK_FOV_FRACTION  0.5f
#endif

#ifdef __cplusplus
}
#endif
