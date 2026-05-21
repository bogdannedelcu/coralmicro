# OP-S10-W21 — Unified calib bringup (extrinsics + PID + persistence)

**WBS:** `OP-S10-W21`
**Created:** 2026-05-21
**Status:** ⬜ Design — awaiting operator approval before implementation
**Drives:** [[sentai-calib-is-production-bringup]]

## Goal

`sentai.calib` becomes a **single on-board entry point** that auto-
discovers everything that varies unit-to-unit:

1. Camera extrinsics — `R_cam_to_body` + `cam_offset_B` (mount mechanical
   tolerance ±2-5°, drone-to-drone)
2. PID gains — `Kp_x`, `Kp_y`, `Kp_yaw` (motor balance, payload mass,
   battery age)
3. (Future / W21-T5) Camera intrinsics — `fx, fy, cx, cy` (lens / focus
   variance)

All discovered parameters PERSIST on-board (`/system/calib.ini`), so the
drone boots fully calibrated forever after.  Same C code runs in
Gazebo (digital twin for bench bringup) and on real HW (production
floor) — operator-stated 2026-05-21:

> "metoda practica de a descoperi parametrii drona/camera, parametrii
> PID, cum reactioneaza drona la un environment cand e comandata din
> sentai. Il scriem pe drona tocmai pentru ca zecile si sutele de drone
> si camere pe care le vom produce in real world sa poata rula acest
> sentai.calib si a-si face calibrarea la camera/lentila/motoare/etc."

## Why Gazebo is the digital twin (not headless synth)

Headless synth (s157 ArUco, s186 WhyCon) validates the inner numerics
(Kabsch SVD, key=value parser, etc.) but cannot catch regressions in:

- Sample collection through real PnP noise / bias
- Detection frame rate effects on autotune relay convergence
- EKF coupling with the relay velocity setpoints
- Marker-backend swap (ArUco → WhyCon) shifting the sample
  distribution shape

These are EXACTLY the failure modes that hit deployment.  Gazebo runs
the full perception → EKF → control stack against an environment we
didn't pre-bake.  Headless smokes stay as numerical regression gates
(s157/s186), Gazebo is the deployment-mirror acceptance gate.

## Current state inventory

| Capability                                  | Today                                          |
|---------------------------------------------|------------------------------------------------|
| `R_cam_to_body` Kabsch numerics             | ✅ `sentai_calib_run_kabsch`                  |
| `R` + `cam_offset_B` atomic commit          | ✅ `sentai_calib_commit_R`                    |
| `Kp_x`, `Kp_y` autotune (Åström-Hägglund)  | ✅ `sentai_calib_task_start(axis, ...)` + ZN  |
| Validation hold w/ Kp                       | ✅ `sentai_calib_hold_start(kp_x, kp_y, ...)` |
| Yaw validation hold                         | ✅ `sentai_calib_hold_yaw_start(...)`         |
| `Kp_yaw` autotune                           | ❌ Only x/y axes supported today              |
| Camera intrinsics auto-cal                  | ❌ Hardcoded everywhere                       |
| `R` + offset PERSISTED                      | ✅ `/system/cam_calib.json`                   |
| `Kp_x`, `Kp_y` PERSISTED                    | ❌ RAM-only, lost on reboot                   |
| Persistence FORMAT                          | ⚠️ JSON — operator-flagged anti-pattern (W21-T2) |
| WhyCon backend integration                  | ❌ Calib hardcoded `sentai_aruco_get_latest`  |
| Bringup orchestrator (C-side, one call)     | ❌ MP-driven via `mission_flow_autotune.py`   |
| Auto-run if `is_calibrated()==False` at boot | ❌                                            |

## Target API

### Single-call bringup entrypoint

```c
typedef struct {
    // Pad geometry — required.  WhyCon by default, ArUco selectable.
    float marker_world_n3[3 * SENTAI_MARKERS_MAX_WORLD];
    int   marker_n;
    int   marker_backend;  // SENTAI_MARKERS_BACKEND_{ARUCO, WHYCON}
    float marker_size_m;

    // Hover envelope.
    float z_hold_m;        // bringup altitude (e.g. 0.78)
    float vmax_m_s;        // relay magnitude (e.g. 0.10)
    float dur_relay_s;     // per-axis relay timeout (e.g. 30)
    float dur_hold_s;      // validation hold duration (e.g. 10)

    // Optional: starting hint for the autotune (helps convergence).
    float kp_hint_x;       // 0.0 = no hint
    float kp_hint_y;
} sentai_calib_bringup_ctx_t;

typedef struct {
    // Output (populated by run_bringup).
    float R_cam_to_body[9];
    float cam_offset_B[3];
    float kp_x, kp_y, kp_yaw;
    float fx, fy, cx, cy;       // future W21-T5
    // Quality.
    int   accepted;             // 0/1
    int   reject_code;          // sentai_calib_reject_t
    float ext_residual_deg;
    float hold_rms_drift_m;
    uint32_t total_duration_ms;
} sentai_calib_bringup_result_t;

// Orchestrates the full bringup sequence in a single FreeRTOS task:
//   1. Take off to z_hold_m
//   2. Sample-collection sweep (4-6 hover poses, varying XY + yaw)
//   3. Kabsch -> R_cam_to_body, cam_offset_B
//   4. commit_R + save_kv
//   5. autotune_x: relay X, ZN -> kp_x
//   6. autotune_y: relay Y, ZN -> kp_y
//   7. (future) autotune_yaw -> kp_yaw
//   8. validation hold: closed-loop with kp_x, kp_y for dur_hold_s,
//      assert rms_drift < threshold
//   9. save_kv all params
//  10. land
// Anything that fails calls abort + land + return reject_code.
int sentai_calib_run_bringup(const sentai_calib_bringup_ctx_t* ctx,
                              sentai_calib_bringup_result_t* out);
```

### MP one-liner wrapper

```python
# Pure orchestration, all heavy work in C.
sentai.calib.run_bringup(z=0.78, vmax=0.10, dur=30.0, backend="whycon")
# Or accept defaults from compiled-in ctx.
sentai.calib.run_bringup()
```

### INI persistence (replaces JSON)

Standard INI key=value, no sections needed for now (flat file).  Easy to
parse on ARM (no JSON dependency), easy to inspect with `cat` on a real
drone over USB CDC-ACM.

```ini
# /system/calib.ini  -- newline-separated key=value, ASCII, append-overwrite
schema=2
R_B_C=0.996194720,-0.087155737,0.000000000,0.087155737,0.996194720,0.000000000,0.000000000,0.000000000,1.000000000
cam_offset_B=-0.040000,0.000000,-0.020000
kp_x=0.390000
kp_y=0.390000
kp_yaw=0.000000
# Future T5:
# fx=288.300,fy=288.300,cx=160.000,cy=120.000
```

Parser: simple `strtok('\n')` + `strchr('=')` + `strtok(',')`, < 100 LoC.
Writer: `fprintf(f, "key=%s\n", value)` line-by-line.  No JSON library
dependency on ARM.  Schema field guards forward compatibility; old
firmware ignores unknown keys (anti-brick).

### Trigger: REPL command, NEVER auto-flight

Operator-stated 2026-05-21: "**nu rula automat calibrarea, se da comanda
din REPL pentru incepere — sa nu ne incurcam**".

At boot, if `!sentai_calib_is_calibrated()`:
- Boot completes normally (no automatic takeoff — anti-brick rule).
- A diag flag / SUBSYS state surfaces the missing calibration to the
  operator (e.g., `sentai.calib.is_calibrated() == False` in REPL).
- The drone WAITS for an explicit `sentai.calib.run_bringup()` call
  from the REPL (over USB CDC-ACM in the lab, over Crazyflie radio
  bridge in production bench).

This rule applies to BOTH SIM and ARM.  No spontaneous flights.  The
operator (or production technician) is always in the loop for the
initial flight command.  The drone CAN refuse to take off for a
mission if `is_calibrated()==False` — that's a separate guard, not
auto-bringup.

## WhyCon integration (W21-T1)

Today: `sentai_calib_task.cc:280` calls `sentai_aruco_get_latest(arr, 16)`.
This is a layering violation post-OP-S10-W19 (sentai.markers unified
namespace).  Refactor to `sentai_markers_get_latest(...)` so calib
works with ANY active backend without code changes.

## Task breakdown

| Task | What | LoC est | Smoke |
|------|------|---------|-------|
| **T1** | Refactor `sentai_calib_task.cc` to use `sentai_markers_*` | ~30 changes | s157 regression PASS |
| **T2** | KV persistance: new `sentai_kv.{h,c}`, replace `format_json/parse_json` | ~150 new + 50 changed | s188 KV smoke (round-trip on disk) |
| **T3** | Schema v2: add kp_x, kp_y, kp_yaw fields + commit_kp() API | ~80 | s189 v2 schema smoke |
| **T4** | `sentai_calib_run_bringup()` C-side orchestrator | ~200 | s187 Gazebo end-to-end |
| **T5** | Camera intrinsics auto-calibration (DEFERRED — own WP) | – | – |
| **T6** | SUBSYS_CALIB state + `is_calibrated()` boot surfacing (NO auto-flight) | ~30 | s190 boot-without-cal flag visible |

T1-T4 are the critical path; T5/T6 are follow-ups.

## Acceptance criteria

s187 Gazebo bringup PASS on the WhyCon square pad:
- Extrinsics: `drift_from_SDF_deg < 1.0°` (R), `||cam_offset_recovered - SDF||_inf < 5 mm`
- PID: `Kp_x, Kp_y ∈ [0.30, 0.50]` (s174 baseline 0.39 ± 0.05)
- Hold validation: rms_drift < 30 mm over 10 s, max_drift < 80 mm
- Persistence: `/system/calib.ini` written, drone reboots, calib state
  restored bit-identical, `is_calibrated()==True`

## Anti-cheat

Calib is still consuming only:
- WhyCon detections (via `sentai_markers_get_latest`) — camera frames
  flow only through bridge per [[sentai-sim-air-gapped-from-truth]]
- cf2 CRTP LOG telemetry (drone EKF pose for the world-side delta
  in Kabsch sample)

No GT injection.  s187 will host a `gt_recorder` post-mortem for
operator visibility but feedback into calib state is forbidden.

## Open questions for operator

1. **First-boot autotrigger** — RESOLVED 2026-05-21: NEVER auto-fly.
   Operator triggers from REPL.  is_calibrated()==False is surfaced as
   a flag; mission code refuses takeoff when uncalibrated.
2. **Service re-calibration** — periodic recheck (drift detection from
   PnP residuals) or only on operator command?  Default proposal: only
   on operator command for now; revisit when we have drift telemetry.
3. **Multi-pad support** — should calib accept a `pad_id` so different
   product variants ship with different pads, or one universal pad?
   Default proposal: universal WhyCon square 32×32 cm + a `pad_id`
   field in INI for future-proofing.
