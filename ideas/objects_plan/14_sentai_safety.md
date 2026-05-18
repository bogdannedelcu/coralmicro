# §25 — `sentai.safety` — firmware-side mission safety service

**WBS**: `OP-S10-W12`
**Opened**: 2026-05-18 (operator decision after s167 SafetyArucoBaseline
crisis showed host-side Python safety violates
[[missions-run-in-sentai-only]] hard rule).
**Authoritative spec**: top-level [`Safety.md`](../../Safety.md).
This chapter is the ObjectsPlan-side anchor.

## Purpose

A NASA/JPL-style supervised safety service that runs INSIDE firmware
(ARM `sentai_runtime` or `sentai_sim` POSIX build), independent of
mission and flight-control logic.  Mission MP arms checks at takeoff
(`sentai.safety.enable_aruco(n_min=4, max_loss_s=1.0)`) and polls a
single boolean (`sentai.safety.aborted()`) at flight-control cadence.
A worker task (`sentai_safety_task`) drives detection at camera FPS
using the EXISTING `sentai.aruco` + `sentai.camera` subsystems — no
pipeline duplicated.

## Why it's its own work package

Per `agent/embeded.md` §3.1 (strict layer separation) + §7.2
(structured event log) + CLAUDE.md core principle ("compute in C/C++,
MP for glue"), continuous per-frame mission-safety monitoring belongs
in C/C++ on the firmware, NEVER in host-side Python.  The host-side
`SafetyMonitor` shipped in s167 iter #6 violated both rules and is
retained only as an interim debug artefact, not extended.

## Scope (first cut)

| Check | Status | Trigger |
|---|---|---|
| ArUco-FOV (`enable_aruco`) | T2 SHIPPED | `n_dets < n_min` sustained for `max_loss_s` seconds (operator HARD RULE [[flowbaseline2-4markers-abort]]: default 4 markers / 1 s) |
| Altitude floor | T-future | PnP-z below floor sustained |
| EKF ceiling | T-future | EKF z above ceiling (catches post-crash integrator runaway) |
| Battery | T-future | V_batt below threshold |
| Link keepalive | T-future | Last CRTP packet older than threshold |

Plus an orthogonal **stale-feed watchdog**: if a feeder stops pushing
for `> STALE_TIMEOUT_S` (default 2 s), the check abort latches with
reason "feeder silent ..." — catches silent SafetyTask death so the
mission cannot fly blind.

## Architecture summary

```
sentai.camera (existing)
  └── sentai_camera_grab_gray_zerocopy()
       ▲
       │
sentai_aruco_detect()  (existing)
  ├── caches result via sentai_aruco_get_latest()
  │
  └── pushed via sentai_safety_on_aruco_result(n_dets, seq, ts)
                                                       │
                                                       ▼
                                       sentai_safety state machine
                                        ├── per-check streak / dwell
                                        ├── sticky abort flag
                                        └── 32-entry event ring

                                       ┌─── MP read (any context) ───┐
                                       │  sentai.safety.aborted()    │
                                       │  sentai.safety.reason()     │
                                       └──────────────────────────────┘
```

The **only new continuous loop** is `sentai_safety_task.cc`.  All
other subsystems (aruco, camera, calib, pipeline) are reused unchanged.

Future checks plug into the same pattern via dedicated push endpoints
(`sentai_safety_on_alt_pose`, `_on_ekf_state`, `_on_battery`,
`_on_link_keepalive`) — never reading sensors directly.

## MP API (minimal — operator-mandated "lucruri simple")

```python
sentai.safety.init()
sentai.safety.enable_aruco(n_min=4, max_loss_s=1.0)
sentai.safety.disable_aruco()
sentai.safety.task_start()
sentai.safety.task_stop()
sentai.safety.aborted() -> bool          # sticky
sentai.safety.reason() -> str            # "" if not aborted
sentai.safety.clear()                    # re-arm boundary only
```

Detailed diagnostics (snapshot, events, task stats) stay in the C
API — exposed via the Flight Recorder (§26 / `OP-S10-W13`) rather
than polluting MP with dicts.

## Anti-cheat invariants

- ArUco feed comes from the real camera pipeline.  Gazebo
  `/dynamic_pose` is NEVER consumed.  See
  [[sentai-sim-air-gapped-from-truth]] + [[cf2-sitl-cheat-odom-gt]].
- Abort flag is read-only from MP.  No `ignore_safety` API.
- `clear()` emits a `SENTAI_SAFETY_EV_CLEAR` event for audit trail.

## Task list (mirrors `wbs.md` OP-S10-W12)

T1 header • T2 state machine • T3 worker task • T4 MP binding (8 fns) •
T5 SIM CMake + dispatch + QSTR • **T6 EXP-s170 SafetyArucoBaseline** (in
progress, operator-named) • T7 mission MP port • T8 docs • T9 ARM build
+ ITCM budget + s127 gate • T10 migrate in-place PGM dump → sentai.fr
(depends OP-S10-W13) • T11 sentai_aruco frame_seq memoisation.

## Cross-references

- [`Safety.md`](../../Safety.md) — authoritative architecture + API
- `examples/sentai_runtime/sentai_safety.{h,cc}`
- `examples/sentai_runtime/sentai_safety_task.{h,cc}`
- `examples/sentai_runtime/bindings/modsentai_safety.c`
- `examples/sentai_runtime/experiments/s170_security_aruco_baseline/`
- §26 — `sentai.fr` flight recorder (sister WP)
- `wbs.md` `OP-S10-W12` full task list
