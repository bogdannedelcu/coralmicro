---
name: sim-mission-flight
description: Canonical sequence for writing a MicroPython mission that flies cf2 SITL inside sentai_sim. Codifies the init order, takeoff settle, hover priming, detection cadence, and journal/summary patterns that EVERY SIM mission needs. Use when writing a new mission_sNNN.py (or debugging one that fails at setup/takeoff/no-markers).
---

# /sim-mission-flight

The pattern below is the load-bearing recipe for flying cf2 SITL in Gazebo from a SentAI mission.  Every regression in s127..s187 traces to skipping one of these steps.  Read this BEFORE writing a new mission; reproduce ALL steps even if they seem redundant.

## Reference implementation

`examples/sentai_runtime/experiments/s182_whycon_gazebo_eval/mission_s182.py` is the canonical mission — copy from it.  s187 (calib bringup) is the simpler descendant that wraps `sentai.calib.run_bringup()`.

## Mandatory init order (in this exact sequence)

```python
import sentai

JOURNAL_NAME = "mission_sNNN_journal.txt"   # bare filename — sim_fs_resolve()
SUMMARY_NAME = "mission_sNNN_summary.json"  # writes them under FS_ROOT

# 1. Open journal FIRST.  Use bare filename, NOT absolute path —
#    sim_fs_resolve rejects absolutes and journal_open returns -1
#    silently.  Future ToDo: auto-open at boot (Task #6 in WBS).
sentai.sim.journal_open(JOURNAL_NAME)

# 2. Calib state load (loads /system/calib.ini if present; no-op otherwise).
sentai.calib.init()

# 3. Markers init FIRST — backend must be live before takeoff so the
#    detection cache exists before missions tick it.
sentai.markers.init("whycon")           # or "aruco"
sentai.markers.set_intrinsics(288.3, 288.3, 160.0, 120.0)   # bridge 320x240
sentai.markers.set_marker_size(0.1088)  # WhyCon outer ring; ArUco edge ≈ 0.094

# 4. Crazy link init — UDP 19850 to cf2 SITL.
sentai.crazy.init()

# 5. Arm + sleep 300 ms.  Arm sends an MAVLink/CRTP arm packet; cf2
#    needs ~300 ms to acknowledge.
sentai.crazy.arm()
sentai.rtos.sleep_ms(300)

# 6. pose_subscribe — RETRY LOOP up to 30×200 ms.  cf2 TOC download
#    takes 1-3 s after link is up.  First subscribe returns -3 (no TOC);
#    keep trying.  Without this the mission has NO pose feedback.
sub_rc = -3
for _try in range(30):
    try:
        sub_rc = sentai.crazy.pose_subscribe(50)
    except (AttributeError, RuntimeError):
        sub_rc = -99
    if sub_rc == 0:
        break
    sentai.rtos.sleep_ms(200)
sentai.rtos.sleep_ms(500)               # let TOC settle
```

## Takeoff settle (mandatory)

```python
Z_HOLD       = 0.78
TAKEOFF_DUR  = 2.5
SETTLE_S     = 4.0

# `takeoff` NOT `hl_takeoff` (the latter does NOT exist).
sentai.crazy.takeoff(Z_HOLD, TAKEOFF_DUR)
sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)   # take-off ramp
sentai.rtos.sleep_ms(int(SETTLE_S * 1000))             # soak (cf2 EKF settles)

# Drop HighLevel commander + prime low-level Generic Setpoint.  Without
# hl_stop, hover() commands are IGNORED — HL trajectory wins.  Without
# the 5× hover priming, cf2's first low-level packet arrives too late
# and the drone DROPS by ~10 cm.
sentai.crazy.hl_stop()
for _ in range(5):
    sentai.crazy.hover(0.0, 0.0, 0.0, Z_HOLD)
    sentai.rtos.sleep_ms(30)
```

## Detection cadence (load-bearing on both SIM and ARM)

SafetyTask CAN drive detection at ~30 Hz — but only if the mission
calls `sentai.safety.task_start(...)` first.  Plain bringup missions
typically do NOT start SafetyTask (they don't need abort-on-marker-loss).
So the rule is: SOMEONE has to drive the detector each tick or
`sentai_markers_get_count()` stays at 0.  Options:

1. **Mission-driven** (s182 pattern):
   ```python
   n = sentai.markers.detect_from_camera()   # per tick
   ```
2. **SafetyTask-driven** (s170 pattern):
   ```python
   sentai.safety.task_start(period_ms=33)
   ```
3. **Orchestrator-driven** (s187 / sentai.calib.run_bringup pattern —
   the orchestrator already ticks detect internally):

```python
# MP-side cadence (mission-driven):
n = sentai.markers.detect_from_camera()   # one tick

# C-side cadence (e.g. inside sentai_calib_bringup orchestrator):
#   sentai_camera_grab_gray_zerocopy(&buf, &w, &h, &seq, &ts);
#   sentai_markers_detect_frame(buf, w, h, seq, ts);
```

The MP `detect_from_camera()` is JUST an MP binding — there is NO
C-side `sentai_markers_detect_from_camera()` symbol.  C consumers
must do the grab + detect_frame pair themselves.

**Zerocopy invariant (load-bearing on ARM)**: `grab_gray_zerocopy`
returns a CONST POINTER into the camera ring buffer.  Never memcpy
the frame.  Hot ARM hardware loop must not waste cycles on copies —
detect_frame reads from the returned pointer in place.

## Hover semantics (Generic Setpoint)

```python
sentai.crazy.hover(vx_body, vy_body, yaw_rate_deg_s, z_absolute_m)
```

- `vx_body`, `vy_body`: instantaneous body-frame velocity m/s.
- `yaw_rate_deg_s`: yaw rate.  0 = lock yaw.
- `z_absolute_m`: ABSOLUTE altitude above takeoff — cf2's altitude PID
  + baro hold it.  Not relative.

**Send every tick (30 Hz)** — cf2's Generic Commander watchdog cuts
motors after ~1 s without a setpoint.

## VPE forwarder pattern (mandatory if Z drift matters)

Without VPE (Vision Position Estimate) feedback, cf2 EKF drifts on Z
because baro noise alone isn't enough to anchor altitude.  Symptom:
drone slowly climbs or sinks during hover.

```python
# Once per tick (or 30 Hz), after a fresh detection:
dets = sentai.markers.get_pose_tuple(...)
drone_W = derive from PnP   # mw[k] - R · tvec_cam[k]  (median across markers)
# Pack ExtPose canal 1, 29 bytes, type=8 prefix:
import struct
p = struct.pack("<Bffffffff", 8, dx, dy, dz, qx, qy, qz, qw)
sentai.crazy.send_crtp(6, 1, p)
```

Without this, **drone Z drifts even though cf2 EKF reports z=Z_HOLD**.
This is observed visually in Gazebo: drone slowly sinks.

## Landing

```python
sentai.crazy.land(LAND_DUR)            # NOT hl_land — same naming bug as takeoff
sentai.rtos.sleep_ms(int((LAND_DUR + 1.0) * 1000))
sentai.crazy.disarm()
```

## Summary write + journal close

```python
sentai.fs.write(SUMMARY_NAME, _ser_val(summary))   # JSON-as-string
sentai.sim.journal_close()
return summary
```

## REPL invocation (from the host run.sh)

```bash
echo "import mission_sNNN; r = mission_sNNN.run(); print('FINAL:', r['status'])" \
    | timeout 240 build-sim/sim/sentai_sim > $WORKDIR/sentai_repl.log 2>&1
```

NEVER pass multi-line code with indentation — the line-buffered embed
REPL chokes on Python-block indentation.  Put missions in `.py` files
in `FS_ROOT` and `import` them.

## Pitfalls already burned (do NOT redo)

- `sentai.crazy.hl_takeoff` / `hl_land` — DO NOT EXIST.  Use plain
  `takeoff` / `land`.  (`hl_stop` does exist — confusing naming.)
- `sentai.markers.set_marker_world(tuple_of_tuples)` — fails, the
  binding wants `bytes/bytearray` packed via `struct.pack`.  For the
  calib bringup orchestrator, the constellation matcher is internal so
  you don't need set_marker_world at all (the orchestrator does
  forward-projection-based association internally).
- `time.sleep` / `import time` — embed port doesn't have `time`.  Use
  `sentai.rtos.sleep_ms(ms)`.
- f-strings (`f"..."`) — embed port doesn't support them.  Use
  `"{}".format(...)`.
- Multi-line `try:` blocks pasted into REPL — IndentationError.  Files
  only.
- Absolute paths to `journal_open` — `sim_fs_resolve` rejects.  Bare
  filename only.
- Skipping the pose_subscribe retry loop — single call returns -3
  (no TOC).  Drone never gets EKF feedback.
- Skipping `hl_stop` between takeoff and hover — Generic Setpoints
  are ignored, HL trajectory wins, mission hovers at wrong altitude.

## Cross-cutting

- Anti-cheat: missions consume ONLY `sentai.markers.*` (camera frames
  via the bridge) and `sentai.crazy.*` (cf2 CRTP telemetry).  Never
  GT injection.  See [[sentai-sim-air-gapped-from-truth]].
- gt_recorder runs host-side ONLY (post-mortem only).  See
  [[gt-recorder-tool]].

## See also

- `[[sim-launch]]` for the cleanup + Gazebo launch invariants.
- `[[crazysim-debug]]` for the silent-failure recovery recipes.
- `[[op-s10-w14-autotune-converged]]` for the autotune-validated
  takeoff+settle parameters (Z_HOLD=0.78, SETTLE_S=4.0).
- `mission_s182.py` — full reference mission.
