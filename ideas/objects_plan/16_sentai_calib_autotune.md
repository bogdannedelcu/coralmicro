# §27 — `sentai.calib.autotune` — in-flight Flow loop autotuner

**WBS**: `OP-S10-W14` (opened 2026-05-18, operator-approved 2026-05-18:
"as vrea sa fie in sentai.calib toata povestea cu calibrarea, dar MP
sa comande doar start/stop... daca e nevoie sa jurnalizeze sa
foloseasca sentai.fr").
**Goal**: stable, auto-calibrated Flow drift-correction loop in SIM
(end-to-end FlowBaseline mission post-cheat-removal that converges
without manual Kp tuning).
**Authoritative C API**: `examples/sentai_runtime/sentai_calib.h`
(extension of existing OP-S6-W1 module).
**Scope** (operator-narrowed 2026-05-18): perception → control loop
parameters, NOT drone-physics sysID.  cf2's inner attitude PID is
already tuned and tracks setpoint reliably — leave it alone.

## 1. Problem statement

After the cf2 SITL cheat-plugin removal ([[cf2-sitl-cheat-odom-gt]]),
the FlowBaseline mission needs a position-feedback gain `Kp_flow` to
correct EKF drift via VPE-forwarded ArUco pose.  We learned manually
during s167 F2 that:
- `Kp_flow` too low → drone drifts 10–30 cm before correction kicks in
- `Kp_flow` too high → drone oscillates on every PnP update (jitter)
- The optimum varies with: PnP noise floor, VPE rate, cf2's internal
  position-loop tuning, camera-IMU time offset `td`.

Operator pivot 2026-05-18: instead of hand-tuning per drone unit, run
a brief automatic ramp at takeoff.  This is a closed-loop
auto-tuning problem, not a system-identification problem.

## 2. SOTA method: relay auto-tuning (Åström-Hägglund 1984)

Drive the closed loop with a relay (binary on/off) instead of a
linear gain.  The loop oscillates spontaneously with measurable
period `T_u` and amplitude `a_y`.  Ziegler-Nichols' "ultimate gain"
formulae then give the optimal `Kp`.

### Velocity-only excitation (operator-mandated 2026-05-18)

We have NO reliable world-frame coordinates yet (R_cam_to_body
calibration is the chicken-and-egg blocker; the whole point of this
WP is to bootstrap it).  We therefore CANNOT use position setpoints
(`sentai_crazy_go_to(x,y,z,…)`) — that command demands a stable
world frame.  We use **velocity setpoints** (`sentai_crazy_hover(vx,
vy, yaw_rate, z_distance)`), which only require local body-frame
direction.

The Flow loop being tuned is therefore the **velocity-feedback
loop**:
```
input  : drift_y (m, PnP-measured displacement from chosen anchor)
gain   : K_p          (1/s — converts drift to velocity setpoint)
output : v_cmd = K_p · drift                              (m/s)
plant  : cf2 inner-loop velocity tracker → world motion
```

Relay form: switch v_cmd between `+v_max` and `-v_max` based on the
sign of measured drift.  Drone oscillates around `drift = 0`.  We
measure `T_u` (full period) and `a_y` (peak drift amplitude).

```
v_cmd = sign(-drift) · v_max     # relay output
K_u   = 4 v_max / (π a_y)        # describing function
                                  # units: [m/s] / [m] = 1/s

K_p   = 0.5  · K_u                # ZN P-only       (recommended start)
K_p   = 0.45 · K_u                # Tyreus-Luyben PI (robust to deadtime)
T_i   = 2.2  · T_u
```

PnP latency (`td` ≈ 10–30 ms in SIM) shows up as a phase lag and
makes `T_u` slightly larger than the pure-system value — the formula
handles this automatically (that's exactly the point of describing-
function analysis).  No explicit deadtime model needed for ZN.

### Why relay, not gradient or BayesOpt
- Relay auto-tune converges in **one excitation cycle** (~5–10 s on
  this drone): single oscillation period gives Tu, two cycles give
  amplitude.  Bayesian Optimization needs 10–30 hover trials.
- Compute is O(N) per sample (peak detector), no GP inference.
- The output is a single Kp + Tu — easy to persist, easy to defend
  in a thesis chapter ("classical autotune, well-cited").

### Why this drone is a good fit
- `sentai_crazy_hover(vx, vy, yaw_rate, z_distance)` takes a
  body-frame velocity vector and an altitude hold — no world frame
  needed.  cf2's altitude PID handles z, our relay drives vx (or
  vy).
- PnP gives 30 Hz position relative to the marker grid with ~1 cm
  noise — plenty to resolve a ±5 cm drift amplitude.
- The drone stays within the camera FOV during low-amplitude
  velocity excitation at z=0.6 m — preserves the
  [[flowbaseline2-4markers-abort]] invariant (n_dets stays at 4).

## 3. Architecture — unified `sentai.calib` C++ task (state ↔ task split)

Operator-mandated 2026-05-18 ("cred ca si sentai.calib tot task de
c++ ar trebui sa fie..."): the existing one-shot `sentai.calib`
library is promoted to a long-running C++ task (mirrors
`sentai.safety` / SafetyTask).  Autotune is one MODE of the task,
not a separate task.

```
                    ┌──────────────────────────────────────────┐
                    │  sentai_calib.{h,cc}  (state + math)      │
                    │   - R_cam_to_body, cam_offset_B           │
                    │   - flow_gains {Kp_x, Kp_y, td_ms}        │
                    │   - Kabsch + Jacobi SVD                   │
                    │   - relay generator + peak detector + ZN  │
                    │   - save/load /system/cam_calib.json      │
                    │   - save/load /system/flow_gains.json     │
                    └─────────────────┬────────────────────────┘
                                      │ used by
                                      ▼
            ┌──────────────────────────────────────────────────┐
            │ sentai_calib_task.{h,cc}                          │
            │   Single FreeRTOS task @ tskIDLE_PRIORITY + 2     │
            │   period: 33 ms (camera FPS)                      │
            │                                                   │
            │   Mode FSM:                                       │
            │      IDLE             — running, doing nothing     │
            │      OBSERVING        — accumulate PnP + cf2 stats │
            │      KABSCH_COLLECT   — fit R_cam_to_body          │
            │      AUTOTUNE_RELAY   — drive ±v_max, measure ZN   │
            │      ABORTED          — safety latched / op stop   │
            │                                                   │
            │   Each tick (per mode):                           │
            │      - read PnP via sentai_aruco_get_latest()     │
            │      - read cf2 altitude via sentai_crazy_get_*   │
            │      - decide command (hover vx/vy or NOOP)        │
            │      - write via sentai_crazy_hover               │
            │      - push samples → sentai.fr.scalars           │
            │      - check sentai.safety.aborted() → ABORTED    │
            └──────────────────────────────────────────────────┘

   READS:  sentai.aruco (cached PnP)
           sentai.crazy.get_altitude
           sentai.safety.aborted
   WRITES: sentai.crazy.hover (velocity setpoint, no world frame)
           sentai.fr.scalars / events
   SAFETY: caller arms sentai.safety.enable_aruco(4, 1.0) before
           setting AUTOTUNE_RELAY mode — if the relay drives the
           drone into a marker-poor pose, abort latches and we
           transition to ABORTED.
```

**Why a single multi-mode task, not multiple tasks**: the operations
are mutually exclusive (you don't Kabsch-fit while relaying — they
need different excitation), and they share state (R, cam_offset,
flow_gains, sample buffers).  One task = one state-machine =
trivially serialised; matches the FreeRTOS "one writer per
resource" discipline from `agent/embeded.md` §3.

## 4. MP API (minimal — operator 2026-05-18)

"Calib e task care e lansat din MP dupa lansare, sau ceva similar...
si mp verifica doar daca s-a terminat.  Apoi mai vedem..."
followup: "eventual poate ii da ca parametri altitudinea la care se
calibreaza sau ceva la initializare... pozitie markeri, dimensiuni...
ceva de initializare care are sens."

Final shipped surface (phase 1):

```python
# Init — persistent context (altitude, marker geometry); idempotent.
sentai.calib.init(z_hold_m,             # altitude held during relay
                   marker_grid_dx_m,     # X spacing between markers (FOV bound)
                   marker_grid_dy_m,     # Y spacing
                   marker_size_m)        # ArUco edge length (m)
                                          -> int

# Existing Kabsch sample API preserved (sentai.calib.run_kabsch etc.).

# NEW autotune surface (4 fns)
sentai.calib.task_start(axis_str, dur_s, vmax_m_s)   -> int
sentai.calib.task_stop()                              -> int
sentai.calib.is_done()                                -> bool
sentai.calib.get_kp(axis_str)                         -> float   # -1.0 if not converged

# Existing save/load extended to also persist flow_gains.
sentai.calib.save() / load()                          -> int
```

Mission MP pattern:

```python
# At boot / mission entry — give calib the world geometry it needs.
sentai.calib.init(z_hold_m=0.60,
                   marker_grid_dx_m=0.20,
                   marker_grid_dy_m=0.15,
                   marker_size_m=0.0625)

sentai.crazy.takeoff(0.6, 2.0)
sentai.rtos.sleep_ms(2500)
sentai.safety.enable_aruco(4, 1.0); sentai.safety.task_start()

sentai.calib.task_start("x", 30.0, 0.10)    # autotune X axis, 30 s, ±10 cm/s
while not sentai.calib.is_done():
    if sentai.safety.aborted(): break
    sentai.rtos.sleep_ms(100)
sentai.calib.task_stop()

kp_x = sentai.calib.get_kp("x")
sentai.calib.save()
sentai.crazy.land(0.0, 2.5)
```

`init` lets the autotune task:
- pick the `z_hold` for `sentai_crazy_hover`'s altitude argument
  (operator: "altitudinea la care se calibreaza")
- bound v_max so that v_max × max-half-period < 0.5 × min(grid_dx,
  grid_dy) — drone never drifts more than half a marker spacing,
  keeping all 4 markers in FOV
- pass marker_size_m to `sentai_aruco_set_intrinsics` if not already set

Internal mode FSM (IDLE / OBSERVING / KABSCH_COLLECT / AUTOTUNE_RELAY
/ ABORTED + AUTOTUNE sub-states ARMING / EXCITING / SETTLING /
DONE_OK / DONE_FAIL) stays inside the task but is NOT exposed to MP
yet — operator: "apoi mai vedem".  When phase 2 needs mode_kabsch /
mode_observe, we add the MP surface; not before.

## 5. Algorithm — AUTOTUNE_RELAY mode (per axis)

Velocity-only relay (operator-mandated; no world-frame go_to).
"Anchor" = the PnP position at mode entry; that becomes the local
zero-drift reference for the relay decision.  We do NOT need an
absolute world frame — the relay just needs *which side* of the
anchor the drone is currently on.

```
on entry (sub_state = ARMING):
  anchor_xy   = current PnP (x,y) at entry tick
  z_hold      = sentai_crazy_get_altitude()
  v_max       = caller-supplied (default 0.10 m/s)
  prev_drift  = 0
  cycles      = []           # ring: {t_ms, drift_m}
  cycle_cnt   = 0
  sub_state   = EXCITING
  t_start     = now_ms()

each 33 ms tick (camera period; one of {x, y} is the tuned axis):
  pnp = sentai_aruco_get_latest()
  if pnp.n_dets < 4: continue                # let safety handle it
  drift = pnp.world[axis] - anchor_xy[axis]
  ts    = now_ms()
  sentai_fr_push_scalar("autotune_drift", drift, ts)

  # Relay: drive velocity AGAINST the drift
  v_cmd = (drift > 0) ? -v_max : +v_max
  if axis == "x":
      sentai_crazy_hover(v_cmd, 0.0, 0.0, z_hold)
  else:                                      # axis == "y"
      sentai_crazy_hover(0.0, v_cmd, 0.0, z_hold)

  # Peak detection (drift derivative zero-crossing)
  if sign_changed(drift, prev_drift):
      cycles.push({ts, drift})
      cycle_cnt += 1
      sentai_fr_push_event("autotune_peak",
                             "cycle=%d t=%u drift=%.3f", cycle_cnt, ts, drift)
  prev_drift = drift

  # Done conditions (sub_state transitions)
  if cycle_cnt >= MIN_CYCLES (=6) AND
     amplitude_stable(cycles, tol=0.15):     # ±15% over last 4 cycles
      T_u    = 2 * mean(inter_peak_time)     # peak-to-peak is half-period
      a_y    = mean(|peak_drift|)
      K_u    = 4 * v_max / (pi * a_y)        # units: 1/s
      K_p    = 0.5 * K_u                     # ZN P-only
      flow_gains.Kp[axis] = K_p
      sub_state = DONE_OK
      sentai_crazy_hover(0, 0, 0, z_hold)    # park (zero velocity)

  if cycle_cnt >= MAX_CYCLES (=20)        OR
     (now - t_start) >= dur_s_max:
      sub_state = DONE_FAIL                  # never stabilised
      sentai_crazy_hover(0, 0, 0, z_hold)
```

`sentai_crazy_hover(vx, vy, yaw_rate, z_distance)` only requires the
local altitude — no world frame.  cf2's onboard PID handles the
velocity tracking; we drive only the SIGN of vx/vy.

### Side-product: `td` estimation (deferred to W14 phase 2)

During EXCITING, PnP velocity (from differencing positions) and the
commanded `v_cmd` series should be in phase modulo `td`.  Compute
`td = argmax_τ ∫ v_pnp(t) · v_cmd(t-τ) dt` with CMSIS-DSP
`arm_correlate_f32` over a 5 s sliding window.  Convergence ~3 s of
relay motion.  **Phase 2** — initial T3 ships without `td`.

## 6. Anti-cheat / ground-rules compliance

- **`sentai_sim` AIR-GAPPED** — autotune reads PnP from
  `sentai_aruco_get_latest()` (camera path) + cf2 altitude (CRTP LOG).
  NO Gazebo `/dynamic_pose` consumed.  Verified by audit.
- **Compute in C/C++** — full state machine + ZN formula in
  `sentai_calib_autotune.cc`.  MP only commands start/stop + reads
  scalars.  No MP per-frame math.
- **HARD RULE [[flowbaseline2-4markers-abort]]** — autotune arms
  `sentai.safety.enable_aruco(4, 1.0)` before EXCITING; if markers
  leave FOV due to unexpected motion, safety latches abort and the
  task reads `sentai.safety.aborted()` → transitions to DONE_FAIL.
- **No heavy data through MP** — drift samples go straight to
  `sentai.fr` from C++; MP never sees raw scalars.
- **English-only artefacts** — this doc + all code/comments in
  English per CLAUDE.md hard rule.
- **WBS append-only** — new W14 / T1..T7, no renumbering.

## 7. Implementation map

| File | Status | Purpose |
|---|---|---|
| `examples/sentai_runtime/sentai_calib.h`              | extend | + autotune state enum, getters |
| `examples/sentai_runtime/sentai_calib.cc`             | extend | + `flow_gains` struct, save/load JSON |
| `examples/sentai_runtime/sentai_calib_autotune.h`     | NEW    | Autotune-specific API contract |
| `examples/sentai_runtime/sentai_calib_autotune.cc`    | NEW    | State machine + ZN math + peak detector |
| `examples/sentai_runtime/sentai_calib_autotune_task.h`| NEW    | Worker task lifecycle |
| `examples/sentai_runtime/sentai_calib_autotune_task.cc`| NEW   | 33 ms drive loop + sentai_crazy_go_to |
| `examples/sentai_runtime/bindings/modsentai_calib.c`  | extend | + 6 MP fns (start/stop/status/get_kp/save/load) |
| `examples/sentai_runtime/CMakeLists.txt`              | edit   | + new .cc files |
| `sim/CMakeLists.txt`                                  | edit   | + new .cc files |
| `examples/sentai_runtime/experiments/s172_flow_autotune_baseline/` | NEW | SIM smoke + verdict |

CMSIS-DSP usage: `arm_correlate_f32` reserved for phase-2 `td` estimation.
Phase-1 relay tuning is scalar-only, < 5 µs/tick on M7.

## 8. SIM smoke test plan (`s172_flow_autotune_baseline`)

Mission MP (runs in sentai_sim per [[missions-run-in-sentai-only]]):

```python
sentai.camera.init()
sentai.safety.init()
sentai.calib.load()                       # load R_cam_to_body if present
sentai.fr.init()
sentai.fr.open("scalars", "/tmp/.../scalars.csv")
sentai.fr.open("events",  "/tmp/.../events.csv")
sentai.fr.task_start()

sentai.crazy.init()
sentai.crazy.arm()
sentai.crazy.takeoff(0.60, 2.0)
sentai.rtos.sleep_ms(2500)

# Arm safety BEFORE relay so 4-marker invariant is enforced
sentai.safety.enable_aruco(4, 1.0)
sentai.safety.task_start()

# Run autotune on X axis
sentai.calib.autotune_start("x", 30.0, 0.05, (0.0, 0.0, 0.6))
while True:
    st = sentai.calib.autotune_status()
    if st[0] in (4, 5):   # DONE_OK / DONE_FAIL
        break
    if sentai.safety.aborted():
        sentai.calib.autotune_stop()
        break
    sentai.rtos.sleep_ms(100)

kp = sentai.calib.autotune_get_kp("x")
sentai.fr.push_event("autotune_result", "kp=%f state=%d" % (kp, st[0]))
sentai.calib.autotune_save()

sentai.crazy.land(0.0, 2.5)
sentai.rtos.sleep_ms(3000)
sentai.safety.task_stop()
sentai.fr.task_stop()
```

Verdict (host-side post-mortem):
- PASS if: state==DONE_OK, Kp ∈ [0.5, 5.0] (sanity bounds), drone landed ≤10 cm from origin, GT recorder logged ≥6 oscillation cycles, no safety abort, flow_gains.json written.
- Optional follow-up gate (W14 done condition): re-run a baseline FlowBaseline mission with the persisted Kp; require `dist_mean ≤ 8 cm` vs ~10 cm hand-tuned baseline.

## 9. WBS

| T# | Task | Status |
|---|---|---|
| T1 | Design doc + math + API (this file) | ✅ |
| T2 | wbs.md OP-S10-W14 row + sub-tasks | ⬜ |
| T3 | sentai_calib_autotune.{h,cc} + task split | ⬜ |
| T4 | MP binding extension (6 fns) | ⬜ |
| T5 | CMake + dispatch wiring + QSTR regen | ⬜ |
| T6 | s172_flow_autotune_baseline experiment | ⬜ |
| T7 | flow_gains.json persistence via FxUser | ⬜ |

Phase 2 (later WP / future work):
- Y axis tuning (mirror X)
- `td` estimation via gyro × PnP cross-correlation
- Re-tune trigger on Kp covariance growth (drift parameters)

## 10. Cross-references

- [[op-s6-w1-calib-shipped]] — existing `sentai.calib` Kabsch / SVD
- [[op-s8-w1-cf2-sim-honest]] — parent crisis (replaced cheat plugin;
  this WP is the long-term fix for the manual F2 calibration that
  s167 hand-rolled)
- [[op-s10-w12-w13-shipped]] — `sentai.safety` arms the autotune
  pre-excitation guard; `sentai.fr` swallows all autotune logs
- [[flowbaseline2-4markers-abort]] — HARD RULE preserved during ±5 cm
  relay (markers stay in FOV at z=0.6 m)
- [[no-heavy-data-through-mp]] — MP never sees raw drift scalars;
  `sentai.fr.push_scalar` from C++ side only
- [[missions-run-in-sentai-only]] — s172 mission MP runs inside SIM
- [[sim-test-must-return-home]] — ≤10 cm verdict gate
- Åström & Hägglund 1984 *Automatica* — relay auto-tuning origin paper
- Ziegler & Nichols 1942 — ultimate-gain formulae
