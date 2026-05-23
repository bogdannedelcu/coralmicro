# s193 iter2 — Strategy A: RPYT takeoff + ExtPos warmup + run_bringup

**Status**: ⬜ READY — awaiting operator validation BEFORE first run.

**Hypothesis**: Combining (a) Classic RPYT takeoff (proven iter-21 of
s190, GT peak 0.91 m, operator-confirmed "perfecta decolarea"), (b)
ExtPos warmup so cf2 Kalman converges to PnP, and (c) the now-working
T10 detector on the 7-marker asymmetric pad, `sentai.calib.run_bringup()`
reaches `DONE_OK` and hits all acceptance thresholds.

**Change vs iter1**: iter1 used HL takeoff and FAILED (`ASCENT_FAIL`).
Root cause: cf2 Kalman hallucinated climb without baro/TOF → motors
never physically lifted (GT z stuck at 0.015 m).  iter2 replaces HL
takeoff with the Classic RPYT path proven in s190 iter-21 / s192 iter1.

**Date**: 2026-05-23

---

## Full process summary — validate before run

### Input world

`sentai_whycon_small.sdf` (upstream CrazySim, modified 2026-05-23
per `ideas/external_patches.md`):

```
7 markers @ z=0.005 m:
  NW (-0.08, +0.08)    NE (+0.08, +0.08)
  W  (-0.06,  0.00)    E  (+0.06,  0.00)
  SW (-0.08, -0.08)    SE (+0.08, -0.08)
  N  (+0.02, +0.10)   ← asymmetric, breaks PnP dual-solution
```

cf2 spawns at (0, 0, 0), no baro (operator HR, plugin patch landed
2026-05-22), no TOF cheat (reverted same day), no flow_deck.  Only
state inputs to cf2 EKF: IMU (1000 Hz) + ExtPos from mission (when
markers visible).

### Mission flow (10 phases)

| # | Phase | Commander | What it does | Pass-out condition |
|---|---|---|---|---|
| 1 | setup | – | `sentai.calib.init() clear()`, `camera.init()`, `markers.init("whycon")` + intrinsics + size + `set_marker_world(7)`, `safety.init()`, `crazy.init() arm() pose_subscribe()` | rc=0 each call |
| 2 | zero-unlock | RPYT(0,0,0,0) | 1.5 s × 30 Hz of zero-thrust packets (cf2 firmware requires before nonzero) | duration elapsed |
| 3 | thrust ramp | RPYT(0,0,0,thr) | Open-loop ramp T_BASE=30000 → T_MAX=37000 over 6 s, watching for first valid PnP (≥4 markers, z>0) | 2 consecutive valid PnP frames |
| 4 | PD altitude lock | RPYT(0,0,0,thr) | `thr = T_HOVER + Kp·(z_target − z_pnp) − Kd·vz_filt`<br>z_target = first PnP z (~0.3–0.4 m typically) | \|vz_filt\| < 10 cm/s × 4 ticks |
| 5 | **ExtPos warmup** | **RPYT** (continues PD) + ExtPos | Feed `ExtPos(x,y,z)` from PnP at 30 Hz so cf2 Kalman anchors to vision.  **CONTINUE PD thrust** (iter-21 fix — no jump to fixed T_HOVER which re-accelerated drone +16 cm). | \|ekf−pnp\| < 10 cm on all 3 axes × 5 ticks → cf2 EKF "warm" |
| 6 | handoff to hover | hover(0,0,0,z) | 6 × 33 ms = 200 ms of Generic Commander hover packets to engage HL mode before orchestrator spawns | duration elapsed |
| 7 | **`run_bringup()`** | orchestrator-driven (hover + ExtPos internally) | C-side FreeRTOS task runs 6 internal phases (see below) | `bringup_is_done() == True` OR timeout |
| 8 | (in 7) assert | – | `sentai.calib.assert_calibrated()` should pass IFF orchestrator accepted | guard check |
| 9 | (in 7) return | hover + go_to | (no XY return this iter — drone stays near takeoff origin) | – |
| 10 | land | RPYT ramp-down | T_HOVER−2000 → 0 over 2.5 s, then 10 zero-thrust packets, disarm | duration elapsed |

### What `sentai.calib.run_bringup()` does internally (phases 1–6)

The C-side orchestrator (`sentai_calib_bringup.cc`) runs as its own
FreeRTOS task spawned by `bringup_start()`.  It uses Generic
Commander hover() at 10 Hz internally, plus its own ExtPos stream
from PnP-derived drone_W (no GT injection).

| Phase | Name | What it does | Pass-out |
|---|---|---|---|
| 1 | **SAMPLE** | Cross sweep (4 pure-axis poses): `+X, +Y, −X, −Y` at radius=0.025 m.  Per pose: 1 s travel @ vmove=0.025 m/s + 2 s settle + 0.5 s capture (5 reads averaged).  Per-read: trigger detect_frame, associate each detection to nearest registered marker via forward-projection (NOT raw marker.id — WhyCon IDs are unstable scan-order), accumulate per-marker tvec + pixel pos + drone EKF pose.  ALSO sends ExtPos from PnP-derived drone_W at 30 Hz for cf2 Kalman anchor. | n_samples ≥ MIN_SAMPLES |
| 2 | **KABSCH** | Solve R_cam_to_body via Jacobi SVD on tvec ↔ marker_world − drone_world correspondences.  Then `cam_offset_B = mean(marker_W − R·tvec − drone_W)` across samples.  Gate on ext_quality.accepted (residual deg < 5°). | quality.accepted |
| 3 | **AUTOTUNE_X** | Spawn `sentai_calib_task_start(AXIS_X, dur_relay=30s, vmax=0.06)` — Åström-Hägglund relay-feedback autotune.  Drives drone left/right via velocity hover commands, observes period of oscillation, derives kp_x via Ziegler-Nichols.  If relay no-converge: soft fallback to kp=0.39 (s174 baseline). | task done |
| 4 | **AUTOTUNE_Y** | Same on Y axis. | task done |
| 5 | **HOLD** | `sentai_calib_hold_start(kp_x, kp_y, vmax_clip=3·vmax, 10 s)`.  Closed-loop XY position hold via PnP feedback with the freshly-tuned gains.  Measures rms drift + max drift over 10 s. | rms < 30 mm AND max < 80 mm |
| 6 | **SAVE** | Persist to `/system/calib.ini` (schema v2, key=value text).  Format: `R_B_C=…,cam_offset_B=…,kp_x=…,kp_y=…`. | INI written |

After phase 6 → `last_phase = DONE_OK`, `accepted = True`,
`assert_calibrated() → True`.  Otherwise `DONE_FAIL` with
`reject_code` in `{INVALID_CTX, FEW_SAMPLES, KABSCH_QUAL,
AUTOTUNE_X, AUTOTUNE_Y, HOLD_DRIFT, SAVE, SAFETY, ABORTED}`.

### Key parameters

```python
# Pad
MARKER_WORLD       = 7 tuples (NW, NE, W, E, SW, SE, N)
MARKER_DIAMETER_M  = 0.0544       # small pad outer ring
FX/FY              = 288.3 px
CX/CY              = 160 / 120

# Takeoff PD (s190 iter-21 — physics-derived from m=0.0282 kg, F_hover=0.277 N)
T_BASE_U16            = 30000     # ESC threshold
T_HOVER_NOMINAL       = 36000     # empirical hover (SDF says 41900, plugin -15%)
T_MAX_U16             = 37000     # ramp cap
T_MIN_U16             = 22000     # below → free-fall
T_MAX_HOLD_U16        = 40000     # PD-hold cap
KP_THRUST_PER_M       = 10000.0   # ω_n=1.5, ζ=0.8 critically-damped (×1.7 theory)
KD_THRUST_PER_M_PER_S = 8000.0
VZ_LPF_ALPHA          = 0.25      # PnP jitter killer
RAMP_S                = 6.0
TICK_MS               = 33        # 30 Hz

# ExtPos warmup
EXTPOS_WARMUP_MAX_S   = 6.0
EKF_CONV_TOL_M        = 0.10      # |ekf − pnp| < 10 cm
EKF_CONV_TICKS_REQ    = 5

# Bringup envelope (s190 iter-26 baseline)
Z_HOLD                = 0.78      # matches where PD+warmup settles
SWEEP_RADIUS_M        = 0.025
SETTLE_S              = 2.0
VMAX_M_S              = 0.06
DUR_RELAY_S           = 30.0      # × 2 axes = 60 s autotune
DUR_HOLD_S            = 10.0
HOLD_RMS_MAX_M        = 0.030

# Land
LAND_DUR              = 2.5
```

### Acceptance thresholds (OP-S10-W21 spec)

| Metric                              | Threshold        |
|-------------------------------------|------------------|
| `R_cam_to_body` drift from SDF GT   | < 1.0°           |
| `cam_offset_B` vs SDF GT (∞-norm)   | < 5 mm           |
| `kp_x`, `kp_y`                      | ∈ [0.30, 0.50]   |
| Hold rms drift                      | < 30 mm / 10 s   |
| Hold max drift                      | < 80 mm          |

SDF ground truth (mirrored in `sentai_calib.cc`):
- `R_SIM = [[0,1,0],[1,0,0],[0,0,-1]]`
- `CAM_OFFSET_SIM = (-0.04, 0, -0.02)`

### Anti-cheat

✅ Mission consumes only `sentai.markers.detect_from_camera()` (camera
frames via bridge) + `sentai.crazy.pose()` (cf2 EKF telemetry over
CRTP LOG).

✅ Orchestrator does the same (`sentai_markers_get_count`,
`sentai_markers_get_latest`, `sentai_crazy_pose`).

✅ No GT injection.  ExtPos values come from PnP (`get_drone_pose_tuple`
inside mission, or coplanar PnP inside orchestrator).  Plugin baro
publishing disabled.  Plugin TOF cheat reverted.

`[[sentai-sim-air-gapped-from-truth]]`.

### Estimated wall-clock budget

| Phase | Time |
|---|---|
| 1 setup | ~1.5 s |
| 2 zero-unlock | 1.5 s |
| 3 thrust ramp | 1–6 s (depends on first-PnP altitude) |
| 4 PD lock | 0.5–4 s |
| 5 ExtPos warmup | 1–6 s |
| 6 handoff | 0.2 s |
| 7 run_bringup SAMPLE | 4 poses × (1+2+0.5) s = 14 s |
| 7 run_bringup KABSCH | < 1 s |
| 7 run_bringup AUTOTUNE_X | up to 30 s |
| 7 run_bringup AUTOTUNE_Y | up to 30 s |
| 7 run_bringup HOLD | 10 s |
| 7 run_bringup SAVE | < 1 s |
| 10 land | 3 s |
| **Total** | **~100 s** (timeout budgets PHASE_TIMEOUT_S=180 s) |

### Risks identified (operator review)

1. **cf2 packet drop** (s190 22-May root cause): IMU @1000Hz + baro
   disabled + ExtPos @30Hz + RPYT @30Hz = 1060 pkt/s in queue.  cf2
   socketlink queue 2000-deep, drain ~600/s.  Possible ExtPos drops
   → Kalman never sees PnP → never converges.
   **Mitigation if hit**: drop RPYT/ExtPos rate during phase 5 to
   15 Hz; or rebuild plugin with IMU @500 Hz (HR violation —
   document).
2. **Camera bridge frozen frames** (observed iter1 launch — 5 unique
   CRC across 340 frames after first 5 s).  If hit again, this kills
   detector recall entirely.
   **Mitigation**: kill orphan gz processes pre-run; check RTF; if
   still slow, restart distrobox.  This is the [[crazysim-debug]]
   pre-flight.
3. **Orchestrator SAMPLE → SAFETY abort** (s190 iter-22..23): the
   sweep moves drone ±2.5 cm, which is small but markers can still
   exit FOV momentarily.  SafetyTask trips on n_dets<4 × 30 frames.
   **Mitigation observed in s190**: the orchestrator now sends its
   own ExtPos during SAMPLE, so cf2 doesn't drift.  Should work post
   T10 detector improvement (recall 0.998 vs previously 0).
4. **Pose ambiguity** still possible if drone yaws ~ 0° AND only the
   6 symmetric markers are visible (N marker out of FOV).  At 0.78 m
   altitude with FOV ~58° horizontal, x-range visible is ±0.43 m, so
   N at (0.02, 0.10) should always be in FOV unless drone drifts X >
   0.40 m.  Should be safe within sweep radius 0.025 m.

### Pass criteria — this iter

1. All 7 mission phases reached (`phase_reached == 10`)
2. `summary['accepted'] == True`
3. All 5 acceptance thresholds met
4. `assert_calibrated() == True`
5. `/system/calib.ini` exists post-run
6. Land error < 25 cm (soft — no XY nav this iter)

### What I am NOT validating in this iter

- Persistence round-trip after reboot (separate iter)
- Re-running with persisted INI loaded (separate iter)
- Multiple z altitudes for sample collection (T8 territory)
- Camera intrinsics auto-cal (T5 — DEFERRED)

---

## Files in this iter

- `mission_s193_iter2.py` (top-level, will be staged as
  `mission_s193.py` by run.sh — symlink at run time, or copy)
- `mission_s193_journal.txt` (post-run)
- `mission_s193_summary.json` (post-run)
- `fr_current/gt.jsonl` (host-side GT recorder)
- `sentai_repl.log` (post-run)
- `verdict_s193.json` (post-run)

## Run command (after operator validates this doc)

```bash
bash examples/sentai_runtime/experiments/s193_calib_full_postT10/run.sh iter2
```

(`run.sh` will need a one-line patch to copy `mission_s193_iter2.py`
instead of `mission_s193.py` — or we rename iter2 mission to
`mission_s193.py` and put the iter1 version at
`mission_s193_iter1_HL_takeoff_FAILED.py` for history.)
