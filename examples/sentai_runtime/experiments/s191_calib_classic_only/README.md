# s191 — Calibration with Classic Commander ONLY (cascaded PD)

**WBS:** `OP-S10-W21-T12`
**Created:** 2026-05-22
**Status:** ⬜ ITER-1 (PD navigation only — sweep+Kabsch in iter-2+)
**Supersedes:** s190 (which still suffered helical drift during the
bringup orchestrator phase after the RPYT→HL handoff transition was
fixed).

## Claim

The entire calibration pipeline (takeoff, hover, sweep, sample,
Kabsch, hold-validate, save) runs on **Classic Commander RPYT only**
(CRTP port 3 ch 0).  No HL handoff, no Generic Commander.  Drone is
controlled by mission-side cascaded PD on X/Y/Z/Yaw using PnP
feedback.  YAW LOCK eliminates the helical drift observed in s190
(cf2 has no mag/lighthouse, yaw integrates ~1°/s without external
correction).

## Operator-stated design 2026-05-22

> "Sa abordam cel de-al doilea commander, ce zici? Sa ramanem cu
> Clasic.  Sa incercam sa facem calibrarea asa cu clasic."
>
> "Asa cu miscari mici pentru a detecta axele, sensul, etc..."
>
> "Decolarea e ok."
>
> "MP-first, ulterior o mutam in C++ pe namespace sentai.calib"

## Architecture

```
Mission-side cascaded PD at 30 Hz on vision PnP feedback:

  Z PD:    thrust    = T_HOVER + Kp_z·(z_t − z) − Kd_z·vz
  X PD:    roll      = −Kp_x·(x_t − x) − Kd_x·vx
  Y PD:    pitch     = +Kp_y·(y_t − y) − Kd_y·vy
  Yaw PD:  yaw_rate  = −Kp_yaw·yaw_pnp        ← YAW LOCK

  → send_crtp(port=3, ch=0,
              struct.pack('<fffH', roll, pitch, yaw_rate, thrust))

  Conservative gains (start at half physics-derived):
    Kp_z = 10000, Kd_z = 8000  (validated in s190)
    Kp_xy = 3000, Kd_xy = 3000 (half of Z — start conservative)
    Kp_yaw = 30   (rad/s per rad yaw error)

  Velocity estimates: finite-diff with LPF (α=0.25) on PnP-derived xyz.
```

## Pre-conditions reused from s190

Phases 1-4 (setup, zero-unlock, thrust ramp, PD altitude lock) are
COPIED verbatim from s190.  Proven to work: iter-21 GT peak 0.91m,
operator visually confirmed "perfecta decolarea".

## Iteration plan

| iter | Adds | Pass criterion |
|------|------|----------------|
| 1    | Phase 5: PD navigate to (0,0,Z_HOLD) + hover 5s | No helical drift; |xyz| stable <5cm for 4s |
| 2    | Phase 6: cross sweep with PD targets (±r,0)(0,±r) | Drone visits 4 cross poses in order |
| 3    | Phase 7: collect N samples per pose | ≥3 samples per pose, n>=4 markers |
| 4    | Phase 8: Kabsch fit + commit_R + commit_offset | R drift < 5°, offset < 10mm |
| 5    | Phase 9: HOLD validation (10s drift < 30mm) | hold_rms < 30mm |
| 6    | Phase 10: Save calib.ini + Land via thrust ramp-down | calib.ini persisted, drone settled |

## Pass criteria iter-1 (this iter)

1. **Takeoff + PD altitude lock**: drone hovers stable z=0.78m for ≥2s
   (already verified in s190)
2. **PD navigation**: after PD altitude lock, mission engages X/Y/Yaw
   PD targeting (0, 0, Z_HOLD).  Drone reaches and HOLDS within 5cm
   for 4 consecutive seconds without helical drift.
3. **No SAFETY abort during hold** (markers stay visible)
4. **`bash sim/scripts/audit_anti_cheat.sh` PASS**

## Files

- `mission_s191.py` — MP mission with cascaded PD (this iter: Phase 1-5)
- `run.sh` — host-side orchestrator (clone of s190's)
- `journal.txt` — per-iter log (append-only)

## How to run

```bash
cd /home/bogdan/work/coralmicro
bash examples/sentai_runtime/experiments/s191_calib_classic_only/run.sh
```
