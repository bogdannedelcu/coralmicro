---
name: objectsplan-l5-handoff
description: "Session handoff (2026-05-14) — L0..L4 shipped on integration/from-180bbb5f. L5 = sentai.object_lifter (inverse-depth EKF: 2D tracklet bearing + class-prior pseudo-depth → 3D world-frame landmark). Resume entry point."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**State at handoff** (2026-05-14, branch `integration/from-180bbb5f`):

- L0 baseline:  `a4454163` FlowBaseline canonical no-wind.
- L1 shipped:   `2717bb27` docs + H3 submodule + s119 smoke.
- L2 shipped:   `eaf67e75` sentai.objects data layer.
- L3 shipped:   `19c40d88` sentai.places (H3 gallery + HSV descriptor + L1 match).
- L4 shipped:   `f8420bec` sentai.servo (action layer skeleton + trace ring).
- s128 L4.1Baseline shipped: `90d2feb2` — closed-loop EKF, 4 markers.
- s129 L4.2Baseline shipped: `f39c46a4` — IBVS via PnP-tvec.
- s130 Stage 4.5Baseline shipped: image-only world-model nav, see
  [[s130-45baseline-shipped]].
- Sim.md:       `294df935` Gazebo-GUI rule reaffirmed (see [[gazebo-gui-required]]).
- L5 **not started** — but Stage 4.5 closure means we now have a clean
  baseline for "consume sentai.objects from a host script with
  image-only localization".

**Operator-deferred:**
- s128 — ArUco -> world model + 1s hover above each marker via REPL
  high-level instructions.  Validates L2 + cf2 control before L5 ships.
  Now also a candidate exercise of L4 `sentai.servo` (call .move() /
  .hover() / .land() as the closed-loop driver in REPL).

**L5 scope (per ideas/objects_plan.md Stage 5):**

`sentai_object_lifter.{h,cc}` — convert each mature 2D tracklet into a
3D landmark estimate using bearing + class-prior pseudo-depth.  Initial
inverse-depth parameterisation (Civera/Davison/Montiel TRO 2008) that
converges with parallax over a few seconds of lateral motion.

Existing reuse: `sentai_tracker.cc` (mature tracklets with TENTATIVE/
CONFIRMED state + age), `sentai.flow.anchor_pose()` (drone pose when
available), `sentai.imu.*` (IMU attitude), CMSIS-DSP `arm_mat_*`,
`sentai_objects.cc` from L2 (destination after convergence).

Files to add (per plan §Stage 5):
- `examples/sentai_runtime/sentai_object_lifter.h`
- `examples/sentai_runtime/sentai_object_lifter.cc` (runs in detection_task
  after tracker.update()).  ARM + SIM via the L2/L3 ifdef pattern.
- (No new MP binding initially — observable via `sentai.objects.list()`.)
- Camera model constants in header (LIFTER_FX_PX, FY, CX, CY) + class-
  prior real-size table (red cube 0.30 m, cardboard 0.40 m, ...).

Per-tracklet algorithm: bearing r_C from bbox center via K^-1, rotate
to world (R_W_B · R_B_C), pseudo-depth d = f·real_size/pixel_w, init
inverse-depth (ρ, σ_ρ), update with parallax obs, convert to Cartesian
when σ_ρ < 0.5/d².  Test (per plan §11 step 5): drone hover 1.5 m +
lateral ±2 m at 1 m/s × 5 s → ||p_obj_est − p_obj_GT|| < 0.3 m, trace(P)
< 0.5.

**L5 deps**: L2 (sentai.objects — destination), sentai_tracker (mature
tracklets).  Does NOT need L3 places or L4 servo.  Camera + flow
anchor_pose are existing pre-L1 infrastructure.

**Resume recipe for L5:**
1. Read ideas/objects_plan.md Stage 5 (lines 361-420 — algorithm + fault model).
2. Mirror L2/L3 file layout: `sentai_object_lifter.{h,cc}` (no MP binding initially).
3. `.sdram_text` + `noinline` on all ARM entry points (per
   [[itcm-budget]] / [[servo-l4-shipped]] lesson — ITCM is TIGHT).
4. `.sdram_bss` on per-tracklet EKF state arrays.
5. Fresh driver test `diag/_t_04_lifter.py`.  Test recipe per
   Sim.md §10w (no leading underscore on the SIM copy).
6. Audit against agent/embeded.md before commit.
7. FlowBaseline gate after commit (per [[gate-every-layer-no-exceptions]]).
8. Atomic commit subject: `ObjectsPlan L5: sentai_object_lifter
   (inverse-depth EKF: bearing + class-prior → 3D landmark)`.

**Open question for L5 start:**
Operator may prefer to do s128 (ArUco-tour smoke) FIRST as a real-world
sanity check on L2 + L3 + L4 servo + cf2 control before adding more
firmware code.  L5 is heavy on numerical-tuning + frame conventions
(plan estimates 5-7 days), so a working closed-loop smoke between L4
and L5 has real diagnostic value.

Related: [[objectsplan]], [[servo-l4-shipped]], [[places-l3-shipped]],
[[objects-l2-shipped]], [[itcm-budget]], [[no-broken-branch-test-reuse]],
[[gate-every-layer-no-exceptions]], [[no-safety-logic-in-explore]],
[[gazebo-gui-required]], [[sim-repl-test-recipe]].
