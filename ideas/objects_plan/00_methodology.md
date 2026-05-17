<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line ranges: 61-118 (§1+§2) and 623-854 (§4-§10). -->

# Chapter 00_methodology — Methodology: NASA/JPL principles, budgets, risks, gates, dep graph, validation, kickoff

WBS anchors: spans OP discipline; RISK-{NN} live here (§4); cross-cutting for all OP-S{N}.

## 1. Principii NASA/JPL aplicate per stagiu

Toate stagiile trebuie să livreze următoarele înainte de a trece la următorul (per `embeded.md` §A-F):

### Fault model documentat
Listă explicită de faulturi credibile per modul (hardware, comunicare, timing, memorie corupție, deadline-uri ratate), cu acțiune deterministă pentru fiecare (drop / retry / degraded / safe / restart).

### Resource budget
| Buget | Limită hard |
|---|---|
| Timp per tick (M7) | < 20 ms @ 50 Hz |
| Heap dinamic post-boot | **ZERO** — static-only |
| Stack per task | declarat + `uxTaskGetStackHighWaterMark` expus |
| **ITCM (`.text`)** | Default `.sdram_text` pentru tot codul nou; ITCM se câștigă cu dovezi de hot path |
| SDRAM | OK 24 MB; budgetează la inițializare |

### Supraveghere
- Heartbeat counter / iters counter expus în stats
- Liveness check la `_start()` cu timeout 500 ms (vezi `sentai_anchor_forward.cc` pattern)
- Toate tranzițiile FSM loggate (dmesg cu severity)

### Observabilitate
- Stats dict per modul (`sentai.<modul>.stats()`)
- dmesg-uri categorisate cu severity
- Pose log CSV la 50 ms (per concept §12.4)

### Recovery
- Per `embeded.md §F`: local retry → local recovery → subsystem restart → degraded → safe → reset.
- NU full reboot ca prim răspuns. NU watchdog blind kick.

### SIM ↔ ARM parity
Fiecare stagiu nou trebuie să livreze:
1. **Header contract** `examples/sentai_runtime/sentai_<name>_shim.h` (dacă diferă comportamentul per platformă)
2. **ARM impl** (real / stub) în `.cc`
3. **SIM impl** în `sim/*.c`
4. **MicroPython binding identic** pe ambele (dict shape identic, key names identice)
5. **Test T1 wire** (struct.pack smoke) → **T2 synth-frame** → **T3 live Gazebo**

---

## 2. Memory & module budget (planning anchor)

ITCM disponibil: ~32 KB liber (post P2.5 relocation, build #1298). Tot codul nou:
**default `.sdram_text` + `.sdram_bss`** — vezi `project_itcm_budget.md`.

Octați estimați per modul nou (cifre conservative; actualizate post-stage):

| Modul | .sdram_text estimat | .sdram_bss estimat | Note |
|---|---:|---:|---|
| `sentai_objects.cc` (map L4) | ~6 KB | 2 KB (32 sloturi × 64 B) | static `object_t map[32]` |
| `sentai_object_ekf.cc` (inverse-depth) | ~8 KB | ~1 KB (state buffers) | CMSIS-DSP matrix ops |
| `sentai_mission_sm.cc` | ~4 KB | <0.5 KB | 3 SM-uri ierarhice |
| `sentai_action_layer.cc` (PBVS/IBVS) | ~5 KB | ~0.5 KB | velocity gen + APF obstacles |
| `sentai_object_lifter.cc` (2D→3D) | ~3 KB | ~0.5 KB | bearing + class prior |
| Total nou | ~26 KB SDRAM | ~5 KB SDRAM | margine confortabilă |

---

## 4. Risk register

| Risk | Severity | Mitigation |
|---|---|---|
| ITCM overflow when adding 4 new modules | HIGH | Default `.sdram_text` for everything; audit map after each stage |
| Object lifter numerical issues (inverse-depth divergence) | HIGH | Stage 5 has explicit F5 (negative ρ reset) + F6 (filter divergence drop); validate against ground truth |
| Mission SM dead-locks (anti-pattern per `embeded.md`) | MEDIUM | Per-state timeouts mandatory; tested with mock track events |
| Action layer velocity jump at PBVS↔IBVS transition | MEDIUM | Blend zone 1.3-1.7 m + velocity slew limit (Stage 4) |
| TPU model accuracy worse than promised in synth data | MEDIUM | Stage 2 includes real-Gazebo valset (not just train set) |
| Loop closure false yaw correction destabilizes flight | HIGH | Stage 6 F1 rejects > 45° corrections; LC events go through fault gates of anchor_forward |
| cf2 SITL diverges from real cf2 firmware behavior | LOW | Validate in parallel with PX4 SITL — different EKF, same intent semantics |
| Heap allocation creeps in via library helpers | MEDIUM | Lint with `grep malloc\|new` in new code; static-only enforced in code review |
| Watchdog kicks during heavy detection | LOW | Existing watchdog discipline preserved; new tasks call `sentai_diag_repl_kick` if > 60s |

---

## 5. NASA/JPL-aligned per-stage gate (mandatory before next stage)

For every stage transition, this checklist must be green:

```
□ Fault model documented in file header (F1..Fn explicit)
□ Resource budget declared (timing + stack + memory)
□ Per-tick liveness counter exposed in stats
□ `_start()` has 500 ms liveness wait (when applicable)
□ All MP binding strings & dict shapes parity ARM ↔ SIM
□ T1 wire test PASS (struct.pack smoke)
□ T2 synth-frame / mock test PASS
□ T3 live Gazebo PASS (when applicable)
□ Unit test for each fault gate (test_fault_gates.py pattern)
□ Stack high-water-mark < 70% of allocated
□ `.sdram_text` placement verified via objdump
□ dmesg category + severity used for state transitions
□ No printf with string literal in standalone .o files (uses dmesg)
□ memory/.md entry added with non-obvious lessons
□ experiment.md updated with delta + measurements
```

---

## 6. Stage dependency graph

```
       ┌─────────────────────────────────────────┐
       │ Stage 0 — gap analysis (this doc)       │
       └────────────────┬────────────────────────┘
                        │
       ┌────────────────▼──────────┐
       │ Stage 1 — sentai.objects  │  (data layer)
       └────┬──────┬───────────────┘
            │      │
   ┌────────▼┐  ┌──▼──────────────────┐
   │Stage 2  │  │Stage 3 — mission SM  │  (cf2 SITL ok, no real detector yet)
   │TPU model│  │SEARCH/APPROACH/FINAL│
   └────┬────┘  └──────┬──────────────┘
        │              │
        └─────┬────────┘
              │
   ┌──────────▼─────────┐
   │Stage 4 — action    │  (intent → velocity)
   │  layer PBVS+IBVS   │
   └──────────┬─────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 5 — object lifter      │
   │  (2D track → 3D inverse-d.) │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 6 — loop closure yaw   │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 7 — full SM (COAST/    │
   │  ALIGN/INITIALIZE/STALE)    │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 8 — end-to-end SITL    │
   │  (cf2 + PX4 parallel)       │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 9 — ARM bring-up +     │
   │  timing budget validation    │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 10 — paper / docs /    │
   │  memory hardening           │
   └─────────────────────────────┘
```

Total: ~3-4 săptămâni dezvoltare focusată (estimat conservativ).

---

## 7. What we explicitly DON'T build

(Per concept §13 + project priorities — keep scope honest):

- **Custom on-M7 ego-motion EKF replacing PX4 EKF2 / cf2 KF** — we feed observations TO existing flight-controller EKFs via `sentai.link.send_vpe` / `sentai.crazy.send_ext_position`. Building our own EKF on M7 is months of work + duplicates proven code. Defer to "if we ever go barebones cf2-firmware-replacement".
- **OctoMap / Voxblox / ESDF** — too heavy for MCU per concept §8.6. Use primitive obstacle list.
- **CubeSLAM / QuadricSLAM cuboid landmarks** — point + class lookup is the MCU-appropriate choice (concept §8.5).
- **Behavior Trees** — FSM remains clearer for < 15 states (concept §9 reference Colledanchise & Ögren).
- **Magnetometer** — explicitly out (concept hardware spec). Loop closure substitutes.
- **Hardware safety state machine (`sentai.safety`)** — battery thresholds, link-loss aborts, IMU faults, geofence breaches, watchdog handlers. Tracked in a **separate plan**, NOT in this objects_plan. The mission FSM (`sentai.explore`, Stage 3) communicates with `sentai.safety` via a thin handshake (safety can force-hold or force-land the mission) but never owns the safety logic itself. Don't push battery / link / IMU thresholds into `sentai.explore` guards — they belong in `sentai.safety`.

---

## 7.5 Canonical exploration pattern — SFLVP (S From Last Visited Place)

**Specified 2026-05-13.**  When `sentai.explore` walks the world model
during the EXPLORE state (§3 stage 3), the canonical traversal is:

  1. Mark the **current cell** as visited (`places.observe(cell, …)`).
  2. Enumerate the **6 hex neighbors** of the current cell (`places.neighbors(cell, 1)` minus the cell itself).
  3. **Visit each neighbor** in turn — fly to its centroid, settle briefly,
     observe (class histogram + HSV embedding once Stage 11.D fires).
  4. After all 6 neighbors are visited, **pick the next central** —
     the neighbor whose own k=1 ring contains the most still-unvisited
     cells (greedy frontier).
  5. Repeat until the explore cell budget (Stage 3.B `cell_budget`)
     is hit or `set_thresholds(..., explore_timeout_ticks)` fires.

The traversal traces an "S" shape from each last-visited place as the
drone leaves a fully-covered group and jumps to the next central — hence
**SFLVP** (S From Last Visited Place).

**Why this shape (vs spiral / lawn-mower / Voronoi-frontier):**

  - **Hex neighbors are pre-computed by H3** — `gridDisk(k=1)` returns
    the 6 cells deterministically.  No path-planning needed inside the
    inner loop.
  - **Coverage is uniform** at each scale — each cell is observed AT
    LEAST once and at most twice (when it's a neighbor of two
    consecutive centrals).
  - **Compatible with the dual-scale convention (§10v)** — at PX4
    natural scale the hex edges are ~10 m, at cf2 1/10 they're ~1 m;
    the same SFLVP algorithm runs unchanged, only `places.init(scale=…)`
    differs.
  - **Backtracking is free** — the "S" jump from the corner of one
    ring to the start of the next is a single fly-through (no zig-zag).

**Implementation home:** lives in `sentai.explore` (Stage 3.C) as a
helper consumed by the EXPLORE state's per-tick action.  The mission FSM
calls `_explore_next_waypoint()` which returns the next H3 cell centroid
to fly to — that's what `sentai.servo.move(...)` then commands.

s125_integrated_demo currently flies a hard-coded 4-corner square (Stage
3.A-era).  Stage 3.C swaps this for SFLVP — same world model, same FSM,
new waypoint generator.

---

## 8. Where new modules live (architecture map)

```
examples/sentai_runtime/
├── sentai_objects.{h,cc}              ← Stage 1 (NEW)
├── sentai_object_lifter.{h,cc}        ← Stage 5 (NEW)
├── sentai_mission_sm.{h,cc}           ← Stage 3 (NEW)
├── sentai_action_layer.{h,cc}         ← Stage 4 (NEW)
├── sentai_tracker.{h,cc}              ← reused (Stage 7 extends)
├── sentai_anchor_forward.cc           ← reused (Stage 6 extends)
├── sentai_aruco_shim.h                ← reused (Stage 6 adds is_loop_closure)
├── sentai_link.{h,cc}                 ← reused (Stage 4 adds send_velocity)
├── sentai_crazy.{h,cc}                ← reused (Stage 4 adds send_velocity)
├── modsentai_objects.c                ← Stage 1 binding (NEW)
├── modsentai_mission.c                ← Stage 3 binding (NEW)
├── modsentai_servo.c                  ← Stage 4 binding (NEW)
├── models/red_cube_v1.tflite          ← Stage 2 artifact
├── experiments/
│   ├── s114_red_cube_model/           ← Stage 2
│   ├── s115_endtoend_cube_landing/    ← Stage 8
│   └── s116_arm_timing_budget/        ← Stage 9
└── paper/
    └── objects_nav.md                 ← Stage 10
```

SIM side mirrors:
```
sim/
├── (sentai_objects.cc shared — pure data, builds for both targets)
├── (sentai_object_lifter.cc shared — math, builds for both)
├── (sentai_mission_sm.cc shared)
├── (sentai_action_layer.cc shared)
├── sim_velocity_sink.c                ← Stage 4 SIM stub for send_velocity (logs to file)
└── modsentai_sim.c                    ← Stage 1/3/4 bindings (parity)
```

---

## 9. Validation infrastructure (cross-stage)

### CI gates
- ARM build: `cmake --build build --target sentai_runtime` → exit 0
- SIM build: `cmake --build build-sim --target sentai_sim` → exit 0
- For each new module: `test_<module>_wire.py` (struct.pack smoke) runs in < 10 s
- Pipeline smoke: existing `aruco_bench` still PASS (no perf regression)
- Bench fps: `sentai.diag.tpu_bench()` ≥ 42 fps (yolo_1 baseline)
- ITCM check: `.text` < 235 KB (leaves 15 KB margin for surprise additions)

### Continuous artifacts per run
- `pose.csv` (concept §12.4) — 50 ms cadence
- `mission.log` — every transition + intent emission
- `lifter.log` — per-tracklet EKF state trace
- `health.csv` — stack hwm + fault counters per task per second

### Re-runnable scenarios
Each experiment under `experiments/sNNN_*` has:
- `run.sh` deterministic launcher (no manual steps)
- `expected.json` PASS thresholds
- `analyze.py` extracts metrics + decides PASS/FAIL
- `README.md` documents reproducibility (seed, distrobox commands, env)

---

## 10. First three actions to kick off

When ready to start coding (after operator review of this plan):

1. **Stage 1.A** — Write `sentai_objects.h` + skeleton `sentai_objects.cc` with the static array + add/get/list. **No EKF, no lifter, no integration.** PR review for API ergonomics + naming consistency with sentai_tracker.
2. **Stage 1.B** — `modsentai_objects.c` MP binding + QSTR regen + `test_objects_wire.py` smoke. PASS = sequential add/get/remove + eviction logic + NaN rejection.
3. **Stage 1.C** — `experiments/s114_objects_map_smoke/` — a 30-second on-board script that fills + drains the map at 10 Hz, watching stack hwm + fault counters. Then commit + ship to memory as "data layer done".

This is the minimal first slice that proves the pattern and unblocks Stages 3-5 in parallel.

---

---

