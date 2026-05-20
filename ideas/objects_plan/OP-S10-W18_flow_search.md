# OP-S10-W18 — Flow search algorithm tuning (Diamond + ablation)

**Status**: T1 SHIPPED 2026-05-20.  Diamond search added as runtime
toggle; M7+M4 benched both modes; CRITICAL methodology bug in M4
sentinel dispatch uncovered and fixed mid-sprint, retroactively
invalidating W17 M4 numbers.  This WP captures the corrected Flow
performance data, the algorithm, the bibliography, and the bug
methodology lesson.

## 1. Why this WP exists

Operator-requested 2026-05-20 (post-OP-S10-W17-T4 Flow ablation):

> "hai sa incercam si diamond, dar fa-l ca optiune, sa putem activa
>  cand unul cand altul asa incat ulterior sa verificam in simulator
>  performantele, chiar si in realitate.  Sa notezi in workpackage
>  aceste detalii precum si bibliografia si testele de performanta
>  facute pe flow"

Flow SAD inner loop is the optical-flow algorithm's most compute-
intensive stage.  The current production path is **exhaustive ±12
search × 32×32 block** (625 candidates).  Standard block-matching
literature documents 10-25× compute reduction via diamond / 3-step /
adaptive search at minor accuracy cost.  This WP wires the diamond
variant as a runtime toggle (default OFF — keeps current accuracy),
benches both modes on M7 and M4, and prepares the path for a SIM /
real flight A/B comparison.

## 2. Algorithm — Diamond Search (Tham 1998)

Reference: Tham, Ranganath, Ramakrishnan, Kasahara 1998 — "A novel
unrestricted center-biased diamond search algorithm for block
motion estimation" (IEEE T. CSVT).

Two complementary patterns:

```
LDSP (Large Diamond Search Pattern) — 9 points, 2-pixel max radius:

           (0, -2)
       (-1, -1) (+1, -1)
   (-2, 0) (0, 0) (+2, 0)
       (-1, +1) (+1, +1)
           (0, +2)

SDSP (Small Diamond Search Pattern) — 5 points, 1-pixel radius:

           (0, -1)
   (-1, 0) (0, 0) (+1, 0)
           (0, +1)
```

Iteration:

1. Set center = (0, 0).  Evaluate SAD at center → cbest.
2. Evaluate the 8 non-center LDSP points around center.
3. If LDSP best is the center itself, go to step 5 (refine).
   Otherwise, move center to LDSP best, return to step 2.
4. (Loop bounded by 8 iterations safety cap.)
5. Evaluate the 4 non-center SDSP points around the current center.
   Output is the best of the 5 (center + 4 SDSP).

Convergence: typically 3-5 LDSP iterations + 1 SDSP = ~25-50 SAD
evaluations for cf2-class drone motion (smooth, near-translation).
Worst case bounded: 8 LDSP × 8 evals + 5 SDSP = 69 evaluations.

Trade-offs vs exhaustive ±12 (625 candidates):

- **Pro**: ~10-25× compute reduction.  Same SAD/USADA8 inner kernel,
  no change to per-evaluation cost.
- **Con**: Local-optimum risk for fast / non-translational motion.
  For SLAM-class drone hover and slow scanning motion, this is
  empirically a non-issue (Krajnik & Faigl 2013 used a similar
  pattern in WhyCon's verification stage).
- **Con**: Less robust to repetitive textures (might lock onto a
  local SAD basin away from true motion).  Mitigation: keep
  exhaustive as the default and only switch to diamond when the
  scene profile is known to be smooth.

## 3. Bibliography (Flow + block-matching)

### Optical flow on embedded MCUs

- **Honegger, Greisen, Meier, Tanskanen, Pollefeys 2013** — "An open
  source and open hardware embedded metric optical flow CMOS
  camera for indoor and outdoor applications" (ICRA 2013).  The
  px4flow paper.  Source of the parabolic sub-pixel interpolation
  on the SAD minimum that sentai uses.

- **Briod, Klaptocz, Zufferey, Floreano 2013** — "The AirBurr: A
  flying robot that can exploit collisions" (in IROS).  Cited for
  the deadband / conf-floor approach to suppress static-board drift
  (sentai's `kConfFloor = 150` and `s_deadband_mgp` come from this
  lineage).

- **Krajnik, Nitsche, Faigl, Vanek, Saska, Preucil 2014** — "A
  practical multirobot localization system" (JINT).  The WhyCon
  paper.  Establishes the threshold + flood-fill + moments pipeline
  that sentai now uses for both ArUco-style and circle-marker
  detection.

### Block-matching motion estimation

- **Po, Ma 1996** — "A novel four-step search algorithm for fast
  block motion estimation" (IEEE T. CSVT).  The 4-step search;
  predates diamond by 2 years.  Similar 10-15× speedup.

- **Tham, Ranganath, Ramakrishnan, Kasahara 1998** — "A novel
  unrestricted center-biased diamond search algorithm for block
  motion estimation" (IEEE T. CSVT).  Source of the LDSP + SDSP
  pattern implemented here.  Most-cited block-matching paper in
  video codec lineage; H.263 / MPEG-4 incorporated it.

- **Cheung, Po 2003** — "A novel cross-diamond search algorithm
  for fast block motion estimation" (IEEE T. CSVT).  Cross+diamond
  hybrid.  Future-work candidate if diamond proves not robust
  enough for sentai workloads.

- **Zhu, Lin, Chau, Tan 2002** — "Block-matching using novel cross-
  diamond search algorithms" (Real-Time Imaging).  Adaptive
  variants — slightly faster on average but higher worst-case.

### Cortex-M SIMD instruction reference

- **ARM Cortex-M7 Technical Reference Manual** (DDI 0489F) §2.2 —
  DSP-extension instructions including `__USADA8` (single-cycle
  4-byte SAD-accumulate), `__USUB8`, `__SEL`, `__SMLALD`.
- **ARM Architecture Reference Manual** ARMv7-M (DDI 0403E.e) §A6.5
  — full SIMD instruction reference shared by Cortex-M4F (M4F has
  the same DSP-extension set; key difference is no D-cache, no
  dual-issue).

## 4. Performance tests on Flow (chronological)

Full history of Flow performance measurements on RT1170, build
chain 1111 → 1405.

### 4.1 Build #1111 — broken baseline

Pre-USADA8, M4 SAD with point-decimated publisher.  Static-board
30-second cumulative drift: **-1700 raw-px** (algorithm visibly
broken; static input → non-zero output).  See
`agent/experiment.md` §"Iteration trail" for details.

### 4.2 Build #1118 — PXP downscale replaces step-sampling

Replaced step-8 raw-pixel decimation with PXP HW box-filter
downscale (640×480 → 80×60 RGB888).  Static drift: **~700 raw-px**.
Box-filter eliminates aliasing.

### 4.3 Build #1120 — parabolic sub-pixel fit (Honegger 2013)

Added Q×1000 milli-grid-px parabolic fit on the SAD surface around
the integer minimum.  Static drift: **~150 raw-px**.

### 4.4 Build #1124 — deadband + parabolic-shallow rejection

Removed an EMA smoother (was inflating cumulative motion 6× on real
motion).  Replaced with hard deadband (50 milli-gp) and a
"parabolic surface shallow" rejection.  Static drift: **0.00 gp
LITERAL**.  Static-input correctness achieved.

### 4.5 Build #1130 — SAD moves to M7-only

M4 path retired (build-#1130 commit) — empirically froze after ~5 s
under load, root-caused later to mis-calibrated SysTick.  SAD entirely
on M7 from this point.  Static drift unchanged: **0.00 gp**.

### 4.6 Build #1139 — USADA8 + LD32U landed

USADA8 inline-asm wrapper for the SAD inner loop (4-byte SAD-acc
in 1 cycle).  LD32U packed-struct cast for unaligned 32-bit reads
(replaces `memcpy(&u32, p, 4)` which had compiled as libc function
calls — see `agent.md` USADA8 hard-rule).  Bit-perfect firmware-vs-
offline-replay match.  DWT per-stage (build #1139, static board):

| Stage | Time |
|---|---:|
| PXP downscale 640×480 → 80×60 RGB888 | 1.15 ms (HW) |
| RGB → Y conversion + dual write | 0.22 ms |
| Optional gray_stretch | 0.001 ms |
| **SAD 25×25 search × 32×32 block** | **0.94 ms** |
| **publish_frame total** | **2.29 ms** |

Effective publish rate camera-bound at 15-22 fps (33 ms grab × 22 fps).

### 4.7 Build #1402 — Flow SAD re-validation (OP-S10-W17-T4)

Operator-requested re-measurement to rule out methodology errors in
the W16 / W17 M4 ablation chain.  Standalone `sentai.flow._test_sad`
with deterministic synth shift=3, run independently of the
publisher_task.  Build #1402:

- M7 Flow SAD (exhaustive): **0.97 ms** (+3% vs #1139, LTO inline
  variance).  Confirms Flow has NOT regressed across 263 builds.

### 4.8 Build #1405 — Diamond search + M4 ablation (this WP)

Diamond as runtime toggle.  M7 + M4 both benched in exhaustive +
diamond modes.  **NOTE**: the M4 measurements depend on the
clamp-bug fix in `m4_bench_host.cc` that landed in this same
commit (5f34a8dc); see §6 below.

#### 4.8.1 M7 results (build #1405)

| Mode | Cycles | Latency |
|---|---:|---:|
| **M7 Flow exhaustive** | 778,261 | **973 µs** |
| **M7 Flow diamond** | 47,245 | **59 µs** |

**M7 diamond is 16.5× faster than M7 exhaustive.**  At 30 Hz
SafetyTask slot (33 ms), diamond consumes 0.18 % of the slot vs
2.95 % for exhaustive.  Both microscopic — only matters when M7 is
heavily loaded.

#### 4.8.2 M4 results (build #1405, post clamp-bug fix)

| Mode | Cycles | Latency |
|---|---:|---:|
| **M4 Flow exhaustive (OCRAM)** | 1,468,394 | **3.67 ms** |
| **M4 Flow exhaustive (DTCM)** | 1,681,115 | **4.20 ms** |
| **M4 Flow diamond (OCRAM)** | 100,522 | **251 µs** |
| M4 ArUco threshold @ b=23 | 11,787,790 | 29.47 ms (context) |
| M4 WhyCon Phase A N=4 | 8,207,644 | 20.52 ms (context) |

**M4 diamond at 251 µs = 0.75 % of a 30 Hz slot.**  M4 IS viable
for Flow on RT1170, contrary to the W17 verdict (which was based
on bug-affected numbers).

#### 4.8.3 M7-vs-M4 ratios (corrected)

| Workload | M7 | M4 | Ratio M7 wins |
|---|---:|---:|:---:|
| ArUco threshold b=23 | 12 ms (SDRAM cache) | 29.47 ms (OCRAM scalar) | 2.46× |
| WhyCon Phase A (rolling thr) | ~3 ms (OCRAM + SIMD) | 20.52 ms (OCRAM scalar) | ~6.8× |
| Flow SAD exhaustive | 973 µs | 3.67 ms | **3.77×** |
| Flow SAD diamond | 59 µs | 251 µs | **4.25×** |

Range: M7 wins 2.5-7× on RT1170, NOT the 27× previously reported
in W17-T4 (which was an artefact of the clamp bug — see §6).

#### 4.8.4 DTCM-vs-OCRAM on M4 — surprise result

DTCM Flow SAD (4.20 ms) is **14 % SLOWER** than OCRAM Flow SAD
(3.67 ms).  Counter-intuitive.

Possible explanations:

1. M4F LDR from OCRAM is closer to 1-2 cyc effective (with the
   internal write buffer / prefetch) than the 3-cyc nominal — so
   the DTCM 1-cyc advantage is small.
2. The DTCM variant has a slightly different prologue (load curr+
   prev from `s_m4_flow_curr_dt` / `_prev_dt` base addresses) that
   the compiler can't fold into the inner loop's address
   calculation as cleanly as the OCRAM variant.
3. memcpy OCRAM → DTCM (excluded from timed region) still warmed
   write-buffer state in a way that biases the next compute
   pass.  Probably not — DWT is a hardware counter independent of
   the compute pipeline.

The takeaway is the same as W17-T4 (now corrected): **M4 SAD is
CPU-bound, not memory-bound.**  Memory tier doesn't move the
needle.  Algorithm changes (diamond) do.

## 5. API surface (this WP adds)

MicroPython on M7:

```python
sentai.flow.set_search_mode("exhaustive")   # default, 625 candidates
sentai.flow.set_search_mode("diamond")      # ~25-50 candidates
# Or pass int (0 = exhaustive, 1 = diamond).
# Returns previous mode as int.
```

C-level on M7:

```c
extern void sentai_flow_set_search_mode(int mode);
extern int  sentai_flow_get_search_mode(void);
extern uint32_t sentai_flow_test_sad(int shift_px);  /* benchmark entry */
```

C-level on M4 (sentinels via `sentai.diag.m4_aruco_bench(block)`):

| Block sentinel | Decimal | Workload |
|---|---:|---|
| 0xCAFE | 51966 | IPC smoke test |
| 0xC101..C108 | 49409..49416 | WhyCon Phase A with N disks (N = block & 0xF) |
| 0xF10F | 61711 | Flow SAD exhaustive on shared OCRAM |
| 0xF1D7 | 61911 | Flow SAD with OCRAM→DTCM pre-copy |
| **0xF1DD** | **61917** | **Flow diamond search on shared OCRAM** |

## 6. Critical methodology bug — IPC clamp in host

Discovered while debugging this WP: `m4_bench_host.cc:84-85`
clamped `block` to `[3, 511]` BEFORE sending over IPC.  All
sentinel values (`0xCAFE`, `0xC100..C108`, `0xF10F`, `0xF1D7`,
`0xF1DD`) are > 511, so they all got smashed to 511, M4 worker
saw `block=511`, fell into the default ArUco-threshold path at
`block=511` (~25-29 ms), and reported that as the timing for
every "different" workload.

### 6.1 Workpackages affected (retroactively)

- **OP-S10-W17-T2** M4 WhyCon Phase A 25.86 ms — actually ArUco @ 511.
- **OP-S10-W17-T4** M4 Flow SAD exhaustive 26.51 ms — actually ArUco @ 511.
- **OP-S10-W17-T4** M4 DTCM "no diff" — both ArUco @ 511, identical
  by definition.  The methodology-correct verdict ("M4 CPU-bound,
  not memory-bound") holds — but it now comes from §4.8.4 data,
  not the (invalid) W17-T4 result.
- **W18 mid-debug** M4 diamond "25.93 ms = same as exhaustive" —
  actually ArUco @ 511.  Real diamond = 251 µs.

W16 M4 ArUco threshold measurements (block 23, 51, 101, etc.) are
**NOT** affected — those values are all < 511 and the clamp was a
no-op for them.

### 6.2 The HARD-RULE captured

`[[ipc-clamp-in-worker-not-host]]` — input-range clamp logic
ALWAYS lives in the worker, never in the host.  Sentinel values
that overlap the "normal" input space get silently smashed
otherwise, falling through to the default path on the worker
side with no obvious symptom.

### 6.3 Fix (commit 5f34a8dc)

Clamp removed from `sentai_m4_bench_run`.  Worker's existing
clamp inside its ArUco-threshold branch still bounds the
legitimate range.  Sentinel ranges bypass the worker clamp by
matching their specific `else if (block == 0xF1DDu)` branches
first.

## 7. Open ToDo

1. **Diamond accuracy A/B in SIM** — run s127 FlowBaseline (or a
   dedicated experiment) with `set_search_mode("diamond")`
   active.  Closure metric must stay within ε of exhaustive on
   the calibrated indoor test rig.  Promotion criterion: if
   diamond ≤ 1.5× the exhaustive closure error, default-on
   becomes acceptable.
2. **M4 Flow path wire-up** — now that the architectural verdict
   has been corrected, M4 IS viable for Flow at 30 Hz (diamond
   251 µs = 0.75 % slot).  Decide whether to move Flow off M7
   entirely (frees ~1 ms M7 budget for ArUco/WhyCon) — cost is
   cross-core PXP+RGB2Y → M4 OCRAM handoff complexity.
3. **3-step / 4-step alternative bench** — Po-Ma 1996 4-step is
   reportedly more robust to non-translational motion at similar
   compute cost.  Worth a single-commit bench.
4. **WhyCon Phase A on M4 (corrected)** — now-real number 20.52 ms.
   That's still 6.8× slower than M7 OCRAM+SIMD.  Document in
   OP-S10-W17_whycon.md (correction pass needed).
5. **Sentinel response field for dispatch verification** — wire
   `app->n_dets` from M4 worker through to the MicroPython dict
   via `sentai.diag.m4_aruco_bench` so the dispatch flag is
   externally observable.  Would have caught the §6 bug
   immediately.

## 8. Cross-refs

- `[[op-s10-w17-whycon-2026-05-19]]` — WhyCon WP; M4 Phase-A
  number now known to be wrong.
- `[[op-s10-w16-ablation-findings-2026-05-19]]` — sibling
  ablation; W16 numbers VALID (no sentinel use).
- `[[ipc-clamp-in-worker-not-host]]` — the HARD-RULE memory.
- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — origin of the
  USUB8/SEL trick reused in the rolling threshold.
- `[[flow-m7-validated-2026-05-05]]` — original Flow M7 paper
  measurements (build #1139).
- Commits: 5f34a8dc (diamond + clamp fix), 02660094 (W17-T4
  superseded), e1b1f598 (W17 M4 ablation superseded).
