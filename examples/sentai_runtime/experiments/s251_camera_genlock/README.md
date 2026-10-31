# EXP-s251 — dual-camera alternation: frame identity, phase offset and software genlock

**WBS:** OP-S10-W23-T9 (approved 2026-10-31).  **Board:** rev 2026-10 (board id 58eb),
builds 1650–1664 RAM-loaded.  **Follows:** s243 (−44 % at 1:1), s245 (no HW
trigger), s246 (29 % tag/content mismatch at full rate, four tag hypotheses
refuted).  **Status: 🟢 SHIPPED (build 1664) — full-rate 1:1 alternation at
44.7 fps with 0 mixed / 0 mislabelled frames; phase kept by an open-loop
pre-trim at init plus an on-the-fly closed loop (no calibration with flips);
the CSI wedge seen along the way (audit F16) was root-caused to the MUX flip
racing the Frame-End packet and fixed by deferring the flip 40 µs on GPT1.**
Sections: measurement result (iter1–7), C implementation (1653–1664), 15-min
soak, F16 root cause, closed loop.

## Result in one table

| config | fps | mixed | wrong tag (task / ISR) | note |
|---|--:|--:|--:|---|
| 1:1, drain=0, dirty=0, **as found** (φ = +411 µs) | 44.8 | 0–1 / 120 | 15 / 15 | iter1–iter3e; deterministic: the 1st frame after every 4th flip (cam1→cam0 direction) |
| 1:1, drain=0, dirty=1 (**production today**) | 22.4 | 0–1 | 1 / 1 | half rate |
| 1:1, drain=0, dirty=0, φ ≈ +1.2 ms … −8.1 ms (> VBLANK) | 35.5 | 2–15 | **0 / 0** | the CSI waits a whole frame on one flip direction; clean but −20 % |
| 1:1, drain=0, dirty=0, **after genlock, φ = 4 µs** | **44.7** | **0 / 120** | **0 / 0** | iter6 phase D; holds afterwards (re-measured φ = 3 µs with patterns off) |

## Method

### Oracle v2 (replaces the 5-row BARS/HBAND oracle of s246)

Both sensors run the OV5640 **rolling colour bars** (`0x503D = 0xC0`); cam1
additionally has the SDE "negative" effect (`0x5580 |= 0x40`, `0x5001 |= 0x80`).
`sentai.camera.peek_col(40)` (new, build 1650) returns the whole 480-row B
column of ONE dequeued buffer plus its two tags, the buffer index and its
FB-done timestamp.  On column 40 (the first bar):

- identity: plateau 255 → cam0, plateau 0 → cam1 (polarity, not geometry —
  a mixed frame shows both plateaus);
- the inverted **stripe moves 8 rows per sensor frame** → an in-picture frame
  counter (60-frame period) that exposes stale / duplicated buffers;
- `mixed` = inverted rows not one contiguous 8-row band.

Pattern catalogue captured on cam0 (files `pat_*.jpg`): 0x82 square,
0x81 random, 0xC0 rolling bars, 0x88 bars + horizontal fade, 0x8C bars +
vertical fade (ramp 0→255 over rows 4..72 on col 40), 0x83 black.  0x84 does
not produce a pattern.  `pat_cam0_roll.jpg` / `pat_cam1_rollneg.jpg` are the
two oracle images.

### FB-done log (build 1651)

`coralmicro_csi_on_frame_complete()` (the SDK-patch hook that the NXP driver
calls when it queues a completed buffer; one store there was shown harmless
in s246 iter7) now records `(GPT1 µs, buffer idx, MUX owner, seq)` into a
64-entry ring and a per-buffer timestamp; `sentai.camera.frame_log(64)`
reads it.  Caveat found in iter4/5: the hook runs only when the driver
queues a frame, so **with no consumer the 3-deep queue fills and the log
freezes** — any phase probe must drain frames while it measures.

### Phase measurement

At `ratio(1,1)` the MUX flips every two frames.  The FB-done interval of the
first frame after a flip to cam1 is `T + φ`, after a flip to cam0 `T − φ`
(φ = cam1 frame start minus cam0 frame start).  When the CSI misses the new
sensor's start it waits a whole frame, so the excess is reduced modulo T and
mapped to (−T/2, T/2].  Steady interval T = 22 283 µs (VGA45: VTS 984 lines ×
22.64 µs; VBLANK = 984 − 960 = 24 lines ≈ 543 µs).

## Findings

1. **No vertical roll, no seam.**  Every buffer is one complete sensor frame
   (iter1 with the vertical-fade oracle: ramp position constant, mixed 0/120).
   The s246 "roll" hypothesis is dead; the CSI aligns every buffer to a frame
   start.
2. **The two sensors are frequency-locked and nearly in phase.**  Both hang on
   one 24 MHz oscillator (sheet 9).  Measured φ(cam1 − cam0) = **+411 µs** as
   found after init (identical across B, B1 and four probes), i.e. 1.8 % of a
   frame.  φ does not drift (same value to the microsecond over minutes); it
   is set once by the sensors' start order at `camera.init()` and therefore
   **differs per boot** (it also changed whenever this experiment shifted it).
3. **Why the full-rate alternation mislabels 1 frame in 8.**  A flip executed
   at FB-done of the old sensor is clean only if the new sensor's frame start
   falls inside the old one's vertical blanking.  Margins: flip→cam1:
   `VBLANK + φ − δ`; flip→cam0: `VBLANK − φ − δ` (δ = DMA-done → flip latency).
   With φ = 411 µs the cam0 margin is 132 µs; it is lost on every 4th flip,
   giving the deterministic 15/120 pattern and its single direction.  At
   φ > VBLANK (iter3e/iter4/iter5 phase D) the CSI simply waits a frame on
   that direction: 0 mislabelled, 35.5 fps.  **Optimum is φ ≈ 0** (margins
   ≈ VBLANK both ways) — the operator's "symmetric spacing" intuition.
4. **Genlock by a one-frame VTS tweak works and is exact.**  Writing
   `TIMING_VTS + d` (0x380E/0x380F) to one sensor, waiting ~1.5 frames and
   restoring shifts that sensor later by `d` lines (iter5: 5 × 156 lines =
   17.66 ms ≡ measured −4.62 ms mod T, iter6: 200 then 40 lines → φ from
   −8150 µs to +909 µs to **+4 µs**).  Holding `VTS + 1` for N frames also
   shifted (iter4), but the probe there was blind (log freeze).  Delaying
   cam0 lowers φ, delaying cam1 raises it.  AEC frame-length extension must
   stay off (`0x3A00 bit 2`; it already is) or the phase would wander.
5. **Stale frames after idle.**  The first grab after ≥ 1 s without a consumer
   returns a buffer that old (`age_us` 0.45–0.97 s, phase E; the earlier
   "cam1 JPEG that showed cam0" was this): with nobody dequeuing, three
   buffers sit full in the queue and the CSI keeps overwriting the fourth;
   `sentai_cam_get_raw_with_recovery()` then "drains and keeps the last" of
   the three stale ones.  Per-buffer FB-done timestamps (build 1651) make an
   age check possible; proposal below.
6. **The tags themselves were right all along** given the frame they tagged:
   task tag and ISR tag agree in every run; the mislabelled frames were the
   ones captured across a flip that landed outside the blanking window.
7. Incidents: iter3 ended in an M7 **LOCKUP** reset (`SRSR 0x18000006`, no
   breadcrumb → not a handled fault) at the start of phase B; not reproduced
   in six further runs — recorded, open.  MicroPython quirk: values above
   2^30 (GPT1 µs after ~18 min uptime, `& 0xFFFFFFFF` results) are boxed
   long-long ints which raise `TypeError` when mixed with floats; the driver
   is integer-only.

## iter7 — synchronous start over I²C (operator question)

Both sensors into software standby (`0x3008 = 0x42`), wait 60 ms (0 frames
arrive), release them back-to-back from MicroPython, measure φ while
dequeuing:

| release order | 1st write → 2nd write | φ(cam1 − cam0) after |
|---|--:|--:|
| cam0 first | 414 µs | +447 µs |
| cam1 first | 402 µs | −383 µs |
| cam0 first | 400 µs | +432 µs |

Each `sentai.camera.reg_write` costs ~400–420 µs through the CameraTask /
LPI2C path, and **φ equals the release gap, with the sign of the order** —
which is also why the as-found value after every `camera.init()` is
≈ +411 µs (cam0 is started first).  Consequences: the phase is deterministic
per firmware (not random), a C-side release with both LPI2C transfers
issued within a few microseconds would land |φ| in the tens of µs by
construction, and the remaining error is removable with one VTS tweak.
Sensors resumed at 45 fps after each restart (17–18 frames / 400 ms).

Hazard: the first attempt of this test faulted the board — see
`agent/board_rev_2026_10_init_audit.md` F15 (NXP CSI driver keeps its
empty-buffer list inside the frame buffers; a DMA write into a still-linked
buffer makes the ISR dereference pixel data).  The record was rescued by
the new crash-text slot (`BUS_FAULT PC=0xEA8A CSI_TransferGetEmptyBuffer,
BFAR=0x00604F90`).  Draining the queue before the standby avoided it in the
second attempt; the defect itself stays open (SDK patch proposal).

## Implementation rule (operator, 2026-10-31)

`diag/_t_s251_genlock.py` is a **measurement driver**, not the mechanism:
per the project rule (CLAUDE.md "compute-intensive work lives in C/C++;
MicroPython is for logic, control and glue") everything below that touches
the hardware is C in the firmware — `sentai_cam_peek_col()`, the CSI
frame-complete hook with its ring and per-buffer timestamps, `sentai_cam_
frame_log()`, the SCCB register access.  The production genlock (proposal
A), the stale-frame guard (C), the CSI driver patch (audit F15) and the
phase monitor (D) will be implemented in C; MicroPython only triggers and
reads them (`sentai.camera.genlock()`, `camera.stats()["phase_us"]`).  The
s251 driver stays as the independent test of that implementation.

## Proposal (next, needs operator go)

A. `sentai.camera.genlock()` — C implementation at `camera.init()`:
   release both sensors from standby back-to-back in C (|φ| → tens of µs
   by construction, iter7), then iter6's loop as verification/trim: at
   `camera.init()`: alternate 1:1 for ~70 frames while dequeuing, estimate φ
   from the FB-done log, apply one VTS tweak (cap 200 lines per step), verify
   (|φ| < 60 µs), two steps max (~3 s).  Expose φ in `camera.stats()` and
   log `SERR_CAM_GENLOCK` with the residual.  Alternative/complement:
   start both sensors from software standby (0x3008) with back-to-back
   writes on LPI2C1/LPI2C6 so φ ≈ 0 by construction, then verify.
B. With A in place: `dirty_skip_n(0)`, `switch_drain(0)` become the default
   for `ratio(1,1)` → 44.7 fps dual-camera instead of 22.4 (s243 −44 % → 0 %).
C. Stale-frame guard in `sentai_cam_get_raw_with_recovery()`: drop queued
   buffers whose FB-done timestamp is older than 2 frame periods (use the
   new `g_cam_buf_ts_us[]`), so the first frame after a pause is fresh.
D. Phase monitor: keep the FB-done log and alarm (SERR) if |φ| grows past
   VBLANK/2 — a cheap continuous health check for the dual-camera path.

## Implementation in C (builds 1653–1657, proposals A–D + F15) — SHIPPED

| build | change | result |
|---|---|---|
| 1653 | `sentai_cam_genlock()` in `sentai_runtime.cc`: phase from the FB-done log while alternating 1:1 and dequeuing (`cam_consume_frames`), one-frame `TIMING_VTS+d` trim, ≤ 3 steps, tol 60 µs; runs at the end of `camera.init()`; stale-frame guard (C); watchdog phase monitor (D); `sentai.camera.genlock()/phase()`, stats fields | iter8: init genlock converged (1 µs) but only after **3 steps that oscillated ±430 µs**; `genlock(1)` failed (−40) with the same oscillation |
| 1654 | **frame-synchronised trim** (`cam_vts_tweak_one_frame`): put the sensor to be trimmed on the MUX, wait for its FB-done edge, write `VTS+d` 5 ms later (latches at the next edge), restore 1.25 T later; `g_cam_genlock_busy` mutes the monitor during the genlock's own alternation | iter10: 395 → **10 µs in 1 step**; `genlock(1)` 421 → −8 µs in 1 step; 1:1 alternation 135/136 frames in 3.0 s = **44 fps**; stale guard fired once (first grab after idle) |
| 1657 | SDK patch `0005-fsl-csi-empty-list-side-array-f15.patch` (audit F15): CSI empty-buffer list moved out of pixel memory, bounds-checked, `coralmicro_csi_on_queue_fault` counter → `SERR_CAM_CSI_QUEUE_FAULT`; `stats()["csi_queue_faults"]` | 15-minute soak below: phase 5–6 µs throughout, 0 faults |
| 1664 | **Deferred MUX flip** (root-cause fix of the F16 wedge): the CSI ISR arms GPT1 output-compare 2 at +40 µs and `sentai_cam_deferred_flip_isr()` flips there; `flip_delay_us()` knob, `flips_deferred/executed` stats; GPT1_IRQn at the CSI priority | iter20: **3/3 cold boots clean at 44 fps**, `flips_deferred == flips_executed`; closed-loop autotune test and s249 regression 15/15 (below).  (iter19 logs named "deferred" actually ran on 1662 — the 1663 build failed on a duplicate symbol — and show the same results with the 20 µs hold.) |
| 1662 | hypothesis test: patch 0005 re-applied + 20 µs busy-wait before the ISR flip (experiment only) | iter18: **3/3 clean with the hold, 2/2 wedged with the hold set to 0** (same build, `poke32`) |
| 1661 | A/B: patch 0005 reverted | iter17: 3/3 clean (vs 8/9 wedged with the patch) → the patch exposed a timing race, see below |
| 1660 | **open-loop pre-trim at init** (`pretrim()`, `genlock_auto` mode 1 default) + **`cam_tune` closed loop** (blind frame-synchronised trim on two consecutive drift samples, dirty-skip re-armed while drifting) | iter15: pre-trim 18 lines → φ −5/−10 µs ✓, but the first alternation wedged on both boots (→ A/B above) |
| 1659 | **init-time genlock made opt-in** (`sentai.camera.genlock_auto(1)`, default off) and the measurement shortened to 48 frames / 12 flips, after the second CSI wedge (iter12, below) | iter13: wedged again during the init measurement (3rd event) → isolation test below |
| 1658 | embeded.md §14 review fixes: VTS restore retried ×3 and reported (−51) instead of leaving a sensor slower; sync-start path releases both sensors on any failure; `phase()` guarded against the running pipeline and restores the caller's ratio/camera; trim write at 0.25 T after the edge with the restore anchored to the pre-write tick (`vTaskDelayUntil`); measurement requires ≥ 32 fresh log entries; stale skips no longer consume the CSI-recovery budget; monitor skips kicks where the log did not advance; `SERR_CAM_PHASE_DRIFT_END` (0x0A10); genlock only in streaming mode; `phase()` returns the error code instead of None | iter12 (regression) |

### 15-minute soak (build 1657, `soak_phase.py --minutes 15`)

Operator question: would a reviewer ask for a long run to show the phase does
not drift, and could the firmware re-align itself?  Cold boot, `camera.init(1,
45)` (genlock 413 → 6 µs in one step), then `ratio(1,1)` with a consumer
(`peek_col`) for 30 windows of 30 s; `stats()` sampled per window.

| | value |
|---|--:|
| windows | 30 × 30 s = 15.0 min |
| frames consumed / sensor frames | 40 286 / 40 317 (**99.92 %**) |
| fps per window | 44 in every window |
| `phase_us` per window (watchdog monitor) | **5…6 µs in every window**; independent `phase()` at the end: 6 µs |
| `phase_drift` / `stall_count` / `csi_queue_faults` | 0 / 0 / 0 |
| `stale_skips` | 30 = exactly one per window (the host's REPL round-trip between windows idles the consumer; the guard drops the aged buffer — intended) |

The phase is pinned, not merely stable: both sensors count the same 24 MHz
clock, so without a register write or a sensor restart it cannot move.  The
monitor therefore stays **report-only** (`SERR_CAM_PHASE_DRIFT` /
`_END`); an automatic re-alignment is cheap to add (the same one-frame trim
from a non-watchdog task when |φ| > VBLANK/2 for two consecutive kicks, rate-
limited) but there is nothing for it to correct in normal operation — it is
recorded as a follow-up, to be added only if a field log ever shows a drift
event.  Files: `results_soak15.csv`, `run_soak15.log`, `soak15_console.txt`.

**Why iter8 oscillated (finding 8).**  OV5640 timing registers latch at the
frame start.  An unsynchronised write/restore pair holds the long VTS for
one OR two frames depending on where in the frame the write landed, so the
shift was `d` or `2d` — with `d = φ` the 2d case flips the sign of φ and the
loop bounces between +430 and −430 µs.  This also explains the iter6 history
(200 lines moved the phase by 2 × 200 lines, 40 lines by 1 × 40).  The fix
is timing, not gain: with the write 5 ms after the trimmed sensor's own
FB-done edge, the long frame spans [0.75 T, 1.75 T + d] after the write and
the restore at 1.25 T lands mid frame with ± 0.5 T of margin → exactly `d`.

**Cold-boot statistics (build 1654, iter11, 4 boots + iter10):** init-time
genlock converged in **1 step on 5/5** boots (residuals 8, −1, −10, −9, 10
µs); 1:1 alternation 134–135 consumed / 135–136 sensor frames per 3 s on
every boot (44 fps).  As-found phase after `camera.init()` was +395…+432 µs
on every boot (= one SCCB write, cam0 started first — deterministic).

**Incident — CSI input wedge during the genlock measurement (iter9 build
1654, iter12 build 1658, iter13 build 1659) = audit F16.**  The camera
stopped delivering frames while the genlock alternated 1:1 at the as-found
phase (+400 µs, cam0 margin ~130 µs): genlock returned −30, then
`GRAB_TIMEOUT`, `SWITCH_FALLBACK`, `DRAIN_TIMEOUT`, `SERR_CAM_STALL`.
Forensics on the iter12 unit (`run_iter12.log`, probes in the audit doc):
CSI `CR3.FRMCNT` frozen at 182, CSI2RX `IRQ_STATUS 0x13D` (CRC, 2-bit ECC,
SoT, SoT-sync, ERRCONTROL), both sensors streaming and answering on I2C;
sensor standby toggles, a MUX switch and `camera.stop()+init()` did NOT
recover it — only a SoC reset.  `drain`/`dirty` are consumer-side only, so
production alternation uses the very same ISR flip; the hazard is the flip
at a bad phase, which is what the genlock removes — hence the W23-T10
proposal to measure the phase without flipping (OV5640 STROBE pads).  The
iter3 LOCKUP had the same shape.

Files: `genlock_check.py` (one boot: init → stats → phase → 3 s alternation →
`genlock(1)` → phase), `run_iter8.log` … `run_iter11_boot4.log`,
`soak_phase.py` / `results_soak15.csv` / `run_soak15.log` (15-minute soak).

### Root cause of the CSI wedge (F16) and fix — iter17–iter19

| build | configuration | clean / boots |
|---|---|--:|
| 1658–1660 | patch 0005 applied, flip in the CSI ISR | 1 / 9 |
| 1661 | patch 0005 reverted, flip in the CSI ISR | 3 / 3 |
| 1662 | patch 0005 + 20 µs hold before the flip | 3 / 3 |
| 1662 | same build, hold set to 0 via `poke32` | 0 / 2 |
| 1664 | patch 0005 + flip deferred to GPT1 OCR2 (+40 µs) | 3 / 3 (iter20) + 3 / 3 on 1662 with the 20 µs hold (iter19) |

A MUX flip executed at DMA-done races the old sensor's **Frame-End short
packet** (sent a few µs after the last line); cutting that burst leaves the
CSI2RX/D-PHY in an error state that only a SoC reset clears.  The ISR's own
latency decided the race: patch 0005 removed one SDRAM read from the re-arm
path and made the loss near-certain; s246's "an XOR in the ISR broke
alternation" was the same mechanism.  The phase was never the deciding
factor (wedges at +400 µs and at −5 µs alike).  Fix: arm a GPT1 output
compare in the CSI ISR and flip in the timer ISR, 40 µs later — no wait in
any ISR, deterministic, far inside the blanking.

### 5-minute soak of the final build (1664, `soak_phase.py --minutes 5 --tag soak5_b1664`)

Cold boot, pre-trim 18 lines, `ratio(1,1)` with a consumer: 10 windows ×
30 s, **13 459 / 13 469 frames consumed (99.93 %)**, 44 fps in every window,
`phase_us` −13…−12 µs throughout (end check `phase()` = −13 µs), 0 drift
events, 0 stalls, 0 CSI queue faults; dmesg only `DETECT`, `PRETRIM 18` and
one task-level switch.  Files: `results_soak5_b1664.csv`, `run_soak5_b1664.log`.

### On-the-fly re-alignment (closed loop) — iter20 `autotune_check.py`, build 1664

Cold boot → `init(1,45)` pre-trims cam0 by 18 lines → 6 s of 1:1 at 44 fps
with φ = −35 µs → one unsynchronised `VTS+40` frame injected on cam1 →
φ = +1776 µs (CSI waits a frame on one direction: 25 fps) → the watchdog
monitor flags `PHASE_DRIFT` and re-arms the dirty skip → `cam_tune` applies
one blind trim of 78 lines to cam1 (`SERR_CAM_PHASE_TRIM`, no MUX flip, no
frame-queue access) → φ = +10 µs, `PHASE_DRIFT_END`, 44 fps again; total
recovery ≈ 15 s (two monitor samples at the 5 s kick + the trim).  dmesg:
`E:0A12:18 E:0A0E:1776 E:0A11:78 E:0A10:10`.  No initial calibration with
flips is needed any more: pre-trim (open loop, deterministic offset) +
monitor + blind trim keep the sensors within ±35 µs of each other.

Files: `alt_only_check.py` (init → alternation → optional genlock, `--settle`,
`--no-genlock`, `--pre`), `autotune_check.py` (pre-trim → alternation →
injected 40-line phase error → closed-loop re-alignment), `run_iter14*` …
`run_iter19*`.

## Files

- `../../diag/_t_s251_genlock.py` — on-board driver (phases A, B, B1, C, D, E).
- `run_s251.py` — host runner (push via HTTP or the REPL uploader, exec, keep
  `CSV|` rows).  Note: `/lib/diag` must exist and `sentai.usb.ip(1)` must be
  on for the HTTP upload.
- `results_iterN.csv`, `run_iterN.log` — rows `F` (per frame), `S` (summary),
  `L` (FB-done log), `P` (phase), `C` (genlock step), `E` (idle grabs).
- `pattern_*.jpg`, `pat_*.jpg`, `day_cam*.jpg` — pattern catalogue and scene
  checks.
