# EXP-s251 — dual-camera alternation: frame identity, phase offset and software genlock

**WBS:** OP-S10-W23-T9 (approved 2026-10-31).  **Board:** rev 2026-10 (board id 58eb),
builds 1650–1651 RAM-loaded.  **Follows:** s243 (−44 % at 1:1), s245 (no HW
trigger), s246 (29 % tag/content mismatch at full rate, four tag hypotheses
refuted).  **Status: 🟢 RESULT — full-rate 1:1 alternation with 0 mixed and 0
mislabelled frames after a software genlock (iter6).**

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

## Files

- `../../diag/_t_s251_genlock.py` — on-board driver (phases A, B, B1, C, D, E).
- `run_s251.py` — host runner (push via HTTP or the REPL uploader, exec, keep
  `CSV|` rows).  Note: `/lib/diag` must exist and `sentai.usb.ip(1)` must be
  on for the HTTP upload.
- `results_iterN.csv`, `run_iterN.log` — rows `F` (per frame), `S` (summary),
  `L` (FB-done log), `P` (phase), `C` (genlock step), `E` (idle grabs).
- `pattern_*.jpg`, `pat_*.jpg`, `day_cam*.jpg` — pattern catalogue and scene
  checks.
