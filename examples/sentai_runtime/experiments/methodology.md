# Methodology — on-board measurement protocol

How the CSV data in this folder is produced.  Written to be pasted
whole into the paper's methodology section; numbers are taken from
current practice on SentAI build #640+ firmware.

---

## 1. Philosophy

Three hard rules govern every experiment in this corpus:

1. **Self-contained sessions.**  Each measurement run lives in its own
   folder under `/diags/sNNN_<name>/` on the device's LittleFS.  A
   folder contains every artefact needed to interpret it offline: raw
   CSVs, per-experiment text descriptors, a session manifest, a
   summary, and scene snapshots from both cameras before and after the
   run.  No external context is required; no measurement refers to
   data outside its own folder.

2. **Ground truth is what is on the device.**  The host-side copies
   under this `experiments/` tree are byte-for-byte mirrors of
   `/diags/` fetched over HTTP GET.  The device, not the host, is
   authoritative: a run is considered complete only once it has
   closed its session and its `manifest.csv` lists every experiment.

3. **Bounded everything.**  Every loop has an explicit maximum
   iteration count, every wait has a timeout, every blocking call has
   a documented upper bound.  No "wait forever" is acceptable in the
   measurement path.  The same principle applies to the firmware that
   produces the data (see [../agent/embeded.md](../agent/embeded.md)
   §B).

---

## 2. Session model

A *session* is the unit of experiment grouping.  Created by
`diag.begin("name")`, closed by `diag.end()`.  On `begin`:

- A persistent counter in `/diags/.counter` is incremented to produce
  the next session id `sNNN`.  Counters are atomic and monotonic; two
  back-to-back sessions with the same name produce distinct folders.
- The session folder `/diags/sNNN_<name>/` is created.
- `manifest.csv` is initialised with a header row.
- Session start time (via `sentai.rtos.uptime`) and heap snapshot
  (`sentai.rtos.heap_info()`) are captured for the eventual
  `summary.txt`.

Within a session, every experiment writes:

- `NNN_<experiment>.csv` — raw per-sample data (one row per iteration)
- `NNN_<experiment>.txt` — human-readable description: what it
   measures, column semantics, parameter block (`cam_id`, resolution,
   `model_path`, `switch_drain`, `ratio`, etc.)
- A row in `manifest.csv` with `seq`, `experiment`, `file`, and a
   short `summary` string (e.g. `"total=62.7ms fps=15.94"`).

On `diag.end()`:

- `summary.txt` is finalised with experiment count, wall duration,
  and RTOS-free heap delta.
- The counter is committed.  Session folder is now immutable from the
  on-device code's point of view — only host-side deletion can
  modify it.

A session is essentially a write-once, durable, timestamped block.
That is the invariant the rest of the methodology leans on.

---

## 3. Per-experiment measurement protocol

### 3.1 Warm-up

Every experiment begins with a warm-up block that exercises every
stage on the measurement path at least once.  For a four-stage
camera-to-detection experiment (E16, E18):

```python
for c in (cam_a, cam_b):
    sentai.camera.select(c)
    sentai.camera.to_tensor()
    sentai.tpu.invoke()
```

This forces:
- CSI frame queue to fill (steady state, not "just after init")
- Both OV5640s to be powered and in streaming mode
- PXP + quant paths to allocate their scratch
- TPU model to execute once (primes EdgeTPU package cache)

After warm-up, the first *measurement* sample is also dropped (first
element of the stats set is skipped in all appendix calculations).
Rationale: the first sample straddles warm-up and measurement in
subtle ways (switch count parity, DMA queue occupancy, arena reset),
and a single extra sample out of 40 is cheap compared to the noise it
removes.

### 3.2 Repetition count

Standard values:
- **x20** for sustained-throughput studies (E14/E15 parallel pipeline):
  enough for `σ/√n ≈ 0.5 ms` confidence on a 65 ms mean.
- **x40** for camera-switch studies (E16/E18): the alternating loop
  produces 20 samples per direction, enough to establish the
  directional split to sub-millisecond precision.
- **x16** for visual JPEG captures (E17): bounded by MP heap capacity
  (JPEG bytes buffered in Python list before LFS write — see §5).

Every repetition is a full independent iteration.  No sample
accumulates state from the previous one beyond the global firmware
state that would exist in steady-state production (TPU model loaded,
pipeline running, camera streaming).

### 3.3 Stage-timing instrumentation

Per-stage times are captured via `sentai.rtos.ticks_ms()` snapshots
around each call:

```python
t0 = _ticks(); sentai.camera.select(cam);      sel_ms  = _ticks() - t0
t0 = _ticks(); sentai.camera.to_tensor();      tens_ms = _ticks() - t0
inv_ms = sentai.tpu.invoke()                   # firmware returns ms
t0 = _ticks(); dets = sentai.tpu.detect(...);  det_ms  = _ticks() - t0
```

`ticks_ms()` is a 32-bit millisecond counter from FreeRTOS; deltas
are computed as unsigned subtraction so the counter wrap at 49.7 days
cannot corrupt a result.  Resolution is 1 ms, which is fine at the
60–200 ms per-iteration granularities we measure.

`invoke_ms` is returned by the firmware itself (not measured host-
side) because it is the one stage where the Python-dispatch overhead
would inflate the number.

### 3.4 CSV schema

Every per-iteration CSV uses the shape `run_index, <stage_ms>…, <counts>`:

```
run_index, cam_id, select_ms, to_tensor_ms, invoke_ms, detect_ms,
num_detections, total_frame_ms
```

One row per iteration, no aggregation.  Statistics (mean, σ, min/max,
percentiles) are computed offline by the appendix generator
[`_build_appendix.py`](./_build_appendix.py) from the raw rows, not
summarised on-device.  This keeps the device code simple and lets a
reviewer re-derive every number without trusting the board.

Column semantics are documented in the matching `NNN_<experiment>.txt`
file.  That descriptor is hand-written per experiment (not generated)
so a future reader can understand what `total_frame_ms` means even if
the experiment function has been refactored away.

---

## 4. Statistics we report

For every sample set `x = {x₁, …, xₙ}` (with the warm-up sample
dropped):

- **mean** — arithmetic; ℝ.
- **stdev σ** — Bessel-corrected sample standard deviation
  (`statistics.stdev` in Python; undefined for n=1, reported as 0).
- **min / max** — raw extremes.
- **median** — 50th percentile; reported where the distribution is
  bimodal (e.g. E16 alternating where cam0 and cam1 cluster apart).
- **FPS** is always derived as `1000 / mean_total_ms`, not as a
  separate measurement — the two are mathematically equivalent but
  reporting only one avoids drift between derived statistics.

For bimodal data (alternating cam0 ↔ cam1) we also report per-camera
means separately, with the directional asymmetry `cam1 − cam0`
computed as a difference of means (not a paired statistic, because
the two sets are independent iterations on different sensors).

No hypothesis tests; the effect sizes are either >> noise (flip-on-
EOF saves ~66 ms from a 211 ms mean) or < noise (Fix A null result,
asymmetry reducing from +64 to +0.1 ms).  Confidence intervals on
individual means are σ/√n ≈ 0.3–0.5 ms, always tighter than the
reported differences.

---

## 5. Scene control and visual evidence

### 5.1 Before/after snapshots

Every session that runs a camera experiment calls
`diag.snapshot_both_cameras("before")` at the start and
`diag.snapshot_both_cameras("after")` at the end.  That helper
([`diag/_session.py`](../diag/_session.py)):

1. Checks the session is active and `sentai.pipeline.running()` is
   false (mid-pipeline `camera.select` would race with PrepTask).
2. For each camera id ∈ {0, 1}:
   - `sentai.camera.select(cid)` — flip MUX
   - throwaway `to_tensor()` — drain stale frames from the previous
     sensor so the saved JPEG is definitely from `cid`
   - `snapshot_scene(when, name="scene_cam<cid>")` — writes JPEG via
     `sentai.camera.to_tensor(path, quality)` at quality 75

This produces the four files `scene_cam{0,1}_{before,after}_WxH.jpg`.
Purpose:
- Documents the exact scene each sensor saw across the run.
- Lets an offline reviewer diff before/after to catch a scene
  disturbance (LED flicker, object movement) that would otherwise
  invalidate the run.
- Makes every session reproducible up to the scene: a future rerun
  should produce similar before-snapshots if the physical setup has
  not changed.

### 5.2 E17 in-RAM JPEGs

For experiments whose *subject* is visual quality (E17 per-switch
JPEGs), we keep every frame's JPEG in MicroPython heap during the
timing loop, then write the whole list to LFS in a cold loop after
the measurement is complete.  See
[`diag/e_pipeline.py:e17_switch_drain_visual`](../diag/e_pipeline.py).
The hot-loop timing therefore reflects only `select + jpeg encode`,
not `+ LFS write`.  Memory budget: ~25 KB/JPEG × 16 reps ≈ 400 KB,
safely inside the 512 KB MicroPython heap.

### 5.3 Verbose flag discipline

Measurement loops always set `sentai.verbose(0)` before the hot loop
and restore it in a `finally`.  The flag gates per-frame firmware
printfs (e.g. `[cam_switch] -> camN via EOF ISR`); at 200+ iterations
those prints saturate CDC-ACM TX and stall the host read path, which
propagates as apparent jitter in the measurement.  Silencing them is
the single most impactful discipline for clean timing data.

---

## 6. Firmware environment control

Before every measurement we set an explicit, known firmware state:

```python
sentai.verbose(0)                 # quiet per-frame prints
sentai.camera.ratio(0, 0)         # disable auto-alternate scheduler
sentai.camera.switch_drain(2)     # conservative default
if sentai.pipeline.running():
    sentai.pipeline.stop()        # release camera MUX ownership
gc.collect()                      # reduce heap pressure before hot loop
```

Rationale:
- `ratio(0,0)` — the auto-alternate scheduler would flip the MUX
  stateless-ly during a measurement; disabling it makes every switch
  a deliberate action.
- `switch_drain(2)` — we document departures from this default
  explicitly (E17 tests threshold 1).
- `pipeline.stop()` — E14/E15 parallel pipeline owns the camera MUX
  and would race with manual `camera.select` during a sequential
  experiment.
- `gc.collect()` — prevents a GC pause mid-loop from biasing a
  particular iteration.  MicroPython's incremental GC would otherwise
  fire unpredictably.

---

## 7. File transfer protocol

### 7.1 Downloading results (host ← board)

Results are fetched over HTTP GET via the `sentai_httpd` /api/ls and
/api/raw endpoints.  The driver is [/tmp/_fetch_diags.py](./_build_appendix.py)
(the actual fetcher was a tmp-one-shot during this corpus's capture;
the authoritative script is the appendix generator which reads
already-downloaded data).

Retry protocol for the GET response code `{"error":"lfs_busy"}`
(returned when the LFS-task queue is processing a different request):
back off 700 ms and retry, up to 5 attempts.  See
[`../paper/lfs.md`](../paper/lfs.md) §4.2 for why root-dir `/api/ls/`
always goes through the slow path and produces this response on the
first attempt.

### 7.2 Uploading diag/ code (host → board)

HTTP POST `/api/write/…` is known to hang on this firmware (see
[`../agent/embeded.md`](../agent/embeded.md): *"We do not upload
files on the board via HTTP, it just doesn't work well.  We use USB
or REPL fs.write in chunks."*).  The canonical uploader is
[`../diag/_host_upload_repl.py`](../diag/_host_upload_repl.py), which
opens `/dev/ttyACM0` and pushes 48-byte chunks via
`sentai.fs.write(path, b'…')` REPL calls.  48 bytes is chosen so that
the escaped `bytes` literal line fits comfortably in the REPL's
256-byte line buffer even when the source file contains UTF-8
non-ASCII characters.

Verification at write-end: the uploader polls `print('LEN:%d' %
len(_d))` after every 5 chunks and on the final flush to catch a
dropped line; a mismatch aborts the upload loudly instead of
producing a truncated on-device file.

### 7.3 USB mass storage (fallback)

If REPL upload fails (e.g. REPL is in the stdout-dead state — a
recurring symptom when a previous experiment left stdout captured),
fall back to MSC mode:

```python
sentai.usb.drive(1)       # warm reset to storage mode; /dev/sda appears
# sudo mount -t lfs /dev/sda /mnt/coral ...
# cp files in/out
```

The warm-reset cycle preserves the LittleFS user partition byte-
identical.  See [`../paper/usb.md`](../paper/usb.md) for details.

### 7.4 Archival host-side copy

This `experiments/` directory is the immutable snapshot of every
`/diags/sNNN_*` on the device at the time of download
(2026-04-20).  Sessions generated after that date exist only on the
device until a fresh `curl /api/ls/diags` + recursive download is
run.  The snapshot was taken twice during the camera-speed work (E15-
E18 subset on 2026-04-20 morning; the pre-E15 groundwork added on
2026-04-20 evening), so this tree is contemporaneous with the
build #640 post-refactor firmware.

---

## 8. Reproducibility

A result in this corpus is reproducible iff all of the following hold
on the rerun:

1. **Same firmware build** — recorded in `summary.txt` as a derived
   number.  Major bumps (Fix A, Fix B, A1-A7 review) are called out
   in the paper's cross-session comparison.  If the rerun uses a
   different build, label its session with a distinct name
   (`..._postfix`, `..._postrefactor`) and do not overwrite the
   earlier session's interpretation.
2. **Same model** — the `.tflite` path is in every experiment's
   descriptor `.txt`.  Model swaps (yolo26n → 1-class) are the
   subject of whole-new sessions (`e14_*` vs `e15_*`), never silent
   parameter changes inside a session.
3. **Same scene** — the before-snapshots are the authoritative
   record.  Comparing `s043/scene_cam0_before.jpg` with
   `s045/scene_cam0_before.jpg` documents whether the lilac
   arrangement, the blue mug, and the lighting are the same between
   the two runs.  Scene drift is treated as a confound: a
   measurement difference attributable to scene change is not a
   firmware fact.
4. **Same warm-up state** — the hard rule (§3.1) is the first
   measurement sample is always dropped.

Numbers that differ by more than `3σ` between two same-firmware
same-scene reruns are investigated before publishing (see
[`paper/cam_switch.md`](../paper/cam_switch.md) §"Head-to-tail
benchmark" where `s043`/`s044`/`s045` agree within ≤ 1 ms on every
stage — that agreement is the reproducibility claim).

---

## 9. Noise budget

Empirical noise floor on this hardware, per the appendix-C tables:

| Quantity | Typical σ | Dominant source |
|---|---:|---|
| `frame_interval_ms` at E15 parallel, steady state | 1.4–1.8 ms | FreeRTOS tick quantisation + PXP jitter |
| `total_frame_ms` at E16/E18 sequential, per direction | 1–3 ms | same |
| `select_ms` at E16 post-Fix B | 0.6 ms | one-tick rounding of EOF-wait |
| `invoke_ms` | 2–3 ms | EdgeTPU firmware boundary (package cache hits vs misses) |
| JPEG byte size | 2–5 KB | scene content variance, not timing |

Claims of "sub-millisecond change" are not defensible in this corpus;
claims of "≥ 10 ms change between two same-firmware reruns" are
robust against any of the above.

---

## 10. What this methodology does NOT cover

- **Power / thermal** — no measurements of MCU die temperature,
  current draw, or thermal throttling.  The RT1176 does not thermal-
  throttle within the 3-second E18 sweep window, but long-duration
  stress runs would require adding those instruments.
- **Sensor-side characterisation** — we measure pipeline cost from
  the CSI receiver outward.  OV5640-internal AEC/AGC convergence
  behaviour is treated as an opaque sensor-level effect and shows up
  only in the "Known limitations" section of the camera-switch paper.
- **Statistical inference tests** — the effect sizes are large
  relative to noise, and a formal hypothesis test would be overkill.
  When it isn't (e.g. "does Fix A change timing?"), the null result
  is called out honestly.
- **Cross-device reproducibility** — every measurement is from one
  physical SentAI board.  A different board of the same revision
  should reproduce within noise; measurements on a future hardware
  revision are outside the scope of this corpus and would require a
  fresh baseline.
