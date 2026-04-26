# SentAI Runtime — Agent Handoff Guide

Written 2026-04-20, last updated 2026-04-25 (post Cale 1+ MoverTask sprint).
Purpose: a fresh agent (or future me without memory) should be able to pick
up this project without re-discovering every trap from scratch.  Read this
top to bottom before touching the codebase.

---

## 1. What this project is

Firmware for the **SentAI board** (Coral Dev Board Micro, NXP i.MX RT1176,
Cortex-M7 @ 800 MHz + EdgeTPU).  It hosts:

- **MicroPython REPL** over USB CDC-ACM (`/dev/ttyACM0`)
- **HTTP server** over USB CDC-NCM (`http://10.0.0.1/`)
- **EdgeTPU inference** (`sentai.tpu.*`)
- **Dual OV5640 cameras** multiplexed via analogue GPIO MUX (`sentai.camera.*`)
- **Diagnostics framework** (`diag.*`, sessions under `/diags/`)
- **File transfer** over CDC-ACM (preferred) or MSC (`sentai.usb.drive(1)`)

Entry points:
- Firmware source root: `examples/sentai_runtime/`
- Build from repo root: `cmake --build build --target sentai_runtime`
- Flash: `python3 scripts/flashtool.py -e sentai_runtime`
- Working dir when running host scripts: `examples/sentai_runtime/`

Reference docs under `examples/sentai_runtime/paper/` — treat as canonical.
Especially [memcpy.md](../paper/memcpy.md), [cam_switch.md](../paper/cam_switch.md),
[lfs.md](../paper/lfs.md), [usb.md](../paper/usb.md).

---

## 2. Rules that override everything else

These are load-bearing.  Violating them has cost whole days of debug.

1. **Never brick the board** — USB CDC must come up before any risky code.
   If `lsusb` shows Google Coral ID `18d1:9307` instead of NXP, firmware is
   dead and only a physical button press fixes it.  See the "ANTI-BRICK"
   section of [embeded.md](embeded.md) (§M).

2. **NASA/JPL discipline for ISRs** — ISR bodies must be the shortest
   possible: no loops, no mutex, no task wakeup, no complex logic.  The
   camera ISR at `libs/camera/camera_support.c:CSI_IRQHandler` is the
   reference example.

   **Memory placement: ISR code AND its hot-path callees MUST live in
   ITCM** (m_text region, addresses 0x0000_0c00..0x0003_F400).  Tag
   them with `__attribute__((section(".ramfunc")))` and ensure
   `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld` has a
   `.ramfunc { *(.ramfunc .ramfunc.*) } > m_text` rule placed BEFORE
   any archive-specific section like `.camera` so the wildcard
   match wins.

   **Why this matters:** SDRAM-resident ISR code via SEMC bus adds
   ~200 ns latency per branch + competes with the very same DMA
   masters the ISR services (CSI, USB, eDMA).  Empirical bug 2026-04-26
   build #904: CSI ISR in SDRAM caused MUX-flip GPIO write to land
   mid-frame instead of in VBLANK, producing visible top-of-frame
   contamination from the previous camera (frame i6 in visual A/B
   showed green tint band on a cam0-tagged LIVING capture).  Moving
   the ISR to ITCM via `.ramfunc` (build #905+) eliminated that
   class of bug.

   **Verify the placement** with `arm-none-eabi-objdump -h
   build/examples/sentai_runtime/sentai_runtime | grep ramfunc` —
   section LMA must be in the m_text address range.  If the section
   shows up at SDRAM addresses (0x8…), the linker rule is missing
   or shadowed by a more-specific rule above it.

   **Apply this to:** every IRQ handler we OWN (`CSI_IRQHandler`,
   future SOF/EOF handlers, USB ISR overrides), and any function
   called from inside an ISR that runs on the hot path (single-IRQ
   tag write helpers, GPIO toggles like `SentaiCamMuxSetFromIsr`).
   Lower-frequency ISRs (e.g. button debouncer, audio DMA) may stay
   in SDRAM until measured to be a bottleneck.

3. **Every timing-sensitive path must be bounded** — no `portMAX_DELAY`
   waits without a separate watchdog; no unbounded polling.  Deadlines
   computed as delta (`now - ts0`), not as absolute targets, because
   `TickType_t` wraps at 49.7 days.

4. **Error codes, not strings** — the runtime uses `SERR_LOG(code, val)`
   from [sentai_error.h](../sentai_error.h).  Never add naked `printf("some
   error: ...")` in a fault path; define a code in `sentai_error.h` and add
   a row in [error_codes.csv](../error_codes.csv).  Codes are immutable
   once allocated: retire via `STATUS=DEPRECATED`, never renumber.

5. **HTTP uploads do not work on this firmware** — `/api/write` hangs.
   Push files via [diag/_host_upload_repl.py](../diag/_host_upload_repl.py)
   (chunked `sentai.fs.write` over REPL) or USB MSC
   (`sentai.usb.drive(1)`).  Do not retry HTTP uploads in a loop.

6. **Writes pass through `sentai_lfs_task`, reads use fast path only for
   RAW** — `/api/ls/` goes to `lfs_task` even on an idle FS
   (see [lfs.md](../paper/lfs.md) build #633 entry for why).

7. **Experiment run cadence — verbose-first, then quiet multi-run** — the
   very first call of a NEW or firmware-touched experiment MUST be a
   single trial with `sentai.verbose(1)`.  Only after that single trial
   finishes cleanly (summary printed, no WDOG reset, no hang) do you wrap
   multiple trials with `sentai.verbose(0)`.  Going straight to verbose=0
   + multi-trial hides early failures and wastes a whole run on a bug you
   could have seen in the first per-frame trace.

8. **`verbose(0)` SILENCES Python prints too** — not just firmware printf.
   Confirmed bug: `sentai.verbose(0)` then `print(...)` produces no output
   on REPL.  Always set `sentai.verbose(1)` before reading state via
   REPL print.  Documented in `memory/feedback_experiment_run_cadence.md`.

9. **Cross-test contamination is real and load-bearing** — a wedged TPU
   from a failed test poisons all subsequent tests until full reflash.
   "I'll just re-run with a different toggle" is wrong: each A/B test
   needs a fresh `flashtool.py -e sentai_runtime`, then upload the test
   driver, then run.  Numbers measured back-to-back without reflash
   between are unreliable (we lost a day in 2026-04-25 chasing a
   "regression" that turned out to be carry-over state).

10. **Build counter (`#xxx`) confirms what's actually flashed** — pyserial
    `sentai.version()` returns `SentAI v1.0 build NNN (date time)`.  After
    flashtool, verify build # incremented vs expected.  Mismatch = stale
    firmware, your code changes aren't live.

---

## 3. Operating tools (all host-side, on Linux)

| Tool | Runs on | What it does |
|---|---|---|
| `python3 scripts/flashtool.py -e sentai_runtime` | host | persistent flash (`--ram` for RAM-only) |
| `python3 diag/_host_upload_repl.py --file <name>` | host | REPL-chunked upload to `/lib/diag/` on board (CHUNK=48 bytes because REPL line buffer is 256 — see [memory:project_upload_diag_repl.md](/home/bogdan/.claude/projects/-home-bogdan-work-coralmicro/memory/project_upload_diag_repl.md)) |
| `python3 repl_run.py --line "..."` | host | sends a REPL line and waits for next `>>> ` — has a stale-prompt bug on long commands (minutes), prefer rolling your own driver based on `_host_upload_repl.py`'s `_send_line` pattern |
| `python3 monserial.py` | host | passive serial log to `sentai_serial.log` |
| `cat /dev/ttyACM0` | host | raw serial — needs `stty -F /dev/ttyACM0 115200 raw -echo -icanon` first |
| `curl http://10.0.0.1/api/ls/<path>` | host | list dir via HTTP GET (fast path works for all subdirs; root takes 2 round-trips because of the LS-always-slow-path fix) |
| `curl http://10.0.0.1/api/raw/<path>` | host | read file via HTTP GET (fast path, ≤256 KB per response) |

The REPL-uploader prefix rule: files starting with `_host_` (and, by
package convention, the `diag/drivers/` subdir) are HOST-ONLY and never
pushed to the board.

---

## 4. How-to: survive a stuck REPL

Symptom: `print("anything")` produces no output on `/dev/ttyACM0`, but the
prompt `>>>` still appears after each command.  Root cause is usually an
experiment that left `sentai.verbose(0)` set or captured stdout.

**Warm-reset recipe** (preserves LFS, does not reflash):

```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
s.write(b'\x03\r\n'); time.sleep(0.3); s.read(4096)
s.write(b'sentai.usb.drive(1)\r\n'); time.sleep(0.5); s.close()
"
# wait for /dev/sda to appear (storage mode)
for i in $(seq 1 15); do [ -b /dev/sda ] && break; sleep 1; done
# send 'q' to exit storage mode — triggers warm reset back to default
printf 'q\r\n' > /dev/ttyACM0
# wait for HTTP to come back
for i in $(seq 1 20); do curl -s -m 2 -o /dev/null http://10.0.0.1/ 2>/dev/null && break; sleep 1; done
```

If that doesn't work, `python3 scripts/flashtool.py -e sentai_runtime`
reflashes the firmware and reboots — slower but cleaner.

---

## 5. How-to: drive the REPL from host scripts

The SentAI REPL is **line-at-a-time**, and it enters a multi-line mode on
`:`-triggered blocks (returns `... ` prompt instead of `>>> `).  This
breaks any host driver that tail-matches `>>> `.

**Canonical `send()` function** — the full-buffer-match pattern used by
`diag/_host_upload_repl.py`:

```python
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.3,
                  rtscts=False, xonxoff=False, dsrdtr=False)
time.sleep(0.3); s.reset_input_buffer()
s.write(b'\x03\r\n'); time.sleep(0.3); s.read(4096)  # Ctrl-C then fresh prompt

def send(line, to=10):
    while s.in_waiting:            # drain stale output first
        s.read(s.in_waiting)
    s.write(line.encode() + b'\r\n')
    dl = time.time() + to
    buf = bytearray()
    while time.time() < dl:
        c = s.read(4096)
        if c: buf.extend(c)
        if b'\r\n>>> ' in bytes(buf):  # full-buffer match, not tail
            break
    return bytes(buf).decode(errors='replace')
```

Never use Python `for:` blocks through this — send line-at-a-time, or use
a list comprehension that stays on one physical line.

**`repl_run.py` is deprecated for long commands** — it has a stale-prompt
bug where a previous iteration's `>>> ` is still in the pyserial buffer
when the next command is sent.

### 5.1 Long-running script idiom (added 2026-04-25)

For diag drivers like `_t_yolo512.py` that take 5-30 s and print a lot of
output, the `\r\n>>> ` tail-match terminates EARLY on intermediate
prompts.  Use a **raw-drain loop** with a sentinel string instead:

```python
def run_driver(s, path, deadline_s=30):
    s.write(f'exec(sentai.fs.read_str("{path}"))\r\n'.encode())
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        c = s.read(8192)
        if c:
            sys.stdout.write(c.decode(errors='replace'))
            sys.stdout.flush()
            if b'=== done ===' in c:  # script's end-of-test marker
                break
```

Drivers are required to print `=== done ===` last so this works.  See
`_t_yolo512.py`, `_t_warm_ab.py` for examples.

### 5.1.1 Estimate driver duration + extend the watchdog (added 2026-04-26)

The combined watchdog task in `sentai_runtime.cc:CombinedWatchdogTask`
treats >120 s of REPL silence as **dead** and stops kicking WDOG1,
which then resets the board ~30 s later.  Long-running drivers must
plan for this BEFORE starting:

**Step 1 — compute expected duration.**  Sum the wall time of every
inner loop and add 25 % margin:

```
T_driver  =  sum(N_frames_i × period_i)  ×  1.25
```

For example, a parity sweep with 5 delay points × 30 frames at
estimated 50 ms / frame:  `5 × 30 × 0.05 × 1.25 ≈ 9.4 s` (well under
the 120 s ceiling).  But the same sweep at delay = 50 ms × 16 points
× 60 frames =  `16 × 60 × 0.10 × 1.25 ≈ 120 s` — touching the edge.

**Step 2 — choose the deadline strategy:**

| Driver wall time | Action |
|---|---|
| < 60 s  | nothing — REPL task auto-heartbeats every 5 s |
| 60-120 s | call `sentai.diag.repl_kick()` once per outer-loop iteration as defense-in-depth |
| > 120 s | **split the driver** into ≤ 5 measurement points per file, reflash between halves (cross-test contamination per §2.9 anyway) |

**Step 3 — host-side timeout.**  The raw-drain loop in §5.1 must use
`deadline_s = T_driver + 30 s` so the host doesn't tear down the
session while the firmware is still printing.

**Why `repl_kick()` instead of just bumping the global threshold?**
Bumping the dead-threshold globally would weaken the protection
against actual hangs (the whole point of the 120 s ceiling is to
detect a wedged REPL).  `repl_kick()` is a per-iteration heartbeat
that proves the script IS making forward progress — same trust model
as the auto-heartbeat in `micropython_task.c`, just explicit.

**API:** `sentai.diag.repl_kick()` — bumps `g_repl_last_activity` to
`now`.  No-op if called more often than every ~5 s; cheap regardless.

**Don't disable the watchdog.**  The board self-healing contract in
`embeded.md §M` requires WDOG1 to stay armed at all times.  No public
API exists to extend the hardware timeout — that is intentional.

### 5.1.2 Experiment output discipline — self-contained drivers (added 2026-04-26)

**Doctrine: every diag driver is SELF-CONTAINED.**  No imports from
other `diag/*.py` files.  No reliance on `__init__.py` re-exports.
The only external dependency is `sentai` (the firmware-provided
namespace).  This eliminates the desync class entirely: when you
upload a driver via `_host_upload_repl.py --file _t_foo.py`, that ONE
file is everything the driver needs.

**Why:** `diag/__init__.py` aggregates many imports.  When ANY of
those sub-modules is out of date on the board (or `__init__.py`
itself), `import diag` silently fails to populate exports — symptom
on the board is `AttributeError: 'module' object has no attribute
'begin'` while the host-side static check sees nothing wrong.  The
old session helper at `diag/_session.py` was a real shared module,
but the per-experiment value it provides (folder + counter) is ~15
lines of code.  Inline it.

**Standard inlined session helper** — copy this verbatim into every
new diag driver that writes files to LFS:

```python
def _session_dir(name):
    """Allocate /diags/sNNN_<name>/ and return the path."""
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception:
        sid = 1
    try:
        sentai.fs.write(counter, str(sid))
    except Exception:
        pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d
```

LFS is **persistent across reflashes** — a JPEG written by build
#885 will still be on the FS when build #889 boots, and `curl
/api/raw/...` will happily serve it.  An hour of confusion was lost
on visual A/B because `i1_cam0.jpg` from a buggy run got mistaken
for new ground truth.  The session pattern (auto-incremented sNNN
folder) eliminates that class of bug — each run gets its own dir.

**Mandatory rules for every diag driver that writes files to LFS:**

LFS is **persistent across reflashes** — a JPEG written by build #885
will still be on the FS when build #889 boots, and `curl
/api/raw/...` will happily serve it.  This caused an hour of
confusion during the visual A/B sessions (`i1_cam0.jpg` from a buggy
run got mistaken for new ground truth in the next run).

**THE ONE TRUE PATTERN — use `diag/_session.py`:**

```python
import sentai
from diag._session import begin, end

sess = begin("visual_ab")        # auto-allocates /diags/sNNN_visual_ab/
print("session dir:", sess.dir)

# ... run the experiment, writing all artefacts under sess.dir ...
fname = "%s/i%d_cam%d.jpg" % (sess.dir, i, cam)

end()  # flushes manifest.csv, summary.txt; closes the session
```

`begin(name)` reads `/diags/.counter`, increments it, and creates
`/diags/sNNN_<name>/` — every run gets a fresh folder.  No clobber,
no leftover, no wipe-then-rewrite race.  E13–E18 drivers in
`diag/e_pipeline.py` are reference implementations; copy from them.

**Mandatory rules for every diag driver that writes files to LFS:**

1. **Inline `_session_dir(name)`** as the first function in the
   driver.  Call it once at the top of `main()` to get a unique
   `sess_dir` path.  All artefacts go under that path.  No imports
   from sibling diag modules.
2. **Filenames embed the build_id** as additional safety against
   leftover-pollution if a manual `cp` lands in the wrong folder:
   `build_id = sentai.version().split("build")[1].split()[0]`
   then `f"{sess_dir}/b{build_id}_i{i}_cam{cam}.jpg"`.
3. **Host download path mirrors `sess_dir`.**  Pull
   `http://10.0.0.1/api/raw/diags/sNNN_<exp>/<file>` and store
   under `/tmp/<exp>_sNNN/`.  `curl /api/ls/diags` returns the
   available sessions.
4. **Header comment names the session pattern** so future maintainers
   know each run lives in its own auto-numbered folder.

Canonical example: [`diag/_t_visual_ab.py`](diag/_t_visual_ab.py) —
inlines `_session_dir`, embeds `build_id` per filename, no diag/
imports beyond `sentai`.

**If you find yourself writing `try: fs.remove(...)` in a loop at the
start of a driver, STOP.**  That's the anti-pattern.  Use sessions.

**Legacy note: `diag/__init__.py` exists** and re-exports many
helpers (E13–E18 still depend on it).  DO NOT add new dependencies
on `__init__.py` from new drivers — the on-board copy desyncs from
git silently and causes `AttributeError` cascades on the next run.
If you must touch existing E13–E18 code, run `python3 upload_diag.py`
from `sentai_runtime/` to push the WHOLE diag/ package atomically
before testing.

### 5.2 Wait for board after flash / WDOG reset

After `flashtool.py` or a wedge that triggers WDOG, wait for NXP ID:

```bash
until lsusb | grep -q "1fc9:c0a1"; do sleep 1; done
sleep 2  # extra settle for USB CDC + REPL ready
```

Don't use chained `sleep 5; sleep 5;` — the harness blocks repeated
sleeps.  Use `until` with a real condition.

---

## 6. How-to: regenerate MicroPython QSTRs

Required whenever you add, rename, or remove any `MP_QSTR_xxx` or
`MP_REGISTER_MODULE(...)` in a `modsentai_*.c`.  Otherwise `import` fails
with `ImportError: module not found`.

```bash
cd examples/sentai_runtime
rm -rf build-embed
make -f ../../third_party/micropython/ports/embed/embed.mk \
     MICROPYTHON_TOP=../../third_party/micropython \
     USER_C_MODULES=$(pwd)/modules \
     micropython-embed-package

# Force CMake to re-link the MP library (GLOB doesn't track QSTR changes)
cd ../..
rm -rf build/examples/sentai_runtime/CMakeFiles/libmicropython.dir/
rm -f  build/examples/sentai_runtime/liblibmicropython.a
cmake -S . -B build && cmake --build build --target sentai_runtime
```

Verify with: `grep -c "<new_symbol>" examples/sentai_runtime/micropython_embed/genhdr/qstrdefs.generated.h`

---

## 7. Diagnostics framework (`diag.*`)

Every measurement run goes into a **session** under `/diags/sNNN_<name>/`
on the device.  Session dirs contain per-experiment CSVs, human-readable
`.txt` descriptions, before/after scene snapshots from both cameras, and
a `manifest.csv` + `summary.txt`.

Shape of an experiment body (copy from `e13_pipeline_full` or
`e18_camera_switch_headtail` as templates):

```python
def e_XX_foo(..., save=True, resolution=(512, 512)):
    from diag._session import _session, begin, end, snapshot_both_cameras
    owned = _session is None
    if owned: begin("e_XX_foo")
    prev_verbose = sentai.verbose(0)  # silence firmware per-frame prints
    if sentai.pipeline.running(): sentai.pipeline.stop()
    try:
        w, h = resolution
        sentai.camera.set_resolution(w, h)
        if sentai.camera.frame_count() == 0: sentai.camera.init(1)
        _ensure_model_loaded(model_path)  # path-keyed, idempotent
        # ... warmup, measurement, CSV save, record ...
    finally:
        sentai.verbose(prev_verbose)
        if owned: end()
    return result
```

Experiment index:

| # | File | Purpose |
|---|---|---|
| E1-E12 | `diag/e_tpu.py`, `e_camera.py`, `e_fs.py`, `e_sensors.py`, `e_system.py` | primitives (TPU, camera, FS, sensors, system) |
| E13 | `diag/e_pipeline.py:e13_pipeline_full` | Sequential vision pipeline, per-stage timing |
| E14 | `diag/e_pipeline.py:e14_pipeline_parallel` | Firmware PrepTask+InferTask pipeline FPS |
| E15 | `diag/e_pipeline.py:e15_pipeline_parallel_512` | E14 wrapper for the 512×512 model |
| E16 | `diag/e_pipeline.py:e16_camera_switch_512` | Alternating cam0↔cam1, sequential |
| E17 | `diag/e_pipeline.py:e17_switch_drain_visual` | Per-switch JPEGs (in-RAM buffer, LFS write deferred) |
| E18 | `diag/e_pipeline.py:e18_camera_switch_headtail` | Three-sweep A/B/C benchmark (fixed cam0 / fixed cam1 / alternating) |

All E1x functions accept `resolution=(w, h)` (default `(512, 512)`).  Pass
`(640, 480)` for VGA or `(320, 240)` for QVGA to measure PXP/JPEG scaling.

---

## 8. Firmware runtime surface (MicroPython)

Partial list — see [SENTAI_API.md](../SENTAI_API.md) for the full tree.
Camera-switch-related knobs, all introduced during this sprint:

| Call | Effect |
|---|---|
| `sentai.camera.select(id)` | Arm a glitch-free flip to cam `id`; ISR consumes on next EOF (≤ 33 ms at 30 fps).  Falls back to sync path after 150 ms if ISR doesn't consume. |
| `sentai.camera.set_resolution(w, h)` + `.init(1)` | Change native sensor resolution (720p / VGA / QVGA supported).  Applies to both cameras (shared CSI-2 receiver). |
| `sentai.camera.switch_drain(n)` | Set post-switch drain threshold (1–10 frames).  Default 2.  With flip-on-EOF fix, n=1 is visually safe. |
| `sentai.camera.ratio(a, b)` | Stateless auto-alternate: over any (a+b)-frame cycle, cam0 gets `a` frames and cam1 gets `b`.  Both zero disables.  Packed 32-bit atomic update. |
| `sentai.diag.cam_stats()` | Returns dict of persistent fault counters: `{switch_ok_eof, switch_fallback, drain_timeout, grab_retry, grab_fatal}`. |
| `sentai.verbose(0\|1)` | Gate per-frame firmware prints + the `[cam_switch]` log line.  Default 1; set 0 inside measurement loops. |

See also `sentai.diag.dmesg()`, `sentai.diag.boot_log()`,
`sentai.diag.crash_log()` for post-mortem breadcrumbs.

---

## 9. Firmware ownership map (who writes what)

| State | Readers | Writers | Protocol |
|---|---|---|---|
| `g_camera_frame_seq` | any | CSI ISR only | monotonic, aligned 32-bit, atomic read |
| `g_cam_current_id` | any | CSI ISR on nominal path, `sentai_cam_switch` on fallback | 32-bit atomic, single writer at a time |
| `g_cam_pending_mux_id` | CSI ISR (consumer) | `sentai_cam_switch` (producer), ISR auto-alternate | `-1` = idle, `0/1` = armed; ISR clears to `-1` after flip |
| `g_cam_switch_seq`, `g_cam_switch_pending` | `sentai_cam_get_raw_with_recovery` | CSI ISR + `HandleSwitchCameraRequest` | ISR writes before clearing pending_mux_id → reader observes consistent pair |
| `g_cam_ratio_packed` | CSI ISR | `sentai_cam_ratio_set` | single 32-bit store = atomic pair update (hi 16 = a, lo 16 = b) |
| `g_cam_switch_*_count` | `sentai.diag.cam_stats()` | only the code that detects the event (single writer per counter) | monotonic since boot |

MUX GPIO flip has two paths:
- **Nominal**: `sentai_cam_switch()` arms → CSI ISR calls `SentaiCamMuxSetFromIsr()` via atomic `DR_SET`/`DR_CLEAR` (no mutex, ISR-safe).
- **Fallback**: `sentai_cam_switch()` times out → calls `cam->SwitchCamera()` which invokes `HandleSwitchCameraRequest()` in task context, writing via `GpioSet()` (takes `g_mutex`).

Both paths read MUX polarity from the same `libs/camera/cam_mux.h` header
— single source of truth, per embeded.md §J.

### 9.1 cam_id ↔ I²C bus ↔ MUX polarity convention (2026-04-26)

The number `cam_id ∈ {0, 1}` is one logical handle that maps 1:1 to BOTH
the sensor's I²C bus AND the analog-MUX GPIO level on this board:

| `cam_id` | I²C bus (`CameraTask::*`) | `Gpio::kCamMux` level | macro (cam_mux.h) |
|---|---|---|---|
| 0 | `i2c_handle_`  (LPI2C1) | 0 (LOW)  | `CAM_MUX_LEVEL_FOR_CAM0` |
| 1 | `i2c_handle2_` (LPI2C2) | 1 (HIGH) | `CAM_MUX_LEVEL_FOR_CAM1` |

The GPIO level numerically *equals* `cam_id`.  Two implications you can
rely on without re-checking:

1. After `WriteToCam(N, reg, val)`, calling `select(N)` will route CSI
   from the *same physical sensor* you just wrote to.  In particular
   `sentai.camera.test_pattern(N, mode)` injects a pattern on the cam
   that `select(N)` will then display.
2. The per-buffer cam_id tag exposed via `sentai.camera.grabbed_id()`
   uses the same numbering, so `grabbed_id()` agreeing with the most
   recent `select()` is correct, not a coincidence.

**Why this paragraph exists.**  The original drop of `cam_mux.h` had
the polarity inverted (`LEVEL_FRONT=1`, `LEVEL_BACK=0`).  That made
`test_pattern(0, BARS)` write the BARS pattern to physical sensor X
while `select(0)` routed CSI from sensor (1−X) — a silent routing
inversion that the visual A/B test couldn't quite pin down because the
*tag → content* mapping was internally consistent (it just happened to
disagree with the I²C side).  Caught by `diag/_t_pattern_31.py` PHASE A
on 2026-04-26 (build #909): `select(0)` showed sensor-1's pattern and
vice versa, in 10/10 frames per side — purely static, no MUX-flip
involved.  Fix shipped by flipping the two macros in `cam_mux.h`; the
header now also defines preferred names `CAM_MUX_LEVEL_FOR_CAM0` /
`_CAM1` to make the convention impossible to misread.

**Hardware-spin guidance.**  If a future board revision re-routes the
analog MUX inputs to swap which physical lens sits on which I²C bus,
the FIX IS NOT to touch `cam_mux.h`.  The schematic-level wiring is
authoritative; this header just expresses the convention "GPIO level
matches `cam_id` matches I²C bus index".  If a re-spin breaks the
convention, run `diag/_t_pattern_31.py` PHASE A first to see *which*
side ended up flipped, then patch the schematic, not the firmware.

### 9.2.1 Dirty-bit "sticky-until-cleared" rule (build #953 lesson)

For any per-slot bit that's SET in an ISR and READ by a consumer
across buffer reuse cycles (`g_cam_buf_dirty[]` is the canonical
example), the ISR must **clear** the bit on every fresh fill that
is not in the marking window.  Without that clear, the bit becomes
sticky: a stale 1 from an earlier event survives into a new fill,
the consumer skips a clean buffer, and the system delivers the
"bad" buffer the bit was meant to filter — defeating the
mechanism's entire purpose.

Pattern (single-writer per side, no lock):

```c
// In the ISR FB-done block:
g_cam_buf_id[idx] = (uint8_t)active_cam;
if (in_marking_window) {
    g_cam_buf_dirty[idx] = 1u;     // mark
} else {
    g_cam_buf_dirty[idx] = 0u;     // ★ clear stale on fresh fill
}

// In the consumer (every return site that hands a buffer back):
if (g_cam_buf_dirty[idx]) {
    g_cam_buf_dirty[idx] = 0u;     // consume the marker
    cam->ReturnRawFrame(idx);
    continue;                       // bounded retry
}
return idx;                         // clean buffer
```

**Two derived rules:**

- **Every return path that hands a buffer to the caller must check
  the dirty bit.**  In `sentai_cam_get_raw_with_recovery` we have
  three (drain-and-keep, slow-path post-switch grab, blocking-grab
  fallback).  All three need the check; missing one creates a
  fall-through path that silently delivers dirty data.
- **Adding a new return path?  Audit dirty-bit handling first.**
  This is now §2 of the load-bearing rules.

### 9.2.2 GPIO + state-mirror always travel together

Any task-context code that flips a GPIO whose state has a
firmware-side mirror (`Gpio::kCamMux` ↔ `g_cam_current_id`) MUST
update both atomically — same function, no early return between
them.  Without that pairing, the ISR's tag block reads a stale
mirror and writes the wrong cam_id into the per-buffer tag for
multi-frame stretches.  See `HandleSwitchCameraRequest` in
`libs/camera/camera.cc:866-880` for the canonical pattern.

### 9.2 cam_id per-buffer tagging — 100% shipped (build #953)

**Current state (post-#953, 2026-04-26): TWO consecutive 100/100
runs at alt 3:1 / VGA45.**  Detailed history and the 4-bug stack
that took us from 91% to 100% is in `experiment.md` under the
"Cam-id 100%" section.  Summary:

- PHASE A (single-camera): 20/20 = 100 %
- PHASE B (alt 3:1, MUX flipping every 4 frames):
  cam0 BARS=50/50 + cam1 HBAND=50/50, scrambled=0, wrong-tag=0.
- Architecture: single-writer FB-done tag in CSI ISR + dirty-skip
  with N=1 + dirty checks at all consumer return sites.

The historical narrative below (preserved so future readers
understand the dead-ends) was the picture as of #918, before the
4 bugs were diagnosed.

---

State of the art (build #918 baseline, 2026-04-26):

- **PHASE A (single-camera, no MUX flip): 100 %**
  `select(N) → grabbed_id() == N → content matches test_pattern(N)`
  in 20/20 frames per camera.  Solid.
- **PHASE B (alt 3:1, MUX flipping every 4 frames): ≈ 90 %**
  Tag distribution roughly 75/25 as expected, but ~10 frames per
  100 carry the wrong (cam_id, content) pair.  Reproducible across
  consecutive runs (87, 90, 91 in three trials).

**Why ≈ 90 % and not 100 %.**  The tag-write architecture has
THREE writers touching `g_cam_buf_id[]`:

1. NXP `coralmicro_csi_on_buffer_arm` hook
   (`libs/camera/camera_support.c`) — writes
   `g_cam_buf_id[slot] = g_cam_current_id` at the moment the empty
   buffer is submitted to CSI hw (predictive).
2. CSI `CSI_IRQHandler` FB-done block
   (lines around 305 / 317) — writes `g_cam_buf_id[idx] =
   active_cam` for the just-completed buffer (retrospective).
3. CSI `CSI_IRQHandler` post-flip re-tag (after MUX flip in
   VBLANK) — writes `g_cam_buf_id[OTHER_slot] = pending` to
   pre-tag the buffer the new camera will fill next.

Last-writer-wins — and at alt switching the order is racy enough
that ~10 % of buffers end up tagged with the *wrong* camera.
`HandleFrameRequest` then snapshots the (already-wrong) tag into
`g_cam_buf_id_task[]`, and `sentai_cam_get_raw_with_recovery`
publishes that into `g_cam_grabbed_id`.

**What we tried that did NOT help (each made it worse, not better):**

| Build | Change | Result |
|---|---|---|
| #912 | Drop on_buffer_arm + post-flip re-tag, FB-done sole writer | 86 % |
| #915 | Same as #912 + reread the per-buffer ISR slot in HandleFrameRequest | 86 %, cam1_tag dropped to 50/50 |
| #916 | Restore everything, read g_cam_buf_id[idx] in HandleFrameRequest | 78 %, cam1_tag dropped further |
| #917 | Make on_frame_complete the SOLE writer of g_cam_buf_id[] | 74 %, cam1_tag down to 1/14 |

The collapse on cam1_tag in particular when only one writer remains
strongly suggests a **mid-frame-mix** mechanism (the MUX flip
sometimes lands AFTER the next frame's fill has already started),
not just a tagging race — when row 0 of a buffer was filled by
camA but the rest by camB, the buffer carries camB's tag (correctly)
yet `peek_row(row=0)` reads camA's content.  This explains why the
multi-writer path (which over-writes the post-flip slot with the
new camera) compensates better than any single-writer simplification.

**The real fix — VBLANK-confirmed flip gate (deferred).**

What's needed to break 90 %:
- Read `CSI_REG_SR & CSI_SR_VSYNC_INT_STATUS` (or use an explicit
  `kCSI_StartOfFrameInterrupt` ISR on the side) to *prove* we are
  inside the sensor's VBLANK before committing the MUX flip.
- If we are not, defer `g_cam_pending_mux_id` to the next IRQ.
- With the flip provably in VBLANK, the next FB has zero rows of
  the old camera, the post-flip re-tag becomes redundant, and
  `g_cam_buf_id[idx] = active_cam` at FB-done is the sole writer
  that's correct by construction.

This is real surgery (touches the CSI ISR, needs SOF-IRQ
instrumentation + bench timing measurements at multiple sensor
modes — VGA45 / VGA30 / SXGA15) and was deferred 2026-04-26.  Until
then, the 90 % cap is intentional — see the `Build #911 baseline`
note in `libs/camera/camera_support.c:CSI_IRQHandler` and the same
comment in `libs/camera/camera.cc:HandleFrameRequest`.

**Diagnostic tool:** `diag/_t_pattern_31.py` (PHASE A static,
PHASE B alt 3:1 dynamic) is the load-bearing test for this work.
A single run writes `/diags/sNNN_pattern_31/log.csv` with per-frame
`(i, cam_tag, seen_pattern, b40_byte)`.  Any change to ISR-side
tagging or MUX scheduling MUST re-run this driver and confirm
PHASE A stays 100 % and PHASE B does not regress below 90 %.

---

## 10. Error code registry

Camera-switch codes (module `0x0A`):

| Code | Name | Meaning |
|---|---|---|
| `0x0A00` | `CAM_SWITCH_EOF` | Info: switch committed via EOF ISR |
| `0x0A01` | `CAM_SWITCH_FALLBACK` | ISR did not consume arm in 150 ms → sync fallback |
| `0x0A02` | `CAM_DRAIN_TIMEOUT` | Post-switch drain wait hit 300 ms ceiling |
| `0x0A03` | `CAM_GRAB_RETRY` | `GetRawFrame` failed, toggling MUX to recover |
| `0x0AF0` | `CAM_GRAB_FAIL` | Fatal: all recovery attempts exhausted |

TPU codes (module `0x0B`):

| Code | Name | Meaning |
|---|---|---|
| `0x0B40` | `TPU_ARENA_ALLOC` | Tensor arena alloc failed |
| `0x0B41` | `TPU_MODEL_LOAD` | Model load from flash failed |
| `0x0B42` | `TPU_ALLOC_TENSORS` | AllocateTensors failed |
| `0x0B43` | `TPU_INPUT_COUNT` | Must have exactly 1 input tensor |
| `0x0B44` | `TPU_NOT_READY` | EdgeTPU not initialized |
| `0x0B50` | `TPU_RING_LOOP_BOUND` | Cale 1 ring transfer loop exceeded bounded iters (val=iters) |
| `0x0B51` | `TPU_RING_SLOT_TIMEOUT` | Ring slot USB-done wait timed out (val=slot_idx) |
| `0x0B52` | `TPU_RING_DMA_FAIL` | eDMA SDRAM->OCRAM producer copy failed (val=bytes) |
| `0x0B53` | `TPU_RING_USB_SUBMIT` | `USB_HostEdgeTpuBulkOutSendAsync` returned non-success (val=usb_status) |
| `0x0B54` | `TPU_RING_DRAIN_TO` | Drain wait for final ring slots timed out (val=slot_idx) |

Rule: to add a new code, (1) pick the next free number in the module's
range, (2) append a row to `examples/sentai_runtime/error_codes.csv`,
(3) add a `#define SERR_<MOD>_<NAME>` to `sentai_error.h`, (4) call it
via `SERR_LOG(SERR_<MOD>_<NAME>, val)`.  **Never renumber an existing
code** — old builds' logs would reinterpret the number.

---

## 11. Recent invariants (things to not re-break)

- **Build #633**: `sentai_lfs_task.cc:sentai_lfs_try_serve` restricts the
  fast path to `LFS_REQ_RAW` only.  LS always queues.  Flipping this back
  re-introduces the 30 s root-ls hang + 2 min watchdog reset loop.
- **Fix A**: `g_cam_switch_seq` snapshot is taken inside
  `HandleSwitchCameraRequest` (`libs/camera/camera.cc`) atomically with
  the `GpioSet()` — not in the task wrapper.  See paper §"Fix A".
- **Fix B**: MUX flip now happens in CSI EOF ISR during VBLANK.
  See paper §"Fix B" and §"Head-to-tail benchmark".
- **DMA memcpy**: `detection_task.cc:sentai_dma_memcpy` uses eDMA channel
  31 with 32-byte AXI bursts.  Pre-DMA cache clean is NOT done (both
  buffers are DMA-written).  See [memcpy.md](../paper/memcpy.md).
- **Camera frame rate**: `DEMO_CAMERA_FRAME_RATE = 45` (in
  `libs/camera/camera_support.h`, validated via OV5640 patch
  `0001-ov5640-vga-30fps-pclkperiod.patch`).  60 FPS register exists but
  T-HSSETTLE not validated.  Don't touch without re-validating CSI lock.

### V22 OCRAM tensor (load-bearing for 42 FPS pipeline)

- `s_tpu_input_buf_single` 786 KB in `.tpu_input` (NOLOAD) → m_ocram.
  USB EHCI reads OCRAM via AXBS, NOT SEMC — eliminates contention with
  CSI camera DMA.  Moving back to SDRAM = pipeline 42→1.8 FPS (V13).
- Counting semaphore `s_sem_bufs_free` max=1 enforces strict serial:
  PrepTask cannot overwrite while InferTask USB-reads.

### Cale 1 ring buffer (2026-04-25, default OFF)

- `.tpu_ring` 72 KB OCRAM at 0x20303400, 2x36 KB ping-pong slots.
- eDMA channel **30** (distinct from ch31 detection memcpy, ch29 mover).
- Toggle `sentai.diag.tpu_ring(1)` to route SendInstructions /
  SendParameters / SendInputs through ring.
- Empirical: ring overhead = -10 FPS pipeline (no gain), kept as infra
  for future architectural experiments.

### MoverTask 3-task pipeline (2026-04-25, default OFF)

- `s_prep_sdram_ring[2][786 KB]` in `.sdram_bss` (1.5 MB SDRAM).
- eDMA channel **29** (distinct).  4 sema (`sdram_free`, `sdram_ready`,
  `ocram_free`, `ocram_ready`).
- Toggle `sentai.diag.mover(1)` to enable.  CURRENTLY WEDGES PIPELINE
  due to SEMC contention with USB BulkIn (output 8 KB → SDRAM arena
  during compute+output, concurrent with eDMA SDRAM read).
- Documented dead-end; infrastructure preserved for future bus-mgmt
  experiments (e.g. CSI XRGB→RGB conversion to free SDRAM bandwidth).

### Fine-grained one-shot SendInputs sync (2026-04-25, default ON)

- Driver `g_sentai_tpu_input_done_sema` pointer + atomic-exchange
  one-shot consume in `SendInputs()` end.
- InferTask arms via `sentai_tpu_set_input_done_sema(sem_bufs_free)`
  before invoke, clears after.  PrepTask unblocks at SendInputs end
  (mid-invoke), parallel with compute+output.
- One-shot needed because yolo_1 with parameter_caching calls
  `SendInputs()` 2× per invoke; without one-shot, sem given 2× per
  invoke → PrepTask 2:1 over-production.
- **Glitch-free**: PrepTask cannot write `.tpu_input` OCRAM during
  USB read.  V22 raw had this race (benign empirically but inference
  quality unmeasured).

### Output tensor in OCRAM = DEAD-END

- Tested V23 (`project_v23_ring_analysis.md`): TFLite custom op
  invariants on arena pointer stability.  Pointer-swap output → 41→0.2 FPS.
- Re-asked 2026-04-25: confirmed dead-end, no path to relocation
  without TFLite source modification.

### Linker dead sections cleanup (2026-04-25)

Removed sections that were 0 bytes in V22 build (libs not link-listed):
- `.curl`, `.a71ch`, `.mbedtls`, `.wiced`, `.httpsrv`
- `.edgefast_bluetooth_text/data/rodata`
- `._settings_handler_static`, `._bt_gatt_service_static`,
  `._bt_l2cap_fixed_chan`

Cosmetic only — no bytes saved.  Re-add from `libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld`
template if those libs ever get link-listed in CMakeLists.txt.

---

## 12. First-contact checklist

When picking up the project fresh:

1. `ls /dev/ttyACM*` — should show `ttyACM0`.  If not, board is dead or in
   ROM bootloader (`lsusb | grep 18d1` will show Google Coral ID).
2. `ping -c 2 -W 2 10.0.0.1` — should succeed if CDC-NCM is up.
3. `curl -s http://10.0.0.1/api/raw/log/boot.log | head -3` — should print
   the current build number.
4. `curl -s http://10.0.0.1/api/ls/diags | python3 -m json.tool | head` —
   first call returns `{"error":"lfs_busy"}` (expected; retry after 600 ms).
5. Open a REPL probe with the `send()` snippet from §5 and try
   `print("alive")`.  If no output after 2 s, do the warm-reset recipe
   in §4.
6. `git status` — expect modifications in `examples/sentai_runtime/*.md`,
   `build_version.*`, and possibly the experiment CSVs if you re-ran
   anything.  See `git log --oneline -10` for recent direction of travel.

---

## 13. Where to look for context when this doc is out of date

1. `paper/` — narrative-style lab notes for each optimisation (memcpy,
   cam_switch, lfs, usb, boot, watchdog, cale1_ring_buffer_plan).
   These are the canonical record.
2. `agent/embeded.md` — NASA/JPL discipline rules that govern the
   codebase.  Start here for "why is this coded this way?".
3. `agent/experiment.md` — chronological session log with all
   measurements, dead-ends, shipped configs.  Each session adds at top.
4. `agent/plan.md` — the long-horizon roadmap.
5. `/home/bogdan/.claude/projects/-home-bogdan-work-coralmicro/memory/*.md`
   — persistent user/project memories.  Check `MEMORY.md` index first.
6. `error_codes.csv` — the growing ledger of every logged fault.
7. `SENTAI_API.md` — runtime Python surface, always kept in sync with the
   C bindings.

---

## 14. Lessons learned (2026-04-25 sprint)

These are durable observations from the Cale 1+ MoverTask sprint —
prepend to your mental model when picking up TPU/pipeline work.

### A. The SEMC bus is the single bottleneck

RT1176 has ONE SDRAM controller (SEMC, 32-bit single-channel).  Any
DMA master writing/reading SDRAM concurrently with USB EHCI transfers
on TPU = wedge.  Confirmed repeatedly:
- V13: USB EHCI reads SDRAM tensor + CSI writes camera buf → wedge
- V23 async ring: PrepTask PXP writes SDRAM during USB instructions read → wedge
- 2026-04-25 MoverTask: eDMA reads SDRAM ring during USB BulkIn writes SDRAM arena → wedge

**No amount of priority tuning, QoS knobs, or AXBS master ID
manipulation has fixed this**.  SDRAM bandwidth is finite (~200 MB/s
peak, less under contention).

### B. OCRAM is the only safe DMA target during invoke

USB EHCI reads OCRAM via AXBS crossbar — independent path from SEMC.
That's why V22 ships .tpu_input in OCRAM.  But OCRAM is 1016 KB:
a single 786 KB tensor consumes 77%, leaving no room for ping-pong.

### C. eDMA is FAST but pulls SDRAM into bus contention

7.1 ms for 786 KB SDRAM→OCRAM via eDMA ch31 (32-byte AXI burst,
~110 MB/s).  But during pipeline, eDMA's SDRAM reads compete with
USB BulkIn's SDRAM writes for output → 100% invoke fail.

### D. yolo_1 with parameter_caching calls `SendInputs()` twice per invoke

Don't assume 1:1 between sema gives and invokes.  The fine-grained
one-shot sync uses atomic-exchange to consume the sema-pointer once,
preventing PrepTask 2:1 over-production.

### E. Cross-test contamination wastes hours

Always reflash between A/B tests.  Build counter `#xxx` confirms what
actually got loaded.  The "regression mystery" of 2026-04-25 (42→24 FPS
on same commit) was 100% cross-test contamination — verified by
running V22 driver verbatim on fresh-flashed V22 firmware = 42 FPS.

### F. `verbose(0)` silences ALL prints, not just firmware

Multiple times confused test results.  Always `sentai.verbose(1)`
before reading state via REPL.

### G. NASA/JPL discipline is enforced by `embeded.md`

Every new task / sema / DMA channel needs:
- Bounded loop (explicit max_iter)
- Bounded waits (no `portMAX_DELAY`)
- Static allocation (no heap in steady state)
- Error code (not printf string) on failure paths
- Counter in diag stats
- Default OFF for new infrastructure

The ring buffer (Cale 1) and MoverTask infrastructure shipped in
2026-04-25 follow this discipline — both default OFF, both have
counters, both have bounded loops with error codes.

### H. Ring buffer / MoverTask are dead-ends but the infra is durable

Don't delete — leave the toggles available.  Future work might find
a way to use them (e.g. with smaller models that fit OCRAM ping-pong,
or with CSI XRGB→RGB conversion that frees SDRAM bandwidth).

### I. Camera framerate is the ceiling

OV5640 VGA45 = 22.2 ms/frame = 45 FPS hardware ceiling.  Pipeline
end-to-end can never exceed this.  V22 at 42 FPS = 94% of camera
ceiling.  Further gains require:
- Different camera (no available)
- Different model size (smaller = faster)
- Different architecture entirely (not RT1176)
