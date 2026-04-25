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
