# SentAI Runtime — Agent Handoff Guide

Written 2026-04-20, last updated 2026-04-28 (post FileX/LevelX migration
+ Phase 3 perf tuning).
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
| `python3 diag/_host_upload_repl.py --file <name>` | host | REPL-chunked upload to `/lib/diag/` on board (uses `sentai.fs.append`, CHUNK=192 — see §3.1) |
| `python3 diag/_host_run_with_var.py --file <remote> --set K=V --timeout T` | host | exec a driver from LFS after seeding REPL globals (e.g. `_target_fps=30`); streams until `=== done ===` |
| `python3 repl_run.py --line "..."` | host | sends a REPL line and waits for next `>>> ` — has a stale-prompt bug on long commands (minutes), prefer rolling your own driver based on `_host_upload_repl.py`'s `_send_line` pattern |
| `python3 monserial.py` | host | passive serial log to `sentai_serial.log` |
| `cat /dev/ttyACM0` | host | raw serial — needs `stty -F /dev/ttyACM0 115200 raw -echo -icanon` first |
| `curl http://10.0.0.1/api/ls/<path>` | host | list dir via HTTP GET (fast path works for all subdirs; root takes 2 round-trips because of the LS-always-slow-path fix) |
| `curl http://10.0.0.1/api/raw/<path>` | host | read file via HTTP GET (fast path, ≤256 KB per response) |

The REPL-uploader prefix rule: files starting with `_host_` (and, by
package convention, the `diag/drivers/` subdir) are HOST-ONLY and never
pushed to the board.

### 3.1 REPL-chunked upload protocol (revised 2026-04-26 build #986)

`diag/_host_upload_repl.py` pushes a local `.py` file into `/lib/diag/`
on the board using a sequence of independent REPL calls — no growing
in-memory accumulator on the firmware side:

```
sentai.fs.remove("/lib/diag/x.py")               # truncate
sentai.fs.append("/lib/diag/x.py", b"<chunk1>")  # 192 raw bytes/line
sentai.fs.append("/lib/diag/x.py", b"<chunk2>")
... (every ~10 chunks) print('LEN:%d' % sentai.fs.size(...))   # drift check
print('OK:x.py:<size>')                          # marker
```

**Why `sentai.fs.append` and NOT the older `_d = _d + b'...'` accumulator
+ single `sentai.fs.write(_d)`:**

- The accumulator pattern allocates a fresh bytes object on every line
  (`_d = _d + chunk`).  Past ~50 chunks the MicroPython heap fragments,
  GC pauses between lines stretch into hundreds of ms, and the USB CDC
  RX FIFO overflows.  Pyserial sees this as a phantom disconnect:
  `device reports readiness to read but returned no data`.
- The MP interpreter is now in `.micropython` (SDRAM) instead of ITCM
  (build #98x, ITCM space reclaimed for camera/TPU hot paths). SDRAM
  instruction fetch is ~3× slower under SEMC contention — the
  accumulator pattern was on the edge before, and the relocation
  pushed it past the cliff.
- `sentai.fs.append(path, data)` opens the file in `O_APPEND|O_CREAT`,
  writes the chunk, closes — each call is independent, no growing
  buffer, no per-line GC blow-up.  Implemented in
  [`modsentai_fs.c:mod_sentai_fs_append`](../modsentai_fs.c) →
  [`LfsUserAppendFile`](../../../libs/base/filesystem.cc).

**Tuning knobs (MUST stay paired):**

- `REPL_LINE_MAX` in [`micropython_task.c`](../micropython_task.c) —
  upper bound on a single REPL line, currently 1024.  Worst-case
  `repr(bytes)` is 4 chars per byte, so 1024 chars hosts ~240 raw
  bytes of payload after `sentai.fs.append('/lib/diag/X', b'...')`
  overhead.
- `CHUNK` in [`_host_upload_repl.py`](../diag/_host_upload_repl.py) —
  raw bytes per append line, currently 192.  Smaller = more
  round-trips = more risk of CDC stall; larger = exceeds line buffer.
  If you raise CHUNK, raise `REPL_LINE_MAX` first and rebuild.

**Performance:** ~3.5 KB driver in ~11 s (LFS open/close per chunk
dominates).  Acceptable for the diag-driver use case.  Do NOT use
this for large blobs (models, JPEG dumps) — those go via USB MSC
(`sentai.usb.drive(1)`) or HTTP `/api/raw/...`.

**Idiomatic call from a host script:**

```bash
cd examples/sentai_runtime
python3 diag/_host_upload_repl.py --file _t_my_driver.py
```

The remote path is always `/lib/diag/<basename>` — the uploader does
not honour absolute remote paths.

### 3.1.1 Large files (models, JPEG dumps): USB MSC ONLY (added 2026-04-28)

**Rule: anything > ~10 KB MUST be uploaded via USB MSC, not REPL.**

The chunked-append uploader is throughput-bounded by the per-line
LFS open/close pair (~30–60 ms/chunk × 192 bytes/chunk ≈ 1–2 KB/s
sustained).  At that rate a 270 KB MSBlock model takes ~3 minutes,
a 5 MB yolo_1 model takes ~50 minutes, and a 7 MB model takes
~85 minutes — long enough to bump into watchdog warnings (60 s
silence) and CDC backpressure events that drop lines.  Both make
the upload look like it succeeds while silently leaving the file
truncated or zero-length on the board.

**Required workflow for any blob ≥ 10 KB:**

```bash
# 1. Switch board to USB MSC mode.  This unmounts the REPL/HTTP
#    surface, exposes the user partition as /dev/sda (FAT16).
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.3)
s.write(b'\x03\r\n'); time.sleep(0.3); s.read(4096)
s.write(b'sentai.usb.drive(1)\r\n'); time.sleep(1)"
for i in $(seq 1 15); do [ -b /dev/sda ] && break; sleep 1; done

# 2. Mount and copy.
sudo mkdir -p /mnt/sentai && sudo mount /dev/sda /mnt/sentai
sudo cp models/iarna_p3p4_*_edgetpu.tflite /mnt/sentai/models/
sync && sudo umount /mnt/sentai

# 3. Exit MSC mode — warm-resets back to default REPL+HTTP boot.
printf 'q\r\n' > /dev/ttyACM0
for i in $(seq 1 20); do
    lsusb | grep -q "1fc9:c0a1" && break
    sleep 1
done
```

This is the documented path on this firmware (FileX Phase 2.1
shipped MSC mount via real NAND OOB at 2048-byte LBA — Linux mounts
the volume as native FAT16, no special tooling required).  The HTTP
`/api/write` endpoint exists but is known broken (rule #5 — "HTTP
uploads do not work").  REPL `fs.append` is for small driver files
ONLY (the on-board `_host_upload_repl.py`'s 192-byte CHUNK is sized
for sub-KB driver source, not multi-MB binaries).

**When this rule has been violated** (2026-04-28): an attempted
REPL upload of a 271 KB model triggered watchdog timeouts mid-stream;
the partial write left the file at 0 bytes on FileX and the
follow-up `tpu.load(...)` failed with rc=-2.  The forensic detail
is in `experiment.md` "Phase 1 multi-slot firmware" session.

### 3.2 Reading "fs.append returned False" / "lfs.c:560 No more free space"

LFS user partition is **not infinite**.  Three large yolo `.tflite`
models + accumulated `/diags/sNNN_*` session dirs can exhaust it.
The append uploader will surface this as `False` after a few chunks
plus a firmware-side `lfs.c:560:error: No more free space` log line.

**Recipe (added 2026-04-27):**

```python
# Drop unused models first — each is 4-5 MB.  Inspect with:
print(sentai.fs.ls("/"))
# Keep only what tests need (e.g. the canonical
# yolo_1_class_512_1_upsample model used by _t_fps_pipeline.py).
sentai.fs.remove("/yolo_1_class_512_half_size_edge_only.tflite")
sentai.fs.remove("/yolo26n.edgetpu_1.tflite")
```

After freeing space, the FIRST upload can still hit a watchdog log
(`E:0500:NNN`) because LFS is doing block-level GC of the freed
extents — retry once, the second attempt completes in normal time.

### 3.3 Uploader strict-True check (added 2026-04-27)

The chunked uploader REQUIRES `True` in the response of every
`sentai.fs.append(...)` line, not just absence of `False`.  Reason:
when CDC RX backpressure drops a single line, the firmware emits
the next `\r\n>>> ` prompt anyway, and a check that only watches
for `False` will silently lose that chunk.  We saw this 2026-04-26:
5 of 10 lines dropped, no `False`, the host believed the upload
finished.  `_send_line` consumers (any new test driver) MUST
validate the expression's printed return value, not just the
prompt-came-back signal.

---

## 4. How-to: survive a stuck REPL

Symptom: `print("anything")` produces no output on `/dev/ttyACM0`, but the
prompt `>>>` still appears after each command.  Root cause is usually an
experiment that left `sentai.verbose(0)` set or captured stdout.

**For a fully-wedged board (REPL silent, probes return nothing) — fastest
recipe (added 2026-04-26):**

```bash
python3 scripts/flashtool.py -e sentai_runtime --ram
```

`--ram` re-loads the existing ELF into RAM and reboots — board is back in
seconds.  The persistent flash isn't touched, so this is the cheapest
"kick" available.  Reserve **full persistent flash** (same command without
`--ram`) for when you actually need a new firmware to survive a power
cycle.  Don't sit through 120 s of REPL-silence + 30 s WDOG for an unwedge
when one `--ram` flash would have unblocked everything.

**CRITICAL: `sys.reset()` invalidates `--ram` firmware** (added 2026-04-27).
`sentai.sys.reset()` triggers `NVIC_SystemReset` which returns control
to the ROM bootloader.  The ROM bootloader runs whatever is in
**persistent flash**, NOT the ELF that `--ram` loaded into RAM.  Symptom
when you forget this: `init(1, 45)` returns 0 → script calls `sys.reset()`
→ board re-enumerates → next `init(1, 45)` returns the OLD binding's
error like `TypeError: function expected at most 1 arguments, got 2`
because the persistently-flashed firmware predates the new MP binding.

**Rule:** any test driver that calls `sys.reset()` (the self-correcting
`-11 → reset` idiom for runtime fps switching, mode switching, etc.)
REQUIRES `python3 scripts/flashtool.py -e sentai_runtime` (no `--ram`).
Use `--ram` ONLY for tests that stay within a single boot cycle.

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

### 4.1 JTAG / SWD via Segger J-Link (last-resort + debug)

The dev board exposes the SWD pins of the M7; a **Segger J-Link PLUS**
is wired in (`lsusb` shows `1366:0101`).  Use it when:

- USB CDC is dead (no `/dev/ttyACM0`, no NXP enum on `1fc9:c0a1`) and
  the board is not in SDP either (`1fc9:013d`) — power cycle + button
  doesn't help, JTAG SYSRESETREQ is the next step.
- You need to step through a HardFault / WDOG-reset path that the
  crash log + `boot_prev.log` can't pinpoint (e.g. ISR-side hangs,
  pre-scheduler `CHECK` failures).
- You want to inspect live state (CSI/PXP regs, OCRAM contents,
  `flow_shared_t` cross-core) without involving REPL.

**Quick reset via J-Link (no GDB):**

```bash
JLinkExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -autoconnect 1 \
  -CommandFile <(printf 'r\nh\nrx 100\ng\nq\n')
```

`r` = reset (SYSRESETREQ via DAP), `h` = halt, `rx 100` = wait 100 ms,
`g` = go, `q` = quit.  Faster than reflashing when you just need to
kick a wedged board and the firmware in flash is fine.

**Live debug session (GDB + JLinkGDBServer):**

```bash
# Terminal A — start the GDB server:
JLinkGDBServerCLExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -port 2331

# Terminal B — attach GDB to the running ELF:
arm-none-eabi-gdb build/examples/sentai_runtime/sentai_runtime \
    -ex "target remote :2331" \
    -ex "monitor reset" -ex "monitor halt"
```

From there: `bt`, `info reg`, `x/16xw 0x202C1000` (flow_shared),
`x/16xw 0x20240000` (DTC-RAM BootPersist), etc.

**Rules of engagement:**

- **Don't `monitor flash` from GDB** — use `flashtool.py` for actual
  reprogramming.  J-Link's flash loader exists for this part but our
  build artefact is a packaged image (sb file + LevelX header) that
  `flashtool.py` knows how to assemble; raw ELF flash via J-Link will
  brick the FileX volume.
- **JTAG ≠ free pass to skip the anti-brick rule (§2.1).**  If the
  firmware bricks the USB CDC, JTAG can recover it but every other
  developer on the project sees a brick.  Treat JTAG as recovery and
  debug, not as a license to ship code that crashes USB bring-up.
- **M4 attach**: same J-Link, swap `-device MIMXRT1176xxxA_M7` for
  `MIMXRT1176xxxA_M4`.  Useful for the flow-task M4 work (DWT, SAD
  loop step-through).  Note: the M4 has no `BOARD_InitBootClocks`
  in our build (intentional, see flow.md §3.2), so PLL state seen
  via JTAG is the M7-configured state.

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

### 5.1.3 Avoid background poll-loops on /dev/ttyACM0 (added 2026-04-27)

If a previous shell command opened a `until python3 -c "...serial..."`
poll-loop on `/dev/ttyACM0` and was not properly torn down, it will
hammer the port with `\x03\x03\r\n` every iteration.  Symptom:
endless `KeyboardInterrupt\r\n>>>` cascading on every test run, no
useful output.

Diagnose:

```bash
lsof /dev/ttyACM0          # who has the port?
ps -ef | grep -E "until.*serial"   # background poll-loops
```

Then `kill -9 <PID>` the offender BEFORE any `flashtool` or test run.
A test driver that inherits this Ctrl-C cascade will print a misleading
`KeyboardInterrupt: ... line N ...` with N pointing to a benign line
in the driver, sending you on a wild goose chase.

### 5.1.4 Self-correcting `init(1, fps)` -> `sys.reset()` idiom (added 2026-04-27)

For runtime fps switching (or any first-init parameter that the
firmware can detect mismatched on the next call), use the
self-correcting REPL idiom:

```python
rc = sentai.camera.init(1, _target_fps)
if rc == -11:                         # camera up at a different fps
    sentai.rtos.sleep_ms(200)
    sentai.sys.reset()                # never returns
elif rc != 0:
    print("FAIL init=%d" % rc)
    print("=== done ===")
else:
    # ... bench body ...
```

The host runner must:

1. Tolerate the `SerialException` thrown when CDC drops during
   `sys.reset()`.
2. Wait for `lsusb | grep -q "1fc9:c0a1"` re-enumeration before
   re-issuing the same command.
3. Use **persistent flash** (no `--ram` — see §4 above).

Reference implementation: `diag/_host_run_with_var.py` + the
`_target_fps`-driven `_t_fps_bench.py` / `_t_fps_pipeline.py`
canonical drivers.

### 5.1.5 No host-side parameter passing — drivers own their sweeps (added 2026-04-28)

**Doctrine: the on-board driver owns the entire experiment.**  The host
script is a thin pipe: push driver, exec ONCE, wait for `=== done ===`,
download the CSV.  Nothing else.

**Forbidden host-side patterns:**

- `send_line(s, "_target_model = '/models/...'")` — seeding REPL globals
  per iteration before exec'ing the driver.  This is brittle: the
  first-boot prompt handshake races with the seed lines and the driver
  starts with `NameError: '_target_model' isn't defined`.  Even when it
  works, the experiment logic is split between two files in two
  languages and the user can't read the on-board file alone to know
  what's being measured.
- Host orchestrators that loop over models / ratios / fps levels and
  reflash + exec + parse output between iterations.  The branching
  logic belongs INSIDE the driver, not on the host.
- Passing parameters via `os.environ` or stdin to a host wrapper that
  then injects them into the REPL session.

**Required pattern: parameters live as module-level constants at the
top of the on-board driver.**

```python
# _t_iarna_p3p4_pipeline.py
MODELS = [
    ("MSBlock", "/models/iarna_p3p4_MSBlock_..._edgetpu.tflite"),
    ("C2f",     "/models/iarna_p3p4_C2f_..._edgetpu.tflite"),
    ("GELAN",   "/models/iarna_p3p4_GELAN_..._edgetpu.tflite"),
]
FPS = 45

# ... bench loop iterates MODELS itself, writes one CSV row per
#     (model, ratio) pair, and persists progress in /diags/.<exp>_state
#     so it can resume after sys.reset() between iterations.
```

**If the experiment legitimately needs a between-iteration `sys.reset()`**
(TPU contamination, fps re-init, etc.), persist progress in a state file
under `/diags/.<exp>_state` and resume on next boot.  The host's only
job in that case is: detect re-enum, re-exec the same driver, repeat
until the driver prints `=== done ===`.  The host code is identical
across experiments — only the on-board file changes.

**Why:** The user's mental model of an experiment is a SINGLE artefact
they can read, push, and re-run.  Splitting the loop between host and
board means (a) you can't replay the experiment without the host
script, (b) the on-board file alone doesn't tell you what was measured,
(c) prompt-handshake races eat seed-globals on first boot.  The whole
point of the §5.1.2 self-contained-driver doctrine is that one file is
the experiment.

**Reference:** `diag/_t_iarna_p3p4_pipeline.py` — embeds 3-model list,
loops internally, writes one CSV across reboots via state-file resume.
Compare to the rejected pattern in early `/tmp/run_iarna_p3p4_pipeline.py`
(reflash + send_line seed-globals per iteration) which broke on first
boot with `NameError`.

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

**Multi-slot TPU surface (Phase 1 + 2a + 2b raw introspection, build #1098+):**

| Call | Effect |
|---|---|
| `sentai.tpu.slot_count()` | 3 (compile-time `kNumTpuSlots`). |
| `sentai.tpu.load_slot(N, path)` | Load model into slot N.  Slot 0 routes through legacy `sentai_load_model(path)`; slots 1+ heap-allocate a 2 MB arena lazily. |
| `sentai.tpu.invoke_slot(N)` | Invoke slot N's interpreter.  Returns elapsed ms or negative error. |
| `sentai.tpu.slot_ready(N)` | bool. |
| `sentai.tpu.set_input_slot(N, bytes)` | **Caveat**: MP heap can't hold tensors ≥ ~256 KB.  Use the camera/PrepTask path for real workloads — this is for tiny REPL tests only. |
| `sentai.tpu.output_slot(N, idx)` | Raw output bytes (length = `output_size_slot`). |
| `sentai.tpu.num_outputs_slot(N)` | int |
| `sentai.tpu.output_size_slot(N, idx)` | bytes |
| `sentai.tpu.output_dims_slot(N, idx)` | tuple (e.g. `(1, 30, 40, 6)`) |
| `sentai.tpu.output_type_slot(N, idx)` | TfLiteType int (3=uint8, 9=int8, 1=float32, ...) |
| `sentai.tpu.output_quant_slot(N, idx)` | `(scale, zero_point)` |
| `sentai.tpu.output_hash(N)` | FNV-1a 32-bit over all output bytes — handy for verifying that slot N produced its expected result. |
| `sentai.pipeline.set_slot_for_cam(cam_id, slot)` | Route per-camera frames to a specific slot (Phase 2a).  Default `{0:0, 1:0}` = legacy single-slot. |
| `sentai.pipeline.slot_stats()` | `(s0, s1, s2)` per-slot invoke counters incremented in InferTask. |

**REPL post-processing**: detect/draw/yolo_info bindings were
intentionally retired in build #1100.  REPL exposes raw output bytes
+ shape/dtype only; structured post-processing (NMS, classification,
keypoints, ...) will return as typed C++ helpers when a real consumer
needs them.  Don't rebuild a Python NMS in MP — heap is too small,
arithmetic on `bytes` is too slow, and the user-side path was always
intended to be C++.

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

### 9.2.3 Sensor framerate is a compile-time constant

`DEMO_CAMERA_FRAME_RATE` in `libs/camera/camera_support.h:107` is
the **only** knob for OV5640 frame rate at boot.  It's a `#define`,
which means changing the runtime fps requires a full rebuild + flash.

The OV5640 driver table in
`third_party/nxp/rt1176-sdk/components/video/camera/device/ov5640/fsl_ov5640.c`
has VGA entries at 15, 30, 45, 60, and 90 fps (confirmed by grepping
`framePerSec` against `kVIDEO_ResolutionVGA`).  Pick one of those,
not an arbitrary number — others fall through to a default that may
or may not produce stable frames.

A `sentai.camera.set_hw(w, h, fps)` binding is referenced in older
diag drivers (`alt_fps_matrix.py`) but **does not exist on this
branch**.  Any new diag driver that calls `set_hw` will fail with
`AttributeError`.  If you need runtime fps switching, add the binding
properly — don't paper over with a `try/except` that masks the
failure.

**Higher-fps init caveat (open issue, 2026-04-26):**  Building with
`DEMO_CAMERA_FRAME_RATE = 45` (or 60) currently produces a firmware
that wedges during the warm-up `select()` calls of new diag drivers.
`E:0A01` (`CAM_SWITCH_FALLBACK`) followed by `E:0A02:300`
(drain timeout) within seconds of boot.  Only WDOG recovery (~3 min)
restores the REPL.  The cam_id correctness work (build #953) is
unaffected — it was validated at VGA30 and is fps-independent in
mechanism — but the VGA{45,60} sensor modes need a separate
init-time investigation before the bench tables in
`experiment.md` can be filled in for those rows.

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

- **Build #633**: `sentai_fs_task.cc:sentai_fs_try_serve` (renamed from
  `sentai_lfs_*` in #1070) restricts the fast path to `FS_REQ_RAW` only.
  LS always queues.  Flipping this back re-introduces the 30 s root-ls
  hang + 2 min watchdog reset loop.
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

### Filesystem layout (2026-04-28+, build #1062+)

The board has TWO filesystem partitions on NAND:

| Partition | Range | Backend | Where |
|---|---|---|---|
| System  | blocks 12..75 (8 MB)   | **LittleFS** | `default.elf`, MicroPython runtime — read-mostly |
| User    | blocks 76..523 (56 MB) | **FileX FAT16 / LevelX** | everything user-visible: `/log/`, `/diags/`, `/.sys/`, models, JPEGs, configs |

**Constraint**: the two filesystems are MUTUALLY EXCLUSIVE on the user range.
Don't write LittleFS code paths against blocks 76..523 — you'll corrupt
the FileX volume on next boot.

---

## 12. How to interact with the user partition (FileX)

### From MicroPython REPL — `sentai.fs.*`

```python
sentai.fs.write("/path/file.bin", b"...")    # truncating write
sentai.fs.append("/path/file.bin", b"...")   # append-only (REPL uploader)
sentai.fs.read("/path/file.bin")             # full file -> bytes
sentai.fs.size("/path/file.bin")             # int, -1 if missing
sentai.fs.exists("/path/file.bin")           # bool
sentai.fs.ls("/dir")                         # list of (name, type, size)
sentai.fs.mkdir("/a/b/c")                    # mkdir -p
sentai.fs.remove("/path/file.bin")           # 0 ok, <0 errno
```

System partition is reachable via `$/`-prefixed paths (read-mostly).

### From C++ — `LfsUser*` helpers (compat) or `FxUser*` (typed)

`libs/base/filesystem.h` exposes the long-standing `LfsUser*()` helper
API — these are now THIN SHIMS over FileX, kept so existing call sites
keep compiling.  New code should prefer the typed C API in
[libs/base/fx_user_fs.h](../../../libs/base/fx_user_fs.h)
(`FxUserReadFile`, `FxUserWriteFile`, `FxUserAppendFile`, `FxUserListDir`,
`FxUserStat`, `FxUserMakeDirs`, `FxUserRemove`, `FxUserSync`).

**`LfsUser()` (raw `lfs_t*` accessor) returns `nullptr`** — Phase 2
ripped out the ~30 raw `lfs_*(LfsUser(), ...)` consumers.  Don't add
new ones; they will null-deref at runtime.

### From the host

```bash
# HTTP browser at http://10.0.0.1/  (after sentai.usb.ip(1))
curl http://10.0.0.1/api/ls/         # list dir, JSON
curl http://10.0.0.1/api/raw/path    # read file, bytes
curl -X POST --data-binary @local /api/write/path   # upload
curl -X POST /api/mkdir/path         # mkdir -p
curl -X POST /api/rm/path            # delete
# CDC-ACM REPL chunked uploader
python3 diag/_host_upload_repl.py --file <name>     # to /lib/diag/
# USB MSC (hot-plug FAT volume)
sentai.usb.drive(1)   # in REPL → enter storage mode → /dev/sda
mount /dev/sda /mnt   # Linux auto-mounts; bidirectional file ops
# Send 'q' to /dev/ttyACM0 → warm reset back to default mode.
```

### Constraints — things to know before you write code

1. **Root directory cap = 256 entries.**  FileX format uses FAT16 with
   a fixed-size root.  Hitting the cap returns `FX_NO_MORE_SPACE` from
   `fx_file_create` (opaque from REPL).  Subdirectories are
   unlimited.  Diag drivers MUST put outputs under
   `/diags/sNNN_<exp>/`, NEVER in root — same rule as §5.1.2 still
   applies.
2. **Sector size = 2048 bytes** (FAT data area = full NAND page).  Min
   storage cost per file = one cluster = 2048 bytes (since cluster=1
   sector).  Don't write thousands of tiny files; batch them.
3. **MSC and FileX are mutually exclusive at runtime.**  In storage
   mode FileX is unmounted; in default mode MSC is not exposed.
   Mode-switch goes via `sentai.usb.drive(1)` → warm reset.  Files
   written from the host in storage mode are visible from REPL after
   exit, and vice versa.
4. **Power-fail durability is per-file-close, not per-write.**
   `fx_file_close` flushes the file's FAT chain.  `FxUserSync()`
   forces a full FAT-table flush — call it before a planned
   `sys.reset()` if you must guarantee earlier writes are on disk.
   Phase 3.2 dropped per-write `fx_media_flush` — DON'T re-add it
   "just in case" (small-file writes regress 40× if you do).
5. **No `lfs_setattr` / `lfs_getattr` substitutes.**  FAT has no
   arbitrary user attributes; old LFS code that stored write-time as
   an attr is silently a no-op.  Use file mtime via `FxUserStat` if
   you need timestamps, or write metadata to a sidecar file.
6. **`/api/ls` may return `{"error":"lfs_busy"}`** under heavy
   concurrent load.  Phase 3.4 dropped the rate from 54% to 0.2% but
   it's not zero.  Browser retries on 600 ms.  Host scripts should
   too.  The JSON tag string is preserved (browser.html depends on
   it) even though the file is now `sentai_fs_task.cc`.
7. **HTTP uploads CAN now work** (small-medium files via
   `/api/write/...`).  Throughput on `/api/raw` is ~16 KB/s with the
   lwip Nagle-off patch.  For >100 KB transfers, USB MSC is faster.

### Write performance — what to expect

Numbers from build #1074, freshly-formatted volume:

| Op | Time | Throughput |
|---|---|---|
| Write 256 B | 67 ms | 4 KB/s (FAT/dir overhead dominates) |
| Write 4 KB | 20 ms | 200 KB/s |
| Write 64 KB | 2.1 s | 30 KB/s |
| Write 256 KB | 1.7 s | 150 KB/s |
| Read 64 KB | 24 ms | 2.6 MB/s |
| Read 256 KB | 96 ms | 2.7 MB/s |
| `/api/ls` latency | 24 ms | — |
| `/api/raw` 64 KB | ~4 s | ~16 KB/s (Nagle-off) |

Caveats:
- Writes degrade as the volume fills and the FAT/dir chains grow.
  Reformat (`sentai.diag.fx_format(0xDEADBEEF)`) before perf runs to
  get comparable numbers.
- The append uploader (`diag/_host_upload_repl.py` with
  `sentai.fs.append`) is dominated by REPL line round-trip latency,
  not FAT cost.

### Recommendations

- Put diag outputs in `/diags/sNNN_<name>/` (auto-numbered).  Inline
  the `_session_dir` helper per §5.1.2 — never depend on
  `diag/__init__.py`.
- For large datasets (models, video dumps), upload via USB MSC
  (`sentai.usb.drive(1)` → drag-drop → `q` to exit).  REPL chunked
  uploader is for small driver scripts (~few KB).
- Before a planned `sys.reset()` after writing important data, call
  `FxUserSync()` (or `sentai.fs.sync()` if exposed).  Without it the
  last writes may not have hit FAT yet.
- If you need to debug while in storage mode (REPL is gone), write
  to the SDRAM debug log — `sentai_storage_log("...")` from C side,
  read back via `sentai.diag.storage_log()` after exiting storage.
- For any new C++ consumer, use `FxUser*()` (typed, returns
  bool/ssize_t).  Don't add new `LfsUser*()` callers — that surface
  is frozen as a compat shim.

### What NOT to do (sharp edges with audit trail)

- Don't carve LevelX spare from the data area to get sub-2048 sector
  size.  See experiment.md "FileX OOB discovery" — the chip has real
  64-byte OOB and the NXP driver exposes it via `length` parameter.
- Don't bump `sectors_per_cluster` without also bumping
  `FX_MAX_SECTOR_CACHE` and `g_fx_media_memory`.  Phase 3.1 tried
  `sectors_per_cluster=4` and regressed 64 KB+ writes ~2×.
- Don't re-enable Nagle in lwip httpd (SDK patch
  `0004-lwip-httpd-empty-body-post.patch` disables it).  Re-enabling
  drops `/api/raw` from 16 KB/s back to 3 KB/s.
- Don't shrink `sentai_fs_task` queue depth back to 1.  See §11
  invariants.

---

## 13. First-contact checklist

When picking up the project fresh:

1. `ls /dev/ttyACM*` — should show `ttyACM0`.  If not, board is dead or in
   ROM bootloader (`lsusb | grep 18d1` will show Google Coral ID).
2. `ping -c 2 -W 2 10.0.0.1` — should succeed if CDC-NCM is up.
3. `curl -s http://10.0.0.1/api/raw/log/boot.log | head -3` — should print
   the current build number.
4. `curl -s http://10.0.0.1/api/ls/diags | python3 -m json.tool | head` —
   on a quiet board, returns the listing within ~25 ms.  Under heavy
   concurrent writes you may see `{"error":"lfs_busy"}` (rare since
   Phase 3.4 — was 54% before, now 0.2%); retry after 600 ms.
5. `mount /dev/sda /mnt` — entering storage mode (`sentai.usb.drive(1)`)
   should expose a Linux-mountable FAT volume.  If `dmesg` says
   `Unsupported sector size` something regressed in fx_nand geometry
   (must be 2048-byte LBAs, see §11 FileX/LevelX section).
5. Open a REPL probe with the `send()` snippet from §5 and try
   `print("alive")`.  If no output after 2 s, do the warm-reset recipe
   in §4.
6. `git status` — expect modifications in `examples/sentai_runtime/*.md`,
   `build_version.*`, and possibly the experiment CSVs if you re-ran
   anything.  See `git log --oneline -10` for recent direction of travel.

---

## 14. Where to look for context when this doc is out of date

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

## 15. Lessons learned (2026-04-25 sprint)

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

For the LittleFS→FileX/LevelX migration and Phase 3 perf-tuning
narrative (4 builds + dead-ends + benchmarks), see the
"FileX/LevelX migration" section in [experiment.md](experiment.md).

---

## 16. Boot path stability review (2026-04-29, build #1102)

Audit pass over `libs/base/main_freertos_m7.cc::real_main()` and
`examples/sentai_runtime/sentai_runtime.cc::app_main()` per
embeded.md principles.  Findings worth knowing for future work.

### What's strong

- **DTC-RAM `BootPersist` struct with magic + ~magic check** survives
  warm reset, cleared by POR — correct semantics for storage-mode
  toggle.
- **`kMaxStorageAttempts = 3`** crash-loop guard for storage-mode
  boots — board falls back to default REPL+IP if storage init keeps
  crashing.  Anti-brick rule §M satisfied.
- **`sentai_boot_progress_mark(code)` checkpoints** stamped through
  the entire boot flow (0x01..0x14) into DTC-RAM `progress` field.
  Next boot reads `prev_progress` to diagnose where the previous
  boot crashed — exactly the breadcrumb pattern embeded.md §I asks for.
- **`vApplicationStackOverflowHook`** strong override saves
  `SERR_SYS_STACK_OVF` + task-name hash before resetting.
- **`vApplicationMallocFailedHook`** strong override in `sentai_fault.cc`
  saves `SERR_SYS_MALLOC_FAIL` + caller LR before resetting.
- **HardFault / MemManage / BusFault / UsageFault** all have armv7-m
  naked handlers in `sentai_fault.cc` that capture the exception
  frame and persist it to DTC-RAM before reset.

### Open concerns (logged for future work, NOT fixed in this pass)

1. **`CHECK(...)` is a no-op pre-scheduler.**  `libs/base/check.h`
   defines `CHECK(a)` as `if (!(a)) { EmergencyWrite(...);
   vTaskSuspendAll(); }`.  `vTaskSuspendAll()` only halts when the
   scheduler is RUNNING; called before `vTaskStartScheduler()` it
   sets a flag and returns.  Code execution continues with the failed
   precondition.  In `real_main` this affects:
   - `CHECK(coralmicro::LfsInit())` — if LFS init fails, `LfsUserInit()`
     runs on a dead `lfs_t*`.
   - `CHECK(coralmicro::LfsUserInit())` — if FileX mount fails,
     downstream code accesses null `g_fx_media`.
   - `CHECK(xTaskCreate(app_main, ...))` — if task create OOMs,
     `vTaskStartScheduler()` runs with no app_main scheduled.

   In all three cases the WDOG (configured in app_main, NOT
   pre-scheduler) eventually fires — but only after USB CDC has
   gone up via `UsbDeviceTask::Init()` (line 444), so the recovery
   path is alive.  **Net assessment**: pre-scheduler CHECK fail is
   detectable post-mortem (boot_log + crash log) but should be
   strengthened.

   **Proposed fix (future)**: replace pre-scheduler CHECK body with
   a busy-loop that kicks WDOG1 every 5 s while logging the failure.
   Lets the host reflash after the firmware self-reports that it's
   stuck.  Requires WDOG1 init to move BEFORE the first CHECK.

2. **WDOG1 is configured AFTER USB CDC** (in `app_main` /
   `sentai_health_init`) — anti-brick rule §M is satisfied because
   USB CDC comes up first (in `UsbDeviceTask::Init` at line 444 of
   `real_main`), but the gap between USB-up and WDOG-armed is
   ~tens of milliseconds where a hard crash would not auto-recover.
   Acceptable risk on this build.

3. **Slot 0 detect/draw still in flash** — `sentai_tpu_detect`
   (~150 lines) and `sentai_tpu_draw` (~250 lines) are NOT exposed
   from REPL anymore (build #1100), but `detection_task.cc` still
   uses `sentai_tpu_detect` for NMS post-processing, so the symbol
   stays.  The MicroPython wrappers (`mod_sentai_detect`,
   `mod_sentai_draw`, `mod_sentai_yolo_info`) WERE removed in
   build #1102 (~80 lines reclaimed from ITCM).

### Crash-survivable boot.log (build #1110, 2026-04-29)

Boot log buffer moved from `.sdram_bss` (zeroed at startup) to a new
NOLOAD section `.sdram_boot_log` placed in `m_sdram` so it survives
warm reset / WDOG / hard fault on this silicon.

Layout: 16 KB total, 16-byte header + 16368 bytes payload.

```c
struct BootLogPersist {
    volatile uint32_t magic;   // kBootLogMagic = 0xB00710B0
    volatile uint32_t check;   // ~kBootLogMagic — both must match
    volatile uint32_t len;     // payload length, 0..kBootLogBufSize
    volatile uint32_t _rsvd;
    char              buf[16368];
};
```

Lifecycle:
1. `boot_log_init()` (early in app_main) reads pre-existing
   `magic+check+len` from SDRAM.  If valid, snapshots
   `g_prev_boot_log_len` for later rescue.  Then re-arms header for
   THIS boot's writes.
2. `boot_log_write()` → `_write` printf hook → bytes accumulate in
   the persistent buffer with `len` updated atomically.
3. `boot_log_fs_init()` (after `FxUserInit` succeeds): if
   `g_prev_boot_log_len > 0`, write the rescued bytes to
   `/log/boot_prev.log` (truncating).  Then start normal
   `boot.log → boot_old.log` rotation for this boot.
4. `boot_log_stop()` (when REPL starts): final flush + `FxUserSync()`
   + clear magic.  This boot reached REPL successfully → no rescue
   needed on next boot.  Without this clear, a clean reset would
   re-flush an already-written trace.

Failure-mode coverage:
- **Crash mid-boot before FxUser mount**: SDRAM buffer + magic survive,
  next boot rescues to `/log/boot_prev.log`. ✓
- **WDOG reset after REPL up**: magic invalidated by `boot_log_stop`,
  no false rescue.  Active trace already in `boot.log`. ✓
- **Power cycle (POR)**: SDRAM zero-initialized at cold start, magic
  invalid, no rescue (intentional — no useful data anyway). ✓

Validated on build #1110:
- Clean `sentai.sys.reset()` → next boot has NO `/log/boot_prev.log` ✓
- boot.log header rotates correctly: `#1110` current, `#prev` in `boot_old.log` ✓

### Boot path Option B (refactor real_main + post-scheduler init task) — DEFERRED

Original concern (agent.md §16 above): `CHECK(...)` macro is no-op
pre-scheduler.  Refactoring `real_main` to defer LfsInit / LfsUserInit /
class registrations into a `boot_init_task` running post-scheduler
would let `CHECK` actually halt cleanly.

**Reason for deferring**: with the crash-survivable boot log shipped
(above), pre-scheduler CHECK fails are now FULLY DIAGNOSABLE
post-mortem — bytes printed before the failure persist in SDRAM
NOLOAD, get rescued to `/log/boot_prev.log` on the next boot.  WDOG
still eventually resets the board.  The Option B refactor would
provide cleaner code structure but no additional operational safety
benefit.  Not worth the boot-path refactor risk for negligible win.

If `CHECK` semantics ever need to actually halt pre-scheduler (e.g.
for safety certification work), then Option B becomes the right
answer.  Until then, the current diagnostics+WDOG combination is
sufficient.

### Multi-slot stack hardening shipped (build #1102)

| Issue | Severity | Fix |
|---|---|---|
| `new` without null-check (slot 0 + slot 1+ paths) | CRITICAL | `new(std::nothrow) T()` + null-check + SERR log |
| `printf("ERROR")` instead of SERR_LOG | MAJOR | All slot error paths log `SERR_TPU_SLOT_*` codes (0x0B70-0x0B74) |
| `set_slot_for_cam` accepts unloaded slot | MAJOR | `slot != 0 && !slot_ready` rejected with `SERR_TPU_SLOT_NOT_READY` (0x0B74) |
| `mod_sentai_detect/draw/yolo_info` dead bindings | minor | Removed from MicroPython surface (table + wrapper bodies) |
| `sync_slot0_to_legacy` / `sync_legacy_to_slot0` unused helpers | minor | Removed (sync done inline in `sentai_load_model_slot`) |

Smoke-tested on persistent build #1102:
- `set_slot_for_cam(1, 1)` with slot 1 unloaded → returns -3, logs `E:0B74:17`
- `load_slot(0, MSBlock)` legacy path: rc=0
- `load_slot(2, GELAN)` heap path: rc=0
- `output_dims_slot(2, 0)` returns `(1, 30, 40, 6)`
- `hasattr(sentai.tpu, "detect")` → `False` (retired)
- `hasattr(sentai.tpu, "draw")` → `False` (retired)

---

## 17. Flow architecture (2026-05-05, build #1139+) — `sentai.flow`

The optical-flow stack lives entirely on **M7** (was on M4 in
builds 720..1129; M4 path retired and `sentai_flow_m4` no longer
built — see `experiment.md` 2026-05-05 session for the why).
Pipeline (single-task on M7):

```
flow publisher_task @ tskIDLE_PRIORITY+2
  while running:
    sentai_cam_grab_latest()               -- waits for next camera frame
    if frame_seq advanced:
      sentai_flow_publish_frame():
        PXP scale 640x480 -> 80x60 RGB888  ~1.15 ms (hardware DMA)
        RGB -> Y conversion + dual write    ~0.22 ms
        SAD 25x25 search × 32x32 block      ~0.94 ms (USAD8 SIMD)
        parabolic sub-pixel fit, conf-floor 150, deadband 50 milli-gp
        publish (dx, dy, sad, conf, frame_seq) into FLOW_SHARED()
    sentai_cam_return_raw()
    taskYIELD()
```

API (no `m4_` prefix anymore):
- `sentai.flow.enable()` -- stamp shared-mem magic (no-op on M7-only)
- `sentai.flow.start(cam_id)` / `stop()` -- spawn/kill publisher_task
- `sentai.flow.read()` -> dict {dx, dy, sad, conf, frame_seq, ...}
- `sentai.flow.body_read()` -> dict with body-frame conversion per cam
- `sentai.flow.gray_snap()` -> bytes (4800 B; allocates MP heap)
- `sentai.flow.gray_to_cache()` -> int (zero-alloc; writes to `sentai.diag.cache_*` for bulk capture)
- `sentai.flow.detail_score()` -> int (gradient energy x100)
- `sentai.flow.gray_stretch([on])` -> dict
- `sentai.flow.pub_stats()` -> dict {frames, grab_fail_total/streak, running}
- `sentai.flow.perf()` -> dict {pxp_cyc, rgb2y_cyc, sad_cyc, total_cyc, grab_cyc, loop_cyc}
   (DWT cycle counts; 800 cyc = 1 us)

Output convention: `dx, dy` in **milli-grid-pixel units (1000 = 1 grid-px,
1 grid-px = 8 raw-px after the PXP step-8 downscale)**.  Integer in
shared mem, host divides by 1000 for fractional values.

### Best practices (Cortex-M7 SIMD, integration discipline)

These were learned the hard way during the 2026-05-05 flow rework.
Apply when writing other tight integer-loop code on M7.

1.  **Use `__USADA8` for sum-of-absolute-differences.**  Cortex-M7
    DSP-extension intrinsic from `<arm_acle.h>` does 4 abs-diffs
    + accumulate in 1 cycle.  Replaces ~16 cycles of scalar
    sub/abs/add ops.  Drop-in for any byte-wise diff loop.

2.  **NEVER use `memcpy(&u32, p, 4)` to load an unaligned word.**
    GCC compiles it as a *function call* to libc memcpy unless
    alignment is statically provable.  ~30 cycles overhead per
    call; in a SAD inner loop 320,000 calls per frame turns the
    loop from 1 ms to 13 ms.  Use the packed-struct trick:
    ```c
    typedef struct { uint32_t v; }
        __attribute__((packed, aligned(1))) u32_unaligned;
    #define LD32U(p) (((const u32_unaligned*)(p))->v)
    ```
    Generates a single `LDR` (Cortex-M7 supports unaligned LDR
    in hardware at ~1 extra cycle).

3.  **Verify the assembly.**  After any inner-loop change, run
    `arm-none-eabi-objdump -d <obj>` and confirm the expected
    instruction is emitted.  Counter-example: my first USAD8 attempt
    DID emit USADA8 but ALSO emitted 8 `bl 0 <memcpy>` calls per
    iteration -- net SLOWER, despite the SIMD intrinsic.  Without
    the disasm I would have shipped a regression.

4.  **EMA smoothing breaks integration.**  An IIR/EMA at the source
    reports a real motion event N times (with decaying weight) ->
    the cumulative sum integrates ~Σ alpha^k ≈ 2x the real motion.
    Use deadband + per-frame conf-floor + parabolic-fit shallow-
    surface rejection instead.  EMA only at the consumer level
    (display smoothing), never at the source of integration.

5.  **For bit-perfect on-board / offline replay, capture the EXACT
    input the algorithm consumed**, not the latest published.  M7-
    only SAD + bulk capture format `(text-header)\n(4800 raw bytes)`
    -- offline numpy replay produces results within 1 milli-gp of
    firmware (rounding only).

6.  **Driver loops MUST dedupe by frame_seq.**  If you sample the
    algorithm output at >camera fps, you'll record each result
    multiple times and the cumsum on the trace will be inflated
    proportionally.  Pattern in `_t_flow_validate.py`:
    ```python
    if d["frame_seq"] == last_logged_seq[0]:
        continue
    last_logged_seq[0] = d["frame_seq"]
    ```

7.  **Hot-path code goes in ITCM (`__attribute__((section(".ramfunc")))`).**
    Same trick the CSI ISR uses (agent.md sec 2).  Marginal in
    practice when D-cache is well-warmed (SAD inner loop benefitted
    only ~5% from ITCM placement) but free safety margin against
    SDRAM bus contention.

### CRITICAL: NXP SDK patches must be applied (auto via CMake)

The submodule at `third_party/nxp/rt1176-sdk` is **upstream-clean +
4 SentAI patches applied at build time** by
`scripts/apply_sdk_patches.sh` (invoked from the top-level
`CMakeLists.txt` configure step).  Patches live in
`patches/coralmicro-rt1176-sdk/` and are required for correct
behaviour:

| Patch | What it fixes | Symptom if missing |
|---|---|---|
| `0001-ov5640-vga-30fps-pclkperiod.patch` | OV5640 VGA@30fps `pclkPeriod` 0x0a → 0x14 | D-PHY loses sync after frame 1; CSI timeouts |
| `0002-ehci-queue-depth-16.patch` | USB EHCI QH/QTD pool 8 → 16 | TPU USB transfer stalls under high pipeline load |
| `0003-fsl-csi-coralmicro-irq-hooks.patch` | CSI ISR hooks for camera_support | flow / pipeline can't tag per-buffer cam_id |
| `0004-lwip-httpd-empty-body-post.patch` | Accept empty-body HTTP POST | `/api/write` HTTP returns 500 on empty body |

**`git status` ALWAYS shows the submodule as dirty** (` m third_party/nxp/rt1176-sdk`)
because patches are applied in-place but never committed.  This is
INTENTIONAL -- the SDK is read-only upstream.  The dirty state
means patches are LIVE.

**Verify patches applied:**
```bash
bash scripts/apply_sdk_patches.sh   # idempotent; "skip already applied" = OK
```

**If you re-init or update the submodule** (e.g. `git submodule
update --init --recursive`), patches are wiped.  Re-run the script
or just run a fresh `cmake -S . -B build` (CMake invokes it at
configure time).

**Symptom of missing OV5640 patch (0001):** camera VGA30 init
returns 0 but `sentai.camera.frame_count()` shows ~half the
expected ISR rate (CSI receiver loses sync after first frame and
times out the rest).  Verify before chasing fps regressions in
flow/pipeline code.

### M4 retirement breadcrumbs (2026-05-05, build #1130)

Why M4 went away:
- `BOARD_InitBootClocks` skipped on M4 (would reset M7-owned camera /
  I2C pins) → SysTick ran ~360x faster than `configCPU_CLOCK_HZ`
  expected → `vTaskDelay(1)` slept ~3 us instead of 1 ms.
- M4 task froze after ~5 s of activity (no HardFault forwarded back
  to M7; loss was silent).
- Empirical throughput: ~1 fps under bulk-capture load (wanted ~25).

`flow_task_m4.cc` is kept on disk as historical reference but no
longer in the build (see `examples/sentai_runtime/CMakeLists.txt`).
Don't re-enable without first fixing:
  (a) calibrating M4 SysTick post-StartM4 (read M4_CLK_ROOT)
  (b) M4 HardFault handler that publishes into shared OCRAM so M7
      can detect and SERR_LOG the crash
  (c) M4 watchdog (separate or M7-supervised via heartbeat-stall
      monitor)

### Real-life validation experiment

`examples/sentai_runtime/experiments/s083_flow_m7_validate/` --
end-to-end run with bulk capture, host-side numpy replay, and
trajectory rendering.  See its README for the headline numbers
(bit-perfect firmware vs offline) and the trajectory PNGs
(`trajectory_clean.png`, `trajectory_on_gray.png`,
`trajectory_per_side.png`) for visual sanity check.
