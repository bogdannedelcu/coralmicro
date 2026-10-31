# EXP-s249 — Firmware hardening regression (REPL / console / RTOS / HTTP)

**WBS:** `OP-S10-W23-T7` (firmware hardening audit; see memory
`project_todo_exhaustive_code_audit.md`).
**Board:** new rev 2026-10 (CAM1 on LPI2C1), RAM-loaded images (board does
not boot from flash yet — BOOT_MODE straps, EXP-s248).
**Dates:** 2026-10-30 / 2026-10-31.  Builds 1623 → 1630.

## What it proves

The 2026-10-30 exhaustive-audit fixes (three `embeded-reviewer` reports:
camera/pipeline, base/boot/USB/health, MicroPython/FS/httpd) do not regress
and the defects they target stay fixed.  The audit item (base m11) asked for
exactly this: "no regression test covers console drop/overflow, descriptor
length, or pre-scheduler CHECK".

## How to run

```bash
# board on USB, firmware RAM-loaded:
venv-flash/bin/python scripts/flashtool.py -e sentai_runtime --ram
python3 examples/sentai_runtime/experiments/s249_hardening_regression/repl_regression.py
# flags: --no-http (no NCM / curl), --no-camera (sensors not mounted)
```

Exit code = number of failures.  Pass criterion: `failures: 0`.

## Checks and the defect behind each

| # | check | defect fixed (build) |
|---|---|---|
| R1 | `print("SBMR1=0x%08X …" % (p(0x40C04008), …))` parses | MicroPython `objint_longlong.c` used `strtoll()` on the non-NUL-terminated lexer vstr → any literal > 2^30 after a longer token failed with `SyntaxError: invalid syntax for integer` (1628, `patches/micropython/0001`) |
| R2 | `99999999999999999999` → `OverflowError` | same patch — silent wrap before |
| R3 | `int("-1086341128")` negative | same patch — `neg` was ignored on the long-int path |
| R4 | CRLF-pasted block runs every body line | `repl_readline` read `\n` after `\r` as an empty line → block ended after the first body line (1628) |
| R5 | 20 KB `print` complete | `_write` → ConsoleM7 dropped bytes beyond ~1 KB; now 256 B slices with back-pressure (1627) |
| R6 | 1200-char line rejected, REPL alive | editor hit `REPL_LINE_MAX` and executed the tail as a second statement (1629) |
| R7 | Ctrl-C → `KeyboardInterrupt`, REPL recovers | ctrl-c monitor stole input bytes; now a flag latched in the USB RX path (1627) |
| R8 | `RecursionError` instead of a reset | `MICROPY_STACK_CHECK` off; 16 KB task stack overflowed into a hardware reset (1627) |
| R9 | `rtos.suspend("tcpip_thread"/"hw_wdog")` refused | protected list lacked tcpip/lfs/fs/wdog/sd tasks (1629) |
| R10 | both sensors capture to SD | `GetFrame` unbounded loop (save_jpeg wedge), `Detect()` false positive, CAM1 on LPI2C1 (1617–1627) |
| R11 | `sentai.imu.read()` dict | LIS2DUX12 driver (new board IMU) (1620s) |
| R12 | `/api/ls`, `/api/raw` ×3 identical over NCM | `cdc_ncm.cc` send loop now bounded (50 × 1 ms, then drop) — was an unbounded `taskYIELD()` spin on tcpip_thread at top priority (1629) |

## Results

### Build 1629 (2026-10-31 00:16)

R1–R9, R11, R12 PASS (manual run of the same checks before the script was
assembled; transcript in the session log).  R10: `init=0`, cam0 5426 B,
cam1 5431 B — captures are black (mean luminance 1.0 vs 77/86 in s247).
Both OV5640s read ID `0x5640`, `0x3008=0x02` (streaming), AEC auto,
exposure `0x03D4` (≈ max) and gain `0x7C` (7.75×): the sensors are live and
pegged at maximum exposure, i.e. **the room was dark at 00:20**, not a
sensor-path regression.  Re-check in daylight.

Task stack high-water marks (words free), build 1629, idle after the run:

| task | prio | HWM free | task | prio | HWM free |
|---|---|---|---|---|---|
| mp_repl | 1 | 3209 | camera_task | 4 | 640 |
| IDLE | 0 | 51 | cdc_ncm_task | 4 | 804 |
| ctrlc | 2 | 231 | usb_device_task | 4 | 738 |
| crazy_rx | 2 | 736 | UsbHostTask | 4 | 786 |
| crazy_cmd | 2 | 262 | pmic_task | 4 | 861 |
| DHCP thread | 3 | 81 | m7_console_task_rx/tx | 3 | 851 / 840 |
| tcpip_thread | 4 | 958 | m4_console_task | 3 | 815 |
| hw_wdog | 3 | 1834 | edgetpu_dfu_task / edgetpu_task | 3 | 189 / 153 |
| Tmr Svc | 4 | 140 | app_main (suspended) | 3 | 2161 |
| btn_usb (susp.) | 1 | 333 | ipc_rx/tx (susp.) | 4 | 889 / 889 |

Tightest: IDLE 51 words (204 B — fine, it runs nothing), DHCP 81 words
(324 B), Tmr Svc 140 words (560 B; the only callback that printed from it
was removed in 1627).  Nothing under 50 words.

### Build 1630 — see bottom of file (appended by the run).

## Other batch-3 changes validated by this run (not separately checked)

- `sentai_dmesg_read()` copies in 256 B slices, each under its own critical
  section (was one 16 KiB memcpy with interrupts masked; the CSI ISR is
  timing-sensitive).  Note: nothing in the tree calls `DMESG_*` yet, the
  ring is empty at runtime — candidate: feed `SERR_LOG` into it.
- `pmic.cc:Transfer()` no longer printf's every I²C transaction; bounded
  retry on busy/arbitration-lost, one line on failure.
- Dead fixed-script MicroPython task removed: −1 MB `.sdram_bss`
  (`.sdram_bss` 14 665 424 B in 1629).
- Watchdog "WARNING ZONE" (6 SD writes per minute of idle) removed; idle
  only logs one snapshot per 10 min.
- `sentai_crazy` semaphores are created once and drained on stop instead
  of deleted/re-created (use-after-free + `g_telem_resp_sem` leak).
- IMU tap_stop cooperative wait 200 ms → 1 s before the force-delete
  fallback (must outlast one LPI2C pin-low timeout).
- `safe_boot_check/clear` report when the card refuses the marker write.

### Build 1630 (2026-10-31 00:24) — `run_build1630.log`

`repl_regression.py`: **R1–R12 all PASS, failures: 0** (R10 cam0 5431 B /
cam1 5426 B, still dark-room black; R12 3× md5 `8036fa17…:5431`).

New finding from the HWM table after the run: **`lfs_task` 49 words free
(196 B of its 2 KB)** after one `/api/ls` + three `/api/raw` — the audit's
M5.  `LFS_TASK_STACK` 512 → 1536 words in build 1631, and
`sentai_fs_task_start()` now reports a failed task/mutex/queue creation.
Also visible: `mp_repl` 464 words free right after the R8 recursion test
(the 3 KB C-side reserve of `mp_stack_set_limit` holds), `tcpip_thread`
647 (was 958 idle).

### Build 1631 (2026-10-31 00:26) — `run_build1631.log`

**R1–R12 all PASS, failures: 0.**  `lfs_task` HWM 49 → 1067 words free
after the same request mix (it uses ≈ 470 words ≈ 1.9 KB: the old 2 KB
stack was one bad path away from overflow).

### Builds 1632 / 1633 (2026-10-31 00:31 / 00:33) — `run_build1632.log`, `run_build1633.log`

Added R13 (`fs.read` single-copy path: 10 240 B round-trip, `/nonexist` →
`OSError`) and R13b (`rtos.dmesg()` non-empty — `sentai_logf`/`SERR_LOG`
now also feed the RAM ring; e.g. `[19064] I E:0A06:3` = camera detect mask).
1632: R13 "failed" only because the check read the last output line and a
bare `sentai.fs.remove(...)` echoes `True` (the embed compiles with
`is_repl=true`); test fixed.  **1633: R1–R13b all PASS, failures: 0.**

Changes validated by 1632/1633 beyond the explicit checks:
- camera frame-holder cap (`frames_held_`, `SERR_CAM_HOLD_LIMIT 0x0A09`):
  hand-out refused when consumers already hold `kFramebufferCount-2`
  buffers, so the CSI keeps two armed; capture (R10) unaffected.
- `UsbDeviceTask::Init()` return now reported (no CHECK — anti-brick).
- `fs.read/read_str` cap 512 KB, `read_base64` 256 KB, one copy instead of
  two.
- ARM build warning-clean in our sources except two pre-existing ones in
  `camera.cc` (`Rgb888ToRgb` unused; missing return at :873 — fixed in 1634).

### Build 1634 (2026-10-31 00:35) — `run_build1634.log`

**R1–R13b all PASS, failures: 0.**  ARM build is now warning-free in our
sources (`VideoConvert` returns its completion flag; `Rgb888ToRgb` marked
unused).

### Builds 1637 → 1640 (2026-10-31 00:45 – 01:05) — `run_build1637..1640.log`

**R1–R13b all PASS on every build.**  What changed underneath (details in
EXP-s250 README):
- 1637: `fs_read_whole()` allocates outside the FS lock; `sentai_fs_lock()`
  reclaims a mutex leaked by the calling task (`SERR_LFX_LOCK_RECLAIMED
  0x0D38`).  Found when the s250 benchmark left the board with every
  `fs.*` call returning `lfs busy`.
- 1638–1640: CDC-NCM TX path — in-flight gate on `tx_buffer_`, ZLP from the
  completion event, hand-off queue 10 → 64 with bounded wait, stale-semaphore
  fix.  `/api/raw` 256 KB: 18.6 s → 49 ms.  R12 (5 KB) never caught this;
  the s250 B5 check (256 KB) now does.
- §14 review of batch 3 (embeded-reviewer, 0 CRITICAL / 3 MAJOR / 10 minor)
  applied: camera hold cap raised to `kFramebufferCount-1` + rate-limited
  log (the `-2` cap fired on `sentai_cam_grab_latest()`'s legitimate
  probe-before-return transient), over-long REPL line now cancels the whole
  block (`-1`, like Ctrl-C) with a 64 KB drain bound, CRLF join is state-
  based (`repl_last_was_cr`) instead of a 5 ms peek only, dmesg_read comment
  corrected, 0x0A09 CSV row moved into the CAM block, embeded.md patch note
  updated.  Open from the review: `patches/micropython/` must be `git add`ed
  with this batch (untracked dir → a fresh clone would lose the fix);
  `vApplicationStackOverflowHook` still calls `SERR_LOG` (now also dmesg)
  on an overflowed stack — pre-existing, candidate for a breadcrumb-only
  path.

### Build 1642 (2026-10-31 07:04) — `run_build1642.log` — OP-S10-W23-T8

**R1–R15 all PASS (18 checks), failures: 0.**  Two checks were added for
this batch:
- **R14 / R14b** — `sentai.fs.stats()` is the new 13-tuple;
  `sentai.camera.stats()` reports `stall_count == 0, stalled == 0` after
  12 s of streaming with no consumer, i.e. the new watchdog liveness
  observer does not false-trigger on an idle-but-live CSI.
- **R15** — slot-FSM contention: 12 rounds of `/api/ls/` and `/api/raw`
  issued concurrently from two curl processes.  12/12 ls bodies parse,
  every raw body has the same MD5 as R12, 3 of 24 requests answered
  `lfs_busy` (by design; the browser retries).  Before 1642 this pattern
  could stream a RAW body from `g_resp_buf` while `lfs_task` overwrote it
  with the next LS (the fast path ran while the slot was PENDING and the
  late `SLOT_READY` relabelled a `SERVING` slot).

What changed underneath (audit T8):
- `sentai_fs_task.cc`: RAW fast path only while the slot is IDLE; the FS
  task publishes READY only for the request it processed (sequence
  number, compare-and-publish in a critical section); counters
  `stale_result`, `busy_fast_raw_pending`.
- Fault path: `sentai_fault_c`, the malloc-failed hook and
  `vApplicationStackOverflowHook` write their record with a no-libc
  formatter into the `.sdram_boot_log` ring (`sentai_fault_text`) instead
  of printf/SERR_LOG from handler mode; it is rescued to
  `/log/boot_prev.log` on the next boot.
- Watchdog camera liveness: `SERR_CAM_STALL 0x0A0A` / `SERR_CAM_STALL_END
  0x0A0B` + `CAM_STALL` crash-log rows, report only.
- `rtos.dmesg()` single copy; `main_freertos_m7.cc` no longer drives
  GPIO10.6 (= USDHC1_DATA1 on this board rev) at boot; T5838 THSEL moved
  to GPIO_AD_18; VDD_1V8_INT_EN comment corrected (held LOW on purpose).
- SIM target links again (`sim/sentai_sim_link_shims.c`), boots to the
  MicroPython banner.

Task stack high-water marks after the run (words free): lfs_task 1073,
hw_wdog 1384, mp_repl 464, IDLE 51, DHCP 81, edgetpu_task 152, Tmr Svc 135.

### Build 1644 (2026-10-31 07:16) — `run_build1644.log`

**R1–R15 all PASS (18 checks), failures: 0.**  Plus manual probes:
`sentai.rtos.http_stats()` → `(28, 0, True)` (the binding existed but was
never registered in the module table); `sentai.fr.open/push_event/stats`
→ `(1, 1, 0, 0, 0, 1, 1)` after one push with the bounded channel lock.

What changed underneath:
- `sentai_fr.cc`: channel lock is a FreeRTOS mutex (priority inheritance)
  taken with a bound — 200 ms for producers/drains, 2 s for the REPL
  configuration calls — instead of a binary semaphore with
  `portMAX_DELAY`.  `sentai_fr_push_debug()` sits under every `printf`,
  so the old wait could park any printing task behind a 76 KB frame
  memcpy or behind `sentai_fr_open()`'s SD I/O (now done before the lock).
  A timed-out producer drops and counts (`SERR_SYS_FR_LOCK_TIMEOUT`
  0x0F10, rate-limited, never from the debug channel to avoid recursion).
- `libs/pmic/pmic.cc`: a PMIC NAK on a rail request or chip-id read no
  longer `CHECK()`s (= immediate reset, reachable from
  `sentai.camera.power_rails`); it prints once and leaves the rail as is.
- `g_network_healthy` now means "no CDC-NCM TX drop during the last
  watchdog interval" (was forced true on every kick).

### Builds 1645 → 1646 (2026-10-31 07:21 – 07:35) — `run_build1645.log`, `run_build1646.log`, `crash_text_probe_1646.log`

**R1–R15 all PASS on both builds (18 checks).**  New: `crash_text_probe.py`
(not part of the automatic run — it resets the board and re-loads the RAM
image): forces a precise BusFault with `sentai.sys.peek32(0x00400000)` and
reads the record back after the reboot.

§14 review of batch 4 (embeded-reviewer): 0 CRITICAL, 2 MAJOR, 9 minor.
- **MAJOR 1 — crash text overwritten before rescue.**  The fault record
  used to be appended to the live boot-log ring; after `boot_log_stop()`
  the next boot's printf re-used that ring from offset 0 before
  `boot_log_fs_init()` copied it out.  Fix (1645): a dedicated
  `CrashTextPersist` slot (1 KB, own magic/check/len/truncated header),
  snapshotted by `boot_log_init()` before the first printf and written to
  `/log/crash_prev.txt` + echoed to the console.
  **Measured on this board (1645): the SDRAM slot does NOT survive** —
  after a reset the board sits in the ROM serial downloader for seconds
  with no SDRAM refresh (BOOT_MODE straps, EXP-s248), so the record
  decayed (`/log/crash_prev.txt` not created; GPR breadcrumb row
  `BUS_FAULT code=0x0FF4 PC=0x80E45054 BFAR=0x00400000` in `crash_013.log`
  proved the handler ran).  1646 moves the slot to DTCM
  (`.noinit.$crash_text`, linker section after the `.stack` placeholder at
  0x2003E910, below `__StackLimit` and above the elf_loader's DTCM use
  that ends at 0x2000A5B0).  Probe result:
  `*** CRASH: code=0x00000FF4 PC=0x80E45054 LR=0x80E4504B CFSR=0x00008200
  HFSR=0 BFAR=0x00400000 | r0..r3 | PRECISERR: precise data bus error @
  BFAR | Stack hint: (none in range)` — **PASS**.
- **MAJOR 2 — ~30 SDRAM stores inside the fault handler could re-fault
  if the fault is the SDRAM path itself.**  Now moot for the slot (DTCM),
  and the handler additionally skips the text when CFSR.BFARVALID and
  BFAR ∈ [0x80000000, 0x8FFFFFFF].  `SRC_SRMR` read on the board = 0
  (all reset modes = system reset), so an M7 lockup still resets;
  documented at the handler.
- Minors applied: slot FSM publishes PENDING before `xQueueSend` and
  rolls back on failure (no longer depends on lfs_task < tcpip_thread
  priority); one `busy_*` counter per `lfs_busy` reply (fast-path skip
  suppresses the Step 4 counter); stale `LFS_QUEUE_DEPTH` rationale
  rewritten; `sentai_fault_text` / `sentai_fault_stack_overflow_record`
  declared in `sentai_fault.h`; camera.init(0) noted as not a
  false-positive source (CSI always streams, EXP-s245).
- Found by the probe, pre-existing: the CFSR decode table in app_main was
  shifted (0x200 labelled INVSTATE, 0x1000 PRECISERR, VECTTBL read from
  CFSR) — every bus fault since v1.0 was mislabelled in `crash_NNN.log`.
  Fixed to ARM DDI0403 bit positions; the same fault now reads
  `[PRECISERR ]` (compare `crash_004.log` vs `crash_013.log`).
- Not done: EMU target (`SENTAI_ARM_EMU`) not built in this batch; the
  watchdog still does one bounded SD write per camera-stall edge (kept:
  once per transition, inside the 30 s window).

DTCM budget after 1646: `.bss` ends 0x2003E504, `.stack` placeholder to
0x2003E908, crash slot 0x2003E910–0x2003ED10, `__StackLimit` 0x2003FBFC
(≈ 3.8 KB free between the slot and the live MSP).

### Build 1647 (2026-10-31 07:41) — `run_build1647.log`

**R1–R15 all PASS (18 checks), failures: 0.**  Cleanup-only batch from the
SD-migration review minors: 19 `[SDCard] ... STEP n` boot prints removed
(boot.log now carries two SD lines: host clock and init result), stale
NAND/LevelX wording in `fx_user_fs.cc` rewritten for the SD backend
(auto-sync rationale, blank-card auto-format rule, `FxUserSync` cost
≈ 0–5 ms per EXP-s250).  No functional change.

### Build 1649 on the reference board (2026-10-31 08:31) — `run_build1649_refboard.log`

Not a regression run of the firmware: build 1649 (board-id in
`sentai.version()` / USB product string, `sentai.sys.uid()`) was RAM-loaded
on the operator's flash-booting reference board for EXP-s252.  R1–R9 and
R14 PASS; R10–R13 FAIL because that board has no microSD, no camera on
LPI2C1 and no LIS2DUX12 (it is most likely the previous hardware revision).
R1 shows the useful bit: `SBMR1=0x4C0 SBMR2=0x02000009` — BOOT_MODE straps
read `10` there.  See `../s252_reference_board_fuses/README.md`.

### Builds 1650–1651 (2026-10-31, EXP-s251 instrumentation) — `run_build1651.log`

**R1–R15 all PASS on 1651.**  1650 adds `sentai.camera.peek_col()`; 1651 the
FB-done frame log in the CSI frame-complete hook (`.ramfunc` 0x208 → 0x278,
camera fps unchanged at 44.4–44.8 in s251).  One unexplained M7 LOCKUP
reset occurred during s251 iter3 (not reproduced in six later runs).
