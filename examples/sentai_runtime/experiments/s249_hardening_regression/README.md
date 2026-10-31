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
