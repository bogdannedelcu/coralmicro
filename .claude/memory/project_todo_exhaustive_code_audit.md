---
name: todo-exhaustive-code-audit
description: "Operator request 2026-10-30 — schedule an exhaustive audit of the SentAI firmware source for race conditions, hangs, unbounded waits and optimisation opportunities; WBS OP-S10-W23 (tasks T1-T8)."
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-30T21:41:44.295Z
---

Operator-stated 2026-10-30 (end of the SD-card / new-board bring-up day):
(paraphrased) put an exhaustive audit of the source code on the TODO
list: there is room for improvements and optimisation, so that we fix
every potential race condition, hang, etc.

**Status: IN PROGRESS, batch 3 = builds 1629-1640 (2026-10-31); regression =
EXP-s249 `repl_regression.py` (15 checks, 0 failures on 1640) + EXP-s250
SD/HTTP benchmark (read 16 MB/s, write 1.7 MB/s, HTTP 5.2 MB/s) (operator, paraphrased: keep improving and refactoring until we have
a stable release).**  WBS: `OP-S10-W23` (appended to ideas/wbs.md 2026-10-31, tasks T1-T8;
T8 = remaining items).  Board-rev work is `OP-S10-W22`.

**Done so far (build 1623, RAM-loaded):**
- Watchdog idle-reset removed: `CombinedWatchdogTask` keeps kicking WDOG1
  on idle; "WDG_DEAD"/forced `NVIC_SystemReset` gone; long-idle only logs a
  task snapshot every 10 min (`WDG_IDLE`).  Board must now survive
  unattended on a RAM image.
- Baseline (`sentai.rtos.heap_info/tasks`): MP GC heap 1 032 384 B
  (560 B used at idle), RTOS/newlib heap 14.5 MB free; low stack
  high-water marks (words): IDLE 51, DHCP 81, ctrlc 101, crazy_cmd 264,
  btn_usb 333.  `print(sentai.rtos.tasks())` (~1.2 KB) is truncated by
  the console TX path — REPL output drop starts well below 6 KB.
- Three embeded-reviewer audits launched in parallel (camera/pipeline;
  base/boot/USB/health; MicroPython/FS-task/httpd) — results feed the
  fix list.

**Batch 3 done (builds 1629-1631):** dmesg chunked read (256 B critical
sections), rtos.suspend protected list (+tcpip/lfs/fs/wdog/sd), cdc_ncm
bounded send (50x1 ms then drop; was unbounded taskYIELD spin at top prio),
pmic per-transfer printf removed + bounded retry, dead 1 MB MP gc_heap
removed, watchdog WARNING-zone SD writes removed, REPL >1023-char line
drained+rejected, safe_boot marker write checked, crazy semaphores
persistent (no delete/recreate, telem sem leak fixed), IMU stop wait 1 s,
lfs_task stack 512->1536 words (s249 measured 49 words free),
camera frame-holder cap (SERR_CAM_HOLD_LIMIT 0x0A09), UsbDeviceTask::Init
return logged, fs.read single-copy + 512 KB cap, SERR_LOG/sentai_logf now
also feed the dmesg ring (was empty forever).  FS lock leak on MemoryError
under lock -> allocate outside lock + owner-reclaim (0x0D38).  CDC-NCM TX
path: tx_buffer_ overwritten while in flight, 10-deep queue vs 16-segment
bursts, stale-semaphore drop -> 256 KB HTTP 18.6 s -> 49 ms (s250).
§14 review of batch 3 applied (hold cap -1 + rate limit, REPL cancel on
over-long line, state-based CRLF).  `patches/micropython/` is UNTRACKED --
must be git-added with the batch.

**Still open from the three reports:** camera M5
(camera liveness in watchdog), M6/M8 (g_cam_current_id single writer), M9
(HWM evidence -> now in s249 README); base M1 (fault hooks printf), M4
(ConsoleM7 heap per printf), M8 (1V8_INT
polarity), M10 (health state silent); MP M4 (fs_task slot FSM), M9 (mic
malloc), M12 (FR lock), M13, M14 (fs_cache sync),
m5 (dmesg 2x copy in rtos.dmesg binding), m8/m9.
SIM link shims still missing.

**Scope to propose:** `libs/base`, `libs/camera`, `libs/sdcard`,
`libs/msc_ums`, `libs/pmic`, `examples/sentai_runtime/*.cc|*.c`,
`bindings/modsentai_*.c`.  Method: embeded.md §14 checklist per module
with the `embeded-reviewer` agent, `fault-model` skill for the subsystems
that never had one (camera task, USB storage mode, health/watchdog),
`watchdog-estimate` for every long driver path.

**Seeds found on 2026-10-30 (so the audit does not rediscover them):**
- `CameraTask::Detect()` false positive: `Read()` fills one buffer from
  both I2C buses and `Detect()` checks the value, not the status
  (`libs/camera/camera.cc`); `CAMERA_DEVICE_Init` aborts on the first
  failed write with no indication which bus failed (tracing now added).
- **CONFIRMED (read the code):** `CombinedWatchdogTask`
  (`sentai_runtime.cc` ~:600-660) treats *no REPL/HTTP activity for
  120 s* as "dead" and stops kicking WDOG1 (30 s) → the board resets
  while perfectly healthy but idle.  Explains every "board found in SDP
  after a pause" today (RAM-loaded image lost on each reset).  Audit
  item: distinguish idle from unresponsive (e.g. REPL task heartbeat /
  self-ping) before relying on this on a flash-booting board.
- ~~REPL input corruption on long lines~~ FIXED build 1628 (2026-10-31):
  was MicroPython `objint_longlong.c` `strtoll` on a non-NUL-terminated
  lexer vstr → `patches/micropython/0001-*.patch` (also fixes
  `int("-big")` sign and silent overflow).  CRLF paste ending blocks →
  fixed in `repl_readline` (swallow `\n` after `\r`).  REPL output
  truncation > ~1 KB → fixed build 1627 (`_write` 256 B slices).
- Pre-existing unrelated-to-FS SIM link errors (`sentai_tpu_set_mode`,
  `sentai_tpu_recover`, `vPortGetRunTimeCounterValue`,
  `sentai_pin_probe_cam_trigger`, `g_sentai_yuv_dual`) — SIM shims
  missing since the July–September ARM work.
- `main_freertos_m7.cc` writes 0 to `VDD_1V8_INT_EN` (GPIO9.20) while the
  comment says "Enable" — polarity vs the new TCR2LN18 LDO (CONTROL
  active-high) powering mic + IMU must be checked.
- Remaining reviewer minors on the SD migration (see
  [[fs-moves-to-sdcard-decision]]): stale NAND/LevelX comments in
  `fx_user_fs.cc`, `[SDCard] STEP n` boot printf noise, mount state
  spread over four booleans, late-mount-after-deadline not logged.
