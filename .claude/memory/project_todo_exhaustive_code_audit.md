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

**Batch 4 done (build 1642, 2026-10-31, s249 18/18):** fs_task slot FSM
(fast path only when IDLE, compare-and-publish with request seq), fault
hooks write a no-libc record into the .sdram_boot_log ring instead of
printf (rescued to /log/boot_prev.log), watchdog camera liveness observer
(SERR_CAM_STALL 0x0A0A / END 0x0A0B, report only; sentai.camera.stats()
stall_count/stalled), sentai.fs.stats() 13-tuple, rtos.dmesg single copy,
THRS GPIO10.6 (= USDHC1_DATA1!) removed from boot, T5838 THSEL -> GPIO_AD_18,
VDD_1V8_INT_EN documented as intentionally LOW (PMIC feeds the rail via R13),
SIM links again (sim/sentai_sim_link_shims.c).

**Batch 4b (build 1644, s249 18/18):** sentai.fr channel lock = mutex with
bounded take (200 ms push / 2 s config, SERR 0x0F10 on timeout, SD I/O moved
out of sentai_fr_open's lock); PMIC rail/chip-id CHECK() -> report + skip;
g_network_healthy = "no NCM TX drops in the last kick interval";
rtos.http_stats() registered; SIM links (shims for http stats too).

**Batch 4c (builds 1645-1646, s249 18/18 + crash_text_probe PASS):** §14
review of batch 4 applied.  KEY LESSON: on this board SDRAM NOLOAD data does
NOT survive a reset (ROM SDP wait, no refresh) -> the crash-text slot lives
in DTCM `.noinit.$crash_text` (ld section after `.stack`, 0x2003E910); the
`.sdram_boot_log` boot_prev.log rescue only works on a flash-booting board.
Fault handler skips text when BFAR is in SDRAM; SRC_SRMR=0 (lockup = system
reset).  CFSR decode table in app_main was shifted since v1.0 (fixed).
fs_task FSM publish-before-send + single busy counter per reply.

**Still open from the three reports (after batch 4c):** camera M5
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

**T9 (approved 2026-10-31) — camera alternation frame identity:** see
`ideas/wbs.md` OP-S10-W23-T9.  Key facts for the next session: both OV5640s
share one 24 MHz XCLK (frequency-locked, phase random per boot, no FSIN pin);
BARS/HBAND oracle is roll-blind (use 0x84/0x8C vertical gradient); task tag
comes from a scalar (copy `g_cam_buf_id[idx]` at dequeue instead); arm-time
tag cannot work (BASEADDR_SWITCH fills the armed buffer at k+2).  Plan =
EXP-s251 (oracle, timestamps, software genlock via one-frame VTS tweak).

**T9 RESULT (EXP-s251, 2026-10-31):** no roll/seam exists; the two OV5640s are
frequency-locked (shared XCLK) with a per-boot phase offset (found +411 us).
A MUX flip is clean only if the new sensor's frame start lies in the old one's
VBLANK (543 us at VGA45) -> margin 132 us -> 1 in 8 frames mislabelled.
Software genlock = one-frame TIMING_VTS+d write shifts a sensor by d lines
exactly; after phi -> 4 us: 1:1 at 44.7 fps, 0 mixed, 0 mislabelled.  Phase
probe MUST consume frames (frame-complete hook only fires when a frame is
queued; log freezes otherwise).  First grab after >=1 s idle is a stale
buffer (0.5-1 s old).  MicroPython: ints > 2^30 are long-long and cannot mix
with floats.  One unexplained M7 LOCKUP reset during iter3.

**T9 FINAL (build 1664, 2026-10-31):** see [[csi-mux-flip-deferred-f16]] --
the wedge below (iter9) was the F16 flip race, fixed by the deferred flip;
genlock_auto default = pre-trim (mode 1); cam_tune closed loop shipped.

**T9 SHIPPED in C (build 1657, 2026-10-31):** `sentai_cam_genlock()` runs at
`camera.init()` (phase from the FB-done log while alternating, one-frame
VTS+d trim SYNCHRONISED to the trimmed sensor's frame edge: write 5 ms after
FB-done, restore 1.25 T later -- an unsynchronised pair shifts by d OR 2d and
oscillates, iter8).  Converges in 1 step (|phi| <= 10 us) on 6/7 cold boots;
on success drain=0/dirty=0 -> ratio(1,1) at 44.7 fps.  Stale guard
(g_cam_buf_ts_us age > 2 T) and watchdog phase monitor (SERR 0x0A0E) shipped
too.  One cold boot (iter9) had the camera wedge right at init before any
flip (genlock -30, then stall) -- not attributed yet.  SDK patch 0005 = F15.

**F15 (2026-10-31, caught by the crash-text path):** fsl_csi.c keeps the
empty-buffer linked list inside the frame buffers; DMA into a still-linked
buffer -> `CSI_TransferGetEmptyBuffer` dereferences a pixel (BFAR 0x00604F90)
in the CSI ISR.  Seen once during the s251 sensor standby/restart test
(up=3534 s); the iter3 LOCKUP is probably the same bug.  Proposal: SDK patch
0005 (side-array list + bounds check + SERR) and the stale-queue guard.
