---
name: USB Mode-Switch Architecture
description: SentAI board USB mode-switch (default REPL+IP vs storage MSC) via DTC-RAM noinit flag + warm reset; LfsUser() init guard fix; rationale for abandoning "all 3 classes simultaneously"
type: project
originSessionId: 93909a81-a37a-4af3-92c6-afc40ae13899
---
The SentAI firmware does NOT run CDC-ACM + CDC-NCM + MSC simultaneously. That
was tried and abandoned: Linux's `usb-storage` and `cdc_ncm` race during
enumeration (bulk-IN STALL → BOT-reset → bus-reset cascade at T+1 s) and the
board becomes unstable. Shipping design is a **mode switch** via
`NVIC_SystemReset` with a persistence flag in DTC-RAM.

**Why:** User pivoted from "all 3 simultaneously" to "mode switch via reboot"
after repeated stability failures, explicitly preferring cascade-free boots
over a unified descriptor.

**How to apply:** When touching the USB composite descriptor, modifying
`sentai_usb_drive_set`, or seeing references to "all 3 interfaces" in docs /
comments, know that:

- Default mode = CDC-ACM + CDC-NCM (REPL + IP). No MSC → no cascade.
- Storage mode = CDC-ACM (anti-brick only, REPL off) + MSC.
- Switch via `drive(1)` / `drive(0)` (warm reset) or user button / reset button.
- Any byte received on `/dev/ttyACM0` in storage mode triggers `drive(0)` →
  default mode (host-side exit without physical access).
- Persistence is DTC-RAM `.noinit_boot_persist` struct (NOT SRC_GPRs):
  `{ magic=0x57500001, check=~magic, attempts, progress, prev_progress }`.
  Warm reset preserves DTC-RAM; hardware POR clears it.
- `LfsUser()` in `libs/base/filesystem.cc` now returns `nullptr` until
  `LfsUserInit` successfully mounts — storage mode deliberately skips
  `LfsUserInit` (host owns the NAND), and earlier code crashed when
  `boot_log_fs_init` called `lfs_mkdir` on an uninitialised volume.
- `ConsoleM7`'s CDC-ACM is the anti-brick anchor and MUST stay registered in
  both modes so `flashtool.py` can always reach the board.
- Crash-loop guard: 3 consecutive storage-mode boots that don't reach
  `sentai_storage_boot_succeeded()` → wipe magic, force default mode. No
  physical access needed to recover from a buggy storage-mode init.

**Empirical findings kept because they are NOT obvious from the code:**

- `NVIC_SystemReset` on RT1176 preserves DTC-RAM at `0x20000000` but
  empirically does NOT preserve `SRC_GPR9..12, 15, 16` (boot.log confirmed
  all six read as zero after warm reset, contrary to the reference manual).
  `SRC_GPR1, 13, 14` do survive — they are used elsewhere for boot-attempt
  and watchdog/lockup counters.
- CPU lockup reset (reset reason `0x10000004` = M7 LOCKUP + M4 reset by M7
  LOCKUP) clears DTC-RAM on this silicon. This means crashes during
  storage-mode init would wipe the magic on recovery, so the crash-loop
  guard's persistent counter must survive via the same DTC-RAM mechanism;
  if `attempts` reaches `kMaxStorageAttempts` the logic ALSO wipes magic
  and forces default, giving a clean abort path rather than a brick.
- Full file-preservation across storage-mode cycle was verified on
  build #582 (2026-04-19): a 30-byte file written to `/test_marker.txt`
  via `sentai.fs.write` survived `drive(1) → /dev/sda visible → 'q' exit
  → sentai.fs.read` and came back byte-identical. Three back-to-back
  cycles all clean. User exhaustive test passed.
