---
name: reference_board_usb_wedge_needs_physical_por
description: A TPU-USB wedge can crash the M7 into a boot-ROM spin; JTAG/uhubctl CANNOT recover it — only a physical POR (unplug/replug) does. Chip+flash stay fine.
metadata:
  type: reference
---

2026-06-26: enabling `desc_cache(1)` (see
[[reference_edgetpu_instruction_caching_256kb]]) wedged a TPU invoke; subsequent
JTAG resets left the board crashed in the **boot ROM**, USB not enumerating at
all. Full diagnosis + what does/doesn't recover it:

**Diagnosis via J-Link (board has SWD, J-Link at 1366:0101):**
- Chip is ALIVE (`Cortex-M7 identified`), flash image intact (`Valid application
  detected`).
- After reset+go it spins at PC=0x00223104 with instruction `E7FE` = `b .`
  (infinite loop). 0x00200000 is the RT1176 **boot ROM** region -> the ROM fails
  to boot from FlexSPI flash and/or fell to SDP but the USB device PHY is wedged
  so SDP (1fc9:013d) never enumerates.
- boot-persist struct (DTCM, symbol `__BOOT_PERSIST_START__` at 0x200082EC, 20
  bytes = magic/check/attempts/progress/prev_progress) reads all-zero -> NOT a
  storage-mode latch; default boot mode. SENTAI_STORAGE_MAGIC=0x57500001 would
  mean MSC/storage mode (no REPL) — that path is in `main_freertos_m7.cc`.

**What does NOT recover it (all tried, all failed):**
- JTAG SYSRESETREQ (`r;g`), JTAG reset-pin (`RSetType 2`), 6x reset loop —
  SYSRESETREQ does not reset the USB PHY / FlexSPI / PMIC domains, so the bad
  volatile state survives and the ROM re-crashes every time.
- `flashtool.py` — needs the board enumerated as 1fc9:c0a1 (firmware) or
  1fc9:013d (SDP) to even start; with no enumeration it says "No Dev Board Micro
  devices detected".
- 1200-baud touch — needs an existing /dev/ttyACM CDC port; none exists.
- `uhubctl` POR — only the `1-9` Genesys hub (05e3:0608) supports ppps on this
  host, and the board's USB-C is plugged into a ROOT xHCI port (no ppps). The
  board was fully absent from lsusb. (1a6e:089a on 1-7 is the operator's separate
  Coral USB Accelerator stick, unrelated.)

**The "reset GPIO" (PMIC_ON_REQ / GPIO13_IO01) — powers OFF but cannot power
back ON remotely:** the board's PMIC enable is `PMIC_ON_REQ` pad =
`IOMUXC_PMIC_ON_REQ_DIG_GPIO13_IO01` (mux reg 0x40C94004 = ALT5, GPIO13 @
0x40CA0000, pin 1; 1=on, 0=off; see commented `enable_pmic_en()` in
`libs/pmic/pmic.cc`). Driving it low via JTAG (set GPIO13 GDIR bit1=1, DR bit1=0,
then write mux=0x5) cleanly powers the WHOLE board down — confirmed 2026-06-26:
after the writes J-Link reports "Target voltage too low" (the SNVS/SWD domain
also lost power). This is a TRUE power-down (clears all volatile state), BUT this
board has NO hardware auto-restart on PMIC_ON_REQ, so it stays off; powering back
up needs a VBUS re-cycle (replug or a ppps hub) — JTAG can't re-assert because the
core is now unpowered. Net: the reset GPIO can force a clean off-state but is NOT
a remote recovery on a non-ppps root port.

**What DOES recover it:** a physical power-on reset (unplug/replug the board's
USB-C, or its power button). A POR clears the USB PHY / FlexSPI / PMIC volatile
state; the ROM then boots flash 1560 -> firmware -> 1fc9:c0a1 -> REPL. **No
reflash needed** — the flash image is intact.

**Future-proofing for remote recovery:** plug the board's USB-C into the `1-9`
ppps hub so a wedge can be POR'd remotely with
`sudo uhubctl -a cycle -l 1-9 -p <port>` (passwordless sudo works on this host).

**Tooling note:** JLinkExe needs `exec DisableAutoUpdateFW` + `exec
SuppressControlPanel`; process-substitution `-CommandFile <(...)` FAILS ("Failed
to read J-Link Command File") — write a real temp .jlink file. flashtool needs
`venv-flash/bin/python3` (has progress+pyusb+hid; the other venvs lack a python
binary).
