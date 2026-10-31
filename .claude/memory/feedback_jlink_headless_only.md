---
name: jlink-headless-only
description: Never launch SEGGER J-Link tools with a GUI — operator cannot click dialogs (license/firmware-update popups hang). Always JLinkExe with -nogui 1 + DisableAutoUpdateFW + SuppressControlPanel and a real command file.
metadata:
  node_type: memory
  type: feedback
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-30T19:17:56.042Z
---

Operator-stated 2026-10-30: "sa nu cumva sa il pornesti cu GUI pentru ca nu
pot apasa pe butoane de GUI … SEGGER afiseaza o fereastra in care accepti o
licenta … si se blocheaza".

**Why:** the session runs headless from the operator's point of view; any
modal (license acceptance, firmware-update prompt, control panel) blocks the
tool forever and looks like a hang.

**Observed 2026-10-30:** even with `-nogui 1` + the two `exec` lines, a
SEGGER dialog still popped up on the operator's desktop on the first run
(the probe reports firmware "J-Link EDU Mini V1" although `lsusb` says
J-Link PLUS → likely the EDU terms-of-use prompt).  Operator happened to be
at the screen and clicked Accept.  Always warn the operator BEFORE the first
JLinkExe call of a session that a dialog may appear, and never run J-Link
when they are away.  DLL V9.54.

**How to apply:**
- `JLinkExe -nogui 1 -device MIMXRT1176xxxA_M7 -if SWD -speed 4000
  -autoconnect 1 -CommandFile /path/real_file.jlink` — process substitution
  for the command file FAILS (see [[reference_board_usb_wedge_needs_physical_por]]);
  write a real temp file in the scratchpad.
- First lines of every command file: `exec DisableAutoUpdateFW` and
  `exec SuppressControlPanel`.
- Never run `JLinkGDBServer`, `JFlash`, `JLinkSWOViewer` GUIs or Ozone.
- J-Link PLUS shows as `1366:0101` in `lsusb`; it is attached to the new
  board rev (see [[new-board-rev-2026-10-open-items]]).  Skill: `jtag-debug`.
