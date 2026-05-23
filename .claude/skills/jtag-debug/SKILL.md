---
name: jtag-debug
description: JTAG/SWD bring-up via Segger J-Link PLUS. Use as last-resort recovery when USB CDC is dead and the board is not in SDP mode (i.e. it is BRICKED), OR for live debug of HardFaults / ISR hangs that don't show in boot.log. Includes M7 + M4 attach recipes.
---

# /jtag-debug

The dev board exposes the SWD pins of the M7.  A **Segger J-Link PLUS** is wired in (host shows `lsusb` → `1366:0101`).  JTAG is the escape hatch out of states that anti-brick (§10) shouldn't have allowed but reality occasionally produces.

## When to use JTAG

- **Bricked board**: USB CDC dead (no `/dev/ttyACM0`, no NXP `1fc9:c0a1`) AND not in SDP either (`1fc9:013d`).  Power cycle + USER button doesn't help.  JTAG SYSRESETREQ is the next step.
- **HardFault / WDOG-reset path** that `boot_prev.log` can't pinpoint (ISR-side hangs, pre-scheduler `CHECK` failures, null-deref before USB init).
- **Live state inspection** without REPL: CSI/PXP registers, OCRAM contents, `flow_shared_t` cross-core, DTC-RAM boot persistence area.

## Quick reset via J-Link (no GDB)

Fastest unwedge when firmware in flash is fine and you just need to kick the board:

```bash
JLinkExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -autoconnect 1 \
  -CommandFile <(printf 'r\nh\nrx 100\ng\nq\n')
```

- `r` = reset (SYSRESETREQ via DAP)
- `h` = halt
- `rx 100` = wait 100 ms
- `g` = go
- `q` = quit

After this, `lsusb | grep 1fc9:c0a1` should show the board enumerated.  Then `flashtool.py` works again.

## Live debug session (GDB + JLinkGDBServer)

**Terminal A — start the GDB server:**
```bash
JLinkGDBServerCLExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -port 2331
```

**Terminal B — attach GDB to the running ELF:**
```bash
arm-none-eabi-gdb /home/bogdan/work/coralmicro/build/examples/sentai_runtime/sentai_runtime \
    -ex "target remote :2331" \
    -ex "monitor reset" -ex "monitor halt"
```

From the GDB prompt:
- `bt` — backtrace
- `info reg` — CPU registers (look for SHCSR / CFSR for fault diagnosis)
- `x/16xw 0x202C1000` — flow_shared cross-core area
- `x/16xw 0x20240000` — DTC-RAM BootPersist (boot counter, log magic, prev log)
- `x/64bx 0x40428000` — CSI register block
- `monitor reset` — full board reset from GDB
- `continue` then Ctrl-C — break in arbitrary task to inspect live FreeRTOS state

## M4 attach

Same J-Link, swap the `-device` flag:
```bash
JLinkGDBServerCLExe -device MIMXRT1176xxxA_M4 -if SWD -speed 4000 -port 2332
```

The M4 has no `BOARD_InitBootClocks` in our build (intentional — see flow.md §3.2), so PLL state seen via JTAG is the M7-configured state.  Useful for the W16 multi-core investigation (SysTick / freeze under load).

## Rules of engagement

- **Do NOT `monitor flash` from GDB.**  Use `flashtool.py` for actual reprogramming — the build artifact is a packaged image (sb file + LevelX header) that `flashtool.py` assembles; raw ELF flash via J-Link bricks the FileX volume.
- **JTAG ≠ free pass to skip anti-brick (§10).**  If firmware bricks USB CDC, JTAG can recover it but every other developer on the project sees a brick.  Treat JTAG as recovery + debug, not as a license to ship code that crashes USB bring-up.
- **JLinkExe needs the user to be in `plugdev` / udev rule allowing `1366:0101`.**  If `JLinkExe` reports "USB error" check `lsusb` first; if the J-Link itself isn't visible, the JTAG hardware setup is the issue, not the firmware.

## Bricked-board playbook

When `lsusb` shows neither `1fc9:c0a1` (NXP firmware) nor `1fc9:013d` (NXP SDP / ROM):

1. **JTAG-reset attempt:**
   ```bash
   JLinkExe -device MIMXRT1176xxxA_M7 -if SWD -speed 4000 -autoconnect 1 \
     -CommandFile <(printf 'r\nh\ng\nq\n')
   ```
2. If `lsusb` now shows `1fc9:013d` (SDP / ROM bootloader) → `flashtool.py -e sentai_runtime` recovers via SDP.
3. If `lsusb` shows `1fc9:c0a1` (firmware up) → REPL on `/dev/ttyACM0` should respond.
4. If `lsusb` shows NEITHER even post-JTAG-reset → the board may be physically damaged (PMIC, USB PHY).  Investigate hardware before further software attempts.

## Reject patterns

- Using JTAG to "fix" a brick instead of root-causing why the firmware bricked USB.
- Single-stepping through ISR code with `step` (J-Link can lose sync with very-high-rate IRQs like CSI).  Use breakpoints + `continue` instead.
- Trusting `bt` after a HardFault if `LR` was corrupted by the fault — read `CFSR` (0xE000ED28) to find the actual fault class first.

## See also

- `agent.md` §4.1 "JTAG / SWD via Segger J-Link"
- `embeded.md` §10 anti-brick (the rule JTAG saves you from)
- `[[arm-flash]]` skill (the preferred recovery path — try `--ram` first, JTAG when even that fails)
