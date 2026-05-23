---
name: anti-brick-check
description: Audit firmware change for anti-brick rule compliance per embeded.md §10. Use before committing changes to boot path, USB init, WDOG, SRC_GPR, or any code that runs pre-scheduler.
---

# /anti-brick-check

Per `embeded.md` §10 — **ABSOLUTE RULE: NEVER brick the board.  There must ALWAYS be a way to reflash without a button press.  Requiring user intervention to recover = SOFTWARE FAILURE.**

## Triggers (run this skill if you touched any of these)

- `libs/base/main_freertos_m7.cc` (boot path)
- `apps/elf_loader/` (bootstrap)
- Anything calling `BOARD_InitClocks`, `SDRAM_Init`, `USB_DeviceInit`
- Watchdog config (`WDOG1_*`)
- Boot counter / SRC_GPR persistence
- Any code that runs BEFORE `vTaskStartScheduler()` returns

## Board states (RT1176)

| USB ID | State | Recovery | Severity |
|---|---|---|---|
| NXP `1fc9:c0a1` | firmware running OR WDOG resets after hang | Automatic via `flashtool.py` | ✅ OK |
| Google Coral `18d1:9307` | ROM bootloader; firmware never started | Manual USER button + SDP | ❌ CRITICAL |
| Not visible | HW failure | JTAG | ☠ CATASTROPHIC |

## Checks

1. **USB CDC init BEFORE any code that could crash:**
   ```bash
   grep -n 'USB_DeviceInit\|cdc_acm_init' /home/bogdan/work/coralmicro/libs/base/main_freertos_m7.cc
   ```
   USB init must appear before any non-trivial driver init that could fault.

2. **No `vTaskDelay` pre-scheduler:**
   ```bash
   grep -rn 'vTaskDelay' /home/bogdan/work/coralmicro/libs/base/ /home/bogdan/work/coralmicro/apps/elf_loader/ /home/bogdan/work/coralmicro/examples/sentai_runtime/sentai_runtime.cc
   ```
   Any call inside `BOARD_*`, `Init*`, or before `vTaskStartScheduler()` → `pxCurrentTCB` null-deref → BRICK.  Use `bounded_delay_ms` or polled spin.

3. **WDOG1 configured ≤ 30 s on 32 kHz independent clock:**
   Inspect `WDOG1_Init` parameters.  Must be on the 32 kHz oscillator (not the CPU clock — CPU hang = WDOG hang).

4. **Boot counter in SRC_GPR:**
   Persists across warm reset.  3 failed boots → RECOVERY MODE (USB + REPL only).

5. **SAFE MODE keeps USB alive:**
   Every error path that goes into a SAFE MODE branch must NOT disable the USB CDC interface.  Search for `USB_Deinit`, `cdc_acm_stop`, `USBPHY1->CTRL_SET` in error/fault handlers.

## Recovery flow expected

- Crash AFTER USB init → WDOG resets in 30 s → NXP `1fc9:c0a1` stays visible → `flashtool.py` works (no button).
- Crash BEFORE USB init → `18d1:9307` → BRICKED → **DO NOT SHIP CODE THAT CAN REACH THIS STATE.**

## Test locally before flashing persistent

Always test new boot code via `--ram` flash first:
```bash
python3 scripts/flashtool.py -e sentai_runtime --ram
```
RAM flash is lost on power cycle, so even a complete brick falls back to whatever the persistent flash has.  Only run persistent flash once RAM mode confirms clean enumeration.

## Reject patterns

- "It's only one delay in init, should be fine" — null-deref is instant brick.
- Disabling WDOG temporarily for debugging without a comment + ticket.
- Adding a new code path BEFORE USB init.
- Skipping the RAM-flash dry-run on boot-path changes.

## See also

- `embeded.md` §10 (anti-brick — absolute rule)
- `agent.md` §2 (rule 1) + §4 (unwedge recipes)
- CLAUDE.md "Working with the board (live HW)"
