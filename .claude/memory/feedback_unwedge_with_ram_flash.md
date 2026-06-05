---
name: Unwedge a stuck board with --ram flash
description: When the board is wedged (REPL silent), reflash with `flashtool.py --ram` to reset it without burning the persistent ELF. Reserve normal `flashtool.py -e sentai_runtime` for when the firmware itself needs to be persisted.
type: feedback
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
When the board is wedged (REPL silent on `/dev/ttyACM0`, `lsusb` shows NXP id but no commands echo), the fastest recovery is:

```bash
python3 scripts/flashtool.py -e sentai_runtime --ram
```

`--ram` flashes to RAM only — the board reboots into the new image without rewriting persistent flash.  Functionally equivalent to a hard reset but FAR faster than waiting ~3 min for the WDOG software dead-threshold to fire.

**Why:** *Why:* the user said this directly 2026-04-26 — `--ram` is the canonical "kick the board" recipe.  Don't sit through 120 s of REPL-silence + 30 s WDOG when one flashtool call would have unblocked everything.

**How to apply:**
- **Default unwedge action**: `flashtool.py --ram` whenever REPL is silent and probes return empty.
- **Use full persistent flash (`-e sentai_runtime` without `--ram`)** only when:
  - You actually want the new firmware to survive a power cycle / next boot.
  - Verifying field-deployment behaviour.
- After a `--ram` flash, the same ELF the board had before the wedge is what runs again — so this is a pure "reset" not a "rebuild".  If you want a *new* build to test, build first, then `--ram` flash to test ephemeral, then `-e sentai_runtime` (no `--ram`) once it's confirmed.

This obsoletes the old "wait for WDOG ~3 min" pattern in agent.md.  Update agent.md §4 / §5.2 to lead with `--ram` flash.
