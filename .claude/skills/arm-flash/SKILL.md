---
name: arm-flash
description: Build, flash, and verify ARM sentai_runtime firmware. Verifies build counter incremented to catch stale flash. Use after firmware C/C++ changes. Codifies anti-brick rule from embeded.md §10.
---

# /arm-flash

## Steps

1. **Read expected build counter** from prior `sentai.version()` output or git log message.

2. **Build:**
   ```bash
   cd /home/bogdan/work/coralmicro
   bash build.sh
   ```
   For incremental: `cmake --build build --target sentai_runtime`.

3. **Verify `.ramfunc` section intact** (catches ISR-code-not-in-ITCM regressions per `embeded.md` §10 + CLAUDE.md "Architecture map"):
   ```bash
   arm-none-eabi-objdump -h build/examples/sentai_runtime/sentai_runtime.elf | grep -E '\.(ramfunc|itcm)'
   ```
   Section sizes and presence should match the prior build.

4. **Flash:**
   - Persistent (survives power cycle): `python3 scripts/flashtool.py -e sentai_runtime`
   - RAM-only (fast unwedge, lost on power cycle, lost on `sys.reset()`): add `--ram`

5. **Verify board enumerates as NXP `1fc9:c0a1`** (NOT `18d1:9307` = Google Coral = ROM bootloader = BRICKED):
   ```bash
   lsusb | grep -E '(1fc9|18d1)'
   ```

6. **Verify build counter via REPL** (operator typically drives REPL):
   ```
   >>> import sentai
   >>> sentai.version()
   SentAI v1.0 build <N> (...)
   ```
   `<N>` must be PRIOR + 1 (or more if multi-build).  MISMATCH = stale flash; re-flash.

## Anti-brick reminders (embeded.md §10)

- If `lsusb` shows `18d1:9307`, board is in ROM bootloader.  Recovery needs USER button + SDP — DO NOT ship code that gets here.
- Crash AFTER USB init → WDOG resets in 30 s → NXP ID stays → `flashtool.py` works.
- Crash BEFORE USB init → `18d1:9307` → bricked.
- `--ram` flash is for fast iteration; on power cycle or NVIC reset the persistent flash runs again.

## Reject patterns

- Skipping the build-counter verification — stale flash silently means you're testing the prior firmware.
- Flashing without first verifying `.ramfunc` symbols still resolve.
- Using `--no-verify` on commit hooks to bypass build.

## See also

- `embeded.md` §10 (anti-brick)
- `agent.md` §2 + §4 (unwedge recipes)
- CLAUDE.md "Build & flash"
