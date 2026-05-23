---
name: error-code-add
description: Add a new error code to the registry, finding the next free number and refusing to reuse retired codes. Codifies embeded.md §7.1 append-only rule. Use when defining a new SERR_*, DMESG_E_*, or anomaly code.
---

# /error-code-add

Per `embeded.md` §7.1 — error codes are append-only.  **NEVER deleted or renumbered** — old binaries reference old codes; renumbering breaks forensics on field logs.  Deprecate in the map; do not reuse.

## Usage

`/error-code-add <subsystem> <symbolic-name> "<human-readable-description>"`

## Steps

1. **Locate the registry** for the subsystem.  Codes are currently scattered:
   - `sentai_dmesg` codes (DMESG_E_*)
   - `sentai_safety` SERR_* (Safety.md §3)
   - `sentai_prep` SERR_* (slot subsystem)
   - `sentai_fr` SERR_* (Flight Recorder)
   - Consolidated registry: TODO per OP-S10-W12-T10 / W15-T3.

2. **Find the next free numeric code** for the subsystem:
   ```bash
   grep -h "SERR_<SUBSYS>_" /home/bogdan/work/coralmicro/examples/sentai_runtime/*.h \
       /home/bogdan/work/coralmicro/libs/sentai/*.h 2>/dev/null \
       | awk '{ print $3 }' | sort -u
   ```
   Pick the next free number.  NEVER reuse a number that appears as `DEPRECATED` or in git history.

3. **Append the new code** to the subsystem's header:
   ```c
   #define SERR_<SUBSYS>_<NAME>  0x<NEXT_FREE>  // <one-line description>
   ```

4. **Update the string-map** (cold-path string lookup function for post-mortem; lives in `.sdram_text` not in the hot path):
   ```c
   case SERR_<SUBSYS>_<NAME>: return "<human-readable>";
   ```

5. **Document the trigger + recovery** in the subsystem header or `paper/<subsystem>_safety.md`:
   - What condition raises this code?
   - What recovery level (§1.3) applies?
   - Is it fatal, recoverable, or degraded-but-allowed?

## Reject patterns

- Reusing a retired code number ("nobody references it any more" — old log files do).
- Renumbering an existing code (breaks every old log file + every binary that wrote logs with the old code).
- Mass-renumbering "to clean up the order".
- Logging an error as a string instead of a code (per §7.1: codes, not strings, in OCRAM/ITCM; strings only in post-hoc map).

## Verify

```bash
# No two codes share a number across the subsystem registry
grep -h "SERR_<SUBSYS>_" *.h | awk '{ print $3 }' | sort | uniq -d
# Should be empty.
```

## See also

- `embeded.md` §7.1 (error codes, append-only)
- `embeded.md` §6.4 (anomaly tracking — IDs never reused)
- Safety.md §3 (sentai.safety SERR_* registry)
