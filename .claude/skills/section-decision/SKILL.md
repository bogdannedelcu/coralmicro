---
name: section-decision
description: Decide which RT1176 memory region a new static buffer should live in. Codifies the OP-S10-W15-T3 flowchart before that WP is scheduled. Use at any new static buffer declaration ≥ 1 KB on the ARM build.
---

# /section-decision

Per `embeded.md` §4.1 (memory discipline) + OP-S10-W15-T3 (Memory Allocation policy, paper/arm_memory_budget.md when shipped) + auto-memory `[[itcm-budget]]`.

## Decision flowchart

1. **Is the buffer accessed from an ISR (read or write)?**
   - Yes → DTCM `.itcm_bss` (data) + `.ramfunc` (code).  No SDRAM (SEMC contention can stall the ISR).
2. **Is it touched by DMA?**
   - Yes → must be in a DMA-reachable region (SDRAM, OCRAM, or non-cached DTCM section).  Watch cache coherency — manual clean/invalidate or the SDK's `DCACHE_*` helpers.
3. **Is it per-frame hot (read every camera tick, ≥ 30 Hz)?**
   - Yes → DTCM `m_data` or OCRAM `m_ocram` (cached, low latency).  32-byte aligned (one D-cache line) per W15-T5 cache-line alignment policy.
4. **Is it ≥ 4 KB and not per-frame hot?**
   - Yes → default to SDRAM (`m_sdram` for code, `.sdram_bss` for data).
5. **Is it cold-path / init-only / log buffer?**
   - Yes → `.sentai_slow` (SDRAM cold-path text) or `.sdram_bss` (data).

## Output

```c
// Hot, cache-friendly, no DMA — place in OCRAM, 32-byte aligned (one D-cache line)
__attribute__((section(".ocram_bss"), aligned(32))) static uint8_t my_buf[SIZE];
```

OR, when W15-T5 lands, use the macro:

```c
SENTAI_HOT_BUF_ALIGN static uint8_t my_buf[SIZE];  // expands to attribute(section, aligned(32))
```

## Hard rules (codified in embeded.md §4.1 + W15)

- Hot buffers ≥ 256 B MUST be 32-byte aligned (one D-cache line).
- Never put DMA buffers in DTCM if cached (cache coherency footgun).
- Never put ISR-touched code in SDRAM (SEMC contention).
- Always document the rationale in a comment above the buffer (which region + why).
- Persistent state buffers (boot counter, fault counters) go in SRC_GPR or FxUser, NOT in BSS.

## Verify

After build, confirm the symbol landed where expected:
```bash
arm-none-eabi-objdump -t build/examples/sentai_runtime/sentai_runtime.elf | grep <symbol_name>
```
Symbol address must fall in the expected region per `MIMXRT1176xxxxx_cm7_ram_mp.ld`.

## Reject patterns

- Default to BSS without thinking — by default `m_data` (DTCM) is the smallest region and fills up first.
- "It's only 100 bytes" — sub-cache-line buffers next to hot buffers cause false sharing.
- Cross-subsystem buffers exposed without explicit ownership — violates §4.1 "no cross-subsystem unsafe memory exposure".

## See also

- `embeded.md` §4.1 (memory discipline)
- OP-S10-W15 (planned memory budget WP) + W15-T5 cache-line alignment policy
- `MIMXRT1176xxxxx_cm7_ram_mp.ld` (linker script — source of truth for region sizes)
- `[[itcm-budget]]` (auto-memory)
