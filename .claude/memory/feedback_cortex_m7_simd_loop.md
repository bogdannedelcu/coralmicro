---
name: Cortex-M7 SIMD inner loop best practices
description: USAD8 + LD32U packed-struct trick + ALWAYS verify objdump. Lessons from the SAD optimization that turned 13ms into 0.94ms.
type: feedback
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
For tight integer SIMD loops on Cortex-M7 (SAD, abs-diff, sum-of-products etc.):

1.  **Use `__USADA8` from `<arm_acle.h>`** -- packs 4 abs-diffs +
    accumulate into 1 cycle.  Drop-in replacement for any byte-wise
    diff loop.  Cuts ~16 cycles to 1.

2.  **NEVER use `memcpy(&u32, p, 4)` for unaligned loads.**  GCC
    compiles it as a CALL to libc memcpy unless alignment is
    statically provable.  ~30 cycles overhead per call.  Use:
    ```c
    typedef struct { uint32_t v; }
        __attribute__((packed, aligned(1))) u32_unaligned;
    #define LD32U(p) (((const u32_unaligned*)(p))->v)
    ```
    Emits a single `LDR` (Cortex-M7 supports unaligned LDR in HW).

3.  **Always verify the assembly** with `arm-none-eabi-objdump -d
    <obj>` after any inner-loop change.  My first USAD8 attempt
    DID emit USADA8 but ALSO 8 `bl 0 <memcpy>` calls per iter -->
    net SLOWER, despite the SIMD intrinsic.  Without disasm I
    would have shipped a regression.

4.  **Hot-path code goes in ITCM** via
    `__attribute__((section(".ramfunc")))`.  Marginal in practice
    when D-cache is well-warmed (~5%) but free safety margin
    against SDRAM bus contention.

**Why:** Real measurement during 2026-05-05 flow optimization:
SAD compute went from 13.27 ms → 0.94 ms (14×) by fixing the
memcpy-to-libc bug + applying USADA8.  The wrong intrinsic call
pattern would have shipped looking like SIMD was useless.

**How to apply:** When optimizing any tight uint8 inner loop on
M7 (image processing, signal processing, matrix abs-diff), reach
for these tools FIRST.  Always verify with objdump.
