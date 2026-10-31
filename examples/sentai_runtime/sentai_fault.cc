// SentAI Fault Management — HardFault handler + crash breadcrumb API
//
// This file provides:
//   1. sentai_fault_save/read/clear  — SRC GPR breadcrumb API
//   2. HardFault_Handler             — naked trampoline → HardFault_C
//   3. BusFault_Handler, UsageFault_Handler, MemManage_Handler
//      — redirect to the same C handler with fault type code
//   4. vApplicationMallocFailedHook  — strong override: save + reset

#include "sentai_fault.h"
#include "sentai_error.h"

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_soc_src.h"

#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Fault-context text output (2026-10-31 audit base-M1).
//
// printf is NOT usable here: the console path takes RTOS locks and
// allocates, and FreeRTOS asserts when a task-level critical section is
// entered from handler mode -- the assert then re-entered the fault path
// and clobbered the breadcrumb.  _write() already drops text inside
// interrupts, so the decoded crash record never reached anyone.
//
// Instead the record goes straight into the .sdram_boot_log NOLOAD ring
// through sentai_fault_text() (sentai_runtime.cc): plain stores, no libc,
// no RTOS.  The next boot rescues that ring to /log/boot_prev.log, where
// `*** CRASH ...` lines are found exactly as the console used to show
// them.  Formatting below is hand-rolled so the handler runs with a few
// dozen bytes of stack and no newlib state.
// ---------------------------------------------------------------------------
static size_t fault_strlen(const char *s) {
    size_t n = 0;
    while (s[n] != '\0' && n < 64u) n++;
    return n;
}
static void fault_puts(const char *s) { sentai_fault_text(s, fault_strlen(s)); }
static void fault_puthex(uint32_t v) {
    char buf[11];
    buf[0] = '0'; buf[1] = 'x';
    for (int i = 0; i < 8; i++) {
        uint32_t nib = (v >> (28 - 4 * i)) & 0xFu;
        buf[2 + i] = (char)(nib < 10u ? ('0' + nib) : ('A' + nib - 10u));
    }
    buf[10] = '\0';
    sentai_fault_text(buf, 10);
}
static void fault_putdec(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do { buf[--i] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u && i > 0);
    sentai_fault_text(buf + i, (size_t)(10 - i));
}
static void fault_field(const char *name, uint32_t v) {
    fault_puts(name); fault_puthex(v);
}

// ---------------------------------------------------------------------------
// Uptime hint — updated externally by health/watchdog task each cycle.
// The HardFault handler reads this without any RTOS calls.
// ---------------------------------------------------------------------------
volatile uint32_t g_sentai_uptime_ms = 0;

// ---------------------------------------------------------------------------
// Cortex-M fault status register addresses (ARM Cortex-M7 architecture)
// ---------------------------------------------------------------------------
#define CFSR_ADDR   ((volatile uint32_t*)0xE000ED28UL)  // Configurable Fault Status
#define HFSR_ADDR   ((volatile uint32_t*)0xE000ED2CUL)  // HardFault Status
#define BFAR_ADDR   ((volatile uint32_t*)0xE000ED38UL)  // Bus Fault Address Register
#define MMFAR_ADDR  ((volatile uint32_t*)0xE000ED34UL)  // MemManage Fault Address
#define AIRCR_ADDR  ((volatile uint32_t*)0xE000ED0CUL)  // Application Interrupt/Reset Control

// CFSR bit fields used for decoding
#define CFSR_MMARVALID  (1UL << 7)   // MMFAR valid
#define CFSR_BFARVALID  (1UL << 15)  // BFAR valid

// ---------------------------------------------------------------------------
// sentai_fault_save — write crash breadcrumb to SRC GPR2-8
// Safe to call from any context: fault handlers, ISRs, task.
// ---------------------------------------------------------------------------
void sentai_fault_save(uint16_t err_code, uint32_t pc, uint32_t lr,
                       uint32_t cfsr, uint32_t fault_addr,
                       uint32_t uptime_ms, uint32_t r0) {
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister2,
                                  SENTAI_FAULT_MAGIC | (uint32_t)err_code);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister3, pc);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister4, lr);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister5, cfsr);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister6, fault_addr);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister7, uptime_ms);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister8, r0);
}

// ---------------------------------------------------------------------------
// sentai_fault_read — read crash breadcrumb from SRC GPR2-8
// ---------------------------------------------------------------------------
bool sentai_fault_read(sentai_crash_record_t *out) {
    uint32_t gpr2 = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister2);
    if ((gpr2 & 0xFF000000UL) != SENTAI_FAULT_MAGIC) {
        return false;
    }
    out->code       = (uint16_t)(gpr2 & 0x0000FFFFUL);
    out->pc         = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister3);
    out->lr         = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister4);
    out->cfsr       = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister5);
    out->fault_addr = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister6);
    out->uptime_ms  = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister7);
    out->r0         = SRC_GetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister8);
    return true;
}

// ---------------------------------------------------------------------------
// sentai_fault_clear — zero GPR2-8
// ---------------------------------------------------------------------------
void sentai_fault_clear(void) {
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister2, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister3, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister4, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister5, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister6, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister7, 0);
    SRC_SetGeneralPurposeRegister(SRC, kSRC_GeneralPurposeRegister8, 0);
}

// ---------------------------------------------------------------------------
// sentai_assert_fail — called by configASSERT before the AIRCR reset.
// Saves SERR_SYS_ASSERT breadcrumb with the call site LR.
// ---------------------------------------------------------------------------
extern "C" void sentai_assert_fail(uint32_t caller_lr) {
    sentai_fault_save(SERR_SYS_ASSERT, /*pc=*/0, caller_lr,
                      /*cfsr=*/0, /*fault_addr=*/0,
                      g_sentai_uptime_ms, /*r0=*/0);
}

// ---------------------------------------------------------------------------
// Fault C handler — called from naked assembly trampolines below.
//   frame     : pointer to the auto-saved exception frame on PSP or MSP
//   fault_code: SERR code identifying which fault handler was entered
//
// Exception frame layout (pushed by hardware, sp points at r0):
//   sp[0]=r0  sp[1]=r1  sp[2]=r2  sp[3]=r3
//   sp[4]=r12 sp[5]=lr  sp[6]=pc  sp[7]=xpsr
//
// extern "C" is required so the naked assembly trampolines can branch here
// by unmangled name.
// ---------------------------------------------------------------------------
extern "C" void sentai_fault_c(uint32_t *frame, uint16_t fault_code) {
    uint32_t pc   = frame[6];
    uint32_t lr   = frame[5];
    uint32_t r0   = frame[0];
    uint32_t cfsr = *CFSR_ADDR;
    uint32_t hfsr = *HFSR_ADDR;

    // Determine precise fault address: BFAR for bus faults, MMFAR for MPU faults
    uint32_t fault_addr = 0;
    if (cfsr & CFSR_BFARVALID) {
        fault_addr = *BFAR_ADDR;
    } else if (cfsr & CFSR_MMARVALID) {
        fault_addr = *MMFAR_ADDR;
    }

    // --- Persist to SRC GPR (survives warm reset, readable on next boot) ---
    sentai_fault_save(fault_code, pc, lr, cfsr, fault_addr,
                      g_sentai_uptime_ms, r0);

    // --- Crash record into the persistent crash-text slot (no printf) ---
    // The slot lives in SDRAM.  If THIS fault is a precise bus fault on the
    // SDRAM window (SEMC / self-refresh / clock problem, BFAR 0x8000_0000..
    // 0x8FFF_FFFF), writing the record would re-fault inside the handler;
    // skip the text and go straight to the reset.  The GPR breadcrumb above
    // already holds code/PC/LR/CFSR/BFAR.  Should a nested fault still
    // happen, SRC_SRMR is at its reset value 0 on this board (read 2026-10-31
    // via sentai.sys.peek32(0x40C04004)), i.e. an M7 lockup generates a
    // system reset, which reset.cc counts as kSRC_M7CoreM7LockUpResetFlag.
    if ((cfsr & CFSR_BFARVALID) &&
        fault_addr >= 0x80000000UL && fault_addr <= 0x8FFFFFFFUL) {
        *AIRCR_ADDR = (0x5FAUL << 16U) | (1UL << 2U);
        for (;;) {}
    }
    fault_puts("\r\n*** CRASH: code="); fault_puthex(fault_code);
    fault_field(" PC=", pc);  fault_field(" LR=", lr);
    fault_field(" CFSR=", cfsr); fault_field(" HFSR=", hfsr);
    fault_field(" BFAR=", fault_addr);
    fault_puts(" up="); fault_putdec(g_sentai_uptime_ms); fault_puts("ms\r\n");
    fault_field("  r0=", frame[0]); fault_field(" r1=", frame[1]);
    fault_field(" r2=", frame[2]);  fault_field(" r3=", frame[3]);
    fault_puts("\r\n");

    // Decode CFSR bits (ARM DDI0403D): MMFSR[7:0] | BFSR[15:8] | UFSR[31:16]
    // MMFSR — MemManage Fault Status
    if (cfsr & 0x00000001UL) fault_puts("  IACCVIOL: MPU instruction access violation\r\n");
    if (cfsr & 0x00000002UL) fault_puts("  DACCVIOL: MPU data access violation\r\n");
    if (cfsr & 0x00000008UL) fault_puts("  MUNSTKERR: MPU fault on exception return\r\n");
    if (cfsr & 0x00000010UL) fault_puts("  MSTKERR: MPU fault on exception entry\r\n");
    // BFSR — Bus Fault Status
    if (cfsr & 0x00000100UL) fault_puts("  IBUSERR: instruction bus error\r\n");
    if (cfsr & 0x00000200UL) fault_puts("  PRECISERR: precise data bus error @ BFAR\r\n");
    if (cfsr & 0x00000400UL) fault_puts("  IMPRECISERR: imprecise data bus error\r\n");
    if (cfsr & 0x00000800UL) fault_puts("  BUNSTKERR: bus fault on exception return\r\n");
    if (cfsr & 0x00001000UL) fault_puts("  BSTKERR: bus fault on exception entry\r\n");
    // UFSR — UsageFault Status
    if (cfsr & 0x00010000UL) fault_puts("  UNDEFINSTR: undefined instruction\r\n");
    if (cfsr & 0x00020000UL) fault_puts("  INVSTATE: invalid CPU state (bad EPSR.T)\r\n");
    if (cfsr & 0x00040000UL) fault_puts("  INVPC: invalid exception return value\r\n");
    if (cfsr & 0x00080000UL) fault_puts("  NOCP: coprocessor access (FPU disabled?)\r\n");
    if (cfsr & 0x01000000UL) fault_puts("  UNALIGNED: unaligned memory access\r\n");
    if (cfsr & 0x02000000UL) fault_puts("  DIVBYZERO: integer divide by zero\r\n");
    // HFSR
    if (hfsr & 0x00000002UL) fault_puts("  VECTTBL: vector table read fault\r\n");
    if (hfsr & 0x40000000UL) fault_puts("  FORCED: hard fault escalated from lower fault\r\n");

    // Scan stack above exception frame for potential return addresses (stack hint)
    // Code lives in ITCM (0x00000000-0x001FFFFF). Thumb addresses have bit 0 set.
    fault_puts("  Stack hint (possible callers):");
    int found = 0;
    for (int i = 8; i < 40 && found < 5; i++) {
        uint32_t word = frame[i];
        // Check for valid Thumb return address in ITCM/OCRAM code range
        if ((word & 1UL) &&
            ((word >= 0x00004000UL && word <= 0x001FFFFFUL) ||
             (word >= 0x20200000UL && word <= 0x2027FFFFUL))) {
            fault_puts(" "); fault_puthex(word & ~1UL);
            found++;
        }
    }
    if (found == 0) fault_puts(" (none in range)");
    fault_puts("\r\n");

    // --- Immediate system reset (triggers watchdog path, increments boot counter) ---
    *AIRCR_ADDR = (0x5FAUL << 16U) | (1UL << 2U);
    for (;;) {}
}

// ---------------------------------------------------------------------------
// HardFault_Handler — naked trampoline
// Determines whether MSP or PSP was active, passes frame pointer to C handler.
// ---------------------------------------------------------------------------
extern "C" __attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "tst  lr, #4       \n\t"   // check EXC_RETURN bit 2: 0=MSP, 1=PSP
        "ite  eq           \n\t"
        "mrseq r0, msp     \n\t"   // r0 = MSP (fault in ISR or main stack)
        "mrsne r0, psp     \n\t"   // r0 = PSP (fault in FreeRTOS task)
        "movw r1, #0x0FFF  \n\t"   // r1 = SERR_SYS_HARDFAULT (low 16 bits)
        "b    sentai_fault_c\n\t"
    );
}

// BusFault, UsageFault, MemManage — redirect to same handler with own SERR code.
// These are escalated to HardFault by default (SHCSR not explicitly enabled),
// so these handlers fire only if the caller enables them in SCB->SHCSR.
// Providing them here ensures correct SERR codes if they're ever enabled.
extern "C" __attribute__((naked)) void BusFault_Handler(void) {
    __asm volatile(
        "tst  lr, #4       \n\t"
        "ite  eq           \n\t"
        "mrseq r0, msp     \n\t"
        "mrsne r0, psp     \n\t"
        "movw r1, #0x0FF4  \n\t"   // r1 = SERR_SYS_BUS_FAULT
        "b    sentai_fault_c\n\t"
    );
}

extern "C" __attribute__((naked)) void UsageFault_Handler(void) {
    __asm volatile(
        "tst  lr, #4       \n\t"
        "ite  eq           \n\t"
        "mrseq r0, msp     \n\t"
        "mrsne r0, psp     \n\t"
        "movw r1, #0x0FF5  \n\t"   // r1 = SERR_SYS_USAGE_FAULT
        "b    sentai_fault_c\n\t"
    );
}

extern "C" __attribute__((naked)) void MemManage_Handler(void) {
    __asm volatile(
        "tst  lr, #4       \n\t"
        "ite  eq           \n\t"
        "mrseq r0, msp     \n\t"
        "mrsne r0, psp     \n\t"
        "movw r1, #0x0FF6  \n\t"   // r1 = SERR_SYS_MEMMANAGE
        "b    sentai_fault_c\n\t"
    );
}

// ---------------------------------------------------------------------------
// sentai_fault_stack_overflow_record -- called from vApplicationStackOverflowHook
// (PendSV / handler context: no printf, no SERR_LOG, no dmesg).
// ---------------------------------------------------------------------------
extern "C" void sentai_fault_stack_overflow_record(const char *task_name,
                                                   uint32_t name_hash,
                                                   uint32_t caller_lr) {
    sentai_fault_save(SERR_SYS_STACK_OVF, /*pc=*/0, caller_lr,
                      /*cfsr=*/0, /*fault_addr=*/name_hash,
                      g_sentai_uptime_ms, /*r0=*/0);
    fault_puts("\r\n*** STACK OVERFLOW in task '");
    if (task_name) {
        size_t n = fault_strlen(task_name);
        sentai_fault_text(task_name, n > 16u ? 16u : n);
    }
    fault_puts("' LR="); fault_puthex(caller_lr);
    fault_puts(" up="); fault_putdec(g_sentai_uptime_ms);
    fault_puts("ms -- resetting\r\n");
}

// ---------------------------------------------------------------------------
// vApplicationMallocFailedHook — strong override of the weak version in
// libs/FreeRTOS/app_callbacks.c.  Saves breadcrumb and resets.
// ---------------------------------------------------------------------------
extern "C" void vApplicationMallocFailedHook(void) {
    // __builtin_return_address(0) gives the call site (who called malloc)
    uint32_t caller_lr = (uint32_t)__builtin_return_address(0);
    sentai_fault_save(SERR_SYS_MALLOC_FAIL, 0, caller_lr, 0, 0,
                      g_sentai_uptime_ms, 0);
    fault_puts("\r\n*** MALLOC FAILED caller LR="); fault_puthex(caller_lr);
    fault_puts(" -- resetting\r\n");
    // Immediate reset
    *AIRCR_ADDR = (0x5FAUL << 16U) | (1UL << 2U);
    for (;;) {}
}
