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

#include <stdio.h>
#include <stdint.h>

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

    // --- Best-effort UART output (goes to boot log ring buffer if active) ---
    printf("\r\n*** CRASH: code=0x%04X PC=0x%08lX LR=0x%08lX "
           "CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX up=%lums\r\n",
           (unsigned)fault_code,
           (unsigned long)pc, (unsigned long)lr,
           (unsigned long)cfsr, (unsigned long)hfsr,
           (unsigned long)fault_addr,
           (unsigned long)g_sentai_uptime_ms);
    printf("  r0=0x%08lX r1=0x%08lX r2=0x%08lX r3=0x%08lX\r\n",
           (unsigned long)frame[0], (unsigned long)frame[1],
           (unsigned long)frame[2], (unsigned long)frame[3]);

    // Decode common CFSR bits for immediate diagnosis
    if (cfsr & 0x00000002UL) printf("  IBUS: instruction bus error\r\n");
    if (cfsr & 0x00001000UL) printf("  PRECISERR: data bus error @ BFAR\r\n");
    if (cfsr & 0x00002000UL) printf("  STKER: bus fault on exception entry\r\n");
    if (cfsr & 0x00000400UL) printf("  UNSTKER: bus fault on exception return\r\n");
    if (cfsr & 0x00000001UL) printf("  IACCVIOL: instruction access violation\r\n");
    if (cfsr & 0x00000002UL) printf("  DACCVIOL: data access violation\r\n");
    if (cfsr & 0x00000200UL) printf("  INVSTATE: invalid CPU state (bad EPSR.T)\r\n");
    if (cfsr & 0x00000400UL) printf("  INVPC: invalid exception return\r\n");
    if (cfsr & 0x00020000UL) printf("  DIVBYZERO: integer divide by zero\r\n");
    if (cfsr & 0x00010000UL) printf("  UNALIGNED: unaligned memory access\r\n");
    if (cfsr & 0x40000000UL) printf("  VECTTBL: vector table read fault\r\n");

    // Scan stack above exception frame for potential return addresses (stack hint)
    // Code lives in ITCM (0x00000000-0x001FFFFF). Thumb addresses have bit 0 set.
    printf("  Stack hint (possible callers):");
    int found = 0;
    for (int i = 8; i < 40 && found < 5; i++) {
        uint32_t word = frame[i];
        // Check for valid Thumb return address in ITCM/OCRAM code range
        if ((word & 1UL) &&
            ((word >= 0x00004000UL && word <= 0x001FFFFFUL) ||
             (word >= 0x20200000UL && word <= 0x2027FFFFUL))) {
            printf(" 0x%08lX", (unsigned long)(word & ~1UL));
            found++;
        }
    }
    if (found == 0) printf(" (none in range)");
    printf("\r\n");

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
// vApplicationMallocFailedHook — strong override of the weak version in
// libs/FreeRTOS/app_callbacks.c.  Saves breadcrumb and resets.
// ---------------------------------------------------------------------------
extern "C" void vApplicationMallocFailedHook(void) {
    // __builtin_return_address(0) gives the call site (who called malloc)
    uint32_t caller_lr = (uint32_t)__builtin_return_address(0);
    sentai_fault_save(SERR_SYS_MALLOC_FAIL, 0, caller_lr, 0, 0,
                      g_sentai_uptime_ms, 0);
    printf("\r\n*** MALLOC FAILED (caller LR=0x%08lX) — resetting\r\n",
           (unsigned long)caller_lr);
    // Immediate reset
    *AIRCR_ADDR = (0x5FAUL << 16U) | (1UL << 2U);
    for (;;) {}
}
