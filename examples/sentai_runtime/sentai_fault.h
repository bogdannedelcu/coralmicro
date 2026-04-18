// SentAI Fault Management — Crash Breadcrumb via SRC GPR Registers
//
// At crash time (HardFault, stack overflow, malloc fail, assert):
//   sentai_fault_save() writes PC, LR, CFSR, fault address, uptime to
//   SRC_GPR[2..8] which survive warm reset (watchdog / SW reset).
//
// At next boot (in app_main, Phase 8 block, before clearing GPRs):
//   sentai_fault_read() retrieves the record, sentai_fault_clear() wipes it.
//   After LFS mounts, the record is written to /log/crash.log.
//
// SRC GPR layout (10 registers total on RT1176, all survive warm reset):
//   GPR1  : boot_attempts  (Phase 8 anti-brick counter — existing)
//   GPR2  : 0xCA000000 | err_code  (magic=0xCA identifies valid crash record)
//   GPR3  : faulting PC
//   GPR4  : LR / return address at fault
//   GPR5  : CFSR (Cortex-M Configurable Fault Status Register)
//   GPR6  : BFAR or MMFAR (bad address if memory access fault, else 0)
//   GPR7  : uptime_ms at crash
//   GPR8  : r0 from exception frame (first function argument at crash)
//   GPR13 : watchdog reset count (reset.cc — do not touch)
//   GPR14 : lockup reset count  (reset.cc — do not touch)

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Magic value in high byte of GPR2 — distinguishes valid record from stale/zero
#define SENTAI_FAULT_MAGIC  0xCA000000UL

// Crash record read from SRC GPR on next boot
typedef struct {
    uint16_t code;        // SERR error code (e.g. SERR_SYS_HARDFAULT)
    uint32_t pc;          // faulting program counter
    uint32_t lr;          // link register / call site at fault
    uint32_t cfsr;        // Cortex-M CFSR (fault type details)
    uint32_t fault_addr;  // BFAR or MMFAR (0 if not a memory access fault)
    uint32_t uptime_ms;   // system uptime at crash (ms, 0 if unavailable)
    uint32_t r0;          // r0 from exception frame (first function argument)
} sentai_crash_record_t;

// Save crash breadcrumb to SRC GPR2-8.
// Safe to call from ANY context: HardFault handler, ISRs, task context.
// Uses only volatile memory writes — no locks, no RTOS, no stack required.
void sentai_fault_save(uint16_t err_code, uint32_t pc, uint32_t lr,
                       uint32_t cfsr, uint32_t fault_addr,
                       uint32_t uptime_ms, uint32_t r0);

// Read crash breadcrumb from SRC GPR2-8.
// Returns true and fills *out if a valid record is present (magic matches).
bool sentai_fault_read(sentai_crash_record_t *out);

// Clear crash breadcrumb (zero GPR2-8). Call right after sentai_fault_read().
void sentai_fault_clear(void);

// Assert-fail hook — called by configASSERT before the system reset.
// Saves a SERR_SYS_ASSERT breadcrumb with the call site LR.
// Strong definition is in sentai_fault.cc; app_callbacks.c provides a weak
// no-op fallback so builds without sentai_fault.cc still link.
void sentai_assert_fail(uint32_t caller_lr);

// Updated periodically by the watchdog task via sentai_fault_set_uptime().
// The HardFault handler reads this to get a coarse crash timestamp.
extern volatile uint32_t g_sentai_uptime_ms;

// Update the coarse uptime hint (called from watchdog/health task).
static inline void sentai_fault_set_uptime(uint32_t ms) {
    g_sentai_uptime_ms = ms;
}

#ifdef __cplusplus
}
#endif
