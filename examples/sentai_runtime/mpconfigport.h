// MicroPython configuration for SentAI board (embed port)
// Minimal config to run simple scripts in a FreeRTOS task

// Include common embed port configuration.
#include <port/mpconfigport_common.h>

// Use the minimal starting configuration (disables all optional features).
#define MICROPY_CONFIG_ROM_LEVEL        (MICROPY_CONFIG_ROM_LEVEL_MINIMUM)

// Enable compiler so we can run source strings.
#define MICROPY_ENABLE_COMPILER         (1)

// Enable GC for memory management.
#define MICROPY_ENABLE_GC               (1)
#define MICROPY_PY_GC                   (1)

// Enable keyboard interrupt (Ctrl+C) support for stopping running scripts.
#define MICROPY_KBD_EXCEPTION           (1)

// Enable mp_sched_schedule so non-MP FreeRTOS tasks (e.g. crazy_rx_task)
// can deliver async events into a Python callback at the next VM tick.
#define MICROPY_ENABLE_SCHEDULER        (1)

// Make MICROPY_BEGIN_ATOMIC_SECTION cross-task safe.
//
// MicroPython's default for the embed port leaves these as a no-op,
// which is fine for single-task interpreters. We schedule callbacks
// from `crazy_rx_task` (a *different* FreeRTOS task than the MP VM
// task), so the sched_queue head/tail/state must be protected against
// concurrent access from both sides — exactly what FreeRTOS critical
// sections are for.
//
// We can't include FreeRTOS headers here directly: this file is also
// processed by the QSTR preprocessor stage (without the firmware
// include paths). Instead we route through tiny extern wrappers
// implemented in mp_embed_safe.c, which lives in the firmware build.
#ifdef __cplusplus
extern "C" {
#endif
void mp_embed_enter_critical(void);
void mp_embed_exit_critical(void);
#ifdef __cplusplus
}
#endif
#define MICROPY_BEGIN_ATOMIC_SECTION()    (mp_embed_enter_critical(), (mp_uint_t)0)
#define MICROPY_END_ATOMIC_SECTION(state) do { (void)(state); mp_embed_exit_critical(); } while (0)

// Enable importing .py files from LittleFS filesystem
#define MICROPY_ENABLE_EXTERNAL_IMPORT  (1)

// Board identification for sys.implementation._machine
#define MICROPY_HW_BOARD_NAME           "SentAI board v1.0"
#define MICROPY_HW_MCU_NAME             "i.MX RT1176"

// ── Enabled features (tier 1: essential + tier 2: quality-of-life) ──────
#define MICROPY_PY_SYS                  (1)
#define MICROPY_PY_IO                   (0)
#define MICROPY_LONGINT_IMPL            (MICROPY_LONGINT_IMPL_LONGLONG)
#define MICROPY_FLOAT_IMPL              (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_PY_BUILTINS_SLICE       (1)
#define MICROPY_PY_BUILTINS_ENUMERATE   (1)
#define MICROPY_PY_BUILTINS_MIN_MAX     (1)
#define MICROPY_PY_BUILTINS_FILTER      (1)
#define MICROPY_PY_BUILTINS_REVERSED    (1)
#define MICROPY_PY_BUILTINS_SET         (1)
#define MICROPY_PY_BUILTINS_FROZENSET   (1)
#define MICROPY_PY_BUILTINS_PROPERTY    (1)
#define MICROPY_PY_BUILTINS_INPUT       (0)   // needs readline (not in embed port)
#define MICROPY_PY_BUILTINS_MEMORYVIEW  (1)
#define MICROPY_PY_BUILTINS_BYTES_HEX   (1)
#define MICROPY_PY_BUILTINS_STR_COUNT   (1)
#define MICROPY_PY_BUILTINS_STR_OP_MODULO (1)
#define MICROPY_PY_ARRAY                (1)
#define MICROPY_PY_COLLECTIONS          (1)
#define MICROPY_PY_MATH                 (1)
#define MICROPY_PY_STRUCT               (1)
#define MICROPY_PY_MICROPYTHON_MEM_INFO (1)
#define MICROPY_PY_ASSIGN_EXPR          (1)   // walrus operator :=
#define MICROPY_PY_ATTRTUPLE            (1)   // named attrs in tuples
#define MICROPY_COMP_CONST              (1)   // const() optimization
#define MICROPY_COMP_DOUBLE_TUPLE_ASSIGN (1)  // a, b = x, y
#define MICROPY_COMP_TRIPLE_TUPLE_ASSIGN (1)  // a, b, c = x, y, z
#define MICROPY_ENABLE_SOURCE_LINE      (1)   // line numbers in tracebacks
#define MICROPY_ERROR_REPORTING         (MICROPY_ERROR_REPORTING_NORMAL)

// ── Disabled (not needed / risky on embedded) ──────────────────────────
#define MICROPY_PY_BUILTINS_COMPLEX     (0)
#define MICROPY_PY_BUILTINS_HELP        (1)
#define MICROPY_PY_BUILTINS_HELP_TEXT   sentai_help_builtin_text
#define MICROPY_PY_BUILTINS_HELP_MODULES (0)
#define MICROPY_PY_CMATH                (0)
#define MICROPY_PY_ALL_SPECIAL_METHODS  (0)
#define MICROPY_PY_REVERSE_SPECIAL_METHODS (0)
#define MICROPY_PY_DESCRIPTORS          (0)
#define MICROPY_PERSISTENT_CODE_LOAD    (0)
#define MICROPY_EMIT_THUMB              (0)
#define MICROPY_EMIT_INLINE_THUMB       (0)
