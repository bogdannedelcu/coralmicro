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

// Enable importing .py files from LittleFS filesystem
#define MICROPY_ENABLE_EXTERNAL_IMPORT  (1)

// Board identification for sys.implementation._machine
#define MICROPY_HW_BOARD_NAME           "SentAI board v1.0"
#define MICROPY_HW_MCU_NAME             "i.MX RT1176"

// Disable everything we don't need to save .text space
#define MICROPY_PY_SYS                  (1)
#define MICROPY_PY_IO                   (0)
#define MICROPY_LONGINT_IMPL            (MICROPY_LONGINT_IMPL_NONE)
#define MICROPY_FLOAT_IMPL              (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_PY_BUILTINS_COMPLEX     (0)
#define MICROPY_PY_BUILTINS_SET         (0)
#define MICROPY_PY_BUILTINS_FROZENSET   (0)
#define MICROPY_PY_BUILTINS_PROPERTY    (0)
#define MICROPY_PY_BUILTINS_ENUMERATE   (0)
#define MICROPY_PY_BUILTINS_FILTER      (0)
#define MICROPY_PY_BUILTINS_REVERSED    (0)
#define MICROPY_PY_BUILTINS_SLICE       (0)
#define MICROPY_PY_BUILTINS_HELP        (0)
#define MICROPY_PY_BUILTINS_MIN_MAX     (0)
#define MICROPY_PY_BUILTINS_INPUT       (0)
#define MICROPY_PY_BUILTINS_MEMORYVIEW  (0)
#define MICROPY_PY_BUILTINS_BYTES_HEX  (0)
#define MICROPY_PY_BUILTINS_STR_COUNT   (0)
#define MICROPY_PY_BUILTINS_STR_OP_MODULO (0)
#define MICROPY_PY_ARRAY                (0)
#define MICROPY_PY_COLLECTIONS          (0)
#define MICROPY_PY_MATH                 (0)
#define MICROPY_PY_CMATH                (0)
#define MICROPY_PY_STRUCT               (0)
#define MICROPY_PY_MICROPYTHON_MEM_INFO (0)
#define MICROPY_ERROR_REPORTING         (MICROPY_ERROR_REPORTING_TERSE)
#define MICROPY_PY_ASSIGN_EXPR          (0)
#define MICROPY_COMP_CONST              (0)
#define MICROPY_COMP_DOUBLE_TUPLE_ASSIGN (0)
#define MICROPY_COMP_TRIPLE_TUPLE_ASSIGN (0)
#define MICROPY_ENABLE_SOURCE_LINE      (0)
#define MICROPY_PY_ALL_SPECIAL_METHODS  (0)
#define MICROPY_PY_REVERSE_SPECIAL_METHODS (0)
#define MICROPY_PY_DESCRIPTORS          (0)
#define MICROPY_PY_ATTRTUPLE            (0)
#define MICROPY_PERSISTENT_CODE_LOAD    (0)
#define MICROPY_EMIT_THUMB              (0)
#define MICROPY_EMIT_INLINE_THUMB       (0)
