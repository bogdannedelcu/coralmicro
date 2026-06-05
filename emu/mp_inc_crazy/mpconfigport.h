// MicroPython configuration for B9/s219 sentai.crazy namespace tests.
//
// Same shape as the FileX mission target, but with the MicroPython scheduler
// enabled because sentai.crazy.on_message uses mp_sched_schedule from the RX
// task.  Keep this separate so older B8 REPL/FileX targets remain unchanged.

#include <port/mpconfigport_common.h>

#define MICROPY_CONFIG_ROM_LEVEL        (MICROPY_CONFIG_ROM_LEVEL_MINIMUM)

#define MICROPY_ENABLE_COMPILER         (1)
#define MICROPY_ENABLE_GC               (1)
#define MICROPY_PY_GC                   (1)
#define MICROPY_LONGINT_IMPL            (MICROPY_LONGINT_IMPL_LONGLONG)
#define MICROPY_FLOAT_IMPL              (MICROPY_FLOAT_IMPL_FLOAT)

#define MICROPY_KBD_EXCEPTION           (0)
#define MICROPY_ENABLE_SCHEDULER        (1)
#define MICROPY_PY_BUILTINS_HELP        (0)
#define MICROPY_PY_BUILTINS_INPUT       (0)

#define MICROPY_ENABLE_EXTERNAL_IMPORT  (1)
#define MICROPY_PY_SYS_PATH             (1)
#define MICROPY_PY_SYS_ATTR_DELEGATION  (1)
#define MICROPY_MODULE_ATTR_DELEGATION  (1)

#define MICROPY_PY_BUILTINS_STR_OP_MODULO (1)

#define MICROPY_PY_SYS                  (1)
#define MICROPY_PY_ARRAY                (1)
#define MICROPY_PY_COLLECTIONS          (1)
#define MICROPY_PY_MATH                 (1)
#define MICROPY_PY_STRUCT               (1)

#define MICROPY_PY_BUILTINS_SLICE       (1)
#define MICROPY_PY_BUILTINS_ENUMERATE   (1)
#define MICROPY_PY_BUILTINS_MIN_MAX     (1)
#define MICROPY_PY_BUILTINS_FILTER      (1)
#define MICROPY_PY_BUILTINS_REVERSED    (1)
#define MICROPY_PY_BUILTINS_SET         (1)
#define MICROPY_PY_BUILTINS_FROZENSET   (1)
#define MICROPY_PY_BUILTINS_PROPERTY    (1)
#define MICROPY_PY_BUILTINS_MEMORYVIEW  (1)
#define MICROPY_PY_BUILTINS_BYTES_HEX   (1)
#define MICROPY_PY_BUILTINS_STR_COUNT   (1)
#define MICROPY_PY_BUILTINS_FLOAT       (1)
#define MICROPY_PY_BUILTINS_COMPLEX     (0)
#define MICROPY_PY_BUILTINS_HELP_MODULES (0)
#define MICROPY_PY_CMATH                (0)
#define MICROPY_PY_DESCRIPTORS          (0)
#define MICROPY_PERSISTENT_CODE_LOAD    (0)
#define MICROPY_EMIT_THUMB              (0)
#define MICROPY_EMIT_INLINE_THUMB       (0)

#define MICROPY_HW_BOARD_NAME           "SentAI EMU"
#define MICROPY_HW_MCU_NAME             "i.MX RT1176 (Renode)"
