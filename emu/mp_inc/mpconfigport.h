// MicroPython configuration for the B8 ARM-emulator REPL spike.
//
// This is intentionally separate from
// `examples/sentai_runtime/mpconfigport.h`: the production config pulls in
// FreeRTOS critical sections, the sentai help table, and module bindings that
// the emu_repl target deliberately omits.
//
// The shared genhdr/qstrdefs.generated.h and genhdr/moduledefs.h were baked
// against the production config, so this file must keep the same enabled
// feature set for any module/QSTR referenced by those generated files. The
// linker has no way to drop unused module table entries, so disabling a
// module here would only produce "undefined reference to mp_module_X" errors.
//
// Differences from production:
//   * MICROPY_KBD_EXCEPTION off (no Ctrl+C path in this spike).
//   * MICROPY_ENABLE_SCHEDULER off (no async callbacks from C tasks).
//   * MICROPY_ENABLE_EXTERNAL_IMPORT off (no filesystem yet).
//   * MICROPY_PY_BUILTINS_HELP off (sentai_help_builtin_text not linked).
//   * No FreeRTOS critical section wrappers (single MP task only).
//   * `sentai` module replaced by an empty stub in sentai_emu_stub_modules.c.

#include <port/mpconfigport_common.h>

#define MICROPY_CONFIG_ROM_LEVEL        (MICROPY_CONFIG_ROM_LEVEL_MINIMUM)

#define MICROPY_ENABLE_COMPILER         (1)
#define MICROPY_ENABLE_GC               (1)
#define MICROPY_PY_GC                   (1)
#define MICROPY_LONGINT_IMPL            (MICROPY_LONGINT_IMPL_LONGLONG)
#define MICROPY_FLOAT_IMPL              (MICROPY_FLOAT_IMPL_FLOAT)

#define MICROPY_KBD_EXCEPTION           (0)
#define MICROPY_ENABLE_SCHEDULER        (0)
#define MICROPY_ENABLE_EXTERNAL_IMPORT  (0)
#define MICROPY_PY_BUILTINS_HELP        (0)
#define MICROPY_PY_BUILTINS_INPUT       (0)

// Match production: the QSTR table baked into genhdr depends on this.
#define MICROPY_PY_BUILTINS_STR_OP_MODULO (1)

// Modules referenced unconditionally by genhdr/moduledefs.h.
#define MICROPY_PY_SYS                  (1)
#define MICROPY_PY_ARRAY                (1)
#define MICROPY_PY_COLLECTIONS          (1)
#define MICROPY_PY_MATH                 (1)
#define MICROPY_PY_STRUCT               (1)

// Frequently-referenced builtins; keep on to avoid unresolved QSTR refs in
// the shared parser/REPL surface.
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
