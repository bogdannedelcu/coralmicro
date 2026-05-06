// mp_embed_safe.c — Safe (return-value) wrapper for mp_embed_exec_str.
//
// Unlike upstream mp_embed_exec_str() which returns void, this variant
// returns 0 on success or -1 on exception. The exception traceback is
// still printed to stdout.
//
// Also hosts the FreeRTOS critical-section wrappers used as the embed
// port's MICROPY_BEGIN/END_ATOMIC_SECTION macros (see mpconfigport.h).
// Wrapping is necessary because the QSTR preprocessor stage cpp's
// mpconfigport.h without the firmware include paths, so we can't
// include FreeRTOS headers there.
//
// Lives in our source tree (not in micropython_embed/port/) so it
// survives QSTR regeneration without any patching.

#include <string.h>
#include "py/compile.h"
#include "py/runtime.h"
#include "py/nlr.h"
#include "port/micropython_embed.h"

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

void mp_embed_enter_critical(void) { taskENTER_CRITICAL(); }
void mp_embed_exit_critical(void)  { taskEXIT_CRITICAL();  }

#if MICROPY_ENABLE_COMPILER
int mp_embed_exec_str_safe(const char *src) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(
            MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
        mp_call_function_0(module_fun);
        nlr_pop();
        return 0;
    } else {
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
        return -1;
    }
}
#endif
