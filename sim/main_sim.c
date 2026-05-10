/*
 * sim/main_sim.c — SentAI SIM entry point (Phase 1 step 8).
 *
 * Boots FreeRTOS scheduler with a single REPL task that reads from stdin,
 * evaluates with MicroPython embed, prints results to stdout.  The terminal
 * acts as a classic console — type `1+1\n`, get `2\n`.
 *
 * Future phases will add tasks for crazy bridge, flow algo, etc., all
 * interacting with the same MicroPython VM via `sentai.*` modules.  For
 * Phase 1 we only need: scheduler boots, MP runs, REPL evaluates expressions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>

#include "FreeRTOS.h"
#include "task.h"

#include "py/runtime.h"
#include "py/lexer.h"
#include "py/builtin.h"
#include "py/objmodule.h"
#include "port/micropython_embed.h"

/* ---- Heap for MicroPython ---- */
#define MP_HEAP_SIZE  (256 * 1024)   /* 256 KB — generous for REPL + small scripts */
static char s_mp_heap[MP_HEAP_SIZE];

/* ---- REPL line buffer ---- */
#define REPL_LINE_MAX  1024
static char s_line[REPL_LINE_MAX];

/* ---- Ctrl-C / SIGINT handler ---- */
static volatile int g_got_sigint = 0;
static void sigint_handler(int sig) {
    (void) sig;
    g_got_sigint = 1;
}

/* ---- EINTR-resilient line reader from stdin ----
 *
 * The FreeRTOS POSIX port delivers SIGALRM at configTICK_RATE_HZ (1 kHz)
 * to drive the scheduler tick.  This interrupts any blocking read()
 * syscall on stdin with errno=EINTR.  glibc's fgets() does NOT retry on
 * EINTR — it returns NULL and sets feof(), making the REPL think the user
 * pressed Ctrl-D after every single tick.
 *
 * Workaround: read one char at a time via read(STDIN_FILENO, ...) with an
 * explicit EINTR-retry loop.  Returns 0 on EOF, -1 on real error, or the
 * line length on success (line is null-terminated, newline stripped). */
static int sim_read_line(char *buf, size_t max_len) {
    size_t pos = 0;
    while (pos < max_len - 1) {
        char c;
        ssize_t n;
        do {
            n = read(STDIN_FILENO, &c, 1);
        } while (n == -1 && errno == EINTR);

        if (n == 0) {
            /* True EOF (Ctrl-D on empty line) */
            if (pos == 0) return 0;
            break;          /* EOF after some bytes — return what we have */
        }
        if (n < 0) {
            return -1;
        }
        if (c == '\n') break;
        if (c == '\r') continue;            /* tolerate CRLF */
        buf[pos++] = c;
    }
    buf[pos] = '\0';
    return (int) pos + 1;   /* >=1 so callers can distinguish from EOF */
}

/* ---- REPL task ----
 *
 * Single FreeRTOS task that runs the MicroPython REPL loop on stdin/stdout.
 * Mirrors the structure of examples/sentai_runtime/micropython_task.c but
 * stripped to the bare minimum: no chunked upload protocol, no Ctrl-C
 * monitor task, no boot.log, no main.py auto-exec.
 */
static void repl_task(void *param) {
    (void) param;

    /* MicroPython needs an approximate stack-top to detect overflow.  On
     * Linux pthreads have ~8 MB stacks by default; using a local var as
     * the marker is fine.  The POSIX port allocates a separate pthread
     * for each FreeRTOS task, so &dummy is on the REPL task's pthread
     * stack. */
    int dummy;
    mp_embed_init(&s_mp_heap[0], MP_HEAP_SIZE, &dummy);

    printf("\n");
    printf("MicroPython on SentAI SIM (FreeRTOS POSIX port + MicroPython embed)\n");
    printf("Type expressions, end with Enter.  Ctrl-D or 'exit' to quit.\n");
    printf("\n");

    for (;;) {
        printf(">>> ");
        fflush(stdout);

        /* Use sim_read_line — fgets is not safe under SIGALRM-driven
         * scheduler ticks (see comment above sim_read_line). */
        int len = sim_read_line(s_line, sizeof(s_line));
        if (len == 0) {
            /* True Ctrl-D / EOF */
            printf("\n[sim] EOF on stdin, exiting\n");
            break;
        }
        if (len < 0) {
            printf("\n[sim] read error on stdin: %s\n", strerror(errno));
            break;
        }

        if (s_line[0] == '\0') continue;
        if (strcmp(s_line, "exit") == 0 || strcmp(s_line, "quit") == 0) {
            printf("[sim] user exit\n");
            break;
        }

        /* Evaluate the line.  mp_embed_exec_str runs `exec(line)`, which
         * does NOT auto-print expression values.  To get REPL behavior
         * (`1+1` -> `2`), wrap in a print/repr if it looks like an
         * expression — the firmware-side micropython_task.c uses the
         * proper REPL parser; we'll do the same later, but for Phase 1
         * a simple heuristic is fine. */
        mp_embed_exec_str(s_line);
    }

    mp_embed_deinit();

    /* End the simulation.  vTaskDelete(NULL) would leave the scheduler
     * running with only idle+timer; we want the process to actually exit
     * so CI can capture the return code. */
    printf("[sim] REPL task done, terminating process\n");
    fflush(stdout);
    exit(0);
}

/* ---- Main ---- */
int main(void) {
    /* Catch Ctrl-C cleanly so the user can interrupt long-running scripts.
     * Default would terminate immediately. */
    signal(SIGINT, sigint_handler);

    printf("[sim] sentai_sim — Phase 1 minimal REPL\n");
    printf("[sim] FreeRTOS POSIX port: tick=%u Hz, heap=%u bytes\n",
           (unsigned) configTICK_RATE_HZ,
           (unsigned) configTOTAL_HEAP_SIZE);
    printf("[sim] MicroPython heap: %u bytes\n", (unsigned) MP_HEAP_SIZE);
    fflush(stdout);

    BaseType_t ok = xTaskCreate(
        repl_task, "repl",
        configMINIMAL_STACK_SIZE * 8,    /* generous: MP can recurse */
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL);
    configASSERT(ok == pdPASS);

    vTaskStartScheduler();

    /* Should never reach here */
    fprintf(stderr, "[sim] FATAL: vTaskStartScheduler returned\n");
    return 1;
}

/* ---- Required FreeRTOS hooks (same pattern as test_freertos.c) ---- */
void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "[sim] FATAL: malloc failed in FreeRTOS heap\n");
    abort();
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void) task;
    fprintf(stderr, "[sim] FATAL: stack overflow in task '%s'\n",
            name ? name : "?");
    abort();
}
#endif

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    static StaticTask_t s_idle_tcb;
    static StackType_t  s_idle_stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &s_idle_tcb;
    *ppxIdleTaskStackBuffer = s_idle_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t s_timer_tcb;
    static StackType_t  s_timer_stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &s_timer_tcb;
    *ppxTimerTaskStackBuffer = s_timer_stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif

/* nlr_jump_fail already provided by embed_util.c — DO NOT redefine */

/* SIM-side stubs for the FreeRTOS critical-section hooks declared in
 * examples/sentai_runtime/mpconfigport.h.  Phase 1 has a single REPL task,
 * so MICROPY_BEGIN/END_ATOMIC_SECTION can be no-ops.  When Phase 3 (crazy
 * bridge over UART socket) lands we'll switch these to real
 * taskENTER_CRITICAL via libfreertos_posix. */
void mp_embed_enter_critical(void) {
    /* no-op in Phase 1 */
}
void mp_embed_exit_critical(void) {
    /* no-op in Phase 1 */
}

/* Help text stub.  On firmware this is a generated symbol from
 * gen_help_embed.py that produces help_txt_data.cc with the SentAI help
 * text in a SDRAM section.  In Phase 1 SIM we just expose a placeholder. */
const char sentai_help_builtin_text[] =
    "SentAI SIM — Phase 1 minimal REPL.\n"
    "MicroPython embed running on FreeRTOS POSIX/Linux port.\n"
    "Most sentai.* bindings are not registered yet (added in Phase 2+).\n"
    "Type expressions, end with Enter.  Ctrl-D to exit.\n";

/* mp_module_sentai stub.  On firmware this is the real module defined in
 * modsentai.c with all its bindings.  In Phase 1 SIM we expose an empty
 * module (the QSTR table still references it because mpconfigport.h
 * is shared with firmware).  `import sentai` succeeds; calling any
 * `sentai.*` function raises AttributeError until Phase 2+ wires bindings. */
const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = NULL,
};

/* Filesystem-import stubs.  Firmware would route these through LFS / FileX
 * (`mp_lexer_new_from_file`, `mp_import_stat`).  Phase 1 SIM has no FS
 * yet — return "not found" so `import foo` from a .py file gracefully
 * raises ImportError. */
mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    (void) filename;
    mp_raise_OSError(2 /* ENOENT */);
}

mp_import_stat_t mp_import_stat(const char *path) {
    (void) path;
    return MP_IMPORT_STAT_NO_EXIST;
}
