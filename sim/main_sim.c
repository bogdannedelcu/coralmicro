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

#include <sys/stat.h>

#include "py/runtime.h"
#include "py/lexer.h"
#include "py/reader.h"
#include "py/builtin.h"
#include "py/objmodule.h"
#include "py/gc.h"
#include "port/micropython_embed.h"

/* Provided by modsentai_sim.c — same path resolver `sentai.fs.*` uses. */
extern const char *sim_fs_root(void);
extern int sim_fs_resolve(const char *bpath, char *out, size_t outsz);

#include "build_version.h"

/* ---- Heap for MicroPython ---- */
#define MP_HEAP_SIZE  (512 * 1024)   /* 512 KB — doubled 2026-05-16: hex_helpers
                                       * hex_image() 16 KB transient × 3 waypoints +
                                       * crtp_log._toc 11 KB + compute_phog/gist
                                       * returns fragmented the old 256 KB.  6 KB
                                       * allocations failed mid-mission. */
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

/* ---- EINTR-resilient line reader from stdin + REPL-TUNNEL fifo ----
 *
 * The FreeRTOS POSIX port delivers SIGALRM at configTICK_RATE_HZ (1 kHz)
 * to drive the scheduler tick.  This interrupts any blocking read()
 * syscall on stdin with errno=EINTR.  glibc's fgets() does NOT retry on
 * EINTR — it returns NULL and sets feof(), making the REPL think the user
 * pressed Ctrl-D after every single tick.
 *
 * Workaround: select(stdin) with a short timeout in an EINTR-retry loop,
 * then read(STDIN_FILENO, ...) ONE BYTE.  Also poll the
 * `sentai_link_repl_rx_pop` FIFO each iteration so commands arriving over
 * MAVLink TUNNEL (host pymavlink → sentai_link reader task → FIFO) are
 * dispatched into the REPL the same way as typed input.
 *
 * Returns 0 on EOF, -1 on real error, or line length on success (line
 * null-terminated, newline stripped, >=1). */
extern int sentai_link_repl_rx_pop(char *out, int max_n);

static int sim_read_line(char *buf, size_t max_len) {
    size_t pos = 0;
    while (pos < max_len - 1) {
        char c;

        /* 1. Drain the radio FIFO first — non-blocking. */
        if (sentai_link_repl_rx_pop(&c, 1) == 1) {
            if (c == '\n') break;
            if (c == '\r') continue;
            buf[pos++] = c;
            continue;
        }

        /* 2. Otherwise wait briefly for stdin or signal. */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = {0, 50000};   /* 50 ms — polls FIFO @ 20 Hz */
        int sel = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;   /* SIGALRM tick — retry */
            return -1;
        }
        if (sel == 0) continue;     /* timeout — go check FIFO again */

        ssize_t n;
        do {
            n = read(STDIN_FILENO, &c, 1);
        } while (n == -1 && errno == EINTR);
        if (n == 0) {
            if (pos == 0) return 0;     /* true EOF on empty line */
            break;
        }
        if (n < 0) return -1;
        if (c == '\n') break;
        if (c == '\r') continue;
        buf[pos++] = c;
    }
    buf[pos] = '\0';
    return (int) pos + 1;
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

    /* Auto-import sentai so the REPL can use sentai.* without an explicit
     * `import sentai` first.  Mirrors examples/sentai_runtime/
     * micropython_task.c:micropython_repl_task() which does the same on
     * the firmware. */
    mp_embed_exec_str("import sentai");

    /* Make `/` (the SIM virtual FS root) importable.  `import hover_logic`
     * then finds `<sim_fs_root>/hover_logic.py` via mp_import_stat above.
     * On firmware this is set up by the LFS-init path. */
    mp_embed_exec_str("import sys\nsys.path.append('/')\nsys.path.append('')\n");

    printf("\n");
    printf("MicroPython on SentAI SIM (FreeRTOS POSIX port + MicroPython embed)\n");
    printf("Type expressions, end with Enter.  Ctrl-D or 'exit' to quit.\n");
    printf("Try: sentai.version(), sentai.fs.write(...), help()\n");
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

    printf("[sim] sentai_sim build #%d (%s)\n", BUILD_VERSION, BUILD_TIMESTAMP);
    printf("[sim] FreeRTOS POSIX port: tick=%u Hz, heap=%u bytes\n",
           (unsigned) configTICK_RATE_HZ,
           (unsigned) configTOTAL_HEAP_SIZE);
    printf("[sim] MicroPython heap: %u bytes\n", (unsigned) MP_HEAP_SIZE);
    fflush(stdout);

    /* Initialize the sentai_prep slot table (refcounts=0, frame_div=1).
     * BSS-zero is defensive; explicit init keeps the contract clean
     * per [[op-s10-w11-prep-pipeline]]. */
    extern void sentai_prep_init(void);
    sentai_prep_init();

    BaseType_t ok = xTaskCreate(
        repl_task, "repl",
        configMINIMAL_STACK_SIZE * 8,    /* generous: MP can recurse */
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL);
    configASSERT(ok == pdPASS);

    /* Phase 4: start the camera socket bridge task.  It listens on a UDS
     * (/tmp/sentai_cam.sock) and feeds incoming frames through the same
     * sentai_pxp_scale + sentai_flow_phase_corr_compute pipeline as the
     * ARM firmware.  Safe to start before the scheduler — it's just an
     * xTaskCreateStatic that returns immediately. */
    extern void sim_camera_bridge_start(void);
    sim_camera_bridge_start();

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

/* Help text — shown by MicroPython's builtin help().  On firmware this
 * is generated from examples/sentai_runtime/help.txt by gen_help_embed.py.
 * SIM uses a curated short version — the firmware help text references
 * many bindings that aren't wired in SIM yet. */
const char sentai_help_builtin_text[] =
    "Welcome to SentAI SIM (MicroPython on FreeRTOS POSIX/Linux port).\n"
    "\n"
    "Available bindings (see modsentai_sim.c for the full list):\n"
    "  sentai.version()           - build identifier\n"
    "  sentai.verbose([on])       - silence/restore [SIM] log output\n"
    "  sentai.io.led_on/off()     - LED stub (printf to stdout)\n"
    "  sentai.rtos.sleep_ms(ms)   - vTaskDelay\n"
    "  sentai.diag.dmesg()        - in-memory log ring (4 KB)\n"
    "  sentai.sys.reset()         - clean exit\n"
    "  sentai.fs.write/append/read/read_str(path[, bytes])\n"
    "  sentai.fs.exists/size/ls/mkdir/remove/sync()\n"
    "    -- backed by ./sentai_sim_root/ (override SENTAI_SIM_ROOT env)\n"
    "\n"
    "Not yet in SIM (Phase 3+):\n"
    "  sentai.crazy.*  (Crazyflie radio bridge over UART socket)\n"
    "  sentai.flow.*   (optical flow on Gazebo camera frames)\n"
    "  sentai.tpu.*    (libedgetpu Linux for EdgeTPU acceleration)\n"
    "\n"
    "REPL: type expression + Enter.  Ctrl-D or sentai.sys.reset() exits.\n";

/* mp_module_sentai is now defined in modsentai_sim.c with real bindings. */

/* Filesystem-import bridge — Phase 5b.
 *
 * `import foo` walks `sys.path`, calling `mp_import_stat()` on each candidate
 * path (e.g. "/foo.py") and then `mp_lexer_new_from_file()` once found.  We
 * route both through `sim_fs_resolve()` so the SIM virtual FS (the same
 * `sentai.fs.*` files see) is the single source of truth.  This lets test
 * scripts use native `import hover_logic` — no `exec(read_str(...))`, no
 * source-string heap copy.
 *
 * The firmware equivalent lives in `examples/sentai_runtime/sentai_runtime.cc`
 * and routes through FileX (`FxUserOpenRead`); behaviour and on-disk layout
 * are identical so the SAME .py files work on both targets. */

#define SIM_IMPORT_PATH_MAX 512

/* Minimal POSIX-fd-backed mp_reader — the embed library's version is gated
 * by MICROPY_READER_POSIX which we leave off (firmware doesn't use it).
 * 32-byte buffer is plenty: lexer pulls 1 byte at a time and refills. */
typedef struct sim_reader_fd_t {
    int fd;
    size_t len;
    size_t pos;
    unsigned char buf[64];
} sim_reader_fd_t;

static mp_uint_t sim_reader_fd_readbyte(void *data) {
    sim_reader_fd_t *r = (sim_reader_fd_t *)data;
    if (r->pos >= r->len) {
        ssize_t n = read(r->fd, r->buf, sizeof r->buf);
        if (n <= 0) return MP_READER_EOF;
        r->len = (size_t)n;
        r->pos = 0;
    }
    return r->buf[r->pos++];
}

static void sim_reader_fd_close(void *data) {
    sim_reader_fd_t *r = (sim_reader_fd_t *)data;
    if (r->fd >= 0) close(r->fd);
    m_del_obj(sim_reader_fd_t, r);
}

mp_import_stat_t mp_import_stat(const char *path) {
    char resolved[SIM_IMPORT_PATH_MAX + 1];
    if (sim_fs_resolve(path, resolved, sizeof resolved) != 0) {
        return MP_IMPORT_STAT_NO_EXIST;
    }
    struct stat st;
    if (stat(resolved, &st) != 0) {
        return MP_IMPORT_STAT_NO_EXIST;
    }
    if (S_ISDIR(st.st_mode)) return MP_IMPORT_STAT_DIR;
    if (S_ISREG(st.st_mode)) return MP_IMPORT_STAT_FILE;
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    const char *bpath = qstr_str(filename);
    char resolved[SIM_IMPORT_PATH_MAX + 1];
    if (sim_fs_resolve(bpath, resolved, sizeof resolved) != 0) {
        mp_raise_OSError(ENOENT);
    }
    int fd = open(resolved, O_RDONLY);
    if (fd < 0) {
        mp_raise_OSError_with_filename(errno, bpath);
    }
    sim_reader_fd_t *r = m_new_obj(sim_reader_fd_t);
    r->fd = fd;
    r->len = 0;
    r->pos = 0;
    mp_reader_t reader = {
        .data = r,
        .readbyte = sim_reader_fd_readbyte,
        .close = sim_reader_fd_close,
    };
    return mp_lexer_new(filename, reader);
}
