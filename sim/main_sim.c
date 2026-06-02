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
#include <pthread.h>
#include <time.h>

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
#include "sentai_fs_sim_backend.h"

#include "build_version.h"
#include "examples/sentai_runtime/sentai_log.h"

/* ---- Heap for MicroPython ---- */
#define MP_HEAP_SIZE  (2 * 1024 * 1024)  /* 2 MB — s197 calibration missions
                                          * keep compact summaries/artifacts and
                                          * journal strings in MP long enough to
                                          * serialize them.  The simulator runs
                                          * on Linux, so this stays cheap while
                                          * avoiding misleading summary
                                          * MemoryError fallbacks. */
static char s_mp_heap[MP_HEAP_SIZE];

/* ---- REPL line buffer ---- */
#define REPL_LINE_MAX  1024
static char s_line[REPL_LINE_MAX];

/* ---- SIM debug tee -----------------------------------------------------
 *
 * SIM stdout/stderr are the closest equivalent to firmware printf debug.
 * Keep them visible to the host console, but also tee the same bytes into
 * $SENTAI_FR_DIR/debug.log or $SENTAI_SIM_ROOT/fr/debug.log so each
 * experiment has a reproducible debug trace next to events.csv/scalars.csv.
 */
typedef struct {
    int read_fd;
    int console_fd;
} sim_debug_tee_t;

static sim_debug_tee_t s_stdout_tee = {-1, -1};
static sim_debug_tee_t s_stderr_tee = {-1, -1};

static void sim_write_all(int fd, const char *buf, ssize_t n) {
    ssize_t off = 0;
    while (off < n) {
        ssize_t w = write(fd, buf + off, (size_t)(n - off));
        if (w < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (w == 0) break;
        off += w;
    }
}

static void *sim_debug_tee_thread(void *arg) {
    sigset_t blocked;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGALRM);  /* FreeRTOS POSIX scheduler tick. */
    sigaddset(&blocked, SIGUSR1);  /* FreeRTOS POSIX task resume signal. */
    (void)pthread_sigmask(SIG_BLOCK, &blocked, NULL);

    sim_debug_tee_t *tee = (sim_debug_tee_t *)arg;
    char buf[512];
    for (;;) {
        ssize_t n = read(tee->read_fd, buf, sizeof buf);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        sim_write_all(tee->console_fd, buf, n);
        (void)sentai_log_write(buf, (int)n);
    }
    return NULL;
}

static int sim_debug_tee_one(int stream_fd, sim_debug_tee_t *tee) {
    int p[2];
    if (pipe(p) != 0) return -1;
    tee->console_fd = dup(stream_fd);
    if (tee->console_fd < 0) {
        close(p[0]);
        close(p[1]);
        return -1;
    }
    if (dup2(p[1], stream_fd) < 0) {
        close(tee->console_fd);
        close(p[0]);
        close(p[1]);
        return -1;
    }
    close(p[1]);
    tee->read_fd = p[0];
    pthread_t th;
    if (pthread_create(&th, NULL, sim_debug_tee_thread, tee) != 0) {
        return -1;
    }
    pthread_detach(th);
    return 0;
}

static void sim_debug_tee_start(void) {
    const char *no_tee = getenv("SENTAI_SIM_NO_DEBUG_TEE");
    if (no_tee && no_tee[0] != '\0' && strcmp(no_tee, "0") != 0) {
        setvbuf(stdout, NULL, _IONBF, 0);
        setvbuf(stderr, NULL, _IONBF, 0);
        return;
    }

    if (sim_debug_tee_one(STDOUT_FILENO, &s_stdout_tee) != 0 ||
        sim_debug_tee_one(STDERR_FILENO, &s_stderr_tee) != 0) {
        fprintf(stderr, "[sim] WARN: debug tee setup failed: %s\n",
                strerror(errno));
    }
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

static void sim_debug_tee_flush(void) {
    fflush(stdout);
    fflush(stderr);
}

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

        /* 2. Poll stdin without blocking the FreeRTOS POSIX task.  A raw
         * blocking read()/select() keeps this pthread "running" from the
         * scheduler's perspective and can starve lower-priority SIM tasks
         * while the host is quiet. */
        ssize_t n;
        do {
            n = read(STDIN_FILENO, &c, 1);
        } while (n == -1 && errno == EINTR);
        if (n == 0) {
            if (pos == 0) return 0;     /* true EOF on empty line */
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                struct timespec ts = {0, 1000000L};
                while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {}
                continue;
            }
            return -1;
        }
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

    /* ---- Auto-start observability (OP-S10-W21-T10): sentai.fr.
     * Every SIM session writes events.csv + scalars.csv under
     * $SENTAI_FR_DIR (or $SENTAI_SIM_ROOT/fr/ by default).  Missions
     * may still re-open channels with experiment-specific paths.
     *
     * frames channel left mission-controlled to avoid filling disk with
     * frames during non-FR sessions. */
    {
        extern int sentai_fr_init(void);
        extern int sentai_fr_open(int ch, const char* path);
        extern int sentai_fr_task_start(void);

        const char *no_fr = getenv("SENTAI_SIM_NO_FR");
        if (no_fr && no_fr[0] != '\0' && strcmp(no_fr, "0") != 0) {
            printf("[sim] sentai.fr auto-start disabled by SENTAI_SIM_NO_FR\n");
        } else {
        const char *fr_dir = getenv("SENTAI_FR_DIR");
        char fr_dir_buf[512];
        if (fr_dir == NULL) {
            snprintf(fr_dir_buf, sizeof fr_dir_buf, "%s/fr", sim_fs_root());
            fr_dir = fr_dir_buf;
        }
        struct stat st;
        if (stat(fr_dir, &st) != 0) {
            if (mkdir(fr_dir, 0755) != 0) {
                fprintf(stderr, "[sim] WARN: mkdir %s failed: %s\n",
                        fr_dir, strerror(errno));
            }
        }

        char events_path[600];
        char scalars_path[600];
        char debug_path[600];
        snprintf(events_path,  sizeof events_path,  "%s/events.csv",  fr_dir);
        snprintf(scalars_path, sizeof scalars_path, "%s/scalars.csv", fr_dir);
        snprintf(debug_path,   sizeof debug_path,   "%s/debug.log",   fr_dir);

        int rc_init     = sentai_fr_init();
        int rc_events   = sentai_fr_open(2 /* SENTAI_FR_CH_EVENTS  */, events_path);
        int rc_scalars  = sentai_fr_open(3 /* SENTAI_FR_CH_SCALARS */, scalars_path);
        int rc_debug    = sentai_fr_open(5 /* SENTAI_FR_CH_DEBUG   */, debug_path);
        int rc_task     = sentai_fr_task_start();
        printf("[sim] sentai.fr auto-start: dir=%s init=%d events=%d "
               "scalars=%d debug=%d task=%d\n",
               fr_dir, rc_init, rc_events, rc_scalars, rc_debug, rc_task);
        }
    }

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
    sim_debug_tee_flush();
    /* exit(0) can run POSIX/libusb/stdio cleanup from inside a FreeRTOS task
     * pthread and leave the simulator process alive after the REPL finished.
     * We flushed the debug tee above, so terminate the whole SIM process now. */
    _Exit(0);
}

/* ---- Main ---- */
int main(void) {
    /* Catch Ctrl-C cleanly so the user can interrupt long-running scripts.
     * Default would terminate immediately. */
    signal(SIGINT, sigint_handler);

    sim_debug_tee_start();

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

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
        tskIDLE_PRIORITY,
        NULL);
    configASSERT(ok == pdPASS);

    /* Phase 4: start the camera socket bridge task.  It listens on a UDS
     * (/tmp/sentai_cam.sock) and feeds incoming frames through the same
     * sentai_pxp_scale + sentai_flow_phase_corr_compute pipeline as the
     * ARM firmware.  Safe to start before the scheduler — it's just an
     * xTaskCreateStatic that returns immediately. */
    extern void sim_camera_bridge_start(void);
    const char *no_cam_bridge = getenv("SENTAI_SIM_NO_CAMERA_BRIDGE");
    if (no_cam_bridge && no_cam_bridge[0] != '\0' &&
        strcmp(no_cam_bridge, "0") != 0) {
        printf("[sim] camera bridge disabled by SENTAI_SIM_NO_CAMERA_BRIDGE\n");
    } else {
        sim_camera_bridge_start();
    }

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
    "  sentai.rtos.dmesg()        - in-memory runtime log ring\n"
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
