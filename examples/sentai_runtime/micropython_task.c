// MicroPython FreeRTOS task implementation
// Supports both fixed-script execution and interactive REPL over serial.
// Ctrl+C (0x03) interrupts running scripts via mp_sched_keyboard_interrupt().

#include "micropython_task.h"
#include "port/micropython_embed.h"
#include "py/runtime.h"

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include <stdio.h>
#include <string.h>
#include "build_version.h"

// GC heap size for MicroPython (64 KB for import support + scripts)
#define MP_GC_HEAP_SIZE (64 * 1024)

// REPL line buffer size
#define REPL_LINE_MAX 256
// Multi-line block buffer size
#define REPL_BLOCK_MAX 2048

// Command history
#define HISTORY_SIZE 20
static char history[HISTORY_SIZE][REPL_LINE_MAX];
static int history_count = 0;   // total lines ever added
static int history_write = 0;   // next circular slot to write

static void history_add(const char* line) {
    if (line[0] == '\0') return;
    // Don't add duplicates of the last entry
    if (history_count > 0) {
        int last = (history_write + HISTORY_SIZE - 1) % HISTORY_SIZE;
        if (strcmp(history[last], line) == 0) return;
    }
    strncpy(history[history_write], line, REPL_LINE_MAX - 1);
    history[history_write][REPL_LINE_MAX - 1] = '\0';
    history_write = (history_write + 1) % HISTORY_SIZE;
    if (history_count < HISTORY_SIZE) history_count++;
}

// Get history entry by index (0 = most recent, 1 = one before, etc.)
static const char* history_get(int idx) {
    if (idx < 0 || idx >= history_count) return NULL;
    int pos = (history_write - 1 - idx + HISTORY_SIZE * 2) % HISTORY_SIZE;
    return history[pos];
}

// Console read/write - implemented in modsentai_hal.cc (C++ bridge)
extern int sentai_console_read(char* buf, int size);
extern void sentai_console_write(const char* buf, int size);

// ===================== Ctrl+C monitor =====================
// A small FreeRTOS task that polls serial for 0x03 (Ctrl+C) while a
// Python script is executing, and raises KeyboardInterrupt in the VM.

static volatile int ctrlc_monitor_running = 0;
static TaskHandle_t ctrlc_monitor_handle = NULL;

static void ctrlc_monitor_task(void* param) {
    (void)param;
    char ch;
    while (ctrlc_monitor_running) {
        int n = sentai_console_read(&ch, 1);
        if (n == 1 && ch == 0x03) {
            mp_sched_keyboard_interrupt();
            // Small delay to avoid flooding interrupts
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    vTaskDelete(NULL);
}

static void ctrlc_monitor_start(void) {
    ctrlc_monitor_running = 1;
    xTaskCreate(ctrlc_monitor_task, "ctrlc",
                512 / sizeof(StackType_t),
                NULL, tskIDLE_PRIORITY + 2,
                &ctrlc_monitor_handle);
}

static void ctrlc_monitor_stop(void) {
    ctrlc_monitor_running = 0;
    // Give the monitor task time to exit
    vTaskDelay(pdMS_TO_TICKS(50));
    ctrlc_monitor_handle = NULL;
}

// Wrapper: execute a script string with Ctrl+C monitoring active
static void mp_exec_str_with_ctrlc(const char* src) {
    ctrlc_monitor_start();
    mp_embed_exec_str(src);
    ctrlc_monitor_stop();
}

// ---------- Helper: read one char from serial, blocking with yield ----------
static int repl_getchar(void) {
    char ch;
    while (1) {
        int n = sentai_console_read(&ch, 1);
        if (n == 1) return (unsigned char)ch;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------- Helper: read one char with timeout (ms). Returns -1 on timeout ----------
static int repl_getchar_timeout(int timeout_ms) {
    char ch;
    int waited = 0;
    while (waited < timeout_ms) {
        int n = sentai_console_read(&ch, 1);
        if (n == 1) return (unsigned char)ch;
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
    return -1;
}

// ---------- Helper: consume remaining bytes of an escape sequence ----------
// After reading ESC [ <ch3>, some sequences have more bytes (e.g. ESC[1;5C).
// Consume everything until a final letter (A-Z, a-z, ~) or timeout.
static void repl_consume_escape_tail(int ch3) {
    // If ch3 is already a final character (letter or ~), sequence is complete
    if ((ch3 >= 'A' && ch3 <= 'Z') || (ch3 >= 'a' && ch3 <= 'z') || ch3 == '~')
        return;
    // Otherwise consume until final char or timeout
    while (1) {
        int c = repl_getchar_timeout(50);
        if (c < 0) break;  // timeout - done
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '~')
            break;  // final char consumed
    }
}

// ---------- Helper: write string to serial ----------
static void repl_puts(const char* s) {
    sentai_console_write(s, strlen(s));
}

// ---------- Helper: clear current line on terminal and rewrite ----------
static void repl_replace_line(char* buf, int* pos, const char* new_line) {
    // Erase current line: move cursor back, overwrite with spaces, move back again
    while (*pos > 0) {
        repl_puts("\b \b");
        (*pos)--;
    }
    int new_len = strlen(new_line);
    if (new_len >= REPL_LINE_MAX) new_len = REPL_LINE_MAX - 1;
    memcpy(buf, new_line, new_len);
    buf[new_len] = '\0';
    *pos = new_len;
    sentai_console_write(buf, new_len);
}

// ---------- Helper: read a line with echo + backspace + history ----------
// Returns line length (without null terminator). Line does NOT include \n.
// Returns -1 on Ctrl+C.
static int repl_readline(char* buf, int max_len) {
    int pos = 0;
    int hist_idx = -1;  // -1 = not browsing history

    while (pos < max_len - 1) {
        int ch = repl_getchar();

        if (ch == '\r' || ch == '\n') {
            repl_puts("\r\n");
            break;
        }
        if (ch == 0x7F || ch == '\b') {
            // Backspace
            if (pos > 0) {
                pos--;
                repl_puts("\b \b");
            }
            continue;
        }
        if (ch == 0x03) {
            // Ctrl+C - cancel current input
            repl_puts("\r\nKeyboardInterrupt\r\n");
            buf[0] = '\0';
            return -1;
        }
        if (ch == 0x1B) {
            // Escape sequence - read with timeout to avoid blocking
            int ch2 = repl_getchar_timeout(100);
            if (ch2 < 0) {
                // Bare ESC pressed - ignore
                continue;
            }
            if (ch2 == '[' || ch2 == 'O') {
                int ch3 = repl_getchar_timeout(100);
                if (ch3 < 0) {
                    // Incomplete sequence - ignore
                    continue;
                }
                if (ch3 == 'A') {
                    // Up arrow - older history
                    if (hist_idx + 1 < history_count) {
                        hist_idx++;
                        const char* h = history_get(hist_idx);
                        if (h) repl_replace_line(buf, &pos, h);
                    }
                } else if (ch3 == 'B') {
                    // Down arrow - newer history
                    if (hist_idx > 0) {
                        hist_idx--;
                        const char* h = history_get(hist_idx);
                        if (h) repl_replace_line(buf, &pos, h);
                    } else if (hist_idx == 0) {
                        hist_idx = -1;
                        repl_replace_line(buf, &pos, "");
                    }
                } else {
                    // Other escape sequence (Left/Right/Home/End/Delete/F-keys etc.)
                    // Consume any remaining bytes to avoid garbage in input
                    repl_consume_escape_tail(ch3);
                }
            }
            // else: ESC followed by something unexpected - just ignore
            continue;
        }
        if (ch >= 0x20 && ch < 0x7F) {
            buf[pos++] = (char)ch;
            // Echo
            char echo = (char)ch;
            sentai_console_write(&echo, 1);
        }
    }
    buf[pos] = '\0';
    return pos;
}

// ---------- Check if a line starts a multi-line block ----------
// Returns 1 if line ends with ':' (for, if, while, def, class, etc.)
static int is_block_start(const char* line) {
    int len = strlen(line);
    // Skip trailing whitespace
    while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\t'))
        len--;
    return (len > 0 && line[len-1] == ':');
}

// ---------- Check if a line is blank (only whitespace) ----------
static int is_blank(const char* line) {
    while (*line) {
        if (*line != ' ' && *line != '\t') return 0;
        line++;
    }
    return 1;
}

// ===================== Fixed-script task =====================

typedef struct {
    const char* script;
} mp_task_params_t;

static void micropython_task(void* param) {
    mp_task_params_t* params = (mp_task_params_t*)param;
    const char* script = params->script;

    printf("[MicroPython] Task started\r\n");

    static char gc_heap[MP_GC_HEAP_SIZE];
    int stack_top;
    mp_embed_init(&gc_heap[0], sizeof(gc_heap), &stack_top);

    printf("[MicroPython] Executing script...\r\n");
    mp_exec_str_with_ctrlc(script);
    printf("[MicroPython] Script finished\r\n");

    mp_embed_deinit();

    vPortFree(params);
    vTaskDelete(NULL);
}

void micropython_start_task(const char* script, unsigned int stack_size,
                            unsigned int priority) {
    mp_task_params_t* params =
        (mp_task_params_t*)pvPortMalloc(sizeof(mp_task_params_t));
    if (!params) {
        printf("[MicroPython] ERROR: Failed to allocate task params\r\n");
        return;
    }
    params->script = script;

    BaseType_t ret = xTaskCreate(
        micropython_task, "mp_task",
        stack_size / sizeof(StackType_t),
        params, priority, NULL);

    if (ret != pdPASS) {
        printf("[MicroPython] ERROR: Failed to create task\r\n");
        vPortFree(params);
    }
}

// ===================== REPL task =====================

static void micropython_repl_task(void* param) {
    (void)param;

    static char gc_heap[MP_GC_HEAP_SIZE];
    int stack_top;

    printf("[MicroPython] REPL task started\r\n");
    mp_embed_init(&gc_heap[0], sizeof(gc_heap), &stack_top);

    // Populate sys.path so 'import mymodule' finds /mymodule.py on LittleFS
    mp_obj_list_append(mp_sys_path, mp_obj_new_str("/", 1));
    mp_obj_list_append(mp_sys_path, mp_obj_new_str("/lib", 4));

    // Auto-import sentai so it's always available in the REPL and main.py
    // This also makes all sub-modules (sentai.io, sentai.fs, etc.) accessible.
    mp_embed_exec_str("import sentai");

    // Auto-run /main.py if it exists on the user partition
    // (skip if USB drive is active — LFS is unmounted)
    {
        extern int sentai_fs_file_exists(const char* path);
        extern int sentai_fs_size(const char* path);
        extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
        extern int sentai_usb_drive_get(void);

        if (!sentai_usb_drive_get() && sentai_fs_file_exists("/main.py")) {
            int size = sentai_fs_size("/main.py");
            if (size > 0) {
                uint8_t* buf = (uint8_t*)m_new(byte, size + 1);
                int n = sentai_fs_read("/main.py", buf, size);

                if (n > 0) {
                    buf[n] = '\0';
                    repl_puts("\r\n[main.py] Running...\r\n");
                    mp_exec_str_with_ctrlc((const char*)buf);
                    repl_puts("[main.py] Finished.\r\n");
                }
                m_del(byte, buf, size + 1);
            }
        }
    }

    repl_puts("\r\n");
    {
        char banner[128];
        snprintf(banner, sizeof(banner),
            "MicroPython REPL on SentAI board v1.0  [build #%d  %s]\r\n",
            BUILD_VERSION, BUILD_TIMESTAMP);
        repl_puts(banner);
    }
    repl_puts("Ctrl+C to interrupt running code or cancel input.\r\n");
    repl_puts("\r\n");

    // Flush captured boot output to /log/boot.log on LittleFS user partition,
    // then switch debug to silent mode for interactive use.
    {
        extern void boot_log_flush(void);
        extern void sentai_debug_set(int level);
        boot_log_flush();
        sentai_debug_set(0);
    }

    static char line[REPL_LINE_MAX];
    static char block[REPL_BLOCK_MAX];

    while (1) {
        repl_puts(">>> ");
        int len = repl_readline(line, sizeof(line));

        if (len <= 0) {
            // Empty line or Ctrl+C - do nothing, show new prompt
            continue;
        }

        // Check if this starts a multi-line block
        if (is_block_start(line)) {
            // Start accumulating a block
            int off = 0;
            int cancelled = 0;
            off += snprintf(block + off, sizeof(block) - off, "%s\n", line);

            while (off < (int)sizeof(block) - REPL_LINE_MAX) {
                repl_puts("... ");
                len = repl_readline(line, sizeof(line));

                if (len == -1) {
                    // Ctrl+C during block input - cancel block
                    cancelled = 1;
                    break;
                }

                if (len == 0 || is_blank(line)) {
                    // Empty line ends the block
                    break;
                }

                off += snprintf(block + off, sizeof(block) - off, "%s\n", line);
            }

            if (!cancelled) {
                block[off] = '\0';
                // Add first line of block to history
                char first_line[REPL_LINE_MAX];
                const char* nl = strchr(block, '\n');
                int flen = nl ? (int)(nl - block) : (int)strlen(block);
                if (flen >= REPL_LINE_MAX) flen = REPL_LINE_MAX - 1;
                memcpy(first_line, block, flen);
                first_line[flen] = '\0';
                history_add(first_line);
                mp_exec_str_with_ctrlc(block);
            }
        } else {
            // Single line - execute with Ctrl+C monitoring
            history_add(line);
            mp_exec_str_with_ctrlc(line);
        }
    }

    // Never reached, but for completeness:
    mp_embed_deinit();
    vTaskDelete(NULL);
}

void micropython_start_repl_task(unsigned int stack_size,
                                 unsigned int priority) {
    BaseType_t ret = xTaskCreate(
        micropython_repl_task, "mp_repl",
        stack_size / sizeof(StackType_t),
        NULL, priority, NULL);

    if (ret != pdPASS) {
        printf("[MicroPython] ERROR: Failed to create REPL task\r\n");
    } else {
        printf("[MicroPython] REPL task created (stack=%u, priority=%u)\r\n",
               stack_size, priority);
    }
}
