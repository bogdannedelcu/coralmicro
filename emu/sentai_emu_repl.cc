// B8.3 ARM emulator REPL spike.
//
// Boots CM7 startup -> ARM FreeRTOS -> single REPL task on LPUART6 -> minimal
// MicroPython that can evaluate `1+1` and print `2`. No filesystem, no
// `sentai.*` hardware bindings, no USB CDC, no camera.
//
// The REPL loop is intentionally cherry-picked from
// `examples/sentai_runtime/micropython_task.c` (single-line variant only).
// History, escape sequences, multi-line blocks, Ctrl+C handling and the
// watchdog hook are all deferred — B8.3 only proves that:
//
//   * LPUART6 RX delivers bytes into mp_hal_stdin_rx_chr;
//   * mp_embed_init reaches a working VM;
//   * mp_embed_exec_str compiles and runs a single-line Python expression;
//   * mp_hal_stdout_tx_strn_cooked drives LPUART6 TX out to the file backend.

#include <stdint.h>
#include <string.h>

extern "C" {
#include "py/compile.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/stackctrl.h"
#include "port/micropython_embed.h"
}

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" void sentai_emu_uart_init(void);

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_repl_lines;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootReplRunning = 0x0400;
constexpr uint32_t kBootReplBanner = 0x0500;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr size_t kReplStackWords = 8 * 1024;        // 32 KiB stack
constexpr size_t kReplLineMax = 256;
constexpr size_t kGcHeapBytes = 32 * 1024;          // 32 KiB GC heap

StaticTask_t g_repl_tcb;
StackType_t g_repl_stack[kReplStackWords] __attribute__((aligned(8)));
uint8_t g_gc_heap[kGcHeapBytes] __attribute__((aligned(8)));
char g_repl_line[kReplLineMax];

void ReplPutString(const char *s) {
    mp_hal_stdout_tx_strn(s, strlen(s));
}

int ReplReadLine(char *buf, size_t max_len) {
    size_t pos = 0;
    while (pos + 1 < max_len) {
        int ch = mp_hal_stdin_rx_chr();
        if (ch < 0) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            ReplPutString("\r\n");
            break;
        }
        if (ch == 0x7F || ch == '\b') {
            if (pos > 0) {
                --pos;
                ReplPutString("\b \b");
            }
            continue;
        }
        if (ch >= 0x20 && ch < 0x7F) {
            buf[pos++] = static_cast<char>(ch);
            char echo = static_cast<char>(ch);
            mp_hal_stdout_tx_strn(&echo, 1);
        }
    }
    buf[pos] = '\0';
    return static_cast<int>(pos);
}

void ReplTask(void *) {
    g_sentai_emu_boot_state = kBootReplRunning;
    sentai_emu_uart_init();
    ReplPutString("\r\nSentAI EMU REPL B8.3\r\n");

    // Stack-top hint: use a local variable address inside this task's stack.
    int stack_top_marker = 0;
    mp_stack_set_top(&stack_top_marker);
    mp_stack_set_limit((kReplStackWords - 256) * sizeof(StackType_t));
    mp_embed_init(g_gc_heap, sizeof(g_gc_heap), &stack_top_marker);

    ReplPutString("MicroPython embed ready\r\n");
    g_sentai_emu_boot_state = kBootReplBanner;

    while (true) {
        ReplPutString(">>> ");
        int len = ReplReadLine(g_repl_line, sizeof(g_repl_line));
        g_sentai_emu_last_tick = xTaskGetTickCount();
        if (len <= 0) {
            continue;
        }
        ++g_sentai_emu_repl_lines;
        mp_embed_exec_str(g_repl_line);
        ++g_sentai_emu_heartbeat;
    }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_repl_lines = 0;
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t task =
        xTaskCreateStatic(ReplTask, "emu_repl", kReplStackWords, nullptr,
                          tskIDLE_PRIORITY + 1, g_repl_stack, &g_repl_tcb);
    if (!task) {
        g_sentai_emu_boot_state = kBootCreateTaskFailed;
        while (true) {
        }
    }
    g_sentai_emu_boot_state = kBootTaskCreated;

    vTaskStartScheduler();

    g_sentai_emu_boot_state = kBootSchedulerReturned;
    while (true) {
    }
}
