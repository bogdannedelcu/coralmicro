// B8.3: minimal mphal port for the emulator REPL target.
//
// LPUART6 (M7 debug UART) is the only console transport in this milestone.
// TX uses a polled TDRE wait; RX uses a polled RDRF wait. Neither path uses
// FreeRTOS-aware blocking, which is intentional: the REPL task runs at the
// only awake priority in this build and any blocking would only serialise it
// against itself.

#include <stdint.h>
#include <stddef.h>

#include "py/mphal.h"

#define LPUART6_BASE   0x40090000u
#define LPUART_STAT    (*(volatile uint32_t *)(LPUART6_BASE + 0x14u))
#define LPUART_CTRL    (*(volatile uint32_t *)(LPUART6_BASE + 0x18u))
#define LPUART_DATA    (*(volatile uint32_t *)(LPUART6_BASE + 0x1Cu))

#define LPUART_STAT_RDRF (1u << 21)
#define LPUART_STAT_TDRE (1u << 23)
#define LPUART_CTRL_RE   (1u << 18)
#define LPUART_CTRL_TE   (1u << 19)

void sentai_emu_uart_init(void) {
    LPUART_CTRL = LPUART_CTRL_RE | LPUART_CTRL_TE;
}

static void emu_uart_tx_byte(uint8_t b) {
    // Spin until the TX data register is empty; Renode flushes immediately,
    // so the loop bound below is a safety guard, not a real timeout.
    for (int i = 0; i < 100000; ++i) {
        if (LPUART_STAT & LPUART_STAT_TDRE) {
            break;
        }
    }
    LPUART_DATA = b;
}

mp_uint_t mp_hal_stdout_tx_strn(const char *str, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        emu_uart_tx_byte((uint8_t)str[i]);
    }
    return len;
}

void mp_hal_delay_ms(mp_uint_t ms) {
    extern void vTaskDelay(uint32_t);
    vTaskDelay((ms == 0) ? 1u : ms);
}

mp_uint_t mp_hal_ticks_ms(void) {
    extern uint32_t xTaskGetTickCount(void);
    return (mp_uint_t)xTaskGetTickCount();
}

void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (str[i] == '\n') {
            emu_uart_tx_byte('\r');
        }
        emu_uart_tx_byte((uint8_t)str[i]);
    }
}

void mp_hal_stdout_tx_str(const char *str) {
    while (*str) {
        emu_uart_tx_byte((uint8_t)*str++);
    }
}

int mp_hal_stdin_rx_chr(void) {
    while ((LPUART_STAT & LPUART_STAT_RDRF) == 0u) {
        // Spin. Renode triggers RDRF when a byte is injected via
        // `lpuart6 WriteChar`.
    }
    return (int)(LPUART_DATA & 0xFFu);
}
