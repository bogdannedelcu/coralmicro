// C++ HAL bridge for the 'sentai' MicroPython module
// Bridges MicroPython C code to coralmicro C++ APIs

#include "libs/base/console_m7.h"
#include "libs/base/filesystem.h"
#include "libs/base/led.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/littlefs/lfs.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpuart.h"

#include <cstring>
#include <vector>

extern "C" {

void sentai_led_set(int on) {
    coralmicro::LedSet(coralmicro::Led::kUser, on != 0);
}

void sentai_sleep_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t sentai_ticks_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

int sentai_console_read(char* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->Read(buf, size);
}

void sentai_console_write(const char* buf, int size) {
    coralmicro::ConsoleM7::GetSingleton()->Write(const_cast<char*>(buf), size);
}

// ===================== Filesystem bridge =====================
// All filesystem operations use the USER LFS partition.
// The system partition (with default.elf) is not accessible from Python.

// Force-format the user LFS partition. Returns 1 on success, 0 on failure.
int sentai_fs_format(void) {
    return coralmicro::LfsUserInit(/*force_format=*/true) ? 1 : 0;
}

// Read file into caller-provided buffer. Returns bytes read, or -1 on error.
int sentai_fs_read(const char* path, uint8_t* buf, int max_size) {
    size_t n = coralmicro::LfsUserReadFile(path, buf, (size_t)max_size);
    return (int)n;
}

// Get file size. Returns -1 if not found.
int sentai_fs_size(const char* path) {
    ssize_t s = coralmicro::LfsUserSize(path);
    return (int)s;
}

// Check if file exists.
int sentai_fs_file_exists(const char* path) {
    return coralmicro::LfsUserFileExists(path) ? 1 : 0;
}

// Check if directory exists.
int sentai_fs_dir_exists(const char* path) {
    return coralmicro::LfsUserDirExists(path) ? 1 : 0;
}

// Write buffer to file. Returns 1 on success, 0 on failure.
int sentai_fs_write(const char* path, const uint8_t* buf, int size) {
    return coralmicro::LfsUserWriteFile(path, buf, (size_t)size) ? 1 : 0;
}

// Remove file or empty directory. Returns 0 on success.
int sentai_fs_remove(const char* path) {
    return coralmicro::LfsUserRemove(path);
}

// Create directories (mkdir -p). Returns 1 on success.
int sentai_fs_makedirs(const char* path) {
    return coralmicro::LfsUserMakeDirs(path) ? 1 : 0;
}

// List directory entries. Calls callback for each entry.
// callback(name, type, size, user_data) - type: 1=file, 2=dir
// Returns number of entries, or -1 on error.
int sentai_fs_listdir(const char* path,
                     void (*callback)(const char* name, int type, int size, void* ud),
                     void* user_data) {
    lfs_dir_t dir;
    int err = lfs_dir_open(coralmicro::LfsUser(), &dir, path);
    if (err < 0) return -1;

    struct lfs_info info;
    int count = 0;
    while (lfs_dir_read(coralmicro::LfsUser(), &dir, &info) > 0) {
        // Skip . and ..
        if (info.name[0] == '.' &&
            (info.name[1] == '\0' || (info.name[1] == '.' && info.name[2] == '\0')))
            continue;
        int t = (info.type == LFS_TYPE_DIR) ? 2 : 1;
        callback(info.name, t, (int)info.size, user_data);
        count++;
    }
    lfs_dir_close(coralmicro::LfsUser(), &dir);
    return count;
}

// ===================== USB Serial bridge =====================
// Reuses the console's CDC ACM port (ttyACM0 on host).
// When open: printf goes to UART only, USB is reserved for Python.
// When closed: normal console (printf → UART + USB).

int sentai_usb_serial_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbSerialOpen() ? 1 : 0;
}

void sentai_usb_serial_close(void) {
    coralmicro::ConsoleM7::GetSingleton()->UsbSerialClose();
}

int sentai_usb_serial_is_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbSerialIsOpen() ? 1 : 0;
}

int sentai_usb_serial_write(const uint8_t* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbTransmit(buf, (size_t)size) ? size : -1;
}

int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbRead(buf, max_size, timeout_ms);
}

int sentai_usb_serial_available(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbAvailable();
}

// ===================== REPL target control =====================

int sentai_console_set_target(int target) {
    using RT = coralmicro::ConsoleM7::ReplTarget;
    auto t = (target == 0) ? RT::kUsb : RT::kUart;
    coralmicro::ConsoleM7::GetSingleton()->SetReplTarget(t);
    return 0;
}

int sentai_console_get_target(void) {
    using RT = coralmicro::ConsoleM7::ReplTarget;
    return (coralmicro::ConsoleM7::GetSingleton()->GetReplTarget() == RT::kUsb) ? 0 : 1;
}

// ===================== UART Serial bridge =====================

int sentai_uart_serial_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartSerialOpen() ? 1 : 0;
}

void sentai_uart_serial_close(void) {
    coralmicro::ConsoleM7::GetSingleton()->UartSerialClose();
}

int sentai_uart_serial_is_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartSerialIsOpen() ? 1 : 0;
}

int sentai_uart_serial_write(const uint8_t* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->UartTransmit(buf, (size_t)size) ? size : -1;
}

int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    return coralmicro::ConsoleM7::GetSingleton()->UartRead(buf, max_size, timeout_ms);
}

int sentai_uart_serial_available(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartAvailable();
}

// ===================== UART baudrate =====================
// LPUART6 is the M7 debug console UART.
// Clock source: OSC_RC_48M_DIV2 = 24 MHz (configured in board init).

#define LPUART6_CLOCK_FREQ  24000000U

void sentai_uart_set_baudrate(uint32_t baudrate) {
    LPUART_SetBaudRate(LPUART6, baudrate, LPUART6_CLOCK_FREQ);
}

void sentai_uart_restore_baudrate(void) {
    LPUART_SetBaudRate(LPUART6, 115200U, LPUART6_CLOCK_FREQ);
}

// ===================== Help file (system partition) =====================

int sentai_help_read(char* buf, int max_size) {
    size_t n = coralmicro::LfsReadFile(
        "examples/sentai_runtime/help.txt",
        reinterpret_cast<uint8_t*>(buf), (size_t)(max_size - 1));
    if (n > 0) {
        buf[n] = '\0';
        return (int)n;
    }
    return -1;
}

}  // extern "C"
