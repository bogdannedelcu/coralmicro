// C++ HAL bridge for the 'coral' MicroPython module
// Bridges MicroPython C code to coralmicro C++ APIs

#include "libs/base/console_m7.h"
#include "libs/base/filesystem.h"
#include "libs/base/led.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/littlefs/lfs.h"

#include <cstring>
#include <vector>

extern "C" {

void coral_led_set(int on) {
    coralmicro::LedSet(coralmicro::Led::kUser, on != 0);
}

void coral_sleep_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t coral_ticks_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

int coral_console_read(char* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->Read(buf, size);
}

void coral_console_write(const char* buf, int size) {
    coralmicro::ConsoleM7::GetSingleton()->Write(const_cast<char*>(buf), size);
}

// ===================== Filesystem bridge =====================

// Read file into caller-provided buffer. Returns bytes read, or -1 on error.
int coral_fs_read(const char* path, uint8_t* buf, int max_size) {
    size_t n = coralmicro::LfsReadFile(path, buf, (size_t)max_size);
    return (int)n;
}

// Get file size. Returns -1 if not found.
int coral_fs_size(const char* path) {
    ssize_t s = coralmicro::LfsSize(path);
    return (int)s;
}

// Check if file exists.
int coral_fs_file_exists(const char* path) {
    return coralmicro::LfsFileExists(path) ? 1 : 0;
}

// Check if directory exists.
int coral_fs_dir_exists(const char* path) {
    return coralmicro::LfsDirExists(path) ? 1 : 0;
}

// Write buffer to file. Returns 1 on success, 0 on failure.
int coral_fs_write(const char* path, const uint8_t* buf, int size) {
    return coralmicro::LfsWriteFile(path, buf, (size_t)size) ? 1 : 0;
}

// Remove file or empty directory. Returns 0 on success.
int coral_fs_remove(const char* path) {
    return lfs_remove(coralmicro::Lfs(), path);
}

// Create directories (mkdir -p). Returns 1 on success.
int coral_fs_makedirs(const char* path) {
    return coralmicro::LfsMakeDirs(path) ? 1 : 0;
}

// List directory entries. Calls callback for each entry.
// callback(name, type, size, user_data) - type: 1=file, 2=dir
// Returns number of entries, or -1 on error.
int coral_fs_listdir(const char* path,
                     void (*callback)(const char* name, int type, int size, void* ud),
                     void* user_data) {
    lfs_dir_t dir;
    int err = lfs_dir_open(coralmicro::Lfs(), &dir, path);
    if (err < 0) return -1;

    struct lfs_info info;
    int count = 0;
    while (lfs_dir_read(coralmicro::Lfs(), &dir, &info) > 0) {
        // Skip . and ..
        if (info.name[0] == '.' &&
            (info.name[1] == '\0' || (info.name[1] == '.' && info.name[2] == '\0')))
            continue;
        int t = (info.type == LFS_TYPE_DIR) ? 2 : 1;
        callback(info.name, t, (int)info.size, user_data);
        count++;
    }
    lfs_dir_close(coralmicro::Lfs(), &dir);
    return count;
}

}  // extern "C"
