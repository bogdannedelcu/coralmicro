// MicroPython port hooks for importing .py files from LittleFS
// Implements mp_import_stat() and mp_lexer_new_from_file() so that
// 'import mymodule' finds and loads /mymodule.py from the user partition.

#include "py/builtin.h"
#include "py/lexer.h"
#include "py/reader.h"
#include "py/runtime.h"

#include <string.h>

// Filesystem HAL (modsentai_hal.cc)
extern int sentai_fs_file_exists(const char* path);
extern int sentai_fs_dir_exists(const char* path);
extern int sentai_fs_size(const char* path);
extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);

// USB guard (main_freertos_m7.cc)
extern int sentai_usb_drive_get(void);

// ---------- mp_import_stat ----------
// Called by the import machinery to check if a path exists on the filesystem.
// Paths arrive as absolute (e.g. "/mymodule", "/mymodule.py", "/lib/utils.py")
// because sys.path entries are absolute.

mp_import_stat_t mp_import_stat(const char* path) {
    // If USB drive is active, LFS is unmounted — cannot check files.
    if (sentai_usb_drive_get()) return MP_IMPORT_STAT_NO_EXIST;

    mp_import_stat_t result = MP_IMPORT_STAT_NO_EXIST;
    if (sentai_fs_dir_exists(path)) {
        result = MP_IMPORT_STAT_DIR;
    } else if (sentai_fs_file_exists(path)) {
        result = MP_IMPORT_STAT_FILE;
    }

    return result;
}

// ---------- mp_lexer_new_from_file ----------
// Called when the import machinery needs to compile a .py file.
// Reads the entire file from LittleFS into a GC-managed buffer,
// creates a memory reader, and returns a lexer.

mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    const char* path = qstr_str(filename);

    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("flash busy: call sentai.usb.drive(0) first"));
    }

    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }

    byte* buf = m_new(byte, size);
    int n = sentai_fs_read(path, buf, size);

    if (n <= 0) {
        m_del(byte, buf, size);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }

    // Create a memory reader that owns the buffer (free_len = size).
    // When the lexer is done, the reader's close() will call m_del on it.
    mp_reader_t reader;
    mp_reader_new_mem(&reader, buf, (size_t)n, (size_t)size);
    return mp_lexer_new(filename, reader);
}
