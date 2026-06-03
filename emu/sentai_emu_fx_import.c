// MicroPython import hook backed by FxUser/FileX.

#include <stdint.h>
#include <stddef.h>

#include "libs/base/fx_user_fs.h"
#include "py/builtin.h"
#include "py/lexer.h"
#include "py/reader.h"
#include "py/runtime.h"

mp_import_stat_t mp_import_stat(const char* path) {
    if (path == NULL) {
        return MP_IMPORT_STAT_NO_EXIST;
    }
    if (FxUserDirExists(path)) {
        return MP_IMPORT_STAT_DIR;
    }
    if (FxUserFileExists(path)) {
        return MP_IMPORT_STAT_FILE;
    }
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_lexer_t* mp_lexer_new_from_file(qstr filename) {
    const char* path = qstr_str(filename);
    ssize_t size = FxUserSize(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    byte* buf = m_new(byte, (size_t)size);
    size_t n = 0;
    if (size > 0) {
        n = FxUserReadFile(path, buf, (size_t)size);
        if (n != (size_t)size) {
            m_del(byte, buf, (size_t)size);
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("short read"));
        }
    }
    mp_reader_t reader;
    mp_reader_new_mem(&reader, buf, n, (size_t)size);
    return mp_lexer_new(filename, reader);
}
