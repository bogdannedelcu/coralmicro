// B8.4 in-firmware virtual filesystem fixture for the ARM emulator.
//
// `mission.py` is baked into the binary so the REPL can run
// `import mission; mission.run()` without any real storage, FileX/LevelX,
// USB MSC, or host bridge.  Later milestones can swap this for an emulator
// flash/SD image without changing the import machinery contract — that is the
// point of the table-driven lookup below.

#include <stdint.h>
#include <string.h>

#include "py/builtin.h"
#include "py/lexer.h"
#include "py/obj.h"
#include "py/reader.h"
#include "py/runtime.h"

static const char kMissionPy[] =
    "def run():\n"
    "    print('MISSION OK from B8.4', 2 + 3)\n";

typedef struct {
    const char *name;
    const char *data;
    size_t size;
} sentai_emu_file_t;

static const sentai_emu_file_t kFiles[] = {
    {"mission.py", kMissionPy, sizeof(kMissionPy) - 1},
};
static const size_t kNumFiles = sizeof(kFiles) / sizeof(kFiles[0]);

static const sentai_emu_file_t *sentai_emu_find_file(const char *path) {
    if (path == NULL) {
        return NULL;
    }
    while (*path == '/' || *path == '.') {
        ++path;
    }
    for (size_t i = 0; i < kNumFiles; ++i) {
        if (strcmp(path, kFiles[i].name) == 0) {
            return &kFiles[i];
        }
    }
    return NULL;
}

mp_import_stat_t mp_import_stat(const char *path) {
    return sentai_emu_find_file(path) ? MP_IMPORT_STAT_FILE
                                      : MP_IMPORT_STAT_NO_EXIST;
}

mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    const char *path = qstr_str(filename);
    const sentai_emu_file_t *f = sentai_emu_find_file(path);
    if (f == NULL) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("emu: file not found"));
    }
    mp_reader_t reader;
    // free_len = 0 so the lexer does not try to m_del the .rodata buffer when
    // it finishes.
    mp_reader_new_mem(&reader, (const byte *)f->data, f->size, 0);
    return mp_lexer_new(filename, reader);
}
