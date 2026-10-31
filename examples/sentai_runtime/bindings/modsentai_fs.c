// ============== sentai.fs — Filesystem (LittleFS) ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// All public functions acquire sentai_fs_lock (2000ms timeout) to serialise
// MP filesystem access against the lfs_task (HTTP GET) and crash_log_write.
// If the lock times out, OSError("lfs busy") is raised.

extern int  sentai_fs_lock(void);
extern void sentai_fs_unlock(void);

// Whole-file reads land on the 1 MB MicroPython GC heap.  Cap them so a
// large capture/log does not raise MemoryError deep inside a mission (or
// fragment the heap for the rest of the session); bulk data goes over HTTP
// /api/raw or the USB drive (audit 2026-10-30, M10).  The read goes
// straight into the result object's storage (vstr), so the file is copied
// once, not twice as with the old m_new + mp_obj_new_bytes pair.
#define FS_READ_MAX_BYTES  (512 * 1024)

// Locks, sizes and reads `path` into `vstr` (len = bytes actually read).
// Raises OSError on lock timeout, missing file, oversize or read error.
//
// LOCK DISCIPLINE: nothing that can raise (allocation!) may run while the
// FS lock is held — a MicroPython exception unwinds straight past the
// unlock and leaves every later fs.* call failing with "lfs busy" until
// reboot (seen in EXP-s250 on build 1634, a MemoryError inside the read).
// Hence two short lock windows with the allocation in between; the size
// can change in between, so the read length is taken from the actual
// return value.
static void fs_read_whole(const char* path, vstr_t* vstr, int max_bytes) {
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int size = sentai_fs_size(path);
    sentai_fs_unlock();
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    if (size > max_bytes) {
        mp_raise_msg_varg(&mp_type_OSError,
            MP_ERROR_TEXT("file too large for fs.read (%d B > %d B): use HTTP /api/raw"),
            size, max_bytes);
    }
    if (size == 0) {
        vstr_init(vstr, 1);
        return;
    }
    vstr_init_len(vstr, (size_t)size);   // may raise MemoryError — no lock held

    if (!sentai_fs_lock()) {
        vstr_clear(vstr);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int n = sentai_fs_read(path, (uint8_t*)vstr->buf, size);
    sentai_fs_unlock();
    if (n < 0) {
        vstr_clear(vstr);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    vstr->len = (size_t)n;
}

// sentai.fs.read(path) -> bytes
static mp_obj_t mod_sentai_fs_read(mp_obj_t path_obj) {
    _fs_check_usb();
    vstr_t vstr;
    fs_read_whole(mp_obj_str_get_str(path_obj), &vstr, FS_READ_MAX_BYTES);
    return mp_obj_new_bytes_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_obj, mod_sentai_fs_read);

// sentai.fs.read_str(path) -> str
static mp_obj_t mod_sentai_fs_read_str(mp_obj_t path_obj) {
    _fs_check_usb();
    vstr_t vstr;
    fs_read_whole(mp_obj_str_get_str(path_obj), &vstr, FS_READ_MAX_BYTES);
    return mp_obj_new_str_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_str_obj, mod_sentai_fs_read_str);

// sentai.fs.read_base64(path) -> str
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static mp_obj_t mod_sentai_fs_read_base64(mp_obj_t path_obj) {
    _fs_check_usb();
    // Raw + base64 copies coexist (2.33x the file), so the cap is lower.
    vstr_t vstr;
    fs_read_whole(mp_obj_str_get_str(path_obj), &vstr, FS_READ_MAX_BYTES / 2);
    if (vstr.len == 0) {
        vstr_clear(&vstr);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    const uint8_t* buf = (const uint8_t*)vstr.buf;
    int n = (int)vstr.len;
    int b64_len = ((n + 2) / 3) * 4;
    char* b64 = m_new(char, b64_len + 1);
    int j = 0;
    for (int i = 0; i < n; i += 3) {
        uint32_t a = buf[i];
        uint32_t b = (i + 1 < n) ? buf[i + 1] : 0;
        uint32_t c = (i + 2 < n) ? buf[i + 2] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        b64[j++] = b64_table[(triple >> 18) & 0x3F];
        b64[j++] = b64_table[(triple >> 12) & 0x3F];
        b64[j++] = (i + 1 < n) ? b64_table[(triple >> 6) & 0x3F] : '=';
        b64[j++] = (i + 2 < n) ? b64_table[triple & 0x3F] : '=';
    }
    b64[j] = '\0';
    vstr_clear(&vstr);
    for (int i = 0; i < j; i += 76) {
        int chunk = (j - i > 76) ? 76 : (j - i);
        sentai_console_write(b64 + i, chunk);
        sentai_console_write("\r\n", 2);
    }
    mp_obj_t result = mp_obj_new_str(b64, j);
    m_del(char, b64, b64_len + 1);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_read_base64_obj, mod_sentai_fs_read_base64);

// sentai.fs.write(path, data) -> bool
static mp_obj_t mod_sentai_fs_write(mp_obj_t path_obj, mp_obj_t data_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int ok = sentai_fs_write(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    sentai_fs_unlock();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_fs_write_obj, mod_sentai_fs_write);

// sentai.fs.append(path, data) -> bool
// Chunked-friendly write: opens with O_APPEND|O_CREAT and appends `data`.
// Used by the REPL uploader to push files larger than what a single REPL
// line buffer (or the heap accumulator `_d = _d + ...`) can stage. Each
// call is independent: no growing _d, no GC pressure between chunks.
extern int sentai_fs_append(const char* path, const uint8_t* buf, int size);
static mp_obj_t mod_sentai_fs_append(mp_obj_t path_obj, mp_obj_t data_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int ok = sentai_fs_append(path, (const uint8_t*)bufinfo.buf, bufinfo.len);
    sentai_fs_unlock();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_fs_append_obj, mod_sentai_fs_append);

// sentai.fs.size(path) -> int (-1 if not found)
static mp_obj_t mod_sentai_fs_size(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    if (!sentai_fs_lock()) {
        return mp_obj_new_int(-1);
    }
    int result = sentai_fs_size(path);
    sentai_fs_unlock();
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_size_obj, mod_sentai_fs_size);

// sentai.fs.exists(path) -> bool
static mp_obj_t mod_sentai_fs_exists(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    if (!sentai_fs_lock()) {
        return mp_obj_new_bool(0);
    }
    int exists = sentai_fs_file_exists(path) || sentai_fs_dir_exists(path);
    sentai_fs_unlock();
    return mp_obj_new_bool(exists);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_exists_obj, mod_sentai_fs_exists);

// sentai.fs.format() -> bool
extern int sentai_fs_format(void);
static mp_obj_t mod_sentai_fs_format(void) {
    if (sentai_usb_drive_get()) sentai_usb_drive_set(0);
    if (!sentai_fs_lock()) {
        return mp_obj_new_bool(0);
    }
    int rc = sentai_fs_format();
    sentai_fs_unlock();
    return mp_obj_new_bool(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_fs_format_obj, mod_sentai_fs_format);

// sentai.fs.remove(path) -> bool
static mp_obj_t mod_sentai_fs_remove(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int rc = sentai_fs_remove(path);
    sentai_fs_unlock();
    return mp_obj_new_bool(rc == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_remove_obj, mod_sentai_fs_remove);

// sentai.fs.mkdir(path) -> bool
static mp_obj_t mod_sentai_fs_mkdir(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int rc = sentai_fs_makedirs(path);
    sentai_fs_unlock();
    return mp_obj_new_bool(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_fs_mkdir_obj, mod_sentai_fs_mkdir);

// sentai.fs.sync() -> bool
//   Force FAT-table flush to NAND.  Phase 3.2 perf-tuning dropped
//   per-write fx_media_flush; callers that need POWER-CYCLE
//   durability for their last writes MUST call this explicitly
//   before they remove USB / pull battery.  sys.reset (NVIC) is
//   safer because SDRAM cache survives across reset, but power
//   cycle clears SDRAM and unflushed FAT chains are lost.
extern int sentai_fs_sync(void);
static mp_obj_t mod_sentai_fs_sync(void) {
    _fs_check_usb();
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int rc = sentai_fs_sync();
    sentai_fs_unlock();
    return mp_obj_new_bool(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_fs_sync_obj, mod_sentai_fs_sync);

// Callback context for listdir
typedef struct {
    mp_obj_list_t* list;
} listdir_ctx_t;

static void listdir_cb(const char* name, int type, int size, void* ud) {
    listdir_ctx_t* ctx = (listdir_ctx_t*)ud;
    mp_obj_t items[3];
    items[0] = mp_obj_new_str(name, strlen(name));
    items[1] = mp_obj_new_int(type);
    items[2] = mp_obj_new_int(size);
    mp_obj_list_append(MP_OBJ_FROM_PTR(ctx->list), mp_obj_new_tuple(3, items));
}

// sentai.fs.ls(path) -> list of (name, type, size) tuples
static mp_obj_t mod_sentai_ls(mp_obj_t path_obj) {
    _fs_check_usb();
    const char* path = mp_obj_str_get_str(path_obj);
    mp_obj_list_t* result = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    listdir_ctx_t ctx = { .list = result };
    if (!sentai_fs_lock()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    int n = sentai_fs_listdir(path, listdir_cb, &ctx);
    sentai_fs_unlock();
    if (n < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("dir not found"));
    }
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_ls_obj, mod_sentai_ls);

// ---- module table ----
static const mp_rom_map_elem_t sentai_fs_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_fs) },
    { MP_ROM_QSTR(MP_QSTR_read),        MP_ROM_PTR(&mod_sentai_fs_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_str),    MP_ROM_PTR(&mod_sentai_fs_read_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_base64), MP_ROM_PTR(&mod_sentai_fs_read_base64_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),       MP_ROM_PTR(&mod_sentai_fs_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_append),      MP_ROM_PTR(&mod_sentai_fs_append_obj) },
    { MP_ROM_QSTR(MP_QSTR_size),        MP_ROM_PTR(&mod_sentai_fs_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_exists),      MP_ROM_PTR(&mod_sentai_fs_exists_obj) },
    { MP_ROM_QSTR(MP_QSTR_format),      MP_ROM_PTR(&mod_sentai_fs_format_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),      MP_ROM_PTR(&mod_sentai_fs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),       MP_ROM_PTR(&mod_sentai_fs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync),        MP_ROM_PTR(&mod_sentai_fs_sync_obj) },
    { MP_ROM_QSTR(MP_QSTR_ls),          MP_ROM_PTR(&mod_sentai_ls_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_fs_globals, sentai_fs_globals_table);
static const mp_obj_module_t sentai_fs_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_fs_globals,
};
