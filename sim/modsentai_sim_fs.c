/* modsentai_sim_fs.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */

/* ===== sentai.fs — Phase 2 LIGHT: file backing on Linux FS =====
 *
 * Maps each board path "/foo/bar" to "<sim_root>/foo/bar" on Linux.
 * The default sim_root is "./sentai_sim_root" relative to the cwd; can
 * be overridden by setting the SENTAI_SIM_ROOT env var before launch.
 *
 * Behavior matches the firmware sentai.fs.* contract:
 *   write(path, bytes) -> truncating write, returns True/False
 *   append(path, bytes) -> append (creates file if absent), True/False
 *   read(path) -> bytes (full file)
 *   read_str(path) -> str (full file decoded UTF-8)
 *   exists(path) -> bool
 *   size(path) -> int (-1 if missing)
 *   ls(dir) -> list of (name, type, size) where type 1=file 2=dir
 *   mkdir(path) -> mkdir -p, returns True/False
 *   remove(path) -> 0 on success, negative errno on fail
 *   sync() -> True (no-op safe; syncs all writes via fsync below)
 *
 * NOT yet implemented: SAFE MODE FSM, FxUserInit/Sync/etc.  Those need
 * the real FileX/LevelX stack from libs/filex+libs/levelx — Phase 2 FULL.
 * Phase 2 LIGHT here is enough to test scripts that only use the
 * sentai.fs.* API surface. */

/* Default set at compile time by sim/CMakeLists.txt to
 * `${CMAKE_BINARY_DIR}/sentai_fs_root`.  Override with SENTAI_SIM_ROOT
 * env var at runtime. */
#ifndef SENTAI_SIM_FS_ROOT_DEFAULT
#define SENTAI_SIM_FS_ROOT_DEFAULT "./sentai_sim_root"
#endif

#define SIM_FS_MAXPATH 512

const char* sim_fs_root(void) {
    static const char *cached = NULL;
    if (cached) return cached;
    const char *env = getenv("SENTAI_SIM_ROOT");
    cached = (env && env[0]) ? env : SENTAI_SIM_FS_ROOT_DEFAULT;
    /* Ensure root exists (mkdir -p; ignore EEXIST).  Walk parents in
     * case the build dir wasn't created yet. */
    char tmp[SIM_FS_MAXPATH + 1];
    strncpy(tmp, cached, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
    if (s_verbose) printf("[sim] FS root: %s\n", cached);
    return cached;
}

/* Resolve board path "/a/b" to full Linux path "<root>/a/b".
 * Returns 0 on success.  buf must be SIM_FS_MAXPATH+1 bytes. */
int sim_fs_resolve(const char *bpath, char *out, size_t outsz) {
    if (!bpath || !out) return -1;
    const char *root = sim_fs_root();
    /* Strip leading slashes from bpath so we don't end up with "//". */
    while (*bpath == '/') bpath++;
    int n = snprintf(out, outsz, "%s/%s", root, bpath);
    if (n < 0 || (size_t) n >= outsz) return -1;
    return 0;
}

int sentai_fs_write(const char* bpath, const uint8_t* data, int size) {
    if (!bpath || !data || size < 0) return -1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;

    char *slash = strrchr(fp, '/');
    if (slash && slash != fp) {
        *slash = '\0';
        char *p = fp;
        while ((p = strchr(p + 1, '/')) != NULL) {
            *p = '\0'; mkdir(fp, 0755); *p = '/';
        }
        mkdir(fp, 0755);
        *slash = '/';
    }

    int fd = open(fp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return -1;
    ssize_t w = write(fd, data, (size_t)size);
    close(fd);
    return w == (ssize_t)size ? size : -1;
}

int sentai_fs_size(const char* bpath) {
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;
    struct stat st;
    if (stat(fp, &st) != 0 || !S_ISREG(st.st_mode)) return -1;
    return (int)st.st_size;
}

int sentai_fs_read(const char* bpath, uint8_t* buf, int max_size) {
    if (!bpath || !buf || max_size < 0) return -1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;
    int fd = open(fp, O_RDONLY);
    if (fd < 0) return -1;
    ssize_t r = read(fd, buf, (size_t)max_size);
    close(fd);
    return r < 0 ? -1 : (int)r;
}

static mp_obj_t sim_mp_fs_write(mp_obj_t path_obj, mp_obj_t buf_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf_obj, &bi, MP_BUFFER_READ);
    int w = sentai_fs_write(bpath, (const uint8_t*)bi.buf, (int)bi.len);
    return mp_obj_new_bool(w == (int)bi.len);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sentai_fs_write_obj, sim_mp_fs_write);

static mp_obj_t sentai_fs_append(mp_obj_t path_obj, mp_obj_t buf_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf_obj, &bi, MP_BUFFER_READ);

    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_bool(0);
    int fd = open(fp, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd < 0) return mp_obj_new_bool(0);
    ssize_t w = write(fd, bi.buf, bi.len);
    close(fd);
    return mp_obj_new_bool(w == (ssize_t) bi.len);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sentai_fs_append_obj, sentai_fs_append);

static mp_obj_t sim_mp_fs_read(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) {
        mp_raise_OSError(ENOENT);
    }
    struct stat st;
    if (stat(fp, &st) != 0 || !S_ISREG(st.st_mode)) {
        mp_raise_OSError(ENOENT);
    }
    int fd = open(fp, O_RDONLY);
    if (fd < 0) mp_raise_OSError(errno);
    vstr_t vstr;
    vstr_init_len(&vstr, st.st_size);
    ssize_t r = read(fd, vstr.buf, st.st_size);
    close(fd);
    if (r != st.st_size) {
        vstr_clear(&vstr);
        mp_raise_OSError(EIO);
    }
    return mp_obj_new_bytes_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_read_obj, sim_mp_fs_read);

static mp_obj_t sentai_fs_read_str(mp_obj_t path_obj) {
    /* Same as read but return str (decoded UTF-8). */
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) mp_raise_OSError(ENOENT);
    struct stat st;
    if (stat(fp, &st) != 0 || !S_ISREG(st.st_mode)) mp_raise_OSError(ENOENT);
    int fd = open(fp, O_RDONLY);
    if (fd < 0) mp_raise_OSError(errno);
    vstr_t vstr;
    vstr_init_len(&vstr, st.st_size);
    ssize_t r = read(fd, vstr.buf, st.st_size);
    close(fd);
    if (r != st.st_size) { vstr_clear(&vstr); mp_raise_OSError(EIO); }
    return mp_obj_new_str_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_read_str_obj, sentai_fs_read_str);

static mp_obj_t sentai_fs_exists(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_bool(0);
    struct stat st;
    return mp_obj_new_bool(stat(fp, &st) == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_exists_obj, sentai_fs_exists);

static mp_obj_t sim_mp_fs_size(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_int(-1);
    struct stat st;
    if (stat(fp, &st) != 0 || !S_ISREG(st.st_mode)) return mp_obj_new_int(-1);
    return mp_obj_new_int_from_uint((unsigned) st.st_size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_size_obj, sim_mp_fs_size);

static mp_obj_t sentai_fs_ls(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) {
        return mp_obj_new_list(0, NULL);
    }
    DIR *d = opendir(fp);
    if (!d) return mp_obj_new_list(0, NULL);
    mp_obj_t lst = mp_obj_new_list(0, NULL);
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (strcmp(de->d_name, ".") == 0) continue;
        if (strcmp(de->d_name, "..") == 0) continue;
        char child[SIM_FS_MAXPATH + 1];
        size_t fp_len = strlen(fp);
        size_t name_len = strlen(de->d_name);
        if (fp_len + 1 + name_len >= sizeof(child)) continue;
        memcpy(child, fp, fp_len);
        child[fp_len] = '/';
        memcpy(child + fp_len + 1, de->d_name, name_len + 1);
        struct stat st;
        if (stat(child, &st) != 0) continue;
        int type = S_ISDIR(st.st_mode) ? 2 : (S_ISREG(st.st_mode) ? 1 : 0);
        size_t fsize = S_ISREG(st.st_mode) ? (size_t) st.st_size : 0;
        mp_obj_t tup[3] = {
            mp_obj_new_str(de->d_name, strlen(de->d_name)),
            mp_obj_new_int(type),
            mp_obj_new_int_from_uint((unsigned) fsize),
        };
        mp_obj_list_append(lst, mp_obj_new_tuple(3, tup));
    }
    closedir(d);
    return lst;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_ls_obj, sentai_fs_ls);

static mp_obj_t sentai_fs_mkdir(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_bool(0);
    /* mkdir -p: walk components */
    for (char *p = fp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(fp, 0755) != 0 && errno != EEXIST) {
                *p = '/';
                return mp_obj_new_bool(0);
            }
            *p = '/';
        }
    }
    int rc = mkdir(fp, 0755);
    return mp_obj_new_bool(rc == 0 || errno == EEXIST);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_mkdir_obj, sentai_fs_mkdir);

static mp_obj_t sentai_fs_remove(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_int(-2);
    if (unlink(fp) == 0) return mp_obj_new_int(0);
    if (errno == EISDIR && rmdir(fp) == 0) return mp_obj_new_int(0);
    return mp_obj_new_int(errno == ENOENT ? -2 : -5);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_remove_obj, sentai_fs_remove);

static mp_obj_t sentai_fs_sync(void) {
    sync();   /* whole-system fsync; cheap and safe in SIM */
    return mp_obj_new_bool(1);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_fs_sync_obj, sentai_fs_sync);

static const mp_rom_map_elem_t sentai_fs_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_fs) },
    { MP_ROM_QSTR(MP_QSTR_write),    MP_ROM_PTR(&sentai_fs_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_append),   MP_ROM_PTR(&sentai_fs_append_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),     MP_ROM_PTR(&sentai_fs_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_str), MP_ROM_PTR(&sentai_fs_read_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_exists),   MP_ROM_PTR(&sentai_fs_exists_obj) },
    { MP_ROM_QSTR(MP_QSTR_size),     MP_ROM_PTR(&sentai_fs_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_ls),       MP_ROM_PTR(&sentai_fs_ls_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),    MP_ROM_PTR(&sentai_fs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),   MP_ROM_PTR(&sentai_fs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync),     MP_ROM_PTR(&sentai_fs_sync_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_fs_globals, sentai_fs_globals_table);
static const mp_obj_module_t sentai_fs_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_fs_globals,
};
