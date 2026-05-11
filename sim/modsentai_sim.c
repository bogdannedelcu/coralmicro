/*
 * sim/modsentai_sim.c — Phase 1.5 sentai module bindings for SIM.
 *
 * Provides hardware-independent bindings so the REPL feels like the real
 * board.  All bindings reuse QSTR table entries already present in the
 * firmware build (examples/sentai_runtime/micropython_embed/genhdr/
 * qstrdefs.generated.h).  No QSTR regen needed.
 *
 * What we expose now:
 *   sentai.version()       -> "SentAI SIM v1.0 (Phase 1.5) ..."
 *   sentai.verbose([on])   -> bool, gates [SIM] log output (set/get)
 *   sentai.io.led_on()     -> printf "[LED] ON"  (no real LED in SIM)
 *   sentai.io.led_off()    -> printf "[LED] OFF"
 *   sentai.rtos.sleep_ms(ms) -> vTaskDelay (real FreeRTOS, EINTR-safe)
 *   sentai.diag.dmesg()    -> last ~4 KB of stdout, ring-buffered
 *   sentai.sys.reset()     -> exit(0) — clean SIM exit
 *
 * Phase 2+ will add sentai.fs.* (FileX/LevelX), sentai.crazy.* (UART
 * socket → CrazySim), sentai.flow.* (camera socket → Gazebo), sentai.tpu.*
 * (libedgetpu Linux).  Same API contract as ARM firmware.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include "py/runtime.h"
#include "py/objstr.h"
#include "py/objmodule.h"
#include "py/objtuple.h"

#include "FreeRTOS.h"
#include "task.h"

#include "build_version.h"

/* ---- sentai.version() ---- */
static mp_obj_t sentai_version(void) {
    /* Mirror the firmware format: "SentAI v1.0 build NNN (timestamp)".
     * SIM has its own build counter (sim/build_version.h), separate from
     * ARM's, so the operator can tell them apart at a glance. */
    static char vers[128];
    int n = snprintf(vers, sizeof(vers),
                     "SentAI SIM v1.0 build %d (%s) - FreeRTOS POSIX + MicroPython embed",
                     BUILD_VERSION, BUILD_TIMESTAMP);
    if (n < 0) n = 0;
    return mp_obj_new_str(vers, (size_t) n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_version_obj, sentai_version);

/* ---- sentai.verbose([on]) ---- */
static int s_verbose = 1;
static mp_obj_t sentai_verbose(size_t n_args, const mp_obj_t *args) {
    int prev = s_verbose;
    if (n_args >= 1) {
        s_verbose = mp_obj_is_true(args[0]) ? 1 : 0;
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_verbose_obj, 0, 1,
                                            sentai_verbose);

/* ===== sentai.io ===== */
static mp_obj_t sentai_io_led_on(void) {
    if (s_verbose) printf("[LED] ON\n");
    fflush(stdout);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_io_led_on_obj, sentai_io_led_on);

static mp_obj_t sentai_io_led_off(void) {
    if (s_verbose) printf("[LED] OFF\n");
    fflush(stdout);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_io_led_off_obj, sentai_io_led_off);

static const mp_rom_map_elem_t sentai_io_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io) },
    { MP_ROM_QSTR(MP_QSTR_led_on),   MP_ROM_PTR(&sentai_io_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),  MP_ROM_PTR(&sentai_io_led_off_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_io_globals, sentai_io_globals_table);
static const mp_obj_module_t sentai_io_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_io_globals,
};

/* ===== sentai.rtos ===== */
static mp_obj_t sentai_rtos_sleep_ms(mp_obj_t ms_obj) {
    mp_int_t ms = mp_obj_get_int(ms_obj);
    if (ms < 0) ms = 0;
    /* vTaskDelay is the real FreeRTOS API — same as on board.  Schedules
     * other tasks for the duration.  No EINTR concern: vTaskDelay is
     * implemented inside the kernel's signal mask. */
    vTaskDelay(pdMS_TO_TICKS((TickType_t) ms));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_rtos_sleep_ms_obj,
                                  sentai_rtos_sleep_ms);

static const mp_rom_map_elem_t sentai_rtos_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rtos) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms), MP_ROM_PTR(&sentai_rtos_sleep_ms_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_rtos_globals, sentai_rtos_globals_table);
static const mp_obj_module_t sentai_rtos_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_rtos_globals,
};

/* ===== sentai.diag — minimal SDRAM-style ring log ===== */
#define SIM_DMESG_BUFSZ 4096
static char s_dmesg[SIM_DMESG_BUFSZ];
static size_t s_dmesg_len = 0;

void sim_dmesg_append(const char *s) {
    /* Best-effort append; truncate from the front when full. */
    size_t n = strlen(s);
    if (n >= SIM_DMESG_BUFSZ) {
        memcpy(s_dmesg, s + (n - (SIM_DMESG_BUFSZ - 1)),
               SIM_DMESG_BUFSZ - 1);
        s_dmesg_len = SIM_DMESG_BUFSZ - 1;
        s_dmesg[s_dmesg_len] = '\0';
        return;
    }
    if (s_dmesg_len + n >= SIM_DMESG_BUFSZ) {
        size_t drop = s_dmesg_len + n - (SIM_DMESG_BUFSZ - 1);
        memmove(s_dmesg, s_dmesg + drop, s_dmesg_len - drop);
        s_dmesg_len -= drop;
    }
    memcpy(s_dmesg + s_dmesg_len, s, n);
    s_dmesg_len += n;
    s_dmesg[s_dmesg_len] = '\0';
}

static mp_obj_t sentai_diag_dmesg(void) {
    return mp_obj_new_str(s_dmesg, s_dmesg_len);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_diag_dmesg_obj, sentai_diag_dmesg);

static const mp_rom_map_elem_t sentai_diag_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_diag) },
    { MP_ROM_QSTR(MP_QSTR_dmesg),    MP_ROM_PTR(&sentai_diag_dmesg_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_diag_globals, sentai_diag_globals_table);
static const mp_obj_module_t sentai_diag_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_diag_globals,
};

/* ===== sentai.sys ===== */
static mp_obj_t sentai_sys_reset(void) {
    printf("[sim] sentai.sys.reset() called — exiting\n");
    fflush(stdout);
    exit(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_sys_reset_obj, sentai_sys_reset);

static const mp_rom_map_elem_t sentai_sys_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sys) },
    { MP_ROM_QSTR(MP_QSTR_reset),    MP_ROM_PTR(&sentai_sys_reset_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sys_globals, sentai_sys_globals_table);
static const mp_obj_module_t sentai_sys_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_sys_globals,
};

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

static const char* sim_fs_root(void) {
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
static int sim_fs_resolve(const char *bpath, char *out, size_t outsz) {
    if (!bpath || !out) return -1;
    const char *root = sim_fs_root();
    /* Strip leading slashes from bpath so we don't end up with "//". */
    while (*bpath == '/') bpath++;
    int n = snprintf(out, outsz, "%s/%s", root, bpath);
    if (n < 0 || (size_t) n >= outsz) return -1;
    return 0;
}

static mp_obj_t sentai_fs_write(mp_obj_t path_obj, mp_obj_t buf_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf_obj, &bi, MP_BUFFER_READ);

    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_bool(0);

    /* Auto-mkdir parent (mirror FxUserWriteFile -p semantics) */
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
    if (fd < 0) return mp_obj_new_bool(0);
    ssize_t w = write(fd, bi.buf, bi.len);
    close(fd);
    return mp_obj_new_bool(w == (ssize_t) bi.len);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sentai_fs_write_obj, sentai_fs_write);

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

static mp_obj_t sentai_fs_read(mp_obj_t path_obj) {
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
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_read_obj, sentai_fs_read);

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

static mp_obj_t sentai_fs_size(mp_obj_t path_obj) {
    const char *bpath = mp_obj_str_get_str(path_obj);
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return mp_obj_new_int(-1);
    struct stat st;
    if (stat(fp, &st) != 0 || !S_ISREG(st.st_mode)) return mp_obj_new_int(-1);
    return mp_obj_new_int_from_uint((unsigned) st.st_size);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_fs_size_obj, sentai_fs_size);

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
        snprintf(child, sizeof(child), "%s/%s", fp, de->d_name);
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

/* ===== sentai.camera — virtGazebo backend (Phase 1.5 stub, Phase 4 real) =====
 *
 * On the ARM firmware sentai.camera drives the OV5640 sensors via CSI ISR
 * + PXP scaling.  In SIM there is NO OV5640 — the camera surface is fed
 * by a virtual sensor backed by a Gazebo simulation.  Phase 4 will wire
 * `sim/camera_socket.c` to a ROS 2 image subscriber.  Phase 1.5 just
 * exposes the API skeleton with sensible stub values so user scripts can
 * be developed and `sentai.camera.backend()` reports the truth.
 *
 * The MicroPython contract is identical to the firmware — same method
 * names, same return shapes — so a script that runs on ARM should run on
 * SIM after Phase 4 lands.  Differences live entirely in the C
 * implementation. */

/* Backend identifier exposed via sentai.camera.init() print line and via
 * `import sentai; sentai.camera_backend` at top-level (string constant,
 * no QSTR regen needed).  Phase 4 will switch this to a real method
 * once QSTRs are regenerated. */
static const char SIM_CAMERA_BACKEND[] = "virt_gazebo";

static mp_obj_t sentai_camera_init(size_t n_args, const mp_obj_t *args) {
    /* On firmware: init(num_frames[, w, h, fps]) -> int rc.  In Phase 1.5
     * stub: returns 0 ("ok") without actually opening a Gazebo
     * connection.  Phase 4 will connect to the bridge socket here. */
    (void) n_args; (void) args;
    if (s_verbose) printf("[camera] init() (virt_gazebo stub - Phase 4 will connect to Gazebo)\n");
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_camera_init_obj, 0, 4, sentai_camera_init);

static mp_obj_t sentai_camera_frame_count(void) {
    /* Phase 1.5 stub: no real frames yet, always 0. */
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_camera_frame_count_obj, sentai_camera_frame_count);

static mp_obj_t sentai_camera_grabbed_id(void) {
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_camera_grabbed_id_obj, sentai_camera_grabbed_id);

static mp_obj_t sentai_camera_select(mp_obj_t cam_id_obj) {
    mp_int_t cam = mp_obj_get_int(cam_id_obj);
    if (s_verbose) printf("[camera] select(%d) (virt_gazebo stub)\n", (int) cam);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_camera_select_obj, sentai_camera_select);

static const mp_rom_map_elem_t sentai_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&sentai_camera_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_frame_count), MP_ROM_PTR(&sentai_camera_frame_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_grabbed_id),  MP_ROM_PTR(&sentai_camera_grabbed_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_select),      MP_ROM_PTR(&sentai_camera_select_obj) },
    /* "backend" is a SIM-only diagnostic — no need for a stable QSTR; use
     * an inline string literal with hashing via mp_obj_new_str.  We
     * surface it as a plain attribute by aliasing the QSTR table.
     * Workaround: map under the closest existing QSTR — pick `version`
     * is wrong; we don't have a clean "backend" QSTR.  For Phase 1.5
     * we just expose it as a method anyway under MP_QSTR_init pattern.
     * Actually simpler: install under MP_QSTR_io  no — wrong name.
     * Cleanest: don't expose it under sentai.camera until QSTR regen
     * adds "backend".  Until then call sim_camera_backend() at the
     * top level: */
};
static MP_DEFINE_CONST_DICT(sentai_camera_globals, sentai_camera_globals_table);
static const mp_obj_module_t sentai_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_camera_globals,
};

/* ===== sentai.flow — Phase 4 real binding (was Phase 1.5 stub) =====
 *
 * Backed by the snapshot updated by sim/camera_bridge_recv.c on every
 * Gazebo frame.  read() returns a 5-tuple matching the firmware contract:
 *     (seq, dx_q1000, dy_q1000, conf, latency_us)
 *
 * Units identical to ARM:
 *   dx, dy   — milli-grid-pixels (1000 = 1 grid-px = 8 raw-px after PXP)
 *   conf     — 0..255 (peak/mean ratio of the phase-corr surface, scaled)
 *   latency  — recv-to-publish wall time, microseconds
 *
 * Body-frame mapping (cam0 vflip=1) lives at the consumer (`_t_flow_to_drone.py`
 * `body_xform`); see Sim.md §10b.  This binding stays raw-image-frame.
 */
typedef struct {
    volatile uint32_t seq;
    volatile int32_t  dx_q1000;
    volatile int32_t  dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t  dz_q1000;        // added 2026-05-11
    volatile uint32_t dz_conf;
} _sim_flow_snapshot_t;
extern const _sim_flow_snapshot_t* sim_camera_flow_snapshot(void);

static mp_obj_t sentai_flow_read(void) {
    const _sim_flow_snapshot_t* s = sim_camera_flow_snapshot();
    /* Snapshot stably: read seq, then payload, then re-read seq.  If the
     * second seq differs we lost the race with the writer — return the
     * later seq's data on a quick retry.  Bounded one retry. */
    uint32_t seq0 = s->seq;
    int32_t  dx   = s->dx_q1000;
    int32_t  dy   = s->dy_q1000;
    uint32_t cf   = s->conf;
    uint64_t lat  = s->latency_us;
    int32_t  dz   = s->dz_q1000;
    uint32_t dzc  = s->dz_conf;
    uint32_t seq1 = s->seq;
    if (seq1 != seq0) {
        dx  = s->dx_q1000;
        dy  = s->dy_q1000;
        cf  = s->conf;
        lat = s->latency_us;
        dz  = s->dz_q1000;
        dzc = s->dz_conf;
        seq0 = seq1;
    }
    /* Tuple: (seq, dx_q1000, dy_q1000, conf, latency_us, dz_q1000, dz_conf)
     * dz_q1000 is µ/frame (parts-per-million altitude rate); diagnostic only,
     * cf2 EKF does NOT consume it. */
    mp_obj_t items[7] = {
        mp_obj_new_int_from_uint(seq0),
        mp_obj_new_int(dx),
        mp_obj_new_int(dy),
        mp_obj_new_int_from_uint(cf),
        mp_obj_new_int_from_ull(lat),
        mp_obj_new_int(dz),
        mp_obj_new_int_from_uint(dzc),
    };
    return mp_obj_new_tuple(7, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_flow_read_obj, sentai_flow_read);

static const mp_rom_map_elem_t sentai_flow_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_flow) },
    { MP_ROM_QSTR(MP_QSTR_read),     MP_ROM_PTR(&sentai_flow_read_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_flow_globals, sentai_flow_globals_table);
static const mp_obj_module_t sentai_flow_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_flow_globals,
};

/* ===== sentai.tpu — Phase 5 SIM via pycoral helper ============================
 * Mirrors examples/sentai_runtime/modsentai_tpu.c MP_QSTR table. The C
 * entry points sentai_tpu_*() are implemented in sim/sim_tpu_shim.c and
 * forward to a pycoral daemon over /tmp/sentai_tpu.sock.
 */
#include "examples/sentai_runtime/sentai_tpu_shim.h"

static mp_obj_t sentai_tpu_load(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_tpu_load_model(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_load_obj, sentai_tpu_load);

static mp_obj_t sentai_tpu_invoke_mp(void) {
    return mp_obj_new_int(sentai_tpu_invoke());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_invoke_obj, sentai_tpu_invoke_mp);

static mp_obj_t sentai_tpu_ready_mp(void) {
    return mp_obj_new_bool(sentai_tpu_is_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_ready_obj, sentai_tpu_ready_mp);

static mp_obj_t sentai_tpu_num_outputs_mp(void) {
    return mp_obj_new_int(sentai_tpu_num_outputs());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_num_outputs_obj, sentai_tpu_num_outputs_mp);

static mp_obj_t sentai_tpu_output_size_mp(mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_size(mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_size_obj, sentai_tpu_output_size_mp);

static mp_obj_t sentai_tpu_output_mp(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int sz  = sentai_tpu_get_output_size(idx);
    const void* d = sentai_tpu_get_output_data(idx);
    if (!d || sz <= 0) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("no output"));
    return mp_obj_new_bytes((const byte*)d, sz);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_obj, sentai_tpu_output_mp);

static mp_obj_t sentai_tpu_output_dims_mp(mp_obj_t idx_obj) {
    int idx = mp_obj_get_int(idx_obj);
    int n   = sentai_tpu_get_output_num_dims(idx);
    if (n > 8) n = 8;
    mp_obj_t items[8];
    for (int i = 0; i < n; ++i)
        items[i] = mp_obj_new_int(sentai_tpu_get_output_dim(idx, i));
    return mp_obj_new_tuple(n, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_dims_obj, sentai_tpu_output_dims_mp);

static mp_obj_t sentai_tpu_output_type_mp(mp_obj_t idx_obj) {
    return mp_obj_new_int(sentai_tpu_get_output_type(mp_obj_get_int(idx_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_type_obj, sentai_tpu_output_type_mp);

static mp_obj_t sentai_tpu_input_quant_mp(void) {
    float scale = 0.0f; int32_t zp = 0;
    sentai_tpu_input_quant(&scale, &zp);
    mp_obj_t items[2] = { mp_obj_new_float(scale), mp_obj_new_int(zp) };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_input_quant_obj, sentai_tpu_input_quant_mp);

static mp_obj_t sentai_tpu_output_quant_mp(mp_obj_t idx_obj) {
    float scale = 0.0f; int32_t zp = 0;
    sentai_tpu_output_quant(mp_obj_get_int(idx_obj), &scale, &zp);
    mp_obj_t items[2] = { mp_obj_new_float(scale), mp_obj_new_int(zp) };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_output_quant_obj, sentai_tpu_output_quant_mp);

static mp_obj_t sentai_tpu_input_type_mp(void) {
    return mp_obj_new_int(sentai_tpu_input_type());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_tpu_input_type_obj, sentai_tpu_input_type_mp);

/* sentai.tpu.set_input(bytes) — pushes a flat byte buffer into the input tensor.
 * Useful for loading a pre-resized image without going through the camera. */
static mp_obj_t sentai_tpu_set_input_mp(mp_obj_t buf_obj) {
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf_obj, &bi, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_tpu_set_input_slot(0, (const uint8_t*)bi.buf, bi.len));
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_tpu_set_input_obj, sentai_tpu_set_input_mp);

static const mp_rom_map_elem_t sentai_tpu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_tpu) },
    { MP_ROM_QSTR(MP_QSTR_load),         MP_ROM_PTR(&sentai_tpu_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_invoke),       MP_ROM_PTR(&sentai_tpu_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_ready),        MP_ROM_PTR(&sentai_tpu_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_num_outputs),  MP_ROM_PTR(&sentai_tpu_num_outputs_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_size),  MP_ROM_PTR(&sentai_tpu_output_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_output),       MP_ROM_PTR(&sentai_tpu_output_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_dims),  MP_ROM_PTR(&sentai_tpu_output_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_type),  MP_ROM_PTR(&sentai_tpu_output_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_quant),  MP_ROM_PTR(&sentai_tpu_input_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_output_quant), MP_ROM_PTR(&sentai_tpu_output_quant_obj) },
    { MP_ROM_QSTR(MP_QSTR_input_type),   MP_ROM_PTR(&sentai_tpu_input_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_input),    MP_ROM_PTR(&sentai_tpu_set_input_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_tpu_globals, sentai_tpu_globals_table);
static const mp_obj_module_t sentai_tpu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_tpu_globals,
};

/* ===== top-level sentai module ===== */
static const mp_rom_map_elem_t sentai_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai) },
    { MP_ROM_QSTR(MP_QSTR_version),  MP_ROM_PTR(&sentai_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_verbose),  MP_ROM_PTR(&sentai_verbose_obj) },
    { MP_ROM_QSTR(MP_QSTR_io),       MP_ROM_PTR(&sentai_io_module) },
    { MP_ROM_QSTR(MP_QSTR_rtos),     MP_ROM_PTR(&sentai_rtos_module) },
    { MP_ROM_QSTR(MP_QSTR_diag),     MP_ROM_PTR(&sentai_diag_module) },
    { MP_ROM_QSTR(MP_QSTR_sys),      MP_ROM_PTR(&sentai_sys_module) },
    { MP_ROM_QSTR(MP_QSTR_fs),       MP_ROM_PTR(&sentai_fs_module) },
    { MP_ROM_QSTR(MP_QSTR_camera),   MP_ROM_PTR(&sentai_camera_module) },
    { MP_ROM_QSTR(MP_QSTR_flow),     MP_ROM_PTR(&sentai_flow_module) },
    { MP_ROM_QSTR(MP_QSTR_tpu),      MP_ROM_PTR(&sentai_tpu_module) },
};
static MP_DEFINE_CONST_DICT(sentai_globals, sentai_globals_table);

/* This symbol is referenced from moduledefs.h (generated for the firmware
 * build).  Replaces the empty stub previously in main_sim.c. */
const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_globals,
};
