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

/* ===== sentai.pipeline — Phase 5.6 minimal SIM ============================
 *
 * ARM has 35+ pipeline functions covering tracking, events, camera config,
 * health, etc.  For SIM we ship the essentials needed to demonstrate the
 * camera→TPU loop:
 *
 *   pipeline.tick()             — sync: latest cam frame → CPU resize →
 *                                  TPU set_input → invoke.  Returns
 *                                  invoke time in ms (-1 on failure).
 *   pipeline.start()            — spawn a FreeRTOS task running tick()
 *                                  in a loop at target_fps_hz.
 *   pipeline.stop()             — clear the task's run flag.
 *   pipeline.running()          — bool.
 *   pipeline.stats()            — dict {frames, last_invoke_ms,
 *                                  total_invoke_ms, last_err}.
 *
 * Camera frame must come from the Gazebo bridge (camera_bridge_recv has
 * a copy in s_rgb_full_pub).  TPU helper must be running.
 */
#include "FreeRTOS.h"
#include "task.h"

extern size_t sim_camera_latest_rgb(uint8_t* dst, size_t max_bytes,
                                     int* out_w, int* out_h, uint32_t* out_seq);

#define PIPE_CAM_W   640
#define PIPE_CAM_H   480
#define PIPE_CAM_SZ  (PIPE_CAM_W * PIPE_CAM_H * 3)

static uint8_t  s_pipe_cam_buf[PIPE_CAM_SZ];
static uint8_t  s_pipe_resized[1024 * 1024];   // up to ~1 MB resized tensor
static volatile int     s_pipe_running = 0;
static TaskHandle_t     s_pipe_task = NULL;
static volatile uint32_t s_pipe_frames = 0;
static volatile uint32_t s_pipe_last_ms = 0;
static volatile uint32_t s_pipe_total_ms = 0;
static volatile int     s_pipe_last_err = 0;
static volatile uint32_t s_pipe_target_fps = 10;  // safe default; helper ~16ms invoke

/* Bilinear-ish CPU resize (nearest-neighbour for speed; works for the
 * "let me see something running" smoke level.  Production should use
 * area-resampling for accuracy — same pattern as sentai_pxp_shim_sim.c. */
static int sim_resize_rgb888_nearest(const uint8_t* src, int sw, int sh,
                                      uint8_t* dst, int dw, int dh) {
    if (!src || !dst) return -1;
    for (int y = 0; y < dh; ++y) {
        int sy = (y * sh) / dh;
        if (sy >= sh) sy = sh - 1;
        const uint8_t* srow = src + sy * sw * 3;
        uint8_t* drow = dst + y * dw * 3;
        for (int x = 0; x < dw; ++x) {
            int sx = (x * sw) / dw;
            if (sx >= sw) sx = sw - 1;
            drow[x*3+0] = srow[sx*3+0];
            drow[x*3+1] = srow[sx*3+1];
            drow[x*3+2] = srow[sx*3+2];
        }
    }
    return 0;
}

/* Core single-tick: cam → resize → set_input → invoke.  Returns inference
 * latency in ms, or negative on error. */
static int pipeline_tick_once(void) {
    if (!sentai_tpu_is_ready()) return -10;

    /* Get latest 640x480 RGB frame. */
    int cw, ch; uint32_t seq;
    size_t got = sim_camera_latest_rgb(s_pipe_cam_buf, sizeof(s_pipe_cam_buf),
                                        &cw, &ch, &seq);
    if (got == 0) return -11;   /* no frame yet (Gazebo bridge not running?) */

    /* Query TPU input dimensions. */
    int iw=0, ih=0, ic=0, itype=0, izp=0;
    uint8_t* dummy = NULL;
    if (sentai_get_tensor_info(&iw, &ih, &ic, &dummy, &itype, &izp) != 0) {
        return -12;
    }
    if (ic != 3) return -13;   /* SIM resize path is RGB888 only */
    int resized_bytes = iw * ih * 3;
    if ((size_t)resized_bytes > sizeof(s_pipe_resized)) return -14;

    if (sim_resize_rgb888_nearest(s_pipe_cam_buf, cw, ch,
                                   s_pipe_resized, iw, ih) != 0) return -15;

    /* Push to TPU + invoke. */
    if (sentai_tpu_set_input_slot(0, s_pipe_resized, resized_bytes) != 0) return -16;

    TickType_t t0 = xTaskGetTickCount();
    int rc = sentai_tpu_invoke();
    if (rc != 0) return -17 - rc;
    uint32_t dt = (uint32_t)(xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;

    s_pipe_frames    += 1;
    s_pipe_last_ms    = dt;
    s_pipe_total_ms  += dt;
    s_pipe_last_err   = 0;

    /* Update tracker history (declared below; forward-decl). */
    extern void track_record(void);
    track_record();

    /* If the loaded model is SSD-style (4 outputs), auto-feed SentAI-SORT
     * with decoded detections.  Defined below — forward-decl + later
     * code calls it.  See ssd_decode_into() further down. */
    extern int  pipeline_autoupdate_tracker_if_ssd(void);
    pipeline_autoupdate_tracker_if_ssd();
    return (int)dt;
}

static mp_obj_t sentai_pipeline_tick_mp(void) {
    int ms = pipeline_tick_once();
    if (ms < 0) s_pipe_last_err = ms;
    return mp_obj_new_int(ms);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tick_obj, sentai_pipeline_tick_mp);

static void pipeline_task(void* arg) {
    (void)arg;
    s_pipe_running = 1;
    while (s_pipe_running) {
        uint32_t period_ms = 1000 / (s_pipe_target_fps ? s_pipe_target_fps : 1);
        TickType_t t0 = xTaskGetTickCount();
        pipeline_tick_once();
        TickType_t spent = xTaskGetTickCount() - t0;
        TickType_t period = pdMS_TO_TICKS(period_ms);
        if (spent < period) vTaskDelay(period - spent);
        else                vTaskDelay(1);
    }
    s_pipe_task = NULL;
    vTaskDelete(NULL);
}

static mp_obj_t sentai_pipeline_start_mp(size_t n_args, const mp_obj_t* args) {
    if (s_pipe_running) return mp_obj_new_int(-1);
    if (n_args >= 1) s_pipe_target_fps = mp_obj_get_int(args[0]);
    BaseType_t ok = xTaskCreate(pipeline_task, "pipeline",
                                 configMINIMAL_STACK_SIZE * 16,
                                 NULL, tskIDLE_PRIORITY + 2, &s_pipe_task);
    if (ok != pdPASS) return mp_obj_new_int(-2);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_start_obj, 0, 1, sentai_pipeline_start_mp);

static mp_obj_t sentai_pipeline_stop_mp(void) {
    s_pipe_running = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_stop_obj, sentai_pipeline_stop_mp);

static mp_obj_t sentai_pipeline_running_mp(void) {
    return mp_obj_new_bool(s_pipe_running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_running_obj, sentai_pipeline_running_mp);

static mp_obj_t sentai_pipeline_stats_mp(void) {
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),
                      mp_obj_new_int_from_uint(s_pipe_frames));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_invoke_ms_max),
                      mp_obj_new_int_from_uint(s_pipe_last_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_sum),
                      mp_obj_new_int_from_uint(s_pipe_total_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_running),
                      mp_obj_new_bool(s_pipe_running));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_stats_obj, sentai_pipeline_stats_mp);

static mp_obj_t sentai_pipeline_target_fps_mp(size_t n_args, const mp_obj_t* args) {
    if (n_args >= 1) s_pipe_target_fps = mp_obj_get_int(args[0]);
    return mp_obj_new_int(s_pipe_target_fps);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_target_fps_obj, 0, 1, sentai_pipeline_target_fps_mp);

/* sentai.pipeline.detect(top_n=5) — returns top-N (idx, score_byte) tuples
 * from the FIRST output tensor of the loaded model.  Works for any
 * classification head (mobilenet 1001 classes, etc.).  For YOLO/heatmap
 * outputs the caller has to interpret bytes themselves.
 *
 * Returns: tuple of N tuples, each (class_idx_int, score_byte_int).
 * If no model loaded or no output: empty tuple. */
static mp_obj_t sentai_pipeline_detect_mp(size_t n_args, const mp_obj_t* args) {
    int top_n = (n_args >= 1) ? mp_obj_get_int(args[0]) : 5;
    if (top_n < 1) top_n = 1;
    if (top_n > 16) top_n = 16;
    if (!sentai_tpu_is_ready() || sentai_tpu_num_outputs() < 1) {
        return mp_obj_new_tuple(0, NULL);
    }
    int sz = sentai_tpu_get_output_size(0);
    const uint8_t* d = (const uint8_t*)sentai_tpu_get_output_data(0);
    if (!d || sz <= 0) return mp_obj_new_tuple(0, NULL);

    /* Bounded heap-free top-N by repeated linear scan; sz<=1001 typical. */
    int best_idx[16];
    uint8_t best_val[16];
    int found = 0;
    for (int rank = 0; rank < top_n; ++rank) {
        int   bi = -1;
        int   bv = -1;
        for (int i = 0; i < sz; ++i) {
            int v = d[i];
            /* Skip indices already chosen at higher rank. */
            int already = 0;
            for (int k = 0; k < rank; ++k) if (best_idx[k] == i) { already = 1; break; }
            if (already) continue;
            if (v > bv) { bv = v; bi = i; }
        }
        if (bi < 0) break;
        best_idx[rank] = bi;
        best_val[rank] = (uint8_t)bv;
        ++found;
    }
    mp_obj_t out[16];
    for (int i = 0; i < found; ++i) {
        mp_obj_t pair[2] = {
            mp_obj_new_int(best_idx[i]),
            mp_obj_new_int(best_val[i]),
        };
        out[i] = mp_obj_new_tuple(2, pair);
    }
    return mp_obj_new_tuple(found, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_detect_obj, 0, 1, sentai_pipeline_detect_mp);

/* sentai.pipeline.tracks() — returns list of "stable" classifications
 * over a sliding window of recent step()s.  A class counts as "tracked"
 * if it appeared in top-3 of >= 3 of the last 5 frames.  Useful for
 * filtering noise from per-frame detect().
 *
 * Returns: tuple of (class_idx, hit_count_in_window) tuples.  Empty
 * if no model, no recent frames, or no class qualified.
 */
#define TRACK_WINDOW 5
#define TRACK_TOPN   3
#define TRACK_MIN_HITS 3
static int s_track_hist[TRACK_WINDOW][TRACK_TOPN];
static int s_track_idx = 0;
static int s_track_count = 0;

/* Internal: called from pipeline_tick_once after a successful invoke.
 * Records the top-3 class indices into the rolling window. */
void track_record(void) {
    if (!sentai_tpu_is_ready() || sentai_tpu_num_outputs() < 1) return;
    int sz = sentai_tpu_get_output_size(0);
    const uint8_t* d = (const uint8_t*)sentai_tpu_get_output_data(0);
    if (!d || sz <= 0) return;

    int slot = s_track_idx % TRACK_WINDOW;
    int chosen[TRACK_TOPN];
    for (int rank = 0; rank < TRACK_TOPN; ++rank) {
        int bi = -1, bv = -1;
        for (int i = 0; i < sz; ++i) {
            int already = 0;
            for (int k = 0; k < rank; ++k) if (chosen[k] == i) { already = 1; break; }
            if (already) continue;
            int v = d[i];
            if (v > bv) { bv = v; bi = i; }
        }
        chosen[rank] = bi;
        s_track_hist[slot][rank] = bi;
    }
    s_track_idx++;
    if (s_track_count < TRACK_WINDOW) s_track_count++;
}

static mp_obj_t sentai_pipeline_tracks_mp(void) {
    /* Count how often each class appears in the window. */
    int n_eff = s_track_count < TRACK_WINDOW ? s_track_count : TRACK_WINDOW;
    int classes[TRACK_WINDOW * TRACK_TOPN];
    int counts [TRACK_WINDOW * TRACK_TOPN];
    int n_unique = 0;
    for (int slot = 0; slot < n_eff; ++slot) {
        for (int r = 0; r < TRACK_TOPN; ++r) {
            int c = s_track_hist[slot][r];
            if (c < 0) continue;
            int found = -1;
            for (int u = 0; u < n_unique; ++u) if (classes[u] == c) { found = u; break; }
            if (found >= 0) counts[found]++;
            else { classes[n_unique] = c; counts[n_unique] = 1; n_unique++; }
        }
    }
    /* Emit qualifying entries unsorted; caller can sort by count. */
    mp_obj_t out[16];
    int n_out = 0;
    for (int i = 0; i < n_unique && n_out < 16; ++i) {
        if (counts[i] >= TRACK_MIN_HITS) {
            mp_obj_t pair[2] = {
                mp_obj_new_int(classes[i]),
                mp_obj_new_int(counts[i]),
            };
            out[n_out++] = mp_obj_new_tuple(2, pair);
        }
    }
    return mp_obj_new_tuple(n_out, out);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracks_obj, sentai_pipeline_tracks_mp);

/* Reset track history (e.g., after model swap). */
static mp_obj_t sentai_pipeline_track_reset_mp(void) {
    s_track_idx = 0;
    s_track_count = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_track_reset_obj, sentai_pipeline_track_reset_mp);

/* ============= SSD MobileNet V2 detections + SentAI-SORT integration =====
 *
 * SSD MobileNet V2 (tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite) has
 * postprocess built-in.  4 outputs:
 *   out[0]: scores       float32 [20]
 *   out[1]: boxes        float32 [20,4] (ymin,xmin,ymax,xmax normalized)
 *   out[2]: num_detect   float32 [1]
 *   out[3]: classes      float32 [20]
 *
 * We pack these into the shared sentai_runtime/detection_task.h `Detection`
 * struct, then feed them to the BoT-SORT-adapted SentAI-SORT tracker
 * (sentai_tracker_*) — same code as ARM.  REPL surface mirrors the ARM
 * sentai.pipeline.* names so user scripts port unchanged.
 */
#include "examples/sentai_runtime/sentai_tracker.h"
#include "examples/sentai_runtime/detection_task.h"

#define SIM_SSD_INPUT_W 300
#define SIM_SSD_INPUT_H 300
#define SIM_SSD_INPUT_C 3
#define SIM_MAX_DETS    32

static Detection s_dets[SIM_MAX_DETS];
static int       s_n_dets = 0;
static uint32_t  s_det_frame_seq = 0;
/* Last input tensor we sent to TPU — needed by tracker's histogram path. */
extern uint8_t  s_pipe_resized[];

static int ssd_decode_into(int conf_thresh_permil) {
    s_n_dets = 0;
    if (!sentai_tpu_is_ready()) return -1;
    if (sentai_tpu_num_outputs() < 4) return -2;

    int sz_scores = sentai_tpu_get_output_size(0);
    int sz_boxes  = sentai_tpu_get_output_size(1);
    int sz_num    = sentai_tpu_get_output_size(2);
    int sz_class  = sentai_tpu_get_output_size(3);
    const float* sc  = (const float*)sentai_tpu_get_output_data(0);
    const float* bx  = (const float*)sentai_tpu_get_output_data(1);
    const float* nm  = (const float*)sentai_tpu_get_output_data(2);
    const float* cls = (const float*)sentai_tpu_get_output_data(3);
    if (!sc || !bx || !nm || !cls) return -3;
    (void)sz_scores; (void)sz_boxes; (void)sz_num; (void)sz_class;

    int n_avail = (int)nm[0];
    if (n_avail > 20) n_avail = 20;
    int kept = 0;
    for (int i = 0; i < n_avail && kept < SIM_MAX_DETS; ++i) {
        int permil = (int)(sc[i] * 1000.0f + 0.5f);
        if (permil < conf_thresh_permil) continue;
        float ymin = bx[i*4+0], xmin = bx[i*4+1];
        float ymax = bx[i*4+2], xmax = bx[i*4+3];
        Detection* d = &s_dets[kept++];
        d->x1 = (int16_t)(xmin * SIM_SSD_INPUT_W);
        d->y1 = (int16_t)(ymin * SIM_SSD_INPUT_H);
        d->x2 = (int16_t)(xmax * SIM_SSD_INPUT_W);
        d->y2 = (int16_t)(ymax * SIM_SSD_INPUT_H);
        d->conf_permil = (int16_t)permil;
        d->class_id    = (int16_t)cls[i];
    }
    s_n_dets = kept;
    s_det_frame_seq++;
    return kept;
}

/* Called from pipeline_tick_once() in the start() task — auto-feeds
 * SentAI-SORT with the latest detections when an SSD-style model is
 * loaded (4 outputs).  No-op for classification models. */
int pipeline_autoupdate_tracker_if_ssd(void) {
    if (sentai_tpu_num_outputs() < 4) return 0;
    int n = ssd_decode_into(300);   /* 30% threshold default */
    if (n <= 0) return 0;
    return sentai_tracker_update(s_dets, n,
                                  s_pipe_resized,
                                  SIM_SSD_INPUT_W, SIM_SSD_INPUT_H, SIM_SSD_INPUT_C,
                                  0 /* zp */, s_det_frame_seq);
}

/* sentai.pipeline.detections(thresh_permil=300) — returns list of
 * (x1, y1, x2, y2, conf_permil, class_id).  Decodes from the CURRENT
 * cached TPU output (last invoke).  Caller is expected to have done
 * pipeline.step() (or pipeline.start) before reading. */
static mp_obj_t sentai_pipeline_detections_mp(size_t n_args, const mp_obj_t* args) {
    int thresh = (n_args >= 1) ? mp_obj_get_int(args[0]) : 300;
    int n = ssd_decode_into(thresh);
    if (n < 0) return mp_obj_new_tuple(0, NULL);
    mp_obj_t out[SIM_MAX_DETS];
    for (int i = 0; i < n; ++i) {
        mp_obj_t t[6] = {
            mp_obj_new_int(s_dets[i].x1),
            mp_obj_new_int(s_dets[i].y1),
            mp_obj_new_int(s_dets[i].x2),
            mp_obj_new_int(s_dets[i].y2),
            mp_obj_new_int(s_dets[i].conf_permil),
            mp_obj_new_int(s_dets[i].class_id),
        };
        out[i] = mp_obj_new_tuple(6, t);
    }
    return mp_obj_new_tuple(n, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_detections_obj, 0, 1, sentai_pipeline_detections_mp);

/* sentai.pipeline.tracker_update(thresh_permil=300) — decodes detections
 * from current TPU output then feeds them to SentAI-SORT.  Returns the
 * number of confirmed tracks after update. */
static mp_obj_t sentai_pipeline_tracker_update_mp(size_t n_args, const mp_obj_t* args) {
    int thresh = (n_args >= 1) ? mp_obj_get_int(args[0]) : 300;
    int n = ssd_decode_into(thresh);
    if (n < 0) return mp_obj_new_int(n);
    int confirmed = sentai_tracker_update(s_dets, n,
                                            s_pipe_resized,
                                            SIM_SSD_INPUT_W, SIM_SSD_INPUT_H, SIM_SSD_INPUT_C,
                                            0 /* zp */, s_det_frame_seq);
    return mp_obj_new_int(confirmed);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_update_obj, 0, 1, sentai_pipeline_tracker_update_mp);

/* sentai.pipeline.tracker_enable(bool) */
static mp_obj_t sentai_pipeline_tracker_enable_mp(mp_obj_t v) {
    int on = mp_obj_is_true(v) ? 1 : 0;
    if (on) { sentai_tracker_reset(); sentai_tracker_set_enabled(1); }
    else      sentai_tracker_set_enabled(0);
    return mp_obj_new_int(on);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_pipeline_tracker_enable_obj, sentai_pipeline_tracker_enable_mp);

/* sentai.pipeline.tracker_reset() */
static mp_obj_t sentai_pipeline_tracker_reset_mp(void) {
    sentai_tracker_reset();
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracker_reset_obj, sentai_pipeline_tracker_reset_mp);

/* sentai.pipeline.tracker_tracks() — list of (id, class_id, x1,y1,x2,y2,
 * conf_permil, state, hits, age) */
#define SIM_MAX_TRACKS 16
static mp_obj_t sentai_pipeline_tracker_tracks_mp(void) {
    TrackedObject buf[SIM_MAX_TRACKS];
    int n = sentai_tracker_get_tracks(buf, SIM_MAX_TRACKS);
    if (n < 0) n = 0;
    mp_obj_t out[SIM_MAX_TRACKS];
    for (int i = 0; i < n; ++i) {
        mp_obj_t t[10] = {
            mp_obj_new_int(buf[i].id),
            mp_obj_new_int(buf[i].class_id),
            mp_obj_new_int(buf[i].x1),
            mp_obj_new_int(buf[i].y1),
            mp_obj_new_int(buf[i].x2),
            mp_obj_new_int(buf[i].y2),
            mp_obj_new_int(buf[i].conf_permil),
            mp_obj_new_int(buf[i].state),
            mp_obj_new_int(buf[i].hits),
            mp_obj_new_int(buf[i].age_frames),
        };
        out[i] = mp_obj_new_tuple(10, t);
    }
    return mp_obj_new_tuple(n, out);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracker_tracks_obj, sentai_pipeline_tracker_tracks_mp);

/* sentai.pipeline.tracker_camera(cam_id, fov_h_deg, fov_v_deg) — set
 * camera intrinsics for the projection path.  Mount angles default to
 * 0 (drone looking straight down). */
static mp_obj_t sentai_pipeline_tracker_camera_mp(size_t n_args, const mp_obj_t* args) {
    int cam = (n_args >= 1) ? mp_obj_get_int(args[0]) : 0;
    CameraConfig cfg = {0};
    cfg.fov_h_deg = (n_args >= 2) ? mp_obj_get_float(args[1]) : 58.0f;
    cfg.fov_v_deg = (n_args >= 3) ? mp_obj_get_float(args[2]) : 45.0f;
    cfg.mount_pitch_deg = 0; cfg.mount_roll_deg = 0; cfg.mount_yaw_deg = 0;
    cfg.ground_ref = 0;  // CENTROID — drone overhead
    sentai_tracker_set_camera(cam, &cfg);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_camera_obj, 0, 3, sentai_pipeline_tracker_camera_mp);

/* sentai.pipeline.tracker_pose(altitude_cm, heading_deg=-1) — set drone
 * altitude + compass so tracks get ground-plane coordinates. */
static mp_obj_t sentai_pipeline_tracker_pose_mp(size_t n_args, const mp_obj_t* args) {
    int alt_cm  = (n_args >= 1) ? mp_obj_get_int(args[0]) : 100;
    int heading = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    sentai_tracker_set_pose(alt_cm, heading, 0.0f, 0.0f);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_pose_obj, 0, 2, sentai_pipeline_tracker_pose_mp);

/* sentai.pipeline.tracker_event(timeout_ms=0) — non-blocking event poll.
 * Returns None if no event, otherwise tuple
 * (type, id, class_id, x1,y1,x2,y2, conf_permil, frame_seq). */
static mp_obj_t sentai_pipeline_tracker_event_mp(size_t n_args, const mp_obj_t* args) {
    int timeout_ms = (n_args >= 1) ? mp_obj_get_int(args[0]) : 0;
    TrackEvent evt;
    if (!sentai_tracker_get_event(&evt, timeout_ms)) return mp_const_none;
    mp_obj_t t[9] = {
        mp_obj_new_int(evt.type),
        mp_obj_new_int(evt.id),
        mp_obj_new_int(evt.class_id),
        mp_obj_new_int(evt.x1),
        mp_obj_new_int(evt.y1),
        mp_obj_new_int(evt.x2),
        mp_obj_new_int(evt.y2),
        mp_obj_new_int(evt.conf_permil),
        mp_obj_new_int_from_uint(evt.frame_seq),
    };
    return mp_obj_new_tuple(9, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_event_obj, 0, 1, sentai_pipeline_tracker_event_mp);

static const mp_rom_map_elem_t sentai_pipeline_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_pipeline) },
    { MP_ROM_QSTR(MP_QSTR_step),        MP_ROM_PTR(&sentai_pipeline_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&sentai_pipeline_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&sentai_pipeline_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),     MP_ROM_PTR(&sentai_pipeline_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),       MP_ROM_PTR(&sentai_pipeline_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_target_fps),  MP_ROM_PTR(&sentai_pipeline_target_fps_obj) },
    { MP_ROM_QSTR(MP_QSTR_predict),      MP_ROM_PTR(&sentai_pipeline_detect_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracks),      MP_ROM_PTR(&sentai_pipeline_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_infer_reset), MP_ROM_PTR(&sentai_pipeline_track_reset_obj) },
    /* SSD + SORT bindings */
    { MP_ROM_QSTR(MP_QSTR_detections),     MP_ROM_PTR(&sentai_pipeline_detections_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_update), MP_ROM_PTR(&sentai_pipeline_tracker_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_enable), MP_ROM_PTR(&sentai_pipeline_tracker_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_reset),  MP_ROM_PTR(&sentai_pipeline_tracker_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_tracks), MP_ROM_PTR(&sentai_pipeline_tracker_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_camera), MP_ROM_PTR(&sentai_pipeline_tracker_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_pose),   MP_ROM_PTR(&sentai_pipeline_tracker_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_event),  MP_ROM_PTR(&sentai_pipeline_tracker_event_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pipeline_globals, sentai_pipeline_globals_table);
static const mp_obj_module_t sentai_pipeline_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_pipeline_globals,
};

/* ===== top-level sentai module ===== */
/* ===== sentai.link — SIM MAVLink bridge (Phase 6) ============================
 * Slim mirror of examples/sentai_runtime/modsentai_link.c.  Same Python
 * surface as ARM (init/stop/debug/heartbeat/send/stats); transport is
 * UDP via sentai_uart_serial_udp.c instead of LPUART.
 */
extern int sentai_link_init(uint32_t baudrate, uint8_t sysid, uint8_t compid);
extern int sentai_link_stop(void);
extern void sentai_link_set_debug(int level);
extern int sentai_link_send_heartbeat(uint8_t type);
extern int sentai_link_send_statustext(uint8_t severity, const char* text);
extern void sentai_link_get_stats(uint32_t out[8]);
extern int sentai_link_cmd_arm(int do_arm);
extern int sentai_link_cmd_takeoff(float altitude_m);
extern int sentai_link_cmd_land(void);
extern int sentai_link_cmd_set_mode(uint8_t main_mode, uint8_t sub_mode);
extern int sentai_link_send_flow(float dx_rad, float dy_rad,
                                  uint32_t dt_us, uint8_t quality,
                                  float distance_m);

static mp_obj_t sim_link_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 57600;
    uint8_t sysid  = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 1;
    uint8_t compid = (n_args > 2) ? (uint8_t)mp_obj_get_int(args[2]) : 191;
    return mp_obj_new_int(sentai_link_init(baudrate, sysid, compid));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_init_obj, 0, 3, sim_link_init);

static mp_obj_t sim_link_stop(void) { return mp_obj_new_int(sentai_link_stop()); }
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_stop_obj, sim_link_stop);

static mp_obj_t sim_link_debug(mp_obj_t lvl) {
    sentai_link_set_debug(mp_obj_get_int(lvl));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sim_link_debug_obj, sim_link_debug);

static mp_obj_t sim_link_heartbeat(size_t n_args, const mp_obj_t *args) {
    uint8_t type = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 18;
    return mp_obj_new_int(sentai_link_send_heartbeat(type));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_heartbeat_obj, 0, 1, sim_link_heartbeat);

static mp_obj_t sim_link_send(size_t n_args, const mp_obj_t *args) {
    const char* text = mp_obj_str_get_str(args[0]);
    uint8_t severity = (n_args > 1) ? (uint8_t)mp_obj_get_int(args[1]) : 6;
    return mp_obj_new_int(sentai_link_send_statustext(severity, text));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_send_obj, 1, 2, sim_link_send);

static mp_obj_t sim_link_stats(void) {
    uint32_t s[8] = {0};
    sentai_link_get_stats(s);
    mp_obj_t items[8] = {
        mp_obj_new_int_from_uint(s[0]), mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]), mp_obj_new_int_from_uint(s[3]),
        mp_obj_new_int_from_uint(s[4]), mp_obj_new_int_from_uint(s[5]),
        mp_obj_new_int_from_uint(s[6]), mp_obj_new_int_from_uint(s[7]),
    };
    return mp_obj_new_tuple(8, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_stats_obj, sim_link_stats);

static mp_obj_t sim_link_arm(size_t n_args, const mp_obj_t *args) {
    int do_arm = (n_args > 0) ? mp_obj_get_int(args[0]) : 1;
    return mp_obj_new_int(sentai_link_cmd_arm(do_arm));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_arm_obj, 0, 1, sim_link_arm);

static mp_obj_t sim_link_takeoff(size_t n_args, const mp_obj_t *args) {
    float alt = (n_args > 0) ? mp_obj_get_float(args[0]) : 1.0f;
    return mp_obj_new_int(sentai_link_cmd_takeoff(alt));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_takeoff_obj, 0, 1, sim_link_takeoff);

static mp_obj_t sim_link_land(void) {
    return mp_obj_new_int(sentai_link_cmd_land());
}
static MP_DEFINE_CONST_FUN_OBJ_0(sim_link_land_obj, sim_link_land);

/* sentai.link.send_flow(dx_rad, dy_rad, dt_us, quality=200, distance_m=1.0)
 * Build + send MAVLINK_MSG_ID_OPTICAL_FLOW_RAD to PX4.  Used by hover
 * scripts to feed sentai.flow into PX4 EKF2 (set EKF2_AID_MASK to
 * include flow). */
static mp_obj_t sim_link_send_flow(size_t n_args, const mp_obj_t *args) {
    float dx = mp_obj_get_float(args[0]);
    float dy = mp_obj_get_float(args[1]);
    uint32_t dt = (uint32_t)mp_obj_get_int(args[2]);
    uint8_t q = (n_args > 3) ? (uint8_t)mp_obj_get_int(args[3]) : 200;
    float dist = (n_args > 4) ? mp_obj_get_float(args[4]) : 1.0f;
    return mp_obj_new_int(sentai_link_send_flow(dx, dy, dt, q, dist));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sim_link_send_flow_obj, 3, 5, sim_link_send_flow);

/* set_mode: dropped from binding for MVP (QSTR `set_mode` not in pool).
 * MP_REGISTER_MODULE regen needed to expose it.  In the meantime
 * sentai.link.send_command_long(176, ...) (also not exposed yet) or
 * direct python pymavlink-from-host is the workaround. */

static const mp_rom_map_elem_t sentai_link_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_link) },
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&sim_link_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&sim_link_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),     MP_ROM_PTR(&sim_link_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_heartbeat), MP_ROM_PTR(&sim_link_heartbeat_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),      MP_ROM_PTR(&sim_link_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),     MP_ROM_PTR(&sim_link_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),       MP_ROM_PTR(&sim_link_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),   MP_ROM_PTR(&sim_link_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),      MP_ROM_PTR(&sim_link_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_flow), MP_ROM_PTR(&sim_link_send_flow_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_link_globals, sentai_link_globals_table);
static const mp_obj_module_t sentai_link_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_link_globals,
};


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
    { MP_ROM_QSTR(MP_QSTR_pipeline), MP_ROM_PTR(&sentai_pipeline_module) },
    { MP_ROM_QSTR(MP_QSTR_link),     MP_ROM_PTR(&sentai_link_module) },
};
static MP_DEFINE_CONST_DICT(sentai_globals, sentai_globals_table);

/* This symbol is referenced from moduledefs.h (generated for the firmware
 * build).  Replaces the empty stub previously in main_sim.c. */
const mp_obj_module_t mp_module_sentai = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_globals,
};
