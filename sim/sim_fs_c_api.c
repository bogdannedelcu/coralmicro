/*
 * sim/sim_fs_c_api.c — C-side bridge for sentai_fs_* used by SIM modules.
 *
 * ARM exposes C-callable sentai_fs_write/read/size from modsentai_hal.cc.
 * SIM equivalent: thin wrappers around stdio FILE* I/O rooted at the
 * SENTAI_SIM_FS_ROOT virtual filesystem.
 *
 * Why: modsentai_slam.c (and similar shared C modules) declare these
 * functions at file scope as `extern int ...` and call them from C
 * code paths (slam save/load).  On SIM we need real impls that
 * accept the same signature; without them link fails.
 *
 * NOTE: kept SEPARATE from sim/modsentai_sim.c so the MP wrapper
 * functions there (also named sentai_fs_write etc.) don't collide
 * — those are static / mp_obj_t-returning and live in a different TU.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* SIM FS root resolution (matches modsentai_sim.c::sim_fs_resolve). */
static const char* sim_fs_root(void) {
    const char* r = getenv("SENTAI_SIM_FS_ROOT");
    if (r && *r) return r;
    return "/tmp/sentai_fs_root";
}

static int sim_fs_resolve(const char* path, char* out, int outsz) {
    if (!path || !out || outsz < 2) return -1;
    const char* root = sim_fs_root();
    /* path starts with '/' → joined as root + path */
    int n = snprintf(out, outsz, "%s%s", root, (path[0] == '/' ? "" : "/"));
    if (n <= 0 || n >= outsz) return -1;
    int rem = outsz - n;
    int m = snprintf(out + n, rem, "%s", path);
    if (m <= 0 || m >= rem) return -1;
    return 0;
}

int sentai_fs_write(const char* path, const uint8_t* data, int len) {
    if (!path || !data || len < 0) return -1;
    char full[512];
    if (sim_fs_resolve(path, full, sizeof(full)) != 0) return -2;
    /* Create parent dir if needed (mkdir -p style). */
    char dir[512];
    strncpy(dir, full, sizeof(dir) - 1); dir[sizeof(dir) - 1] = 0;
    char* slash = strrchr(dir, '/');
    if (slash) {
        *slash = 0;
        struct stat st;
        if (stat(dir, &st) != 0) {
            char cmd[600];
            snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", dir);
            if (system(cmd) != 0) return -3;
        }
    }
    FILE* f = fopen(full, "wb");
    if (!f) return -4;
    size_t w = fwrite(data, 1, (size_t)len, f);
    fclose(f);
    return (int)w == len ? len : -5;
}

int sentai_fs_read(const char* path, uint8_t* buf, int max_size) {
    if (!path || !buf || max_size <= 0) return -1;
    char full[512];
    if (sim_fs_resolve(path, full, sizeof(full)) != 0) return -2;
    FILE* f = fopen(full, "rb");
    if (!f) return -3;
    size_t r = fread(buf, 1, (size_t)max_size, f);
    fclose(f);
    return (int)r;
}

int sentai_fs_size(const char* path) {
    if (!path) return -1;
    char full[512];
    if (sim_fs_resolve(path, full, sizeof(full)) != 0) return -2;
    struct stat st;
    if (stat(full, &st) != 0) return -3;
    return (int)st.st_size;
}
