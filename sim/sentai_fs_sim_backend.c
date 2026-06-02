/*
 * sentai_fs_sim_backend.c
 *
 * POSIX filesystem backend for the shared sentai.fs binding.  This file is
 * the simulator port layer: it maps board paths like "/images/cat.bmp" to the
 * per-run SENTAI_SIM_ROOT directory and implements the C ABI used by
 * examples/sentai_runtime/bindings/modsentai_fs.c.
 */

#include "sentai_fs_sim_backend.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef SENTAI_SIM_FS_ROOT_DEFAULT
#define SENTAI_SIM_FS_ROOT_DEFAULT "./sentai_sim_root"
#endif

static pthread_mutex_t s_fs_mu = PTHREAD_MUTEX_INITIALIZER;

static void mkdir_p_host_(const char* path) {
    char tmp[SIM_FS_MAXPATH + 1];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    for (char* p = tmp + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
}

const char* sim_fs_root(void) {
    static const char* cached = NULL;
    if (cached) return cached;
    const char* env = getenv("SENTAI_SIM_ROOT");
    cached = (env && env[0]) ? env : SENTAI_SIM_FS_ROOT_DEFAULT;
    mkdir_p_host_(cached);
    printf("[sim] FS root: %s\n", cached);
    return cached;
}

int sim_fs_resolve(const char* bpath, char* out, size_t outsz) {
    if (!bpath || !out || outsz == 0) return -1;
    const char* root = sim_fs_root();
    while (*bpath == '/') ++bpath;
    int n = snprintf(out, outsz, "%s/%s", root, bpath);
    if (n < 0 || (size_t)n >= outsz) return -1;
    return 0;
}

int sentai_fs_lock(void) {
    return pthread_mutex_lock(&s_fs_mu) == 0 ? 1 : 0;
}

void sentai_fs_unlock(void) {
    pthread_mutex_unlock(&s_fs_mu);
}

static int ensure_parent_dir_(const char* full_path) {
    char tmp[SIM_FS_MAXPATH + 1];
    strncpy(tmp, full_path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char* slash = strrchr(tmp, '/');
    if (!slash || slash == tmp) return 0;
    *slash = '\0';
    mkdir_p_host_(tmp);
    return 0;
}

int sentai_fs_write(const char* bpath, const uint8_t* data, int size) {
    if (!bpath || !data || size < 0) return -1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;
    ensure_parent_dir_(fp);

    int fd = open(fp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return -1;
    ssize_t w = write(fd, data, (size_t)size);
    close(fd);
    return w == (ssize_t)size ? 1 : 0;
}

int sentai_fs_append(const char* bpath, const uint8_t* data, int size) {
    if (!bpath || !data || size < 0) return -1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;
    ensure_parent_dir_(fp);

    int fd = open(fp, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd < 0) return -1;
    ssize_t w = write(fd, data, (size_t)size);
    close(fd);
    return w == (ssize_t)size ? 1 : 0;
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
    int total = 0;
    while (total < max_size) {
        ssize_t r = read(fd, buf + total, (size_t)(max_size - total));
        if (r < 0) {
            if (errno == EINTR) continue;
            close(fd);
            return -1;
        }
        if (r == 0) break;
        total += (int)r;
    }
    close(fd);
    return total;
}

int sentai_fs_file_exists(const char* bpath) {
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return 0;
    struct stat st;
    return (stat(fp, &st) == 0 && S_ISREG(st.st_mode)) ? 1 : 0;
}

int sentai_fs_dir_exists(const char* bpath) {
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return 0;
    struct stat st;
    return (stat(fp, &st) == 0 && S_ISDIR(st.st_mode)) ? 1 : 0;
}

int sentai_fs_makedirs(const char* bpath) {
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return 0;
    mkdir_p_host_(fp);
    return 1;
}

int sentai_fs_remove(const char* bpath) {
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -2;
    if (unlink(fp) == 0) return 0;
    if (errno == EISDIR && rmdir(fp) == 0) return 0;
    return errno == ENOENT ? -2 : -5;
}

int sentai_fs_sync(void) {
    sync();
    return 1;
}

int sentai_fs_format(void) {
    /* Deliberately conservative in SIM: experiment runners create fresh
     * fs_root directories.  Avoid recursive deletion from firmware-facing
     * code where a bad SENTAI_SIM_ROOT would be unpleasant. */
    return 1;
}

int sentai_fs_cache_write(const uint8_t* data, int size) {
    (void)data;
    (void)size;
    return -3;
}

typedef struct {
    void (*cb)(const char* name, int type, int size, void* ud);
    void* user_data;
} sim_list_ctx_t;

int sentai_fs_listdir(const char* bpath,
                      void (*callback)(const char* name, int type, int size,
                                       void* ud),
                      void* user_data) {
    if (!callback) return -1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) return -1;
    DIR* d = opendir(fp);
    if (!d) return -1;

    sim_list_ctx_t ctx = {callback, user_data};
    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        if (strcmp(de->d_name, ".") == 0) continue;
        if (strcmp(de->d_name, "..") == 0) continue;
        char child[SIM_FS_MAXPATH + 1];
        int n = snprintf(child, sizeof(child), "%s/%s", fp, de->d_name);
        if (n < 0 || (size_t)n >= sizeof(child)) continue;
        struct stat st;
        if (stat(child, &st) != 0) continue;
        int type = S_ISDIR(st.st_mode) ? 2 : (S_ISREG(st.st_mode) ? 1 : 0);
        int size = S_ISREG(st.st_mode) ? (int)st.st_size : 0;
        ctx.cb(de->d_name, type, size, ctx.user_data);
    }
    closedir(d);
    return 0;
}
