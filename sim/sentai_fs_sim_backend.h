#pragma once

#include <stddef.h>

#define SIM_FS_MAXPATH 512

#ifdef __cplusplus
extern "C" {
#endif

const char* sim_fs_root(void);
int sim_fs_resolve(const char* bpath, char* out, size_t outsz);

#ifdef __cplusplus
}
#endif
