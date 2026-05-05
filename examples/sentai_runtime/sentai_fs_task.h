#pragma once
// sentai_fs_task.h — Dedicated LFS task interface.
//
// All LFS GET operations (directory listing, file reads) run in a dedicated
// lfs_task (priority 2) rather than in tcpip_thread (priority 4, highest).
// This eliminates USB NCM stalls caused by tcpip_thread blocking inside
// lfs_dir_open() / lfs_file_read() when MicroPython holds g_lfs_user_mutex
// during a flash write (~700ms).
//
// tcpip_thread never blocks: sentai_fs_try_serve() is always non-blocking.
// The browser retries on lfs_busy (already implemented in browser.html).

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { FS_REQ_LS = 1, FS_REQ_RAW = 2 } sentai_fs_req_type_t;

// Start LFS task (call once from sentai_runtime at boot, before httpd).
void sentai_fs_task_start(void);

// Try to serve a GET request. NEVER BLOCKS.
//   - IDLE:    posts request to lfs_task, returns 0 (retry)
//   - PENDING: lfs_task working, returns 0 (retry)
//   - READY (path match): returns byte count in sentai_fs_resp_buf(), caller must
//                         call sentai_fs_resp_done() when done streaming
//   - READY (path mismatch): discards, re-queues new request, returns 0 (retry)
//   - SERVING: previous response still streaming, returns 0 (retry)
size_t sentai_fs_try_serve(sentai_fs_req_type_t type, const char* path);

// Pointer to the 256KB SDRAM response buffer filled by the lfs_task.
uint8_t* sentai_fs_resp_buf(void);

// Signal that lwIP has finished streaming from sentai_fs_resp_buf().
// Must be called from FsCloseCustom after a successful sentai_fs_try_serve().
void sentai_fs_resp_done(void);

// Application-level LFS mutex — serializes multi-step lfs_* sequences across tasks.
// Timeout: 2000ms. Returns 1 = acquired, 0 = busy (caller should skip/fail gracefully).
// Do NOT call from tcpip_thread — only from task contexts (hw_wdog, mp_repl, lfs_task).
int  sentai_fs_lock(void);
void sentai_fs_unlock(void);

#ifdef __cplusplus
}
#endif
