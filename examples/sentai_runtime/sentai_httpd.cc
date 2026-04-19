// sentai_httpd.cc — HTTP file browser, static allocations only.
//
// GET /api/ls  and  GET /api/raw  are handled by the dedicated lfs_task
// (priority 2) — tcpip_thread never blocks waiting for LFS.  The browser
// retries on {"error":"lfs_busy"} (already implemented in browser.html).
//
// POST /api/write, /api/mkdir, /api/rm run in tcpip_thread.  They call
// lfs_* directly; the internal g_lfs_user_mutex serialises per-call access.
// Worst-case stall (~700ms on lfs_file_close) is within USB NCM tolerance.

#include <cstdio>
#include <cstring>

#include "browser_html_data.h"
#include "libs/base/filesystem.h"
#include "libs/base/http_server.h"
#include "sentai_lfs_task.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"

extern "C" int  sentai_usb_drive_get(void);
extern "C" void sentai_http_activity(void);

namespace {

// ─── Static SDRAM buffers ─────────────────────────────────────────────────

static constexpr size_t kBrowserHtmlCacheMax = 64 * 1024;
static uint8_t g_browser_html_cache_buf[kBrowserHtmlCacheMax]
    __attribute__((section(".sdram_bss")));
static size_t g_browser_html_cache_len = 0;

// POST upload body (512 KB max)
static constexpr size_t kPostBufSize = 512 * 1024;
static uint8_t g_post_buf[kPostBufSize]
    __attribute__((section(".sdram_bss")));
static size_t g_post_buf_len = 0;
static char   g_post_uri[256] = {};

// POST result
static char g_post_result[256] = "{\"ok\":false}";

// ─── pextension sentinel ─────────────────────────────────────────────────

// Identifies responses that came from the lfs_task SDRAM buffer (g_resp_buf).
// FsCloseCustom calls sentai_lfs_resp_done() only for these.
static const uint8_t g_resp_tag = 0;

// ─── Browser HTML helpers ─────────────────────────────────────────────────

static const uint8_t* BrowserHtmlData() {
  return g_browser_html_cache_len ? g_browser_html_cache_buf : browser_html_data;
}
static size_t BrowserHtmlSize() {
  return g_browser_html_cache_len ? g_browser_html_cache_len : browser_html_data_len;
}

// ─── Path validation ──────────────────────────────────────────────────────

static bool ValidPath(const char* p) {
  return p && p[0] == '/' && !strstr(p, "..");
}

// ─── Deploy/cache browser.html at startup ────────────────────────────────

static void LoadBrowserHtmlCache() {
  g_browser_html_cache_len = 0;
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), "/.sys/browser.html", &info) >= 0 &&
      info.type == LFS_TYPE_REG && info.size > 0 &&
      info.size <= kBrowserHtmlCacheMax) {
    lfs_file_t f;
    if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                      LFS_O_RDONLY) >= 0) {
      lfs_ssize_t n =
          lfs_file_read(coralmicro::LfsUser(), &f,
                        g_browser_html_cache_buf, info.size);
      lfs_file_close(coralmicro::LfsUser(), &f);
      if (n == static_cast<lfs_ssize_t>(info.size)) {
        g_browser_html_cache_len = static_cast<size_t>(n);
        printf("[httpd] browser.html cached from LfsUser (%u bytes)\r\n",
               (unsigned)n);
        return;
      }
    }
  }
  lfs_mkdir(coralmicro::LfsUser(), "/.sys");
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                    LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) >= 0) {
    lfs_file_write(coralmicro::LfsUser(), &f,
                   browser_html_data, browser_html_data_len);
    lfs_file_close(coralmicro::LfsUser(), &f);
    printf("[httpd] Deployed embedded browser.html (%u bytes)\r\n",
           (unsigned)browser_html_data_len);
  }
}

// ─── HTTP server ──────────────────────────────────────────────────────────

// Static busy response — returned when lfs_task hasn't finished yet.
// Browser retries after 600ms (see go() in browser.html).
static const char kLfsBusy[] = "{\"error\":\"lfs_busy\"}";

class SentaiHttpServer : public coralmicro::HttpServer {
 public:
  int FsOpenCustom(struct fs_file* file, const char* name) override {
    std::memset(file, 0, sizeof(*file));

    // ── Static browser UI (never rate-limited) ──────────────────────────
    if (strcmp(name, "/") == 0 || strcmp(name, "/index.html") == 0 ||
        strcmp(name, "/index.shtml") == 0 ||
        strcmp(name, "/browser.html") == 0) {
      sentai_http_activity();
      file->data  = reinterpret_cast<const char*>(BrowserHtmlData());
      file->len   = static_cast<int>(BrowserHtmlSize());
      file->index = file->len;
      file->flags = FS_FILE_FLAGS_HEADER_PERSISTENT;
      return 1;
    }

    // ── POST result (tiny static string) ────────────────────────────────
    if (strcmp(name, "/api/_pr") == 0) {
      sentai_http_activity();
      file->data  = g_post_result;
      file->len   = static_cast<int>(strlen(g_post_result));
      file->index = file->len;
      file->flags = FS_FILE_FLAGS_HEADER_PERSISTENT;
      return 1;
    }

    // ── All other API endpoints ──────────────────────────────────────────
    if (strncmp(name, "/api/", 5) != 0) return 0;

    sentai_http_activity();

    // ── GET /api/ls and /api/raw — served via lfs_task (non-blocking) ──
    if (strncmp(name, "/api/ls", 7) == 0 || strncmp(name, "/api/raw", 8) == 0) {
      sentai_lfs_req_type_t req_type;
      const char* path;

      if (strncmp(name, "/api/ls", 7) == 0) {
        req_type = LFS_REQ_LS;
        path = name + 7;
        // lwIP's httpd rewrites trailing-slash URIs by appending one of its
        // default index filenames (index.shtml/ssi/shtm/html/htm), so a GET
        // for "/api/ls/" arrives here as "/api/ls/index.shtml".  Strip that
        // artefact so the caller's intent (list the parent dir) is honoured.
        static char ls_path_buf[256];
        size_t plen = strlen(path);
        static const char* const kIndexSuffixes[] = {
            "/index.shtml", "/index.ssi", "/index.shtm",
            "/index.html", "/index.htm"};
        for (size_t i = 0; i < sizeof(kIndexSuffixes)/sizeof(kIndexSuffixes[0]); ++i) {
          size_t slen = strlen(kIndexSuffixes[i]);
          if (plen >= slen && strcmp(path + plen - slen, kIndexSuffixes[i]) == 0) {
            size_t keep = plen - slen;
            if (keep >= sizeof(ls_path_buf)) keep = sizeof(ls_path_buf) - 1;
            memcpy(ls_path_buf, path, keep);
            ls_path_buf[keep] = '\0';
            path = ls_path_buf;
            break;
          }
        }
        if (*path == '\0' || strcmp(path, "/") == 0) path = "/";
        else if (!ValidPath(path)) path = "/";
      } else {
        req_type = LFS_REQ_RAW;
        path = name + 8;
        if (!ValidPath(path)) {
          return 0;  // 404
        }
      }

      size_t len = sentai_lfs_try_serve(req_type, path);
      if (len == (size_t)-1) {
        return 0;  // 404: file not found or empty
      }
      if (len == 0) {
        // Not ready — tell browser to retry.
        file->data  = kLfsBusy;
        file->len   = static_cast<int>(sizeof(kLfsBusy) - 1);
        file->index = file->len;
        file->flags = FS_FILE_FLAGS_HEADER_PERSISTENT;
        return 1;
      }

      // Data ready — stream from lfs_task SDRAM buffer.
      file->data  = reinterpret_cast<const char*>(sentai_lfs_resp_buf());
      file->len   = static_cast<int>(len);
      file->index = 0;
      file->flags = FS_FILE_FLAGS_HEADER_PERSISTENT;
      // g_resp_tag tells FsCloseCustom to call sentai_lfs_resp_done().
      file->pextension = const_cast<uint8_t*>(&g_resp_tag);
      return 1;
    }

    return 0;  // unknown API endpoint → 404
  }

  int FsReadCustom(struct fs_file* file, char* buffer, int count) override {
    (void)file; (void)buffer; (void)count;
    return FS_READ_EOF;
  }

  void FsCloseCustom(struct fs_file* file) override {
    if (file->pextension == &g_resp_tag) {
      sentai_lfs_resp_done();
    }
    // No heap to free — all buffers are static.
  }

  // ── POST ────────────────────────────────────────────────────────────────

  err_t PostBegin(void* connection, const char* uri,
                  const char* http_request, u16_t http_request_len,
                  int content_len, char* response_uri,
                  u16_t response_uri_len, u8_t* post_auto_wnd) override {
    (void)connection; (void)http_request; (void)http_request_len;
    (void)response_uri; (void)response_uri_len;
    if (content_len > static_cast<int>(kPostBufSize)) return ERR_MEM;
    snprintf(g_post_uri, sizeof(g_post_uri), "%s", uri);
    g_post_buf_len = 0;
    *post_auto_wnd = 1;
    return ERR_OK;
  }

  err_t PostReceiveData(void* connection, struct pbuf* p) override {
    (void)connection;
    for (struct pbuf* q = p; q; q = q->next) {
      size_t avail = kPostBufSize - g_post_buf_len;
      size_t copy  = (q->len < avail) ? q->len : avail;
      memcpy(g_post_buf + g_post_buf_len, q->payload, copy);
      g_post_buf_len += copy;
    }
    pbuf_free(p);
    return ERR_OK;
  }

  void PostFinished(void* connection, char* response_uri,
                    u16_t response_uri_len) override {
    (void)connection;
    snprintf(g_post_result, sizeof(g_post_result),
             "{\"ok\":false,\"error\":\"unknown endpoint\"}");

    if (sentai_usb_drive_get()) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"USB drive active — call "
               "sentai.usb.drive(0) first\"}");
    } else if (strncmp(g_post_uri, "/api/write/", 11) == 0) {
      DoWrite(g_post_uri + 10);
    } else if (strncmp(g_post_uri, "/api/mkdir/", 11) == 0) {
      DoMkdir(g_post_uri + 10);
    } else if (strncmp(g_post_uri, "/api/rm/", 8) == 0) {
      DoRemove(g_post_uri + 7);
    }

    snprintf(response_uri, response_uri_len, "/api/_pr");
  }

 private:
  // POST handlers run in tcpip_thread. They call lfs_* directly without
  // holding s_lfs_mutex — the internal g_lfs_user_mutex serialises each call.
  // Worst-case stall: ~700ms on lfs_file_close (flash GC); acceptable for
  // user-initiated writes (infrequent, within USB NCM watchdog tolerance).

  void DoWrite(const char* path) {
    if (!ValidPath(path)) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"invalid path\"}");
      return;
    }
    char p[256];
    snprintf(p, sizeof(p), "%s", path);
    for (size_t i = 1; p[i]; ++i) {
      if (p[i] == '/') {
        p[i] = '\0'; lfs_mkdir(coralmicro::LfsUser(), p); p[i] = '/';
      }
    }
    lfs_file_t f;
    int rc = lfs_file_open(coralmicro::LfsUser(), &f, path,
                           LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (rc < 0) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"open failed (%d)\"}", rc);
      return;
    }
    lfs_ssize_t written =
        lfs_file_write(coralmicro::LfsUser(), &f, g_post_buf, g_post_buf_len);
    lfs_file_close(coralmicro::LfsUser(), &f);
    if (written == static_cast<lfs_ssize_t>(g_post_buf_len)) {
      if (strcmp(path, "/.sys/browser.html") == 0 &&
          g_post_buf_len <= kBrowserHtmlCacheMax) {
        memcpy(g_browser_html_cache_buf, g_post_buf, g_post_buf_len);
        g_browser_html_cache_len = g_post_buf_len;
        printf("[httpd] browser.html cache refreshed (%u bytes)\r\n",
               (unsigned)g_post_buf_len);
      }
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":true,\"size\":%ld}", (long)written);
    } else {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"write incomplete\"}");
    }
  }

  void DoMkdir(const char* path) {
    if (!ValidPath(path)) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"invalid path\"}");
      return;
    }
    char p[256];
    snprintf(p, sizeof(p), "%s", path);
    for (size_t i = 1; p[i]; ++i) {
      if (p[i] == '/') {
        p[i] = '\0'; lfs_mkdir(coralmicro::LfsUser(), p); p[i] = '/';
      }
    }
    int rc = lfs_mkdir(coralmicro::LfsUser(), p);
    if (rc >= 0 || rc == LFS_ERR_EXIST) {
      snprintf(g_post_result, sizeof(g_post_result), "{\"ok\":true}");
    } else {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"mkdir failed (%d)\"}", rc);
    }
  }

  void DoRemove(const char* path) {
    if (!ValidPath(path)) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"invalid path\"}");
      return;
    }
    int rc = lfs_remove(coralmicro::LfsUser(), path);
    if (rc >= 0) {
      snprintf(g_post_result, sizeof(g_post_result), "{\"ok\":true}");
    } else {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"remove failed (%d), "
               "directory not empty?\"}", rc);
    }
  }
};

}  // namespace

extern "C" void sentai_httpd_start(void) {
  static bool started = false;
  if (started) return;
  started = true;

  // LFS task must be started before we can serve GET requests.
  sentai_lfs_task_start();

  LoadBrowserHtmlCache();

  static SentaiHttpServer server;
  coralmicro::UseHttpServer(&server);
  printf("[httpd] File browser at http://10.0.0.1/\r\n");
}
