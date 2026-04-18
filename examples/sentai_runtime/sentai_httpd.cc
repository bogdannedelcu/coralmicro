// sentai_httpd.cc — HTTP file browser server for SentAI runtime
// browser.html is cached in SDRAM at startup for fast serving.
// All API routes operate on LfsUser() (user partition).
//
// NOTE: LFS operations in URI handler run in tcpip thread context.
// This is safe for quick reads but could block network stack briefly.
// For heavy filesystem operations, use REPL (sentai.fs.* functions).

#include <cstdio>
#include <cstring>
#include <vector>

#include "browser_html_data.h"
#include "libs/base/filesystem.h"
#include "libs/base/http_server.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" int sentai_usb_drive_get(void);
extern "C" void sentai_http_activity(void);  // Crash monitor hook

namespace {

// POST result: fixed buffer — no heap allocation in tcpip thread.
// Sized for the longest possible JSON error response.
static char g_post_result[256] = "{\"ok\":false}";

// Browser HTML cache — static SDRAM buffer, never heap-allocated.
// Fits user-uploaded browser.html; falls back to ROM data if too large.
static constexpr size_t kBrowserHtmlCacheMax = 64 * 1024;  // 64 KB
static uint8_t g_browser_html_cache_buf[kBrowserHtmlCacheMax]
    __attribute__((section(".sdram_bss")));
static size_t g_browser_html_cache_len = 0;
// Convenience helpers
static const uint8_t* BrowserHtmlData(void) {
  return g_browser_html_cache_len ? g_browser_html_cache_buf : browser_html_data;
}
static size_t BrowserHtmlSize(void) {
  return g_browser_html_cache_len ? g_browser_html_cache_len : browser_html_data_len;
}

// ── Path validation ──────────────────────────────────────────────────────
static bool ValidPath(const char* p) {
  return p && p[0] == '/' && !strstr(p, "..");
}

// ── JSON helpers ─────────────────────────────────────────────────────────
static void JsonAppend(std::vector<uint8_t>& v, const char* s) {
  v.insert(v.end(), s, s + strlen(s));
}
static void JsonAppendEscaped(std::vector<uint8_t>& v, const char* s) {
  v.push_back('"');
  for (; *s; ++s) {
    if (*s == '"' || *s == '\\') v.push_back('\\');
    v.push_back(static_cast<uint8_t>(*s));
  }
  v.push_back('"');
}

// ── GET /api/ls/<path> → JSON array ─────────────────────────────────────
static std::vector<uint8_t> ApiLs(const char* path) {
  lfs_dir_t dir;
  std::vector<uint8_t> json;
  json.reserve(2048);

  if (lfs_dir_open(coralmicro::LfsUser(), &dir, path) < 0) {
    JsonAppend(json, "[]");
    return json;
  }

  json.push_back('[');
  struct lfs_info info;
  bool first = true;
  while (lfs_dir_read(coralmicro::LfsUser(), &dir, &info) > 0) {
    // Skip . and ..
    if (info.name[0] == '.' &&
        (info.name[1] == '\0' ||
         (info.name[1] == '.' && info.name[2] == '\0')))
      continue;

    if (!first) json.push_back(',');
    first = false;

    json.push_back('{');
    JsonAppend(json, "\"name\":");
    JsonAppendEscaped(json, info.name);
    char buf[64];
    snprintf(buf, sizeof(buf), ",\"type\":\"%s\",\"size\":%lu",
             info.type == LFS_TYPE_DIR ? "dir" : "file",
             (unsigned long)info.size);
    JsonAppend(json, buf);
    json.push_back('}');
  }
  lfs_dir_close(coralmicro::LfsUser(), &dir);
  json.push_back(']');
  return json;
}

// ── Read a file from LfsUser into a vector ──────────────────────────────
// Hard cap of 256 KB: large file downloads belong in the REPL/sentai.fs.
static constexpr size_t kReadUserFileMaxBytes = 256 * 1024;
static std::vector<uint8_t> ReadUserFile(const char* path) {
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), path, &info) < 0 ||
      info.type != LFS_TYPE_REG || info.size > kReadUserFileMaxBytes)
    return {};
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, path, LFS_O_RDONLY) < 0)
    return {};
  std::vector<uint8_t> data(info.size);
  lfs_ssize_t n =
      lfs_file_read(coralmicro::LfsUser(), &f, data.data(), data.size());
  lfs_file_close(coralmicro::LfsUser(), &f);
  if (n != static_cast<lfs_ssize_t>(info.size)) return {};
  return data;
}

// ── Deploy/cache browser.html at startup ────────────────────────────────
// Called from main task before httpd starts - safe to use LFS directly.
static void LoadBrowserHtmlCache(void) {
  g_browser_html_cache_len = 0;

  // Try user-uploaded browser.html first
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), "/.sys/browser.html", &info) >= 0 &&
      info.type == LFS_TYPE_REG && info.size > 0 &&
      info.size <= kBrowserHtmlCacheMax) {
    lfs_file_t f;
    if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                      LFS_O_RDONLY) >= 0) {
      lfs_ssize_t n = lfs_file_read(coralmicro::LfsUser(), &f,
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

  // Deploy embedded version to LFS (idempotent)
  lfs_mkdir(coralmicro::LfsUser(), "/.sys");
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                    LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) >= 0) {
    lfs_file_write(coralmicro::LfsUser(), &f, browser_html_data,
                   browser_html_data_len);
    lfs_file_close(coralmicro::LfsUser(), &f);
    printf("[httpd] Deployed embedded browser.html (%u bytes)\r\n",
           browser_html_data_len);
  }
  // Use ROM data directly — don't copy to SDRAM buffer
  // (BrowserHtmlData() falls back to browser_html_data when cache_len == 0)
}

// ── URI handler (GET requests) ───────────────────────────────────────────
static coralmicro::HttpServer::Content SentaiUriHandler(const char* uri) {
  // Track HTTP activity for crash monitoring
  sentai_http_activity();
  
  // Serve browser UI from cache (no LFS access)
  if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0 ||
      strcmp(uri, "/index.shtml") == 0 || strcmp(uri, "/browser.html") == 0) {
    return coralmicro::HttpServer::StaticBuffer{BrowserHtmlData(), BrowserHtmlSize()};
  }

  // POST result endpoint
  if (strcmp(uri, "/api/_pr") == 0) {
    return coralmicro::HttpServer::StaticBuffer{
        reinterpret_cast<const uint8_t*>(g_post_result), strlen(g_post_result)};
  }

  // Directory listing → JSON
  if (strncmp(uri, "/api/ls", 7) == 0) {
    const char* path = uri + 7;
    if (*path == '\0' || strcmp(path, "/") == 0) path = "/";
    else if (!ValidPath(path)) path = "/";
    return ApiLs(path);
  }

  // Raw file content
  if (strncmp(uri, "/api/raw", 8) == 0) {
    const char* path = uri + 8;
    if (!ValidPath(path)) return {};
    return ReadUserFile(path);
  }

  return {};
}

// ── Server subclass with POST support ────────────────────────────────────
class SentaiHttpServer : public coralmicro::HttpServer {
 public:
  err_t PostBegin(void* connection, const char* uri,
                  const char* http_request, u16_t http_request_len,
                  int content_len, char* response_uri,
                  u16_t response_uri_len, u8_t* post_auto_wnd) override {
    // Cap uploads at 512 KB to bound heap usage in tcpip thread.
    // Larger transfers belong in the REPL.
    if (content_len > 512 * 1024) return ERR_MEM;
    snprintf(post_uri_, sizeof(post_uri_), "%s", uri);
    post_body_.clear();
    if (content_len > 0) post_body_.reserve(static_cast<size_t>(content_len));
    *post_auto_wnd = 1;
    return ERR_OK;
  }

  err_t PostReceiveData(void* connection, struct pbuf* p) override {
    for (struct pbuf* q = p; q; q = q->next) {
      post_body_.insert(post_body_.end(),
                        static_cast<uint8_t*>(q->payload),
                        static_cast<uint8_t*>(q->payload) + q->len);
    }
    pbuf_free(p);
    return ERR_OK;
  }

  void PostFinished(void* connection, char* response_uri,
                    u16_t response_uri_len) override {
    snprintf(g_post_result, sizeof(g_post_result),
             "{\"ok\":false,\"error\":\"unknown endpoint\"}");

    if (sentai_usb_drive_get()) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"USB drive active - call "
               "sentai.usb.drive(0) first\"}");
    } else if (strncmp(post_uri_, "/api/write/", 11) == 0) {
      DoWrite(post_uri_ + 10);
    } else if (strncmp(post_uri_, "/api/mkdir/", 11) == 0) {
      DoMkdir(post_uri_ + 10);
    } else if (strncmp(post_uri_, "/api/rm/", 8) == 0) {
      DoRemove(post_uri_ + 7);
    }

    post_body_.clear();
    post_body_.shrink_to_fit();
    snprintf(response_uri, response_uri_len, "/api/_pr");
  }

 private:
  void DoWrite(const char* path) {
    if (!ValidPath(path)) {
      snprintf(g_post_result, sizeof(g_post_result),
               "{\"ok\":false,\"error\":\"invalid path\"}");
      return;
    }
    // Ensure parent directories exist (walk a local copy of the path)
    char p[256];
    snprintf(p, sizeof(p), "%s", path);
    for (size_t i = 1; p[i]; ++i) {
      if (p[i] == '/') {
        p[i] = '\0';
        lfs_mkdir(coralmicro::LfsUser(), p);
        p[i] = '/';
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
        lfs_file_write(coralmicro::LfsUser(), &f, post_body_.data(),
                       post_body_.size());
    lfs_file_close(coralmicro::LfsUser(), &f);
    if (written == static_cast<lfs_ssize_t>(post_body_.size())) {
      // Refresh SDRAM cache if browser.html was updated and fits
      if (strcmp(path, "/.sys/browser.html") == 0 &&
          post_body_.size() <= kBrowserHtmlCacheMax) {
        memcpy(g_browser_html_cache_buf, post_body_.data(), post_body_.size());
        g_browser_html_cache_len = post_body_.size();
        printf("[httpd] browser.html cache refreshed (%u bytes)\r\n",
               (unsigned)post_body_.size());
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
        p[i] = '\0';
        lfs_mkdir(coralmicro::LfsUser(), p);
        p[i] = '/';
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
               "directory not empty?\"}\n", rc);
    }
  }

  // Fixed-length URI storage — avoids std::string heap alloc in tcpip thread.
  char post_uri_[256];
  std::vector<uint8_t> post_body_;
};

}  // namespace

extern "C" void sentai_httpd_start(void) {
  static bool started = false;
  if (started) return;
  started = true;
  
  // Cache browser.html at startup
  LoadBrowserHtmlCache();
  
  // Start HTTP server
  static SentaiHttpServer server;
  server.AddUriHandler(SentaiUriHandler);
  coralmicro::UseHttpServer(&server);
  printf("[httpd] File browser at http://10.0.0.1/\r\n");
}
