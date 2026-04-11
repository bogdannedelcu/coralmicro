// sentai_httpd.cc — HTTP file browser server for SentAI runtime
// Serves browser.html from LfsUser() (deployed from embedded fallback on first run).
// All API routes operate on LfsUser() (user partition).

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "browser_html_data.h"
#include "libs/base/filesystem.h"
#include "libs/base/http_server.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" int sentai_usb_drive_get(void);

namespace {

// POST result stored here; read back via /api/_pr handler.
// Safe: lwIP httpd is single-threaded (tcpip thread).
static std::string g_post_result = "{\"ok\":false}";

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
static std::vector<uint8_t> ReadUserFile(const char* path) {
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), path, &info) < 0 ||
      info.type != LFS_TYPE_REG || info.size > 8 * 1024 * 1024)
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

// ── Deploy embedded browser.html to LfsUser if not present ──────────────
static void DeployBrowserHtml(void) {
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), "/.sys/browser.html", &info) >= 0) {
    printf("[httpd] browser.html found on LfsUser (%lu bytes)\r\n",
           (unsigned long)info.size);
    return;
  }
  // Create /.sys/ directory and write embedded data
  lfs_mkdir(coralmicro::LfsUser(), "/.sys");
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                     LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) < 0) {
    printf("[httpd] ERROR: cannot create /.sys/browser.html\r\n");
    return;
  }
  lfs_file_write(coralmicro::LfsUser(), &f, browser_html_data,
                  browser_html_data_len);
  lfs_file_close(coralmicro::LfsUser(), &f);
  printf("[httpd] Deployed browser.html to LfsUser (%u bytes)\r\n",
         browser_html_data_len);
}

// ── URI handler (GET requests) ───────────────────────────────────────────
static coralmicro::HttpServer::Content SentaiUriHandler(const char* uri) {
  // Serve browser UI from LfsUser, fallback to embedded
  if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0 ||
      strcmp(uri, "/index.shtml") == 0 || strcmp(uri, "/browser.html") == 0) {
    auto data = ReadUserFile("/.sys/browser.html");
    if (!data.empty()) return data;
    return std::vector<uint8_t>(browser_html_data,
                                browser_html_data + browser_html_data_len);
  }

  // POST result endpoint
  if (strcmp(uri, "/api/_pr") == 0) {
    return std::vector<uint8_t>(g_post_result.begin(), g_post_result.end());
  }

  // Directory listing → JSON
  if (strncmp(uri, "/api/ls", 7) == 0) {
    const char* path = uri + 7;
    if (*path == '\0' || strcmp(path, "/") == 0) path = "";
    if (!ValidPath(path) && *path != '\0') return ApiLs("");
    return ApiLs(path);
  }

  // Raw file content (served from user LFS partition)
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
    if (content_len > 8 * 1024 * 1024) return ERR_MEM;
    post_uri_ = uri;
    post_body_.clear();
    if (content_len > 0) post_body_.reserve(content_len);
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
    g_post_result = "{\"ok\":false,\"error\":\"unknown endpoint\"}";

    if (sentai_usb_drive_get()) {
      g_post_result =
          "{\"ok\":false,\"error\":\"USB drive active - call "
          "sentai.usb.drive(0) first\"}";
    } else if (strncmp(post_uri_.c_str(), "/api/write/", 11) == 0) {
      DoWrite(post_uri_.c_str() + 10);
    } else if (strncmp(post_uri_.c_str(), "/api/mkdir/", 11) == 0) {
      DoMkdir(post_uri_.c_str() + 10);
    } else if (strncmp(post_uri_.c_str(), "/api/rm/", 8) == 0) {
      DoRemove(post_uri_.c_str() + 7);
    }

    post_body_.clear();
    post_body_.shrink_to_fit();
    snprintf(response_uri, response_uri_len, "/api/_pr");
  }

 private:
  void DoWrite(const char* path) {
    if (!ValidPath(path)) {
      g_post_result = "{\"ok\":false,\"error\":\"invalid path\"}";
      return;
    }
    // Ensure parent directories exist
    std::string p(path);
    for (size_t i = 1; i < p.size(); ++i) {
      if (p[i] == '/') {
        p[i] = '\0';
        lfs_mkdir(coralmicro::LfsUser(), p.c_str());
        p[i] = '/';
      }
    }
    lfs_file_t f;
    int rc = lfs_file_open(coralmicro::LfsUser(), &f, path,
                           LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (rc < 0) {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "{\"ok\":false,\"error\":\"open failed (%d)\"}", rc);
      g_post_result = buf;
      return;
    }
    lfs_ssize_t written =
        lfs_file_write(coralmicro::LfsUser(), &f, post_body_.data(),
                       post_body_.size());
    lfs_file_close(coralmicro::LfsUser(), &f);
    if (written == static_cast<lfs_ssize_t>(post_body_.size())) {
      char buf[64];
      snprintf(buf, sizeof(buf), "{\"ok\":true,\"size\":%lu}",
               (unsigned long)written);
      g_post_result = buf;
    } else {
      g_post_result = "{\"ok\":false,\"error\":\"write incomplete\"}";
    }
  }

  void DoMkdir(const char* path) {
    if (!ValidPath(path)) {
      g_post_result = "{\"ok\":false,\"error\":\"invalid path\"}";
      return;
    }
    std::string p(path);
    for (size_t i = 1; i < p.size(); ++i) {
      if (p[i] == '/') {
        p[i] = '\0';
        lfs_mkdir(coralmicro::LfsUser(), p.c_str());
        p[i] = '/';
      }
    }
    int rc = lfs_mkdir(coralmicro::LfsUser(), p.c_str());
    if (rc >= 0 || rc == LFS_ERR_EXIST) {
      g_post_result = "{\"ok\":true}";
    } else {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "{\"ok\":false,\"error\":\"mkdir failed (%d)\"}", rc);
      g_post_result = buf;
    }
  }

  void DoRemove(const char* path) {
    if (!ValidPath(path)) {
      g_post_result = "{\"ok\":false,\"error\":\"invalid path\"}";
      return;
    }
    int rc = lfs_remove(coralmicro::LfsUser(), path);
    if (rc >= 0) {
      g_post_result = "{\"ok\":true}";
    } else {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "{\"ok\":false,\"error\":\"remove failed (%d), "
               "directory not empty?\"}",
               rc);
      g_post_result = buf;
    }
  }

  std::string post_uri_;
  std::vector<uint8_t> post_body_;
};

}  // namespace

extern "C" void sentai_httpd_start(void) {
  static bool started = false;
  if (started) return;
  started = true;
  DeployBrowserHtml();
  static SentaiHttpServer server;
  server.AddUriHandler(SentaiUriHandler);
  coralmicro::UseHttpServer(&server);
  printf("[httpd] File browser at http://10.0.0.1/\r\n");
}
