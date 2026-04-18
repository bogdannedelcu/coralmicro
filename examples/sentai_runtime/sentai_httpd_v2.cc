// sentai_httpd_v2.cc — HTTP file browser using NXP httpsrv (socket-based)
// More stable than lwIP httpd - uses blocking sockets in dedicated task.
// All API routes operate on LfsUser() (user partition).

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "browser_html_data.h"
#include "libs/base/filesystem.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/apps/httpsrv/httpsrv.h"

extern "C" int sentai_usb_drive_get(void);

namespace {

// browser.html cached in SDRAM at startup
static std::vector<uint8_t> g_browser_html_cache;

// ── Path validation ──────────────────────────────────────────────────────
static bool ValidPath(const char* p) {
  return p && p[0] == '/' && !strstr(p, "..");
}

// ── Send HTTP response ───────────────────────────────────────────────────
static void SendResponse(HTTPSRV_CGI_RES_STRUCT* res, int status,
                         const char* content_type, const void* data, size_t len) {
  res->status_code = status;
  res->content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
  res->content_length = len;
  res->data = const_cast<char*>(static_cast<const char*>(data));
  res->data_length = len;
  HTTPSRV_cgi_write(res);
}

static void SendJson(HTTPSRV_CGI_RES_STRUCT* res, int status, const char* json) {
  SendResponse(res, status, "application/json", json, strlen(json));
}

static void SendHtml(HTTPSRV_CGI_RES_STRUCT* res, const void* data, size_t len) {
  res->status_code = HTTPSRV_CODE_OK;
  res->content_type = HTTPSRV_CONTENT_TYPE_HTML;
  res->content_length = len;
  res->data = const_cast<char*>(static_cast<const char*>(data));
  res->data_length = len;
  HTTPSRV_cgi_write(res);
}

// ── Directory listing → JSON ─────────────────────────────────────────────
static std::string ApiLsJson(const char* path) {
  lfs_dir_t dir;
  std::string json;
  json.reserve(4096);

  const char* dir_path = (*path == '\0' || strcmp(path, "/") == 0) ? "/" : path;
  if (lfs_dir_open(coralmicro::LfsUser(), &dir, dir_path) < 0) {
    return "[]";
  }

  json = "[";
  struct lfs_info info;
  bool first = true;
  while (lfs_dir_read(coralmicro::LfsUser(), &dir, &info) > 0) {
    if (info.name[0] == '.' &&
        (info.name[1] == '\0' ||
         (info.name[1] == '.' && info.name[2] == '\0')))
      continue;

    if (!first) json += ",";
    first = false;

    // Read write-time attribute
    uint32_t wt = 0;
    std::string full_path = std::string(dir_path);
    if (full_path.back() != '/') full_path += "/";
    full_path += info.name;
    lfs_getattr(coralmicro::LfsUser(), full_path.c_str(), 0x54, &wt, sizeof(wt));

    json += "{\"name\":\"";
    // Escape name
    for (const char* c = info.name; *c; ++c) {
      if (*c == '"' || *c == '\\') json += '\\';
      json += *c;
    }
    json += "\",\"type\":\"";
    json += (info.type == LFS_TYPE_DIR) ? "dir" : "file";
    json += "\",\"size\":";
    json += std::to_string(info.size);
    json += ",\"wt\":";
    json += std::to_string(wt);
    json += "}";
  }
  lfs_dir_close(coralmicro::LfsUser(), &dir);
  json += "]";
  return json;
}

// ── Read file into buffer ────────────────────────────────────────────────
static std::vector<uint8_t> ReadUserFile(const char* path) {
  struct lfs_info info;
  if (lfs_stat(coralmicro::LfsUser(), path, &info) < 0 ||
      info.type != LFS_TYPE_REG || info.size > 8 * 1024 * 1024)
    return {};
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, path, LFS_O_RDONLY) < 0)
    return {};
  std::vector<uint8_t> data(info.size);
  lfs_ssize_t n = lfs_file_read(coralmicro::LfsUser(), &f, data.data(), data.size());
  lfs_file_close(coralmicro::LfsUser(), &f);
  if (n != static_cast<lfs_ssize_t>(info.size)) return {};
  return data;
}

// ── Load browser.html into cache ─────────────────────────────────────────
static void LoadBrowserHtmlCache(void) {
  auto data = ReadUserFile("/.sys/browser.html");
  if (!data.empty()) {
    g_browser_html_cache = std::move(data);
    printf("[httpsrv] browser.html cached from LfsUser (%u bytes)\r\n",
           (unsigned)g_browser_html_cache.size());
    return;
  }
  // Deploy embedded version
  lfs_mkdir(coralmicro::LfsUser(), "/.sys");
  lfs_file_t f;
  if (lfs_file_open(coralmicro::LfsUser(), &f, "/.sys/browser.html",
                    LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) >= 0) {
    lfs_file_write(coralmicro::LfsUser(), &f, browser_html_data, browser_html_data_len);
    lfs_file_close(coralmicro::LfsUser(), &f);
  }
  g_browser_html_cache.assign(browser_html_data, browser_html_data + browser_html_data_len);
  printf("[httpsrv] browser.html cached from embedded (%u bytes)\r\n",
         (unsigned)g_browser_html_cache.size());
}

// ══════════════════════════════════════════════════════════════════════════
// CGI Handlers
// ══════════════════════════════════════════════════════════════════════════

// GET / or /index.html → browser.html
static int CgiIndex(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  if (!g_browser_html_cache.empty()) {
    SendHtml(&res, g_browser_html_cache.data(), g_browser_html_cache.size());
  } else {
    SendHtml(&res, browser_html_data, browser_html_data_len);
  }
  return 0;
}

// GET /api/ls[/path] → JSON directory listing
static int CgiLs(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  // Extract path from query string or use root
  const char* path = "/";
  if (req->query_string && strlen(req->query_string) > 0) {
    // query_string format: path=/some/path
    const char* p = strstr(req->query_string, "path=");
    if (p) path = p + 5;
  }
  
  std::string json = ApiLsJson(path);
  SendJson(&res, HTTPSRV_CODE_OK, json.c_str());
  return 0;
}

// GET /api/raw?path=/file → raw file content
static int CgiRaw(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  const char* path = nullptr;
  if (req->query_string) {
    const char* p = strstr(req->query_string, "path=");
    if (p) path = p + 5;
  }
  
  if (!path || !ValidPath(path)) {
    SendJson(&res, HTTPSRV_CODE_BAD_REQ, "{\"ok\":false,\"error\":\"invalid path\"}");
    return 0;
  }
  
  auto data = ReadUserFile(path);
  if (data.empty()) {
    SendJson(&res, HTTPSRV_CODE_NOT_FOUND, "{\"ok\":false,\"error\":\"not found\"}");
    return 0;
  }
  
  res.status_code = HTTPSRV_CODE_OK;
  res.content_type = HTTPSRV_CONTENT_TYPE_OCTETSTREAM;
  res.content_length = data.size();
  res.data = reinterpret_cast<char*>(data.data());
  res.data_length = data.size();
  HTTPSRV_cgi_write(&res);
  return 0;
}

// POST /api/write?path=/file [body=file content]
static int CgiWrite(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  if (sentai_usb_drive_get()) {
    SendJson(&res, HTTPSRV_CODE_FORBIDDEN,
             "{\"ok\":false,\"error\":\"USB drive active\"}");
    return 0;
  }
  
  const char* path = nullptr;
  if (req->query_string) {
    const char* p = strstr(req->query_string, "path=");
    if (p) path = p + 5;
  }
  
  if (!path || !ValidPath(path)) {
    SendJson(&res, HTTPSRV_CODE_BAD_REQ, "{\"ok\":false,\"error\":\"invalid path\"}");
    return 0;
  }
  
  // Read POST body
  std::vector<uint8_t> body;
  if (req->content_length > 0 && req->content_length < 8 * 1024 * 1024) {
    body.resize(req->content_length);
    uint32_t total = 0;
    while (total < req->content_length) {
      uint32_t n = HTTPSRV_cgi_read(req->ses_handle,
                                    reinterpret_cast<char*>(body.data() + total),
                                    req->content_length - total);
      if (n == 0) break;
      total += n;
    }
    body.resize(total);
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
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"open failed (%d)\"}", rc);
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR, buf);
    return 0;
  }
  
  lfs_ssize_t written = lfs_file_write(coralmicro::LfsUser(), &f, body.data(), body.size());
  lfs_file_close(coralmicro::LfsUser(), &f);
  
  if (written == static_cast<lfs_ssize_t>(body.size())) {
    // Record write time
    uint32_t uptime_s = xTaskGetTickCount() / configTICK_RATE_HZ;
    lfs_setattr(coralmicro::LfsUser(), path, 0x54, &uptime_s, sizeof(uptime_s));
    
    // Refresh cache if browser.html
    if (strcmp(path, "/.sys/browser.html") == 0) {
      g_browser_html_cache = std::move(body);
      printf("[httpsrv] browser.html cache refreshed\r\n");
    }
    
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"size\":%ld}", (long)written);
    SendJson(&res, HTTPSRV_CODE_OK, buf);
  } else {
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR, "{\"ok\":false,\"error\":\"write incomplete\"}");
  }
  return 0;
}

// POST /api/mkdir?path=/dir
static int CgiMkdir(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  if (sentai_usb_drive_get()) {
    SendJson(&res, HTTPSRV_CODE_FORBIDDEN,
             "{\"ok\":false,\"error\":\"USB drive active\"}");
    return 0;
  }
  
  const char* path = nullptr;
  if (req->query_string) {
    const char* p = strstr(req->query_string, "path=");
    if (p) path = p + 5;
  }
  
  if (!path || !ValidPath(path)) {
    SendJson(&res, HTTPSRV_CODE_BAD_REQ, "{\"ok\":false,\"error\":\"invalid path\"}");
    return 0;
  }
  
  // Create parent directories
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
    SendJson(&res, HTTPSRV_CODE_OK, "{\"ok\":true}");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"mkdir failed (%d)\"}", rc);
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR, buf);
  }
  return 0;
}

// POST /api/rm?path=/file
static int CgiRm(HTTPSRV_CGI_REQ_STRUCT* req) {
  HTTPSRV_CGI_RES_STRUCT res = {0};
  res.ses_handle = req->ses_handle;
  
  if (sentai_usb_drive_get()) {
    SendJson(&res, HTTPSRV_CODE_FORBIDDEN,
             "{\"ok\":false,\"error\":\"USB drive active\"}");
    return 0;
  }
  
  const char* path = nullptr;
  if (req->query_string) {
    const char* p = strstr(req->query_string, "path=");
    if (p) path = p + 5;
  }
  
  if (!path || !ValidPath(path)) {
    SendJson(&res, HTTPSRV_CODE_BAD_REQ, "{\"ok\":false,\"error\":\"invalid path\"}");
    return 0;
  }
  
  int rc = lfs_remove(coralmicro::LfsUser(), path);
  if (rc >= 0) {
    SendJson(&res, HTTPSRV_CODE_OK, "{\"ok\":true}");
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"ok\":false,\"error\":\"remove failed (%d), dir not empty?\"}", rc);
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR, buf);
  }
  return 0;
}

// CGI table
static const HTTPSRV_CGI_LINK_STRUCT g_cgi_table[] = {
    {const_cast<char*>("index"),    CgiIndex},
    {const_cast<char*>("api/ls"),   CgiLs},
    {const_cast<char*>("api/raw"),  CgiRaw},
    {const_cast<char*>("api/write"), CgiWrite},
    {const_cast<char*>("api/mkdir"), CgiMkdir},
    {const_cast<char*>("api/rm"),   CgiRm},
    {nullptr, nullptr}  // End marker
};

}  // namespace

extern "C" void sentai_httpd_start(void) {
  static bool started = false;
  if (started) return;
  started = true;
  
  LoadBrowserHtmlCache();
  
  // Configure httpsrv
  static HTTPSRV_PARAM_STRUCT params;
  memset(&params, 0, sizeof(params));
  
  // Bind to any address, port 80
  struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(&params.address);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(80);
  addr->sin_addr.s_addr = INADDR_ANY;
  
  params.root_dir = nullptr;      // No filesystem root - using CGI only
  params.index_page = "index";    // CGI name for /
  params.max_uri = 256;
  params.max_ses = 2;
  params.task_prio = 4;
  params.cgi_lnk_tbl = g_cgi_table;
  params.ssi_lnk_tbl = nullptr;
  params.auth_table = nullptr;
  
  uint32_t handle = HTTPSRV_init(&params);
  if (handle == 0) {
    printf("[httpsrv] ERROR: Failed to start HTTP server\r\n");
    return;
  }
  
  printf("[httpsrv] File browser at http://10.0.0.1/\r\n");
}
