// sentai_httpd_v2.cc — HTTP file browser using NXP httpsrv (socket-based)
// More stable than lwIP httpd - uses blocking sockets in dedicated task.
// All API routes operate on LfsUser() (user partition).

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "browser_html_data.h"
#include "libs/base/filesystem.h"
#include "libs/base/fx_user_fs.h"
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
struct LsJsonCtx {
  std::string* json;
  bool first;
};

static int ls_json_cb(const FxDirEntry* e, void* user) {
  LsJsonCtx* ctx = static_cast<LsJsonCtx*>(user);
  if (!ctx->first) ctx->json->append(",");
  ctx->first = false;
  ctx->json->append("{\"name\":\"");
  for (const char* c = e->name; *c; ++c) {
    if (*c == '"' || *c == '\\') ctx->json->push_back('\\');
    ctx->json->push_back(*c);
  }
  ctx->json->append("\",\"type\":\"");
  ctx->json->append(e->is_dir ? "dir" : "file");
  ctx->json->append("\",\"size\":");
  ctx->json->append(std::to_string(e->size));
  ctx->json->append(",\"wt\":");
  ctx->json->append(std::to_string(e->mtime_s));
  ctx->json->append("}");
  return 0;
}

static std::string ApiLsJson(const char* path) {
  std::string json;
  json.reserve(4096);
  const char* dir_path = (*path == '\0' || strcmp(path, "/") == 0) ? "/" : path;
  json = "[";
  LsJsonCtx ctx = { &json, true };
  int rc = FxUserListDir(dir_path, ls_json_cb, &ctx);
  if (rc < 0) return "[]";
  json += "]";
  return json;
}

// ── Read file into buffer ────────────────────────────────────────────────
static std::vector<uint8_t> ReadUserFile(const char* path) {
  ssize_t sz = FxUserSize(path);
  if (sz < 0 || (size_t)sz > 8u * 1024u * 1024u) return {};
  std::vector<uint8_t> data((size_t)sz);
  if (sz == 0) return data;
  size_t n = FxUserReadFile(path, data.data(), data.size());
  if (n != data.size()) return {};
  return data;
}

// ── Load browser.html into cache ─────────────────────────────────────────
static void LoadBrowserHtmlCache(void) {
  auto data = ReadUserFile("/.sys/browser.html");
  if (!data.empty()) {
    g_browser_html_cache = std::move(data);
    printf("[httpsrv] browser.html cached from user FS (%u bytes)\r\n",
           (unsigned)g_browser_html_cache.size());
    return;
  }
  // Deploy embedded version
  FxUserMakeDir("/.sys");
  FxUserWriteFile("/.sys/browser.html",
                  reinterpret_cast<const uint8_t*>(browser_html_data),
                  browser_html_data_len);
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
  
  /* FxUserWriteFile creates parent directories internally. */
  int ok = FxUserWriteFile(path, body.data(), body.size());
  if (ok) {
    /* Refresh cache if browser.html */
    if (strcmp(path, "/.sys/browser.html") == 0) {
      g_browser_html_cache = std::move(body);
      printf("[httpsrv] browser.html cache refreshed\r\n");
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"size\":%lu}",
             (unsigned long)body.size());
    SendJson(&res, HTTPSRV_CODE_OK, buf);
  } else {
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR,
             "{\"ok\":false,\"error\":\"write failed\"}");
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
  
  if (FxUserMakeDirs(path)) {
    SendJson(&res, HTTPSRV_CODE_OK, "{\"ok\":true}");
  } else {
    SendJson(&res, HTTPSRV_CODE_INTERNAL_ERROR,
             "{\"ok\":false,\"error\":\"mkdir failed\"}");
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
  
  int rc = FxUserRemove(path);
  if (rc == 0) {
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
