#ifndef _LIBS_BASE_HTTPD_H_
#define _LIBS_BASE_HTTPD_H_

#include <cstring>
#include <functional>
#include <utility>
#include <variant>
#include <vector>

#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/apps/fs.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/apps/httpd.h"

namespace valiant {

class HttpServer {
   public:
    virtual ~HttpServer() = default;

    virtual err_t PostBegin(void* connection, const char* uri,
                            const char* http_request, u16_t http_request_len,
                            int content_len, char* response_uri,
                            u16_t response_uri_len, u8_t* post_auto_wnd) {
        return ERR_ARG;
    }
    virtual err_t PostReceiveData(void* connection, struct pbuf* p) {
        return ERR_ARG;
    };
    virtual void PostFinished(void* connection, char* response_uri,
                              u16_t response_uri_len){};

    virtual void CgiHandler(struct fs_file* file, const char* uri,
                            int iNumParams, char** pcParam, char** pcValue){};

    virtual int FsOpenCustom(struct fs_file* file, const char* name);
    virtual int FsReadCustom(struct fs_file* file, char* buffer, int count);
    virtual void FsCloseCustom(struct fs_file* file);

   public:
    struct StaticBuffer {
        const uint8_t* buffer;
        size_t size;
    };

    using Content = std::variant<std::monostate,        // Not found
                                 std::string,           // Filename
                                 std::vector<uint8_t>,  // Dynamic buffer
                                 StaticBuffer>;         // Static buffer

    using UriHandler = std::function<Content(const char* uri)>;

    void AddUriHandler(UriHandler handler) {
        uri_handlers_.push_back(std::move(handler));
    }

   private:
    std::vector<UriHandler> uri_handlers_;
};

struct FileSystemUriHandler {
    static constexpr char kPrefix[] = "/fs/";
    static constexpr size_t kPrefixLength = sizeof(kPrefix) - 1;

    HttpServer::Content operator()(const char* uri) {
        if (std::strncmp(uri, kPrefix, kPrefixLength) == 0)
            return std::string{uri + kPrefixLength - 1};
        return {};
    }
};

void UseHttpServer(HttpServer* server);

}  // namespace valiant

#endif  // _LIBS_BASE_HTTPD_H_
