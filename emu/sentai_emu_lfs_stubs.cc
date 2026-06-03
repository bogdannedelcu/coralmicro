// B8 emulator storage support stubs.
//
// The FileX user partition persists its bad-block table through the legacy
// system LittleFS helpers.  The ARM-emulator storage gate does not bring up the
// system partition yet, so these helpers are deliberately inert.  The user
// partition itself still runs through production FileX/LevelX and the emulated
// raw-NAND bridge.

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "libs/base/filesystem.h"

namespace coralmicro {

lfs_t* Lfs() { return nullptr; }
bool LfsInit(bool force_format) {
  (void)force_format;
  return true;
}
bool LfsMakeDirs(const char* path) {
  (void)path;
  return true;
}
std::string LfsDirname(const char* path) {
  if (!path) return std::string();
  const char* slash = nullptr;
  for (const char* p = path; *p; ++p) {
    if (*p == '/') slash = p;
  }
  if (!slash || slash == path) return std::string("/");
  return std::string(path, static_cast<size_t>(slash - path));
}
ssize_t LfsSize(const char* path) {
  (void)path;
  return -1;
}
bool LfsDirExists(const char* path) {
  (void)path;
  return false;
}
bool LfsFileExists(const char* path) {
  (void)path;
  return false;
}
bool LfsReadFile(const char* path, std::vector<uint8_t>* buf) {
  (void)path;
  if (buf) buf->clear();
  return false;
}
bool LfsReadFile(const char* path, std::string* str) {
  (void)path;
  if (str) str->clear();
  return false;
}
size_t LfsReadFile(const char* path, uint8_t* buf, size_t size) {
  (void)path;
  (void)buf;
  (void)size;
  return 0;
}
bool LfsWriteFile(const char* path, const uint8_t* buf, size_t size) {
  (void)path;
  (void)buf;
  (void)size;
  return true;
}
bool LfsWriteFile(const char* path, const std::string& str) {
  return LfsWriteFile(path, reinterpret_cast<const uint8_t*>(str.data()),
                      str.size());
}

lfs_t* LfsUser() { return nullptr; }
bool LfsUserInit(bool force_format) {
  (void)force_format;
  return false;
}
bool LfsUserRemount() { return false; }
bool LfsUserMakeDirs(const char* path) {
  (void)path;
  return false;
}
ssize_t LfsUserSize(const char* path) {
  (void)path;
  return -1;
}
bool LfsUserDirExists(const char* path) {
  (void)path;
  return false;
}
bool LfsUserFileExists(const char* path) {
  (void)path;
  return false;
}
bool LfsUserReadFile(const char* path, std::vector<uint8_t>* buf) {
  (void)path;
  if (buf) buf->clear();
  return false;
}
bool LfsUserReadFile(const char* path, std::string* str) {
  (void)path;
  if (str) str->clear();
  return false;
}
size_t LfsUserReadFile(const char* path, uint8_t* buf, size_t size) {
  (void)path;
  (void)buf;
  (void)size;
  return 0;
}
bool LfsUserWriteFile(const char* path, const uint8_t* buf, size_t size) {
  (void)path;
  (void)buf;
  (void)size;
  return false;
}
bool LfsUserWriteFile(const char* path, const std::string& str) {
  return LfsUserWriteFile(path, reinterpret_cast<const uint8_t*>(str.data()),
                          str.size());
}
bool LfsUserAppendFile(const char* path, const uint8_t* buf, size_t size) {
  (void)path;
  (void)buf;
  (void)size;
  return false;
}
int LfsUserRemove(const char* path) {
  (void)path;
  return -1;
}

}  // namespace coralmicro

extern "C" int sentai_logf(const char* tag, const char* fmt, ...) {
  (void)tag;
  (void)fmt;
  return 0;
}

extern "C" void sentai_repl_activity(void) {}
