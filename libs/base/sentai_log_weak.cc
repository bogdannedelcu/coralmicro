// Weak fallback for sentai_logf() so libs/base and libs/camera link into
// vendor examples (blink_led, hello_world, ...) that do not include the
// SentAI runtime.  The runtime's strong definition (sentai_log.cc) wins.
#include <cstdarg>
#include <cstdio>
extern "C" __attribute__((weak)) int sentai_logf(const char* tag, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  if (tag) printf("[%s] ", tag);
  int n = vprintf(fmt, ap);
  va_end(ap);
  return n;
}
