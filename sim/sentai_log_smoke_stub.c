#include <stdarg.h>
#include <stdio.h>

int sentai_logf(const char* tag, const char* fmt, ...) {
  printf("[%s] ", tag ? tag : "sentai");
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
  fflush(stdout);
  return 0;
}
