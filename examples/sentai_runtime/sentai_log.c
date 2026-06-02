// sentai_log.c -- small, task-friendly logging shim.

#include "sentai_log.h"

#include "sentai_fr.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

int sentai_log_write(const char* data, int len) {
  if (!data || len <= 0) return -1;
  int off = 0;
  int rc_last = 0;
  while (off < len) {
    int n = len - off;
    if (n > SENTAI_FR_DEBUG_TEXT_LEN) n = SENTAI_FR_DEBUG_TEXT_LEN;
    rc_last = sentai_fr_push_debug(data + off, n);
    off += n;
  }
  return rc_last;
}

int sentai_logf(const char* tag, const char* fmt, ...) {
  if (!fmt) return -1;

  char buf[SENTAI_FR_DEBUG_TEXT_LEN];
  int off = 0;
  if (tag && tag[0]) {
    off = snprintf(buf, sizeof(buf), "[%s] ", tag);
    if (off < 0) return -2;
    if (off >= (int)sizeof(buf)) off = (int)sizeof(buf) - 1;
  }

  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf + off, sizeof(buf) - (size_t)off, fmt, ap);
  va_end(ap);
  if (n < 0) return -3;

  int len = off + n;
  if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
  if (len <= 0) return 0;

  if (buf[len - 1] != '\n') {
    if (len < (int)sizeof(buf)) {
      buf[len++] = '\n';
    } else {
      buf[(int)sizeof(buf) - 1] = '\n';
    }
  }
  return sentai_fr_push_debug(buf, len);
}
