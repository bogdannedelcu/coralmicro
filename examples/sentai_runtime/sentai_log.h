// sentai_log.h -- best-effort runtime debug logging through sentai.fr.
//
// Use this from FreeRTOS tasks instead of printf/fprintf.  Producers only
// enqueue a bounded debug chunk; the FR worker is the single file writer.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int sentai_log_write(const char* data, int len);
int sentai_logf(const char* tag, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

