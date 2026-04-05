// mp_embed_safe.h — Declaration for mp_embed_exec_str_safe()
#ifndef MP_EMBED_SAFE_H
#define MP_EMBED_SAFE_H

#ifdef __cplusplus
extern "C" {
#endif

// Like mp_embed_exec_str but returns 0 on success, -1 on exception.
// The exception traceback is still printed to stdout.
int mp_embed_exec_str_safe(const char *src);

#ifdef __cplusplus
}
#endif

#endif // MP_EMBED_SAFE_H
