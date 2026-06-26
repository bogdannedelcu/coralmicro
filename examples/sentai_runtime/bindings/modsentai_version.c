// ============== sentai top-level version / verbosity ==============
// This file is #include'd from modsentai.c / emulator module wrappers.

#ifndef SENTAI_VERSION_PREFIX
#define SENTAI_VERSION_PREFIX "SentAI v1.0"
#endif

#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)
#define SENTAI_VERSION_STR \
    SENTAI_VERSION_PREFIX " build " STRINGIFY(BUILD_VERSION) " (" BUILD_TIMESTAMP ")"

// sentai.version() -> str
static mp_obj_t mod_sentai_version(void) {
    return mp_obj_new_str(SENTAI_VERSION_STR, strlen(SENTAI_VERSION_STR));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_version_obj, mod_sentai_version);

// sentai.verbose([flag]) -> int
//   With no args: returns current verbose flag (1 = prints on, 0 = silent).
//   With an arg : sets the flag and returns the new value.
// Always returns the PREVIOUS value so callers can save-and-restore:
//   prev = sentai.verbose(0); ...; sentai.verbose(prev)
extern int  sentai_verbose_get(void);
extern void sentai_verbose_set(int v);
static mp_obj_t mod_sentai_verbose(size_t n_args, const mp_obj_t *args) {
    int prev = sentai_verbose_get();
    if (n_args >= 1) {
        sentai_verbose_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_verbose_obj,
                                            0, 1, mod_sentai_verbose);
