/* modsentai_sim_journal.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* =========================================================================
 * sentai.sim — SIM-only diagnostic utilities
 * =========================================================================
 *
 * journal_* — structured append-mode log file the host can read post-mortem.
 * Use case: complex integration tests (e.g. s128 L4.1Baseline) record
 * every action + servo.status snapshot so a crash mid-mission leaves an
 * auditable trail on disk.  The host parses the journal at end-of-run
 * (or after a crash) to localize WHERE and WHY things broke.
 *
 * SYSTEM MODEL (per agent/embeded.md §A "system model first")
 *
 * Fault model:
 *   F1 journal_open(unresolvable path)  -> return -1, errors++
 *   F2 journal_open(fopen failure)      -> return -1, errors++
 *   F3 journal_write while not open     -> silent noop, return -1
 *                                          (so optional logging never
 *                                          disrupts test flow)
 *   F4 fprintf truncation / fflush err  -> swallowed locally, errors++
 *
 * Execution model:
 *   - MP-task context only.  Single global FILE*.  No locking — REPL is
 *     single-threaded for command dispatch.  Camera_bridge thread does
 *     NOT touch this file.
 *   - Each journal_write does fwrite + fflush so a SIGKILL leaves the
 *     file in a parseable state up to the last completed write.
 *   - Each line is monotonic-ms-timestamped:
 *         <t_ms> <label> <repr(value)>\n
 *     value is omitted (rendered as "-") when call site passes None.
 *
 * Recovery model:
 *   - All errors are local: caller continues even on journal write fail.
 *   - journal_close() flushes + closes; subsequent writes are noops.
 *
 * Memory: 1 FILE*, 1 char[SIM_FS_MAXPATH+1] path, 2 uint32_t counters.
 *         Zero heap.
 *
 * Output format example:
 *     # sentai.sim journal opened path=/.../journal.txt truncate=1
 *     12345 boot -
 *     12356 servo_arm {'rc': 0, 'armed': 1, ...}
 *     12378 servo_takeoff {'flight': 1, ...}
 *     ...
 * Host-side parser: split each line by 2 whitespace chunks
 *     -> (t_ms, label, rest).  If rest starts with '{' or '[' run
 *     ast.literal_eval(rest) to recover the value.
 * ========================================================================= */
static FILE*    g_sim_jfp;
static char     g_sim_jpath[SIM_FS_MAXPATH + 1];
static uint32_t g_sim_jlines;
static uint32_t g_sim_jerrors;

static uint64_t sim_journal_t_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* journal_open(path[, truncate=True]) -> 0 ok, -1 fail */
static mp_obj_t sentai_sim_journal_open(size_t n_args, const mp_obj_t *args) {
    if (g_sim_jfp) { fclose(g_sim_jfp); g_sim_jfp = NULL; g_sim_jpath[0] = '\0'; }
    const char *bpath = mp_obj_str_get_str(args[0]);
    int truncate = (n_args >= 2) ? mp_obj_is_true(args[1]) : 1;
    char fp[SIM_FS_MAXPATH + 1];
    if (sim_fs_resolve(bpath, fp, sizeof(fp)) != 0) {
        g_sim_jerrors++;
        return mp_obj_new_int(-1);
    }
    FILE *f = fopen(fp, truncate ? "w" : "a");
    if (!f) { g_sim_jerrors++; return mp_obj_new_int(-1); }
    g_sim_jfp = f;
    g_sim_jlines = 0;
    strncpy(g_sim_jpath, fp, sizeof(g_sim_jpath) - 1);
    g_sim_jpath[sizeof(g_sim_jpath) - 1] = '\0';
    /* Header so post-mortem tooling recognises the format. */
    fprintf(g_sim_jfp,
            "# sentai.sim journal opened path=%s truncate=%d t_ms=%llu\n",
            fp, truncate, (unsigned long long)sim_journal_t_ms());
    fflush(g_sim_jfp);
    g_sim_jlines = 1;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_sim_journal_open_obj,
                                            1, 2, sentai_sim_journal_open);

/* journal_close() -> 0 */
static mp_obj_t sentai_sim_journal_close(void) {
    if (g_sim_jfp) {
        fprintf(g_sim_jfp, "# closed t_ms=%llu lines=%u errors=%u\n",
                (unsigned long long)sim_journal_t_ms(),
                g_sim_jlines, g_sim_jerrors);
        fclose(g_sim_jfp);
        g_sim_jfp = NULL;
    }
    g_sim_jpath[0] = '\0';
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_sim_journal_close_obj,
                                 sentai_sim_journal_close);

/* journal_write(label[, value=None]) -> 0 ok, -1 not open
 * Line: <t_ms> <label> <repr(value)|'-'>\n */
static mp_obj_t sentai_sim_journal_write(size_t n_args, const mp_obj_t *args) {
    if (!g_sim_jfp) return mp_obj_new_int(-1);
    const char *label = mp_obj_str_get_str(args[0]);
    uint64_t t = sim_journal_t_ms();

    if (n_args >= 2 && args[1] != mp_const_none) {
        /* Use the standard MP idiom for repr -> contiguous bytes:
         * mp_print_helper into a vstr-backed mp_print_t.  vstr_add_strn
         * matches the (void*, const char*, size_t) signature of
         * mp_print_strn_t. */
        vstr_t v;
        vstr_init(&v, 64);
        mp_print_t print = {
            .data       = &v,
            .print_strn = (mp_print_strn_t)vstr_add_strn,
        };
        mp_obj_print_helper(&print, args[1], PRINT_REPR);
        if (fprintf(g_sim_jfp, "%llu %s %.*s\n",
                    (unsigned long long)t, label,
                    (int)v.len, v.buf) <= 0) {
            g_sim_jerrors++;
        }
        vstr_clear(&v);
    } else {
        if (fprintf(g_sim_jfp, "%llu %s -\n",
                    (unsigned long long)t, label) <= 0) {
            g_sim_jerrors++;
        }
    }
    fflush(g_sim_jfp);
    g_sim_jlines++;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_sim_journal_write_obj,
                                            1, 2, sentai_sim_journal_write);

/* journal_status() -> dict {open, path, lines, errors} */
static mp_obj_t sentai_sim_journal_status(void) {
    mp_obj_dict_t *d = MP_OBJ_TO_PTR(mp_obj_new_dict(4));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_open),
                      mp_obj_new_bool(g_sim_jfp != NULL));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_path),
                      mp_obj_new_str(g_sim_jpath, strlen(g_sim_jpath)));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_lines),
                      mp_obj_new_int(g_sim_jlines));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_errors),
                      mp_obj_new_int(g_sim_jerrors));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_sim_journal_status_obj,
                                 sentai_sim_journal_status);

static const mp_rom_map_elem_t sentai_sim_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_sim) },
    { MP_ROM_QSTR(MP_QSTR_journal_open),   MP_ROM_PTR(&sentai_sim_journal_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_journal_close),  MP_ROM_PTR(&sentai_sim_journal_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_journal_write),  MP_ROM_PTR(&sentai_sim_journal_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_journal_status), MP_ROM_PTR(&sentai_sim_journal_status_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sim_globals, sentai_sim_globals_table);
static const mp_obj_module_t sentai_sim_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_sim_globals,
};
