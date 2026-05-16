/* modsentai_sim_diag.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== sentai.diag — minimal SDRAM-style ring log ===== */
#define SIM_DMESG_BUFSZ 4096
static char s_dmesg[SIM_DMESG_BUFSZ];
static size_t s_dmesg_len = 0;

void sim_dmesg_append(const char *s) {
    /* Best-effort append; truncate from the front when full. */
    size_t n = strlen(s);
    if (n >= SIM_DMESG_BUFSZ) {
        memcpy(s_dmesg, s + (n - (SIM_DMESG_BUFSZ - 1)),
               SIM_DMESG_BUFSZ - 1);
        s_dmesg_len = SIM_DMESG_BUFSZ - 1;
        s_dmesg[s_dmesg_len] = '\0';
        return;
    }
    if (s_dmesg_len + n >= SIM_DMESG_BUFSZ) {
        size_t drop = s_dmesg_len + n - (SIM_DMESG_BUFSZ - 1);
        memmove(s_dmesg, s_dmesg + drop, s_dmesg_len - drop);
        s_dmesg_len -= drop;
    }
    memcpy(s_dmesg + s_dmesg_len, s, n);
    s_dmesg_len += n;
    s_dmesg[s_dmesg_len] = '\0';
}

static mp_obj_t sentai_diag_dmesg(void) {
    return mp_obj_new_str(s_dmesg, s_dmesg_len);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_diag_dmesg_obj, sentai_diag_dmesg);

static const mp_rom_map_elem_t sentai_diag_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_diag) },
    { MP_ROM_QSTR(MP_QSTR_dmesg),    MP_ROM_PTR(&sentai_diag_dmesg_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_diag_globals, sentai_diag_globals_table);
static const mp_obj_module_t sentai_diag_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_diag_globals,
};

