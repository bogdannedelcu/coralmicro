// ============== sentai.sdcard — microSD socket (USDHC1) ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Thin REPL surface over libs/sdcard (C++) via sentai_sdcard_glue.cc.
// Block-level diagnostics only (the card's filesystem is sentai.fs, FileX).
// Scalars and short strings only; sector data never crosses into the
// MicroPython heap.  Read-only.

extern int  sentai_sdcard_init(int deadline_ms);
extern void sentai_sdcard_deinit(void);
extern int  sentai_sdcard_info(uint32_t *blocks, uint32_t *block_size,
                               uint32_t *capacity_mb, int *high_capacity,
                               char *name, size_t name_len);
extern int  sentai_sdcard_probe(char *out, size_t out_len);
extern int  sentai_sdcard_hexdump(uint32_t lba);

// sentai.sdcard.init(deadline_ms=5000) -> bool
// Bounded (audit F14): returns False after deadline_ms if SD_Init() does not
// answer (no card / no wiring); dmesg gets SERR_LFX_SD_INIT_TIMEOUT.  A later
// call returns the worker's result once it finishes.
// Powers the card (SD_PWREN_B low), selects 3.3 V pad rail, runs the SD
// init sequence.  Idempotent.
static mp_obj_t mod_sentai_sdcard_init(size_t n_args, const mp_obj_t *args) {
    int dl = (n_args >= 1) ? mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_bool(sentai_sdcard_init(dl) == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_sdcard_init_obj, 0, 1, mod_sentai_sdcard_init);

// sentai.sdcard.init_stage() -> int
// Progress breadcrumb of the last SD init (ConfigureSDHost/SD_Init steps,
// 100 = worker entered, 10 = SD_Init returned); diagnostic for a stuck init.
extern volatile uint32_t g_sd_init_stage;
static mp_obj_t mod_sentai_sdcard_init_stage(void) {
    return mp_obj_new_int_from_uint(g_sd_init_stage);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sdcard_init_stage_obj, mod_sentai_sdcard_init_stage);

// sentai.sdcard.deinit()
static mp_obj_t mod_sentai_sdcard_deinit(void) {
    sentai_sdcard_deinit();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sdcard_deinit_obj, mod_sentai_sdcard_deinit);

// sentai.sdcard.info() -> (blocks, block_size, capacity_mb, high_capacity, name)
static mp_obj_t mod_sentai_sdcard_info(void) {
    uint32_t blocks = 0, bsize = 0, mb = 0;
    int hc = 0;
    char name[8] = {0};
    if (sentai_sdcard_info(&blocks, &bsize, &mb, &hc, name, sizeof(name)) != 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sdcard not initialised"));
    }
    mp_obj_t items[5] = {
        mp_obj_new_int_from_uint(blocks),
        mp_obj_new_int_from_uint(bsize),
        mp_obj_new_int_from_uint(mb),
        mp_obj_new_bool(hc),
        mp_obj_new_str(name, strlen(name)),
    };
    return mp_obj_new_tuple(5, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sdcard_info_obj, mod_sentai_sdcard_info);

// sentai.sdcard.probe() -> str
// Read-only: identifies partition table / filesystem from LBA0 (+VBR).
static mp_obj_t mod_sentai_sdcard_probe(void) {
    char buf[256];
    int rc = sentai_sdcard_probe(buf, sizeof(buf));
    if (rc == -2) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sdcard not initialised"));
    if (rc == -1) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sdcard read failed"));
    return mp_obj_new_str(buf, strlen(buf));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sdcard_probe_obj, mod_sentai_sdcard_probe);

// sentai.sdcard.hexdump(lba=0)
// Prints one 512-byte sector to the console.  Read-only.
static mp_obj_t mod_sentai_sdcard_hexdump(size_t n_args, const mp_obj_t *args) {
    uint32_t lba = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 0;
    int rc = sentai_sdcard_hexdump(lba);
    if (rc == -2) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sdcard not initialised"));
    if (rc == -1) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sdcard read failed"));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_sdcard_hexdump_obj, 0, 1, mod_sentai_sdcard_hexdump);

// ---- module table ----
static const mp_rom_map_elem_t sentai_sdcard_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sdcard) },
    { MP_ROM_QSTR(MP_QSTR_init),     MP_ROM_PTR(&mod_sentai_sdcard_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),   MP_ROM_PTR(&mod_sentai_sdcard_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_stage),     MP_ROM_PTR(&mod_sentai_sdcard_init_stage_obj) },
    { MP_ROM_QSTR(MP_QSTR_info),     MP_ROM_PTR(&mod_sentai_sdcard_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_probe),    MP_ROM_PTR(&mod_sentai_sdcard_probe_obj) },
    { MP_ROM_QSTR(MP_QSTR_hexdump),  MP_ROM_PTR(&mod_sentai_sdcard_hexdump_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sdcard_globals, sentai_sdcard_globals_table);
static const mp_obj_module_t sentai_sdcard_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_sdcard_globals,
};
