// ============== sentai top-level helpers ==============
// This file is #include'd from modsentai.c / modsentai_sim.c.

extern int sentai_usb_drive_get(void);
extern int sentai_console_set_target(int target);
extern int sentai_console_get_target(void);
extern void sentai_link_set_debug(int level);
extern int sentai_help_read(char* buf, int max_size);
extern int g_audio_debug;

static int help_find_section(const char* buf, int len, const char* section,
                             const char** out_start, const char** out_end) {
    char marker[32];
    int mlen = snprintf(marker, sizeof(marker), "[%s]", section);
    const char* buf_end = buf + len;

    for (const char* p = buf; p < buf_end - mlen; p++) {
        if ((p == buf || *(p - 1) == '\n') && memcmp(p, marker, mlen) == 0) {
            const char* start = p + mlen;
            while (start < buf_end && *start != '\n') start++;
            if (start < buf_end) start++;
            const char* end = start;
            while (end < buf_end) {
                if (*end == '[' && (end == start || *(end - 1) == '\n')) break;
                end++;
            }
            *out_start = start;
            *out_end = end;
            return 1;
        }
    }
    return 0;
}

static void help_print(const char* text, int len) {
    const char* p = text;
    const char* end = text + len;
    char line[120];
    while (p < end) {
        const char* nl = p;
        while (nl < end && *nl != '\n') nl++;
        int llen = nl - p;
        if (llen > (int)sizeof(line) - 3) llen = (int)sizeof(line) - 3;
        memcpy(line, p, llen);
        line[llen] = '\r';
        line[llen + 1] = '\n';
        line[llen + 2] = '\0';
        mp_print_str(MP_PYTHON_PRINTER, line);
        p = (nl < end) ? nl + 1 : end;
    }
}

// sentai.help([topic])
static mp_obj_t mod_sentai_help(size_t n_args, const mp_obj_t* args) {
    const char* topic = (n_args > 0) ? mp_obj_str_get_str(args[0]) : NULL;

    #define HELP_BUF_SIZE 73728
    char* hbuf = (char*)malloc(HELP_BUF_SIZE);
    if (!hbuf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("help buf alloc"));
    }
    int n = sentai_help_read(hbuf, HELP_BUF_SIZE);

    if (n <= 0) {
        free(hbuf);
        mp_print_str(MP_PYTHON_PRINTER, "Help file not found on flash.\r\n");
        return mp_const_none;
    }

    if (topic == NULL) {
        const char *start, *end;
        if (help_find_section(hbuf, n, "overview", &start, &end)) {
            help_print(start, end - start);
        } else {
            help_print(hbuf, n);
        }
    } else if (strcmp(topic, "all") == 0) {
        help_print(hbuf, n);
    } else {
        const char *start, *end;
        if (help_find_section(hbuf, n, topic, &start, &end)) {
            help_print(start, end - start);
        } else {
            mp_print_str(MP_PYTHON_PRINTER,
                "Unknown topic. Available: io, rtos, tpu, fs, camera, imu, mic, usb, uart, console, mesh, link, crazy, pipeline, serial, all\r\n");
        }
    }

    free(hbuf);
    #undef HELP_BUF_SIZE
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_help_obj, 0, 1,
                                           mod_sentai_help);

// sentai.console([target]) -> str
static mp_obj_t mod_sentai_console(size_t n_args, const mp_obj_t* args) {
    if (n_args > 0) {
        const char* target = mp_obj_str_get_str(args[0]);
        if (strcmp(target, "usb") == 0) {
            sentai_console_set_target(0);
        } else if (strcmp(target, "uart") == 0) {
            sentai_console_set_target(1);
        } else {
            mp_raise_ValueError(MP_ERROR_TEXT("use 'usb' or 'uart'"));
        }
    }
    int t = sentai_console_get_target();
    return mp_obj_new_str(t == 0 ? "usb" : "uart", t == 0 ? 3 : 4);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_console_obj, 0, 1,
                                           mod_sentai_console);

// sentai.debug(level) -> None
static mp_obj_t mod_sentai_debug(mp_obj_t level_obj) {
    int level = mp_obj_get_int(level_obj);
    g_audio_debug = level;
    sentai_link_set_debug(level);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_debug_obj, mod_sentai_debug);

// sentai.run(path) - Read and execute a .py file from flash.
static mp_obj_t mod_sentai_run(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);

    _fs_check_usb();
    int size = sentai_fs_size(path);
    if (size < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("file not found"));
    }
    byte* buf = m_new(byte, size + 1);
    int n = sentai_fs_read(path, (uint8_t*)buf, size);

    if (n <= 0) {
        m_del(byte, buf, size + 1);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("read error"));
    }
    buf[n] = '\0';

    mp_lexer_t* lex = mp_lexer_new_from_str_len(
        qstr_from_str(path), (const char*)buf, n, size + 1);
    qstr source_name = lex->source_name;
    mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
    mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
    mp_call_function_0(module_fun);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_run_obj, mod_sentai_run);
