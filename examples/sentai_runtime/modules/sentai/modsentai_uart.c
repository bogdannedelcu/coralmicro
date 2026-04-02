// ============== sentai.uart — UART serial I/O ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.uart.open(baudrate=38400) -> bool
static mp_obj_t mod_sentai_uart_serial_open(size_t n_args, const mp_obj_t *args) {
    int baudrate = (n_args > 0) ? mp_obj_get_int(args[0]) : 38400;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    if (baudrate != 115200 && baudrate > 0) {
        sentai_uart_set_baudrate((uint32_t)baudrate);
    }
    return mp_obj_new_bool(sentai_uart_serial_open());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_uart_serial_open_obj, 0, 1, mod_sentai_uart_serial_open);

// sentai.uart.close()
static mp_obj_t mod_sentai_uart_serial_close(void) {
    sentai_uart_restore_baudrate();
    sentai_uart_serial_close();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uart_serial_close_obj, mod_sentai_uart_serial_close);

// sentai.uart.write(data) -> int
static mp_obj_t mod_sentai_uart_serial_write(mp_obj_t data_obj) {
    if (!sentai_uart_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART serial not open"));
    }
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int n = sentai_uart_serial_write((const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(n);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_uart_serial_write_obj, mod_sentai_uart_serial_write);

// sentai.uart.read(max_bytes=256, timeout_ms=1000) -> bytes
static mp_obj_t mod_sentai_uart_serial_read(size_t n_args, const mp_obj_t *args) {
    if (!sentai_uart_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART serial not open"));
    }
    int max_bytes = (n_args > 0) ? mp_obj_get_int(args[0]) : 256;
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (max_bytes <= 0 || max_bytes > 2048) max_bytes = 256;
    uint8_t* buf = m_new(uint8_t, max_bytes);
    int n = sentai_uart_serial_read(buf, max_bytes, timeout_ms);
    if (n <= 0) {
        m_del(uint8_t, buf, max_bytes);
        return mp_obj_new_bytes((const byte*)"", 0);
    }
    mp_obj_t result = mp_obj_new_bytes(buf, n);
    m_del(uint8_t, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_uart_serial_read_obj, 0, 2, mod_sentai_uart_serial_read);

// sentai.uart.available() -> int
static mp_obj_t mod_sentai_uart_serial_available(void) {
    return mp_obj_new_int(sentai_uart_serial_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_uart_serial_available_obj, mod_sentai_uart_serial_available);

// ---- module table ----
static const mp_rom_map_elem_t sentai_uart_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_uart) },
    { MP_ROM_QSTR(MP_QSTR_open),              MP_ROM_PTR(&mod_sentai_uart_serial_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),             MP_ROM_PTR(&mod_sentai_uart_serial_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),             MP_ROM_PTR(&mod_sentai_uart_serial_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),              MP_ROM_PTR(&mod_sentai_uart_serial_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_uart_serial_available_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_uart_globals, sentai_uart_globals_table);
static const mp_obj_module_t sentai_uart_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_uart_globals,
};
