// ============== sentai.usb — USB mass storage + serial ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.usb.drive(on) -> int (1=enabled, 0=disabled)
// Enable/disable USB mass storage. 1=drive visible to host, 0=ejected.
static mp_obj_t mod_sentai_usb_drive(mp_obj_t on_obj) {
    int on = mp_obj_get_int(on_obj);
    if (on) {
        printf("\r\nUSB drive on — LFS unmounted, host can mount now\r\n");
    }
    int rc = sentai_usb_drive_set(on);
    if (!on) {
        printf("\r\nUSB drive off — LFS remounted\r\n>>> ");
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_usb_drive_obj, mod_sentai_usb_drive);

// ===================== USB Serial functions =====================

// sentai.usb.open() -> bool
// Opens USB CDC ACM port for Python serial I/O.
// Requires REPL on UART. Fails if USB drive is active.
static mp_obj_t mod_sentai_usb_serial_open(void) {
    if (sentai_console_get_target() != 1) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on UART: call sentai.console('uart')"));
    }
    if (sentai_usb_drive_get()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("USB drive active: call sentai.usb.drive(0) first"));
    }
    return mp_obj_new_bool(sentai_usb_serial_open());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_open_obj, mod_sentai_usb_serial_open);

// sentai.usb.close()
// Returns USB CDC ACM to normal console mode.
static mp_obj_t mod_sentai_usb_serial_close(void) {
    sentai_usb_serial_close();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_close_obj, mod_sentai_usb_serial_close);

// sentai.usb.write(data) -> int (bytes written, -1 on error)
// data: str or bytes
static mp_obj_t mod_sentai_usb_serial_write(mp_obj_t data_obj) {
    if (!sentai_usb_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("serial not open"));
    }
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    int n = sentai_usb_serial_write((const uint8_t*)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(n);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_usb_serial_write_obj, mod_sentai_usb_serial_write);

// sentai.usb.read(max_bytes=256, timeout_ms=1000) -> bytes
// Returns up to max_bytes of data received from USB host.
// timeout_ms: -1=block forever, 0=non-blocking, >0=wait up to N ms
static mp_obj_t mod_sentai_usb_serial_read(size_t n_args, const mp_obj_t *args) {
    if (!sentai_usb_serial_is_open()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("serial not open"));
    }
    int max_bytes = (n_args > 0) ? mp_obj_get_int(args[0]) : 256;
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (max_bytes <= 0 || max_bytes > 2048) max_bytes = 256;
    uint8_t* buf = m_new(uint8_t, max_bytes);
    int n = sentai_usb_serial_read(buf, max_bytes, timeout_ms);
    if (n <= 0) {
        m_del(uint8_t, buf, max_bytes);
        return mp_obj_new_bytes((const byte*)"", 0);
    }
    mp_obj_t result = mp_obj_new_bytes(buf, n);
    m_del(uint8_t, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_usb_serial_read_obj, 0, 2, mod_sentai_usb_serial_read);

// sentai.usb.available() -> int (bytes waiting in RX buffer)
static mp_obj_t mod_sentai_usb_serial_available(void) {
    return mp_obj_new_int(sentai_usb_serial_available());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_usb_serial_available_obj, mod_sentai_usb_serial_available);

// ===================== USB IP (CDC-NCM Ethernet) =====================

// sentai.usb.ip(on) -> int (1=active, 0=inactive, -1=not available)
// Query or report status of USB Ethernet (CDC-NCM).
// NCM is always active when USB is connected; this returns the status.
// Pass 1 to confirm network is enabled, 0 is a no-op (NCM stays active).
static mp_obj_t mod_sentai_usb_ip(mp_obj_t on_obj) {
    extern int sentai_usb_ip_set(int on);
    extern int sentai_usb_ip_get(void);
    int on = mp_obj_get_int(on_obj);
    if (on) {
        int rc = sentai_usb_ip_set(1);
        if (rc < 0) {
            mp_raise_msg(&mp_type_OSError,
                MP_ERROR_TEXT("USB IP (CDC-NCM) not available"));
        }
        extern void sentai_httpd_start(void);
        sentai_httpd_start();
        printf("USB Ethernet (CDC-NCM) active at 10.0.0.1 — http://10.0.0.1/\r\n");
        return mp_obj_new_int(rc);
    }
    return mp_obj_new_int(sentai_usb_ip_get());
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_usb_ip_obj, mod_sentai_usb_ip);

// ---- module table ----
static const mp_rom_map_elem_t sentai_usb_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_usb) },
    { MP_ROM_QSTR(MP_QSTR_drive),             MP_ROM_PTR(&mod_sentai_usb_drive_obj) },
    { MP_ROM_QSTR(MP_QSTR_ip),                MP_ROM_PTR(&mod_sentai_usb_ip_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),              MP_ROM_PTR(&mod_sentai_usb_serial_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),             MP_ROM_PTR(&mod_sentai_usb_serial_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),             MP_ROM_PTR(&mod_sentai_usb_serial_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),              MP_ROM_PTR(&mod_sentai_usb_serial_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),         MP_ROM_PTR(&mod_sentai_usb_serial_available_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_usb_globals, sentai_usb_globals_table);
static const mp_obj_module_t sentai_usb_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_usb_globals,
};
