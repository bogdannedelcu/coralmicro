// Minimal SDK stubs for B8.8 USB-host emulator probes.
//
// These keep the gate focused on the production USB host task.  They do not
// model cache maintenance or the SDK debug console.

#include <stdarg.h>
#include <stdint.h>

void DCACHE_CleanByRange(uint32_t address, uint32_t size_byte) {
    (void)address;
    (void)size_byte;
}

void DCACHE_CleanInvalidateByRange(uint32_t address, uint32_t size_byte) {
    (void)address;
    (void)size_byte;
}

int DbgConsole_Printf(const char *fmt_s, ...) {
    (void)fmt_s;
    return 0;
}

void sentai_usb_edgetpu_dump_eps(void) {
}

void sentai_usb_edgetpu_begin_enum(uint8_t total) {
    (void)total;
}

void sentai_usb_edgetpu_record_ep(uint8_t addr, uint8_t attrs,
                                  uint16_t maxp, uint8_t interval) {
    (void)addr;
    (void)attrs;
    (void)maxp;
    (void)interval;
}
