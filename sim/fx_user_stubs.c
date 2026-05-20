// fx_user_stubs.c — SIM stubs for FxUser FAT API.
//
// On the board, FxUser sits on top of LevelX+FileX writing to the
// NAND user partition.  In SIM (x86 Linux), there's no FxUser — POSIX
// fopen() lives in host stdio.  Two callers (sentai_aruco_detect_pgm_file
// + sentai_whycon_test_pgm) reference FxUserSize / FxUserReadFile as
// the HW fallback path.  These stubs return "not found" so the SIM
// always takes the POSIX path; the symbols resolve at link time.

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>  // ssize_t

ssize_t FxUserSize(const char* path) {
    (void)path;
    return -1;
}

size_t FxUserReadFile(const char* path, uint8_t* buf, size_t size) {
    (void)path;
    (void)buf;
    (void)size;
    return 0;
}
