// C glue for the 'sentai.sdcard' MicroPython module.
// Bridges bindings/modsentai_sdcard.c (C) to coralmicro::SDCard (C++).
// Block-level diagnostics only (init/info/probe/hexdump); file access goes
// through sentai.fs (FileX on the same card, libs/base/fx_user_fs.cc).
//
// Hardware (schematic page 16 "Interface" + page 6 "RT1176 PART2"):
// microSD socket on USDHC1, supply gated by SD_PWREN_B (GPIO_AD_12),
// pad rail selected by SD1_VSELECT (GPIO_AD_16).  Pin/rail handling
// lives in libs/sdcard/sdcard.cc; this file only exposes a thin,
// scalar-returning API so no block data crosses into MicroPython
// (see [[no-heavy-data-through-mp]]).

#include "libs/sdcard/sdcard.h"
#include "sentai_error.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_common.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// Single 512-byte sector buffer.  USDHC moves data by ADMA, so the
// buffer must be DMA-reachable and 32-byte aligned; NonCacheable keeps
// it out of the D-cache maintenance path entirely.
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t s_sector[512], 32);

uint32_t le32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// Copy an 11-byte FAT volume label, trimming trailing spaces.
void copy_label(const uint8_t* src, char* dst, size_t dst_len) {
  size_t n = 11;
  while (n > 0 && (src[n - 1] == ' ' || src[n - 1] == 0)) --n;
  if (n >= dst_len) n = dst_len - 1;
  for (size_t i = 0; i < n; ++i) {
    dst[i] = (src[i] >= 0x20 && src[i] < 0x7F) ? (char)src[i] : '?';
  }
  dst[n] = 0;
}

// Classify a sector that is expected to be a volume boot record.
// Returns the number of chars written (0 = not a recognised VBR).
int describe_vbr(const uint8_t* s, char* out, size_t out_len) {
  if (!(s[0] == 0xEB || s[0] == 0xE9)) return 0;
  char label[12] = {0};
  if (memcmp(s + 3, "EXFAT   ", 8) == 0) {
    return snprintf(out, out_len, "exFAT bytes/sec=%u sec/clus=%u",
                    1u << s[0x6C], 1u << s[0x6D]);
  }
  if (memcmp(s + 0x52, "FAT32", 5) == 0) {
    copy_label(s + 0x47, label, sizeof(label));
    return snprintf(out, out_len,
                    "FAT32 bytes/sec=%u sec/clus=%u total_sec=%u label='%s'",
                    le16(s + 0x0B), s[0x0D], (unsigned)le32(s + 0x20), label);
  }
  if (memcmp(s + 0x36, "FAT", 3) == 0) {
    copy_label(s + 0x2B, label, sizeof(label));
    uint32_t total = le16(s + 0x13) ? le16(s + 0x13) : le32(s + 0x20);
    // s+0x36 is "FAT12   " / "FAT16   " as written by the formatter.
    return snprintf(out, out_len,
                    "%.5s bytes/sec=%u sec/clus=%u total_sec=%u label='%s'",
                    (const char*)(s + 0x36), le16(s + 0x0B), s[0x0D],
                    (unsigned)total, label);
  }
  return 0;
}

}  // namespace

extern "C" {

// deadline_ms <= 0 -> default.  Returns 0 ok, -1 failed, -2 timeout
// (SERR_LFX_SD_INIT_TIMEOUT), -3 a previous init is still running.
int sentai_sdcard_init(int deadline_ms) {
  uint32_t dl = deadline_ms > 0 ? (uint32_t)deadline_ms : coralmicro::SDCard::kInitDeadlineMs;
  int r = coralmicro::SDCard::InitBounded(dl);
  if (r == 1) return 0;
  if (r == -1) { SERR_LOG(SERR_LFX_SD_INIT_TIMEOUT, dl); return -2; }
  if (r == -2) return -3;
  SERR_LOG(SERR_LFX_SD_INIT_FAIL, 0);
  return -1;
}

void sentai_sdcard_deinit(void) { coralmicro::SDCard::Deinit(); }

int sentai_sdcard_bus_info(uint32_t* clock_hz, int* timing_mode, int* bus_width) {
  coralmicro::SDCardInfo info;
  if (!coralmicro::SDCard::GetCardInfo(&info)) return -1;
  *clock_hz = info.bus_clock_hz;
  *timing_mode = info.timing_mode;
  *bus_width = info.bus_width;
  return 0;
}

int sentai_sdcard_info(uint32_t* blocks, uint32_t* block_size,
                       uint32_t* capacity_mb, int* high_capacity, char* name,
                       size_t name_len) {
  coralmicro::SDCardInfo info;
  if (!coralmicro::SDCard::GetCardInfo(&info)) return -1;
  *blocks = info.block_count;
  *block_size = info.block_size;
  *capacity_mb = (uint32_t)(info.capacity_bytes / (1024ULL * 1024ULL));
  *high_capacity = info.is_high_capacity ? 1 : 0;
  snprintf(name, name_len, "%s", info.product_name);
  return 0;
}

// Read-only identification of what is on the card: raw VBR (unpartitioned
// "superfloppy"), MBR + first used partition's VBR, or GPT.  Writes a
// human-readable one-liner into `out`.  Returns 0 on success, -1 on read
// failure, -2 if the card is not initialised.
int sentai_sdcard_probe(char* out, size_t out_len) {
  if (!coralmicro::SDCard::IsCardInserted()) return -2;
  if (!coralmicro::SDCard::ReadBlocks(0, s_sector, 1)) return -1;

  const uint8_t* s = s_sector;
  const bool sig = (s[510] == 0x55 && s[511] == 0xAA);

  char vbr[128];
  if (describe_vbr(s, vbr, sizeof(vbr)) > 0) {
    snprintf(out, out_len, "LBA0=VBR (no partition table) %s", vbr);
    return 0;
  }
  if (!sig) {
    snprintf(out, out_len,
             "LBA0 has no 0x55AA signature (blank/unknown) first bytes "
             "%02X %02X %02X %02X",
             s[0], s[1], s[2], s[3]);
    return 0;
  }

  // MBR.  Walk the four primary entries, report the first non-empty one.
  for (int i = 0; i < 4; ++i) {
    const uint8_t* e = s + 0x1BE + 16 * i;
    const uint8_t type = e[4];
    if (type == 0) continue;
    const uint32_t start = le32(e + 8);
    const uint32_t count = le32(e + 12);
    if (type == 0xEE) {
      snprintf(out, out_len, "LBA0=GPT protective MBR (part%d start=%u)", i,
               (unsigned)start);
      return 0;
    }
    if (!coralmicro::SDCard::ReadBlocks(start, s_sector, 1)) {
      snprintf(out, out_len,
               "LBA0=MBR part%d type=0x%02X start=%u sectors=%u (VBR read "
               "FAILED)",
               i, type, (unsigned)start, (unsigned)count);
      return -1;
    }
    if (describe_vbr(s_sector, vbr, sizeof(vbr)) == 0) {
      snprintf(vbr, sizeof(vbr), "unrecognised VBR");
    }
    snprintf(out, out_len, "LBA0=MBR part%d type=0x%02X start=%u sectors=%u %s",
             i, type, (unsigned)start, (unsigned)count, vbr);
    return 0;
  }
  snprintf(out, out_len, "LBA0=MBR with empty partition table");
  return 0;
}

// Diagnostic: print one sector as hex + ASCII (16 bytes/line) to the
// console.  Read-only.
int sentai_sdcard_hexdump(uint32_t lba) {
  if (!coralmicro::SDCard::IsCardInserted()) return -2;
  if (!coralmicro::SDCard::ReadBlocks(lba, s_sector, 1)) return -1;
  for (int row = 0; row < 512; row += 16) {
    printf("%08X  ", (unsigned)(lba * 512u + row));
    for (int i = 0; i < 16; ++i) printf("%02X ", s_sector[row + i]);
    printf(" |");
    for (int i = 0; i < 16; ++i) {
      uint8_t c = s_sector[row + i];
      putchar((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    printf("|\r\n");
  }
  return 0;
}

// Filesystem access to the card is NOT here any more: the SentAI user FS
// (sentai.fs.*) is FileX on this card via libs/base/fx_user_fs.cc —
// one FAT stack per medium.  This module stays block-level diagnostics.

}  // extern "C"
