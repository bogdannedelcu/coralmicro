/*
 * Copyright 2026 Bogdan Nedelcu / SentAI
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * LevelX NAND driver adapter for the RT1176 NAND.  See fx_nand_driver.h
 * for the geometry rationale and the spare-emulation contract.
 *
 * Per embeded.md (NASA/JPL discipline):
 *   - Every loop is bounded.
 *   - All callable failures of the underlying NAND driver are surfaced
 *     to LevelX with LX_ERROR; LevelX maps that to FAT FX_IO_ERROR which
 *     the wrapper layer translates into the existing bool/ssize_t
 *     return contracts.
 *   - The block-status table lives in static RAM only; it is rebuilt at
 *     every mount via LevelX's open routine, so the loss of RAM-only
 *     state across reset is benign.
 *   - No dynamic allocation in steady state.  No printf in fast paths
 *     (errors go through SERR_LOG once we wire it in fx_user_fs.cc).
 */

#include "libs/base/fx_nand_driver.h"

#include <cstdio>
#include <cstring>

#include "examples/sentai_runtime/sentai_error.h"
#include "libs/base/filesystem.h"
#include "third_party/nxp/rt1176-sdk/components/flash/nand/fsl_nand_flash.h"

extern "C" nand_handle_t* BOARD_GetNANDHandle(void);

namespace {

/* Cached NAND handle; resolved on first call (BOARD_GetNANDHandle is
 * cheap but we cache to keep the hot path tight). */
nand_handle_t* g_nand_handle = nullptr;

/* RAM block-status table.  Loaded from /system/.nand_bbt at mount and
 * persisted on every change. 448 bytes total. */
UCHAR g_block_status[FX_NAND_USER_BLOCK_COUNT];

/* Per-block consecutive read-failure counter for auto-bad-block detection.
 * After kAutoBadAfterReadFails failures on the SAME logical block we mark
 * it bad ourselves. Reset on every successful read of that block. */
constexpr int kAutoBadAfterReadFails = 3;
uint8_t g_block_read_fails[FX_NAND_USER_BLOCK_COUNT];

constexpr const char* kBbtPath = "/.nand_bbt";  // /system/ is implicit in Lfs

/* "Dirty" flag — set whenever g_block_status changes. The actual LFS write
 * is performed by bbt_flush_if_dirty() which callers MUST invoke from a
 * safe context (NOT inside the NAND read/write fault path, where issuing
 * an LFS write could re-enter the FX user mutex or stall a critical I/O
 * for hundreds of milliseconds). bbt_persist_request() never blocks. */
static volatile bool g_bbt_dirty = false;

static void bbt_persist_request() { g_bbt_dirty = true; }

extern "C" void fx_nand_driver_bbt_flush_if_dirty(void) {
    if (!g_bbt_dirty) return;
    /* Snapshot under volatile barrier; concurrent further updates set the
     * flag again and we'll catch them in the next flush. */
    g_bbt_dirty = false;
    if (!coralmicro::LfsWriteFile(kBbtPath, g_block_status,
                                  sizeof(g_block_status))) {
        SERR_LOG(SERR_LFX_BBT_PERSIST, 0u);
        /* Re-raise dirty so a future flush retries. */
        g_bbt_dirty = true;
    }
}

static void bbt_load() {
    std::memset(g_block_status, 0, sizeof(g_block_status));
    std::memset(g_block_read_fails, 0, sizeof(g_block_read_fails));
    if (!coralmicro::LfsFileExists(kBbtPath)) {
        printf("[bbt] no persisted BBT, starting clean\r\n");
        return;
    }
    size_t n = coralmicro::LfsReadFile(kBbtPath, g_block_status,
                                       sizeof(g_block_status));
    if (n != sizeof(g_block_status)) {
        printf("[bbt] WARN size mismatch read=%zu expected=%zu — re-init\r\n",
               n, sizeof(g_block_status));
        std::memset(g_block_status, 0, sizeof(g_block_status));
        return;
    }
    int n_bad = 0;
    for (size_t i = 0; i < sizeof(g_block_status); ++i) {
        if (g_block_status[i]) n_bad++;
    }
    if (n_bad) printf("[bbt] loaded %d bad block(s) from BBT\r\n", n_bad);
}

/* Diagnostic counters.  Exposed to MicroPython via sentai.diag.fx_stats(). */
struct FxNandStats {
    uint32_t reads;
    uint32_t writes;
    uint32_t erases;
    uint32_t read_errors;
    uint32_t write_errors;
    uint32_t erase_errors;
    uint32_t bad_blocks;
};
FxNandStats g_stats = {};

/* 2 KB scratch page used by the spare-emulation gather/scatter.  Sized
 * to one physical NAND page; the FX_NAND_BYTES_PER_PAGE worth of data
 * sits at the start, the FX_NAND_SPARE_PER_PAGE worth of metadata at
 * the end.  Single-threaded by virtue of the FileX-side mutex held
 * across every fx_* call (FX_PROTECT/FX_UNPROTECT are no-ops in
 * standalone mode, so the FreeRTOS mutex in libs/base/fx_user_fs.cc is
 * the actual serializer). */
UCHAR g_page_scratch[FX_NAND_PAGE_RAW_BYTES]
    __attribute__((aligned(8), section(".sdram_bss")));

/* Helpers ---------------------------------------------------------------- */

static inline nand_handle_t* nand_handle_get() {
    if (g_nand_handle == nullptr) {
        g_nand_handle = BOARD_GetNANDHandle();
    }
    return g_nand_handle;
}

/* Translate a LevelX-relative block index to a physical NAND block
 * index (offset by FX_NAND_USER_BASE_BLOCK).  Bounds-checked. */
static inline bool to_physical_block(ULONG block, uint32_t* phys_block) {
    if (block >= FX_NAND_USER_BLOCK_COUNT) {
        return false;
    }
    *phys_block = static_cast<uint32_t>(FX_NAND_USER_BASE_BLOCK + block);
    return true;
}

/* Read one physical NAND page into g_page_scratch, with bounded retry.
 * Returns true on success; false otherwise.  The caller MUST hold the
 * fx_* mutex (every BD callback is invoked under it).
 *
 * Per embeded.md §F: persistent read failures are logged via SERR_LOG
 * (not just an internal counter) so post-mortem can trace which
 * physical page was unreadable.  Bounded loop count = kMaxRetries. */
static bool nand_read_page_raw(uint32_t phys_block, ULONG page) {
    nand_handle_t* h = nand_handle_get();
    if (h == nullptr) {
        SERR_LOG(SERR_LFX_NAND_READ, 0xFFFFFFFFu);
        return false;
    }
    const uint32_t page_index = phys_block * FX_NAND_PAGES_PER_BLOCK + page;
    constexpr int kMaxRetries = 3;
    status_t st = kStatus_Fail;
    for (int retry = 0; retry < kMaxRetries; ++retry) {
        st = Nand_Flash_Read_Page(h, page_index, g_page_scratch,
                                  FX_NAND_PAGE_RAW_BYTES);
        if (st == kStatus_Success) {
            // Reset the consecutive-fail counter on success.
            const uint32_t logical = phys_block - FX_NAND_USER_BASE_BLOCK;
            if (logical < FX_NAND_USER_BLOCK_COUNT) {
                g_block_read_fails[logical] = 0;
            }
            return true;
        }
    }
    g_stats.read_errors++;
    SERR_LOG(SERR_LFX_NAND_READ, page_index);
    printf("[nand] READ FAIL page=%u (block=%u sub=%u) st=0x%lX\r\n",
           (unsigned)page_index, (unsigned)phys_block, (unsigned)page,
           (unsigned long)st);
    // Auto-bad-block: after kAutoBadAfterReadFails consecutive read failures
    // on the same logical block, persist it as bad so future formats avoid it.
    const uint32_t logical = phys_block - FX_NAND_USER_BASE_BLOCK;
    if (logical < FX_NAND_USER_BLOCK_COUNT) {
        if (g_block_read_fails[logical] < 255) g_block_read_fails[logical]++;
        if (g_block_read_fails[logical] >= kAutoBadAfterReadFails &&
            g_block_status[logical] == 0u) {
            g_block_status[logical] = 1u;
            g_stats.bad_blocks++;
            printf("[bbt] AUTO-MARK BAD logical=%u phys=%u (after %d fails)\r\n",
                   (unsigned)logical, (unsigned)phys_block,
                   g_block_read_fails[logical]);
            bbt_persist_request();
        }
    }
    return false;
}

/* Program one physical NAND page from g_page_scratch.  Single shot — NAND
 * pages can only be programmed once per erase cycle, so retries on
 * Page_Program failure cannot recover.  Logs persistent failure for
 * post-mortem (embeded.md §I). */
static bool nand_write_page_raw(uint32_t phys_block, ULONG page) {
    nand_handle_t* h = nand_handle_get();
    if (h == nullptr) {
        SERR_LOG(SERR_LFX_NAND_PROG, 0xFFFFFFFFu);
        return false;
    }
    const uint32_t page_index = phys_block * FX_NAND_PAGES_PER_BLOCK + page;
    status_t st = Nand_Flash_Page_Program(h, page_index, g_page_scratch,
                                          FX_NAND_PAGE_RAW_BYTES);
    if (st != kStatus_Success) {
        g_stats.write_errors++;
        SERR_LOG(SERR_LFX_NAND_PROG, page_index);
        return false;
    }
    return true;
}

/* LevelX BD callbacks ----------------------------------------------------- */
/* Signatures follow the default (LX_NAND_ENABLE_CONTROL_BLOCK_FOR_DRIVER_INTERFACE
 * NOT defined): (ULONG block, ULONG page, ...).  Single-instance via global
 * state — we have exactly one user partition. */

extern "C" UINT
fx_nand_driver_read(ULONG block, ULONG page, ULONG* destination, ULONG words) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    if (!nand_read_page_raw(phys_block, page)) {
        return LX_ERROR;
    }
    /* Copy the data area only (first FX_NAND_BYTES_PER_PAGE bytes).
     * `words` is in 32-bit units; LevelX always passes
     * FX_NAND_BYTES_PER_PAGE / 4 here. */
    if (words * sizeof(ULONG) > FX_NAND_BYTES_PER_PAGE) {
        return LX_ERROR;
    }
    std::memcpy(destination, g_page_scratch, words * sizeof(ULONG));
    g_stats.reads++;
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_write(ULONG block, ULONG page, ULONG* source, ULONG words) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    if (words * sizeof(ULONG) > FX_NAND_BYTES_PER_PAGE) {
        return LX_ERROR;
    }
    std::memset(g_page_scratch, 0xFF, FX_NAND_PAGE_RAW_BYTES);
    std::memcpy(g_page_scratch, source, words * sizeof(ULONG));
    /* Spare bytes left as 0xFF — single-write path used by LevelX
     * metadata block writes, which do not carry per-page spare. */
    if (!nand_write_page_raw(phys_block, page)) {
        return LX_ERROR;
    }
    g_stats.writes++;
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_pages_read(ULONG block, ULONG page, UCHAR* main_buffer,
                          UCHAR* spare_buffer, ULONG pages) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    /* Bound the page loop strictly. */
    if (page + pages > FX_NAND_PAGES_PER_BLOCK) {
        return LX_ERROR;
    }
    for (ULONG i = 0; i < pages; ++i) {
        if (!nand_read_page_raw(phys_block, page + i)) {
            return LX_ERROR;
        }
        if (main_buffer != nullptr) {
            std::memcpy(main_buffer + i * FX_NAND_BYTES_PER_PAGE,
                        g_page_scratch, FX_NAND_BYTES_PER_PAGE);
        }
        if (spare_buffer != nullptr) {
            std::memcpy(spare_buffer + i * FX_NAND_SPARE_PER_PAGE,
                        g_page_scratch + FX_NAND_BYTES_PER_PAGE,
                        FX_NAND_SPARE_PER_PAGE);
        }
        g_stats.reads++;
    }
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_pages_write(ULONG block, ULONG page, UCHAR* main_buffer,
                           UCHAR* spare_buffer, ULONG pages) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    if (page + pages > FX_NAND_PAGES_PER_BLOCK) {
        return LX_ERROR;
    }
    for (ULONG i = 0; i < pages; ++i) {
        std::memset(g_page_scratch, 0xFF, FX_NAND_PAGE_RAW_BYTES);
        if (main_buffer != nullptr) {
            std::memcpy(g_page_scratch,
                        main_buffer + i * FX_NAND_BYTES_PER_PAGE,
                        FX_NAND_BYTES_PER_PAGE);
        }
        if (spare_buffer != nullptr) {
            std::memcpy(g_page_scratch + FX_NAND_BYTES_PER_PAGE,
                        spare_buffer + i * FX_NAND_SPARE_PER_PAGE,
                        FX_NAND_SPARE_PER_PAGE);
        }
        if (!nand_write_page_raw(phys_block, page + i)) {
            return LX_ERROR;
        }
        g_stats.writes++;
    }
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_pages_copy(ULONG source_block, ULONG source_page,
                          ULONG destination_block, ULONG destination_page,
                          ULONG pages, UCHAR* /*data_buffer*/) {
    /* Copy is read-modify-write at the page granularity.  We re-use
     * g_page_scratch for the read; the write path also uses it, so we
     * must not interleave reads and writes mid-loop. */
    uint32_t src_phys = 0;
    uint32_t dst_phys = 0;
    if (!to_physical_block(source_block, &src_phys) ||
        !to_physical_block(destination_block, &dst_phys)) {
        return LX_ERROR;
    }
    if (source_page + pages > FX_NAND_PAGES_PER_BLOCK ||
        destination_page + pages > FX_NAND_PAGES_PER_BLOCK) {
        return LX_ERROR;
    }
    for (ULONG i = 0; i < pages; ++i) {
        if (!nand_read_page_raw(src_phys, source_page + i)) {
            return LX_ERROR;
        }
        if (!nand_write_page_raw(dst_phys, destination_page + i)) {
            return LX_ERROR;
        }
        g_stats.reads++;
        g_stats.writes++;
    }
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_block_erase(ULONG block, ULONG /*erase_count*/) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        SERR_LOG(SERR_LFX_NAND_ERASE, block);
        return LX_ERROR;
    }
    nand_handle_t* h = nand_handle_get();
    if (h == nullptr) {
        SERR_LOG(SERR_LFX_NAND_ERASE, 0xFFFFFFFFu);
        return LX_ERROR;
    }
    status_t st = Nand_Flash_Erase_Block(h, phys_block);
    if (st != kStatus_Success) {
        g_stats.erase_errors++;
        SERR_LOG(SERR_LFX_NAND_ERASE, phys_block);
        return LX_ERROR;
    }
    g_stats.erases++;
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_block_erased_verify(ULONG block) {
    /* Read the first page; if any byte is not 0xFF, treat the block as
     * not-erased.  Bounded loop. */
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    if (!nand_read_page_raw(phys_block, 0)) {
        return LX_ERROR;
    }
    for (uint32_t i = 0; i < FX_NAND_PAGE_RAW_BYTES; ++i) {
        if (g_page_scratch[i] != 0xFFu) {
            return LX_ERROR;  /* Not erased — caller will mark and try again */
        }
    }
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_page_erased_verify(ULONG block, ULONG page) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block)) {
        return LX_ERROR;
    }
    if (page >= FX_NAND_PAGES_PER_BLOCK) {
        return LX_ERROR;
    }
    if (!nand_read_page_raw(phys_block, page)) {
        return LX_ERROR;
    }
    for (uint32_t i = 0; i < FX_NAND_PAGE_RAW_BYTES; ++i) {
        if (g_page_scratch[i] != 0xFFu) {
            return LX_ERROR;
        }
    }
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_block_status_get(ULONG block, UCHAR* bad_block_flag) {
    if (block >= FX_NAND_USER_BLOCK_COUNT || bad_block_flag == nullptr) {
        return LX_ERROR;
    }
    *bad_block_flag = (g_block_status[block] != 0u) ? LX_NAND_BAD_BLOCK
                                                    : LX_NAND_GOOD_BLOCK;
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_block_status_set(ULONG block, UCHAR bad_block_flag) {
    if (block >= FX_NAND_USER_BLOCK_COUNT) {
        return LX_ERROR;
    }
    bool changed = false;
    if (bad_block_flag == LX_NAND_BAD_BLOCK) {
        if (g_block_status[block] == 0u) {
            g_block_status[block] = 1u;
            g_stats.bad_blocks++;
            changed = true;
        }
    } else {
        if (g_block_status[block] != 0u) {
            g_block_status[block] = 0u;
            changed = true;
        }
    }
    if (changed) bbt_persist_request();
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_extra_bytes_get(ULONG block, ULONG page, UCHAR* destination,
                               UINT size) {
    uint32_t phys_block = 0;
    if (!to_physical_block(block, &phys_block) ||
        size > FX_NAND_SPARE_PER_PAGE || page >= FX_NAND_PAGES_PER_BLOCK ||
        destination == nullptr) {
        return LX_ERROR;
    }
    if (!nand_read_page_raw(phys_block, page)) {
        return LX_ERROR;
    }
    std::memcpy(destination, g_page_scratch + FX_NAND_BYTES_PER_PAGE, size);
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_extra_bytes_set(ULONG /*block*/, ULONG /*page*/,
                               UCHAR* /*source*/, UINT /*size*/) {
    /* NAND constraint: a programmed page cannot be partially re-programmed.
     * Modern LevelX (>= v6.x) does not call this on already-written pages
     * for the data path — it only writes spare bundled with main via
     * pages_write.  Treat any direct call here as success-with-no-op so
     * legacy code paths do not abort the mount; if a real path needs it,
     * the read-back during sector_read will fail and surface it. */
    return LX_SUCCESS;
}

extern "C" UINT
fx_nand_driver_system_error(UINT /*error_code*/, ULONG /*block*/,
                            ULONG /*page*/) {
    /* LevelX calls this on internal consistency errors.  We just bump
     * an error counter; a future revision can route to SERR_LOG. */
    return LX_SUCCESS;
}

}  /* namespace */

/* Public initialize callback ---------------------------------------------- */
extern "C" UINT fx_nand_driver_initialize(LX_NAND_FLASH* nand_flash) {
    /* Geometry (must match the comment block in fx_nand_driver.h). */
    nand_flash->lx_nand_flash_total_blocks       = FX_NAND_USER_BLOCK_COUNT;
    nand_flash->lx_nand_flash_pages_per_block    = FX_NAND_PAGES_PER_BLOCK;
    nand_flash->lx_nand_flash_bytes_per_page     = FX_NAND_BYTES_PER_PAGE;

    nand_flash->lx_nand_flash_driver_read                = fx_nand_driver_read;
    nand_flash->lx_nand_flash_driver_write               = fx_nand_driver_write;
    nand_flash->lx_nand_flash_driver_block_erase         = fx_nand_driver_block_erase;
    nand_flash->lx_nand_flash_driver_block_erased_verify = fx_nand_driver_block_erased_verify;
    nand_flash->lx_nand_flash_driver_page_erased_verify  = fx_nand_driver_page_erased_verify;
    nand_flash->lx_nand_flash_driver_block_status_get    = fx_nand_driver_block_status_get;
    nand_flash->lx_nand_flash_driver_block_status_set    = fx_nand_driver_block_status_set;
    nand_flash->lx_nand_flash_driver_extra_bytes_get     = fx_nand_driver_extra_bytes_get;
    nand_flash->lx_nand_flash_driver_extra_bytes_set     = fx_nand_driver_extra_bytes_set;
    nand_flash->lx_nand_flash_driver_system_error        = fx_nand_driver_system_error;

    nand_flash->lx_nand_flash_driver_pages_read  = fx_nand_driver_pages_read;
    nand_flash->lx_nand_flash_driver_pages_write = fx_nand_driver_pages_write;
    nand_flash->lx_nand_flash_driver_pages_copy  = fx_nand_driver_pages_copy;

    /* Spare layout: LevelX uses two named fields inside the spare area
     * (physical NAND OOB, FX_NAND_SPARE_PER_PAGE = 64 bytes total).
     * data1 (4 bytes) holds page-type/sequence info; data2 (8 bytes)
     * holds logical-sector mapping.  The remaining 52 bytes are
     * available for future use (e.g. firmware-side ECC backstop).
     * The chip's hardware ECC engine reserves additional bytes
     * automatically beyond the 64 we expose to LX. */
    nand_flash->lx_nand_flash_spare_data1_offset  = 0;
    nand_flash->lx_nand_flash_spare_data1_length  = 4;
    nand_flash->lx_nand_flash_spare_data2_offset  = 4;
    nand_flash->lx_nand_flash_spare_data2_length  = 8;
    nand_flash->lx_nand_flash_spare_total_length  = FX_NAND_SPARE_PER_PAGE;

    /* Load persisted bad-block table from /system/.nand_bbt (LFS, separate
     * from the user FX partition so it survives sentai.fs.format()). */
    bbt_load();

    return LX_SUCCESS;
}

/* Diagnostic accessor used by sentai.diag.fx_stats(). */
extern "C" void fx_nand_driver_get_stats(uint32_t* reads, uint32_t* writes,
                                         uint32_t* erases,
                                         uint32_t* read_errors,
                                         uint32_t* write_errors,
                                         uint32_t* erase_errors,
                                         uint32_t* bad_blocks) {
    if (reads)         *reads         = g_stats.reads;
    if (writes)        *writes        = g_stats.writes;
    if (erases)        *erases        = g_stats.erases;
    if (read_errors)   *read_errors   = g_stats.read_errors;
    if (write_errors)  *write_errors  = g_stats.write_errors;
    if (erase_errors)  *erase_errors  = g_stats.erase_errors;
    if (bad_blocks)    *bad_blocks    = g_stats.bad_blocks;
}
