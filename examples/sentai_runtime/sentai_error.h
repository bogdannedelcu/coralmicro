/*
 * SentAI Error Code Taxonomy
 * 
 * Format: 0xMMEE where MM=module, EE=error
 * 
 * Modules:
 *   0x01 = Link (MAVLink)
 *   0x02 = Mesh (Meshtastic)
 *   0x03 = Crazy (Crazyflie)
 *   0x04 = Detection pipeline
 *   0x05 = Watchdog
 *   0x06 = HTTP server
 *   0x07 = Filesystem
 *   0x08 = MicroPython
 *   0x09 = Audio
 *   0x0A = Camera
 *   0x0B = TPU
 *   0x0F = System
 * 
 * Error types (EE):
 *   0x01-0x0F = Timeouts
 *   0x10-0x1F = Queue/buffer issues
 *   0x20-0x2F = Hardware errors
 *   0x30-0x3F = Protocol errors
 *   0x40-0x4F = Resource errors
 *   0xF0-0xFF = Fatal errors
 */

#ifndef SENTAI_ERROR_H
#define SENTAI_ERROR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== Module IDs =====================
#define SERR_MOD_LINK    0x0100
#define SERR_MOD_MESH    0x0200
#define SERR_MOD_CRAZY   0x0300
#define SERR_MOD_DET     0x0400
#define SERR_MOD_WDG     0x0500
#define SERR_MOD_HTTP    0x0600
#define SERR_MOD_FS      0x0700
#define SERR_MOD_MP      0x0800
#define SERR_MOD_AUDIO   0x0900
#define SERR_MOD_CAM     0x0A00
#define SERR_MOD_TPU     0x0B00
#define SERR_MOD_LFX     0x0D00  /* FileX/LevelX migration (0x0Dxx) */
#define SERR_MOD_FLOW    0x0E00  /* sentai.flow stack (0x0Exx) */
#define SERR_MOD_SYS     0x0F00

// ===================== Flow Errors (0x0Exx) =====================
#define SERR_FLOW_M4_NOT_ALIVE       (SERR_MOD_FLOW | 0x01) // magic timeout at enable
#define SERR_FLOW_PUB_GRAB           (SERR_MOD_FLOW | 0x02) // publisher cam_grab_latest fail
#define SERR_FLOW_PUB_TASK_CREATE    (SERR_MOD_FLOW | 0x03) // publisher task create fail
#define SERR_FLOW_M4_HEARTBEAT_STALL (SERR_MOD_FLOW | 0x10) // M4 heartbeat not advancing
#define SERR_FLOW_M4_FRAME_DROPPED   (SERR_MOD_FLOW | 0x11) // M4 dropped frames (publisher faster than M4)
#define SERR_FLOW_BAD_CAM_ID         (SERR_MOD_FLOW | 0x20) // cmd_start with bad cam_id
#define SERR_FLOW_NOTIFY_TIMEOUT     (SERR_MOD_FLOW | 0x30) // ISR notify not received within deadline (sensor stalled?)
#define SERR_FLOW_NOTIFY_OVERRUN     (SERR_MOD_FLOW | 0x31) // ISR notifies arriving faster than task can drain (compute overrun)

// ===================== Link Errors (0x01xx) =====================
#define SERR_LINK_TX_TIMEOUT    (SERR_MOD_LINK | 0x01)  // TX mutex timeout
#define SERR_LINK_TX_FAIL       (SERR_MOD_LINK | 0x02)  // UART write failed
#define SERR_LINK_RX_DROP       (SERR_MOD_LINK | 0x10)  // RX queue full
#define SERR_LINK_RX_PARSE      (SERR_MOD_LINK | 0x30)  // Parse error

// ===================== Mesh Errors (0x02xx) =====================
#define SERR_MESH_TX_TIMEOUT    (SERR_MOD_MESH | 0x01)  // TX mutex timeout
#define SERR_MESH_TX_FAIL       (SERR_MOD_MESH | 0x02)  // UART write failed
#define SERR_MESH_RX_DROP       (SERR_MOD_MESH | 0x10)  // RX queue full
#define SERR_MESH_RX_PARSE      (SERR_MOD_MESH | 0x30)  // Protobuf decode fail

// ===================== Crazy Errors (0x03xx) =====================
#define SERR_CRAZY_TX_TIMEOUT   (SERR_MOD_CRAZY | 0x01) // TX mutex timeout
#define SERR_CRAZY_TX_FAIL      (SERR_MOD_CRAZY | 0x02) // UART write failed
#define SERR_CRAZY_CTS_TIMEOUT  (SERR_MOD_CRAZY | 0x03) // CTS wait timeout
#define SERR_CRAZY_RX_DROP      (SERR_MOD_CRAZY | 0x10) // RX queue full
#define SERR_CRAZY_PING_TIMEOUT (SERR_MOD_CRAZY | 0x04) // Ping timeout
#define SERR_CRAZY_ARM_FAIL     (SERR_MOD_CRAZY | 0x30) // Arm command failed
#define SERR_CRAZY_UNLOCK_FAIL  (SERR_MOD_CRAZY | 0x31) // Unlock RPYT failed
#define SERR_CRAZY_CTS_ABORT    (SERR_MOD_CRAZY | 0x32) // CTS failures → abort
#define SERR_CRAZY_LOG_TOC      (SERR_MOD_CRAZY | 0x33) // Log TOC info failed
#define SERR_CRAZY_LOG_CREATE   (SERR_MOD_CRAZY | 0x34) // Log create block failed
#define SERR_CRAZY_LOG_START    (SERR_MOD_CRAZY | 0x35) // Log start block failed

// ===================== Detection Errors (0x04xx) =====================
#define SERR_DET_QUEUE_FULL     (SERR_MOD_DET | 0x10)   // Detection queue full
#define SERR_DET_QUEUE_CORRUPT  (SERR_MOD_DET | 0x11)   // Queue recv failed after send
#define SERR_DET_INVOKE_FAIL    (SERR_MOD_DET | 0x20)   // TPU invoke failed
#define SERR_DET_TENSOR_SIZE    (SERR_MOD_DET | 0x40)   // Tensor too large
#define SERR_DET_TENSOR_INFO    (SERR_MOD_DET | 0x41)   // Cannot read tensor info
#define SERR_DET_MODEL_NONE     (SERR_MOD_DET | 0x42)   // Model not loaded
#define SERR_DET_CAM_NONE       (SERR_MOD_DET | 0x43)   // Camera not initialized
#define SERR_DET_SYNC_FAIL      (SERR_MOD_DET | 0x44)   // Sync primitives failed
#define SERR_DET_TASK_FAIL      (SERR_MOD_DET | 0x45)   // Task creation failed
#define SERR_DET_ALREADY_RUN    (SERR_MOD_DET | 0x46)   // Already running
#define SERR_DET_EXIT_DIRTY     (SERR_MOD_DET | 0x47)   // Tasks didn't exit cleanly
#define SERR_DET_STARTED        (SERR_MOD_DET | 0x80)   // Started (info)
#define SERR_DET_STOPPED        (SERR_MOD_DET | 0x81)   // Stopped (info)

// ===================== Watchdog Errors (0x05xx) =====================
#define SERR_WDG_WARN           (SERR_MOD_WDG | 0x01)   // Warning: idle > 15s
#define SERR_WDG_DEAD           (SERR_MOD_WDG | 0xF0)   // Fatal: both interfaces dead
#define SERR_WDG_KICK           (SERR_MOD_WDG | 0x00)   // OK: kicked (info only)
#define SERR_WDG_RESET          (SERR_MOD_WDG | 0xF1)   // Software reset triggered

// ===================== HTTP Errors (0x06xx) =====================
#define SERR_HTTP_ALLOC         (SERR_MOD_HTTP | 0x40)  // Allocation failed
#define SERR_HTTP_TIMEOUT       (SERR_MOD_HTTP | 0x01)  // Request timeout

// ===================== Camera Errors (0x0Axx) =====================
#define SERR_CAM_SWITCH_EOF       (SERR_MOD_CAM | 0x00)  // Info: switch via EOF ISR (val=cam_id)
#define SERR_CAM_SWITCH_FALLBACK  (SERR_MOD_CAM | 0x01)  // Sync fallback: ISR did not consume arm (val=cam_id)
#define SERR_CAM_DRAIN_TIMEOUT    (SERR_MOD_CAM | 0x02)  // Post-switch drain wait hit 300ms ceiling (val=wait_ms)
#define SERR_CAM_GRAB_RETRY       (SERR_MOD_CAM | 0x03)  // GetRawFrame failed, toggling to recover (val=recovery_attempt)
#define SERR_CAM_GRAB_FAIL        (SERR_MOD_CAM | 0xF0)  // Fatal: GetRawFrame failed after all recoveries (val=cam_id)

// ===================== TPU Errors (0x0Bxx) =====================
#define SERR_TPU_ARENA_ALLOC    (SERR_MOD_TPU | 0x40)   // Arena allocation failed
#define SERR_TPU_MODEL_LOAD     (SERR_MOD_TPU | 0x41)   // Model load failed
#define SERR_TPU_ALLOC_TENSORS  (SERR_MOD_TPU | 0x42)   // AllocateTensors failed
#define SERR_TPU_INPUT_COUNT    (SERR_MOD_TPU | 0x43)   // Must have 1 input tensor
#define SERR_TPU_NOT_READY      (SERR_MOD_TPU | 0x44)   // EdgeTPU not initialized
// Cale 1 ring-buffer USB transfer path (v1.3)
#define SERR_TPU_RING_LOOP_BOUND   (SERR_MOD_TPU | 0x50) // Ring transfer loop exceeded bounded iters (val=iters)
#define SERR_TPU_RING_SLOT_TIMEOUT (SERR_MOD_TPU | 0x51) // Waiting for slot USB-done took too long (val=slot_idx)
#define SERR_TPU_RING_DMA_FAIL     (SERR_MOD_TPU | 0x52) // eDMA SDRAM->OCRAM producer copy failed (val=bytes)
#define SERR_TPU_RING_USB_SUBMIT   (SERR_MOD_TPU | 0x53) // USB_HostEdgeTpuBulkOutSendAsync returned non-success (val=usb_status)
#define SERR_TPU_RING_DRAIN_TO     (SERR_MOD_TPU | 0x54) // Drain wait for final slots timed out (val=slot_idx)
// Per-invoke USB stage failures (EdgeTpuExecutable::Invoke).  val=bytes pending when failure occurred.
#define SERR_TPU_INV_PARAMS        (SERR_MOD_TPU | 0x60) // SendParameters failed
#define SERR_TPU_INV_INPUTS        (SERR_MOD_TPU | 0x61) // SendInputs failed
#define SERR_TPU_INV_INSTR         (SERR_MOD_TPU | 0x62) // SendInstructions failed
#define SERR_TPU_INV_OUTPUT        (SERR_MOD_TPU | 0x63) // GetOutputs failed
// Multi-slot extension (Phase 1+2, 2026-04-28).
#define SERR_TPU_SLOT_OOB          (SERR_MOD_TPU | 0x70) // Slot index out of range (val=slot)
#define SERR_TPU_SLOT_ALLOC        (SERR_MOD_TPU | 0x71) // Slot arena malloc failed (val=slot)
#define SERR_TPU_SLOT_INTERP       (SERR_MOD_TPU | 0x72) // new MicroInterpreter returned NULL (val=slot)
#define SERR_TPU_SLOT_VEC          (SERR_MOD_TPU | 0x73) // new std::vector returned NULL (val=slot)
#define SERR_TPU_SLOT_NOT_READY    (SERR_MOD_TPU | 0x74) // Pipeline routed cam to unloaded slot (val=cam<<4|slot)

// ===================== Filesystem Errors (0x07xx) =====================
#define SERR_FS_MUTEX_TIMEOUT   (SERR_MOD_FS | 0x01)    // LFS mutex timeout
#define SERR_FS_READ_FAIL       (SERR_MOD_FS | 0x20)    // Read error
#define SERR_FS_WRITE_FAIL      (SERR_MOD_FS | 0x21)    // Write error

// ===================== FileX/LevelX Errors (0x0Dxx) =====================
// Stage codes returned by libs/base/fx_user_fs.cc::fx_smoke_run().
// Phase 1 of LFS->FileX migration; codes 0x0D00..0x0D2F reserved.
#define SERR_LFX_CONFIRM        (SERR_MOD_LFX | 0x00)  // Caller passed wrong magic confirm
#define SERR_LFX_LX_FORMAT      (SERR_MOD_LFX | 0x01)  // lx_nand_flash_format failed (val=lx_status)
#define SERR_LFX_LX_OPEN        (SERR_MOD_LFX | 0x02)  // lx_nand_flash_open failed (val=lx_status)
#define SERR_LFX_FX_FORMAT      (SERR_MOD_LFX | 0x03)  // fx_media_format failed (val=fx_status)
#define SERR_LFX_FX_OPEN        (SERR_MOD_LFX | 0x04)  // fx_media_open failed (val=fx_status)
#define SERR_LFX_FT             (SERR_MOD_LFX | 0x05)  // fx_fault_tolerant_enable failed (non-fatal)
#define SERR_LFX_FILE_CREATE    (SERR_MOD_LFX | 0x10)  // fx_file_create failed (val=fx_status)
#define SERR_LFX_FILE_OPEN      (SERR_MOD_LFX | 0x11)  // fx_file_open failed (val=fx_status)
#define SERR_LFX_FILE_WRITE     (SERR_MOD_LFX | 0x12)  // fx_file_write failed (val=fx_status)
#define SERR_LFX_FILE_READ      (SERR_MOD_LFX | 0x13)  // fx_file_read failed or short read
#define SERR_LFX_VERIFY         (SERR_MOD_LFX | 0x14)  // Readback content mismatch
#define SERR_LFX_CLOSE          (SERR_MOD_LFX | 0x20)  // fx_media_close failed (val=fx_status)
// NAND BD-adapter faults (Phase 3.5 audit, 2026-04-28)
#define SERR_LFX_NAND_READ      (SERR_MOD_LFX | 0x21)  // Nand_Flash_Read_Page failed after retries (val=phys_page)
#define SERR_LFX_NAND_PROG      (SERR_MOD_LFX | 0x22)  // Nand_Flash_Page_Program failed (val=phys_page)
#define SERR_LFX_NAND_ERASE     (SERR_MOD_LFX | 0x23)  // Nand_Flash_Erase_Block failed (val=phys_block)
#define SERR_LFX_LBA_RANGE      (SERR_MOD_LFX | 0x24)  // MSC LBA out of range (val=lba)
#define SERR_LFX_LOCK_TIMEOUT   (SERR_MOD_LFX | 0x25)  // FxUser mutex acquisition timed out (val=ms)
#define SERR_LFX_NOT_MOUNTED    (SERR_MOD_LFX | 0x26)  // FxUser* called on unmounted volume (val=op_id)
#define SERR_LFX_BBT_PERSIST    (SERR_MOD_LFX | 0x27)  // /system/.nand_bbt write failed
// 2026-05-09 — bounded retry + SAFE-MODE refactor of FxUserInit/FxUserSync.
// Previously FxUserInit silently auto-formatted on any mount failure,
// destroying user data after a single transient ECC error.  These codes
// trace the new path: retry-then-SAFE-MODE; auto-format only on virgin NAND.
#define SERR_LFX_MOUNT_RETRY_OK (SERR_MOD_LFX | 0x28)  // mount succeeded after N retries (val=attempt)
#define SERR_LFX_MOUNT_FAIL_SAFE (SERR_MOD_LFX | 0x29) // mount failed; entered SAFE MODE (val=last_lx_status)
#define SERR_LFX_FIRST_BOOT_FORMAT (SERR_MOD_LFX | 0x2A) // virgin NAND, auto-formatting (val=lx_status)
#define SERR_LFX_SYNC_LX_CLOSE  (SERR_MOD_LFX | 0x2B)  // FxUserSync: lx_close failed (val=lx_status)
#define SERR_LFX_SYNC_LX_REOPEN (SERR_MOD_LFX | 0x2C)  // FxUserSync: lx_reopen failed (val=lx_status); volume now unmounted
#define SERR_LFX_LXONLY_FAIL_SAFE (SERR_MOD_LFX | 0x2D) // FxUserOpenLxOnly: failed; refusing implicit format (val=lx_status)
#define SERR_LFX_RESTORE_LFS    (SERR_MOD_LFX | 0xF0)  // LfsUserInit re-format failed after smoke

// ===================== USB Errors (0x0Cxx) =====================
#define SERR_MOD_USB     0x0C00
#define SERR_USB_NCM_TX_DROP    (SERR_MOD_USB | 0x01)  // NCM TX dropped (not attached)
#define SERR_USB_NCM_RX_DROP    (SERR_MOD_USB | 0x02)  // NCM RX dropped (endpoint busy)
#define SERR_USB_MSC_READ_FAIL  (SERR_MOD_USB | 0x10)  // NAND read fail (val=page)
#define SERR_USB_MSC_WRITE_FAIL (SERR_MOD_USB | 0x11)  // NAND write fail (val=page)
#define SERR_USB_MSC_ERASE_FAIL (SERR_MOD_USB | 0x12)  // NAND erase fail (val=block)

// ===================== System Errors (0x0Fxx) =====================
#define SERR_SYS_BOOT_ATTEMPT   (SERR_MOD_SYS | 0x01)   // Boot attempt N (val=attempt_count)
#define SERR_SYS_RECOVERY_MODE  (SERR_MOD_SYS | 0xF0)   // Recovery mode entered (val=attempts)
#define SERR_SYS_STACK_OVF      (SERR_MOD_SYS | 0xF1)   // Stack overflow
#define SERR_SYS_MALLOC_FAIL    (SERR_MOD_SYS | 0xF2)   // Malloc failed
#define SERR_SYS_ASSERT         (SERR_MOD_SYS | 0xF3)   // Assert failed
#define SERR_SYS_BUS_FAULT      (SERR_MOD_SYS | 0xF4)   // Bus fault (BusFault_Handler)
#define SERR_SYS_USAGE_FAULT    (SERR_MOD_SYS | 0xF5)   // Usage fault (div-by-zero, INVSTATE)
#define SERR_SYS_MEMMANAGE      (SERR_MOD_SYS | 0xF6)   // MemManage/MPU fault
#define SERR_SYS_HARDFAULT      (SERR_MOD_SYS | 0xFF)   // Hard fault

// ===================== Helper macros =====================
#define SERR_MODULE(code)  (((code) >> 8) & 0xFF)
#define SERR_ERROR(code)   ((code) & 0xFF)

// Compact error logging: "E:MMEE:val" format (12 chars max)
// Example: "E:0101:3" = Link TX timeout, retry 3
#define SERR_LOG(code, val) printf("E:%04X:%lu\r\n", (unsigned)(code), (unsigned long)(val))

// Log with context (for crash log): includes timestamp
#define SERR_LOG_TS(code, val) do { \
    uint32_t _ts = xTaskGetTickCount() * portTICK_PERIOD_MS; \
    printf("E:%04X:%lu@%lu\r\n", (unsigned)(code), (unsigned long)(val), (unsigned long)_ts); \
} while(0)

#ifdef __cplusplus
}
#endif

#endif // SENTAI_ERROR_H
