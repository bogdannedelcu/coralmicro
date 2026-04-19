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
#define SERR_MOD_SYS     0x0F00

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

// ===================== TPU Errors (0x0Bxx) =====================
#define SERR_TPU_ARENA_ALLOC    (SERR_MOD_TPU | 0x40)   // Arena allocation failed
#define SERR_TPU_MODEL_LOAD     (SERR_MOD_TPU | 0x41)   // Model load failed
#define SERR_TPU_ALLOC_TENSORS  (SERR_MOD_TPU | 0x42)   // AllocateTensors failed
#define SERR_TPU_INPUT_COUNT    (SERR_MOD_TPU | 0x43)   // Must have 1 input tensor
#define SERR_TPU_NOT_READY      (SERR_MOD_TPU | 0x44)   // EdgeTPU not initialized

// ===================== Filesystem Errors (0x07xx) =====================
#define SERR_FS_MUTEX_TIMEOUT   (SERR_MOD_FS | 0x01)    // LFS mutex timeout
#define SERR_FS_READ_FAIL       (SERR_MOD_FS | 0x20)    // Read error
#define SERR_FS_WRITE_FAIL      (SERR_MOD_FS | 0x21)    // Write error

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
