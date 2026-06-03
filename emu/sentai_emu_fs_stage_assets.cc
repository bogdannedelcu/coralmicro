// B8.9c asset staging gate: host files -> guest FxUser/FileX/LevelX NAND.
//
// Renode's asset_bridge only provides bytes from host files.  The guest still
// owns the filesystem semantics and writes through the production FxUser*
// stack, so the final REPL boot sees a normal user-partition FAT volume.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "libs/base/fx_user_fs.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_stage_files;
extern "C" volatile uint32_t g_sentai_emu_stage_bytes;
extern "C" volatile uint32_t g_sentai_emu_stage_errors;
extern "C" volatile uint32_t g_sentai_emu_stage_checksum;
extern "C" volatile uint32_t g_sentai_emu_fx_reads;
extern "C" volatile uint32_t g_sentai_emu_fx_writes;
extern "C" volatile uint32_t g_sentai_emu_fx_erases;
extern "C" volatile uint32_t g_sentai_emu_fx_errors;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootStageRunning = 0x0500;
constexpr uint32_t kBootStagePass = 0x0700;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;
constexpr uint32_t kBootFxInitFailed = 0xE701;
constexpr uint32_t kBootMkdirFailed = 0xE702;
constexpr uint32_t kBootAssetStatFailed = 0xE703;
constexpr uint32_t kBootAssetReadFailed = 0xE704;
constexpr uint32_t kBootAssetWriteFailed = 0xE705;
constexpr uint32_t kBootAssetSizeFailed = 0xE706;
constexpr uint32_t kBootSyncFailed = 0xE707;
constexpr uint32_t kBootFxFormatFailed = 0xE708;

constexpr uintptr_t kLpuart6Base = 0x40090000u;
constexpr uint32_t kLpuartStatOffset = 0x14u;
constexpr uint32_t kLpuartCtrlOffset = 0x18u;
constexpr uint32_t kLpuartDataOffset = 0x1Cu;
constexpr uint32_t kLpuartStatTdre = 1u << 23;
constexpr uint32_t kLpuartCtrlRe = 1u << 18;
constexpr uint32_t kLpuartCtrlTe = 1u << 19;

constexpr uintptr_t kAssetBridgeBase = 0x40901000u;
constexpr uint32_t kAssetCmdStat = 1u;
constexpr uint32_t kAssetCmdRead = 2u;
constexpr uint32_t kAssetStatusOk = 1u;
constexpr size_t kChunkBytes = 64 * 1024;

struct StageAsset {
  uint32_t index;
  const char* guest_path;
  const char* parent_dir;
};

constexpr StageAsset kAssets[] = {
    {0, "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite", "/models"},
    {1, "/images/cat_640x480.bmp", "/images"},
    {2, "/mission.py", nullptr},
};

StaticTask_t g_stage_tcb;
StackType_t g_stage_stack[configMINIMAL_STACK_SIZE * 48]
    __attribute__((aligned(8)));
uint8_t g_stage_buf[kChunkBytes] __attribute__((aligned(8)));

volatile uint32_t& UartReg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kLpuart6Base + offset);
}

void UartInit() {
  UartReg(kLpuartCtrlOffset) = kLpuartCtrlRe | kLpuartCtrlTe;
}

void UartPutChar(char ch) {
  for (int i = 0; i < 1000; ++i) {
    if (UartReg(kLpuartStatOffset) & kLpuartStatTdre) break;
  }
  UartReg(kLpuartDataOffset) = static_cast<uint8_t>(ch);
}

void UartWrite(const char* s) {
  while (*s) UartPutChar(*s++);
}

void UartWriteHex(uint32_t value) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  UartWrite("0x");
  for (int shift = 28; shift >= 0; shift -= 4) {
    UartPutChar(kHex[(value >> shift) & 0xFu]);
  }
}

void UartWriteDec(uint32_t value) {
  char tmp[11];
  int n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0u && n < static_cast<int>(sizeof(tmp)));
  while (n > 0) UartPutChar(tmp[--n]);
}

void UartWriteLineHex(const char* label, uint32_t value) {
  UartWrite(label);
  UartWrite("=");
  UartWriteHex(value);
  UartWrite("\r\n");
}

volatile uint32_t& AssetReg(uint32_t offset) {
  return *reinterpret_cast<volatile uint32_t*>(kAssetBridgeBase + offset);
}

bool AssetStat(uint32_t index, uint32_t* size) {
  AssetReg(0x04) = index;
  AssetReg(0x00) = kAssetCmdStat;
  if (AssetReg(0x14) != kAssetStatusOk) return false;
  if (size) *size = AssetReg(0x18);
  return true;
}

bool AssetRead(uint32_t index, uint32_t offset, uint8_t* dst, uint32_t len,
               uint32_t* bytes_read, uint32_t* checksum) {
  AssetReg(0x04) = index;
  AssetReg(0x08) = offset;
  AssetReg(0x0C) = reinterpret_cast<uint32_t>(dst);
  AssetReg(0x10) = len;
  AssetReg(0x00) = kAssetCmdRead;
  if (AssetReg(0x14) != kAssetStatusOk) return false;
  if (bytes_read) *bytes_read = AssetReg(0x1C);
  if (checksum) *checksum = AssetReg(0x20);
  return true;
}

void SetStats() {
  uint32_t reads = 0;
  uint32_t writes = 0;
  uint32_t erases = 0;
  uint32_t read_errors = 0;
  uint32_t write_errors = 0;
  uint32_t erase_errors = 0;
  uint32_t bad_blocks = 0;
  fx_nand_driver_get_stats(&reads, &writes, &erases, &read_errors,
                           &write_errors, &erase_errors, &bad_blocks);
  g_sentai_emu_fx_reads = reads;
  g_sentai_emu_fx_writes = writes;
  g_sentai_emu_fx_erases = erases;
  g_sentai_emu_fx_errors =
      read_errors + write_errors + erase_errors + bad_blocks;
}

void Fail(uint32_t code) {
  ++g_sentai_emu_stage_errors;
  SetStats();
  g_sentai_emu_boot_state = code;
  UartWrite("FS_STAGE FAIL ");
  UartWriteHex(code);
  UartWrite("\r\n");
}

bool StageOne(const StageAsset& asset) {
  uint32_t expected = 0;
  if (!AssetStat(asset.index, &expected)) {
    Fail(kBootAssetStatFailed);
    return false;
  }
  if (asset.parent_dir && !FxUserMakeDirs(asset.parent_dir)) {
    Fail(kBootMkdirFailed);
    return false;
  }

  const ssize_t existing = FxUserSize(asset.guest_path);
  if (existing == static_cast<ssize_t>(expected)) {
    ++g_sentai_emu_stage_files;
    UartWrite("FS_STAGE SKIP index=");
    UartWriteDec(asset.index);
    UartWrite(" size=");
    UartWriteDec(expected);
    UartWrite(" path=");
    UartWrite(asset.guest_path);
    UartWrite("\r\n");
    return true;
  }

  uint32_t offset = 0;
  bool first = true;
  while (offset < expected) {
    const uint32_t remaining = expected - offset;
    const uint32_t want =
        remaining > kChunkBytes ? static_cast<uint32_t>(kChunkBytes)
                                : remaining;
    uint32_t got = 0;
    uint32_t checksum = 0;
    if (!AssetRead(asset.index, offset, g_stage_buf, want, &got, &checksum) ||
        got == 0 || got > want) {
      Fail(kBootAssetReadFailed);
      return false;
    }
    const int ok = first
                       ? FxUserWriteFile(asset.guest_path, g_stage_buf, got)
                       : FxUserAppendFile(asset.guest_path, g_stage_buf, got);
    if (!ok) {
      Fail(kBootAssetWriteFailed);
      return false;
    }
    first = false;
    offset += got;
    g_sentai_emu_stage_bytes += got;
    g_sentai_emu_stage_checksum =
        (g_sentai_emu_stage_checksum + checksum) & 0xFFFFFFFFu;
  }

  if (expected == 0 && !FxUserWriteFile(asset.guest_path, nullptr, 0)) {
    Fail(kBootAssetWriteFailed);
    return false;
  }

  const ssize_t actual = FxUserSize(asset.guest_path);
  if (actual != static_cast<ssize_t>(expected)) {
    Fail(kBootAssetSizeFailed);
    return false;
  }

  ++g_sentai_emu_stage_files;
  UartWrite("FS_STAGE FILE index=");
  UartWriteDec(asset.index);
  UartWrite(" size=");
  UartWriteDec(expected);
  UartWrite(" path=");
  UartWrite(asset.guest_path);
  UartWrite("\r\n");
  return true;
}

void StageTask(void*) {
  g_sentai_emu_boot_state = kBootStageRunning;
  UartWrite("SentAI EMU FS asset staging start\r\n");

  if (!FxUserInit(0)) {
    UartWrite("FS_STAGE FORMAT\r\n");
    if (!FxUserInit(1)) {
      Fail(kBootFxFormatFailed);
      while (true) vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  for (const StageAsset& asset : kAssets) {
    if (!StageOne(asset)) {
      while (true) vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  if (!FxUserSync()) {
    Fail(kBootSyncFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  SetStats();
  g_sentai_emu_boot_state = kBootStagePass;
  UartWrite("FS_STAGE PASS\r\n");
  UartWriteLineHex("FS_STAGE files", g_sentai_emu_stage_files);
  UartWriteLineHex("FS_STAGE bytes", g_sentai_emu_stage_bytes);
  UartWriteLineHex("FS_STAGE checksum", g_sentai_emu_stage_checksum);
  UartWriteLineHex("FS_STAGE reads", g_sentai_emu_fx_reads);
  UartWriteLineHex("FS_STAGE writes", g_sentai_emu_fx_writes);
  UartWriteLineHex("FS_STAGE erases", g_sentai_emu_fx_erases);

  while (true) {
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_stage_files = 0;
volatile uint32_t g_sentai_emu_stage_bytes = 0;
volatile uint32_t g_sentai_emu_stage_errors = 0;
volatile uint32_t g_sentai_emu_stage_checksum = 0;
volatile uint32_t g_sentai_emu_fx_reads = 0;
volatile uint32_t g_sentai_emu_fx_writes = 0;
volatile uint32_t g_sentai_emu_fx_erases = 0;
volatile uint32_t g_sentai_emu_fx_errors = 0;
}

extern "C" int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  g_sentai_emu_boot_state = kBootEnteredMain;
  UartInit();
  UartWrite("SentAI EMU FS asset staging boot\r\n");

  TaskHandle_t task =
      xTaskCreateStatic(StageTask, "fs_stage", configMINIMAL_STACK_SIZE * 48,
                        nullptr, tskIDLE_PRIORITY + 2, g_stage_stack,
                        &g_stage_tcb);
  if (!task) {
    g_sentai_emu_boot_state = kBootCreateTaskFailed;
    while (true) {}
  }

  g_sentai_emu_boot_state = kBootTaskCreated;
  vTaskStartScheduler();

  g_sentai_emu_boot_state = kBootSchedulerReturned;
  while (true) {}
}
