// B8 storage gate: production FileX/LevelX over emulated raw NAND.
//
// This deliberately does not create a RAM filesystem.  The only emulator
// injection is below fx_nand_driver_* at the raw NAND page/program/erase
// boundary, preserving the ARM user-partition stack.

#include <stdint.h>
#include <string.h>

#include "libs/base/fx_user_fs.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_fs_size;
extern "C" volatile uint32_t g_sentai_emu_fs_read_ok;
extern "C" volatile uint32_t g_sentai_emu_fx_reads;
extern "C" volatile uint32_t g_sentai_emu_fx_writes;
extern "C" volatile uint32_t g_sentai_emu_fx_erases;
extern "C" volatile uint32_t g_sentai_emu_fx_errors;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootStorageTaskRunning = 0x0500;
constexpr uint32_t kBootStoragePass = 0x0600;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;
constexpr uint32_t kBootFxInitFailed = 0xE601;
constexpr uint32_t kBootMkdirFailed = 0xE602;
constexpr uint32_t kBootWriteFailed = 0xE603;
constexpr uint32_t kBootSyncFailed = 0xE604;
constexpr uint32_t kBootSizeFailed = 0xE605;
constexpr uint32_t kBootReadFailed = 0xE606;
constexpr uint32_t kBootCompareFailed = 0xE607;

constexpr uintptr_t kLpuart6Base = 0x40090000u;
constexpr uint32_t kLpuartStatOffset = 0x14u;
constexpr uint32_t kLpuartCtrlOffset = 0x18u;
constexpr uint32_t kLpuartDataOffset = 0x1Cu;
constexpr uint32_t kLpuartStatTdre = 1u << 23;
constexpr uint32_t kLpuartCtrlRe = 1u << 18;
constexpr uint32_t kLpuartCtrlTe = 1u << 19;

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

void UartWriteLineHex(const char* label, uint32_t value) {
  UartWrite(label);
  UartWrite("=");
  UartWriteHex(value);
  UartWrite("\r\n");
}

StaticTask_t g_storage_tcb;
StackType_t g_storage_stack[configMINIMAL_STACK_SIZE * 32]
    __attribute__((aligned(8)));

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
  g_sentai_emu_fx_errors = read_errors + write_errors + erase_errors +
                           bad_blocks;
}

void Fail(uint32_t code) {
  SetStats();
  g_sentai_emu_boot_state = code;
  UartWrite("FX_STORAGE FAIL ");
  UartWriteHex(code);
  UartWrite("\r\n");
}

void StorageTask(void*) {
  g_sentai_emu_boot_state = kBootStorageTaskRunning;
  UartWrite("SentAI EMU FX storage smoke start\r\n");

  static constexpr char kPath[] = "/models/smoke.txt";
  static constexpr uint8_t kPayload[] =
      "SentAI B8 FileX LevelX NAND bridge smoke\n";
  uint8_t buf[sizeof(kPayload)] = {};

  if (!FxUserInit(1)) {
    Fail(kBootFxInitFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (!FxUserMakeDirs("/models")) {
    Fail(kBootMkdirFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (!FxUserWriteFile(kPath, kPayload, sizeof(kPayload) - 1)) {
    Fail(kBootWriteFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (!FxUserSync()) {
    Fail(kBootSyncFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  const ssize_t size = FxUserSize(kPath);
  if (size != static_cast<ssize_t>(sizeof(kPayload) - 1)) {
    g_sentai_emu_fs_size = size < 0 ? 0xFFFFFFFFu : static_cast<uint32_t>(size);
    Fail(kBootSizeFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  const size_t n = FxUserReadFile(kPath, buf, sizeof(buf));
  if (n != sizeof(kPayload) - 1) {
    g_sentai_emu_fs_size = static_cast<uint32_t>(n);
    Fail(kBootReadFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (memcmp(buf, kPayload, sizeof(kPayload) - 1) != 0) {
    Fail(kBootCompareFailed);
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  g_sentai_emu_fs_size = static_cast<uint32_t>(size);
  g_sentai_emu_fs_read_ok = 1;
  SetStats();
  g_sentai_emu_boot_state = kBootStoragePass;
  UartWrite("FX_STORAGE PASS\r\n");
  UartWriteLineHex("FX_STORAGE size", g_sentai_emu_fs_size);
  UartWriteLineHex("FX_STORAGE reads", g_sentai_emu_fx_reads);
  UartWriteLineHex("FX_STORAGE writes", g_sentai_emu_fx_writes);
  UartWriteLineHex("FX_STORAGE erases", g_sentai_emu_fx_erases);

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
volatile uint32_t g_sentai_emu_fs_size = 0;
volatile uint32_t g_sentai_emu_fs_read_ok = 0;
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
  UartWrite("SentAI EMU FX storage boot\r\n");

  TaskHandle_t task = xTaskCreateStatic(StorageTask, "fx_storage",
                                        configMINIMAL_STACK_SIZE * 32,
                                        nullptr, tskIDLE_PRIORITY + 2,
                                        g_storage_stack, &g_storage_tcb);
  if (!task) {
    g_sentai_emu_boot_state = kBootCreateTaskFailed;
    while (true) {}
  }

  g_sentai_emu_boot_state = kBootTaskCreated;
  vTaskStartScheduler();

  g_sentai_emu_boot_state = kBootSchedulerReturned;
  while (true) {}
}
