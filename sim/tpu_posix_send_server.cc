// tpu_posix_send_server.cc -- host-side physical Coral Send* bridge.
//
// This is intentionally below EdgeTpuManager/EdgeTpuExecutable.  A guest running
// in Renode produces the same parameter/input/instruction/output/event calls as
// the ARM firmware; this process owns the physical libusb Coral connection and
// executes those calls through the production TpuDriver implementation.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "libs/tpu/edgetpu_driver.h"
#include "libs/tpu/usb_host_edgetpu.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" {
volatile uint8_t g_sentai_tpu_trace = 0;

extern volatile uint32_t g_sentai_tpu_send_params_calls;
extern volatile uint32_t g_sentai_tpu_send_params_bytes;
extern volatile uint32_t g_sentai_tpu_send_ins_calls;
extern volatile uint32_t g_sentai_tpu_send_ins_bytes;
extern volatile uint32_t g_sentai_tpu_send_inputs_calls;
extern volatile uint32_t g_sentai_tpu_send_inputs_bytes;
extern void sentai_tpu_call_reset(void);

extern volatile uint32_t g_sentai_tpu_posix_usb_out_calls;
extern volatile uint64_t g_sentai_tpu_posix_usb_out_req;
extern volatile uint64_t g_sentai_tpu_posix_usb_out_done;
extern volatile uint64_t g_sentai_tpu_posix_usb_out_us;
extern volatile uint32_t g_sentai_tpu_posix_usb_in_calls;
extern volatile uint64_t g_sentai_tpu_posix_usb_in_req;
extern volatile uint64_t g_sentai_tpu_posix_usb_in_done;
extern volatile uint64_t g_sentai_tpu_posix_usb_in_us;
extern volatile uint32_t g_sentai_tpu_posix_usb_event_calls;
extern volatile uint64_t g_sentai_tpu_posix_usb_event_req;
extern volatile uint64_t g_sentai_tpu_posix_usb_event_done;
extern volatile uint64_t g_sentai_tpu_posix_usb_event_us;
extern volatile uint32_t g_sentai_tpu_posix_usb_intr_calls;
extern volatile uint64_t g_sentai_tpu_posix_usb_intr_req;
extern volatile uint64_t g_sentai_tpu_posix_usb_intr_done;
extern volatile uint64_t g_sentai_tpu_posix_usb_intr_us;
extern volatile uint32_t g_sentai_tpu_posix_usb_timeouts;
extern volatile uint32_t g_sentai_tpu_posix_usb_failed;
extern void sentai_tpu_posix_usb_stats_reset(void);

extern volatile uint32_t g_sentai_tpu_chunk_size;
extern volatile uint32_t g_sentai_tpu_sim_bulkin_chunk_size;
extern volatile uint32_t g_sentai_tpu_sim_outfeed_chunk_length;
extern volatile uint32_t g_sentai_tpu_sim_bulkin_queue_depth;
extern volatile int g_sentai_tpu_sim_break_on_short_bulkin;
extern volatile int g_sentai_tpu_sim_sleep_after_bulkin;
extern volatile int g_sentai_tpu_posix_fast_sync_wait;
}

namespace {

constexpr uint32_t kSimMaxBulkOutChunkSize = 1024u * 1024u;

struct Args {
  const char* cmd_path = nullptr;
  const char* perf_name = "low";
  coralmicro::PerformanceMode perf = coralmicro::PerformanceMode::kLow;
  uint32_t chunk_size = kSimMaxBulkOutChunkSize;
  uint32_t bulkin_chunk_size = 256u;
  uint32_t outfeed_chunk_length = 0x20u;
  uint32_t bulkin_queue_depth = 0;
  bool break_short_bulkin = false;
  bool sleep_after_bulkin = false;
  bool fast_sync_wait = false;
};

std::vector<uint8_t> ReadFile(const char* path) {
  std::FILE* f = std::fopen(path, "rb");
  if (!f) return {};
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (n < 0) {
    std::fclose(f);
    return {};
  }
  std::vector<uint8_t> out(static_cast<size_t>(n));
  if (!out.empty()) {
    size_t got = std::fread(out.data(), 1, out.size(), f);
    if (got != out.size()) {
      std::fclose(f);
      return {};
    }
  }
  std::fclose(f);
  return out;
}

bool WriteFile(const char* path, const uint8_t* data, size_t size) {
  std::FILE* f = std::fopen(path, "wb");
  if (!f) return false;
  size_t wrote = size == 0 ? 0 : std::fwrite(data, 1, size, f);
  std::fclose(f);
  return wrote == size;
}

const char* SkipSpace(const char* p) {
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

void ResetStats() {
  sentai_tpu_call_reset();
  sentai_tpu_posix_usb_stats_reset();
}

void PrintStats(const char* cmd, bool ok, long long ms) {
  std::printf(
      "SEND_STATS cmd=%s ok=%d ms=%lld "
      "params_calls=%u params_bytes=%u "
      "input_calls=%u input_bytes=%u "
      "ins_calls=%u ins_bytes=%u "
      "usb_out_calls=%u usb_out_req=%llu usb_out_done=%llu usb_out_us=%llu "
      "usb_in_calls=%u usb_in_req=%llu usb_in_done=%llu usb_in_us=%llu "
      "usb_event_calls=%u usb_event_req=%llu usb_event_done=%llu usb_event_us=%llu "
      "usb_intr_calls=%u usb_intr_req=%llu usb_intr_done=%llu usb_intr_us=%llu "
      "usb_timeouts=%u usb_failed=%u\n",
      cmd, ok ? 1 : 0, ms,
      (unsigned)g_sentai_tpu_send_params_calls,
      (unsigned)g_sentai_tpu_send_params_bytes,
      (unsigned)g_sentai_tpu_send_inputs_calls,
      (unsigned)g_sentai_tpu_send_inputs_bytes,
      (unsigned)g_sentai_tpu_send_ins_calls,
      (unsigned)g_sentai_tpu_send_ins_bytes,
      (unsigned)g_sentai_tpu_posix_usb_out_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_out_req,
      (unsigned long long)g_sentai_tpu_posix_usb_out_done,
      (unsigned long long)g_sentai_tpu_posix_usb_out_us,
      (unsigned)g_sentai_tpu_posix_usb_in_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_in_req,
      (unsigned long long)g_sentai_tpu_posix_usb_in_done,
      (unsigned long long)g_sentai_tpu_posix_usb_in_us,
      (unsigned)g_sentai_tpu_posix_usb_event_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_event_req,
      (unsigned long long)g_sentai_tpu_posix_usb_event_done,
      (unsigned long long)g_sentai_tpu_posix_usb_event_us,
      (unsigned)g_sentai_tpu_posix_usb_intr_calls,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_req,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_done,
      (unsigned long long)g_sentai_tpu_posix_usb_intr_us,
      (unsigned)g_sentai_tpu_posix_usb_timeouts,
      (unsigned)g_sentai_tpu_posix_usb_failed);
}

bool LoadPayload(const char* path, std::vector<uint8_t>* payload) {
  *payload = ReadFile(path);
  if (!payload->empty()) return true;
  std::FILE* f = std::fopen(path, "rb");
  if (!f) return false;
  std::fclose(f);
  return true;
}

bool ReadNextCommand(const char* cmd_path, uint32_t* last_seq, char* line,
                     size_t line_size) {
  if (!cmd_path) {
    return std::fgets(line, static_cast<int>(line_size), stdin) != nullptr;
  }

  while (true) {
    std::vector<uint8_t> bytes = ReadFile(cmd_path);
    if (!bytes.empty()) {
      bytes.push_back(0);
      uint32_t seq = 0;
      int pos = 0;
      if (std::sscanf(reinterpret_cast<const char*>(bytes.data()), "%u %n",
                      &seq, &pos) == 1 &&
          seq != *last_seq && pos > 0 &&
          pos < static_cast<int>(bytes.size())) {
        *last_seq = seq;
        std::snprintf(line, line_size, "%s",
                      reinterpret_cast<const char*>(bytes.data()) + pos);
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void ServerTask(void* arg) {
  const Args* args = reinterpret_cast<const Args*>(arg);

  usb_host_edgetpu_instance_t* inst = nullptr;
  usb_status_t st = USB_HostEdgeTpuOpenPosix(&inst);
  if (st != kStatus_USB_Success || !inst) {
    std::printf("SEND_SERVER_FAIL open_posix status=%u\n", (unsigned)st);
    std::fflush(stdout);
    _Exit(2);
  }

  coralmicro::TpuDriver driver;
  if (!driver.Initialize(inst, args->perf)) {
    std::printf("SEND_SERVER_FAIL driver_initialize\n");
    USB_HostEdgeTpuClosePosix(inst);
    std::fflush(stdout);
    _Exit(3);
  }

  std::printf("SEND_SERVER_READY perf=%s chunk_size=%u bulkin_chunk_size=%u "
              "outfeed_chunk_length=0x%x bulkin_queue_depth=%u\n",
              args->perf_name, (unsigned)g_sentai_tpu_chunk_size,
              (unsigned)g_sentai_tpu_sim_bulkin_chunk_size,
              (unsigned)g_sentai_tpu_sim_outfeed_chunk_length,
              (unsigned)g_sentai_tpu_sim_bulkin_queue_depth);
  std::fflush(stdout);

  char line[512];
  uint32_t last_seq = 0;
  while (ReadNextCommand(args->cmd_path, &last_seq, line, sizeof(line))) {
    char* nl = std::strchr(line, '\n');
    if (nl) *nl = 0;
    nl = std::strchr(line, '\r');
    if (nl) *nl = 0;

    if (std::strncmp(line, "stop", 4) == 0) {
      std::printf("SEND_SERVER_STOPPED\n");
      std::fflush(stdout);
      break;
    }

    const auto t0 = std::chrono::steady_clock::now();
    ResetStats();
    bool ok = false;
    const char* done_cmd = "unknown";
    uint32_t bytes = 0;

    if (std::strncmp(line, "params", 6) == 0) {
      done_cmd = "params";
      const char* path = SkipSpace(line + 6);
      std::vector<uint8_t> payload;
      ok = LoadPayload(path, &payload) &&
           driver.SendParameters(payload.data(), payload.size());
      bytes = static_cast<uint32_t>(payload.size());
    } else if (std::strncmp(line, "inputs", 6) == 0) {
      done_cmd = "inputs";
      const char* path = SkipSpace(line + 6);
      std::vector<uint8_t> payload;
      ok = LoadPayload(path, &payload) &&
           driver.SendInputs(payload.data(), payload.size());
      bytes = static_cast<uint32_t>(payload.size());
    } else if (std::strncmp(line, "ins", 3) == 0) {
      done_cmd = "ins";
      const char* path = SkipSpace(line + 3);
      std::vector<uint8_t> payload;
      ok = LoadPayload(path, &payload) &&
           driver.SendInstructions(payload.data(), payload.size());
      bytes = static_cast<uint32_t>(payload.size());
    } else if (std::strncmp(line, "output", 6) == 0) {
      done_cmd = "output";
      uint32_t length = 0;
      int pos = 0;
      if (std::sscanf(line + 6, "%u %n", &length, &pos) == 1) {
        const char* path = SkipSpace(line + 6 + pos);
        std::vector<uint8_t> out(length);
        ok = driver.GetOutputs(out.data(), length) &&
             WriteFile(path, out.data(), out.size());
        bytes = length;
      }
    } else if (std::strncmp(line, "event", 5) == 0) {
      done_cmd = "event";
      ok = driver.ReadEvent();
    }

    const auto t1 = std::chrono::steady_clock::now();
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    PrintStats(done_cmd, ok, ms);
    std::printf("SEND_SERVER_DONE cmd=%s rc=%d bytes=%u ms=%lld\n",
                done_cmd, ok ? 0 : 1, (unsigned)bytes, ms);
    std::fflush(stdout);
  }

  USB_HostEdgeTpuClosePosix(inst);
  std::fflush(stdout);
  _Exit(0);
}

bool ParsePerf(const char* mode, Args* args) {
  args->perf_name = mode;
  if (std::strcmp(mode, "low") == 0) {
    args->perf = coralmicro::PerformanceMode::kLow;
  } else if (std::strcmp(mode, "medium") == 0) {
    args->perf = coralmicro::PerformanceMode::kMedium;
  } else if (std::strcmp(mode, "high") == 0) {
    args->perf = coralmicro::PerformanceMode::kHigh;
  } else if (std::strcmp(mode, "max") == 0) {
    args->perf = coralmicro::PerformanceMode::kMax;
  } else {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  static Args args;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--server-cmd") == 0 && i + 1 < argc) {
      args.cmd_path = argv[++i];
    } else if (std::strcmp(argv[i], "--perf") == 0 && i + 1 < argc) {
      if (!ParsePerf(argv[++i], &args)) {
        std::printf("SEND_SERVER_FAIL bad_perf=%s\n", argv[i]);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--chunk-size") == 0 && i + 1 < argc) {
      args.chunk_size = static_cast<uint32_t>(
          std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--bulkin-chunk-size") == 0 &&
               i + 1 < argc) {
      args.bulkin_chunk_size = static_cast<uint32_t>(
          std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--outfeed-chunk-length") == 0 &&
               i + 1 < argc) {
      args.outfeed_chunk_length = static_cast<uint32_t>(
          std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--bulkin-queue-depth") == 0 &&
               i + 1 < argc) {
      args.bulkin_queue_depth = static_cast<uint32_t>(
          std::strtoul(argv[++i], nullptr, 0));
    } else if (std::strcmp(argv[i], "--break-short-bulkin") == 0) {
      args.break_short_bulkin = true;
    } else if (std::strcmp(argv[i], "--sleep-after-bulkin") == 0) {
      args.sleep_after_bulkin = true;
    } else if (std::strcmp(argv[i], "--fast-sync-wait") == 0) {
      args.fast_sync_wait = true;
    } else {
      std::printf("usage: %s [--server-cmd PATH] [--perf low|medium|high|max]\n",
                  argv[0]);
      return 1;
    }
  }

  if (args.chunk_size < 4096u) args.chunk_size = 4096u;
  if (args.chunk_size > kSimMaxBulkOutChunkSize) {
    args.chunk_size = kSimMaxBulkOutChunkSize;
  }
  if (args.bulkin_chunk_size < 64u) args.bulkin_chunk_size = 64u;
  if (args.bulkin_chunk_size > 64u * 1024u) {
    args.bulkin_chunk_size = 64u * 1024u;
  }
  if (args.outfeed_chunk_length != 0x20u &&
      args.outfeed_chunk_length != 0x80u) {
    args.outfeed_chunk_length = 0x20u;
  }
  if (args.bulkin_queue_depth > 32u) args.bulkin_queue_depth = 32u;
  g_sentai_tpu_chunk_size = args.chunk_size;
  g_sentai_tpu_sim_bulkin_chunk_size = args.bulkin_chunk_size;
  g_sentai_tpu_sim_outfeed_chunk_length = args.outfeed_chunk_length;
  g_sentai_tpu_sim_bulkin_queue_depth = args.bulkin_queue_depth;
  g_sentai_tpu_sim_break_on_short_bulkin = args.break_short_bulkin ? 1 : 0;
  g_sentai_tpu_sim_sleep_after_bulkin = args.sleep_after_bulkin ? 1 : 0;
  g_sentai_tpu_posix_fast_sync_wait = args.fast_sync_wait ? 1 : 0;

  if (xTaskCreate(ServerTask, "tpu_send_srv", 16384, &args,
                  tskIDLE_PRIORITY + 1, nullptr) != pdPASS) {
    std::printf("SEND_SERVER_FAIL xTaskCreate\n");
    return 1;
  }
  vTaskStartScheduler();
  std::printf("SEND_SERVER_FAIL scheduler_returned\n");
  return 1;
}
