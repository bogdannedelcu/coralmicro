// sentai_edgetpu_physical_probe.cc -- host-side physical Coral USB probe.
//
// This tool intentionally does not use PyCoral.  It calls libedgetpu's public
// C++ API directly to enumerate and open a real USB Edge TPU on the host.

#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "edgetpu.h"

namespace {

const char* DeviceTypeName(edgetpu::DeviceType type) {
  switch (type) {
    case edgetpu::DeviceType::kApexPci:
      return "pci";
    case edgetpu::DeviceType::kApexUsb:
      return "usb";
  }
  return "unknown";
}

std::string JsonEscape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

void PrintOptions(const edgetpu::EdgeTpuManager::DeviceOptions& opts) {
  bool first = true;
  std::cout << "{";
  for (const auto& kv : opts) {
    if (!first) std::cout << ",";
    first = false;
    std::cout << "\"" << JsonEscape(kv.first) << "\":\""
              << JsonEscape(kv.second) << "\"";
  }
  std::cout << "}";
}

}  // namespace

int main(int argc, char** argv) {
  bool open_device = false;
  bool force_dfu = false;
  std::string performance = "Low";
  std::string path;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--open") {
      open_device = true;
    } else if (arg == "--force-dfu") {
      force_dfu = true;
    } else if (arg == "--performance" && i + 1 < argc) {
      performance = argv[++i];
    } else if (arg == "--path" && i + 1 < argc) {
      path = argv[++i];
    } else {
      std::cerr << "usage: " << argv[0]
                << " [--open] [--path SYSFS_PATH] [--performance Low|Medium|High|Max]"
                << " [--force-dfu]\n";
      return 2;
    }
  }

  edgetpu::EdgeTpuManager* manager = edgetpu::EdgeTpuManager::GetSingleton();
  if (!manager) {
    std::cout << "{\"ok\":false,\"error\":\"manager_null\"}\n";
    return 1;
  }

  auto devices = manager->EnumerateEdgeTpu();
  std::cout << "{\"ok\":true,\"version\":\"" << JsonEscape(manager->Version())
            << "\",\"device_count\":" << devices.size() << ",\"devices\":[";
  for (size_t i = 0; i < devices.size(); ++i) {
    if (i) std::cout << ",";
    std::cout << "{\"type\":\"" << DeviceTypeName(devices[i].type)
              << "\",\"path\":\"" << JsonEscape(devices[i].path) << "\"}";
  }
  std::cout << "]";

  if (open_device) {
    edgetpu::EdgeTpuManager::DeviceOptions options;
    options["Performance"] = performance;
    options["Usb.AlwaysDfu"] = force_dfu ? "True" : "False";
    options["Usb.MaxBulkInQueueLength"] = "4";

    std::shared_ptr<edgetpu::EdgeTpuContext> ctx =
        manager->OpenDevice(edgetpu::DeviceType::kApexUsb, path, options);
    if (!ctx) {
      std::cout << ",\"open_ok\":false,\"open_error\":\"context_null\"";
    } else {
      const auto& rec = ctx->GetDeviceEnumRecord();
      std::cout << ",\"open_ok\":true,\"open_type\":\""
                << DeviceTypeName(rec.type) << "\",\"open_path\":\""
                << JsonEscape(rec.path) << "\",\"ready\":"
                << (ctx->IsReady() ? "true" : "false")
                << ",\"device_options\":";
      PrintOptions(ctx->GetDeviceOptions());
    }
  }

  std::cout << "}\n";
  return 0;
}
