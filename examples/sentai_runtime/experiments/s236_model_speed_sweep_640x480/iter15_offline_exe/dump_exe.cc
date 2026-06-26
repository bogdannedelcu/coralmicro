// Offline darwinn-executable dumper. Extracts the EdgeTPU executable from a
// .tflite (edgetpu-custom-op flexbuffer -> Package -> MultiExecutable ->
// Executables) and prints the structure: type, parameter_caching_token,
// instruction bitstreams (count + sizes), output layers (size_bytes, dims,
// execution_count), scratch. Used to compare why c2f_thick fails on board
// while c2f_deep works, and whether instructions can be cached (reused).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <fstream>

#include "third_party/flatbuffers/include/flatbuffers/flatbuffers.h"
#include "third_party/flatbuffers/include/flatbuffers/flexbuffers.h"
#include "third_party/tflite-micro/tensorflow/lite/schema/schema_generated.h"
#include "libs/tpu/executable_generated.h"

namespace darwinn = platforms::darwinn;

static std::vector<uint8_t> ReadFile(const char* path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  std::streamsize n = f.tellg();
  f.seekg(0);
  std::vector<uint8_t> buf(n);
  f.read(reinterpret_cast<char*>(buf.data()), n);
  return buf;
}

static const char* ExeTypeName(int t) {
  switch (t) {
    case darwinn::ExecutableType_STAND_ALONE: return "STAND_ALONE";
    case darwinn::ExecutableType_EXECUTION_ONLY: return "EXECUTION_ONLY";
    case darwinn::ExecutableType_PARAMETER_CACHING: return "PARAMETER_CACHING";
    default: return "?";
  }
}

static void DumpExecutable(const darwinn::Executable* e, const char* tag) {
  printf("  --- executable [%s] type=%s ---\n", tag, ExeTypeName((int)e->type()));
  uint64_t tok = e->parameter_caching_token();
  printf("    parameter_caching_token = 0x%016llx\n", (unsigned long long)tok);
  printf("    scratch_size_bytes      = %d\n", e->scratch_size_bytes());

  auto* instr = e->instruction_bitstreams();
  if (instr) {
    printf("    instruction_bitstreams: %u chunk(s)\n",
           flatbuffers::VectorLength(instr));
    uint32_t total = 0;
    for (uint32_t i = 0; i < flatbuffers::VectorLength(instr); ++i) {
      auto* bs = instr->Get(i)->bitstream();
      uint32_t len = bs ? flatbuffers::VectorLength(bs) : 0;
      total += len;
      printf("      [%u] %u bytes\n", i, len);
    }
    printf("    instruction TOTAL = %u bytes\n", total);
  } else {
    printf("    instruction_bitstreams: none\n");
  }

  // dma_hints sequence (this is what Invoke iterates; shows SCRATCH/Fence)
  auto* hints = e->dma_hints();
  if (hints && hints->hints()) {
    auto* hv = hints->hints();
    printf("    dma_hints: %u\n", flatbuffers::VectorLength(hv));
    const char* descN[] = {"OUTPUT", "INPUT", "PARAMETER", "SCRATCH"};
    for (uint32_t i = 0; i < flatbuffers::VectorLength(hv); ++i) {
      const darwinn::DmaHint* h = hv->Get(i);
      switch (h->any_hint_type()) {
        case darwinn::AnyHint_DmaDescriptorHint: {
          auto* d = h->any_hint_as_DmaDescriptorHint();
          int desc = (int)d->meta()->desc();
          const char* nm = d->meta()->name() ? d->meta()->name()->c_str() : "";
          printf("      [%2u] DMA  %-9s dir=%-7s size=%-8d off=%-8d %s\n", i,
                 (desc >= 0 && desc < 4) ? descN[desc] : "?",
                 h->direction() == darwinn::Direction_INFEED ? "INFEED" : "OUTFEED",
                 d->size_in_bytes(), d->offset_in_bytes(), nm);
          break;
        }
        case darwinn::AnyHint_InstructionHint:
          printf("      [%2u] INSTR chunk=%d\n", i,
                 h->any_hint_as_InstructionHint()->instruction_chunk_index());
          break;
        case darwinn::AnyHint_InterruptHint:
          printf("      [%2u] INTERRUPT\n", i); break;
        case darwinn::AnyHint_FenceHint:
          printf("      [%2u] FENCE\n", i); break;
        default: printf("      [%2u] (none)\n", i);
      }
    }
  }

  auto* outs = e->output_layers();
  if (outs) {
    printf("    output_layers: %u\n", flatbuffers::VectorLength(outs));
    for (uint32_t i = 0; i < flatbuffers::VectorLength(outs); ++i) {
      const darwinn::Layer* L = outs->Get(i);
      const char* nm = L->name() ? L->name()->c_str() : "(noname)";
      printf("      [%u] %-28s size_bytes=%d  x=%d y=%d z=%d  exec_count=%d  "
             "padded=%d  actual~=%d\n",
             i, nm, L->size_bytes(), L->x_dim(), L->y_dim(), L->z_dim(),
             L->execution_count_per_inference(),
             L->size_bytes() * L->execution_count_per_inference(),
             L->x_dim() * L->y_dim() * L->z_dim());
    }
  }
}

static void DumpTflite(const char* path) {
  printf("======================= %s =======================\n", path);
  std::vector<uint8_t> tfl = ReadFile(path);
  const tflite::Model* model = tflite::GetModel(tfl.data());
  if (!model) { printf("  not a tflite model\n"); return; }

  // find the edgetpu-custom-op operator and its custom_options (flexbuffer)
  const flatbuffers::Vector<uint8_t>* custom_opts = nullptr;
  auto* sgs = model->subgraphs();
  for (uint32_t s = 0; s < flatbuffers::VectorLength(sgs) && !custom_opts; ++s) {
    auto* ops = sgs->Get(s)->operators();
    for (uint32_t o = 0; o < flatbuffers::VectorLength(ops); ++o) {
      auto* op = ops->Get(o);
      auto* oc = model->operator_codes()->Get(op->opcode_index());
      const char* cc = oc->custom_code() ? oc->custom_code()->c_str() : "";
      if (cc && strstr(cc, "edgetpu")) { custom_opts = op->custom_options(); break; }
    }
  }
  if (!custom_opts) { printf("  no edgetpu-custom-op found\n"); return; }

  // flexbuffer map -> key "4" (kKeyExecutable) -> Package binary
  auto root = flexbuffers::GetRoot(custom_opts->data(),
                                   flatbuffers::VectorLength(custom_opts));
  auto map = root.AsMap();
  auto pkg_str = map["4"].AsString();
  const darwinn::Package* pkg =
      flatbuffers::GetRoot<darwinn::Package>(pkg_str.c_str());
  auto* sme = pkg->serialized_multi_executable();
  if (!sme || flatbuffers::VectorLength(sme) == 0) {
    printf("  no serialized_multi_executable\n"); return;
  }
  const darwinn::MultiExecutable* multi =
      flatbuffers::GetRoot<darwinn::MultiExecutable>(sme->data());
  auto* execs = multi->serialized_executables();
  printf("  multi_executable: %u executable(s)\n",
         flatbuffers::VectorLength(execs));
  for (uint32_t i = 0; i < flatbuffers::VectorLength(execs); ++i) {
    auto* es = execs->Get(i);
    const darwinn::Executable* e =
        flatbuffers::GetRoot<darwinn::Executable>(
            (const uint8_t*)es->c_str());
    char tag[16]; snprintf(tag, sizeof(tag), "exe%u", i);
    DumpExecutable(e, tag);
  }
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) DumpTflite(argv[i]);
  return 0;
}
