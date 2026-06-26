/*
 * Copyright 2022 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIBS_TPU_EDGETPU_DRIVER_H_
#define LIBS_TPU_EDGETPU_DRIVER_H_

#include <cstdint>
#include <vector>

#include "libs/tpu/darwinn/driver/config/beagle/beagle_chip_config.h"
#include "libs/tpu/darwinn/driver/hardware_structures.h"
#include "libs/tpu/usb_host_edgetpu.h"

namespace coralmicro {

enum class PerformanceMode {
  kLow,
  kMedium,
  kHigh,
  kMax,
};

enum class DescriptorTag {
  kUnknown = -1,
  kInstructions = 0,
  kInputActivations = 1,
  kParameters = 2,
  kOutputActivations = 3,
  kInterrupt0 = 4,
  kInterrupt1 = 5,
  kInterrupt2 = 6,
  kInterrupt3 = 7,
};

class TpuDriver {
 public:
  TpuDriver() = default;
  TpuDriver(const TpuDriver&) = delete;
  TpuDriver& operator=(const TpuDriver&) = delete;
  bool Initialize(usb_host_edgetpu_instance_t* usb_instance,
                  PerformanceMode mode);
  bool SendParameters(const uint8_t* data, uint32_t length) const;
  bool SendInputs(const uint8_t* data, uint32_t length) const;
  bool SendInstructions(const uint8_t* data, uint32_t length) const;
  bool GetOutputs(uint8_t* data, uint32_t length) const;
  // Host-mediated activation spill for wide models (BASE_ADDRESS_SCRATCH).
  // GetScratch drains an OUTFEED spill (device->host, bulk-IN, like
  // GetOutputs); SendScratch returns it on the next INFEED (host->device,
  // routed on the input-activation tag).  The host just parks the TPU's
  // working set in M7 RAM between the two halves of a split compute.
  bool GetScratch(uint8_t* data, uint32_t length) const;
  bool SendScratch(const uint8_t* data, uint32_t length) const;
  bool ReadEvent() const;
  // sentai: read + print the TPU HIB error-status + scalar-core run-status CSRs
  // over USB. Use after a failed invoke to see whether the TPU latched a
  // hardware fault (unsupported op / memory fault) and where its scalar core
  // stopped. CSR path is independent of the wedged compute pipeline.
  void DumpErrorCsrs() const;
  // sentai: read TPU error CSRs and latch them into the RAM global
  // g_sentai_tpu_fault_csr so they survive the USB-stack wedge that a failed
  // invoke causes (CDC REPL dies, but M7 RAM is JTAG-readable). Called from
  // the invoke fault path.
  void LatchErrorCsrs(uint16_t code) const;
  float GetTemperature();

 private:
  enum class RegisterSize {
    kRegSize32,
    kRegSize64,
  };

  bool BulkOutTransfer(uint8_t endpoint, const uint8_t* data,
                       uint32_t data_length,
                       DescriptorTag tag = DescriptorTag::kUnknown) const;
  ssize_t BulkOutTransferInternal(uint8_t endpoint, const uint8_t* data,
                                  uint32_t data_length,
                                  DescriptorTag tag) const;
  bool BulkInTransfer(uint8_t* data, uint32_t data_length,
                      DescriptorTag tag = DescriptorTag::kOutputActivations) const;
  ssize_t BulkInTransferInternal(uint8_t endpoint, uint8_t* data,
                                 uint32_t data_length,
                                 DescriptorTag tag) const;

  bool SendData(DescriptorTag tag, const uint8_t* data, uint32_t length) const;
  bool WriteHeader(DescriptorTag tag, uint32_t length,
                   uint8_t endpoint) const;
  std::vector<uint8_t> PrepareHeader(DescriptorTag tag, uint32_t length) const;
  // No-heap variant: writes 8 bytes into caller-supplied buffer.
  // Used on hot paths (WriteHeader, SendInputs) to avoid per-invoke
  // std::vector alloc/free churn.  Static helper — no instance state.
  static void PrepareHeaderInto(DescriptorTag tag, uint32_t length,
                                uint8_t out[8]);

  bool CSRTransfer(uint64_t reg, void* data, bool read, RegisterSize reg_size);
  bool Read32(uint64_t reg, uint32_t* val);
  bool Read64(uint64_t reg, uint64_t* val);
  bool Write32(uint64_t reg, uint32_t val);
  bool Write64(uint64_t reg, uint64_t val);
  bool DoRunControl(platforms::darwinn::driver::RunControl run_state);

  platforms::darwinn::driver::config::BeagleChipConfig chip_config_;
  usb_host_edgetpu_instance_t* usb_instance_ = nullptr;
};

}  // namespace coralmicro

#endif  // LIBS_TPU_EDGETPU_DRIVER_H_
