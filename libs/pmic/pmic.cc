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

#include "libs/pmic/pmic.h"

#include <cstdio>

#include "libs/base/check.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

namespace coralmicro {
namespace {
// MCP16701 PMIC I2C address (7-bit, default from datasheet)
constexpr uint8_t kPmicAddress = 0x5B;
constexpr uint32_t kMaxTransferRetries = 10;

// MCP16701 register addresses (volatile config space at 0x200 offset from NVM)
struct PmicRegisters {
  enum : uint16_t {
    // Device identification
    kDeviceId = 0x001,

    // Volatile LDO config registers (CFG1 — bit 0 = ENABLE)
    kLdo1Cfg1 = 0x259,  // VDD_1V8_CAM2
    kLdo2Cfg1 = 0x260,  // VDD_2V8_CAM2
    kLdo3Cfg1 = 0x267,  // VDD_2V8_CAM1
    kLdo4Cfg1 = 0x26E,  // VDD_1V8_CAM1

    kUnknown = 0xFFF,
  };
};
}  // namespace

// MCP16701 I2C subaddress encoding:
//   OPCODE_H = [N5:N0, A9, A8]  (byte count + upper address bits)
//   OPCODE_L = [A7:A0]          (lower address bits)
// The NXP I2C driver sends high byte first when subaddressSize=2.
uint32_t PmicTask::MakeSubaddress(uint16_t reg, uint8_t byte_count) {
  uint8_t opcode_h = (byte_count << 2) | ((reg >> 8) & 0x3);
  uint8_t opcode_l = reg & 0xFF;
  return (static_cast<uint32_t>(opcode_h) << 8) | opcode_l;
}

bool PmicTask::Read(uint16_t reg, uint8_t* val) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = kPmicAddress;
  transfer.direction = kLPI2C_Read;
  transfer.subaddress = MakeSubaddress(reg, 1);
  transfer.subaddressSize = 2;
  transfer.data = val;
  transfer.dataSize = sizeof(*val);
  return Transfer(&transfer);
}

bool PmicTask::Write(uint16_t reg, uint8_t val) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = kPmicAddress;
  transfer.direction = kLPI2C_Write;
  transfer.subaddress = MakeSubaddress(reg, 1);
  transfer.subaddressSize = 2;
  transfer.data = &val;
  transfer.dataSize = sizeof(val);
  if (!Transfer(&transfer)) return false;

  // MCP16701 requires a dummy read after every write
  uint8_t dummy;
  lpi2c_master_transfer_t dummy_transfer;
  dummy_transfer.flags = kLPI2C_TransferDefaultFlag;
  dummy_transfer.slaveAddress = kPmicAddress;
  dummy_transfer.direction = kLPI2C_Read;
  dummy_transfer.subaddress = MakeSubaddress(reg, 1);
  dummy_transfer.subaddressSize = 2;
  dummy_transfer.data = &dummy;
  dummy_transfer.dataSize = sizeof(dummy);
  return Transfer(&dummy_transfer);
}
  
bool PmicTask::Transfer(lpi2c_master_transfer_t* transfer) {
  status_t res = kStatus_Success;
  uint32_t attempts = 0;

  res = LPI2C_RTOS_Transfer(i2c_handle_, transfer);
  printf("%s|0x%04X=0x%02X|(s1: %ld)\n", 
      transfer->direction == kLPI2C_Read ? "Rx" : "Tx", 
      transfer->subaddress, *(uint8_t*)transfer->data, res);

  // do {
  //   if (res == kStatus_LPI2C_Busy) {
  //     taskYIELD();
  //   } else if (res == kStatus_LPI2C_ArbitrationLost) {
  //     attempts++;
  //     if (attempts >= kMaxTransferRetries) {
  //       break;
  //     } else {
  //       // Retry right away.
  //     }
  //   }
  //   res = LPI2C_RTOS_Transfer(i2c_handle_, transfer);
  // } while ((res == kStatus_LPI2C_Busy) ||
  //          (res == kStatus_LPI2C_ArbitrationLost));

  return res == kStatus_Success;
}

void PmicTask::Init(lpi2c_rtos_handle_t* i2c_handle) {
  QueueTask::Init();
  i2c_handle_ = i2c_handle;
}

void PmicTask::HandleRailRequest(const pmic::RailRequest& rail) {
  auto reg = PmicRegisters::kUnknown;
  uint8_t val;
  switch (rail.rail) {
    case PmicRail::kCam2_2V8:
      reg = PmicRegisters::kLdo2Cfg1;
      break;
    case PmicRail::kCam1_2V8:
      reg = PmicRegisters::kLdo3Cfg1;
      break;
    case PmicRail::kCam2_1V8:
      reg = PmicRegisters::kLdo1Cfg1;
      break;
    case PmicRail::kCam1_1V8:
      reg = PmicRegisters::kLdo4Cfg1;
      break;
  }
  CHECK(Read(reg, &val));
  if (rail.enable) {
    val |= 1;
  } else {
    val &= ~1;
  }
  CHECK(Write(reg, val));
}

uint8_t PmicTask::HandleChipIdRequest() {
  uint8_t device_id = 0xff;
  CHECK(Read(PmicRegisters::kDeviceId, &device_id));
  return device_id;
}

void PmicTask::RequestHandler(pmic::Request* req) {
  pmic::Response resp;
  resp.type = req->type;
  switch (req->type) {
    case pmic::RequestType::kRail:
      HandleRailRequest(req->request.rail);
      break;
    case pmic::RequestType::kChipId:
      resp.response.chip_id = HandleChipIdRequest();
      break;
  }
  if (req->callback) req->callback(resp);
}

void PmicTask::SetRailState(PmicRail rail, bool enable) {
  pmic::Request req;
  req.type = pmic::RequestType::kRail;
  req.request.rail.rail = rail;
  req.request.rail.enable = enable;
  SendRequest(req);
}

uint8_t PmicTask::GetChipId() {
  pmic::Request req;
  req.type = pmic::RequestType::kChipId;
  pmic::Response resp = SendRequest(req);
  return resp.response.chip_id;
}

}  // namespace coralmicro
