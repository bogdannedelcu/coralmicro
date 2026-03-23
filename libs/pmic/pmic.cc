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
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_gpio.h"

namespace coralmicro {
namespace {
// MCP16701 PMIC I2C address (7-bit, default from datasheet)
constexpr uint8_t kPmicAddress = 0x5B;
constexpr uint32_t kMaxTransferRetries = 10;

// CRC-8 with polynomial C(x) = x^8 + x^4 + x^3 + x^2 + 1 (0x1D), seed 0xFF.
// The CRC covers every byte sent/received on the I2C bus, including the I2C
// address byte(s), exactly as required by the MCP16701 datasheet.
uint8_t Crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (static_cast<uint8_t>(crc << 1) ^ 0x1D)
                         :  static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

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

    // Volatile Status Registers (Table 2-20, read-only or R/RoR)
    // System status
    kStsSysd = 0x290,  // nUVLO_A[7] NVM_BUSY[5] NVMWRT_NOK[4] BOOT_DONE[1] BOOT_NOK[0]
    kStsSysa = 0x291,  // TSD[5] TWR[4] STRTFAIL[1] STRTOK[0]

    // Buck channel status base — pairs of (fault, state) per channel
    // Fault:  FAULT[7] HICCUP[6] ILIM[5] ILIMNEG[4] ZCD[3] VMONINT[0]
    // State:  POK[5] OV[4] UV[3] SSDONE[2] ENS[0]
    kStsB1F  = 0x2A0,
    kStsB1S  = 0x2A1,
    kStsB2F  = 0x2A2,
    kStsB2S  = 0x2A3,
    kStsB3F  = 0x2A4,
    kStsB3S  = 0x2A5,
    kStsB4F  = 0x2A6,
    kStsB4S  = 0x2A7,
    kStsB5F  = 0x2A8,
    kStsB5S  = 0x2A9,
    kStsB6F  = 0x2AA,
    kStsB6S  = 0x2AB,
    kStsB7F  = 0x2AC,
    kStsB7S  = 0x2AD,
    kStsB8F  = 0x2AE,
    kStsB8S  = 0x2AF,

    // LDO status base — pairs of (fault, state)
    // Fault:  FAULT[7] ILIM[5]  (LCF: FAULT[7] only)
    // State:  POK[5] SSDONE[2] ENS[0]  (L4S also has SELVL4S[7])
    kStsL1F  = 0x2B0,
    kStsL1S  = 0x2B1,
    kStsL2F  = 0x2B2,
    kStsL2S  = 0x2B3,
    kStsL3F  = 0x2B4,
    kStsL3S  = 0x2B5,
    kStsL4F  = 0x2B6,
    kStsL4S  = 0x2B7,
    kStsLCF  = 0x2B8,
    kStsLCS  = 0x2B9,

    // Watchdog status (WD_CNT[7:4], WD_CLEAR[0])
    kWdCnt   = 0x2C0,

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
  const uint32_t subaddr = MakeSubaddress(reg, 1);
  const uint8_t opcode_h = (subaddr >> 8) & 0xFF;
  const uint8_t opcode_l =  subaddr       & 0xFF;

  // Read DATA byte followed by CRC byte.
  uint8_t rx[2];

  lpi2c_master_transfer_t transfer;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = kPmicAddress;
  transfer.direction      = kLPI2C_Read;
  transfer.subaddress     = subaddr;
  transfer.subaddressSize = 2;
  transfer.data           = rx;
  transfer.dataSize       = sizeof(rx);
  if (!Transfer(&transfer)) return false;

  // CRC covers: [addr|W, OPCODE_H, OPCODE_L, addr|R, DATA]
  const uint8_t crc_input[] = {
      static_cast<uint8_t>((kPmicAddress << 1) | 0),  // addr|W
      opcode_h, opcode_l,
      static_cast<uint8_t>((kPmicAddress << 1) | 1),  // addr|R
      rx[0],                                            // DATA
  };
  const uint8_t expected = Crc8(crc_input, sizeof(crc_input));
  if (rx[1] != expected) {
    printf("[PMIC] CRC mismatch Read  reg=0x%03X data=0x%02X "
           "got=0x%02X exp=0x%02X\r\n",
           reg, rx[0], rx[1], expected);
    return false;
  }

  *val = rx[0];
  return true;
}

// Enable the PMIC EN pin
// static void enable_pmic_en()
// {
// // IOMUXC_PMIC_ON_REQ_DIG_GPIO13_IO01
// #define PMIC_EN_GPIO      GPIO13
// #define PMIC_EN_PIN       1U
//   // Initialize VDD_1V8_INT_EN pin as output
//   gpio_pin_config_t pmic_pin_config = {
//       .direction = kGPIO_DigitalOutput,
//       .outputLogic = 0,
//       .interruptMode = kGPIO_NoIntmode,
//   };

//   GPIO_PinInit(PMIC_EN_GPIO, PMIC_EN_PIN, &pmic_pin_config);
//   GPIO_PinWrite(PMIC_EN_GPIO, PMIC_EN_PIN, 1);

//   printf("[PMIC] Init PMIC_EN\n");
// }

// static void set_pmic_en(bool enable)
// {
//   GPIO_PinWrite(PMIC_EN_GPIO, PMIC_EN_PIN, enable ? 1 : 0);
//   vTaskDelay(pdMS_TO_TICKS(1));
//   printf("[PMIC] Set PMIC_EN to %s\n", enable ? "enabled" : "disabled");
// }

bool PmicTask::Write(uint16_t reg, uint8_t val) {
  const uint32_t subaddr = MakeSubaddress(reg, 1);
  const uint8_t opcode_h = (subaddr >> 8) & 0xFF;
  const uint8_t opcode_l =  subaddr       & 0xFF;

  // CRC covers: [addr|W, OPCODE_H, OPCODE_L, DATA]
  const uint8_t crc_input[] = {
      static_cast<uint8_t>((kPmicAddress << 1) | 0),  // addr|W
      opcode_h, opcode_l,
      val,
  };
  const uint8_t crc = Crc8(crc_input, sizeof(crc_input));

  // Send DATA then CRC as a two-byte payload.
  uint8_t payload[2] = {val, crc};
  lpi2c_master_transfer_t transfer;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = kPmicAddress;
  transfer.direction      = kLPI2C_Write;
  transfer.subaddress     = subaddr;
  transfer.subaddressSize = 2;
  transfer.data           = payload;
  transfer.dataSize       = sizeof(payload);
  if (!Transfer(&transfer)) return false;

  // MCP16701 requires a dummy read after every write.
  // Read both the data byte and its CRC byte.
  uint8_t dummy[2] = {0x00, 0x00};
  lpi2c_master_transfer_t dummy_xfer;
  dummy_xfer.flags          = kLPI2C_TransferDefaultFlag;
  dummy_xfer.slaveAddress   = kPmicAddress;
  dummy_xfer.direction      = kLPI2C_Read;
  dummy_xfer.subaddress     = subaddr;
  dummy_xfer.subaddressSize = 2;
  dummy_xfer.data           = dummy;
  dummy_xfer.dataSize       = sizeof(dummy);
  bool ret = Transfer(&dummy_xfer);

  if (dummy[0] != payload[0]) {
    printf("[PMIC] Dummy read mismatch after Write reg=0x%03X data=0x%02X "
           "got=0x%02X\r\n",
           reg, payload[0], dummy[0]);
  }

  return ret;
}
  
bool PmicTask::Transfer(lpi2c_master_transfer_t* transfer) {
  status_t res = kStatus_Success;
  uint32_t attempts = 0;

  res = LPI2C_RTOS_Transfer(i2c_handle_, transfer);
  printf("[PMIC]: %s|0x%04X=0x%02X%02X|(s1: %ld)\n", 
      transfer->direction == kLPI2C_Read ? "Rx" : "Tx", 
      transfer->subaddress, 
      ((uint8_t*)transfer->data)[0],
      ((uint8_t*)transfer->data)[1],
      res);

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

void PmicTask::HandleStatusDump() {
  uint8_t d, a;

  // System status
  bool ok_d = Read(PmicRegisters::kStsSysd, &d);
  bool ok_a = Read(PmicRegisters::kStsSysa, &a);
  if (ok_d && ok_a) {
    printf("[PMIC] SYS D=0x%02X(nUVLO_A=%d NVM_BUSY=%d NVMWRT_NOK=%d BOOT_DONE=%d BOOT_NOK=%d)"
           " A=0x%02X(TSD=%d TWR=%d STRTFAIL=%d STRTOK=%d)\r\n",
           d, !!(d & 0x80), !!(d & 0x20), !!(d & 0x10), !!(d & 0x02), !!(d & 0x01),
           a, !!(a & 0x20), !!(a & 0x10), !!(a & 0x02), !!(a & 0x01));
  }

  // Buck channels — read all 8 pairs and print on two compact lines
  // Fault bits: FAULT[7] HICCUP[6] ILIM[5] ILIMNEG[4] ZCD[3] VMONINT[0]
  // State bits: POK[5] OV[4] UV[3] SSDONE[2] ENS[0]
  uint8_t bf[8], bs[8];
  for (int i = 0; i < 8; ++i) {
    bf[i] = 0; bs[i] = 0;
    Read(static_cast<uint16_t>(PmicRegisters::kStsB1F + i * 2), &bf[i]);
    Read(static_cast<uint16_t>(PmicRegisters::kStsB1S + i * 2), &bs[i]);
  }
  printf("[PMIC] B1:F=%02X S=%02X  B2:F=%02X S=%02X  B3:F=%02X S=%02X  B4:F=%02X S=%02X\r\n",
         bf[0], bs[0], bf[1], bs[1], bf[2], bs[2], bf[3], bs[3]);
  printf("[PMIC] B5:F=%02X S=%02X  B6:F=%02X S=%02X  B7:F=%02X S=%02X  B8:F=%02X S=%02X\r\n",
         bf[4], bs[4], bf[5], bs[5], bf[6], bs[6], bf[7], bs[7]);

  // LDO 1-4 status
  // Fault bits: FAULT[7] ILIM[5]
  // State bits: POK[5] SSDONE[2] ENS[0]
  uint8_t lf[4], ls[4];
  for (int i = 0; i < 4; ++i) {
    lf[i] = 0; ls[i] = 0;
    Read(static_cast<uint16_t>(PmicRegisters::kStsL1F + i * 2), &lf[i]);
    Read(static_cast<uint16_t>(PmicRegisters::kStsL1S + i * 2), &ls[i]);
  }
  printf("[PMIC] L1:F=%02X S=%02X  L2:F=%02X S=%02X  L3:F=%02X S=%02X  L4:F=%02X S=%02X\r\n",
         lf[0], ls[0], lf[1], ls[1], lf[2], ls[2], lf[3], ls[3]);

  // LDO Controller status and Watchdog
  uint8_t lcf = 0, lcs = 0, wdcnt = 0;
  Read(PmicRegisters::kStsLCF, &lcf);
  Read(PmicRegisters::kStsLCS, &lcs);
  Read(PmicRegisters::kWdCnt, &wdcnt);
  printf("[PMIC] LC:F=%02X S=%02X  WD_CNT=%d\r\n",
         lcf, lcs, (wdcnt >> 4) & 0x0F);
}

void PmicTask::Init(lpi2c_rtos_handle_t* i2c_handle) {
  QueueTask::Init();
  i2c_handle_ = i2c_handle;
  // enable_pmic_en();
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
    case pmic::RequestType::kStatusDump:
      HandleStatusDump();
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

void PmicTask::DumpStatus() {
  pmic::Request req;
  req.type = pmic::RequestType::kStatusDump;
  SendRequest(req);
}

}  // namespace coralmicro
