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

#include "libs/msc_ums/msc_ums.h"

#include <cstring>

#include "fsl_cache.h"
#include "libs/base/check.h"
#include "libs/base/fx_user_fs.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

extern "C" void sentai_storage_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

#define DATA_IN (0)
#define DATA_OUT (1)
#define LOGICAL_UNIT_SUPPORTED (1)

namespace coralmicro {
namespace {
/* MSC LBA geometry — matches the FileX/LevelX volume so the host sees a
 * real FAT mountable directly via `mount /dev/sda /mnt`. */
constexpr size_t kLbaSize  = FX_USER_LBA_SIZE;   /* 2016 */
constexpr int    kLbaCount = FX_USER_LBA_COUNT;  /* 28160 */

/* USB bulk URB buffer: hold an integral number of LBAs.  Four LBAs at
 * 2016 = 8064 bytes — fits one default URB. */
constexpr size_t kLbasPerUrb = 4u;
constexpr size_t kBulkBufBytes = kLbasPerUrb * kLbaSize;       /* 8064 */
}  // namespace

/* Word-aligned for USB DMA. */
uint32_t g_mscReadRequestBuffer[(kBulkBufBytes + 3u) / 4u];
uint32_t g_mscWriteRequestBuffer[(kBulkBufBytes + 3u) / 4u];

usb_device_inquiry_data_fromat_struct_t g_InquiryInfo = {
    (USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER
     << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
        USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
    (uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT
              << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
    USB_DEVICE_MSC_UFI_VERSIONS,
    0x02,
    USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
    {0x00, 0x00, 0x00},
    {'S', 'E', 'N', 'T', 'A', 'I'},
    {'F', 'L', 'A', 'S', 'H', ' ', 'S', 'T', 'O', 'R', 'A', 'G', 'E'},
    {'0', '0', '0', '1'}};
usb_device_mode_parameters_header_struct_t g_ModeParametersHeader = {
    /*refer to ufi spec mode parameter header*/
    0x0000, /*!< Mode Data Length*/
    0x00,   /*!<Default medium type (current mounted medium type)*/
    0x00,   /*!MODE SENSE command, a Write Protected bit of zero indicates the
               medium is write enabled*/
    {0x00, 0x00, 0x00, 0x00} /*!<This bit should be set to zero*/
};

std::map<class_handle_t, MscUms *> MscUms::handle_map_;

MscUms::MscUms() {}

void MscUms::Init(uint8_t bulk_in_ep, uint8_t bulk_out_ep, uint8_t data_iface) {
  bulk_in_ep_ = bulk_in_ep;
  bulk_out_ep_ = bulk_out_ep;
  msc_ums_data_endpoints_[DATA_IN].endpointAddress = bulk_in_ep | (USB_IN << 7);
  msc_ums_data_endpoints_[DATA_OUT].endpointAddress =
      bulk_out_ep | (USB_OUT << 7);
  msc_ums_interfaces_[0].interfaceNumber = data_iface;

  // Update the descriptor sent to host during enumeration.
  descriptor_.iface.interface_number = data_iface;
  descriptor_.in_ep.endpoint_address = bulk_in_ep | (USB_IN << 7);
  descriptor_.out_ep.endpoint_address = bulk_out_ep | (USB_OUT << 7);
}

void MscUms::SetClassHandle(class_handle_t class_handle) {
  handle_map_[class_handle] = this;
  class_handle_ = class_handle;
}

bool MscUms::HandleEvent(uint32_t event, void *param) {
  usb_status_t status = kStatus_USB_Success;

  switch (event) {
    case kUSB_DeviceEventSetConfiguration:
    case 10:  // kUSB_DeviceEventSetInterface — benign, ignore
      // Don't care.
      break;
    default:
      printf("%s unhandled event %ld\r\n", __PRETTY_FUNCTION__, event);
      status = kStatus_USB_Error;
  }
  return (status == kStatus_USB_Success);
}

usb_status_t MscUms::Handler(uint32_t event, void *param) {
  usb_status_t error = kStatus_USB_Success;
  usb_device_lba_information_struct_t *lbaInformation;
  usb_device_lba_app_struct_t *lba;
  usb_device_ufi_app_struct_t *ufi;
  usb_device_capacity_information_struct_t *capacityInformation;

  switch (event) {
    case kUSB_DeviceMscEventReadResponse:
      lba = (usb_device_lba_app_struct_t *)param;
      break;
    case kUSB_DeviceMscEventWriteResponse: {
      lba = (usb_device_lba_app_struct_t *)param;
      if (write_protected_) {
        error = kStatus_USB_InvalidRequest;  // CHECK CONDITION: write protected
        break;
      }
      /* The USB controller DMA-wrote into lba->buffer; invalidate cache so
       * M7 reads the fresh physical bytes when LX memcpy's into its scratch. */
      DCACHE_InvalidateByRange(reinterpret_cast<uint32_t>(lba->buffer),
                               lba->size);
      size_t size = lba->size;
      const uint8_t *buf = lba->buffer;
      uint32_t lba_idx = lba->offset;
      uint32_t lba_count = (uint32_t)(size / kLbaSize);
      sentai_storage_log("MSC W lba=%u cnt=%u", (unsigned)lba_idx,
                         (unsigned)lba_count);
      bool ok = true;
      while (size >= kLbaSize) {
        if (!FxUserMscWrite(lba_idx, buf)) {
          sentai_storage_log("MSC W FAIL lba=%u", (unsigned)lba_idx);
          ok = false;
          break;
        }
        ++lba_idx;
        buf += kLbaSize;
        size -= kLbaSize;
      }
      if (!ok) error = kStatus_USB_InvalidRequest;
      break;
    }
    case kUSB_DeviceMscEventWriteRequest:
      lba = (usb_device_lba_app_struct_t *)param;
      lba->buffer = (uint8_t *)&g_mscWriteRequestBuffer[0];
      break;
    case kUSB_DeviceMscEventReadRequest: {
      lba = (usb_device_lba_app_struct_t *)param;
      lba->buffer = (uint8_t *)&g_mscReadRequestBuffer[0];
      size_t size = lba->size;
      uint8_t *buf = lba->buffer;
      uint32_t lba_idx = lba->offset;
      uint32_t lba_count = (uint32_t)(size / kLbaSize);
      sentai_storage_log("MSC R lba=%u cnt=%u", (unsigned)lba_idx,
                         (unsigned)lba_count);
      bool ok = true;
      while (size >= kLbaSize) {
        if (!FxUserMscRead(lba_idx, buf)) {
          sentai_storage_log("MSC R FAIL lba=%u", (unsigned)lba_idx);
          ok = false;
          break;
        }
        ++lba_idx;
        buf += kLbaSize;
        size -= kLbaSize;
      }
      if (ok) {
        /* M7 stores hit cache; flush so USB DMA reads fresh physical. */
        DCACHE_CleanByRange(reinterpret_cast<uint32_t>(lba->buffer),
                            lba->size);
      } else {
        /* Zero-fill the buffer so the host gets a clean error path
         * rather than stale cache contents. */
        std::memset(lba->buffer, 0, lba->size);
        DCACHE_CleanByRange(reinterpret_cast<uint32_t>(lba->buffer),
                            lba->size);
        error = kStatus_USB_InvalidRequest;
      }
      break;
    }
    case kUSB_DeviceMscEventGetLbaInformation:
      lbaInformation = (usb_device_lba_information_struct_t *)param;
      lbaInformation->logicalUnitNumberSupported = LOGICAL_UNIT_SUPPORTED;
      lbaInformation->logicalUnitInformations[0].lengthOfEachLba = kLbaSize;
      lbaInformation->logicalUnitInformations[0].totalLbaNumberSupports =
          kLbaCount;
      lbaInformation->logicalUnitInformations[0].bulkInBufferSize =
          kBulkBufBytes;
      lbaInformation->logicalUnitInformations[0].bulkOutBufferSize =
          kBulkBufBytes;
      sentai_storage_log("MSC GetLbaInfo lba_size=%u total=%u",
                         (unsigned)kLbaSize, (unsigned)kLbaCount);
      break;
    case kUSB_DeviceMscEventTestUnitReady:
      /*change the test unit ready command's sense data if need, be careful to
       * modify*/
      ufi = (usb_device_ufi_app_struct_t *)param;
      if (!unit_ready_) {
        ufi->requestSense->senseKey = 0x02;  // NOT READY
        ufi->requestSense->additionalSenseCode = 0x3A;  // MEDIUM NOT PRESENT
        ufi->requestSense->additionalSenseQualifer = 0x00;
        error = kStatus_USB_Error;
      } else if (media_changed_) {
        media_changed_ = false;
        ufi->requestSense->senseKey = 0x06;  // UNIT ATTENTION
        ufi->requestSense->additionalSenseCode = 0x28;  // NOT READY TO READY CHANGE
        ufi->requestSense->additionalSenseQualifer = 0x00;
        error = kStatus_USB_Error;  // first TUR after insert fails with UA
      }
      break;
    case kUSB_DeviceMscEventInquiry:
      ufi = (usb_device_ufi_app_struct_t *)param;
      ufi->size = sizeof(usb_device_inquiry_data_fromat_struct_t);
      ufi->buffer = (uint8_t *)&g_InquiryInfo;
      break;
    case kUSB_DeviceMscEventModeSense:
      ufi = (usb_device_ufi_app_struct_t *)param;
      // Update WP bit (byte 2, bit 7 = write-protect) dynamically so the host
      // sees current write-protect state on every MODE SENSE poll.
      g_ModeParametersHeader.wpDpfua = write_protected_ ? 0x80 : 0x00;
      ufi->size = sizeof(usb_device_mode_parameters_header_struct_t);
      ufi->buffer = (uint8_t *)&g_ModeParametersHeader;
      break;
    case kUSB_DeviceMscEventModeSelectResponse:
      ufi = (usb_device_ufi_app_struct_t *)param;
      break;
    case kUSB_DeviceMscEventRequestSense:
      // Return success: NXP MSC class layer sends the stored sense data
      // (set by previous TUR/READ that failed with UNIT_ATTENTION etc.).
      // Returning InvalidRequest here stalls the bulk-IN endpoint, causing
      // usb-storage to do BOT-Reset → port reset → bus reset cascade.
      error = kStatus_USB_Success;
      break;
    case kUSB_DeviceMscEventModeSelect:
    case kUSB_DeviceMscEventFormatComplete:
    case kUSB_DeviceMscEventRemovalRequest:
      error = kStatus_USB_InvalidRequest;
      break;
    case kUSB_DeviceMscEventReadCapacity:
      capacityInformation = (usb_device_capacity_information_struct_t *)param;
      capacityInformation->lengthOfEachLba = kLbaSize;
      capacityInformation->totalLbaNumberSupports = kLbaCount;
      sentai_storage_log("MSC ReadCapacity lba=%u tot=%u",
                         (unsigned)kLbaSize, (unsigned)kLbaCount);
      break;
    case kUSB_DeviceMscEventReadFormatCapacity:
      capacityInformation = (usb_device_capacity_information_struct_t *)param;
      capacityInformation->lengthOfEachLba = kLbaSize;
      capacityInformation->totalLbaNumberSupports = kLbaCount;
      sentai_storage_log("MSC ReadFormatCap lba=%u tot=%u",
                         (unsigned)kLbaSize, (unsigned)kLbaCount);
      break;
    default:
      error = kStatus_USB_InvalidRequest;
      break;
  }
  return error;
}

usb_status_t MscUms::Handler(class_handle_t class_handle, uint32_t event,
                             void *param) {
  return handle_map_[class_handle]->Handler(event, param);
}

}  // namespace coralmicro
