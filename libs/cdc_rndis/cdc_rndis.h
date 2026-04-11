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

#ifndef LIBS_CDC_RNDIS_CDC_RNDIS_H_
#define LIBS_CDC_RNDIS_CDC_RNDIS_H_

#include <map>

/* clang-format off */
#include "libs/usb/descriptors.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/netifapi.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/device/usb_device.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_class.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_cdc_acm.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_cdc_rndis.h"
/* clang-format on */

namespace coralmicro {

// USB RNDIS (Remote NDIS) network device class.
// Provides USB Ethernet networking compatible with Android, Windows, and Linux.
class CdcRndis {
 public:
  CdcRndis() = default;
  CdcRndis(const CdcRndis &) = delete;
  CdcRndis &operator=(const CdcRndis &) = delete;
  void Init(uint8_t interrupt_ep, uint8_t bulk_in_ep, uint8_t bulk_out_ep,
            uint8_t comm_iface, uint8_t data_iface);
  const usb_device_class_config_struct_t &config_data() const {
    return config_;
  }
  const void *descriptor_data() const { return &mutable_descriptor_; }
  size_t descriptor_data_size() const { return sizeof(mutable_descriptor_); }
  void SetClassHandle(class_handle_t class_handle);
  bool HandleEvent(uint32_t event, void *param);

 private:
  // CDC-ACM class callback (receives RNDIS encapsulated commands + data)
  static std::map<class_handle_t, CdcRndis *> handle_map_;
  static usb_status_t StaticHandler(class_handle_t class_handle, uint32_t event,
                                    void *param) {
    return handle_map_[class_handle]->Handler(event, param);
  }
  usb_status_t Handler(uint32_t event, void *param);

  // RNDIS application callback (provides link speed, MAC, etc.)
  static usb_status_t StaticRndisCallback(class_handle_t handle,
                                          uint32_t event, void *param);

  // LwIP hooks
  static err_t StaticNetifInit(struct netif *netif) {
    return static_cast<CdcRndis *>(netif->state)->NetifInit(netif);
  }
  err_t NetifInit(struct netif *netif);

  static err_t StaticTxFunc(struct netif *netif, struct pbuf *p) {
    return static_cast<CdcRndis *>(netif->state)->TxFunc(netif, p);
  }
  err_t TxFunc(struct netif *netif, struct pbuf *p);

  static void StaticTaskFunction(void *param) {
    static_cast<CdcRndis *>(param)->TaskFunction(param);
  }
  void TaskFunction(void *param);

  err_t TransmitFrame(void *buffer, uint32_t length);
  err_t ReceiveFrame(uint8_t *buffer, uint32_t length);
  void ProcessRxPacket(uint8_t *buffer, uint32_t length);

  // Max Ethernet frame (1518) + RNDIS header (44) + margin
  static constexpr size_t kRndisBufferSize = 1600;

  // RNDIS state
  usb_device_cdc_rndis_struct_t *rndis_handle_ = nullptr;
  class_handle_t class_handle_ = nullptr;
  volatile bool attached_ = false;

  uint8_t interrupt_ep_, bulk_in_ep_, bulk_out_ep_;
  QueueHandle_t tx_queue_;

  // Buffers (aligned via alignas for DMA)
  alignas(4) uint8_t tx_buffer_[kRndisBufferSize];
  alignas(4) uint8_t rx_buffer_[kRndisBufferSize];

  ip4_addr_t netif_ipaddr_, netif_netmask_, netif_gw_;
  struct netif netif_;

  // USB class structures use CDC-ACM (RNDIS sits on top of CDC-ACM)
  usb_device_endpoint_struct_t cdc_acm_comm_endpoints_[1] = {
      {
          .endpointAddress = 0,
          .transferType = USB_ENDPOINT_INTERRUPT,
          .maxPacketSize = 16,
          .interval = 8,
      },
  };
  usb_device_endpoint_struct_t cdc_acm_data_endpoints_[2] = {
      {
          .endpointAddress = 0,
          .transferType = USB_ENDPOINT_BULK,
          .maxPacketSize = 512,
          .interval = 0,
      },
      {
          .endpointAddress = 0,
          .transferType = USB_ENDPOINT_BULK,
          .maxPacketSize = 512,
          .interval = 0,
      },
  };
  usb_device_interface_struct_t cdc_acm_comm_interface_[1] = {
      {
          .alternateSetting = 0,
          .endpointList =
              {
                  .count = ARRAY_SIZE(cdc_acm_comm_endpoints_),
                  .endpoint = cdc_acm_comm_endpoints_,
              },
          .classSpecific = nullptr,
      },
  };
  usb_device_interface_struct_t cdc_acm_data_interface_[1] = {
      {
          .alternateSetting = 0,
          .endpointList =
              {
                  .count = ARRAY_SIZE(cdc_acm_data_endpoints_),
                  .endpoint = cdc_acm_data_endpoints_,
              },
          .classSpecific = nullptr,
      },
  };
  usb_device_interfaces_struct_t cdc_acm_interfaces_[2] = {
      // Communication interface (classCode must be 0x02 for NXP CDC-ACM init;
      // the actual USB descriptor uses 0xE0/0x01/0x03 for Android compat)
      {
          .classCode = 0x02,
          .subclassCode = 0x02,
          .protocolCode = 0xFF,
          .interfaceNumber = 0,
          .interface = cdc_acm_comm_interface_,
          .count = ARRAY_SIZE(cdc_acm_comm_interface_),
      },
      // Data interface
      {
          .classCode = 0x0A,
          .subclassCode = 0x00,
          .protocolCode = 0x00,
          .interfaceNumber = 0,
          .interface = cdc_acm_data_interface_,
          .count = ARRAY_SIZE(cdc_acm_data_interface_),
      },
  };
  usb_device_interface_list_t cdc_acm_interface_list_[1] = {
      {
          .count = ARRAY_SIZE(cdc_acm_interfaces_),
          .interfaces = cdc_acm_interfaces_,
      },
  };
  usb_device_class_struct_t class_struct_{
      .interfaceList = cdc_acm_interface_list_,
      .type = kUSB_DeviceClassTypeCdc,
      .configurations = ARRAY_SIZE(cdc_acm_interface_list_),
  };
  usb_device_class_config_struct_t config_{
      .classCallback = StaticHandler,
      .classHandle = nullptr,
      .classInfomation = &class_struct_,
  };

  // USB descriptor for RNDIS (same layout as CDC-ACM but with RNDIS protocol)
  static constexpr CdcAcmClassDescriptor descriptor_ = {
      .iad0 =
          {
              .length = sizeof(InterfaceAssociationDescriptor),
              .descriptor_type = 0x0B,
              .first_interface = 0,  // Updated at Init
              .interface_count = 2,
              .function_class = 0xE0,    // Wireless Controller
              .function_subclass = 0x01,
              .function_protocol = 0x03, // RNDIS (Android-compatible)
              .interface = 0,
          },
      .cmd_iface =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,  // Updated at Init
              .alternate_setting = 0,
              .num_endpoints = 1,
              .interface_class = 0xE0,    // Wireless Controller
              .interface_subclass = 0x01,
              .interface_protocol = 0x03, // RNDIS (Android-compatible)
              .interface = 0,
          },
      .cmd_hdr_fd =
          {
              .length = sizeof(CdcHeaderFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x00,
              .cdc = 0x0110,
          },
      .cmd_mgmt_fd =
          {
              .function_length = sizeof(CdcCallManagementFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x01,
              .capabilities = 0x01,
              .data_interface = 1,  // Updated at Init
          },
      .cmd_acm_fd =
          {
              .function_length = sizeof(CdcAcmFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x02,
              .capabilities = 0x00,
          },
      .cmd_union_fd =
          {
              .function_length = sizeof(CdcUnionFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x06,
              .controller_iface = 0,   // Updated at Init
              .peripheral_iface0 = 1,  // Updated at Init
          },
      .cmd_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 | 0x80,  // Updated at Init
              .attributes = 0x03,            // Interrupt
              .max_packet_size = 16,
              .interval = 8,
          },
      .data_iface =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,  // Updated at Init
              .alternate_setting = 0,
              .num_endpoints = 2,
              .interface_class = 0x0A,  // CDC Data
              .interface_subclass = 0x00,
              .interface_protocol = 0x00,
              .interface = 0,
          },
      .in_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 | 0x80,  // Updated at Init
              .attributes = 0x02,            // Bulk
              .max_packet_size = 512,
              .interval = 0,
          },
      .out_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 & 0x7F,  // Updated at Init
              .attributes = 0x02,            // Bulk
              .max_packet_size = 512,
              .interval = 0,
          },
  };

  // Mutable copy of descriptor (endpoint/interface numbers set at Init)
  CdcAcmClassDescriptor mutable_descriptor_;
};

}  // namespace coralmicro

#endif  // LIBS_CDC_RNDIS_CDC_RNDIS_H_
