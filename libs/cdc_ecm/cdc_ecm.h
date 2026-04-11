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

#ifndef LIBS_CDC_ECM_CDC_ECM_H_
#define LIBS_CDC_ECM_CDC_ECM_H_

#include <map>

/* clang-format off */
#include "libs/usb/descriptors.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/netifapi.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/device/usb_device.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_class.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_cdc_acm.h"
/* clang-format on */

// CDC-ECM SET_ETHERNET_PACKET_FILTER bRequest code
#define USB_CDC_SET_ETHERNET_PACKET_FILTER 0x43U
// Event code forwarded by our modified CDC-ACM default handler: 0x100 + bRequest
#define kUSB_DeviceCdcEventEcmSetPacketFilter (0x100U + USB_CDC_SET_ETHERNET_PACKET_FILTER)

namespace coralmicro {

// USB CDC-ECM (Ethernet Control Model) network device class.
// Provides USB Ethernet networking compatible with Android, iOS, macOS, Linux.
class CdcEcm {
 public:
  CdcEcm() = default;
  CdcEcm(const CdcEcm &) = delete;
  CdcEcm &operator=(const CdcEcm &) = delete;
  void Init(uint8_t interrupt_ep, uint8_t bulk_in_ep, uint8_t bulk_out_ep,
            uint8_t comm_iface, uint8_t data_iface);
  const usb_device_class_config_struct_t &config_data() const {
    return config_;
  }
  const void *descriptor_data() const { return &mutable_descriptor_; }
  size_t descriptor_data_size() const { return sizeof(mutable_descriptor_); }
  void SetClassHandle(class_handle_t class_handle) {
    handle_map_[class_handle] = this;
    class_handle_ = class_handle;
  }
  bool HandleEvent(uint32_t event, void *param);

 private:
  static std::map<class_handle_t, CdcEcm *> handle_map_;
  static usb_status_t StaticHandler(class_handle_t class_handle, uint32_t event,
                                    void *param) {
    return handle_map_[class_handle]->Handler(event, param);
  }
  usb_status_t Handler(uint32_t event, void *param);

  static err_t StaticNetifInit(struct netif *netif) {
    return static_cast<CdcEcm *>(netif->state)->NetifInit(netif);
  }
  err_t NetifInit(struct netif *netif);

  static err_t StaticTxFunc(struct netif *netif, struct pbuf *p) {
    return static_cast<CdcEcm *>(netif->state)->TxFunc(netif, p);
  }
  err_t TxFunc(struct netif *netif, struct pbuf *p);

  static void StaticTaskFunction(void *param) {
    static_cast<CdcEcm *>(param)->TaskFunction(param);
  }
  void TaskFunction(void *param);

  err_t TransmitFrame(void *buffer, uint32_t length);
  err_t ReceiveFrame(uint8_t *buffer, uint32_t length);

  // Max Ethernet frame size
  static constexpr size_t kEcmBufferSize = 1536;

  class_handle_t class_handle_ = nullptr;
  volatile bool attached_ = false;

  uint8_t interrupt_ep_, bulk_in_ep_, bulk_out_ep_;
  QueueHandle_t tx_queue_;

  alignas(4) uint8_t tx_buffer_[kEcmBufferSize];
  alignas(4) uint8_t rx_buffer_[kEcmBufferSize];

  ip4_addr_t netif_ipaddr_, netif_netmask_, netif_gw_;
  struct netif netif_;

  // CDC-ACM transport (NXP driver handles endpoint/interface management)
  usb_device_endpoint_struct_t cdc_acm_comm_endpoints_[1] = {
      {
          .endpointAddress = 0,
          .transferType = USB_ENDPOINT_INTERRUPT,
          .maxPacketSize = 16,
          .interval = 32,
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
      // Communication interface (classCode 0x02 for NXP CDC-ACM init)
      {
          .classCode = 0x02,
          .subclassCode = 0x06,  // ECM
          .protocolCode = 0x00,
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

  // CDC-ECM USB descriptor
  static constexpr CdcEcmClassDescriptor descriptor_ = {
      .iad0 =
          {
              .length = sizeof(InterfaceAssociationDescriptor),
              .descriptor_type = 0x0B,
              .first_interface = 0,  // Updated at Init
              .interface_count = 2,
              .function_class = 0x02,    // CDC
              .function_subclass = 0x06, // ECM
              .function_protocol = 0x00,
              .interface = 0,
          },
      .cmd_iface =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,  // Updated at Init
              .alternate_setting = 0,
              .num_endpoints = 1,
              .interface_class = 0x02,    // CDC
              .interface_subclass = 0x06, // ECM
              .interface_protocol = 0x00,
              .interface = 0,
          },
      .cmd_hdr_fd =
          {
              .length = sizeof(CdcHeaderFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x00,
              .cdc = 0x0120,
          },
      .cmd_union_fd =
          {
              .function_length = sizeof(CdcUnionFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x06,
              .controller_iface = 0,   // Updated at Init
              .peripheral_iface0 = 1,  // Updated at Init
          },
      .cmd_enet_fd =
          {
              .function_length = sizeof(CdcEthernetFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x0F,  // Ethernet Networking
              .mac_address = 4,            // String descriptor index 4
              .ethernet_statistics = 0,
              .max_segment_size = 1514,
              .number_mc_filters = 0,
              .number_power_filters = 0,
          },
      .cmd_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 | 0x80,  // Updated at Init
              .attributes = 0x03,            // Interrupt
              .max_packet_size = 16,
              .interval = 32,
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
  CdcEcmClassDescriptor mutable_descriptor_;
};

}  // namespace coralmicro

#endif  // LIBS_CDC_ECM_CDC_ECM_H_
