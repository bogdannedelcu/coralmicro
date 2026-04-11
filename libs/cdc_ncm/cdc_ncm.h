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

#ifndef LIBS_CDC_NCM_CDC_NCM_H_
#define LIBS_CDC_NCM_CDC_NCM_H_

#include <map>

/* clang-format off */
#include "libs/usb/descriptors.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/netifapi.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/device/usb_device.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_class.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/output/source/device/class/usb_device_cdc_acm.h"
/* clang-format on */

// NCM class-specific requests
#define USB_CDC_GET_NTB_PARAMETERS    0x80U
#define USB_CDC_SET_NTB_INPUT_SIZE    0x86U
#define USB_CDC_GET_NTB_INPUT_SIZE    0x85U
#define USB_CDC_SET_NTB_FORMAT        0x84U
#define USB_CDC_GET_NTB_FORMAT        0x83U

// Event codes from our patched CDC-ACM: 0x100 + bRequest
#define kUSB_DeviceCdcEventNcmGetNtbParameters  (0x100U + USB_CDC_GET_NTB_PARAMETERS)
#define kUSB_DeviceCdcEventNcmSetNtbInputSize   (0x100U + USB_CDC_SET_NTB_INPUT_SIZE)
#define kUSB_DeviceCdcEventNcmGetNtbInputSize   (0x100U + USB_CDC_GET_NTB_INPUT_SIZE)
#define kUSB_DeviceCdcEventNcmSetNtbFormat      (0x100U + USB_CDC_SET_NTB_FORMAT)
#define kUSB_DeviceCdcEventNcmGetNtbFormat      (0x100U + USB_CDC_GET_NTB_FORMAT)
#define kUSB_DeviceCdcEventEcmSetPacketFilter   (0x100U + 0x43U)

// NTH16 (NCM Transfer Header, 16-bit)
struct NcmNth16 {
  uint32_t signature;       // "NCMH" = 0x484D434E
  uint16_t header_length;   // 12
  uint16_t sequence;
  uint16_t block_length;    // Total NTB size
  uint16_t ndp_index;       // Offset to first NDP16
} __attribute__((packed));

// NDP16 (NCM Datagram Pointer, 16-bit)
struct NcmNdp16Header {
  uint32_t signature;       // "NCM0" = 0x304D434E
  uint16_t length;          // Size of this NDP in bytes
  uint16_t next_ndp_index;  // 0 = last NDP
} __attribute__((packed));

struct NcmNdp16Datagram {
  uint16_t datagram_index;  // Offset from NTB start
  uint16_t datagram_length; // 0/0 = terminator
} __attribute__((packed));

// NTB Parameters structure returned by GET_NTB_PARAMETERS
struct NcmNtbParameters {
  uint16_t length;
  uint16_t ntb_formats_supported;  // bit 0 = NTB-16
  uint32_t ntb_in_max_size;
  uint16_t ndp_in_divisor;
  uint16_t ndp_in_payload_remainder;
  uint16_t ndp_in_alignment;
  uint16_t reserved0;
  uint32_t ntb_out_max_size;
  uint16_t ndp_out_divisor;
  uint16_t ndp_out_payload_remainder;
  uint16_t ndp_out_alignment;
  uint16_t ntb_out_max_datagrams;
} __attribute__((packed));

namespace coralmicro {

class CdcNcm {
 public:
  CdcNcm() = default;
  CdcNcm(const CdcNcm &) = delete;
  CdcNcm &operator=(const CdcNcm &) = delete;
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
  static std::map<class_handle_t, CdcNcm *> handle_map_;
  static usb_status_t StaticHandler(class_handle_t class_handle, uint32_t event,
                                    void *param) {
    return handle_map_[class_handle]->Handler(event, param);
  }
  usb_status_t Handler(uint32_t event, void *param);
  void SendConnectionNotification();
  void SendSpeedChangeNotification();

  static err_t StaticNetifInit(struct netif *netif) {
    return static_cast<CdcNcm *>(netif->state)->NetifInit(netif);
  }
  err_t NetifInit(struct netif *netif);

  static err_t StaticTxFunc(struct netif *netif, struct pbuf *p) {
    return static_cast<CdcNcm *>(netif->state)->TxFunc(netif, p);
  }
  err_t TxFunc(struct netif *netif, struct pbuf *p);

  static void StaticTaskFunction(void *param) {
    static_cast<CdcNcm *>(param)->TaskFunction(param);
  }
  void TaskFunction(void *param);

  err_t TransmitFrame(void *buffer, uint32_t length);
  void ProcessRxNtb(uint8_t *buffer, uint32_t length);

  // NTB sizes — tuned to fit in DTCM (m_data).
  // Single datagram per NTB: NTH16(12) + frame(<=1514) + pad(<=3) + NDP16(16) = ~1545
  static constexpr uint32_t kNtbMaxSize = 1600;
  // Max Ethernet frame
  static constexpr size_t kMaxFrameSize = 1514;

  class_handle_t class_handle_ = nullptr;
  volatile bool attached_ = false;
  uint8_t data_iface_ = 0;

  uint8_t interrupt_ep_, bulk_in_ep_, bulk_out_ep_;
  QueueHandle_t tx_queue_;
  uint16_t tx_sequence_ = 0;

  alignas(4) uint8_t tx_buffer_[kNtbMaxSize];
  alignas(4) uint8_t rx_buffer_[kNtbMaxSize];

  // CDC notification buffers (sent on interrupt endpoint)
  // NETWORK_CONNECTION: 8 bytes setup header
  alignas(4) uint8_t notify_network_conn_[8];
  // CONNECTION_SPEED_CHANGE: 8 bytes setup + 8 bytes speed data
  alignas(4) uint8_t notify_speed_change_[16];
  uint8_t comm_iface_ = 0;

  // NTB parameters reported to host
  static constexpr NcmNtbParameters ntb_params_ = {
      .length = sizeof(NcmNtbParameters),
      .ntb_formats_supported = 0x0001,  // NTB-16 only
      .ntb_in_max_size = kNtbMaxSize,
      .ndp_in_divisor = 4,
      .ndp_in_payload_remainder = 0,
      .ndp_in_alignment = 4,
      .reserved0 = 0,
      .ntb_out_max_size = kNtbMaxSize,
      .ndp_out_divisor = 4,
      .ndp_out_payload_remainder = 0,
      .ndp_out_alignment = 4,
      .ntb_out_max_datagrams = 1,
  };

  ip4_addr_t netif_ipaddr_, netif_netmask_, netif_gw_;
  struct netif netif_;

  // NXP CDC-ACM transport structures
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
  usb_device_interface_struct_t cdc_acm_data_interface_[2] = {
      // Alt setting 0 — no endpoints
      {
          .alternateSetting = 0,
          .endpointList =
              {
                  .count = 0,
                  .endpoint = nullptr,
              },
          .classSpecific = nullptr,
      },
      // Alt setting 1 — bulk endpoints active
      {
          .alternateSetting = 1,
          .endpointList =
              {
                  .count = ARRAY_SIZE(cdc_acm_data_endpoints_),
                  .endpoint = cdc_acm_data_endpoints_,
              },
          .classSpecific = nullptr,
      },
  };
  usb_device_interfaces_struct_t cdc_acm_interfaces_[2] = {
      // Communication interface
      {
          .classCode = 0x02,
          .subclassCode = 0x0D,  // NCM
          .protocolCode = 0x00,
          .interfaceNumber = 0,
          .interface = cdc_acm_comm_interface_,
          .count = ARRAY_SIZE(cdc_acm_comm_interface_),
      },
      // Data interface (2 alt settings)
      {
          .classCode = 0x0A,
          .subclassCode = 0x00,
          .protocolCode = 0x01,  // NCM
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

  // CDC-NCM USB descriptor
  static constexpr CdcNcmClassDescriptor descriptor_ = {
      .iad0 =
          {
              .length = sizeof(InterfaceAssociationDescriptor),
              .descriptor_type = 0x0B,
              .first_interface = 0,
              .interface_count = 2,
              .function_class = 0x02,    // CDC
              .function_subclass = 0x0D, // NCM
              .function_protocol = 0x00,
              .interface = 0,
          },
      .cmd_iface =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,
              .alternate_setting = 0,
              .num_endpoints = 1,
              .interface_class = 0x02,    // CDC
              .interface_subclass = 0x0D, // NCM
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
              .controller_iface = 0,
              .peripheral_iface0 = 1,
          },
      .cmd_enet_fd =
          {
              .function_length = sizeof(CdcEthernetFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x0F,
              .mac_address = 4,  // String descriptor index 4
              .ethernet_statistics = 0,
              .max_segment_size = 1514,
              .number_mc_filters = 0,
              .number_power_filters = 0,
          },
      .cmd_ncm_fd =
          {
              .function_length = sizeof(CdcNcmFunctionalDescriptor),
              .descriptor_type = 0x24,
              .descriptor_subtype = 0x1A,
              .bcd_ncm = 0x0100,
              .network_capabilities = 0x00,
          },
      .cmd_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 | 0x80,
              .attributes = 0x03,  // Interrupt
              .max_packet_size = 16,
              .interval = 32,
          },
      // Data interface alt setting 0 — no endpoints
      .data_iface_alt0 =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,
              .alternate_setting = 0,
              .num_endpoints = 0,
              .interface_class = 0x0A,
              .interface_subclass = 0x00,
              .interface_protocol = 0x01,  // NCM
              .interface = 0,
          },
      // Data interface alt setting 1 — with endpoints
      .data_iface_alt1 =
          {
              .length = sizeof(InterfaceDescriptor),
              .descriptor_type = 0x04,
              .interface_number = 0,
              .alternate_setting = 1,
              .num_endpoints = 2,
              .interface_class = 0x0A,
              .interface_subclass = 0x00,
              .interface_protocol = 0x01,  // NCM
              .interface = 0,
          },
      .in_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 | 0x80,
              .attributes = 0x02,  // Bulk
              .max_packet_size = 512,
              .interval = 0,
          },
      .out_ep =
          {
              .length = sizeof(EndpointDescriptor),
              .descriptor_type = 0x05,
              .endpoint_address = 0 & 0x7F,
              .attributes = 0x02,  // Bulk
              .max_packet_size = 512,
              .interval = 0,
          },
  };

  CdcNcmClassDescriptor mutable_descriptor_;
};

}  // namespace coralmicro

#endif  // LIBS_CDC_NCM_CDC_NCM_H_
