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

#include "libs/cdc_ncm/cdc_ncm.h"

#include <cstring>
#include <vector>

#include "libs/base/check.h"
#include "libs/base/tasks.h"
#include "libs/base/utils.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/etharp.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/netif/ethernet.h"

extern "C" void start_dhcp_server(uint32_t local_addr);

#define DATA_OUT (1)
#define DATA_IN (0)

#define NTH16_SIGNATURE 0x484D434EU  // "NCMH"
#define NDP16_SIGNATURE 0x304D434EU  // "NCM0"

namespace coralmicro {

std::map<class_handle_t, CdcNcm *> CdcNcm::handle_map_;

void CdcNcm::Init(uint8_t interrupt_ep, uint8_t bulk_in_ep,
                   uint8_t bulk_out_ep, uint8_t comm_iface,
                   uint8_t data_iface) {
  interrupt_ep_ = interrupt_ep;
  bulk_in_ep_ = bulk_in_ep;
  bulk_out_ep_ = bulk_out_ep;
  data_iface_ = data_iface;
  comm_iface_ = comm_iface;

  // Set endpoint addresses
  cdc_acm_comm_endpoints_[0].endpointAddress = interrupt_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_IN].endpointAddress =
      bulk_in_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_OUT].endpointAddress =
      bulk_out_ep | (USB_OUT << 7);

  // Set interface numbers
  cdc_acm_interfaces_[0].interfaceNumber = comm_iface;
  cdc_acm_interfaces_[1].interfaceNumber = data_iface;

  // Patch mutable descriptor
  mutable_descriptor_ = descriptor_;
  mutable_descriptor_.iad0.first_interface = comm_iface;
  mutable_descriptor_.cmd_iface.interface_number = comm_iface;
  mutable_descriptor_.cmd_union_fd.controller_iface = comm_iface;
  mutable_descriptor_.cmd_union_fd.peripheral_iface0 = data_iface;
  mutable_descriptor_.cmd_ep.endpoint_address = interrupt_ep | 0x80;
  mutable_descriptor_.data_iface_alt0.interface_number = data_iface;
  mutable_descriptor_.data_iface_alt1.interface_number = data_iface;
  mutable_descriptor_.in_ep.endpoint_address = bulk_in_ep | 0x80;
  mutable_descriptor_.out_ep.endpoint_address = bulk_out_ep & 0x7F;

  // TX queue and task
  tx_queue_ = xQueueCreate(10, sizeof(void *));
  CHECK(tx_queue_);
  CHECK(xTaskCreate(CdcNcm::StaticTaskFunction, "cdc_ncm_task",
                    configMINIMAL_STACK_SIZE * 10, this, kUsbDeviceTaskPriority,
                    nullptr) == pdPASS);

  // Network setup
  std::string usb_ip;
  if (!GetUsbIpAddress(&usb_ip) ||
      !ipaddr_aton(usb_ip.c_str(), &netif_ipaddr_)) {
    IP4_ADDR(&netif_ipaddr_, 10, 0, 0, 1);
  }
  IP4_ADDR(&netif_netmask_, 255, 255, 255, 0);
  IP4_ADDR(&netif_gw_, 0, 0, 0, 0);
  netifapi_netif_add(&netif_, &netif_ipaddr_, &netif_netmask_, &netif_gw_, this,
                     CdcNcm::StaticNetifInit, tcpip_input);
  netifapi_netif_set_default(&netif_);
  netifapi_netif_set_link_up(&netif_);
  netifapi_netif_set_up(&netif_);
  start_dhcp_server(netif_ipaddr_.addr);
}

void CdcNcm::TaskFunction(void *param) {
  while (true) {
    std::vector<uint8_t> *packet;
    if (xQueueReceive(tx_queue_, &packet, portMAX_DELAY) == pdTRUE) {
      TransmitFrame(packet->data(), packet->size());
      delete packet;
    }
  }
}

err_t CdcNcm::NetifInit(struct netif *netif) {
  netif->name[0] = 'u';
  netif->name[1] = 's';
  netif->output = etharp_output;
  netif->linkoutput = CdcNcm::StaticTxFunc;
  netif->mtu = 1514;
  netif->hwaddr_len = 6;
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

  netif->hwaddr[0] = 0x00;
  netif->hwaddr[1] = 0x1A;
  netif->hwaddr[2] = 0x11;
  netif->hwaddr[3] = 0xBA;
  netif->hwaddr[4] = 0xDF;
  netif->hwaddr[5] = 0xAD;

  return ERR_OK;
}

err_t CdcNcm::TxFunc(struct netif *netif, struct pbuf *p) {
  if (!attached_) {
    return ERR_IF;
  }
  auto *packet = new std::vector<uint8_t>(p->tot_len);
  if (pbuf_copy_partial(p, packet->data(), p->tot_len, 0) != p->tot_len) {
    delete packet;
    return ERR_IF;
  }
  if (xQueueSendToBack(tx_queue_, &packet, 0) != pdTRUE) {
    delete packet;
    return ERR_IF;
  }
  return ERR_OK;
}

err_t CdcNcm::TransmitFrame(void *buffer, uint32_t length) {
  if (!attached_) {
    return ERR_IF;
  }
  if (length > kMaxFrameSize) {
    return ERR_BUF;
  }

  // Build NTB16 with single datagram:
  // [NTH16][Ethernet frame (aligned to 4)][NDP16 header][datagram entry][terminator]
  memset(tx_buffer_, 0, kNtbMaxSize);

  // Ethernet frame starts right after NTH16 header
  uint16_t datagram_offset = sizeof(NcmNth16);
  memcpy(tx_buffer_ + datagram_offset, buffer, length);

  // NDP16 starts after the datagram, aligned to 4 bytes
  uint16_t ndp_offset = (datagram_offset + length + 3) & ~3U;

  // Build NDP16: header + 1 datagram entry + terminator (0,0)
  NcmNdp16Header *ndp = reinterpret_cast<NcmNdp16Header *>(tx_buffer_ + ndp_offset);
  ndp->signature = NDP16_SIGNATURE;
  ndp->length = sizeof(NcmNdp16Header) + 2 * sizeof(NcmNdp16Datagram);
  ndp->next_ndp_index = 0;

  NcmNdp16Datagram *entries =
      reinterpret_cast<NcmNdp16Datagram *>(tx_buffer_ + ndp_offset + sizeof(NcmNdp16Header));
  entries[0].datagram_index = datagram_offset;
  entries[0].datagram_length = length;
  entries[1].datagram_index = 0;  // Terminator
  entries[1].datagram_length = 0;

  uint16_t total_len = ndp_offset + ndp->length;

  // Build NTH16
  NcmNth16 *nth = reinterpret_cast<NcmNth16 *>(tx_buffer_);
  nth->signature = NTH16_SIGNATURE;
  nth->header_length = sizeof(NcmNth16);
  nth->sequence = tx_sequence_++;
  nth->block_length = total_len;
  nth->ndp_index = ndp_offset;

  usb_status_t status;
  while (true) {
    status = USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, tx_buffer_,
                                  total_len);
    if (status == kStatus_USB_Busy) {
      taskYIELD();
    } else {
      break;
    }
  }

  // Send ZLP if needed
  if (status == kStatus_USB_Success &&
      (total_len % cdc_acm_data_endpoints_[DATA_IN].maxPacketSize) == 0) {
    static uint8_t zlp = 0;
    USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, &zlp, 0);
  }

  if (status != kStatus_USB_Success) {
    return ERR_IF;
  }
  return ERR_OK;
}

void CdcNcm::ProcessRxNtb(uint8_t *buffer, uint32_t length) {
  if (length < sizeof(NcmNth16)) {
    return;
  }

  NcmNth16 *nth = reinterpret_cast<NcmNth16 *>(buffer);
  if (nth->signature != NTH16_SIGNATURE) {
    return;
  }
  if (nth->ndp_index == 0 || nth->ndp_index + sizeof(NcmNdp16Header) > length) {
    return;
  }

  NcmNdp16Header *ndp =
      reinterpret_cast<NcmNdp16Header *>(buffer + nth->ndp_index);
  if (ndp->signature != NDP16_SIGNATURE) {
    return;
  }

  // Parse datagram entries
  NcmNdp16Datagram *entries = reinterpret_cast<NcmNdp16Datagram *>(
      buffer + nth->ndp_index + sizeof(NcmNdp16Header));
  uint16_t max_entries =
      (ndp->length - sizeof(NcmNdp16Header)) / sizeof(NcmNdp16Datagram);

  for (uint16_t i = 0; i < max_entries; i++) {
    if (entries[i].datagram_index == 0 && entries[i].datagram_length == 0) {
      break;  // Terminator
    }
    if (entries[i].datagram_index + entries[i].datagram_length > length) {
      break;  // Invalid
    }

    uint8_t *frame = buffer + entries[i].datagram_index;
    uint16_t frame_len = entries[i].datagram_length;

    struct pbuf *p = pbuf_alloc(PBUF_RAW, frame_len, PBUF_POOL);
    if (p) {
      pbuf_take(p, frame, frame_len);
      err_t ret = netif_.input(p, &netif_);
      if (ret != ERR_OK) {
        pbuf_free_callback(p);
      }
    }
  }
}

void CdcNcm::SendConnectionNotification() {
  // USB CDC NETWORK_CONNECTION notification (0x00)
  // bmRequestType: 0xA1 (device-to-host, class, interface)
  // bNotification: 0x00 (NETWORK_CONNECTION)
  // wValue: 1 = connected
  // wIndex: comm interface number
  // wLength: 0
  notify_network_conn_[0] = 0xA1;  // bmRequestType
  notify_network_conn_[1] = 0x00;  // NETWORK_CONNECTION
  notify_network_conn_[2] = 0x01;  // wValue low = connected
  notify_network_conn_[3] = 0x00;  // wValue high
  notify_network_conn_[4] = comm_iface_;  // wIndex low
  notify_network_conn_[5] = 0x00;  // wIndex high
  notify_network_conn_[6] = 0x00;  // wLength low
  notify_network_conn_[7] = 0x00;  // wLength high

  USB_DeviceCdcAcmSend(class_handle_, interrupt_ep_, notify_network_conn_, 8);
}

void CdcNcm::SendSpeedChangeNotification() {
  // USB CDC CONNECTION_SPEED_CHANGE notification (0x2A)
  // 8 bytes header + 8 bytes data (DL bitrate + UL bitrate)
  notify_speed_change_[0] = 0xA1;  // bmRequestType
  notify_speed_change_[1] = 0x2A;  // CONNECTION_SPEED_CHANGE
  notify_speed_change_[2] = 0x00;  // wValue low
  notify_speed_change_[3] = 0x00;  // wValue high
  notify_speed_change_[4] = comm_iface_;  // wIndex low
  notify_speed_change_[5] = 0x00;  // wIndex high
  notify_speed_change_[6] = 0x08;  // wLength low (8 bytes data)
  notify_speed_change_[7] = 0x00;  // wLength high

  // DL bitrate: 100 Mbps = 100000000 = 0x05F5E100
  notify_speed_change_[8]  = 0x00;
  notify_speed_change_[9]  = 0xE1;
  notify_speed_change_[10] = 0xF5;
  notify_speed_change_[11] = 0x05;
  // UL bitrate: 100 Mbps
  notify_speed_change_[12] = 0x00;
  notify_speed_change_[13] = 0xE1;
  notify_speed_change_[14] = 0xF5;
  notify_speed_change_[15] = 0x05;

  USB_DeviceCdcAcmSend(class_handle_, interrupt_ep_, notify_speed_change_, 16);
}

bool CdcNcm::HandleEvent(uint32_t event, void *param) {
  switch (event) {
    case kUSB_DeviceEventSetConfiguration: {
      attached_ = true;
      // Send network connection + speed notifications to bring link UP
      SendConnectionNotification();
      break;
    }
    case kUSB_DeviceEventSetInterface: {
      // Host sets alt setting 1 on data interface to activate bulk endpoints
      attached_ = true;
      // Send speed change after alt setting activated
      SendSpeedChangeNotification();
      // Prime the bulk OUT endpoint for NTB reception
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           kNtbMaxSize);
      break;
    }
    default:
      return false;
  }
  return true;
}

usb_status_t CdcNcm::Handler(uint32_t event, void *param) {
  usb_status_t ret = kStatus_USB_InvalidRequest;
  auto *acm_param =
      static_cast<usb_device_cdc_acm_request_param_struct_t *>(param);
  auto *ep_cb =
      static_cast<usb_device_endpoint_callback_message_struct_t *>(param);

  switch (event) {
    case kUSB_DeviceCdcEventSendResponse:
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventRecvResponse:
      if (ep_cb->length != 0 && ep_cb->buffer) {
        ProcessRxNtb(ep_cb->buffer, ep_cb->length);
      }
      // Re-prime
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           kNtbMaxSize);
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventSerialStateNotif:
      ((usb_device_cdc_acm_struct_t *)class_handle_)->hasSentState = 0;
      ret = kStatus_USB_Success;
      break;

    // NCM class requests
    case kUSB_DeviceCdcEventNcmGetNtbParameters:
      *(acm_param->buffer) = (uint8_t *)&ntb_params_;
      *(acm_param->length) = sizeof(ntb_params_);
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventEcmSetPacketFilter:
      ret = kStatus_USB_Success;
      break;

    default:
      // ACK any other forwarded class request
      if (event >= 0x100U) {
        ret = kStatus_USB_Success;
      }
      break;
  }

  return ret;
}

}  // namespace coralmicro
