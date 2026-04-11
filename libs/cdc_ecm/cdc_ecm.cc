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

#include "libs/cdc_ecm/cdc_ecm.h"

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

namespace coralmicro {

std::map<class_handle_t, CdcEcm *> CdcEcm::handle_map_;

void CdcEcm::Init(uint8_t interrupt_ep, uint8_t bulk_in_ep,
                   uint8_t bulk_out_ep, uint8_t comm_iface,
                   uint8_t data_iface) {
  interrupt_ep_ = interrupt_ep;
  bulk_in_ep_ = bulk_in_ep;
  bulk_out_ep_ = bulk_out_ep;

  // Set endpoint addresses
  cdc_acm_comm_endpoints_[0].endpointAddress = interrupt_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_IN].endpointAddress =
      bulk_in_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_OUT].endpointAddress =
      bulk_out_ep | (USB_OUT << 7);

  // Set interface numbers
  cdc_acm_interfaces_[0].interfaceNumber = comm_iface;
  cdc_acm_interfaces_[1].interfaceNumber = data_iface;

  // Patch mutable descriptor with runtime endpoint/interface values
  mutable_descriptor_ = descriptor_;
  mutable_descriptor_.iad0.first_interface = comm_iface;
  mutable_descriptor_.cmd_iface.interface_number = comm_iface;
  mutable_descriptor_.cmd_union_fd.controller_iface = comm_iface;
  mutable_descriptor_.cmd_union_fd.peripheral_iface0 = data_iface;
  mutable_descriptor_.cmd_ep.endpoint_address = interrupt_ep | 0x80;
  mutable_descriptor_.data_iface.interface_number = data_iface;
  mutable_descriptor_.in_ep.endpoint_address = bulk_in_ep | 0x80;
  mutable_descriptor_.out_ep.endpoint_address = bulk_out_ep & 0x7F;

  // TX queue and task
  tx_queue_ = xQueueCreate(10, sizeof(void *));
  CHECK(tx_queue_);
  CHECK(xTaskCreate(CdcEcm::StaticTaskFunction, "cdc_ecm_task",
                    configMINIMAL_STACK_SIZE * 10, this, kUsbDeviceTaskPriority,
                    nullptr) == pdPASS);

  // Network setup
  std::string usb_ip;
  if (!GetUsbIpAddress(&usb_ip) ||
      !ipaddr_aton(usb_ip.c_str(), &netif_ipaddr_)) {
    IP4_ADDR(&netif_ipaddr_, 10, 10, 10, 1);
  }
  IP4_ADDR(&netif_netmask_, 255, 255, 255, 0);
  IP4_ADDR(&netif_gw_, 0, 0, 0, 0);
  netifapi_netif_add(&netif_, &netif_ipaddr_, &netif_netmask_, &netif_gw_, this,
                     CdcEcm::StaticNetifInit, tcpip_input);
  netifapi_netif_set_default(&netif_);
  netifapi_netif_set_link_up(&netif_);
  netifapi_netif_set_up(&netif_);
  start_dhcp_server(netif_ipaddr_.addr);
}

void CdcEcm::TaskFunction(void *param) {
  while (true) {
    std::vector<uint8_t> *packet;
    if (xQueueReceive(tx_queue_, &packet, portMAX_DELAY) == pdTRUE) {
      TransmitFrame(packet->data(), packet->size());
      delete packet;
    }
  }
}

err_t CdcEcm::NetifInit(struct netif *netif) {
  netif->name[0] = 'u';
  netif->name[1] = 's';
  netif->output = etharp_output;
  netif->linkoutput = CdcEcm::StaticTxFunc;
  netif->mtu = 1514;
  netif->hwaddr_len = 6;
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

  // MAC address matching descriptor string index 4
  netif->hwaddr[0] = 0x00;
  netif->hwaddr[1] = 0x1A;
  netif->hwaddr[2] = 0x11;
  netif->hwaddr[3] = 0xBA;
  netif->hwaddr[4] = 0xDF;
  netif->hwaddr[5] = 0xAD;

  return ERR_OK;
}

err_t CdcEcm::TxFunc(struct netif *netif, struct pbuf *p) {
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

err_t CdcEcm::TransmitFrame(void *buffer, uint32_t length) {
  if (!attached_) {
    return ERR_IF;
  }

  // CDC-ECM: raw Ethernet frame, no wrapping header
  if (length > kEcmBufferSize) {
    return ERR_BUF;
  }
  memcpy(tx_buffer_, buffer, length);

  usb_status_t status;
  while (true) {
    status = USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, tx_buffer_,
                                  length);
    if (status == kStatus_USB_Busy) {
      taskYIELD();
    } else {
      break;
    }
  }

  // Send ZLP if transfer is multiple of max packet size
  if (status == kStatus_USB_Success &&
      (length % cdc_acm_data_endpoints_[DATA_IN].maxPacketSize) == 0) {
    static uint8_t zlp = 0;
    USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, &zlp, 0);
  }

  if (status != kStatus_USB_Success) {
    return ERR_IF;
  }
  return ERR_OK;
}

err_t CdcEcm::ReceiveFrame(uint8_t *buffer, uint32_t length) {
  struct pbuf *frame = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
  if (!frame) {
    return ERR_BUF;
  }
  pbuf_take(frame, buffer, length);
  err_t ret = netif_.input(frame, &netif_);
  if (ret != ERR_OK) {
    pbuf_free_callback(frame);
    return ERR_IF;
  }
  return ERR_OK;
}

bool CdcEcm::HandleEvent(uint32_t event, void *param) {
  switch (event) {
    case kUSB_DeviceEventSetConfiguration: {
      attached_ = true;
      // Prime the bulk OUT endpoint for receiving
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           cdc_acm_data_endpoints_[DATA_OUT].maxPacketSize);
      break;
    }
    case kUSB_DeviceEventSetInterface:
      break;
    default:
      return false;
  }
  return true;
}

usb_status_t CdcEcm::Handler(uint32_t event, void *param) {
  usb_status_t ret = kStatus_USB_InvalidRequest;
  auto *ep_cb =
      static_cast<usb_device_endpoint_callback_message_struct_t *>(param);

  switch (event) {
    case kUSB_DeviceCdcEventSendResponse:
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventRecvResponse:
      // CDC-ECM: raw Ethernet frame received on bulk OUT
      if (ep_cb->length != 0 && ep_cb->buffer) {
        ReceiveFrame(ep_cb->buffer, ep_cb->length);
      }
      // Re-prime the endpoint
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           cdc_acm_data_endpoints_[DATA_OUT].maxPacketSize);
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventSerialStateNotif:
      ((usb_device_cdc_acm_struct_t *)class_handle_)->hasSentState = 0;
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventEcmSetPacketFilter:
      // Accept and ACK SET_ETHERNET_PACKET_FILTER — no action needed
      ret = kStatus_USB_Success;
      break;

    default:
      // ACK any other forwarded class request to avoid STALLs
      if (event >= 0x100U) {
        ret = kStatus_USB_Success;
      }
      break;
  }

  return ret;
}

}  // namespace coralmicro
