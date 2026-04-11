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

#include "libs/cdc_rndis/cdc_rndis.h"

#include <cstring>
#include <memory>
#include <vector>

#include "libs/base/check.h"
#include "libs/base/tasks.h"
#include "libs/base/utils.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/lwip/etharp.h"
#include "third_party/nxp/rt1176-sdk/middleware/lwip/src/include/netif/ethernet.h"

extern "C" void start_dhcp_server(uint32_t local_addr);

#define DATA_OUT (1)
#define DATA_IN (0)

// RNDIS USB overhead: 44-byte header (rndis_packet_msg_struct_t)
#define RNDIS_USB_OVERHEAD_SIZE (44U)
#define RNDIS_DATA_OFFSET (36U)

namespace coralmicro {

std::map<class_handle_t, CdcRndis *> CdcRndis::handle_map_;

void CdcRndis::Init(uint8_t interrupt_ep, uint8_t bulk_in_ep,
                     uint8_t bulk_out_ep, uint8_t comm_iface,
                     uint8_t data_iface) {
  interrupt_ep_ = interrupt_ep;
  bulk_in_ep_ = bulk_in_ep;
  bulk_out_ep_ = bulk_out_ep;

  // Configure endpoint addresses
  cdc_acm_comm_endpoints_[0].endpointAddress = interrupt_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_IN].endpointAddress =
      bulk_in_ep | (USB_IN << 7);
  cdc_acm_data_endpoints_[DATA_OUT].endpointAddress =
      bulk_out_ep | (USB_OUT << 7);

  // Configure interface numbers
  cdc_acm_interfaces_[0].interfaceNumber = comm_iface;
  cdc_acm_interfaces_[1].interfaceNumber = data_iface;

  // Create mutable copy of descriptor and update dynamic fields
  mutable_descriptor_ = descriptor_;
  mutable_descriptor_.iad0.first_interface = comm_iface;
  mutable_descriptor_.cmd_iface.interface_number = comm_iface;
  mutable_descriptor_.cmd_mgmt_fd.data_interface = data_iface;
  mutable_descriptor_.cmd_union_fd.controller_iface = comm_iface;
  mutable_descriptor_.cmd_union_fd.peripheral_iface0 = data_iface;
  mutable_descriptor_.cmd_ep.endpoint_address = interrupt_ep | 0x80;
  mutable_descriptor_.data_iface.interface_number = data_iface;
  mutable_descriptor_.in_ep.endpoint_address = bulk_in_ep | 0x80;
  mutable_descriptor_.out_ep.endpoint_address = bulk_out_ep & 0x7F;

  // Create TX queue and background task
  tx_queue_ = xQueueCreate(10, sizeof(void *));
  CHECK(tx_queue_);
  CHECK(xTaskCreate(CdcRndis::StaticTaskFunction, "cdc_rndis_task",
                    configMINIMAL_STACK_SIZE * 10, this, kUsbDeviceTaskPriority,
                    nullptr) == pdPASS);

  // Read USB IP address
  std::string usb_ip;
  if (!GetUsbIpAddress(&usb_ip) ||
      !ipaddr_aton(usb_ip.c_str(), &netif_ipaddr_)) {
    IP4_ADDR(&netif_ipaddr_, 10, 10, 10, 1);
  }
  IP4_ADDR(&netif_netmask_, 255, 255, 255, 0);
  IP4_ADDR(&netif_gw_, 0, 0, 0, 0);
  netifapi_netif_add(&netif_, &netif_ipaddr_, &netif_netmask_, &netif_gw_, this,
                     CdcRndis::StaticNetifInit, tcpip_input);
  netifapi_netif_set_default(&netif_);
  netifapi_netif_set_link_up(&netif_);
  netifapi_netif_set_up(&netif_);
  start_dhcp_server(netif_ipaddr_.addr);
}

void CdcRndis::SetClassHandle(class_handle_t class_handle) {
  handle_map_[class_handle] = this;
  class_handle_ = class_handle;

  // Initialize the RNDIS protocol layer on top of CDC-ACM
  usb_device_cdc_rndis_config_struct_t rndis_config;
  rndis_config.devMaxTxSize =
      RNDIS_USB_OVERHEAD_SIZE + 1518;  // Max Ethernet frame
  rndis_config.rndisCallback = CdcRndis::StaticRndisCallback;
  usb_status_t status =
      USB_DeviceCdcRndisInit(class_handle, &rndis_config, &rndis_handle_);
  if (status != kStatus_USB_Success) {
    DbgConsole_Printf("[RNDIS] USB_DeviceCdcRndisInit failed: %d\r\n", status);
  }
}

void CdcRndis::TaskFunction(void *param) {
  while (true) {
    std::vector<uint8_t> *packet;
    if (xQueueReceive(tx_queue_, &packet, portMAX_DELAY) == pdTRUE) {
      TransmitFrame(packet->data(), packet->size());
      delete packet;
    }
  }
}

err_t CdcRndis::NetifInit(struct netif *netif) {
  netif->name[0] = 'u';
  netif->name[1] = 's';
  netif->output = etharp_output;
  netif->linkoutput = CdcRndis::StaticTxFunc;
  netif->mtu = 1500;
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

err_t CdcRndis::TxFunc(struct netif *netif, struct pbuf *p) {
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

err_t CdcRndis::TransmitFrame(void *buffer, uint32_t length) {
  if (!attached_ || !rndis_handle_) {
    return ERR_IF;
  }

  // Build RNDIS_PACKET_MSG header
  uint32_t rndis_len = RNDIS_USB_OVERHEAD_SIZE + length;
  rndis_packet_msg_struct_t *hdr =
      reinterpret_cast<rndis_packet_msg_struct_t *>(tx_buffer_);
  memset(hdr, 0, sizeof(rndis_packet_msg_struct_t));
  hdr->messageType = USB_LONG_TO_LITTLE_ENDIAN(RNDIS_PACKET_MSG);
  hdr->messageLength = USB_LONG_TO_LITTLE_ENDIAN(rndis_len);
  hdr->dataOffset = USB_LONG_TO_LITTLE_ENDIAN(RNDIS_DATA_OFFSET);
  hdr->dataLength = USB_LONG_TO_LITTLE_ENDIAN(length);

  // Copy Ethernet frame after header
  memcpy(tx_buffer_ + RNDIS_USB_OVERHEAD_SIZE, buffer, length);

  usb_status_t status;
  while (true) {
    status = USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, tx_buffer_,
                                  rndis_len);
    if (status == kStatus_USB_Busy) {
      taskYIELD();
    } else {
      break;
    }
  }

  // Send ZLP if transfer is multiple of max packet size
  if (status == kStatus_USB_Success &&
      (rndis_len % cdc_acm_data_endpoints_[DATA_IN].maxPacketSize) == 0) {
    static uint8_t zlp = 0;
    USB_DeviceCdcAcmSend(class_handle_, bulk_in_ep_, &zlp, 0);
  }

  if (status != kStatus_USB_Success) {
    DbgConsole_Printf("[RNDIS] USB_DeviceCdcAcmSend failed\r\n");
    return ERR_IF;
  }
  return ERR_OK;
}

err_t CdcRndis::ReceiveFrame(uint8_t *buffer, uint32_t length) {
  struct pbuf *frame = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
  if (!frame) {
    DbgConsole_Printf("[RNDIS] Failed to allocate pbuf\r\n");
    return ERR_BUF;
  }
  pbuf_take(frame, buffer, length);
  err_t ret = netif_.input(frame, &netif_);
  if (ret != ERR_OK) {
    DbgConsole_Printf("[RNDIS] tcpip_input() failed %d\r\n", ret);
    pbuf_free_callback(frame);
    return ERR_IF;
  }
  return ERR_OK;
}

void CdcRndis::ProcessRxPacket(uint8_t *buffer, uint32_t length) {
  if (length < RNDIS_USB_OVERHEAD_SIZE) {
    return;
  }
  rndis_packet_msg_struct_t *hdr =
      reinterpret_cast<rndis_packet_msg_struct_t *>(buffer);
  uint32_t msg_type = USB_LONG_TO_LITTLE_ENDIAN(hdr->messageType);
  if (msg_type != RNDIS_PACKET_MSG) {
    return;
  }
  uint32_t data_offset = USB_LONG_TO_LITTLE_ENDIAN(hdr->dataOffset);
  uint32_t data_length = USB_LONG_TO_LITTLE_ENDIAN(hdr->dataLength);

  // dataOffset is relative to the start of the dataOffset field (byte 8)
  uint8_t *data = buffer + 8 + data_offset;
  if (data_length > 0 && (data + data_length) <= (buffer + length)) {
    ReceiveFrame(data, data_length);
  }
}

usb_status_t CdcRndis::StaticRndisCallback(class_handle_t handle,
                                            uint32_t event, void *param) {
  usb_status_t error = kStatus_USB_Success;
  usb_device_cdc_rndis_request_param_struct_t *rndisParam =
      (usb_device_cdc_rndis_request_param_struct_t *)param;

  // Find the CdcRndis instance - same class_handle as the CDC-ACM handle
  CdcRndis *self = nullptr;
  for (auto &kv : handle_map_) {
    self = kv.second;
    break;
  }
  if (!self) return kStatus_USB_Error;

  switch (event) {
    case kUSB_DeviceCdcEventAppGetLinkSpeed:
      // Report 100 Mbps (in 100bps units)
      *((uint32_t *)rndisParam->buffer) = 1000000;
      break;
    case kUSB_DeviceCdcEventAppGetSendPacketSize:
      *((uint32_t *)rndisParam->buffer) = USB_LONG_TO_LITTLE_ENDIAN(
          (uint32_t)self->cdc_acm_data_endpoints_[DATA_IN].maxPacketSize);
      break;
    case kUSB_DeviceCdcEventAppGetRecvPacketSize:
      *((uint32_t *)rndisParam->buffer) = USB_LONG_TO_LITTLE_ENDIAN(
          (uint32_t)self->cdc_acm_data_endpoints_[DATA_OUT].maxPacketSize);
      break;
    case kUSB_DeviceCdcEventAppGetMacAddress:
      memcpy(rndisParam->buffer, self->netif_.hwaddr, 6);
      break;
    case kUSB_DeviceCdcEventAppGetMaxFrameSize:
      *((uint32_t *)rndisParam->buffer) = (1518 + RNDIS_USB_OVERHEAD_SIZE);
      break;
    case kUSB_DeviceCdcEventAppGetLinkStatus:
      // Always connected (no physical Ethernet port)
      *((uint32_t *)rndisParam->buffer) = 1;
      break;
    default:
      break;
  }
  return error;
}

bool CdcRndis::HandleEvent(uint32_t event, void *param) {
  switch (event) {
    case kUSB_DeviceEventSetConfiguration: {
      attached_ = true;
      // Reset RNDIS state on new configuration
      if (rndis_handle_) {
        uint8_t *message;
        uint32_t len;
        USB_DeviceCdcRndisHaltCommand(rndis_handle_);
        USB_DeviceCdcRndisResetCommand(rndis_handle_, &message, &len);
      }
      // Start receiving data
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           cdc_acm_data_endpoints_[DATA_OUT].maxPacketSize);
      break;
    }
    case kUSB_DeviceEventSetInterface:
      break;
    default:
      DbgConsole_Printf("[RNDIS] %s unhandled event %lu\r\n",
                        __PRETTY_FUNCTION__, event);
      return false;
  }
  return true;
}

usb_status_t CdcRndis::Handler(uint32_t event, void *param) {
  usb_status_t ret = kStatus_USB_InvalidRequest;
  auto *acm_param =
      static_cast<usb_device_cdc_acm_request_param_struct_t *>(param);
  auto *ep_cb =
      static_cast<usb_device_endpoint_callback_message_struct_t *>(param);

  switch (event) {
    case kUSB_DeviceCdcEventSendResponse:
      // TX complete
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventRecvResponse:
      // Data received from host
      if (ep_cb->length != 0 && ep_cb->buffer) {
        ProcessRxPacket(ep_cb->buffer, ep_cb->length);
      }
      // Re-arm receive
      USB_DeviceCdcAcmRecv(class_handle_, bulk_out_ep_, rx_buffer_,
                           cdc_acm_data_endpoints_[DATA_OUT].maxPacketSize);
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventSerialStateNotif:
      ((usb_device_cdc_acm_struct_t *)class_handle_)->hasSentState = 0;
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventSendEncapsulatedCommand:
      // RNDIS control message from host
      if (rndis_handle_) {
        if (1 == acm_param->isSetup) {
          *(acm_param->buffer) = rndis_handle_->rndisCommand;
          *(acm_param->length) = RNDIS_MAX_EXPECTED_COMMAND_SIZE;
        } else {
          USB_DeviceCdcRndisMessageSet(rndis_handle_, acm_param->buffer,
                                       acm_param->length);
        }
      }
      ret = kStatus_USB_Success;
      break;

    case kUSB_DeviceCdcEventGetEncapsulatedResponse:
      // RNDIS control response to host
      if (rndis_handle_) {
        ret = USB_DeviceCdcRndisMessageGet(rndis_handle_, acm_param->buffer,
                                           acm_param->length);
      }
      break;

    default:
      DbgConsole_Printf("[RNDIS] Unhandled CDC event: %lu\r\n", event);
      break;
  }

  return ret;
}

}  // namespace coralmicro
