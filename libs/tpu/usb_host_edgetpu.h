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

#ifndef LIBS_TPU_USB_HOST_EDGETPU_H_
#define LIBS_TPU_USB_HOST_EDGETPU_H_

#if defined(SENTAI_PLATFORM_SIM) && !defined(SENTAI_ARM_EMU)
#include <stdbool.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>

typedef enum _usb_status {
  kStatus_USB_Success = 0x00U,
  kStatus_USB_Error,
  kStatus_USB_Busy,
  kStatus_USB_InvalidHandle,
  kStatus_USB_InvalidParameter,
  kStatus_USB_InvalidRequest,
  kStatus_USB_ControllerNotFound,
  kStatus_USB_InvalidControllerInterface,
  kStatus_USB_NotSupported,
  kStatus_USB_Retry,
  kStatus_USB_TransferStall,
  kStatus_USB_TransferFailed,
  kStatus_USB_AllocFail,
  kStatus_USB_LackSwapBuffer,
  kStatus_USB_TransferCancel,
  kStatus_USB_BandwidthFail,
  kStatus_USB_MSDStatusFail,
  kStatus_USB_EHCIAttached,
  kStatus_USB_EHCIDetached,
  kStatus_USB_DataOverRun,
} usb_status_t;

typedef void *usb_host_handle;
typedef void *usb_device_handle;
typedef void *usb_host_configuration_handle;
typedef void *usb_host_interface_handle;
typedef void *usb_host_class_handle;
typedef void *usb_host_pipe_handle;
typedef void usb_host_transfer_t;

typedef struct _usb_setup_struct {
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} usb_setup_struct_t;

typedef void (*transfer_callback_t)(void *param, uint8_t *data,
                                    uint32_t dataLen, usb_status_t status);

#define USB_ENDPOINT_CONTROL (0x00U)
#define USB_ENDPOINT_ISOCHRONOUS (0x01U)
#define USB_ENDPOINT_BULK (0x02U)
#define USB_ENDPOINT_INTERRUPT (0x03U)
#define USB_OUT (0U)
#define USB_IN (1U)
#define USB_REQUEST_TYPE_DIR_OUT (0x00U)
#define USB_REQUEST_TYPE_DIR_IN (0x80U)
#define USB_REQUEST_TYPE_TYPE_STANDARD (0U)
#define USB_REQUEST_TYPE_TYPE_CLASS (0x20U)
#define USB_REQUEST_TYPE_TYPE_VENDOR (0x40U)
#define USB_REQUEST_TYPE_RECIPIENT_DEVICE (0x00U)
#define USB_REQUEST_TYPE_RECIPIENT_INTERFACE (0x01U)
#define USB_REQUEST_STANDARD_SET_INTERFACE (0x0BU)
#else
#include "usb.h"
#include "usb_host_config.h" // Must be above "usb_host.h"
#include "usb_host.h"
#include "usb_spec.h"
#endif

#define USB_HOST_EDGETPU_CLASS_CODE (0xFF)
#define USB_HOST_EDGETPU_SUBCLASS_CODE (0xFF)
/* Upper bound of pipe slots.  The single_ep firmware enumerates exactly 6
 * endpoints (1 IRQ IN + 1 bulk IN + 1 bulk OUT = 3 endpoints, reported via
 * a configuration where `epCount == 6`).  The multi_ep firmware variant
 * exposes more bulk IN/OUT pairs and reports a larger epCount.  Sizing the
 * pipe array to 16 lets either variant enumerate without overrunning the
 * on-stack/task-scope instance state.  RAM cost: ~10 extra pipe structs
 * ≈ 40 bytes, irrelevant on this target.  Host code only USES pipes with
 * endpoint addresses matching the BULK_OUT / BULK_IN / INTERRUPT_IN
 * numbers below, so extra endpoints are opened but idle — no extra USB
 * traffic. */
#define USB_EDGETPU_ENDPOINT_NUM 16
/* Minimum epCount required to call the EdgeTPU "functional" — we need at
 * least one bulk IN, one bulk OUT, and the interrupt IN = 3 endpoints.
 * The strict equality check (epCount == 6) that used to live in
 * USB_HostEdgeTpuOpenDataInterface has been relaxed to >= this bound so
 * the multi_ep variant (typically 8–10 endpoints) also enumerates. */
#define USB_EDGETPU_MIN_ENDPOINTS 3
#define USB_EDGETPU_BULK_OUT_ENDPOINT_NUM 3
#define USB_EDGETPU_BULK_IN_ENDPOINT_NUM 2
#define USB_EDGETPU_INTERRUPT_IN_ENDPOINT_NUM 1
#define USB_EDGETPU_BULK_OUT_PACKET_SIZE 512
/* 256 B matches the EdgeTPU device-advertised wMaxPacketSize for the
 * bulk-IN endpoint (verified empirically: configuring 512 here yields
 * identical wire behaviour and identical E15 FPS — the wire is always
 * 256-byte packets because the device enforces it via its descriptor).
 * Kept at 256 for consistency with the device spec. */
#define USB_EDGETPU_BULK_IN_PACKET_SIZE 256
#define USB_EDGETPU_INTERRRUPT_ENDPOINT_INDEX 5

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  USB_EDGETPU_TRANSFER_READY,
  USB_EDGETPU_TRANSFER_BUSY,
} usb_host_edgetpu_transfer_status_t;

typedef struct _usb_host_edgetpu_pipe {
  usb_host_pipe_handle pipeHandle;
  uint8_t pipeType;
  uint16_t packetSize;
  uint8_t endPoint;
  uint8_t direction;
  transfer_callback_t callbackFn;
  void *callbackParam;
  usb_host_transfer_t *activeTransfer;
  usb_host_edgetpu_transfer_status_t transferStatus;
  bool connected;
} usb_host_edgetpu_pipe_t;

typedef struct _usb_host_edgetpu_instance {
#if defined(SENTAI_PLATFORM_SIM) && !defined(SENTAI_ARM_EMU)
  libusb_context *usb_ctx;
  libusb_device_handle *dev;
  int interface_number;
  uint8_t bulk_out_ep[4];
  uint8_t bulk_in_ep[3];
  uint8_t interrupt_in_ep;
  bool kernel_detached;
#endif
  usb_host_handle hostHandle;     /*!< This instance's related host handle*/
  usb_device_handle deviceHandle; /*!< This instance's related device handle*/
  usb_host_interface_handle
      interfaceHandle; /*!< This instance's related interface handle*/
  usb_host_edgetpu_pipe_t pipes[USB_EDGETPU_ENDPOINT_NUM]; /* pipes */
  usb_host_pipe_handle
      controlPipe; /*!< This instance's related device control pipe*/
  usb_host_transfer_t *controlTransfer; /*!< Ongoing control transfer*/
  transfer_callback_t
      controlCallbackFn;      /*!< control transfer callback function pointer*/
  void *controlCallbackParam; /*!< control transfer callback parameter*/
  usb_host_edgetpu_transfer_status_t controlTransferStatus;
} usb_host_edgetpu_instance_t;

usb_status_t USB_HostEdgeTpuInit(usb_device_handle deviceHandle,
                                 usb_host_class_handle *classHandle);

usb_status_t USB_HostEdgeTpuDeinit(usb_device_handle deviceHandle,
                                   usb_host_class_handle classHandle);

usb_status_t USB_HostEdgeTpuGetStatus(usb_host_class_handle classHandle,
                                      uint8_t *statusData,
                                      transfer_callback_t callbackFn,
                                      void *callbackParam);

usb_status_t USB_HostEdgeTpuDetach(usb_host_class_handle classHandle,
                                   uint16_t timeout,
                                   transfer_callback_t callbackFn,
                                   void *callbackParam);

usb_status_t USB_HostEdgeTpuSetInterface(
    usb_host_class_handle classHandle,
    usb_host_interface_handle interfaceHandle, uint8_t alternateSetting,
    transfer_callback_t callbackFn, void *callbackParam);

/* length widened from uint16_t (65 535 B cap) to uint32_t so the TPU
 * driver can push a whole parameter / instruction / input batch in
 * a single URB.  Fewer URBs → fewer submit/sem/callback round-trips. */
usb_status_t USB_HostEdgeTpuBulkOutSend(
    usb_host_edgetpu_instance_t *tpuInstance, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam);

usb_status_t USB_HostEdgeTpuBulkInRecv(usb_host_edgetpu_instance_t *tpuInstance,
                                       uint8_t endPoint, uint8_t *buffer,
                                       uint32_t bufferLength,
                                       transfer_callback_t callbackFn,
                                       void *callbackParam);

/* NASA/JPL fault-tolerance: cancel any in-flight transfer on the
 * named endpoint + direction.  Idempotent (no-op if nothing is
 * in flight).  Used by TpuDriver on sema timeout to avoid the
 * orphan-transfer cascade described in
 * examples/sentai_runtime/agent/experiment.md §V19.
 * Caller MUST wait on its sema again after this returns so the
 * cancel callback can land on valid memory. */
usb_status_t USB_HostEdgeTpuCancelInFlight(
    usb_host_edgetpu_instance_t *tpuInstance, uint8_t endPoint,
    uint8_t direction);

/* Async variants — unlock multiple in-flight transfers per pipe.
 * See usb_host_edgetpu.c for the per-transfer callback pool. */
usb_status_t USB_HostEdgeTpuBulkOutSendAsync(
    usb_host_edgetpu_instance_t *tpuInstance, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam);
usb_status_t USB_HostEdgeTpuBulkInRecvAsync(
    usb_host_edgetpu_instance_t *tpuInstance, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam);

usb_status_t USB_HostEdgeTpuControl(usb_host_edgetpu_instance_t *tpuInstance,
                                    usb_setup_struct_t *setupPacket,
                                    uint8_t *buffer,
                                    transfer_callback_t callbackFn,
                                    void *callbackParam);

#if defined(SENTAI_PLATFORM_SIM) && !defined(SENTAI_ARM_EMU)
usb_status_t USB_HostEdgeTpuOpenPosix(usb_host_edgetpu_instance_t **instance);
usb_status_t USB_HostEdgeTpuClosePosix(usb_host_edgetpu_instance_t *instance);
#endif

#ifdef __cplusplus
}
#endif

#endif  // LIBS_TPU_USB_HOST_EDGETPU_H_
