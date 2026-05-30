// POSIX/libusb backend for the small USB_HostEdgeTpu* contract used by
// libs/tpu/edgetpu_driver.cc.  This is not a second TPU implementation; it is
// the host transport backend for the same EdgeTPU driver path used on ARM.

#define _POSIX_C_SOURCE 200809L

#include "libs/tpu/usb_host_edgetpu.h"
#include "libs/tpu/apex_firmware.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define EDGETPU_VID 0x1a6e
#define EDGETPU_PID_DFU 0x089a
#define EDGETPU_APP_VID 0x18d1
#define EDGETPU_PID_APP 0x9302
#define EDGETPU_TIMEOUT_MS 2000
#define EDGETPU_DFU_TRANSFER_SIZE 256

#define DFU_DNLOAD 1
#define DFU_UPLOAD 2
#define DFU_GETSTATUS 3
#define DFU_REQ_OUT 0x21
#define DFU_REQ_IN 0xA1
#define DFU_STATE_DFU_IDLE 2
#define DFU_STATE_DNLOAD_IDLE 5

static usb_status_t usb_status_from_libusb(int rc) {
  if (rc >= 0) return kStatus_USB_Success;
  if (rc == LIBUSB_ERROR_TIMEOUT) return kStatus_USB_TransferFailed;
  if (rc == LIBUSB_ERROR_PIPE) return kStatus_USB_TransferStall;
  if (rc == LIBUSB_ERROR_BUSY) return kStatus_USB_Busy;
  if (rc == LIBUSB_ERROR_NO_MEM) return kStatus_USB_AllocFail;
  return kStatus_USB_Error;
}

static int dfu_get_status(libusb_device_handle *dev, int iface,
                          uint8_t status[6]) {
  memset(status, 0, 6);
  return libusb_control_transfer(dev, DFU_REQ_IN, DFU_GETSTATUS, 0,
                                 (uint16_t)iface, status, 6,
                                 EDGETPU_TIMEOUT_MS);
}

static int dfu_download_block(libusb_device_handle *dev, int iface,
                              uint16_t block, const uint8_t *data,
                              uint16_t len) {
  return libusb_control_transfer(dev, DFU_REQ_OUT, DFU_DNLOAD, block,
                                 (uint16_t)iface, (unsigned char *)data, len,
                                 EDGETPU_TIMEOUT_MS);
}

static int dfu_upload_block(libusb_device_handle *dev, int iface,
                            uint16_t block, uint8_t *data, uint16_t len) {
  return libusb_control_transfer(dev, DFU_REQ_IN, DFU_UPLOAD, block,
                                 (uint16_t)iface, data, len,
                                 EDGETPU_TIMEOUT_MS);
}

static void posix_sleep_ms(unsigned ms) {
  struct timespec ts;
  ts.tv_sec = ms / 1000U;
  ts.tv_nsec = (long)(ms % 1000U) * 1000000L;
  nanosleep(&ts, NULL);
}

static void dfu_sleep_status_poll(const uint8_t status[6]) {
  unsigned poll_ms = (unsigned)status[1] |
                     ((unsigned)status[2] << 8) |
                     ((unsigned)status[3] << 16);
  if (poll_ms == 0) poll_ms = 1;
  posix_sleep_ms(poll_ms);
}

static int open_dfu_device(libusb_context *ctx, libusb_device_handle **out,
                           int *iface_out) {
  libusb_device_handle *dev =
      libusb_open_device_with_vid_pid(ctx, EDGETPU_VID, EDGETPU_PID_DFU);
  if (!dev) return LIBUSB_ERROR_NO_DEVICE;
  int iface = 0;
  if (libusb_kernel_driver_active(dev, iface) == 1) {
    libusb_detach_kernel_driver(dev, iface);
  }
  int rc = libusb_claim_interface(dev, iface);
  if (rc != 0) {
    libusb_close(dev);
    return rc;
  }
  *out = dev;
  *iface_out = iface;
  return 0;
}

static usb_status_t edgetpu_dfu_load_if_needed(libusb_context *ctx) {
  libusb_device_handle *app =
      libusb_open_device_with_vid_pid(ctx, EDGETPU_APP_VID, EDGETPU_PID_APP);
  if (app) {
    libusb_close(app);
    return kStatus_USB_Success;
  }

  libusb_device_handle *dfu = NULL;
  int iface = 0;
  int rc = open_dfu_device(ctx, &dfu, &iface);
  if (rc != 0) return usb_status_from_libusb(rc);

  printf("[tpu-posix] DFU loading EdgeTPU firmware len=%u\n",
         apex_firmware_bin_len);

  uint32_t offset = 0;
  uint16_t block = 0;
  while (offset <= apex_firmware_bin_len) {
    uint16_t n = (uint16_t)(apex_firmware_bin_len - offset);
    if (n > EDGETPU_DFU_TRANSFER_SIZE) n = EDGETPU_DFU_TRANSFER_SIZE;
    const uint8_t *src = n ? (const uint8_t *)(apex_firmware_bin + offset)
                           : NULL;
    rc = dfu_download_block(dfu, iface, block, src, n);
    if (rc < 0 || rc != n) {
      printf("[tpu-posix] DFU_DNLOAD failed block=%u rc=%d\n", block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }

    uint8_t status[6];
    rc = dfu_get_status(dfu, iface, status);
    if (rc != 6) {
      printf("[tpu-posix] DFU_GETSTATUS failed block=%u rc=%d\n", block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }
    dfu_sleep_status_poll(status);
    if (status[0] != 0 ||
        (n > 0 && status[4] != DFU_STATE_DNLOAD_IDLE) ||
        (n == 0 && status[4] != DFU_STATE_DFU_IDLE)) {
      printf("[tpu-posix] DFU status unexpected block=%u err=%u state=%u\n",
             block, status[0], status[4]);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return kStatus_USB_Error;
    }
    if (n == 0) break;
    offset += n;
    ++block;
  }

  uint8_t verify[EDGETPU_DFU_TRANSFER_SIZE];
  offset = 0;
  block = 0;
  while (offset < apex_firmware_bin_len) {
    uint16_t n = (uint16_t)(apex_firmware_bin_len - offset);
    if (n > EDGETPU_DFU_TRANSFER_SIZE) n = EDGETPU_DFU_TRANSFER_SIZE;
    rc = dfu_upload_block(dfu, iface, block, verify, EDGETPU_DFU_TRANSFER_SIZE);
    if (rc < (int)n || memcmp(verify, apex_firmware_bin + offset, n) != 0) {
      printf("[tpu-posix] DFU_UPLOAD verify failed block=%u rc=%d\n",
             block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }
    offset += n;
    ++block;
  }

  printf("[tpu-posix] DFU firmware verified; resetting USB device\n");
  libusb_release_interface(dfu, iface);
  libusb_reset_device(dfu);
  libusb_close(dfu);

  for (int i = 0; i < 150; ++i) {
    posix_sleep_ms(100);
    app = libusb_open_device_with_vid_pid(ctx, EDGETPU_APP_VID,
                                          EDGETPU_PID_APP);
    if (app) {
      libusb_close(app);
      return kStatus_USB_Success;
    }
  }
  return kStatus_USB_ControllerNotFound;
}

static uint8_t posix_out_ep(const usb_host_edgetpu_instance_t *inst,
                            uint8_t ep) {
  if (!inst || ep >= 4) return 0;
  if (inst->bulk_out_ep[ep]) return inst->bulk_out_ep[ep];
  return (uint8_t)(ep | LIBUSB_ENDPOINT_OUT);
}

static uint8_t posix_in_ep(const usb_host_edgetpu_instance_t *inst,
                           uint8_t ep) {
  if (!inst) return 0;
  if (ep < 3 && inst->bulk_in_ep[ep]) return inst->bulk_in_ep[ep];
  if (inst->interrupt_in_ep && ep == (inst->interrupt_in_ep & 0x0f)) {
    return inst->interrupt_in_ep;
  }
  return (uint8_t)(ep | LIBUSB_ENDPOINT_IN);
}

static void discover_endpoints(usb_host_edgetpu_instance_t *inst,
                               libusb_device *dev) {
  struct libusb_config_descriptor *cfg = NULL;
  if (libusb_get_active_config_descriptor(dev, &cfg) != 0) return;
  for (int i = 0; i < cfg->bNumInterfaces; ++i) {
    const struct libusb_interface *iface = &cfg->interface[i];
    for (int a = 0; a < iface->num_altsetting; ++a) {
      const struct libusb_interface_descriptor *alt = &iface->altsetting[a];
      if (alt->bInterfaceClass != 0xff) continue;
      inst->interface_number = alt->bInterfaceNumber;
      for (int e = 0; e < alt->bNumEndpoints; ++e) {
        const struct libusb_endpoint_descriptor *desc = &alt->endpoint[e];
        uint8_t addr = desc->bEndpointAddress;
        uint8_t num = addr & 0x0f;
        uint8_t type = desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
        if ((addr & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT &&
            type == LIBUSB_TRANSFER_TYPE_BULK && num < 4) {
          inst->bulk_out_ep[num] = addr;
        } else if ((addr & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN &&
                   type == LIBUSB_TRANSFER_TYPE_BULK && num < 3) {
          inst->bulk_in_ep[num] = addr;
        } else if ((addr & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN &&
                   type == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
          inst->interrupt_in_ep = addr;
        }
      }
      libusb_free_config_descriptor(cfg);
      return;
    }
  }
  libusb_free_config_descriptor(cfg);
}

usb_status_t USB_HostEdgeTpuOpenPosix(usb_host_edgetpu_instance_t **out) {
  if (!out) return kStatus_USB_InvalidParameter;
  *out = NULL;
  usb_host_edgetpu_instance_t *inst =
      (usb_host_edgetpu_instance_t *)calloc(1, sizeof(*inst));
  if (!inst) return kStatus_USB_AllocFail;
  inst->interface_number = 0;

  int rc = libusb_init(&inst->usb_ctx);
  if (rc != 0) {
    free(inst);
    return usb_status_from_libusb(rc);
  }
  usb_status_t dfu_st = edgetpu_dfu_load_if_needed(inst->usb_ctx);
  if (dfu_st != kStatus_USB_Success) {
    libusb_exit(inst->usb_ctx);
    free(inst);
    return dfu_st;
  }
  inst->dev = libusb_open_device_with_vid_pid(
      inst->usb_ctx, EDGETPU_APP_VID, EDGETPU_PID_APP);
  if (!inst->dev) {
    libusb_exit(inst->usb_ctx);
    free(inst);
    return kStatus_USB_ControllerNotFound;
  }

  libusb_device *dev = libusb_get_device(inst->dev);
  discover_endpoints(inst, dev);

  if (libusb_kernel_driver_active(inst->dev, inst->interface_number) == 1) {
    if (libusb_detach_kernel_driver(inst->dev, inst->interface_number) == 0) {
      inst->kernel_detached = true;
    }
  }
  rc = libusb_claim_interface(inst->dev, inst->interface_number);
  if (rc != 0) {
    USB_HostEdgeTpuClosePosix(inst);
    return usb_status_from_libusb(rc);
  }

  printf("[tpu-posix] opened Coral USB interface=%d out=%02x,%02x,%02x in=%02x,%02x irq=%02x\n",
         inst->interface_number, inst->bulk_out_ep[1], inst->bulk_out_ep[2],
         inst->bulk_out_ep[3], inst->bulk_in_ep[1], inst->bulk_in_ep[2],
         inst->interrupt_in_ep);
  *out = inst;
  return kStatus_USB_Success;
}

usb_status_t USB_HostEdgeTpuClosePosix(usb_host_edgetpu_instance_t *inst) {
  if (!inst) return kStatus_USB_Success;
  if (inst->dev) {
    libusb_release_interface(inst->dev, inst->interface_number);
    if (inst->kernel_detached) {
      libusb_attach_kernel_driver(inst->dev, inst->interface_number);
    }
    libusb_close(inst->dev);
  }
  if (inst->usb_ctx) libusb_exit(inst->usb_ctx);
  free(inst);
  return kStatus_USB_Success;
}

usb_status_t USB_HostEdgeTpuBulkOutSend(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;
  int transferred = 0;
  int rc = libusb_bulk_transfer(inst->dev, posix_out_ep(inst, endPoint),
                                buffer, (int)length, &transferred,
                                EDGETPU_TIMEOUT_MS);
  usb_status_t st = usb_status_from_libusb(rc);
  if (rc == 0 && transferred != (int)length) st = kStatus_USB_TransferFailed;
  if (callbackFn) callbackFn(callbackParam, buffer, (uint32_t)transferred, st);
  return st;
}

usb_status_t USB_HostEdgeTpuBulkInRecv(usb_host_edgetpu_instance_t *inst,
                                       uint8_t endPoint, uint8_t *buffer,
                                       uint32_t bufferLength,
                                       transfer_callback_t callbackFn,
                                       void *callbackParam) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;
  int transferred = 0;
  uint8_t ep = posix_in_ep(inst, endPoint);
  int rc;
  if (inst->interrupt_in_ep && ep == inst->interrupt_in_ep) {
    rc = libusb_interrupt_transfer(inst->dev, ep, buffer, (int)bufferLength,
                                   &transferred, EDGETPU_TIMEOUT_MS);
  } else {
    rc = libusb_bulk_transfer(inst->dev, ep, buffer, (int)bufferLength,
                              &transferred, EDGETPU_TIMEOUT_MS);
  }
  usb_status_t st = usb_status_from_libusb(rc);
  if (callbackFn) callbackFn(callbackParam, buffer, (uint32_t)transferred, st);
  return st;
}

usb_status_t USB_HostEdgeTpuBulkOutSendAsync(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam) {
  return USB_HostEdgeTpuBulkOutSend(inst, endPoint, buffer, length, callbackFn,
                                    callbackParam);
}

usb_status_t USB_HostEdgeTpuBulkInRecvAsync(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam) {
  return USB_HostEdgeTpuBulkInRecv(inst, endPoint, buffer, length, callbackFn,
                                   callbackParam);
}

usb_status_t USB_HostEdgeTpuCancelInFlight(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t direction) {
  (void)inst;
  (void)endPoint;
  (void)direction;
  return kStatus_USB_Success;
}

usb_status_t USB_HostEdgeTpuControl(usb_host_edgetpu_instance_t *inst,
                                    usb_setup_struct_t *setupPacket,
                                    uint8_t *buffer,
                                    transfer_callback_t callbackFn,
                                    void *callbackParam) {
  if (!inst || !inst->dev || !setupPacket) return kStatus_USB_InvalidHandle;
  int rc = libusb_control_transfer(
      inst->dev, setupPacket->bmRequestType, setupPacket->bRequest,
      setupPacket->wValue, setupPacket->wIndex, buffer, setupPacket->wLength,
      EDGETPU_TIMEOUT_MS);
  usb_status_t st = usb_status_from_libusb(rc);
  uint32_t n = rc > 0 ? (uint32_t)rc : 0;
  if (callbackFn) callbackFn(callbackParam, buffer, n, st);
  return st;
}

usb_status_t USB_HostEdgeTpuInit(usb_device_handle deviceHandle,
                                 usb_host_class_handle *classHandle) {
  (void)deviceHandle;
  if (!classHandle) return kStatus_USB_InvalidParameter;
  return USB_HostEdgeTpuOpenPosix((usb_host_edgetpu_instance_t **)classHandle);
}

usb_status_t USB_HostEdgeTpuDeinit(usb_device_handle deviceHandle,
                                   usb_host_class_handle classHandle) {
  (void)deviceHandle;
  return USB_HostEdgeTpuClosePosix((usb_host_edgetpu_instance_t *)classHandle);
}

usb_status_t USB_HostEdgeTpuSetInterface(
    usb_host_class_handle classHandle,
    usb_host_interface_handle interfaceHandle, uint8_t alternateSetting,
    transfer_callback_t callbackFn, void *callbackParam) {
  (void)interfaceHandle;
  (void)alternateSetting;
  usb_status_t st = classHandle ? kStatus_USB_Success : kStatus_USB_InvalidHandle;
  if (callbackFn) callbackFn(callbackParam, NULL, 0, st);
  return st;
}

usb_status_t USB_HostEdgeTpuGetStatus(usb_host_class_handle classHandle,
                                      uint8_t *statusData,
                                      transfer_callback_t callbackFn,
                                      void *callbackParam) {
  (void)statusData;
  usb_status_t st = classHandle ? kStatus_USB_Success : kStatus_USB_InvalidHandle;
  if (callbackFn) callbackFn(callbackParam, NULL, 0, st);
  return st;
}

usb_status_t USB_HostEdgeTpuDetach(usb_host_class_handle classHandle,
                                   uint16_t timeout,
                                   transfer_callback_t callbackFn,
                                   void *callbackParam) {
  (void)timeout;
  usb_status_t st = classHandle ? kStatus_USB_Success : kStatus_USB_InvalidHandle;
  if (callbackFn) callbackFn(callbackParam, NULL, 0, st);
  return st;
}
