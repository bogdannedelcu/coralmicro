// POSIX/libusb backend for the small USB_HostEdgeTpu* contract used by
// libs/tpu/edgetpu_driver.cc.  This is not a second TPU implementation; it is
// the host transport backend for the same EdgeTPU driver path used on ARM.

#define _POSIX_C_SOURCE 200809L

#include "libs/tpu/usb_host_edgetpu.h"
#include "libs/tpu/apex_firmware.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "examples/sentai_runtime/sentai_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>

#define EDGETPU_VID 0x1a6e
#define EDGETPU_PID_DFU 0x089a
#define EDGETPU_APP_VID 0x18d1
#define EDGETPU_PID_APP 0x9302
#define EDGETPU_TIMEOUT_MS 2000
#define EDGETPU_DFU_TRANSFER_SIZE 256
#define EDGETPU_ASYNC_POOL_SIZE 8
#define EDGETPU_EVENT_STACK_WORDS 4096
#define EDGETPU_CONTROL_MAX_DATA 4096

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

typedef struct {
  volatile int in_use;
  transfer_callback_t cb;
  void *cb_param;
  uint8_t *buffer;
  uint8_t ep;
  uint8_t kind;
  uint32_t requested;
  uint64_t started_us;
  SemaphoreHandle_t done;
  volatile int completed;
  volatile usb_status_t status;
  volatile uint32_t transferred;
} posix_transfer_ctx_t;

enum {
  kPosixXferOut = 0,
  kPosixXferIn = 1,
  kPosixXferEvent = 2,
  kPosixXferIntr = 3,
};

static posix_transfer_ctx_t s_async_pool[EDGETPU_ASYNC_POOL_SIZE];
static StaticTask_t s_event_task_tcb;
static StackType_t s_event_task_stack[EDGETPU_EVENT_STACK_WORDS];
static TaskHandle_t s_event_task;
static libusb_context *s_event_usb_ctx;
static volatile int s_event_task_running;
static uint8_t s_control_buf[LIBUSB_CONTROL_SETUP_SIZE + EDGETPU_CONTROL_MAX_DATA];
extern volatile uint8_t g_sentai_tpu_trace;
volatile int g_sentai_tpu_posix_fast_sync_wait = 0;
volatile uint32_t g_sentai_tpu_posix_usb_out_calls = 0;
volatile uint64_t g_sentai_tpu_posix_usb_out_req = 0;
volatile uint64_t g_sentai_tpu_posix_usb_out_done = 0;
volatile uint64_t g_sentai_tpu_posix_usb_out_us = 0;
volatile uint32_t g_sentai_tpu_posix_usb_out_short = 0;
volatile uint32_t g_sentai_tpu_posix_usb_in_calls = 0;
volatile uint64_t g_sentai_tpu_posix_usb_in_req = 0;
volatile uint64_t g_sentai_tpu_posix_usb_in_done = 0;
volatile uint64_t g_sentai_tpu_posix_usb_in_us = 0;
volatile uint32_t g_sentai_tpu_posix_usb_in_short = 0;
volatile uint32_t g_sentai_tpu_posix_usb_event_calls = 0;
volatile uint64_t g_sentai_tpu_posix_usb_event_req = 0;
volatile uint64_t g_sentai_tpu_posix_usb_event_done = 0;
volatile uint64_t g_sentai_tpu_posix_usb_event_us = 0;
volatile uint32_t g_sentai_tpu_posix_usb_event_short = 0;
volatile uint32_t g_sentai_tpu_posix_usb_intr_calls = 0;
volatile uint64_t g_sentai_tpu_posix_usb_intr_req = 0;
volatile uint64_t g_sentai_tpu_posix_usb_intr_done = 0;
volatile uint64_t g_sentai_tpu_posix_usb_intr_us = 0;
volatile uint32_t g_sentai_tpu_posix_usb_intr_short = 0;
volatile uint32_t g_sentai_tpu_posix_usb_timeouts = 0;
volatile uint32_t g_sentai_tpu_posix_usb_failed = 0;

static void posix_sleep_ms(unsigned ms);
static uint64_t monotonic_us(void);

void sentai_tpu_posix_usb_stats_reset(void) {
  g_sentai_tpu_posix_usb_out_calls = 0;
  g_sentai_tpu_posix_usb_out_req = 0;
  g_sentai_tpu_posix_usb_out_done = 0;
  g_sentai_tpu_posix_usb_out_us = 0;
  g_sentai_tpu_posix_usb_out_short = 0;
  g_sentai_tpu_posix_usb_in_calls = 0;
  g_sentai_tpu_posix_usb_in_req = 0;
  g_sentai_tpu_posix_usb_in_done = 0;
  g_sentai_tpu_posix_usb_in_us = 0;
  g_sentai_tpu_posix_usb_in_short = 0;
  g_sentai_tpu_posix_usb_event_calls = 0;
  g_sentai_tpu_posix_usb_event_req = 0;
  g_sentai_tpu_posix_usb_event_done = 0;
  g_sentai_tpu_posix_usb_event_us = 0;
  g_sentai_tpu_posix_usb_event_short = 0;
  g_sentai_tpu_posix_usb_intr_calls = 0;
  g_sentai_tpu_posix_usb_intr_req = 0;
  g_sentai_tpu_posix_usb_intr_done = 0;
  g_sentai_tpu_posix_usb_intr_us = 0;
  g_sentai_tpu_posix_usb_intr_short = 0;
  g_sentai_tpu_posix_usb_timeouts = 0;
  g_sentai_tpu_posix_usb_failed = 0;
}

static void add_u32(volatile uint32_t *dst, uint32_t value) {
  __sync_fetch_and_add(dst, value);
}

static void add_u64(volatile uint64_t *dst, uint64_t value) {
  __sync_fetch_and_add(dst, value);
}

static uint8_t transfer_kind(const usb_host_edgetpu_instance_t *inst,
                             uint8_t ep) {
  if (!(ep & LIBUSB_ENDPOINT_IN)) return kPosixXferOut;
  if (inst && inst->interrupt_in_ep && ep == inst->interrupt_in_ep) {
    return kPosixXferIntr;
  }
  if ((ep & 0x0f) == 2) return kPosixXferEvent;
  return kPosixXferIn;
}

static void stats_submit(uint8_t kind, uint32_t requested) {
  switch (kind) {
    case kPosixXferOut:
      add_u32(&g_sentai_tpu_posix_usb_out_calls, 1);
      add_u64(&g_sentai_tpu_posix_usb_out_req, requested);
      break;
    case kPosixXferEvent:
      add_u32(&g_sentai_tpu_posix_usb_event_calls, 1);
      add_u64(&g_sentai_tpu_posix_usb_event_req, requested);
      break;
    case kPosixXferIntr:
      add_u32(&g_sentai_tpu_posix_usb_intr_calls, 1);
      add_u64(&g_sentai_tpu_posix_usb_intr_req, requested);
      break;
    case kPosixXferIn:
    default:
      add_u32(&g_sentai_tpu_posix_usb_in_calls, 1);
      add_u64(&g_sentai_tpu_posix_usb_in_req, requested);
      break;
  }
}

static void stats_complete(uint8_t kind, uint32_t requested, uint32_t actual,
                           usb_status_t status, uint64_t elapsed_us) {
  const int short_xfer = status == kStatus_USB_Success && actual < requested;
  if (status == kStatus_USB_TransferFailed) {
    add_u32(&g_sentai_tpu_posix_usb_timeouts, 1);
  } else if (status != kStatus_USB_Success) {
    add_u32(&g_sentai_tpu_posix_usb_failed, 1);
  }
  switch (kind) {
    case kPosixXferOut:
      add_u64(&g_sentai_tpu_posix_usb_out_done, actual);
      add_u64(&g_sentai_tpu_posix_usb_out_us, elapsed_us);
      if (short_xfer) add_u32(&g_sentai_tpu_posix_usb_out_short, 1);
      break;
    case kPosixXferEvent:
      add_u64(&g_sentai_tpu_posix_usb_event_done, actual);
      add_u64(&g_sentai_tpu_posix_usb_event_us, elapsed_us);
      if (short_xfer) add_u32(&g_sentai_tpu_posix_usb_event_short, 1);
      break;
    case kPosixXferIntr:
      add_u64(&g_sentai_tpu_posix_usb_intr_done, actual);
      add_u64(&g_sentai_tpu_posix_usb_intr_us, elapsed_us);
      if (short_xfer) add_u32(&g_sentai_tpu_posix_usb_intr_short, 1);
      break;
    case kPosixXferIn:
    default:
      add_u64(&g_sentai_tpu_posix_usb_in_done, actual);
      add_u64(&g_sentai_tpu_posix_usb_in_us, elapsed_us);
      if (short_xfer) add_u32(&g_sentai_tpu_posix_usb_in_short, 1);
      break;
  }
}

static posix_transfer_ctx_t *alloc_async_ctx(void) {
  for (int i = 0; i < EDGETPU_ASYNC_POOL_SIZE; ++i) {
    if (__sync_bool_compare_and_swap(&s_async_pool[i].in_use, 0, 1)) {
      memset((void *)&s_async_pool[i], 0, sizeof(s_async_pool[i]));
      s_async_pool[i].in_use = 1;
      return &s_async_pool[i];
    }
  }
  return NULL;
}

static void free_async_ctx(posix_transfer_ctx_t *ctx) {
  if (!ctx) return;
  __sync_synchronize();
  ctx->in_use = 0;
}

static void posix_libusb_event_task(void *arg) {
  (void)arg;
  while (s_event_task_running && s_event_usb_ctx) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    (void)libusb_handle_events_timeout_completed(s_event_usb_ctx, &tv, NULL);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  vTaskDelete(NULL);
}

static void ensure_event_task(libusb_context *ctx) {
  if (s_event_task) return;
  s_event_usb_ctx = ctx;
  s_event_task_running = 1;
  s_event_task = xTaskCreateStatic(
      posix_libusb_event_task, "tpu_usb_evt", EDGETPU_EVENT_STACK_WORDS,
      NULL, tskIDLE_PRIORITY + 2, s_event_task_stack, &s_event_task_tcb);
}

static void posix_transfer_cb(struct libusb_transfer *transfer) {
  posix_transfer_ctx_t *ctx = (posix_transfer_ctx_t *)transfer->user_data;
  usb_status_t st = usb_status_from_libusb(transfer->status);
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) st = kStatus_USB_Success;
  if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT) st = kStatus_USB_TransferFailed;
  if (transfer->status == LIBUSB_TRANSFER_CANCELLED) st = kStatus_USB_Error;
  if (ctx) {
    stats_complete(ctx->kind, ctx->requested,
                   (uint32_t)transfer->actual_length, st,
                   monotonic_us() - ctx->started_us);
    ctx->status = st;
    ctx->transferred = (uint32_t)transfer->actual_length;
    ctx->completed = 1;
    if (ctx->cb) ctx->cb(ctx->cb_param, ctx->buffer, ctx->transferred, st);
    if (ctx->done) {
      xSemaphoreGive(ctx->done);
    } else {
      free_async_ctx(ctx);
    }
  }
  libusb_free_transfer(transfer);
}

static void pump_libusb_once(libusb_context *ctx) {
  if (!ctx) return;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100;
  (void)libusb_handle_events_timeout_completed(ctx, &tv, NULL);
}

static uint64_t monotonic_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static usb_status_t submit_transfer(usb_host_edgetpu_instance_t *inst,
                                    uint8_t ep,
                                    uint8_t *buffer,
                                    uint32_t length,
                                    transfer_callback_t callbackFn,
                                    void *callbackParam,
                                    int sync,
                                    usb_status_t *sync_status,
                                    uint32_t *sync_transferred) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;

  struct libusb_transfer *transfer = libusb_alloc_transfer(0);
  if (!transfer) return kStatus_USB_AllocFail;

  StaticSemaphore_t sema_buf;
  SemaphoreHandle_t sema = NULL;
  posix_transfer_ctx_t *ctx = NULL;
  if (sync) {
    sema = xSemaphoreCreateBinaryStatic(&sema_buf);
    if (!sema) {
      libusb_free_transfer(transfer);
      return kStatus_USB_AllocFail;
    }
    ctx = alloc_async_ctx();
    if (!ctx) {
      libusb_free_transfer(transfer);
      return kStatus_USB_Busy;
    }
    ctx->done = sema;
  } else {
    ctx = alloc_async_ctx();
    if (!ctx) {
      libusb_free_transfer(transfer);
      return kStatus_USB_Busy;
    }
  }
  ctx->cb = callbackFn;
  ctx->cb_param = callbackParam;
  ctx->buffer = buffer;
  ctx->ep = ep;
  ctx->kind = transfer_kind(inst, ep);
  ctx->requested = length;
  ctx->started_us = monotonic_us();
  ctx->status = kStatus_USB_Error;
  ctx->transferred = 0;
  ctx->completed = 0;
  stats_submit(ctx->kind, length);

  if (inst->interrupt_in_ep && ep == inst->interrupt_in_ep) {
    libusb_fill_interrupt_transfer(transfer, inst->dev, ep, buffer,
                                   (int)length, posix_transfer_cb, ctx,
                                   EDGETPU_TIMEOUT_MS);
  } else if (ep & LIBUSB_ENDPOINT_IN) {
    libusb_fill_bulk_transfer(transfer, inst->dev, ep, buffer, (int)length,
                              posix_transfer_cb, ctx, EDGETPU_TIMEOUT_MS);
  } else {
    libusb_fill_bulk_transfer(transfer, inst->dev, ep, buffer, (int)length,
                              posix_transfer_cb, ctx, EDGETPU_TIMEOUT_MS);
  }

  if (sync && !(ep & LIBUSB_ENDPOINT_IN) && length > 8) {
    posix_sleep_ms(1);
  }
  if (g_sentai_tpu_trace) {
    sentai_logf("usb-xfer", "submit ep=%02x len=%lu sync=%d",
                ep, (unsigned long)length, sync);
  }
  int rc = libusb_submit_transfer(transfer);
  if (g_sentai_tpu_trace) {
    sentai_logf("usb-xfer", "submit rc=%d ep=%02x len=%lu",
                rc, ep, (unsigned long)length);
  }
  if (rc != 0) {
    add_u32(&g_sentai_tpu_posix_usb_failed, 1);
    free_async_ctx(ctx);
    libusb_free_transfer(transfer);
    return usb_status_from_libusb(rc);
  }
  if (!sync) {
    ensure_event_task(inst->usb_ctx);
    return kStatus_USB_Success;
  }
  uint64_t deadline_us = monotonic_us() +
                         (uint64_t)(EDGETPU_TIMEOUT_MS + 500) * 1000ULL;
  if (g_sentai_tpu_trace) {
    sentai_logf("usb-xfer", "wait ep=%02x len=%lu",
                ep, (unsigned long)length);
  }
  while (!ctx->completed) {
    pump_libusb_once(inst->usb_ctx);
    (void)xSemaphoreTake(sema, 0);
    if (ctx->completed &&
        (g_sentai_tpu_posix_fast_sync_wait || (ep & LIBUSB_ENDPOINT_IN))) {
      break;
    }
    if (monotonic_us() >= deadline_us) break;
    posix_sleep_ms(1);
  }
  if (!ctx->completed) {
    sentai_logf("tpu-posix", "usb timeout ep=%02x len=%lu sync=%d",
                ep, (unsigned long)length, sync);
    libusb_cancel_transfer(transfer);
    deadline_us = monotonic_us() + 250000ULL;
    while (!ctx->completed) {
      pump_libusb_once(inst->usb_ctx);
      (void)xSemaphoreTake(sema, 0);
      if (monotonic_us() >= deadline_us) break;
      posix_sleep_ms(1);
    }
  }
  if (!ctx->completed) {
    ctx->cb = NULL;
    ctx->cb_param = NULL;
    ctx->done = NULL;
    if (sync_status) *sync_status = kStatus_USB_TransferFailed;
    if (sync_transferred) *sync_transferred = 0;
    return kStatus_USB_TransferFailed;
  }
  usb_status_t final_status = ctx->status;
  uint32_t final_transferred = ctx->transferred;
  free_async_ctx(ctx);
  if (sync_status) *sync_status = final_status;
  if (sync_transferred) *sync_transferred = final_transferred;
  return final_status;
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
  while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {}
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

static libusb_device_handle *open_app_device_with_retry(libusb_context *ctx) {
  for (int i = 0; i < 30; ++i) {
    libusb_device_handle *dev =
        libusb_open_device_with_vid_pid(ctx, EDGETPU_APP_VID, EDGETPU_PID_APP);
    if (dev) return dev;
    posix_sleep_ms(100);
  }
  return NULL;
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

  sentai_logf("tpu-posix", "DFU loading EdgeTPU firmware len=%u",
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
      sentai_logf("tpu-posix", "DFU_DNLOAD failed block=%u rc=%d", block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }

    uint8_t status[6];
    rc = dfu_get_status(dfu, iface, status);
    if (rc != 6) {
      sentai_logf("tpu-posix", "DFU_GETSTATUS failed block=%u rc=%d",
                  block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }
    dfu_sleep_status_poll(status);
    if (status[0] != 0 ||
        (n > 0 && status[4] != DFU_STATE_DNLOAD_IDLE) ||
        (n == 0 && status[4] != DFU_STATE_DFU_IDLE)) {
      sentai_logf("tpu-posix",
                  "DFU status unexpected block=%u err=%u state=%u",
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
      sentai_logf("tpu-posix", "DFU_UPLOAD verify failed block=%u rc=%d",
                  block, rc);
      libusb_release_interface(dfu, iface);
      libusb_close(dfu);
      return usb_status_from_libusb(rc < 0 ? rc : LIBUSB_ERROR_IO);
    }
    offset += n;
    ++block;
  }

  sentai_logf("tpu-posix", "DFU firmware verified; resetting USB device");
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

  sentai_logf("tpu-posix", "libusb_init");
  int rc = libusb_init(&inst->usb_ctx);
  if (rc != 0) {
    free(inst);
    return usb_status_from_libusb(rc);
  }
  sentai_logf("tpu-posix", "dfu/app probe");
  usb_status_t dfu_st = edgetpu_dfu_load_if_needed(inst->usb_ctx);
  if (dfu_st != kStatus_USB_Success) {
    libusb_exit(inst->usb_ctx);
    free(inst);
    return dfu_st;
  }
  sentai_logf("tpu-posix", "open app device");
  inst->dev = open_app_device_with_retry(inst->usb_ctx);
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

  sentai_logf("tpu-posix",
              "opened Coral USB interface=%d out=%02x,%02x,%02x in=%02x,%02x irq=%02x",
              inst->interface_number, inst->bulk_out_ep[1],
              inst->bulk_out_ep[2], inst->bulk_out_ep[3],
              inst->bulk_in_ep[1], inst->bulk_in_ep[2],
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
  usb_status_t cb_st = kStatus_USB_Error;
  uint32_t transferred = 0;
  usb_status_t st = submit_transfer(inst, posix_out_ep(inst, endPoint),
                                    buffer, length, callbackFn, callbackParam,
                                    1, &cb_st, &transferred);
  (void)transferred;
  if (st == kStatus_USB_Success && cb_st != kStatus_USB_Success) st = cb_st;
  return st;
}

usb_status_t USB_HostEdgeTpuBulkInRecv(usb_host_edgetpu_instance_t *inst,
                                       uint8_t endPoint, uint8_t *buffer,
                                       uint32_t bufferLength,
                                       transfer_callback_t callbackFn,
                                       void *callbackParam) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;
  uint8_t ep = posix_in_ep(inst, endPoint);
  usb_status_t cb_st = kStatus_USB_Error;
  uint32_t transferred = 0;
  usb_status_t st = submit_transfer(inst, ep, buffer, bufferLength,
                                    callbackFn, callbackParam, 1,
                                    &cb_st, &transferred);
  if (st == kStatus_USB_Success && cb_st != kStatus_USB_Success) st = cb_st;
  return st;
}

usb_status_t USB_HostEdgeTpuBulkOutSendAsync(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;
  return submit_transfer(inst, posix_out_ep(inst, endPoint), buffer, length,
                         callbackFn, callbackParam, 0, NULL, NULL);
}

usb_status_t USB_HostEdgeTpuBulkInRecvAsync(
    usb_host_edgetpu_instance_t *inst, uint8_t endPoint, uint8_t *buffer,
    uint32_t length, transfer_callback_t callbackFn, void *callbackParam) {
  if (!inst || !inst->dev) return kStatus_USB_InvalidHandle;
  return submit_transfer(inst, posix_in_ep(inst, endPoint), buffer, length,
                         callbackFn, callbackParam, 0, NULL, NULL);
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
  if (setupPacket->wLength > EDGETPU_CONTROL_MAX_DATA) {
    return kStatus_USB_InvalidParameter;
  }

  struct libusb_transfer *transfer = libusb_alloc_transfer(0);
  if (!transfer) return kStatus_USB_AllocFail;

  StaticSemaphore_t sema_buf;
  SemaphoreHandle_t sema = xSemaphoreCreateBinaryStatic(&sema_buf);
  posix_transfer_ctx_t *ctx = alloc_async_ctx();
  if (!sema || !ctx) {
    if (ctx) free_async_ctx(ctx);
    libusb_free_transfer(transfer);
    return kStatus_USB_AllocFail;
  }

  libusb_fill_control_setup(s_control_buf, setupPacket->bmRequestType,
                            setupPacket->bRequest, setupPacket->wValue,
                            setupPacket->wIndex, setupPacket->wLength);
  if (!(setupPacket->bmRequestType & LIBUSB_ENDPOINT_IN) &&
      setupPacket->wLength > 0 && buffer) {
    memcpy(s_control_buf + LIBUSB_CONTROL_SETUP_SIZE, buffer,
           setupPacket->wLength);
  }

  ctx->cb = NULL;
  ctx->cb_param = NULL;
  ctx->buffer = s_control_buf;
  ctx->done = sema;
  ctx->status = kStatus_USB_Error;
  ctx->transferred = 0;
  ctx->completed = 0;

  libusb_fill_control_transfer(transfer, inst->dev, s_control_buf,
                               posix_transfer_cb, ctx, EDGETPU_TIMEOUT_MS);

  if (g_sentai_tpu_trace) {
    sentai_logf("usb-ctrl", "submit bm=%02x req=%u val=%04x idx=%04x len=%u",
                setupPacket->bmRequestType, setupPacket->bRequest,
                setupPacket->wValue, setupPacket->wIndex,
                setupPacket->wLength);
  }

  int rc = libusb_submit_transfer(transfer);
  if (rc != 0) {
    free_async_ctx(ctx);
    libusb_free_transfer(transfer);
    sentai_logf("tpu-posix",
                "ctrl fail bm=%02x req=%u val=%04x idx=%04x len=%u rc=%d",
                setupPacket->bmRequestType, setupPacket->bRequest,
                setupPacket->wValue, setupPacket->wIndex,
                setupPacket->wLength, rc);
    return usb_status_from_libusb(rc);
  }

  uint64_t deadline_us = monotonic_us() +
                         (uint64_t)(EDGETPU_TIMEOUT_MS + 500) * 1000ULL;
  while (!ctx->completed) {
    pump_libusb_once(inst->usb_ctx);
    (void)xSemaphoreTake(sema, 0);
    if (monotonic_us() >= deadline_us) break;
    posix_sleep_ms(1);
  }
  if (!ctx->completed) {
    sentai_logf("tpu-posix",
                "ctrl timeout bm=%02x req=%u val=%04x idx=%04x len=%u",
                setupPacket->bmRequestType, setupPacket->bRequest,
                setupPacket->wValue, setupPacket->wIndex,
                setupPacket->wLength);
    libusb_cancel_transfer(transfer);
    deadline_us = monotonic_us() + 250000ULL;
    while (!ctx->completed) {
      pump_libusb_once(inst->usb_ctx);
      (void)xSemaphoreTake(sema, 0);
      if (monotonic_us() >= deadline_us) break;
      posix_sleep_ms(1);
    }
  }

  if (!ctx->completed) {
    ctx->done = NULL;
    usb_status_t st = kStatus_USB_TransferFailed;
    if (callbackFn) callbackFn(callbackParam, buffer, 0, st);
    return st;
  }

  usb_status_t st = ctx->status;
  uint32_t n = ctx->transferred;
  if ((setupPacket->bmRequestType & LIBUSB_ENDPOINT_IN) &&
      buffer && setupPacket->wLength > 0 && st == kStatus_USB_Success) {
    uint32_t copy_n = n;
    if (copy_n > setupPacket->wLength) copy_n = setupPacket->wLength;
    memcpy(buffer, s_control_buf + LIBUSB_CONTROL_SETUP_SIZE, copy_n);
  }
  free_async_ctx(ctx);
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
