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

#include "libs/base/console_m7.h"

#include <unistd.h>

#include <cstdio>
#include <functional>

#include "libs/base/check.h"
#include "libs/base/ipc_m7.h"
#include "libs/base/ipc_message_buffer.h"
#include "libs/base/mutex.h"
#include "libs/base/tasks.h"
#include "libs/usb/usb_device_task.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/utilities/debug_console/fsl_debug_console.h"

using namespace std::placeholders;

extern "C" int DbgConsole_SendDataReliable(uint8_t*, size_t);
extern "C" int _write(int handle, char* buffer, int size) {
  if ((handle != STDOUT_FILENO) && (handle != STDERR_FILENO)) {
    return -1;
  }

  // Convert bare \n to \r\n for USB/UART terminals.
  // Use a stack buffer to avoid heap allocation/fragmentation.
  // Worst case: every byte is a bare \n → 2× size, capped at 512.
  char stack_buf[512];
  char* out = buffer;
  int out_len = size;

  // Quick scan: does any \n lack a preceding \r?
  bool needs_patch = false;
  for (int i = 0; i < size; ++i) {
    if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
      needs_patch = true;
      break;
    }
  }
  if (needs_patch) {
    int j = 0;
    int cap = (int)sizeof(stack_buf);
    for (int i = 0; i < size && j < cap - 1; ++i) {
      if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
        stack_buf[j++] = '\r';
      }
      if (j < cap) stack_buf[j++] = buffer[i];
    }
    out = stack_buf;
    out_len = j;
  }

  coralmicro::ConsoleM7::GetSingleton()->Write(out, out_len);
  return size;
}

extern "C" int _read(int handle, char* buffer, int size) {
  if (handle != STDIN_FILENO) {
    return -1;
  }

  int bytes_read = coralmicro::ConsoleM7::GetSingleton()->Read(buffer, size);
  return bytes_read > 0 ? bytes_read : -1;
}

namespace coralmicro {

uint8_t ConsoleM7::m4_console_buffer_storage_[kM4ConsoleBufferSize]
    __attribute__((section(".noinit.$rpmsg_sh_mem")));

void ConsoleM7::EmergencyWrite(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int len =
      vsnprintf(emergency_buffer_.data(), emergency_buffer_.size(), fmt, ap);
  va_end(ap);

  DbgConsole_SendDataReliable(
      reinterpret_cast<uint8_t*>(emergency_buffer_.data()), len);
  DbgConsole_Flush();
  cdc_acm_.Transmit(reinterpret_cast<uint8_t*>(emergency_buffer_.data()), len);
}

void ConsoleM7::Write(char* buffer, int size) {
  if (!tx_task_) {
    return;
  }
  ConsoleMessage msg = {
      size,
      new uint8_t[size],
  };
#ifdef BLOCKING_PRINTF
  msg.semaphore = xSemaphoreCreateBinaryStatic(&msg.semaphore_storage);
#endif
  memcpy(msg.str, buffer, size);
  xQueueSend(console_queue_, &msg, portMAX_DELAY);
#ifdef BLOCKING_PRINTF
  xSemaphoreTake(msg.semaphore, portMAX_DELAY);
  vSemaphoreDelete(msg.semaphore);
#endif
}

int ConsoleM7::Read(char* buffer, int size) {
  // If REPL target is USB, read from USB RX buffer
  if (repl_target_ == ReplTarget::kUsb) {
    int n = (usb_rx_available_ < (size_t)size) ? (int)usb_rx_available_ : size;
    if (n <= 0) return -1;
    for (int i = 0; i < n; i++) {
      buffer[i] = (char)usb_rx_buf_[usb_rx_read_];
      usb_rx_read_ = (usb_rx_read_ + 1) % kUsbRxBufferSize;
    }
    usb_rx_available_ -= n;
    return n;
  }

  if (!rx_task_) {
    return -1;
  }
  MutexLock lock(rx_mutex_);
  int bytes_to_return = std::min(size, static_cast<int>(rx_buffer_available_));

  if (!bytes_to_return) {
    return -1;
  }

  int bytes_to_read = bytes_to_return;
  if (rx_buffer_read_ > rx_buffer_write_) {
    memcpy(buffer, &rx_buffer_[rx_buffer_read_],
           kRxBufferSize - rx_buffer_read_);
    bytes_to_read -= kRxBufferSize - rx_buffer_read_;
    if (bytes_to_read) {
      memcpy(buffer + (bytes_to_return - bytes_to_read), &rx_buffer_[0],
             bytes_to_read);
    }
  } else {
    memcpy(buffer, &rx_buffer_[rx_buffer_read_], bytes_to_read);
  }
  rx_buffer_available_ -= bytes_to_return;
  rx_buffer_read_ = (rx_buffer_read_ + bytes_to_return) % kRxBufferSize;

  return bytes_to_return;
}

void ConsoleM7::M4ConsoleTaskFn(void* param) {
  IpcMessage m4_console_buffer_msg;
  m4_console_buffer_msg.type = IpcMessageType::kSystem;
  m4_console_buffer_msg.message.system.type =
      IpcSystemMessageType::kConsoleBufferPtr;
  m4_console_buffer_msg.message.system.message.console_buffer_ptr =
      GetM4ConsoleBufferPtr();
  IpcM7::GetSingleton()->SendMessage(m4_console_buffer_msg);

  size_t rx_bytes;
  char buf[16];
  while (true) {
    rx_bytes = xStreamBufferReceive(m4_console_buffer_->stream_buffer, buf,
                                    sizeof(buf), pdMS_TO_TICKS(10));
    if (rx_bytes > 0) {
      fwrite(buf, 1, rx_bytes, stdout);
    }
  }
}

// TODO(atv): At the moment, this only reads from DbgConsole, not USB.
void ConsoleM7::M7ConsoleTaskRxFn(void* param) {
  while (true) {
    uint8_t ch = static_cast<uint8_t>(DbgConsole_Getchar());
    MutexLock lock(rx_mutex_);
    assert(rx_buffer_write_ < kRxBufferSize);
    rx_buffer_[rx_buffer_write_] = ch;
    rx_buffer_write_ = (rx_buffer_write_ + 1) % kRxBufferSize;
    if (rx_buffer_write_ == rx_buffer_read_) {
      rx_buffer_read_ = (rx_buffer_read_ + 1) % kRxBufferSize;
      assert(rx_buffer_read_ < kRxBufferSize);
    } else {
      ++rx_buffer_available_;
      if (rx_buffer_available_ > kRxBufferSize) {
      }
      assert(rx_buffer_available_ <= kRxBufferSize);
    }
    // Signal data available for UART reads (serial or REPL poll)
    if (rx_sem_) xSemaphoreGive(rx_sem_);
  }
}

void ConsoleM7::SetLogPipe(StreamBufferHandle_t pipe) { log_pipe_ = pipe; }
void ConsoleM7::SetLogCallback(LogCallback cb) { log_callback_ = cb; }

void ConsoleM7::M7ConsoleTaskTxFn(void* param) {
  while (true) {
    ConsoleMessage msg;
    if (xQueueReceive(console_queue_, &msg, portMAX_DELAY) == pdTRUE) {
      // Mirror to TCP log pipe if registered; non-blocking — drops if full.
      StreamBufferHandle_t pipe = log_pipe_;
      if (pipe) {
        xStreamBufferSend(pipe, msg.str, msg.len, 0);
      }
      // Mirror to HTTP log callback if registered.
      LogCallback cb = log_callback_;
      if (cb) {
        cb(reinterpret_cast<const char*>(msg.str), msg.len);
      }
      // Route output to REPL target only, with retry on failure
      bool ok = false;
      for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (repl_target_ == ReplTarget::kUart) {
          DbgConsole_SendDataReliable(msg.str, msg.len);
          ok = true;
        } else {
          ok = cdc_acm_.Transmit(msg.str, msg.len);
          if (!ok) vTaskDelay(pdMS_TO_TICKS(10));
        }
      }
      delete[] msg.str;
#ifdef BLOCKING_PRINTF
      DbgConsole_Flush();
      xSemaphoreGive(msg.semaphore);
#endif
    }
  }
}

void usb_device_task(void* param) {
  while (true) {
    coralmicro::UsbDeviceTask::GetSingleton()->UsbDeviceTaskFn();
    taskYIELD();
  }
}

void ConsoleM7::Init(bool init_tx, bool init_rx) {
  m4_console_buffer_ =
      reinterpret_cast<IpcStreamBuffer*>(m4_console_buffer_storage_);
  m4_console_buffer_->stream_buffer = xStreamBufferCreateStatic(
      kM4ConsoleBufferBytes, 1, m4_console_buffer_->stream_buffer_storage,
      &m4_console_buffer_->static_stream_buffer);

  cdc_acm_.Init(
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_descriptor_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_interface_value(),
      coralmicro::UsbDeviceTask::GetSingleton()->next_interface_value(),
      std::bind(&ConsoleM7::UsbRxHandler, this, _1, _2));

  usb_rx_sem_ = xSemaphoreCreateBinary();
  CHECK(usb_rx_sem_);
  rx_sem_ = xSemaphoreCreateBinary();
  CHECK(rx_sem_);
  coralmicro::UsbDeviceTask::GetSingleton()->AddDevice(
      cdc_acm_.config_data(),
      std::bind(&coralmicro::CdcAcm::SetClassHandle, &cdc_acm_, _1),
      std::bind(&coralmicro::CdcAcm::HandleEvent, &cdc_acm_, _1, _2),
      cdc_acm_.descriptor_data(), cdc_acm_.descriptor_data_size());

  console_queue_ = xQueueCreate(64, sizeof(ConsoleMessage));
  CHECK(console_queue_);

  rx_mutex_ = xSemaphoreCreateMutex();
  CHECK(rx_mutex_);

  CHECK(xTaskCreate(usb_device_task, "usb_device_task",
                    configMINIMAL_STACK_SIZE * 10, nullptr,
                    kUsbDeviceTaskPriority, nullptr) == pdPASS);
  if (init_tx) {
    CHECK(xTaskCreate(StaticM7ConsoleTaskTxFn, "m7_console_task_tx",
                      configMINIMAL_STACK_SIZE * 10, nullptr,
                      kConsoleTaskPriority, &tx_task_) == pdPASS);
  }
  if (init_rx) {
    CHECK(xTaskCreate(StaticM7ConsoleTaskRxFn, "m7_console_task_rx",
                      configMINIMAL_STACK_SIZE * 10, nullptr,
                      kConsoleTaskPriority, &rx_task_) == pdPASS);
  }
  if (IpcM7::HasM4Application()) {
    CHECK(xTaskCreate(StaticM4ConsoleTaskFn, "m4_console_task",
                      configMINIMAL_STACK_SIZE * 10, nullptr,
                      kConsoleTaskPriority, nullptr) == pdPASS);
  }
}

IpcStreamBuffer* ConsoleM7::GetM4ConsoleBufferPtr() {
  return m4_console_buffer_;
}

// ===================== USB Serial Bridge =====================

void ConsoleM7::UsbRxHandler(const uint8_t* data, uint32_t len) {
  // Called from USB ISR context when data arrives on the CDC ACM bulk OUT EP.
  if (len == 0) return;
  // Buffer USB data when REPL reads from USB or USB serial mode is active
  if (repl_target_ != ReplTarget::kUsb && !usb_serial_mode_) return;

  for (uint32_t i = 0; i < len; i++) {
    if (usb_rx_available_ >= kUsbRxBufferSize) break;  // drop if full
    usb_rx_buf_[usb_rx_write_] = data[i];
    usb_rx_write_ = (usb_rx_write_ + 1) % kUsbRxBufferSize;
    ++usb_rx_available_;
  }
  // Wake up any task blocked in UsbRead()
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(usb_rx_sem_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool ConsoleM7::UsbSerialOpen() {
  if (repl_target_ != ReplTarget::kUart) return false;  // USB is used by REPL
  if (usb_serial_mode_) return true;  // already open

  // Flush USB RX buffer
  usb_rx_read_ = 0;
  usb_rx_write_ = 0;
  usb_rx_available_ = 0;
  xSemaphoreTake(usb_rx_sem_, 0);  // clear any pending signal

  usb_serial_mode_ = true;
  return true;
}

void ConsoleM7::UsbSerialClose() {
  usb_serial_mode_ = false;
  // Drain pending RX
  usb_rx_read_ = 0;
  usb_rx_write_ = 0;
  usb_rx_available_ = 0;
}

bool ConsoleM7::UsbTransmit(const uint8_t* buf, size_t len) {
  if (!usb_serial_mode_) return false;
  // CdcAcm::Transmit has a 512 byte limit per call, chunk if needed
  const uint8_t* p = buf;
  size_t remaining = len;
  while (remaining > 0) {
    size_t chunk = (remaining > 512) ? 512 : remaining;
    if (!cdc_acm_.Transmit(p, chunk)) return false;
    p += chunk;
    remaining -= chunk;
  }
  return true;
}

int ConsoleM7::UsbRead(uint8_t* buf, int max_size, int timeout_ms) {
  if (!usb_serial_mode_) return -1;

  // Wait for data if buffer is empty
  if (usb_rx_available_ == 0) {
    if (timeout_ms == 0) return 0;
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTake(usb_rx_sem_, ticks);
  }

  // Copy available data
  int n = (usb_rx_available_ < (size_t)max_size) ? (int)usb_rx_available_ : max_size;
  for (int i = 0; i < n; i++) {
    buf[i] = usb_rx_buf_[usb_rx_read_];
    usb_rx_read_ = (usb_rx_read_ + 1) % kUsbRxBufferSize;
  }
  usb_rx_available_ -= n;
  return n;
}

// ===================== REPL Target Switching =====================

void ConsoleM7::SetReplTarget(ReplTarget target) {
  if (target == repl_target_) return;

  // Close serial mode on the destination peripheral (it's about to become REPL)
  if (target == ReplTarget::kUsb && usb_serial_mode_) {
    UsbSerialClose();
  }
  if (target == ReplTarget::kUart && uart_serial_mode_) {
    UartSerialClose();
  }

  // Flush both RX buffers
  {
    MutexLock lock(rx_mutex_);
    rx_buffer_read_ = 0;
    rx_buffer_write_ = 0;
    rx_buffer_available_ = 0;
  }
  usb_rx_read_ = 0;
  usb_rx_write_ = 0;
  usb_rx_available_ = 0;
  if (usb_rx_sem_) xSemaphoreTake(usb_rx_sem_, 0);
  if (rx_sem_) xSemaphoreTake(rx_sem_, 0);

  repl_target_ = target;
}

// ===================== UART Serial Bridge =====================

bool ConsoleM7::UartSerialOpen() {
  if (repl_target_ != ReplTarget::kUsb) return false;  // UART is used by REPL
  if (uart_serial_mode_) return true;  // already open

  // Flush UART RX buffer
  {
    MutexLock lock(rx_mutex_);
    rx_buffer_read_ = 0;
    rx_buffer_write_ = 0;
    rx_buffer_available_ = 0;
  }
  if (rx_sem_) xSemaphoreTake(rx_sem_, 0);

  uart_serial_mode_ = true;
  return true;
}

void ConsoleM7::UartSerialClose() {
  if (!uart_serial_mode_) return;
  uart_serial_mode_ = false;

  // Flush buffer
  {
    MutexLock lock(rx_mutex_);
    rx_buffer_read_ = 0;
    rx_buffer_write_ = 0;
    rx_buffer_available_ = 0;
  }
}

bool ConsoleM7::UartTransmit(const uint8_t* buf, size_t len) {
  if (!uart_serial_mode_) return false;
  DbgConsole_SendDataReliable(const_cast<uint8_t*>(buf), len);
  return true;
}

int ConsoleM7::UartRead(uint8_t* buf, int max_size, int timeout_ms) {
  if (!uart_serial_mode_) return -1;

  // Wait for data if buffer is empty
  if (rx_buffer_available_ == 0) {
    if (timeout_ms == 0) return 0;
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTake(rx_sem_, ticks);
  }

  // Copy available data from rx_buffer_
  MutexLock lock(rx_mutex_);
  int n = (rx_buffer_available_ < (size_t)max_size) ? (int)rx_buffer_available_ : max_size;
  if (n <= 0) return 0;

  for (int i = 0; i < n; i++) {
    buf[i] = rx_buffer_[rx_buffer_read_];
    rx_buffer_read_ = (rx_buffer_read_ + 1) % kRxBufferSize;
  }
  rx_buffer_available_ -= n;
  return n;
}

int ConsoleM7::UartAvailable() const {
  return (int)rx_buffer_available_;
}

}  // namespace coralmicro
