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

#include "libs/audio/audio_service.h"

#include <cstdio>
#include <memory>

#include "libs/base/check.h"

namespace coralmicro {
namespace {
enum class MessageType : uint8_t {
  kAddCallback,
  kRemoveCallback,
  kStop,
};

struct Message {
  MessageType type;
  QueueHandle_t queue;
  union {
    struct {
      void* ctx;
      AudioService::Callback fn;
    } add;

    struct {
      int id;
    } remove;
  };
};

struct Cb {
  int id;
  void* ctx;
  AudioService::Callback fn;
};

bool EraseCallbackById(std::vector<Cb>& callbacks, int id) {
  auto it = std::find_if(std::begin(callbacks), std::end(callbacks),
                         [id](const auto& cb) { return cb.id == id; });
  if (it == std::end(callbacks)) return false;
  callbacks.erase(it);
  return true;
}
}  // namespace

AudioReader::AudioReader(AudioDriver* driver, const AudioDriverConfig& config)
    : driver_(driver), dma_buffer_size_ms_(config.dma_buffer_size_ms) {
  printf("[AudioReader] Creating: dma_buffer_ms=%d\r\n",
         config.dma_buffer_size_ms);
  const auto dma_buffer_size_samples = config.dma_buffer_size_samples();
  buffer_.resize(dma_buffer_size_samples);
  printf("[AudioReader] Buffer size: %zu samples\r\n", dma_buffer_size_samples);

  printf("[AudioReader] Creating ring buffer: size=%zu, trigger=%zu\r\n",
         dma_buffer_size_samples * config.num_dma_buffers,
         dma_buffer_size_samples);
  ring_buffer_.Create(
      /*xBufferSize=*/dma_buffer_size_samples * config.num_dma_buffers,
      /*xTriggerLevel=*/dma_buffer_size_samples);
  CHECK(ring_buffer_.Ok());

  printf("[AudioReader] Enabling audio driver\r\n");
  driver->Enable(config, this, Callback);
  printf("[AudioReader] Created successfully\r\n");
}

AudioReader::~AudioReader() {
  printf("[AudioReader] Destroying, disabling driver\r\n");
  driver_->Disable();
}

size_t AudioReader::FillBuffer() {
  auto received_size = ring_buffer_.Receive(
      buffer_.data(), buffer_.size(), pdMS_TO_TICKS(2 * dma_buffer_size_ms_));
  if (received_size != buffer_.size()) {
    ++underflow_count_;
    printf("[AudioReader] FillBuffer underflow #%lu: got %zu/%zu\r\n",
           static_cast<unsigned long>(underflow_count_), received_size,
           buffer_.size());
  }
  return received_size;
}

void AudioReader::Callback(void* ctx, const int32_t* buf, size_t size) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  auto* self = static_cast<AudioReader*>(ctx);
  auto sent_size =
      self->ring_buffer_.SendFromISR(buf, size, &xHigherPriorityTaskWoken);
  if (size != sent_size) {
    ++self->overflow_count_;
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

AudioService::AudioService(AudioDriver* driver, const AudioDriverConfig& config,
                           int task_priority, int drop_first_samples_ms)
    : driver_(driver),
      config_(config),
      drop_first_samples_(
          MsToSamples(config.sample_rate, drop_first_samples_ms)),
      queue_(xQueueCreate(5, sizeof(Message))) {
  printf("[AudioService] Creating: priority=%d, drop_first_ms=%d\r\n",
         task_priority, drop_first_samples_ms);
  CHECK(queue_);
  CHECK(xTaskCreate(StaticRun, "audio_service", configMINIMAL_STACK_SIZE * 30,
                    this, task_priority, &task_) == pdPASS);
  printf("[AudioService] Task created successfully\r\n");
}

AudioService::~AudioService() {
  printf("[AudioService] Destroying, sending stop message\r\n");
  Message msg{};
  msg.type = MessageType::kStop;
  CHECK(xQueueSendToBack(queue_, &msg, portMAX_DELAY) == pdTRUE);

  while (eTaskGetState(task_) != eSuspended) taskYIELD();
  vTaskDelete(task_);

  vQueueDelete(queue_);
  printf("[AudioService] Destroyed\r\n");
}

int AudioService::AddCallback(void* ctx, AudioService::Callback fn) {
  printf("[AudioService] AddCallback: ctx=%p\r\n", ctx);
  Message msg{};
  msg.type = MessageType::kAddCallback;
  msg.queue = xQueueCreate(1, sizeof(int));
  msg.add.ctx = ctx;
  msg.add.fn = fn;
  CHECK(msg.queue);
  CHECK(xQueueSendToBack(queue_, &msg, portMAX_DELAY) == pdTRUE);

  int id;
  CHECK(xQueueReceive(msg.queue, &id, portMAX_DELAY) == pdTRUE);
  vQueueDelete(msg.queue);
  printf("[AudioService] AddCallback: assigned id=%d\r\n", id);
  return id;
}

bool AudioService::RemoveCallback(int id) {
  printf("[AudioService] RemoveCallback: id=%d\r\n", id);
  Message msg{};
  msg.type = MessageType::kRemoveCallback;
  msg.queue = xQueueCreate(1, sizeof(int));
  msg.remove.id = id;
  CHECK(msg.queue);
  CHECK(xQueueSendToBack(queue_, &msg, portMAX_DELAY) == pdTRUE);

  int found;
  CHECK(xQueueReceive(msg.queue, &found, portMAX_DELAY) == pdTRUE);
  vQueueDelete(msg.queue);
  printf("[AudioService] RemoveCallback: id=%d, found=%d\r\n", id, found);
  return found;
}

void AudioService::StaticRun(void* param) {
  static_cast<const AudioService*>(param)->Run();
  vTaskSuspend(nullptr);
}

void AudioService::Run() const {
  printf("[AudioService] Run: task started\r\n");
  std::vector<Cb> callbacks;
  callbacks.reserve(3);

  std::vector<int> callbacks_to_remove;
  callbacks_to_remove.reserve(3);

  std::unique_ptr<AudioReader> reader;

  int id_counter = 0;

  Message msg;
  while (true) {
    const auto timeout_ticks = callbacks.empty() ? portMAX_DELAY : 0;
    if (xQueueReceive(queue_, &msg, timeout_ticks) == pdTRUE) {
      switch (msg.type) {
        case MessageType::kAddCallback: {
          int id = id_counter++;
          printf("[AudioService] Run: adding callback id=%d (total=%zu)\r\n",
                 id, callbacks.size() + 1);
          callbacks.push_back({id, msg.add.ctx, msg.add.fn});
          CHECK(xQueueSendToBack(msg.queue, &id, portMAX_DELAY) == pdTRUE);
        } break;

        case MessageType::kRemoveCallback: {
          printf("[AudioService] Run: removing callback id=%d\r\n",
                 msg.remove.id);
          int found = EraseCallbackById(callbacks, msg.remove.id);
          CHECK(xQueueSendToBack(msg.queue, &found, portMAX_DELAY) == pdTRUE);
        } break;
        case MessageType::kStop:
          printf("[AudioService] Run: stop received\r\n");
          return;
      }
      continue;
    }

    if (callbacks.empty()) {
      if (reader) {
        printf("[AudioService] Run: no callbacks, destroying reader\r\n");
        reader.reset();
      }
      continue;
    }

    if (!reader) {
      printf("[AudioService] Run: creating AudioReader, "
             "drop_first=%zu samples\r\n", drop_first_samples_);
      reader = std::make_unique<AudioReader>(driver_, config_);
      reader->Drop(drop_first_samples_);
      printf("[AudioService] Run: AudioReader ready\r\n");
    }

    // Blocks until buffer is full or timeout.
    auto size = reader->FillBuffer();

    callbacks_to_remove.clear();
    for (const auto& cb : callbacks)
      if (!cb.fn(cb.ctx, reader->Buffer().data(), size))
        callbacks_to_remove.push_back(cb.id);

    for (int id : callbacks_to_remove) {
      printf("[AudioService] Run: callback id=%d requested removal\r\n", id);
      EraseCallbackById(callbacks, id);
    }

    if (callbacks.empty()) {
      printf("[AudioService] Run: all callbacks removed, destroying reader\r\n");
      reader.reset();
    }
  }
}

LatestSamples::LatestSamples(size_t num_samples)
    : mutex_(xSemaphoreCreateMutex()), samples_(num_samples) {
  printf("[LatestSamples] Creating: num_samples=%zu\r\n", num_samples);
  CHECK(mutex_);
}

LatestSamples::~LatestSamples() {
  printf("[LatestSamples] Destroying\r\n");
  vSemaphoreDelete(mutex_);
}

}  // namespace coralmicro
