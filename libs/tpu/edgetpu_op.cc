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

#include "libs/tpu/edgetpu_op.h"

#include "libs/tpu/edgetpu_manager.h"
#include "third_party/tflite-micro/tensorflow/lite/c/common.h"

extern "C" volatile uint16_t g_sentai_tpu_invoke_fail_code;

namespace coralmicro {
namespace {
void* CustomOpInit(TfLiteContext* context, const char* buffer, size_t length) {
  return EdgeTpuManager::GetSingleton()->RegisterPackage(buffer, length);
}

void CustomOpFree(TfLiteContext* context, void* buffer) {}

TfLiteStatus CustomOpPrepare(TfLiteContext* context, TfLiteNode* node) {
  if (node->user_data == nullptr) return kTfLiteError;
  // Reserve the host scratch buffer wide models need for activation spill
  // (BASE_ADDRESS_SCRATCH DMAs).  Must happen here -- the arena request API
  // is Prepare-only.  No-op for models that keep activations on-chip.
  EdgeTpuPackage* package = static_cast<EdgeTpuPackage*>(node->user_data);
  return EdgeTpuManager::GetSingleton()->PrepareScratch(package, context);
}

/* Stage tag set by EdgeTpuExecutable::Invoke when one of the USB
 * transfers (params/inputs/instructions/outputs) fails.  We log it
 * via the firmware error-stream after Invoke returns kTfLiteError so
 * the failing stage is visible alongside other E:CCCC:V codes. */
TfLiteStatus CustomOpInvoke(TfLiteContext* context, TfLiteNode* node) {
  EdgeTpuPackage* package = static_cast<EdgeTpuPackage*>(node->user_data);
  TfLiteStatus rc = EdgeTpuManager::GetSingleton()->Invoke(package, context, node);
  if (rc != kTfLiteOk) {
    uint16_t code = ::g_sentai_tpu_invoke_fail_code;
    if (code != 0) printf("E:%04X:0\r\n", (unsigned)code);
  }
  return rc;
}
}  // namespace

TfLiteRegistration* RegisterCustomOp() {
  static TfLiteRegistration registration = {
      CustomOpInit,
      CustomOpFree,
      CustomOpPrepare,
      CustomOpInvoke,
  };
  return &registration;
}
}  // namespace coralmicro
