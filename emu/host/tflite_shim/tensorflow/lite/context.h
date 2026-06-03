// Minimal TensorFlow Lite context shim for host tools that only use the
// libedgetpu public device-management API.  Do not use this for interpreter
// execution; it only provides the declarations needed by edgetpu.h.
#ifndef SENTAI_EMU_HOST_TFLITE_CONTEXT_SHIM_H_
#define SENTAI_EMU_HOST_TFLITE_CONTEXT_SHIM_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum TfLiteStatus {
  kTfLiteOk = 0,
  kTfLiteError = 1,
} TfLiteStatus;

typedef enum TfLiteExternalContextType {
  kTfLiteEdgeTpuContext = 1,
} TfLiteExternalContextType;

typedef struct TfLiteContext TfLiteContext;
typedef struct TfLiteRegistration TfLiteRegistration;

typedef struct TfLiteExternalContext {
  TfLiteExternalContextType type;
  TfLiteStatus (*Refresh)(TfLiteContext* context,
                          struct TfLiteExternalContext* external_context);
} TfLiteExternalContext;

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_EMU_HOST_TFLITE_CONTEXT_SHIM_H_
