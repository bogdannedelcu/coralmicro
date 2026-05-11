// sentai_tpu_shim.h — shared declaration of the TPU C entry points.
//
// Same pattern as sentai_pxp_shim.h / sentai_fft_shim.h:
//   ARM target: implementations live in sentai_runtime.cc and drive the
//               on-board Apex EdgeTPU over USB via libedgetpu+tflite-micro.
//   SIM target: implementations live in sim/sim_tpu_shim.c and forward
//               to a host-side pycoral helper daemon over a Unix socket
//               (/tmp/sentai_tpu.sock).  When a Coral USB stick is
//               plugged in, pycoral uses it; otherwise it falls back to
//               CPU TFLite Runtime (slower but fully functional).
//
// MicroPython REPL contract is identical on both targets: the same
// modsentai_tpu globals_table maps the same MP_QSTRs to the same C
// entry points listed below, so any user .py script that runs on the
// hardware also runs in SIM unchanged.
//
// All functions return a non-zero error code on failure (typically -1
// for "not loaded yet" or "out of range"); zero or a positive value
// otherwise.  Pointer accessors return NULL on failure.

#ifndef SENTAI_TPU_SHIM_H_
#define SENTAI_TPU_SHIM_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Single-slot (default slot 0) — current-generation API ───────────
int   sentai_tpu_is_ready(void);
int   sentai_tpu_num_outputs(void);
int   sentai_tpu_get_output_size(int idx);
const void* sentai_tpu_get_output_data(int idx);
int   sentai_tpu_get_output_num_dims(int idx);
int   sentai_tpu_get_output_dim(int idx, int dim);
int   sentai_tpu_get_output_type(int idx);
int   sentai_tpu_input_quant(float* scale, int32_t* zero_point);
int   sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);
int   sentai_tpu_input_type(void);

int   sentai_tpu_invoke(void);
int   sentai_tpu_invoke_with_input(uint8_t* input_buf);

// ── Model load (SIM-only entry point — ARM uses a different path) ───
// path is a host filesystem path on SIM (e.g.,
// "/home/bogdan/work/coralmicro/models/mobilenet_v2_324_quant_bayered_3channel_edgetpu.tflite").
// Returns 0 on success.
int   sentai_tpu_load_model(const char* path);

// ── Multi-slot (Phase 1 multi-EP firmware on ARM) ───────────────────
int   sentai_tpu_slot_count(void);
int   sentai_tpu_slot_ready(int slot);
int   sentai_tpu_load_model_slot(int slot, const char* path);
int   sentai_tpu_invoke_slot(int slot);
int   sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf);
int   sentai_tpu_num_outputs_slot(int slot);
int   sentai_tpu_get_output_size_slot(int slot, int idx);
const void* sentai_tpu_get_output_data_slot(int slot, int idx);
int   sentai_tpu_get_output_num_dims_slot(int slot, int idx);
int   sentai_tpu_get_output_dim_slot(int slot, int idx, int dim);
int   sentai_tpu_get_output_type_slot(int slot, int idx);
int   sentai_tpu_output_quant_slot(int slot, int idx,
                                    float* scale, int32_t* zero_point);
int   sentai_tpu_set_input_slot(int slot,
                                 const uint8_t* data, size_t bytes);
uint32_t sentai_tpu_output_hash_slot(int slot);

// ── Tensor metadata + camera→tensor pipeline glue ───────────────────
// Returns 0 + populates the input tensor metadata when a model is loaded
// (single-slot path).  *buf points into the live tensor arena.
int   sentai_get_tensor_info(int* w, int* h, int* ch,
                              uint8_t** buf, int* type, int* zp);

// On ARM these wrap PXP+CSI to fill the input tensor from the active
// camera frame.  On SIM they're stubbed (Phase 5.6 wires them via the
// camera_bridge_recv ring once we have a TPU input downscaler).
int   sentai_cam_to_tensor(void);
int   sentai_cam_to_tensor_ex(const char* save_path, int quality);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_TPU_SHIM_H_
