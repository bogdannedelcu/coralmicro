/*
 * apex_firmware.h — indirection layer so edgetpu_dfu_task.cc can upload
 * either of the two EdgeTPU firmware blobs (single_ep or multi_ep)
 * without code changes.  The CMake flag SENTAI_TPU_MULTI_EP selects
 * which C file (apex_latest_*_bin.c) gets compiled into the DFU task
 * library — this header maps the runtime-facing symbols onto whichever
 * one is present.
 */
#ifndef LIBS_TPU_APEX_FIRMWARE_H_
#define LIBS_TPU_APEX_FIRMWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SENTAI_TPU_MULTI_EP) && SENTAI_TPU_MULTI_EP
extern unsigned char apex_latest_multi_ep_bin[];
extern unsigned int  apex_latest_multi_ep_bin_len;
#define apex_firmware_bin     apex_latest_multi_ep_bin
#define apex_firmware_bin_len apex_latest_multi_ep_bin_len
#else
extern unsigned char apex_latest_single_ep_bin[];
extern unsigned int  apex_latest_single_ep_bin_len;
#define apex_firmware_bin     apex_latest_single_ep_bin
#define apex_firmware_bin_len apex_latest_single_ep_bin_len
#endif

#ifdef __cplusplus
}
#endif

#endif  /* LIBS_TPU_APEX_FIRMWARE_H_ */
