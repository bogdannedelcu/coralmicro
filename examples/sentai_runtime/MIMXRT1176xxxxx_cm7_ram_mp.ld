/*
** ###################################################################
**     Processors:          MIMXRT1176AVM8A_cm7
**                          MIMXRT1176CVM8A_cm7
**                          MIMXRT1176DVMAA_cm7
**
**     Compiler:            GNU C Compiler
**     Reference manual:    IMXRT1170RM, Rev E, 12/2019
**     Version:             rev. 0.1, 2018-03-05
**     Build:               b200828
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* HEAP_SIZE was 16 MB; reduced to 14 MB on 2026-05-18 to make room for
 * sentai_fr frame/event/scalar pools in .sdram_bss.  Newlib malloc still
 * gets 14 MB which is enormous for an M7 firmware (MP has its own 1 MB
 * GC heap in .sdram_bss, FreeRTOS has its own ucHeap). */
HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x00E00000;
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x0400;
RPMSG_SHMEM_SIZE = DEFINED(__use_shmem__) ? 0x2000 : 0;
NCACHE_SIZE = 0x8000;

/*
 * Valid memory regions:
 * 0x00000000 - 0x0003FFFF (256KB)
 * 0x20000000 - 0x2003FFFF (256KB)
 * 0x20240000 - 0x202BFFFF (512KB)
 * 0x202C0000 - 0x2033FFFF (512KB)
 */

/* Specify the memory areas */
/* 2026-04-22 OCRAM layout: merged OCRAM1 (0x20240000) + OCRAM2 into a
   single 1 MB contiguous region so a 786 KB yolo_1 tensor ping-pong
   buffer can live here instead of SDRAM.  RPMSG shared window moved
   to the END of OCRAM2 (0x2033E000) to free the middle.

   Why OCRAM and not DTCM: tensor buffers are read by USB EHCI DMA
   master, not CPU; OCRAM has its own bus path to the USB/EDMA
   crossbar that does NOT go through SEMC (the SDRAM contention
   bottleneck identified in the 2026-04-22 staged isolation).  DTCM
   is M7-private tightly-coupled memory — EHCI DMA can't reach it. */
MEMORY
{
  m_interrupts          (RX)  : ORIGIN = 0x00000800, LENGTH = 0x00000400
  m_text                (RX)  : ORIGIN = 0x00000c00, LENGTH = 0x0003F400
  m_ncache              (RW)  : ORIGIN = 0x20000000, LENGTH = NCACHE_SIZE
  m_data                (RW)  : ORIGIN = 0x20000000 + NCACHE_SIZE, LENGTH = 0x00040000 - NCACHE_SIZE
  m_ocram               (RX)  : ORIGIN = 0x20240000, LENGTH = 0x00100000 - RPMSG_SHMEM_SIZE
  rpmsg_sh_mem          (RW)  : ORIGIN = 0x20340000 - RPMSG_SHMEM_SIZE, LENGTH = RPMSG_SHMEM_SIZE
  m_heap                (RW)  : ORIGIN = 0x80000000, LENGTH = HEAP_SIZE
  m_sdram               (RX)  : ORIGIN = 0x80000000 + HEAP_SIZE, LENGTH = 0x02000000 - HEAP_SIZE
  m_ncamera             (RW)  : ORIGIN = 0x82000000, LENGTH = 0x01800000
}

/* Define output sections */
SECTIONS
{
  __NCACHE_REGION_START = ORIGIN(m_ncache);
  __NCACHE_REGION_SIZE  = LENGTH(m_ncache);

  __NCAMERA_REGION_START = ORIGIN(m_ncamera);
  __NCAMERA_REGION_SIZE  = LENGTH(m_ncamera);

  __RPMSG_SH_MEM_START = ORIGIN(rpmsg_sh_mem);
  __RPMSG_SH_MEM_SIZE  = LENGTH(rpmsg_sh_mem);

  /* NOINIT section for rpmsg_sh_mem */
  .noinit_rpmsg_sh_mem (NOLOAD) : ALIGN(4)
  {
     __RPMSG_SH_MEM_START__ = .;
     *(.noinit.$rpmsg_sh_mem*)
     . = ALIGN(4) ;
     __RPMSG_SH_MEM_END__ = .;
  } > rpmsg_sh_mem

  /* section for storing the secondary core image */
  .core1_code :
  {
     . = ALIGN(4) ;
    KEEP (*(.core1_code))
     *(.core1_code*)
     . = ALIGN(4) ;
  } > m_sdram

  /* The startup code goes first into internal RAM */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    __Vectors = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
  } > m_interrupts

  /* .a71ch / .curl / .mbedtls / .wiced / .httpsrv / .edgefast_bluetooth_*
   * sections were declared in the original Coral linker template but
   * those static libs are NOT link-listed in target_link_libraries for
   * sentai_runtime (see CMakeLists.txt).  arm-none-eabi-size confirmed
   * each section was 0 bytes in the final ELF.  Removed 2026-04-25 to
   * keep the linker minimal and reflect the actual build composition.
   * If you re-enable bluetooth / TLS / HTTPS server in the future,
   * restore the corresponding section blocks from the upstream Coral
   * linker (libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld). */

  /* .lwip — moved OCRAM → SDRAM 2026-04-22 to free space for the
     TPU tensor arena (1 MB OCRAM).  Network stack is not on the
     per-invoke hot path. */
  .lwip :
  {
     . = ALIGN(4);
     __lwip_start = .;
     *liblibs_nxp_rt1176-sdk_lwip.a:*(.text .text*)
     *liblibs_nxp_rt1176-sdk_lwip.a:*(.data .data*)
     *liblibs_nxp_rt1176-sdk_lwip.a:*(.rodata .rodata*)
     *liblibs_nxp_rt1176-sdk_lwip_httpd.a:*(.text .text*)
     *liblibs_nxp_rt1176-sdk_lwip_httpd.a:*(.data .data*)
     *liblibs_nxp_rt1176-sdk_lwip_httpd.a:*(.rodata .rodata*)
     *liblibs_nxp_rt1176-sdk_lwip_mdns.a:*(.text .text*)
     *liblibs_nxp_rt1176-sdk_lwip_mdns.a:*(.data .data*)
     *liblibs_nxp_rt1176-sdk_lwip_mdns.a:*(.rodata .rodata*)
     *liblibs_base-m7_http_server.a:*(.text .text*)
     *liblibs_base-m7_http_server.a:*(.data .data*)
     *liblibs_base-m7_http_server.a:*(.rodata .rodata*)
     __lwip_end = .;
     . = ALIGN(4);
  } > m_sdram

  .tensorflow :
  {
     . = ALIGN(4);
     __tensorflow_start = .;
     *liblibs_tensorflow-m7.a:*(.text .text*)
     *liblibs_tensorflow-m7.a:*(.data .data*)
     *liblibs_tensorflow-m7.a:*(.rodata .rodata*)
     *liblibs_kissfft-m7.a*:*(.text .text*)
     *liblibs_kissfft-m7.a*:*(.data .data*)
     *liblibs_kissfft-m7.a*:*(.rodata .rodata*)
     __tensorflow_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* CMSIS-DSP TransformFunctions + CommonTables -- ~30 KB of FFT
   * code/tables added 2026-05-05 for flow_phase_corr.cc.  Routed to
   * SDRAM to avoid m_text overflow.  FFT runs at ~30 fps from a
   * FreeRTOS task (NOT in ISR), so SDRAM instruction-fetch latency
   * is acceptable -- single arm_cfft_f32 dominated by twiddle data
   * accesses, not branch jitter.
   *
   * Match by SECTION NAME (function-sections puts each function in
   * its own .text.<symbol> subsection).  Filename-based match doesn't
   * work because GNU ld treats the archive member name as the full
   * stored path including CMakeFiles/.../obj prefix, not basename. */
  .cmsis_dsp :
  {
     . = ALIGN(4);
     __cmsis_dsp_start = .;
     *(.text.arm_cfft_f32 .text.arm_cfft_init_f32 .text.arm_cfft_q*)
     *(.text.arm_cfft_radix*_f32 .text.arm_cfft_radix*_butterfly_f32)
     *(.text.arm_cfft_radix8by*_f32)
     *(.text.arm_bitreversal2 .text.arm_bitreversal_32 .text.arm_bitreversal*_f32)
     *(.text.arm_radix8_butterfly_f32)
     *(.rodata.armBitRevIndexTable_64 .rodata.armBitRevIndexTable*)
     *(.rodata.twiddleCoef_64 .rodata.twiddleCoef_*)
     *(.rodata.arm_cfft_sR_f32_len*)
     *(.rodata.arm_cfft_sR_q*)
     __cmsis_dsp_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* USB Host placed in OCRAM (was SDRAM, 2026-04-22 TPU pipeline
   * sprint).  Rationale: under camera pipeline load, PXP DMA writes
   * SDRAM at ~40 MB/s.  When USB_OTG2 IOC fires, the M7 must fetch
   * the ISR + host-task code to process the completion.  If that
   * code lives in SDRAM, the fetch stalls behind PXP bus traffic
   * → IOC handling delayed → our 50 ms sema timeout fires for
   * ~39% of URBs during pipeline (measured via
   * sentai.diag.async_stats()).  Moving to OCRAM (on-chip SRAM,
   * not behind SEMC) eliminates SDRAM contention for USB code
   * fetch.  Cost: ~15 KB extra OCRAM (usb_host_ehci.c 6.4 KB text
   * + usb_host_hci.c 1.4 KB + hub + devices + small BSS).  Free
   * after moving libjpeg to SDRAM earlier this sprint. */
  .usb_host :
  {
     . = ALIGN(4);
     __usb_host_start = .;
     *usb_host_ehci.c.obj(.text .text*)
     *usb_host_ehci.c.obj(.data .data*)
     *usb_host_ehci.c.obj(.rodata .rodata*)
     *usb_host_hci.c.obj(.text .text*)
     *usb_host_hci.c.obj(.data .data*)
     *usb_host_hci.c.obj(.rodata .rodata*)
     *usb_host_devices.c.obj(.text .text*)
     *usb_host_devices.c.obj(.data .data*)
     *usb_host_devices.c.obj(.rodata .rodata*)
     *usb_host_hub.c.obj(.text .text*)
     *usb_host_hub.c.obj(.data .data*)
     *usb_host_hub.c.obj(.rodata .rodata*)
     *usb_host_hub_app.c.obj(.text .text*)
     *usb_host_hub_app.c.obj(.data .data*)
     *usb_host_hub_app.c.obj(.rodata .rodata*)
     __usb_host_end = .;
     . = ALIGN(4);
  } > m_ocram

  /* .libjpeg: moved OCRAM → SDRAM 2026-04-22 (TPU pipeline sprint).
     Frees ~103 KB OCRAM for future tensor ping-pong relocation.
     JPEG encode/decode is a non-hot path (camera snapshot, web UI)
     — SDRAM bus latency does not affect TPU/camera cadence. */
  .libjpeg :
  {
     . = ALIGN(4);
     __libjpeg_start = .;
     *liblibs_libjpeg.a:*(.text .text*)
     *liblibs_libjpeg.a:*(.data .data*)
     *liblibs_libjpeg.a:*(.rodata .rodata*)
     __libjpeg_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* .micropython: moved OCRAM → SDRAM 2026-04-22 (TPU pipeline sprint).
     MicroPython interpreter hot loop doesn't dominate frame-time
     budget; the large size (~208 KB) was the biggest OCRAM consumer.
     Freeing it lets tensor ping-pong buffers live in OCRAM. */
  .micropython :
  {
     . = ALIGN(4);
     __micropython_start = .;
     *liblibmicropython.a:*(.text .text*)
     *liblibmicropython.a:*(.data .data*)
     *liblibmicropython.a:*(.rodata .rodata*)
     __micropython_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* libm math functions — moved OCRAM → SDRAM 2026-04-22 to free
     space for the TPU output tensor relocation. */
  .libm :
  {
     . = ALIGN(4);
     *libm.a:*(.text .text*)
     *libm.a:*(.data .data*)
     *libm.a:*(.rodata .rodata*)
     . = ALIGN(4);
  } > m_sdram

  /* .audio + .shine: moved OCRAM → SDRAM 2026-04-22 (TPU pipeline
     sprint).  Not in the TPU hot path; freed ~10 KB OCRAM to hold
     the relocated NXP USB host code that MUST run fast. */
  .audio :
  {
     . = ALIGN(4);
     __audio_start = .;
     *liblibs_audio_freertos.a:*(.text .text*)
     *liblibs_audio_freertos.a:*(.data .data*)
     *liblibs_audio_freertos.a:*(.rodata .rodata*)
     __audio_end = .;
     . = ALIGN(4);
  } > m_sdram

  .shine :
  {
     . = ALIGN(4);
     __shine_start = .;
     *libshine.a:*(.text .text*)
     *libshine.a:*(.data .data*)
     *libshine.a:*(.rodata .rodata*)
     __shine_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* .aifes — moved OCRAM → SDRAM 2026-04-22 to free space for the
     TPU output tensor relocation.  Not on the per-invoke hot path. */
  .aifes :
  {
     . = ALIGN(4);
     __aifes_start = .;
     *liblibs_aifes_m7.a:*(.text .text*)
     *liblibs_aifes_m7.a:*(.data .data*)
     *liblibs_aifes_m7.a:*(.rodata .rodata*)
     *liblibs_aifes_task_m7.a:*(.text .text*)
     *liblibs_aifes_task_m7.a:*(.data .data*)
     *liblibs_aifes_task_m7.a:*(.rodata .rodata*)
     __aifes_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* .sentai_slow: moved OCRAM → SDRAM 2026-04-22 (TPU pipeline sprint).
     Contains tracker/link/mesh/httpd/tfl_bridge — none are on the
     per-invoke hot path.  Frees ~63 KB OCRAM for relocations. */
  .sentai_slow :
  {
     . = ALIGN(4);
     __sentai_slow_start = .;
     *sentai_tracker.cc.obj(.text .text*)
     *sentai_tracker.cc.obj(.data .data*)
     *sentai_tracker.cc.obj(.rodata .rodata*)
     *sentai_link.cc.obj(.text .text*)
     *sentai_link.cc.obj(.data .data*)
     *sentai_link.cc.obj(.rodata .rodata*)
     *sentai_mesh.cc.obj(.text .text*)
     *sentai_mesh.cc.obj(.data .data*)
     *sentai_mesh.cc.obj(.rodata .rodata*)
     *libmeshtastic_pb.a:*(.text .text*)
     *libmeshtastic_pb.a:*(.data .data*)
     *libmeshtastic_pb.a:*(.rodata .rodata*)
     *libnanopb.a:*(.text .text*)
     *libnanopb.a:*(.data .data*)
     *libnanopb.a:*(.rodata .rodata*)
     *sentai_httpd.cc.obj(.text .text*)
     *sentai_httpd.cc.obj(.data .data*)
     *sentai_httpd.cc.obj(.rodata .rodata*)
     *sentai_tfl_bridge.cc.obj(.text .text*)
     *sentai_tfl_bridge.cc.obj(.data .data*)
     *sentai_tfl_bridge.cc.obj(.rodata .rodata*)
     *sentai_slow_bridge.cc.obj(.text .text*)
     *sentai_slow_bridge.cc.obj(.data .data*)
     *sentai_slow_bridge.cc.obj(.rodata .rodata*)
     *sentai_crazy.cc.obj(.text .text*)
     *sentai_crazy.cc.obj(.data .data*)
     *sentai_crazy.cc.obj(.rodata .rodata*)
     /* Task #44 (2026-05-16): sentai.crazy.pose_* — CRTP LOG state machine.
        Cold-path (mission-driven, ≤10 Hz subscribe/poll). */
     *sentai_crazy_log.cc.obj(.text .text*)
     *sentai_crazy_log.cc.obj(.data .data*)
     *sentai_crazy_log.cc.obj(.rodata .rodata*)
     /* ObjectsPlan L2 (2026-05-13): persistent world-frame object map.
        Pure cold-path data layer, no per-frame work — same SDRAM
        treatment as sentai_tracker.cc.  Per [[itcm-budget]]. */
     *sentai_objects.cc.obj(.text .text*)
     *sentai_objects.cc.obj(.data .data*)
     *sentai_objects.cc.obj(.rodata .rodata*)
     /* ObjectsPlan L3 (2026-05-14): H3-indexed place gallery + Uber H3
        library.  Both cold-path: the gallery is queried at ~1 Hz from
        the explore SM, never on the per-frame TPU path.  H3 lib is
        ~14 KB of .text — keep ITCM headroom by routing here. */
     *sentai_places.cc.obj(.text .text*)
     *sentai_places.cc.obj(.data .data*)
     *sentai_places.cc.obj(.rodata .rodata*)
     *liblibh3_arm.a:*(.text .text*)
     *liblibh3_arm.a:*(.data .data*)
     *liblibh3_arm.a:*(.rodata .rodata*)
     /* ObjectsPlan L4 (2026-05-14): sentai.servo action layer skeleton.
        Pure FSM + 16-entry trace ring, called at <50 Hz from mission.
        Cold-path data layer; SDRAM matches L2/L3 pattern. */
     *sentai_servo.cc.obj(.text .text*)
     *sentai_servo.cc.obj(.data .data*)
     *sentai_servo.cc.obj(.rodata .rodata*)
     /* ObjectsPlan L5 (2026-05-15): inverse-depth EKF landmark lifter.
        Per-tracklet 1-state EKF; called at ≤30 Hz from detection_task
        after tracker.update().  Math is all float (FPU native), no
        ISR, no HW — fits SDRAM cold-path budget. */
     *sentai_object_lifter.cc.obj(.text .text*)
     *sentai_object_lifter.cc.obj(.data .data*)
     *sentai_object_lifter.cc.obj(.rodata .rodata*)
     /* ObjectsPlan L6 (2026-05-15): sentai.explore mission FSM.
        Pure FSM wrapping L4 servo + L5 lifter; operator-driven via
        REPL/radio, <50 Hz tick rate.  Cold-path; SDRAM matches L4/L5
        pattern.  Captures .rodata string literals (state names) to
        keep m_text margin. */
     *sentai_explore.cc.obj(.text .text*)
     *sentai_explore.cc.obj(.data .data*)
     *sentai_explore.cc.obj(.rodata .rodata*)
     /* ObjectsPlan OP-S6-W1 (2026-05-17): sentai.calib Kabsch.
        One-shot at takeoff (~15 ms); Jacobi 3x3 SVD + FxUser JSON
        persist.  Cold-path; .sdram_text matches the L4/L5/L6 pattern.
        Captures .rodata (default R_B_C constants + JSON format
        strings) to keep ITCM headroom intact per [[itcm-budget]]. */
     *sentai_calib.cc.obj(.text .text*)
     *sentai_calib.cc.obj(.data .data*)
     *sentai_calib.cc.obj(.rodata .rodata*)
     /* ObjectsPlan OP-S6-W3 (2026-05-17): sentai.aruco detector + PnP.
        Per-frame detect ~5-10 ms on M7 (estimated from s111 building
        blocks).  Routed to .sdram_text — preprocessing buffers live
        in .sdram_bss (~150 KB).  Detect runs in camera_bridge_recv /
        detection_task; readers via slot snapshot. */
     *sentai_aruco.cc.obj(.text .text*)
     *sentai_aruco.cc.obj(.data .data*)
     *sentai_aruco.cc.obj(.rodata .rodata*)
     /* OP-S10-W14 sentai.calib autotune + worker task — VPE
        forwarder + Aström-Hägglund relay state machine + hold
        controller.  Cold-path (~30 Hz mission tick).  Route to
        .sdram_text per [[itcm-budget]] (.text overflowed 504 B
        after T18 IPPE_SQUARE port). */
     *sentai_calib_autotune.cc.obj(.text .text*)
     *sentai_calib_autotune.cc.obj(.data .data*)
     *sentai_calib_autotune.cc.obj(.rodata .rodata*)
     *sentai_calib_task.cc.obj(.text .text*)
     *sentai_calib_task.cc.obj(.data .data*)
     *sentai_calib_task.cc.obj(.rodata .rodata*)
     /* Track A places (s139+, 2026-05-16): PHOG descriptor compute.
        Cold-path ~1 Hz, ~10 ms on M7; .sdram_text per [[itcm-budget]]. */
     *sentai_phog.cc.obj(.text .text*)
     *sentai_phog.cc.obj(.data .data*)
     *sentai_phog.cc.obj(.rodata .rodata*)
     /* Track A places (s140, 2026-05-16): GIST-lite descriptor.
        Cold-path ~1 Hz, ~1-2 ms on M7 post-decimation; .sdram_text. */
     *sentai_gist.cc.obj(.text .text*)
     *sentai_gist.cc.obj(.data .data*)
     *sentai_gist.cc.obj(.rodata .rodata*)
     /* Track A places (s161, 2026-05-17): HSV 16h×4s histogram.
        Slot-native uint8[64] (no quantize step needed).  Cold-path
        ~150 µs on M7; .sdram_text per [[itcm-budget]]. */
     *sentai_hsv.cc.obj(.text .text*)
     *sentai_hsv.cc.obj(.data .data*)
     *sentai_hsv.cc.obj(.rodata .rodata*)
     /* Camera-frame slot infrastructure (Phase 1a foundation,
        2026-05-17).  Pure storage + atomic-publish; called from
        PrepTask context in detection_task.cc (future integration).
        .sdram_text per [[itcm-budget]]. */
     *sentai_prep.cc.obj(.text .text*)
     *sentai_prep.cc.obj(.data .data*)
     *sentai_prep.cc.obj(.rodata .rodata*)
     /* OP-S10-W11-T3 (Phase 1c, 2026-05-17): SlamTask perception
        loop.  Cold path (~200 µs/frame); no hot-loop work here, so
        .sdram_text is the natural placement per [[itcm-budget]]. */
     *slam_task.cc.obj(.text .text*)
     *slam_task.cc.obj(.data .data*)
     *slam_task.cc.obj(.rodata .rodata*)
     /* OP-S10-W11-T3.1 (2026-05-17): sentai_health.cc — module
        health state machine.  Called at ≤30 Hz aggregate from all
        SUBSYS_* call sites; no per-frame hot work.  Move out of
        m_text to free room for ITCM-resident hot paths. */
     *sentai_health.cc.obj(.text .text*)
     *sentai_health.cc.obj(.data .data*)
     *sentai_health.cc.obj(.rodata .rodata*)
     /* OP-S10-W12 (2026-05-18): sentai.safety state machine + worker.
        Worker ticks at 6.6 Hz off ArUco frames; no ISR / per-pixel work.
        Cold-path; .sdram_text to keep ITCM hot. */
     *sentai_safety.cc.obj(.text .text*)
     *sentai_safety.cc.obj(.data .data*)
     *sentai_safety.cc.obj(.rodata .rodata*)
     *sentai_safety_task.cc.obj(.text .text*)
     *sentai_safety_task.cc.obj(.data .data*)
     *sentai_safety_task.cc.obj(.rodata .rodata*)
     /* OP-S10-W13 (2026-05-18): sentai.fr Flight Recorder.  Producers
        do brief mutex+memcpy (≤ a few KHz aggregate worst case); the
        drain task is 50 Hz with PGM writes outside the lock.  No ISR /
        no per-pixel work — .sdram_text per [[itcm-budget]]. */
     *sentai_fr.cc.obj(.text .text*)
     *sentai_fr.cc.obj(.data .data*)
     *sentai_fr.cc.obj(.rodata .rodata*)
     *sentai_fr_task.cc.obj(.text .text*)
     *sentai_fr_task.cc.obj(.data .data*)
     *sentai_fr_task.cc.obj(.rodata .rodata*)
     /* MP bindings (2026-05-18 cleanup): all cold-path REPL/script
        glue.  Pulled out of ITCM to make room for hot-path additions.
        HAL bindings are the biggest single contributor; safety/fr are
        small but kept here for consistency. */
     *modsentai_hal.cc.obj(.text .text*)
     *modsentai_hal.cc.obj(.data .data*)
     *modsentai_hal.cc.obj(.rodata .rodata*)
     *modsentai_safety.c.obj(.text .text*)
     *modsentai_safety.c.obj(.data .data*)
     *modsentai_safety.c.obj(.rodata .rodata*)
     *modsentai_fr.c.obj(.text .text*)
     *modsentai_fr.c.obj(.data .data*)
     *modsentai_fr.c.obj(.rodata .rodata*)
     __sentai_slow_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* .ramfunc — TIME-CRITICAL functions placed in ITCM for
     single-cycle instruction fetch.  Captures functions tagged
     __attribute__((section(".ramfunc"))) from any archive,
     including liblibs_camera_freertos.a (which is otherwise
     placed in .camera SDRAM section below).  Must come BEFORE
     .camera so the wildcard match here wins.
     Build #905+: CSI_IRQHandler is the first .ramfunc client —
     placement matters because the ISR fires at sensor rate AND
     contains the early-VBLANK MUX flip.  Single-cycle ITCM fetch
     vs. ~200 ns SDRAM fetch saves time critical for landing the
     flip in the brief sensor VBLANK window. */
  .ramfunc :
  {
     . = ALIGN(4);
     __ramfunc_start = .;
     *(.ramfunc .ramfunc.*)
     __ramfunc_end = .;
     . = ALIGN(4);
  } > m_text

  /* Build #958 audit: modsentai_*.c are #include'd into modsentai.c
     which compiles to ONE .obj inside liblibmicropython.a; that
     library is already routed to SDRAM via the .micropython
     section above.  Same for micropython_embed sources.  No
     additional section needed for MP bindings.  Cold-path code
     in sentai_runtime.cc that should leave ITCM is tagged
     individually via __attribute__((section(".sdram_text"))) and
     collected here. */
  .sdram_text :
  {
     . = ALIGN(4);
     __sdram_text_start = .;
     *(.sdram_text .sdram_text.*)
     __sdram_text_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* .cdc_ncm + .camera — moved OCRAM → SDRAM 2026-04-22 (small
     sections, tolerable SDRAM latency since not per-invoke hot). */
  .cdc_ncm :
  {
     . = ALIGN(4);
     __cdc_ncm_start = .;
     *liblibs_cdc_ncm_freertos.a:*(.text .text*)
     *liblibs_cdc_ncm_freertos.a:*(.data .data*)
     *liblibs_cdc_ncm_freertos.a:*(.rodata .rodata*)
     __cdc_ncm_end = .;
     . = ALIGN(4);
  } > m_sdram

  .camera :
  {
     . = ALIGN(4);
     __camera_start = .;
     *liblibs_camera_freertos.a:*(.text .text*)
     *liblibs_camera_freertos.a:*(.data .data*)
     *liblibs_camera_freertos.a:*(.rodata .rodata*)
     __camera_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* edgefast_bluetooth + ethermind libraries removed 2026-04-25 (not
   * link-listed for sentai_runtime, all sections were 0 bytes). */

  /* .filex / .levelx — FileX FAT filesystem + LevelX NAND wear
     leveling (Eclipse ThreadX, MIT).  Phase 1 of LFS->FileX user-
     partition migration.  Both libs are cold-path: only invoked
     during sentai.diag.fx_smoke() runs and (eventually) the
     LfsUser-replacement helpers.  SDRAM placement keeps ITCM/OCRAM
     reserved for the TPU+camera hot path. */
  .filex :
  {
     . = ALIGN(4);
     __filex_start = .;
     *liblibs_filex.a:*(.text .text*)
     *liblibs_filex.a:*(.data .data*)
     *liblibs_filex.a:*(.rodata .rodata*)
     __filex_end = .;
     . = ALIGN(4);
  } > m_sdram

  .levelx :
  {
     . = ALIGN(4);
     __levelx_start = .;
     *liblibs_levelx.a:*(.text .text*)
     *liblibs_levelx.a:*(.data .data*)
     *liblibs_levelx.a:*(.rodata .rodata*)
     __levelx_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* fx_nand_driver.cc + fx_user_fs.cc are also cold-path; route them
     to SDRAM rather than spending precious ITCM on smoke-only code. */
  .fx_glue :
  {
     . = ALIGN(4);
     __fx_glue_start = .;
     *fx_nand_driver.cc.obj(.text .text*)
     *fx_nand_driver.cc.obj(.data .data*)
     *fx_nand_driver.cc.obj(.rodata .rodata*)
     *fx_user_fs.cc.obj(.text .text*)
     *fx_user_fs.cc.obj(.data .data*)
     *fx_user_fs.cc.obj(.rodata .rodata*)
     __fx_glue_end = .;
     . = ALIGN(4);
  } > m_sdram

  /* The program code and other data goes into internal RAM */
  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    KEEP(*(.rodata.debug))
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  /* Bluetooth KEEP tables removed 2026-04-25 (no bluetooth libs linked). */

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  __etext = .;    /* define a global symbol at end of code */
  __DATA_ROM = .; /* Symbol is used by startup for data initialization */

  __VECTOR_RAM = ORIGIN(m_interrupts);
  __RAM_VECTOR_TABLE_SIZE_BYTES = 0x0;

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    *(m_usb_dma_init_data)
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    *(.got)
    *(.got*)
    *(.got.plt)
    *(.got.plt*)
    KEEP(*(.jcr*))
    . = ALIGN(4);
    _net_buf_pool_list = .;
    KEEP(*(SORT(._net_buf_pool*)))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data
  __NDATA_ROM = __DATA_ROM + (__data_end__ - __data_start__);
  .ncache.init : AT(__NDATA_ROM)
  {
    __noncachedata_start__ = .;   /* create a global symbol at ncache data start */
    *(NonCacheable.init)
    . = ALIGN(4);
    __noncachedata_init_end__ = .;   /* create a global symbol at initialized ncache data end */
  } > m_ncache
  . = __noncachedata_init_end__;
  .ncache :
  {
    *(NonCacheable)
    . = ALIGN(4);
    __noncachedata_end__ = .;     /* define a global symbol at ncache data end */
  } > m_ncache
  __SDRAM_ROM = __NDATA_ROM + (__noncachedata_init_end__ - __noncachedata_start__);
  .sdram_data : AT(__SDRAM_ROM)
  {
    . = ALIGN(4);
    __sdram_data_start__ = .;
    KEEP(*(.sdram_data*))
    . = ALIGN(4);
    __sdram_data_end__ = .;
  } > m_sdram

  /* Help text embedded directly in SDRAM (VMA=LMA, written by elfloader). */
  .help_data :
  {
    . = ALIGN(4);
    KEEP(*(.help_data*))
    . = ALIGN(4);
  } > m_sdram

  /* Persistent boot-mode flag — survives NVIC_SystemReset / WDOG reset
   * because DTC-RAM contents are preserved by warm reset on RT1176.
   * Cleared only by hardware POR (reset button / power cycle).
   * Placed BEFORE .bss so the startup BSS-zero loop skips it. */
  .noinit_boot_persist (NOLOAD) : ALIGN(4)
  {
    __BOOT_PERSIST_START__ = .;
    *(.noinit.$boot_persist*)
    . = ALIGN(4);
    __BOOT_PERSIST_END__ = .;
  } > m_data

  /* Storage-mode debug log buffer — 16 KB in SDRAM, NOLOAD so the
   * startup BSS-zero loop does not clear it across warm reset.  The
   * MSC handler appends events here while the FileX volume is unmounted
   * (storage mode); on next default-mode boot the buffer is flushed to
   * /log/storage_debug.log.  See sentai_storage_log() in
   * libs/base/fx_user_fs.cc. */
  .sdram_storage_log (NOLOAD) : ALIGN(8)
  {
    __SDRAM_STORAGE_LOG_START__ = .;
    *(.sdram_storage_log .sdram_storage_log.*)
    . = ALIGN(8);
    __SDRAM_STORAGE_LOG_END__ = .;
  } > m_sdram

  /* Crash-survivable boot log buffer (2026-04-29).  16 KB in SDRAM
   * NOLOAD — preserved across NVIC_SystemReset / WDOG reset on this
   * silicon, so a firmware that crashes BEFORE the FileX user volume
   * mounts does not lose its boot trace.  Header carries magic +
   * check + length so the next boot can detect a valid pre-crash
   * buffer and flush it to /log/boot_prev.log.  Cleared by hardware
   * POR (button / power cycle) like the rest of SDRAM. */
  .sdram_boot_log (NOLOAD) : ALIGN(8)
  {
    __SDRAM_BOOT_LOG_START__ = .;
    *(.sdram_boot_log .sdram_boot_log.*)
    . = ALIGN(8);
    __SDRAM_BOOT_LOG_END__ = .;
  } > m_sdram

  /* Phase-correlation breadcrumb ring (added 2026-05-05).  ~256 bytes
   * NOLOAD so its content survives NVIC_SystemReset / WDOG reset.
   * Inspected at app_main boot via sentai_flow_phase_corr_dump_bc()
   * to isolate where flow_phase_corr_compute() crashed in the
   * previous run.  Cleared only by hardware POR. */
  .sdram_phase_corr_bc (NOLOAD) : ALIGN(8)
  {
    __SDRAM_PHASE_CORR_BC_START__ = .;
    *(.sdram_phase_corr_bc .sdram_phase_corr_bc.*)
    . = ALIGN(8);
    __SDRAM_PHASE_CORR_BC_END__ = .;
  } > m_sdram

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(m_usb_dma_noninit_data)
    /* Exclude libraries whose .bss is intentionally placed in SDRAM
     * (sdram_bss section below) to keep the M7 DTCM .bss small.  The
     * crypto / edgefast_bluetooth / ethermind libs are NOT linked for
     * sentai_runtime — kept here as harmless no-ops in case they are
     * re-enabled in the future. */
    *(EXCLUDE_FILE(
        *liblibmicropython.a
        *liblibs_audio_freertos.a
        *liblibs_nxp_rt1176-sdk_lwip_httpd.a
        *liblibs_base-m7_http_server.a
        *sentai_httpd.cc.obj
    ) .bss)
    *(EXCLUDE_FILE(
        *liblibmicropython.a
        *liblibs_audio_freertos.a
        *liblibs_nxp_rt1176-sdk_lwip_httpd.a
        *liblibs_base-m7_http_server.a
        *sentai_httpd.cc.obj
    ) .bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data
  __DATA_END = __SDRAM_ROM;
  text_end = ORIGIN(m_text) + LENGTH(m_text);
  ASSERT(__DATA_END <= text_end, "region m_text overflowed with text and data")

  .sdram_bss (NOLOAD) :
  {
    . = ALIGN(4);
    __sdram_bss_start__ = .;
    *(.sdram_bss*)
    /* micropython_bss */
    *liblibmicropython.a:*(.bss .bss*)
    /* audio_bss — AudioDriver/Service internals are not DMA-critical */
    *liblibs_audio_freertos.a:*(.bss .bss*)
    /* httpd_bss — HTTP server state (no DMA) */
    *liblibs_nxp_rt1176-sdk_lwip_httpd.a:*(.bss .bss*)
    *liblibs_base-m7_http_server.a:*(.bss .bss*)
    *sentai_httpd.cc.obj(.bss .bss*)
    /* edgefast_bluetooth + ethermind + httpsrv .bss removed 2026-04-25
     * (libs not link-listed for sentai_runtime). */
    . = ALIGN(4);
    __sdram_bss_end__ = .;
  } > m_sdram

  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    __HeapLimit = .;
    __heap_limit = .; /* Add for _sbrk */
  } > m_heap

  .stack :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
  } > m_data

  .ncamera :
  {
    __noncachecameradata_start__ = .;
    *(NonCacheableCamera)
    . = ALIGN(64);
    __noncachecameradata_end__ = .;
  } > m_ncamera

  /* V22 .tpu_input — 786 KB tensor input buffer in OCRAM so USB EHCI
     SendInputs reads via AXBS (no SEMC contention with CSI DMA).
     Critical for the 42 FPS pipeline ceiling. */
  .tpu_input (NOLOAD) :
  {
    . = ALIGN(64);
    __tpu_input_start__ = .;
    *(.tpu_input)
    . = ALIGN(64);
    __tpu_input_end__ = .;
  } > m_ocram
  ASSERT(__tpu_input_end__ <= ORIGIN(m_ocram) + LENGTH(m_ocram),
         "ERROR: .tpu_input overflows m_ocram into rpmsg_sh_mem")

  /* OP-S10-W16 ablation 2026-05-19: .ocram_bss for the apples-to-
   * apples M7-vs-M4 thresh comparison.  Lands in m_ocram AFTER
   * .tpu_input (~230 KB free here).  NOLOAD = BSS, zero-init at
   * boot via crt. */
  .ocram_bss (NOLOAD) :
  {
    . = ALIGN(32);
    __ocram_bss_start__ = .;
    *(.ocram_bss*)
    . = ALIGN(32);
    __ocram_bss_end__ = .;
  } > m_ocram
  ASSERT(__ocram_bss_end__ <= ORIGIN(m_ocram) + LENGTH(m_ocram),
         "ERROR: .ocram_bss overflows m_ocram (TPU staging collision)")

  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data) + LENGTH(m_data);
  __StackLimit = __StackTop - STACK_SIZE;
  PROVIDE(__stack = __StackTop);

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

