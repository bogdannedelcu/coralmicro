# OP-S10-W25 — SentAI namespace gap research (what exists, what the hardware offers, what the literature suggests)

**WBS:** `OP-S10-W25` (planning work package, operator-requested 2026-10-31; registered in `ideas/wbs.md` under OP-S10 after W24).
**Status:** research + gap analysis only — no code, no renumbering.  Task names below are `OP-S10-W25-T<k>` *candidates*.
**Scope rules honoured:** hot paths in C/C++ (ITCM is at 86 % — `agent/arm_memory_budget.md`, default placement `.sdram_text`/`.sdram_bss`); MicroPython orchestrates only; no heavy data through MP; experiments under `examples/sentai_runtime/experiments/sNNN_*` (`s258` is already claimed by OP-S10-W24-T11; next free: `s259`).
**Sources read:** `examples/sentai_runtime/SENTAI_API.md`, `bindings/modsentai_*.c` (39 files), `modsentai.c` (module registration), `agent/agent.md` §8–§9, `agent/ov5640registers.md`, `agent/arm_memory_budget.md`, `libs/{camera,tpu,audio,base,lis2dux12,t5838}`, `third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/`, `third_party/st/lis2dux12-pid/lis2dux12_reg.h`, `ideas/objects_plan/09_namespace_audit.md`.

---

## A. What exists — the `sentai.*` namespace today

Registered in `examples/sentai_runtime/modsentai.c`: `version verbose help debug console run io rtos sdcard tpu fs camera usb uart mesh link crazy imu mic pipeline flow aifes kmeans pca anomaly dtw hmm rl slam objects places servo object_lifter calib markers safety fr explore tfl sys` (39 names).  `aruco` / `whycon` binding files exist but are reached through `sentai.markers` (`help.txt:223` calls them aliases).  `sentai.diag` / `sentai.sleep` appear in the 2026-05 audit (`09_namespace_audit.md` §19.1) but are **not** in today's `modsentai.c` registration table.

"Hot in C?" = is the per-frame / per-sample work done in C/C++ (Y), or is the module a pure data structure / algorithm whose cost is already small (alg), or a thin wrapper (W).

| Module (binding) | What it does | Hot in C? | C/C++ behind it | Hardware blocks |
|---|---|---|---|---|
| `camera` (`modsentai_camera.c`, 1051 l) | OV5640 init/stop, dual-sensor MUX select + ratio alternation, genlock/phase/pretrim/autotune (EXP-s251/s254), JPEG (RGB and native YUV paths), `to_tensor`, replay, prep slots, AEC/gain/ISP presets, test pattern, raw `reg_read/write` | Y | `libs/camera/camera.cc`, `camera_support.c` (CSI ISR in `.ramfunc`, GPT1-deferred MUX flip), `sentai_runtime.cc` (`sentai_cam_*`), `sentai_prep.cc`, `libjpeg` | MIPI CSI-2 RX + CSI, PXP (scaler, CSC1, rotate, YUV fmt), GPT1, LPI2C1/2, GPIO MUX, PMIC rails, SDRAM `m_ncamera` |
| `tpu` (`modsentai_tpu.c`, 785 l) | Model load (3 slots), invoke, outputs/quant, URB stats/trace, CSR error dump, recover, zero-copy/async input, instruction cache knobs | Y | `libs/tpu/edgetpu_{driver,manager,executable}.cc`, `usb_host_edgetpu.c`, EHCI in OCRAM `.usb_host`, TFLM | USB OTG2 EHCI host, OCRAM `.tpu_input` 900 KB, DTCM bulk staging 32 KB |
| `pipeline` (1433 l) | Camera→PXP→quant→TPU→NMS→SentAI-SORT continuous pipeline, per-cam slot routing, parity, auto-recovery (W24-T7), detections/tracks/events | Y | `detection_task.cc` (PrepTask/InferTask, eDMA memcpy), `sentai_tracker.cc` (BoT-SORT-derived 8-state KF), `sentai_prep.cc` | PXP, eDMA DMA0 ch, TPU, GPT1 µs timing |
| `flow` (412 l) | 80×60 PXP-downscaled gray, SAD/`__USADA8` block matching + CMSIS `arm_cfft_f32` phase correlation, body-frame transform, anchor mode, detail score, stretch | Y | `flow_task.cc`, `flow_phase_corr.cc`, `flow_shared.h`; `flow_task_m4.cc` historical | PXP, M7 SIMD/FPU, DWT cycle counter |
| `markers` (715 l) + `aruco`/`whycon` | ArUco (SIMD adaptive threshold, 22 ms) + WhyCon-lite (6 ms @ 4 markers), PnP, cam extrinsics, marker world map, drone pose, observation window | Y | `sentai_markers.cc`, `sentai_aruco.cc`, `sentai_whycon*.cc`, `sentai_svd3.cc` | M7 SIMD, OCRAM fill stack, PXP gray |
| `calib` (1373 l) | Kabsch/SVD `R_cam_to_body` + cam offset, INI persistence, Kp autotune, orientation bring-up task FSM | alg | `sentai_calib*.cc`, `sentai_calib_autotune.cc`, `sentai_calib_orientation_task.cc` | — (uses markers, crazy) |
| `servo` (420 l) | Action layer: arm/takeoff/move/go_to/hover/land, IBVS centroid command, marker-servo task FSM, SIM/CF2/PX4 backends | W | `sentai_servo.cc`, `sentai_servo_marker_task.cc` | — (UART radio via crazy) |
| `explore` (328 l) | Mission FSM (IDLE…LOST, 10 states), home, goto, tunables, trace | W | `sentai_explore.cc` | — |
| `objects` / `object_lifter` / `places` | Object map (TENTATIVE→CONFIRMED→STALE), inverse-depth lifter (Civera), H3 place graph with PHOG/GIST/HSV descriptors + SLAM task | alg / Y for descriptors | `sentai_objects.cc`, `sentai_object_lifter.cc`, `sentai_places.cc`, `sentai_phog.cc`, `sentai_gist.cc`, `sentai_hsv.cc`, `slam_task.cc`, `h3_gen/` | M7 FPU; GIST scratch `s_decim` in `.sdram_bss` |
| `slam` (1075 l) | Detection-based 2D EKF-SLAM, ≤ 64 landmarks, optional stereo range from `baseline*focal/disparity`, `imu_correct`, save/load | alg | `slam_task.cc`, binding | — |
| `safety` (125 l) | Abort triggers fed by ArUco/markers observations, SafetyTask ~100 ms tick | W | `sentai_safety.cc`, `sentai_safety_task.cc` | — |
| `fr` (117 l) | Frame/event recorder (events + scalars → SD), drain task | Y (ring) | `sentai_fr.cc`, `sentai_fr_task.cc`, pools in `.sdram_bss` | USDHC1 via FileX |
| `crazy` (870 l) | CRTP over CPX over UART2: arm/takeoff/land/hover/go_to, telemetry, LOG subscription, ext-pos/pose, flow inject | Y (UART ISR) | `sentai_crazy.cc`, `sentai_crazy_log.cc` | LPUART2 |
| `link` (482 l) / `mesh` (199 l) | MAVLink bridge (PX4 obstacle distance, VPE, flow) / Meshtastic detections | W | `sentai_link.cc`, `sentai_mesh.cc` (`.sentai_slow`) | LPUART |
| `imu` (177 l) | LIS2DUX12 init (100 Hz, ±2 g), read/degrees/radians, hardware double-tap task, raw reg access, bus scan | W | `libs/lis2dux12/lis2dux12.cc` over ST `lis2dux12-pid` | LPI2C, INT1 (INT2 N/C per `lis2dux12.h:10`) |
| `mic` (90 l) | PDM capture 16 kHz into RAM ring (`modsentai_hal.cc:639`), `level()`, MP3 via shine | Y (eDMA) | `libs/audio/audio_driver.cc` (PDM+eDMA), `third_party/shine` | PDM, eDMA DMA0 |
| `aifes` / `kmeans` / `pca` / `anomaly` / `dtw` / `hmm` / `rl` | On-device NN training (AIfES), clustering, PCA, Mahalanobis+CUSUM, DTW (IMU/mic scalar frames), HMM, Q/MAB/DQN | alg (`.aifes` in SDRAM) | `libs/aifes`, bindings | FPU |
| `tfl` (343 l) | TFLite-Micro CPU inference (`models/xor.yaml` only) | alg | `sentai_tfl_bridge.cc` | — |
| `fs` / `sdcard` / `usb` / `uart` / `io` / `rtos` / `sys` | FileX on SD, block device, MSC/NCM/CDC, LED/GPIO probe, tasks/heap/dmesg/suspend, reset/uid/peek/poke/health | W | `libs/base/fx_user_fs.cc`, `fx_sd_driver.cc`, `libs/sdcard`, `sentai_health.cc`, `sentai_dmesg.cc` | USDHC1, USB OTG1 device, WDOG1, SRC_GPR |

**Observations on the existing surface**

1. Every perception hot path is already in C with the right memory placement (`agent/arm_memory_budget.md` §"Rules").  New work must follow the same shape: C task + zero-copy slot + scalar/tuple MP accessor.
2. `sentai.mic` is a recorder, nothing more: no spectrum, no MFCC, no classifier; `dtw.record_add_mic` / `aifes.from_mic` consume the raw level scalar.  The PDM path (eDMA, 16 kHz) is the only audio compute today.
3. `sentai.imu` polls at 100 Hz from MP and uses one hardware engine (double-tap).  No FIFO, no free-fall, no spectrum.
4. The two OV5640s are **time-multiplexed through one CSI receiver** (never simultaneous); `paper/related_embedded_inference.md:96-110` and `paper/cam_switch.md:728-736` state this explicitly.  No stereo baseline is documented anywhere in the tree; `slam.update_stereo` takes a user-supplied baseline and has no HW validation.
5. The Edge TPU junction temperature is readable (`libs/tpu/edgetpu_driver.cc:2159 TpuDriver::GetTemperature`, `edgetpu_manager.cc:289`) but **not exposed** by `sentai.tpu` or `sentai.sys.health()`.
6. The Cortex-M4 is not in the build (`CMakeLists.txt:275-287`: flow SAD moved to M7 at build #1130; only a bare-metal ArUco cycle bench remains, OP-S10-W16 open).

---

## B. What the hardware offers that is unused or under-used

### B.1 RT1176 on-chip blocks (ground truth: `third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/`)

Usage column = files in `examples/sentai_runtime`, `libs/`, `apps/elf_loader` referencing the driver prefix (grep, 2026-10-31).

| Block | SDK driver | Used today | Where / how | Gap → one-line idea |
|---|---|---|---|---|
| PXP 2-D engine | `fsl_pxp` | **Yes** (scale, CSC1, rotate, YUV format) | `sentai_runtime.cc`, `libs/camera/camera.cc`, `aruco_bench.cc` | Unused PXP features: alpha-blend/overlay (AS surface), LUT, dither, multi-pass tiling — e.g. hardware ROI crop + rescale of a detected bbox to the TPU input size (zero-CPU "zoom-to-object"), or overlay of debug boxes into the JPEG preview |
| CSI + MIPI CSI-2 RX | `fsl_csi`, `fsl_mipi_csi2rx` | **Yes** (+ patches 0001/0003/0005) | `camera_support.c` ISR | CSI-2 virtual channels not routable by NXP driver (`ov5640registers.md:155-163`); FSIN frame-sync pin of the OV5640 unused — genlock is done by MUX-flip phase instead |
| eDMA / DMAMUX | `fsl_edma` | **Yes** (2 users) | `detection_task.cc:1195` dma_memcpy staging→tensor; `audio_driver.cc` PDM | Idle channels could do SDRAM→OCRAM prefetch of gray tiles for descriptors, or scatter-gather frame-to-SD recording without M7 copies |
| GPT1/GPT2 | `fsl_gpt` | **Yes** (GPT1 OCR2 deferred MUX flip; µs clock) | `camera_support.c:672`, `libs/base/timer.cc` | GPT2 free: hardware input-capture of an external sync pulse (drone IMU tick) for timestamping frames |
| PIT, QTMR, ENC, FlexPWM | `fsl_pit`, `fsl_qtmr`, `fsl_enc`, `fsl_pwm` | PIT/QTMR/ENC **no**; PWM only in `libs/base/pwm.cc` (W22-T3 holds FlexPWM1 pads low) | — | QTMR input capture = cheap precise timestamp of OV5640 VSYNC/strobe for sensor fusion; PWM = camera strobe/IR LED for active-illumination markers |
| XBAR / AOI | `fsl_xbara`, `fsl_aoi` | XBAR only in `pwm.cc` | — | Route VSYNC → GPT capture, or ADC trigger, without ISR |
| LPADC / ACMP / DAC | `fsl_lpadc`, `fsl_acmp`, `fsl_dac12` | LPADC only in `libs/base/analog.cc`, no SentAI caller | — | Battery/rail voltage of the deck (brownout prediction — see memory #1226), motor-current sense if wired |
| TMPSNS (die temperature) | `fsl_tempsensor` | Driver wrapped in `libs/base/tempsense.cc`, **no SentAI caller** | — | `sentai.sys.health()` field: M7 die temp + TPU temp → thermal throttling of invoke rate |
| CAAM (AES/SHA/RSA/ECC/RNG) | `fsl_caam` | **RNG only**, `libs/base/random.cc`, no SentAI caller | — | Hash-chained mission log (`sentai.fr`), signed calibration INI, attestation of firmware build id |
| PUF, Key manager, IEE, XRDC2, OCOTP | `fsl_puf`, `fsl_key_manager`, `fsl_iee`, `fsl_xrdc2`, `fsl_ocotp` | **No** (fuses only via `peek32`, EXP-s255) | — | Device-unique key without storing it; HAB secure boot is the natural next step after the s255 boot-fuse work |
| SNVS | `fsl_snvs_hp/lp` | RTC only (`timer.cc`), reset reason (`reset.cc`) | — | Tamper pins / secure RTC; battery-backed monotonic counter for mission numbering |
| SEMC (SDRAM), FlexSPI (NAND), uSDHC1 | `fsl_semc`, `fsl_flexspi`, `fsl_usdhc` | **Yes** | board init, LittleFS system, FileX on SD | uSDHC2 free; SDR104 / ADMA2 tuning of card writes for frame recording |
| ENET / ENET_QoS (1588) | `fsl_enet`, `fsl_enet_qos` | `libs/base/ethernet.cc` exists, **not linked** into `sentai_runtime` (no PHY on this board) | — | 1588 not reachable without PHY; time sync must come from GPT + CRTP timestamps instead |
| FlexCAN (CAN-FD) | `fsl_flexcan` | **No** | — | Would need transceiver; not relevant to the cf2 UART bridge |
| SAI / SPDIF / MQS / ASRC | `fsl_sai`, `fsl_mqs`, `fsl_asrc` | **No** (MQS clock only in `clock_config.c`) | — | MQS = PWM audio out on a GPIO: audible beacon/alarm from the deck with zero extra hardware |
| PDM | `fsl_pdm_edma` | **Yes** 16 kHz, 1 ch | `libs/audio` | 48 kHz available (`audio_driver.h:31`); hardware HPF/decimation filters configurable; T5838 AAD wake-on-sound driver present (`libs/t5838/t5838.h`) with **no caller** |
| FlexIO | `fsl_flexio_*` | **No** | — | Second camera interface (`flexio_camera`), extra UART for a second radio, or WS2812 LED bar |
| eLCDIF / LCDIFv2 / MIPI DSI | `fsl_elcdif`, `fsl_lcdifv2` | **No** | — | Not useful on a flying deck; skip |
| Cortex-M4 + MU/RPMSG + SEMA4 | `fsl_mu`, `fsl_sema4`, `libs/base/ipc*.cc` | SEMA4 yes (mutex); M4 image removed from build | `CMakeLists.txt:275` | OP-S10-W16 still open: candidates = IMU FIFO drain + vibration FFT, mic spectrogram, SD recorder — all bandwidth-light, latency-tolerant |
| FlexRAM re-partition | `fsl_flexram`, `fsl_soc_flexram_allocate` | **No** (default 256 KB ITCM / 256 KB DTCM, `MIMXRT1176xxxxx_cm7_ram_mp.ld:67-69`) | — | ITCM is the tightest region (86 %, `arm_memory_budget.md`); a 320/192 KB ITCM/DTCM split is a 64 KB ITCM gain if DTCM users (lwIP/NCM pools, bulk staging) still fit — needs the W15 A/B discipline |
| MECC / XECC | `fsl_mecc`, `fsl_xecc` | **No** | — | ECC on OCRAM/SDRAM for the thesis reliability angle (costs RAM) |
| CDOG, EWM, RTWDOG | — | WDOG1 only (`sentai_runtime.cc`, `watchdog.cc`) | — | Code-watchdog for control-flow integrity of the ISR path (anti-tamper) |
| GPC / SSARC low power | `fsl_gpc_3`, `fsl_ssarc` | **No** | — | Landed-idle power states; T5838 AAD + wake |
| M7 DWT / FPU / SIMD | CMSIS | **Yes** | `flow_task.cc`, `aruco_bench.cc`, `__USADA8` loops | CMSIS-DSP linked (`CMakeLists.txt:359`) but only `arm_cfft_f32` len-64 used; the rfft/MFCC/matrix kernels are free to use |

### B.2 OV5640 features (registers in `agent/ov5640registers.md`; touched registers grepped in `sentai_runtime.cc`, `camera_support.c`, `camera.cc`)

| Feature | Registers | State | Idea |
|---|---|---|---|
| AEC/AGC manual, target zones, gain ceiling | 0x3500-0x3503, 0x3A0F-0x3A1F, 0x3A18/19 | **Used** (`sentai_cam_aec_set`, `gain_ceiling_set`, `test_pattern` rails) | Per-frame exposure lock during marker tracking (AEC dominance note in memory) |
| Flip/mirror, SDE contrast/saturation, ISP presets | 0x3820/21, 0x5580-0x5586 | **Used** (`rotate`, `isp_preset`) | — |
| Test pattern, embedded data line | 0x503D, 0x501F/0x4707 | pattern used; embedded line **failed** (ISP overwrites) | — |
| Windowing / ROI crop + on-sensor scaling | 0x3800-0x380B, 0x5001 | Only through the NXP resolution table | Sensor-side "digital PTZ": crop to the tracked object at constant output size → no CSI timing change, 4× more pixels on target for free (must keep HTS/VTS, see D-PHY caveat `ov5640registers.md:244-281`) |
| 2×2 binning / sub-sampling | 0x3814/0x3815 | SXGA 2×2 binned mode only via driver table | Low-light binned VGA for night marker tracking |
| Night mode / band filter | 0x3A00, 0x3A05 | driver defaults only | Auto frame-rate drop in low light vs flow consistency — expose as a knob |
| Strobe / FREX, FSIN | 0x3B00-0x3B0C, FSIN pin | strobe reg appears only in the init table; FSIN not wired/used | Hardware flash sync for an IR LED (active markers); FSIN would be the "proper" genlock if a future board rev wires it |
| AWB manual gains, CMX, LENC, DPC | 0x3400-0x3406, 0x5180-0x5188, 0x5380-0x539B, 0x5800+ | init table only | AWB lock for colour-based HSV descriptors (`sentai_hsv.cc`) — colour descriptors are invalid under free-running AWB |
| Autofocus firmware (VCM) | 0x3022-0x3029, 0x8000 download | **No** | Only if the module has a VCM lens; current modules appear fixed-focus — verify before planning |

### B.3 Sensors other than the cameras

**IMU — LIS2DUX12** (`libs/lis2dux12/lis2dux12.h`; ST PID driver `third_party/st/lis2dux12-pid/lis2dux12_reg.h`).  3-axis accelerometer only (no gyro on the deck; `project_no_flow_deck_camera_imu_only.md`).  Driver calls used: `init_set, mode_set (100 Hz ±2 g), xl_data_get, outt_data_get, tap_config_set, wakeup_config_set, int_config_set, pin_int1/int2_route_set, all_sources_get`.  Available but **unused** in the PID header: ODR up to **800 Hz** (`LIS2DUX12_800Hz_HP`), hardware **FIFO** with watermark (`lis2dux12_fifo_*`), **free-fall** (`lis2dux12_ff_*`), **6D orientation** (`sixd_*`), significant-motion, tilt, step counter, and the on-sensor **Machine Learning Core** (`lis2dux12_mlc_*`, decision trees running inside the sensor).  Only INT1 is wired (`lis2dux12.h:10`); the binding comment in `modsentai_imu.c:88` still says INT2 — doc drift to fix.  Each `imu.read()` is one I2C transaction from MP at ≤ 100 Hz.

**Microphone — TDK T5838 PDM** (`libs/audio`, `libs/t5838`).  Used: 16 kHz mono into a RAM ring, `level()`, MP3 save.  Unused: 48 kHz, PDM hardware HPF/filter config, the T5838 **Always-Active-Detection** wake modes (driver written for board rev 2026-10, no caller), any spectral processing.  One microphone ⇒ **no direction-of-arrival is possible** (DoA needs ≥ 2 spatially separated channels); only level, spectrum, and classification.

**Edge TPU health**: `GetTemperature()` and the CSR error dump exist in `libs/tpu`; `urb_sync_stats`, `csr_errors`, attach/detach counters (W24) are exposed, temperature is not.

---

## C. What the literature suggests — candidate functionalities

Survey method: ~35 web searches (arXiv, IEEE Xplore, MDPI, ACM/IEEE SEC, DATE, Science Robotics, TU Delft, ETH-PBL/PULP, Coral docs), 2022-2026 preferred, canonical older anchors kept where nothing newer exists.  Every citation below was seen in a search result or fetched; "(snippet)" marks numbers taken from a search snippet rather than the full text.  None of these papers is already in `10_bibliography.md` (§20.1-20.6).

Legend — **Hot path**: C/C++ block that does the per-sample/per-frame work; **MP**: what MicroPython orchestrates; **Cost**: ITCM / SDRAM / compute estimate under the W15 rules (new code defaults to `.sdram_text`; ITCM only for a verified inner loop); **Dup**: overlaps an existing module.

### C.1 Audio (one PDM microphone)

| # | Functionality | Why it fits / hot path | MP role | Cost | Citation | Candidate task |
|---|---|---|---|---|---|---|
| C1 | **Acoustic motor/propeller anomaly** from one mic: 16 kHz PDM → CMSIS-DSP `arm_rfft_fast_f32` 512-pt log-mel frames → tiny FC autoencoder (MLPerf-Tiny AD class) on M7 (or TPU) → reconstruction-error score into `sentai.anomaly`/`sentai.safety` | PDM+eDMA already stream to RAM (`libs/audio`); FFT in `.sdram_text` (1-2 ms per 512-pt frame on M7); AE < 100 KB | `mic.spectrum()` scalar bands, threshold, trigger | SDRAM ~150 KB (model + mel bank); ITCM 0; ~3 % M7 at 30 frames/s | Liu, Chen, Zheng, *Audio-Based Fault Diagnosis for Quadrotors using CNN + Transfer Learning*, ACC 2020, arXiv:2003.02649; MicroNet-AD-S AUC 95.35 % on MIMII, 253 KB flash / 114 KB SRAM — Banbury et al., *MicroNets*, MLSys 2021, arXiv:2010.11267; MLPerf Tiny AD benchmark arXiv:2106.07597 | `OP-S10-W25-T1` `sentai.mic.spectrum()` + `sentai.mic.anomaly()` (EXP-s259+ bench on recorded motor audio) |
| C2 | **Keyword spotting** ("stop", "land") as a safety input | DS-CNN / MicroNet-KWS-S: 102 KB flash / 53 KB SRAM, 108 ms on an M7 @ 216 MHz → ~25-30 ms on our M7, or one TPU invoke | `mic.kws()` → `safety` abort | SDRAM ~200 KB; CPU 25 ms/word | Zhang et al., *Hello Edge*, arXiv:1711.07128; MicroNets arXiv:2010.11267 | low thesis value — demo / FW |
| C3 | **Sound-source direction**: **not possible** with one microphone (needs ≥ 2 channels; monaural DoA requires a learned source spectrum plus a direction-dependent scatterer) | — | — | — | El Badawy & Dokmanić, arXiv:1801.03740; Saxena & Ng, ICRA 2009 | none — state in docs |
| C4 | Ego-noise: with one mic the propeller noise is the *signal* (C1), not something to remove; arrays with 4-8 mics are what the MAV literature uses for enhancement | — | — | — | arXiv:2211.02690 (ICA 2022) | none |

### C.2 IMU-based state (LIS2DUX12 accelerometer only)

| # | Functionality | Why it fits / hot path | MP role | Cost | Citation | Candidate task |
|---|---|---|---|---|---|---|
| C5 | **Vibration spectrum / motor health**: ODR 800 Hz into the sensor FIFO (watermark IRQ on INT1), eDMA-free I2C burst read, CMSIS `arm_rfft_fast_f32` 256-pt per axis → blade-pass band energies | FIFO + 800 Hz exist in the PID driver and are unused (B.3); FFT < 0.5 ms; cf2 IMU over CRTP is only ~6 Hz so it cannot do this | `imu.spectrum()` tuple of band energies; thresholds; feed `anomaly`/`safety` | SDRAM ~8 KB; ITCM 0; < 1 % M7 | Ghalamchi, Jia, Mueller, *Real-Time Vibration-Based Propeller Fault Diagnosis for Multicopters*, IEEE/ASME T-Mech 2020 (snippet); MDPI Sensors 22(16):6037, 2022 (k-means on IMU vibration) | `OP-S10-W25-T2` `sentai.imu.fifo_start(odr)` + `spectrum()` |
| C6 | **Free-fall / collision watchdog** independent of the drone: on-sensor free-fall engine (`lis2dux12_ff_*`) + |a| > 3 g spike → `safety` abort | Zero CPU: sensor engines raise INT1; redundancy argument (two independent detectors) | `imu.freefall_start()`, `safety` hook | ~1 KB | Wang et al., *Air Bumper*, arXiv:2307.06101 (2023); Lew et al., *Contact Inertial Odometry*, arXiv:1909.00079; cf2 `sitaw.c` free-fall + supervisor tumble | `OP-S10-W25-T3` |
| C7 | Accelerometer-only flow derotation — **rejected**: needs a dynamics model and fails in hover; keep gyro rates from the cf2 EKF over CRTP (current design) | — | — | — | Fuller et al., Science Robotics 2022, DOI 10.1126/scirobotics.abq8184; Braber et al., IMAV 2018 | none (document as rejected) |

### C.3 Vision — geometry

| # | Functionality | Why it fits / hot path | MP role | Cost | Citation | Candidate task |
|---|---|---|---|---|---|---|
| C8 | **Flow divergence / time-to-contact**: least-squares divergence of the existing 80×60 SAD field → constant-divergence landing (replaces altitude-only descent in L6 LAND) and frontal TTC alarm in EXPLORE | Field already computed in `flow_task.cc`; divergence fit is a few hundred FLOPs per frame | `flow.divergence()` scalar; `explore`/`servo` land law | ~0 | Ho, de Croon et al., TRO 2018, arXiv:1609.06767; Scheper & de Croon, arXiv:1912.07735; Hazelaar et al., *Bioinspired adaptive visual servoing control for quadrotors*, Bioinspir. Biomim. 20(3) 036014, 2025, DOI 10.1088/1748-3190/adcdde; Paparazzi `optical_flow_landing` module | `OP-S10-W25-T4` (highest value/cost ratio) |
| C9 | **Alternating-frame stereo**: the two OV5640s are time-multiplexed (A.4); at 0.5 m/s a 33 ms offset moves the camera ~1.7 cm, comparable to any plausible baseline → naive stereo invalid in motion.  Feasible variants: (a) hover-only stereo (cf2 EKF |v| < 0.05 m/s), (b) motion-stereo using the EKF pose delta as the extrinsic — which is what the L5 inverse-depth lifter already does | Disparity on PXP-downscaled gray via `__USADA8` block match (reuse flow kernel) | `slam.update_stereo` already takes a baseline; needs HW baseline measurement first | ITCM reuse; SDRAM 2 gray slots | Yang et al., *Asynchronous Multi-View SLAM*, ICRA 2021, arXiv:2101.06562; Albl et al., arXiv:1704.06843; Seki & Okutomi CVPR 2007 | `OP-S10-W25-T5` (measure baseline + hover-stereo bench) — no MCU paper found: genuine gap |
| C10 | MCU VO/VIO (PicoVO on STM32F767 ~33 FPS QVGA; LEVIO ORB+BA on GAP9 20 FPS) — thesis reuses the cf2 EKF as ego-motion oracle; cite as "chosen not to" | — | — | — | Kühne et al., *LEVIO*, arXiv:2602.03294 (2026); arXiv:2509.10021 (downfacing VIO on GAP9); Kühne, Magno, Benini, arXiv:2305.13055 (OF on 8-core GAP8) | anchor only (§20.5) |
| C11 | **Monocular relative depth on TPU** (~310 K-param distilled net, 1.24 FPS on GAP8 (snippet) → fits the 8 MB TPU cache, likely > 20 FPS here) as pseudo-depth prior for L5 and obstacle cue | TPU invoke in `detection_task.cc` slot 1; PXP downscale | `pipeline.set_slot_for_cam`, scalar depth at bbox | 0.3 MB model; +1 invoke/frame on a transfer-bound USB link (W24-T3) | Zhang et al., *Channel-Aware Distillation Transformer for Depth Estimation on Nano Drones*, arXiv:2303.10386; FastDepth ICRA 2019 arXiv:1903.03273 | FW (indoor retrain needed) |

### C.4 Vision — semantics and learning on the Edge TPU (inference only — no training on TPU)

| # | Functionality | Why it fits / hot path | MP role | Cost | Citation | Candidate task |
|---|---|---|---|---|---|---|
| C12 | **Few-shot object re-identification by weight imprinting**: frozen MobileNet backbone on TPU → L2-normalised embedding → per-object prototype (vector average) → cosine k-NN; "is this the red box from cell H3-x?" for L2 UNKNOWN slots | Embedding = one TPU invoke; prototype math is µs in C (`sentai_objects.cc` gains a 128-512-float descriptor per slot) | `objects.imprint(id)`, `objects.reid()` returning (id, score) | SDRAM 64 slots × 512 B; model ~4 MB (shares cache with the detector — check the 8 MB ceiling) | Coral *Retrain a classification model on-device with weight imprinting* (ImprintingEngine docs); Gabdullin & Raskovalov, arXiv:2209.11024 (re-ID on Coral); arXiv:2607.18101 (frozen NPU backbone + on-device head, 2026); arXiv:2403.07851 (prototype FSCIL at 50 mW) | `OP-S10-W25-T6` |
| C13 | **Landing-zone segmentation** (floor / obstacle / marker / unknown) — EdgeTPU-DeepLab-slim 65.9 ms on a Dev Board; expect ~2× on our USB 2.0 link | TPU slot + PXP; C argmax over the mask to a "flat-floor fraction" scalar | `pipeline` slot, `explore` LAND gate | ~3 MB model; 100+ ms/frame | Coral semantic-segmentation model zoo; Kinahan & Smeaton arXiv:2111.14557; arXiv:2606.14767 (2026); NeuroSymLand arXiv:2510.22204 | FW |
| C14 | **Person following / pose** (PULP-Frontnet class CNN, 160×96, <87 mW on GAP8) — one more class in `objects` + existing PBVS in `servo` | TPU; nothing new in C | mission script | model only | Palossi et al., PULP-Frontnet, IEEE IoT J. 2022, arXiv:2103.10873; Crupi et al., arXiv:2402.13756 | FW / demo |
| C15 | **Lightweight VPR (DrosoNet-class)** as Track A' between the hand-crafted PHOG/GIST and the deferred DNN track | Random-projection + voting, KB-size, C in `.sdram_text` | `places.compute_droso()` | small | Arcanjo et al., *Patch-DrosoNet*, arXiv:2305.05256; RA-L 2024 arXiv:2312.12995; Ferrarini et al., embedded VPR benchmark arXiv:2109.11002 | FW (Dup: `sentai.places` descriptors) |

### C.5 System: recording, health, security, timing, cores

| # | Functionality | Why it fits / hot path | MP role | Cost | Citation | Candidate task |
|---|---|---|---|---|---|---|
| C16 | **Event-triggered recording with pre-trigger ring**: keep N gray/JPEG frames + IMU/mic scalars in a `.sdram_bss` ring; on `safety` abort / anomaly, flush to SD via `sentai.fr` | eDMA/FileX write path exists; ring in SDRAM | `fr.trigger()`, `fr.pretrigger_s(n)` | SDRAM ~2-6 MB | Heliyon 2024 UAV logger (PMC10881421); RADD arXiv:2505.01947 (2025); patent US11858652B2 — weak academic literature | `OP-S10-W25-T7` (Dup: extends `sentai.fr`) |
| C17 | **Companion-board self-health**: expose TPU junction temp (`GetTemperature`), M7 die temp (TMPSNS), USB/URB error counters, rail voltage (LPADC) in `sys.health()`; thermal throttling of invoke rate | all readers exist in `libs/` (B.1) | `sys.health()` dict | ~0 | no paper found on companion-board health; closest analogue Baumeister et al., *Tiny Drone, Tiny Resources: RTLola monitors on Crazyflie*, 2025; ADAM, SEAMS 2024 | `OP-S10-W25-T8` — small contribution claim |
| C18 | **Secure boot / attestation**: HAB-signed boot image (CAAM SHA/RSA at boot), CAAM-hashed `fr` log chain, PUF/OCOTP device key; firmware runs from RAM via `elf_loader` so encrypted XIP does not apply | boot-time only | `sys.attest()` returns hash | 0 runtime | NXP *Democratizing Security for Embedded Systems using i.MX RT* (RT1170 HAB/TRNG/AES); NXP community m-p/1433035; Privaros arXiv:2002.06512; Privadome arXiv:2205.04961; arXiv:2601.08229 (2026 sUAS security survey) | FW (after EXP-s255 fuses) |
| C19 | **Time base MCU↔cf2**: GPT1 µs frame tags exist; CRTP has no timestamp. Add a one-way offset/skew estimator between GPT1 and the cf2 tick; ENET 1588 is **not applicable** (no PHY, no Ethernet to the drone) | QTMR/GPT capture of VSYNC (B.1) | `crazy.clock_offset()` | ~0 | Tschopp et al., *VersaVIS*, Sensors 20(5):1439, 2020; Nissov, Khedekar, Alexis, arXiv:2507.05717 (2025); Liu et al., arXiv:2103.16045 | `OP-S10-W25-T9` |
| C20 | **M4 offload**: literature supports M4 only for I/O pipelining, not compute — matches W16 ablation (M7+SDRAM 2.4× faster than M4+OCRAM). Right M4 jobs: IMU FIFO drain + FFT (C5), mic frames (C1), SD recorder (C16) | RPMSG/MU present | — | M4 image in SDRAM | Lee et al., *Dual-Core-Based Microcontrollers Inference Design*, IEEE Access 12, 2024; arXiv:2605.09357 (split inference on RT1062 Teensys, 2026) | fold into OP-S10-W16 |
| C21 | **Edge TPU limits** corroborating W24-T3 (transfer-bound invoke, 8 MB parameter cache step) | — | — | — | Ni et al., DATE 2022 (PETET); van der Staay et al., *Stress-Testing USB Accelerators*, IEEE/ACM SEC 2024; arXiv:2107.12486 (USB 3 vs 2: up to 7×); Seshadri et al., AccML 2020 | cite in TPU chapter |

Not resolved by the search (no peer-reviewed source found): MCU time-multiplexed stereo (C9), pre-trigger logging on UAV MCUs (C16), companion-board thermal/USB health (C17), accelerometer FFT on a *companion* MCU (C5).  Re-verify before quoting numbers: Ghalamchi 2020 and CADiT 1.24 FPS (snippet only).

---

## D. Ranking — value for ObjectsPlan × feasibility on this board

Score = value (1-5, contribution to autonomous object-driven navigation + thesis narrative) × feasibility (1-5, existing primitives, ITCM/SDRAM headroom, HW risk).

| Rank | Candidate task | What | Value | Feas. | Score | Hot path (C) / MP | Depends on |
|---|---|---|---|---|---|---|---|
| 1 | `OP-S10-W25-T4` | `sentai.flow.divergence()` + constant-divergence LAND law (C8) | 5 | 5 | 25 | LSQ fit in `flow_task.cc` / `explore`+`servo` land state | nothing new |
| 2 | `OP-S10-W25-T6` | Weight-imprinting re-ID in `sentai.objects` (C12) | 5 | 4 | 20 | embedding invoke + prototype math in `sentai_objects.cc` / `objects.imprint/reid` | TPU cache budget vs detector |
| 3 | `OP-S10-W25-T8` | `sys.health()` gains TPU temp, die temp, URB counters, rail voltage + invoke throttling (C17) | 3 | 5 | 15 | readers already in `libs/tpu`, `libs/base` / dict | — |
| 4 | `OP-S10-W25-T3` | On-sensor free-fall + collision spike → `safety` (C6) | 3 | 5 | 15 | LIS2DUX12 engines, INT1 / `imu.freefall_start()` | — |
| 5 | `OP-S10-W25-T7` | Pre-trigger ring + event-triggered flush in `sentai.fr` (C16) | 3 | 4 | 12 | SDRAM ring + FileX drain / `fr.trigger()` | SD write rate (F17 open) |
| 6 | `OP-S10-W25-T2` | IMU FIFO @ 800 Hz + CMSIS rFFT vibration bands (C5) | 3 | 4 | 12 | I2C burst + `arm_rfft_fast_f32` / `imu.spectrum()` | board rigidly mounted on airframe |
| 7 | `OP-S10-W25-T1` | Mic spectrum + motor acoustic anomaly (C1) | 3 | 3 | 9 | rFFT + log-mel + FC-AE / `mic.spectrum/anomaly` | mount, ego-noise level |
| 8 | `OP-S10-W25-T9` | GPT1↔cf2 clock offset estimator; VSYNC capture on QTMR (C19) | 2 | 4 | 8 | QTMR capture ISR / `crazy.clock_offset()` | — |
| 9 | `OP-S10-W25-T5` | Hover-only stereo bench: measure physical baseline, disparity via `__USADA8` (C9) | 3 | 2 | 6 | reuse flow kernel / `slam.update_stereo` | baseline unknown; cameras alternate |
| 10 | `OP-S10-W25-T10` | OV5640 sensor-side ROI "digital PTZ" on the tracked bbox at constant output size (B.2) | 3 | 2 | 6 | I2C window regs (0x3800-0x380B, 0x5001) / `camera.roi()` | D-PHY re-init hazard — must keep HTS/VTS |

Hardware-side enablers that are not namespace features but unblock the above: FlexRAM 320/192 KB ITCM/DTCM re-split (B.1, candidate `OP-S10-W25-T11`, A/B under W15 rules); AWB lock for colour descriptors (B.2); T5838 AAD wake-on-sound + GPC idle (B.1/B.3, FW).  Deferred to FutureWork: KWS (C2), mono depth (C11), landing segmentation (C13), person following (C14), DrosoNet VPR (C15), HAB secure boot (C18), M4 compute offload (C20, close under W16).

## E. Things we already have and should not rebuild

- **Detection + tracking**: `sentai.pipeline` (TPU detector, NMS in C, SentAI-SORT) — no `sentai.detect`/`sentai.track` (`09_namespace_audit.md` §19.5 D).
- **Marker pose**: `sentai.markers` (ArUco + WhyCon + PnP + extrinsics + Kabsch via `sentai_svd3.cc`); `aruco`/`whycon` are aliases.
- **Ego-motion**: cf2 EKF over CRTP (`sentai.crazy.pose*`) is the L1 oracle; no on-board VO/VIO (C10 is an anchor, not a plan).
- **Optical flow**: `sentai.flow` (PXP + SIMD SAD + CMSIS phase correlation, body-frame transform, anchor mode) — divergence is an *addition*, not a rewrite.
- **World model**: `sentai.objects` + `sentai.object_lifter` (inverse depth) + `sentai.places` (H3, PHOG/GIST/HSV) + `sentai.slam` — motion-stereo (C9-b) is already what the lifter does.
- **Mission / action**: `sentai.explore` FSM + `sentai.servo` backends; abort logic lives only in `sentai.safety`.
- **Calibration**: `sentai.calib` (Kabsch, INI persistence, Kp autotune, orientation bring-up).
- **ML utilities**: `aifes`, `kmeans`, `pca`, `anomaly` (Mahalanobis + CUSUM), `dtw`, `hmm`, `rl` — C1/C5 scores should feed `sentai.anomaly`, not a new detector.
- **Recorder + logs**: `sentai.fr`, `rtos.dmesg`, `sys.health()` with STAT-LED codes — extend, do not fork.
- **TPU robustness**: URB slots/tokens, auto-recovery, CSR dump (OP-S10-W24) — only the temperature read is missing.
- **Dual-camera genlock**: MUX-flip phase alignment on GPT1 (EXP-s251/s254) — do not attempt FSIN wiring in firmware; it is a board-rev question.
