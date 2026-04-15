## Section Source Map

This appendix maps each chapter and implementation subchapter to source material that describes the concepts used in the manuscript. It is intended as a reusable citation-planning artifact for the final MDPI Drones submission.

### Abstract

- Embedded accelerator-backed inference on constrained hosts: [@Jouppi2017; @TensorFlowLite2024; @TFLM2024].
- Lightweight onboard UAV autonomy and fully onboard visual navigation: [@Palossi2019; @PX4Docs2024].
- Flight-control interoperability and drone messaging workflows: [@MAVLinkDocs2024; @CrazyflieDocs2024].
- Operational and regulatory framing for inhabited-space drone use: [@EASA2024].

### Introduction

- Tradeoffs between low-mass UAV autonomy, onboard perception, and compute budget: [@Palossi2019; @CrazyflieDocs2024].
- Accelerator-centric embedded AI deployment and host-side orchestration: [@Jouppi2017; @TensorFlowLite2024; @TFLM2024].
- PX4-oriented autopilot integration and interoperable telemetry/control links: [@PX4Docs2024; @MAVLinkDocs2024].
- Regulatory framing for low-altitude operations in populated areas: [@EASA2024].

### Related Work

#### Autonomous visual navigation for nano-UAVs

- Onboard autonomy under severe mass and power limits: [@Palossi2019; @CrazyflieDocs2024].

#### Embedded AI accelerators and systolic-array inference

- TPU-style accelerator design and its deployment implications: [@Jouppi2017; @TensorFlowLite2024; @TFLM2024].

#### YOLO-family object detectors and compact detection models

- Single-stage real-time detection and later YOLO refinements: [@Redmon2016; @Redmon2018; @Bochkovskiy2020].

#### MicroPython and high-level programmability on embedded systems

- Embedded scripting for rapid iteration over resource-constrained systems: [@MicroPythonDocs2024; @FreeRTOS2024].

#### Lightweight embedded learning, sequence modeling, and SLAM primitives

- Clustering and dimensionality-reduction methods used by `sentai.kmeans` and `sentai.pca`: [@Jain2010; @Jolliffe2016].
- Statistical monitoring and novelty-detection concepts used by `sentai.anomaly`: [@Chandola2009; @Pimentel2014].
- Sequence matching and probabilistic temporal decoding used by `sentai.dtw` and `sentai.hmm`: [@Salvador2007; @Bilmes1998].
- Reinforcement-learning primitives exposed by `sentai.rl`: [@Watkins1992; @Auer2002; @Russo2018; @Mnih2015].
- SLAM literature underlying `sentai.slam`: [@DurrantWhyte2006; @Cadena2016].

#### Vision pipelines for drone perception and viewpoint-aware sensing

- Camera interface and embedded image-path concepts: [@MIPI2024; @NXPIMXRT1170RM2024].
- Lightweight tracking-by-detection baselines and embedded-friendly association logic: [@Bewley2016; @Zhang2022].

#### Open datasets, synthetic data, and regulation-aware navigation

- Operational constraints that motivate regulation-aware perception: [@EASA2024].
- Embedded aerial autonomy context for onboard sensing and decision-making: [@Palossi2019].

### Implementation

- Co-design of MCU orchestration, accelerator inference, and developer-facing runtime layers: [@Jouppi2017; @FreeRTOS2024; @MicroPythonDocs2024; @TFLM2024].

### 1. System Architecture and Design Rationale

- Accelerator-plus-MCU architectures for edge inference: [@Jouppi2017; @TensorFlowLite2024; @TFLM2024].
- RTOS-backed embedded control and task partitioning: [@FreeRTOS2024].
- Scriptable embedded user environments for robotics experimentation: [@MicroPythonDocs2024].

### 2. RTOS Integration, Memory Layout, and Accelerator Feeding

- FreeRTOS task decomposition, queues, and synchronization primitives: [@FreeRTOS2024].
- Host-side tensor preparation and microcontroller-oriented inference runtime constraints: [@TFLM2024; @TensorFlowLite2024].
- RT1170 memory, DMA, cache, CSI, and PXP concepts: [@NXPIMXRT1170RM2024].

### 3. Dual-Camera Vision Pipeline and Perception Stack

- MIPI CSI-2 capture and embedded camera-interface fundamentals: [@MIPI2024; @NXPIMXRT1170RM2024].
- Detection and tracking baselines used as conceptual anchors: [@Redmon2016; @Redmon2018; @Bochkovskiy2020; @Bewley2016; @Zhang2022].
- Edge deployment path from captured image to quantized tensor: [@TensorFlowLite2024; @TFLM2024].

### 4. Flight Control, Telemetry, and External Integration

- Crazyflie as an indoor experimental UAV platform: [@CrazyflieDocs2024].
- PX4 autopilot concepts and vehicle-integration model: [@PX4Docs2024].
- MAVLink messaging and telemetry transport semantics: [@MAVLinkDocs2024].

### 5. Programming Model and User-Facing Runtime Surface

- Interactive embedded scripting and REPL-oriented workflows: [@MicroPythonDocs2024].
- RTOS observability and runtime diagnostics on embedded targets: [@FreeRTOS2024].
- Filesystem-backed deployment and data persistence on MCU devices: [@LittleFS2024].
- AIfES-backed on-device learning and compact embedded-ML building blocks: [@AIfES2024; @Jain2010; @Jolliffe2016; @Chandola2009; @Pimentel2014; @Salvador2007; @Bilmes1998; @Watkins1992; @Auer2002; @Russo2018; @Mnih2015; @DurrantWhyte2006; @Cadena2016].

### Conclusion

- Evidence base for compact onboard autonomy and integrated perception/control: [@Palossi2019; @Jouppi2017; @PX4Docs2024; @MAVLinkDocs2024].

### Future Work

- Portability across flight stacks and autopilot ecosystems: [@PX4Docs2024; @MAVLinkDocs2024; @CrazyflieDocs2024].
- Further optimization of dataflow and inference pipelines on MCU-class targets: [@TFLM2024; @TensorFlowLite2024; @NXPIMXRT1170RM2024].
- Regulation-aware autonomy and operational constraints in inhabited spaces: [@EASA2024].