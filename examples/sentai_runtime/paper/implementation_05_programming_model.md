## 5. Programming Model and User-Facing Runtime Surface

An implementation of this kind must be evaluated not only by its internal efficiency but also by how coherently it exposes its capabilities to the user. This is especially important on MCU-based platforms, where debugging is often more difficult than on Linux-class systems because the developer has less runtime introspection, fewer interactive tools, and a much narrower margin for trial-and-error when something goes wrong during boot or during a real-time task interaction. For this reason, SentAI does not expose a single debugging path, but a small set of complementary connection modes over USB that make the board significantly easier to inspect and operate in practice.

The first mode is an interactive MicroPython REPL available over the serial interface, which allows a researcher or operator to connect directly to the board, inspect the runtime state, execute commands live, load scripts, and request contextual help from within the system itself. This is a consequential design choice: the platform is not only flashed and executed as a fixed firmware image, but can also be interrogated and steered interactively during development and testing. In practice, the REPL becomes the primary entry point for experimentation, rapid diagnosis, and iterative mission design.

The second mode is USB networking over IP, through which the firmware exposes an onboard web server and file-browser interface. This gives the board a lightweight service surface that can be reached directly from the host, making it possible to inspect and transfer artifacts without depending exclusively on the serial console. The third mode is USB mass storage, in which the LittleFS user partition is exposed as a mounted drive so that images, scripts, and models can be uploaded or downloaded directly from the host machine. Taken together, these three USB-facing paths, namely serial REPL, IP connectivity with web access, and mounted LittleFS storage, form a practical answer to the usual debugging and deployment friction of MCU-based systems.

The built-in help interface available from the REPL makes this programming model discoverable without forcing the user to leave the target system. This matters because the runtime is relatively rich: it does not expose a single detector call or a single flight command, but a full namespace of platform primitives intended to support sensing, inference, telemetry, storage, and high-level control from one embedded environment. The same environment also supports script-driven startup through `/main.py`, which is automatically executed at boot before the interactive session begins. This gives the platform an important dual character: it can behave as an interactive research instrument during development, but it can also behave as a self-starting embedded application once the desired mission logic has been written.

This startup behavior is complemented by a boot logging mechanism implemented directly in the runtime. During initialization, console and driver output are captured into `/log/boot.log`, while the previous boot log is rotated to `/log/boot_old.log`. This is an important practical feature for embedded debugging, because many failures on an MCU platform occur before a user can attach interactively to the REPL. By preserving the initialization trace on LittleFS, the system allows the user to inspect the outcome of FreeRTOS bring-up, early peripheral initialization, and startup script execution even after the board has already moved past the failing point.

The `sentai` namespace organizes the platform into functional modules such as `io`, `rtos`, `tpu`, `fs`, `camera`, `imu`, `usb`, `link`, `crazy`, and `pipeline`, alongside several other utility interfaces. This structure is significant because it reveals the intended user abstraction: the operator is not expected to manipulate a collection of unrelated firmware endpoints, but to work with a coherent runtime in which device functions are grouped by task and by experimental purpose. The platform can query RTOS state, inspect heap usage, load models, capture or switch cameras, read inertial measurements, start a continuous detector, track objects, communicate over MAVLink, or command a drone, all through a single embedded language environment.

This helps explain why MicroPython is a central implementation choice rather than a peripheral convenience. The interpreter does not replace the lower-level real-time system; instead, it defines a stable experimentation layer above it. The help text demonstrates this duality clearly. On one hand, users access simple operations such as `sentai.camera.to_tensor()` or `sentai.tpu.invoke()`. On the other hand, the same interface reaches into advanced platform features such as tracking events, per-camera geometry, ground-plane projection, and MAVLink message exchange. The implementation thereby condenses a complex firmware system into a form that is operable from concise Python scripts.

Several modules are especially important for understanding the research value of the platform. The `rtos` module exposes runtime observability primitives such as task inspection, CPU statistics, and heap information, which are essential when the system is tuned for sustained real-time throughput. The `tpu` module provides the basic inference control plane: model loading, tensor access, invocation, quantization metadata, and detector-oriented post-processing. The `camera` module complements it by managing capture, JPEG export, transfer to the tensor path, runtime camera switching, and frame sequencing. The `imu` module supplies the inertial measurements needed by viewpoint-aware tracking and projection. The `fs` and `usb` modules support a practical workflow in which models, scripts, captured outputs, and debugging artifacts can be transferred to and from the device without rebuilding the firmware for every iteration.

The help surface also shows that these capabilities were designed to be exercised directly from the target rather than only through host-side tooling. Typical examples include immediate tensor feeding from the camera, live detector startup, autopilot connection, or direct Crazyflie flight commands, for example:

```python
sentai.camera.to_tensor()
sentai.tpu.invoke()
print(sentai.tpu.detect())
```

```python
sentai.link.init(57600)
sentai.pipeline.start(0.3, 0.45, 50, True)
while True:
	msg = sentai.link.receive()
	if msg:
		print(msg)
```

```python
sentai.crazy.init()
sentai.crazy.arm()
sentai.crazy.fly(0.3, 3000)
sentai.crazy.fly_stop()
```

At the control and autonomy layer, `link`, `crazy`, and `pipeline` form the most distinctive part of the namespace. The `link` module exposes MAVLink communication as a first-class runtime capability, allowing the board to exchange telemetry and structured messages with an autopilot or a ground-control system. The `crazy` module does the same for the indoor Crazyflie target, exposing high-level flight functions directly to Python. The `pipeline` module unifies the continuous detector, tracker, event stream, per-camera geometry, and pose-aware projection logic into a single interface. Together, these modules reveal the real ambition of the implementation: the embedded AI platform is not only an inference endpoint, but a scriptable autonomy substrate in which perception, communication, and high-level flight behavior can be composed interactively.

The `pipeline` module is especially illustrative. According to the documented runtime surface, it exposes continuous detection, statistics, tracker snapshots, event streams, configurable association parameters, pose setting, and per-camera geometry. This is not a trivial wrapper around a detector. It is the public face of the deeper implementation presented in the previous subsections: the parallel camera-to-TPU pipeline, the tracking-by-detection logic, and the geometry-aware projection model all become accessible through a minimal but expressive Python API. The same can be said for the `link` and `crazy` modules, which reveal that flight and telemetry functions are not separate applications but first-class capabilities of the runtime itself.

From a research-methodology perspective, this matters because it changes the pace and style of experimentation. A researcher can move from firmware bring-up to mission logic, data capture, tracking inspection, and communication testing without leaving the same runtime environment. This reduces iteration cost and makes the platform more suitable for exploratory autonomy research, where control policies, perception thresholds, and communication strategies change frequently.

The built-in help surface also provides indirect evidence of implementation maturity. A platform that documents precise operational behaviors, module boundaries, argument conventions, and example workflows has generally progressed beyond proof-of-concept code. In SentAI, the fact that these capabilities can be explored directly from the serial REPL supports the claim that the implementation has been organized as a reusable experimental environment rather than as a collection of hidden firmware entry points.

The programming model therefore completes the overall argument of the chapter. The EdgeTPU provides the inference power, the MCU and RTOS provide the orchestration, the cameras and IMU provide the sensing context, and MicroPython makes the whole system operable as a research platform. The implementation succeeds not only when it runs efficiently, but when it turns that efficiency into a usable and extensible interface for autonomous drone experimentation.