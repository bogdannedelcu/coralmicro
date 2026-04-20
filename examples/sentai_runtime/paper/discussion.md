## Discussion

*The per-result discussion is in §8 of [evaluation.md](evaluation.md);
threats to the interpretation are catalogued in
[threats_to_validity.md](threats_to_validity.md); the wider systems-
and-platform framing is below.*

The preceding implementation results indicate that the main contribution of the platform lies in system integration rather than in any single algorithmic element. Its value depends on the extent to which sensing, inference, memory placement, scheduling, and communication can operate together under strict resource constraints. For this reason, a meaningful evaluation should consider not only model accuracy, but also end-to-end latency, dropped-frame behavior, camera-switch overhead, telemetry cadence, and the stability of obstacle outputs delivered to higher-level control components — the same metrics that drive the RQ1–RQ5 decomposition in [evaluation.md](evaluation.md).

A **second-order observation** that emerges from stacking the
measurements taken together: each optimisation described in the
implementation chapters removed one bottleneck and exposed the
next.  The CPU memcpy ([memcpy.md](memcpy.md)) was invisible as long
as the camera-drain path was the dominant cost; once that was fixed
elsewhere in the codebase the memcpy rose to 32 % of the InferTask
critical path.  The parallel-pipeline ceiling at 15 FPS
([evaluation.md](evaluation.md) Table 1) was invisible as long as
InferTask and PrepTask were roughly balanced; once the memcpy was
sped up, PrepTask became the unambiguous bottleneck — and only then
was the system camera-rate-limited in a way that made the
alternating-switch study well-defined.  The 83 ms per-switch tax
([evaluation.md](evaluation.md) Table 3) is the next layer: now that
the fixed-camera throughput is at the sensor ceiling, alternation is
where the remaining engineering budget matters.  This layering is
not an artefact of presentation order; it is a property of the
platform.  A reader adapting these techniques to a different board
should expect to re-discover whichever layer is the bottleneck on
that hardware first, and to have a different "next optimisation" as a
result.

A **third observation** concerns the role of the measurement
framework itself.  The same `diag` sessions, the same warm-up
protocol, the same per-iteration CSV format captured both the first
6 FPS E10/E11 memory-and-CPU probes (Appendix A of
[experiments/README.md](../experiments/README.md)) and the final 16
FPS head-to-tail benchmark.  No methodology change was needed
between those two endpoints despite nearly an order of magnitude of
performance improvement and the addition of four new experiment
classes (E15–E18).  This suggests that the self-contained-session
discipline described in [experiments/methodology.md](../experiments/methodology.md)
is a stable scaffolding for measurement-driven embedded-systems work
— a claim we make explicitly here because the alternative (bespoke
measurement harnesses per optimisation) is the default in the
embedded literature we surveyed
([related_embedded_inference.md](related_embedded_inference.md)) and
a barrier to cross-study comparability.

The platform also exposes a useful design space for mixed inference and lightweight adaptation. EdgeTPU-backed models, CPU-side TensorFlow Lite Micro execution, and the embedded-ML modules available through MicroPython make it possible to compare alternative placements of perception and decision components within the same runtime. This is relevant for future work on few-shot adaptation, anomaly-triggered reconfiguration, temporal modeling, and compact policy learning, where the main question is not raw model scale but how such mechanisms behave under embedded timing and memory constraints.

An additional direction concerns the use of the REPL and runtime namespaces as a bounded tool interface for an external large language model. In such a configuration, the language model would not consume raw sensor streams or issue low-level control commands. Instead, it would operate over discrete or aggregated signals produced by the device, such as detector summaries, track events, anomaly flags, obstacle bins, or mission-state variables, and would use the REPL to generate scripts or call higher-level functions. This arrangement is attractive because it preserves local execution of sensing and safety-critical logic while allowing offboard reasoning over compact symbolic state.

At the same time, the limitations of the platform remain explicit. MCU-class resources impose strong constraints on model size, buffering, concurrency, and recoverability. Any future extension involving online learning or LLM-mediated tool use would therefore need clear restrictions on admissible commands, local validation of generated actions, and strict separation between high-level reasoning and real-time control. In this sense, the platform is best viewed as a controlled environment for embedded autonomy experiments rather than as a replacement for larger companion-computer architectures.