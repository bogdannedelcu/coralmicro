# AirREPL — Paper Plan

**Status**: parked artifact, post-thesis target (see `FutureWork.md`
FW17). This file is the **living paper plan**, updated as work
accumulates. Companion to thesis, not part of thesis main body.

**Last updated**: 2026-05-15

---

## 1. One-liner

> **AirREPL: A Turing-complete embedded Python REPL on Cortex-M7,
> addressable over a 30 B/packet radio link, designed for LLM-agent
> consumption.**

A wireless sensor MCU exposes its full MicroPython REPL over the
Crazyflie CRTP radio. An LLM agent issues short `$exec` expressions
in flight and reads back compact, structured replies. The agent can
re-task the sensor, swap DNN models, query semantic state, and close
mission-level control loops — without re-flashing firmware, without
adding a Linux SBC, and within the energy budget of a solar-powered
node.

## 2. Why this is publishable (gap claim)

Verified May 2026 lit review (saved as `[[airrepl-paper-lit-review]]`
memory; full citation list in `FutureWork.md` FW17):

- **Drone × LLM** literature treats the drone as a dumb endpoint
  for fixed-vocabulary commands (MAVLink, MCP-schema, MiniSpec
  DSL). Cloud LLM → ground-station → radio → drone.
- **Code-as-Policies / PromptCraft** generate Python for robot
  control, but execute on ROS workstation / PC, not on MCU.
- **LLM for embedded** is *build-time codegen* (firmware
  synthesis), not *runtime* command channel.
- **Solar + LoRa + edge-AI sentinels** exist commercially, but use
  Pi-class SBCs with full Ollama / fixed inference pipelines.
- **MCP** is currently schema-bound; AirREPL is Turing-complete
  Python.

**The missing artifact**: live, in-flight, Turing-complete
reprogramming of an MCU runtime over a sub-100-byte radio link,
designed end-to-end for LLM-agent consumption.

## 3. Contribution (claim, 5 bullets — these go into the abstract)

C1. **Turing-complete reprogramming over a constrained radio**.
    Show that ~30 B/packet CRTP is enough to express LLM-driven
    re-tasking, despite earlier MAVLink/MCP designs assuming
    fixed-vocabulary commands.

C2. **REPL semantics designed for LLM consumption** (not human):
    short `repr` round-trips, alias trick for telemetry, ~6 Hz
    polling ceiling, structured exception channel, idempotent
    fragments.

C3. **Pure-MCU target**. No Linux SBC, no Ollama on device. Runs
    on a Cortex-M7 (NXP RT1176) with FreeRTOS. Power budget
    compatible with a small solar panel + 18650 cell.

C4. **Anti-brick supervised runtime**. The LLM agent cannot wedge
    the MCU with bad Python. Watchdog, FreeRTOS task supervision,
    structured error registry, hardware-recoverable boot stages.

C5. **Privacy-by-architecture**. Bandwidth constraint forces
    semantic-only telemetry. Raw frames cannot leave the node by
    construction. Compliance posture (EU AI Act, GDPR data
    minimization) follows from the design, not from a policy bolt-on.

## 4. What we already have

### 4.1 Hardware platform
- Coral Dev Board Micro (NXP i.MX RT1176 Cortex-M7 @ 800 MHz, dual
  OV5640 cameras, EdgeTPU)
- Crazyflie 2.1 quadcopter (Cortex-M4 STM32F405) with Crazyradio
  2.4 GHz dongle
- Custom power + UART wiring between Coral and Crazyflie
- All proven on bench + in flight

### 4.2 Firmware artifact (SentAI)
- `examples/sentai_runtime/` — full MCU firmware in C/C++
- MicroPython embedded port with `sentai.*` bindings (camera, TPU,
  flow, places, objects, servo, object_lifter, crazy radio bridge,
  filesystem)
- USB CDC-ACM REPL (development) **and** Crazyflie CRTP radio
  REPL (deployment)
- Anti-brick + watchdog + FxUser persistent FS + structured error
  codes — all production-grade
- See `examples/sentai_runtime/agent/agent.md` for canonical
  architecture description

### 4.3 Radio REPL substrate
- Crazy bridge auto-inits in firmware before `/main.py`
- CRTP transport at ~6 Hz polling ceiling for ~150 B replies
- Inline `$exec` for short Python expressions
- Alias-variable trick: pre-define short-name variables on the
  drone, then poll them by single-letter alias (saves bytes)
- See `feedback_radio_no_file_transfer.md` (memory) for the
  exact discipline that defines AirREPL constraints

### 4.4 Proof-of-concept missions (s091–s132)
A sequence of ~30 experiments under
`examples/sentai_runtime/experiments/sNNN_*/` already demonstrates
the LLM-agent loop pattern, where the **host Python script acts as
a thin LLM-agent surrogate** driving the drone over the REPL:

- s091 — ArUco hover with cf2 (4.8–9.7 cm drift, 100% 4-marker)
- s107 — PX4 + flow architecture validated
- s128 — closed-loop EKF 4-marker tour
- s129 — IBVS centering via PnP-tvec → body delta
- s130 — image-only world-model nav (4 markers, pos err 1.2 cm
  median, **the EKF is never read by the control loop** — pure
  semantic-telemetry-driven autonomy)
- s131 — Civera inverse-depth EKF math validation
- s132 — **L5 lifter end-to-end Gazebo integration**: cf2
  takeoff + lateral pass + `init_from_bbox` + `update_bbox` all
  driven via REPL `$exec`. First-try PASS, err_xy=1.7 cm,
  σ_ρ ratio=0.014.

s132 specifically is the cleanest "this is the AirREPL pattern" demo
already in the repo: a host Python orchestrator (which a Claude /
GPT-4 agent could trivially replace) issues structured `$exec`
calls, parses compact replies, decides next action.

### 4.5 ReplDriver pattern (the ABI candidate)
Pattern emerged in `mission_l41.py` and refined through s132:
- `exec_repr(expr) -> str` — eval Python expr on drone, return
  `repr()` of result
- `exec_int(expr) -> int` — eval and cast to int
- `exec_value(expr)` — eval and parse the repr back into Python
  object (round-trip via `ast.literal_eval`)
- Structured exceptions: parse errors vs runtime errors vs
  link timeouts vs MCU watchdog reset

This is the "interface" candidate that should be **formalized in
the paper** as the AirREPL protocol.

### 4.6 Existing related infra
- `sentai.sim.journal_*` — structured append-mode log for
  post-mortem (paper-friendly: every experiment is replayable
  from journal)
- FlowBaseline regression gate (`s127`) — every commit benchmarks
  optical flow drift, so claims about non-regression are empirical

## 5. Experiments to run for publication

This is the centerpiece. Each experiment below is **necessary** for
some part of the claim, with explicit setup, metrics, pass criteria,
and the contribution bullet it supports.

Numbering convention: **E1..E10**. Treat them as `sNNN_*/` style
experiments — each one gets its own folder under
`examples/sentai_runtime/experiments/`, README + script + captured
data, runnable end-to-end.

**Status legend**: `REQUIRED` (must have for publication), `STRONG`
(expansion scope, makes paper much stronger), `OPTIONAL` (nice to
have, can be deferred).

---

### E1 — Protocol latency + throughput characterization [REQUIRED]

**Proves**: C1 (Turing-complete reprogramming over constrained radio
is feasible at usable speed).

**Setup**:
- Drone on bench, motors disarmed, Crazyradio on host
- Host script issues 1000 `$exec` calls of varying payload size
  (10 B, 30 B, 60 B, 90 B, 120 B Python expressions)
- 4 alias-poll modes: cold-each-call, warm-cached, single-alias-loop,
  multi-alias-batched

**Metrics**:
- Round-trip latency distribution: p50, p95, p99, max
- Sustained throughput at saturation (cmd/s)
- MTU efficiency: useful Python bytes / total CRTP bytes
- Packet retransmit rate

**Pass criteria**:
- p50 latency < 200 ms for 30 B `$exec`
- p99 latency < 500 ms
- Sustained > 5 cmd/s for short polls (alias trick)
- MTU efficiency > 50% at 30 B payload

**Folder**: `experiments/p001_airrepl_latency/`

**Effort**: ~2 days (script + data + plots).

---

### E2 — RSSI / range sensitivity [REQUIRED]

**Proves**: AirREPL works at sensor-network deployment distances,
not just bench (defends C3 deployability).

**Setup**:
- 5 distances: 1 m, 5 m, 15 m, 30 m, 50 m line-of-sight outdoor
  (also indoor with walls: through 1, 2, 3 drywall partitions)
- 100 `$exec` calls at each distance
- Measure RSSI / link quality reported by cflib

**Metrics**:
- Success rate vs distance + obstacle count
- Latency increase vs RSSI
- Retransmit count vs RSSI

**Pass criteria**:
- > 95% success at 30 m LOS
- > 90% success at 3-wall indoor
- p50 latency degrades gracefully (no cliff)

**Folder**: `experiments/p002_airrepl_rssi/`

**Effort**: ~2 days (need outdoor location + walking around).

---

### E3 — LLM agent task suite [REQUIRED — the cornerstone]

**Proves**: C2 (REPL semantics actually work for LLM-agent
consumption, not just for humans).

**Setup**: Define 10 tasks of increasing complexity. Each LLM agent
receives the AirREPL ABI documentation + task description, must
solve without human intervention.

Task list:
1. T1 — Confirm drone alive: read battery + version
2. T2 — Read camera state and report resolution + fps
3. T3 — Arm + hover 1 m + land (basic motion)
4. T4 — Find ArUco marker id=0, report pixel coords
5. T5 — Fly to ArUco marker id=0, hover above it 5 s, land
6. T6 — Swap DNN model: load model X, run inference, report result
7. T7 — 4-marker tour: visit id 0,1,2,3 in order
8. T8 — Lifter init+update: do s132-style 3D landmark estimation
9. T9 — Recovery: after forced packet loss, resume mission
10. T10 — Composed mission: scan, find target by class, approach,
    report semantic obs

**Agents to test**:
- Claude Opus 4.7 (anchor)
- Claude Sonnet 4.6
- GPT-4.x
- GPT-3.5 / 4o-mini (cheaper tier)
- Llama 3.1 70B (open-weight)
- (optional) Gemini 2.x

**Metrics per (task × agent)**:
- Success: yes / no / partial
- # AirREPL commands issued
- Prompt + response tokens consumed
- Wall-clock time to completion
- # recovery attempts after errors
- Cost in USD (where applicable)

**Pass criteria**:
- Claude Opus achieves > 80% pass rate across T1–T10
- At least one open-weight model achieves > 50% on T1–T5
- Token efficiency (avg < 5k tokens per task)

**Folder**: `experiments/p003_llm_agent_suite/` with sub-folders per
agent + task.

**Effort**: ~7 days (this is the biggest experiment; needs careful
prompt engineering + a lot of LLM API calls).

**Risk**: stochastic LLM outputs. Mitigation: 3 runs per task per
agent, report mean + variance. Fix model versions.

---

### E4 — Anti-brick fault injection [REQUIRED]

**Proves**: C4 (LLM cannot wedge MCU with bad Python).

**Setup**: 50 deliberately-malformed `$exec` inputs:
- 10 syntax errors (`for x in:`, `1/`, unbalanced parens)
- 10 NameError / TypeError (`undefined_var`, `1 + "string"`)
- 5 infinite loops (`while True: pass`)
- 5 memory blowups (`x = [0] * 10**9`)
- 5 deep recursion (`def f(): f(); f()`)
- 5 hardware-poke attempts (raw memory writes if exposed)
- 10 deliberately Crashy-Python (segfault patterns, double-free
  via ctypes if available)

For each, issue via AirREPL, observe MCU behavior.

**Metrics**:
- # MCU still alive after (link responsive within 5 s)
- # returned structured error trace (vs link silence)
- # required reboot to recover
- # required physical button (= **BRICK**, must be 0)

**Pass criteria**:
- 100% MCU still alive after any input
- 0 bricks
- Structured error trace in > 80% of cases (others may be link-
  timeout, but MCU itself responsive on next poll)

**Folder**: `experiments/p004_airrepl_antibrick/`

**Effort**: ~2 days.

---

### E5 — Energy per command + solar budget [REQUIRED for C3]

**Proves**: C3 (pure-MCU runtime is power-budget compatible with
solar deployment).

**Setup**:
- INA219 or similar power meter inline with Coral Dev Board Micro
  USB power
- 4 scenarios:
  1. Idle baseline (no $exec, no camera, no TPU)
  2. Idle + camera active (continuous OV5640)
  3. Idle + TPU active (continuous inference at 30 fps)
  4. $exec stream at 3 Hz (alias polling pattern)
- 60 s each, sampled at 1 kHz

**Metrics**:
- Mean wattage per scenario
- Peak wattage during $exec
- Joules per $exec (delta from idle baseline × duration)
- Total daily energy consumption at projected duty cycle

**Pass criteria**:
- Idle < 200 mW
- $exec adds < 50 mJ per command
- Projected 24 h energy < 4 Wh at 10% duty cycle (well within
  5 W solar panel winter budget)

**Folder**: `experiments/p005_airrepl_energy/`

**Effort**: ~2 days (need INA219 + soldering or breakout board).

---

### E6 — Comparative table: same task, three interfaces [REQUIRED]

**Proves**: AirREPL is materially better than MAVLink + MCP for the
**class of tasks we care about** (sensor re-tasking, semantic
queries, ad-hoc operations).

**Setup**: 3 canonical tasks implemented end-to-end via each
interface.

Tasks:
- TC1: Fly drone to ArUco marker id=0 and hover
- TC2: Swap onboard DNN model (load new TFLite file, init, run)
- TC3: Query semantic observation: "what objects do you see right
  now, with positions"

Implementations:
- **MAVLink baseline**: PX4 + MAVSDK or DroneKit, full command chain
- **MCP baseline**: write minimal MCP server in front of MAVLink,
  expose tools, run from same LLM
- **AirREPL**: same LLM, our Crazyflie radio + sentai.* bindings

**Metrics per (task × interface)**:
- Total bytes on the wire (command + response)
- End-to-end latency
- Agent-side code complexity (LOC of agent's mental "tool
  vocabulary")
- LLM tokens consumed to drive the task
- Adaptability: how many bytes to add a *new* task variant
  (e.g., "fly to marker 2 instead of 0", "switch to different
  DNN")

**Pass criteria** (none binary, qualitative + tabular):
- AirREPL has fewer bytes / lower latency / less complexity on at
  least 4 out of 9 (task × metric) cells
- TC2 specifically demonstrates AirREPL flexibility that MAVLink
  cannot match (model swap is not in MAVLink vocabulary)
- TC3 demonstrates ad-hoc query that MCP needs custom schema for
  but AirREPL handles via Python expression

**Folder**: `experiments/p006_airrepl_vs_mavlink_vs_mcp/`

**Effort**: ~5 days (implementing MAVLink + MCP baselines is the
expensive part).

**Risk**: care not to strawman MAVLink / MCP. Implement them
honestly. Be willing to admit AirREPL loses on tasks like
"hover precisely at GPS waypoint" where MAVLink wins.

---

### E7 — Composed mission stress test (s132 scaled up) [REQUIRED]

**Proves**: AirREPL composes — the s132 pattern (init + iterative
update via REPL) extends to realistic mission complexity.

**Setup**: 4-marker indoor tour with mid-mission DNN swap:
- Takeoff
- Visit marker 0, lift, record
- Switch DNN model to a different detection target
- Find new target type, approach, report obs
- Return to marker 0
- Land
- All via AirREPL driver (no hand-coded waypoints; LLM agent
  must compose from primitives)

**Metrics**:
- Mission completion rate over 20 runs
- Total mission time
- # AirREPL commands per mission
- Drift error (lifted positions vs ground truth)
- # recovery attempts

**Pass criteria**:
- > 80% completion rate
- Drift error < 30 cm
- < 50 commands per mission

**Folder**: `experiments/p007_airrepl_composed_mission/`

**Effort**: ~4 days.

---

### E8 — Privacy property audit [REQUIRED for C5]

**Proves**: C5 (raw frames never leave node).

**Setup**:
- Static analysis: grep all `sentai.*` MicroPython bindings for
  any path that returns raw camera buffer, raw audio buffer, raw
  IMU stream
- Dynamic analysis: run AirREPL session, packet-capture all CRTP
  traffic, verify no payload contains image-like or audio-like
  byte sequences (entropy + structure heuristics)
- Formal statement: enumerate all data egress paths

**Metrics**:
- # raw-buffer-egress paths found (target: 0 or documented
  exceptions only)
- Max bytes/s leaving the node in 5-minute session

**Pass criteria**:
- 0 raw-frame egress paths in default `sentai.*` API
- (If any exist) clearly labeled and gated behind explicit opt-in
- All egress < bandwidth physically forcing semantic-only

**Folder**: `experiments/p008_airrepl_privacy_audit/`

**Effort**: ~2 days.

---

### E9 — 24h solar deployment validation [STRONG, optional]

**Proves**: C3 fully (claim becomes "we did this in the field, not
just calculated it").

**Setup**:
- Coral Dev Board Micro + Crazyradio dongle + Crazyflie hanging
  in a static pose
- 5 W solar panel + 18650 Li-ion + simple charging board
- Outdoor for 24 h
- LLM agent issues a battery + obs query every 10 min via
  AirREPL; periodically issues a "scan and report objects" task

**Metrics**:
- Uptime
- Energy balance (panel input vs MCU output)
- Command success rate over 24 h
- Battery state-of-charge over 24 h
- Failure modes encountered

**Pass criteria**:
- > 95% uptime
- Net positive energy balance over 24 h sun cycle
- > 90% command success rate

**Folder**: `experiments/p009_airrepl_solar_24h/`

**Effort**: ~5 days (mostly waiting + post-processing).

**Why STRONG not REQUIRED**: claim C3 can be defended analytically
via E5 (energy/cmd × duty cycle vs panel budget). E9 is the
empirical version — much more credible but not strictly necessary
for the gap claim.

---

### E10 — Reproducibility bundle [REQUIRED]

**Proves**: reviewers can replicate.

**Setup**:
- Single bring-up script `./reproduce.sh` that:
  - Clones the repo (specific tag)
  - Builds firmware via Docker if needed
  - Flashes board
  - Starts cflib host
  - Runs a 3-command demo
  - Compares output to golden expected
- Hardware BOM with part numbers, costs, suppliers
- Pre-recorded LLM transcripts for E3 tasks (so reviewers without
  API keys can verify)
- Docker image for host side

**Metrics**:
- Time from fresh clone to first successful $exec on virgin Linux
  laptop
- # external dependencies (Linux packages, Python wheels)

**Pass criteria**:
- < 30 min cold-start on standard Linux laptop
- Reproducible across 3 reviewers' machines

**Folder**: `experiments/p010_airrepl_reproducibility/`

**Effort**: ~3 days.

---

## 5.bis Experiment dependency graph + suggested order

```
E1 (latency)  ─┐
E2 (RSSI)     ─┼─→ E6 (comparative table) ──┐
E5 (energy)   ─┘                              │
                                              ├─→ paper
E3 (LLM agent) ─→ E7 (composed mission) ──────┤
                                              │
E4 (anti-brick) ──────────────────────────────┤
E8 (privacy audit) ───────────────────────────┤
E10 (reproducibility) ────────────────────────┘
            E9 (solar 24h, optional) [parallel to all]
```

**Suggested order**:
1. **Phase A (foundation, ~7 days)**: E1, E4, E5 — bench-only,
   no LLM agent dependence, validates the substrate
2. **Phase B (the cornerstone, ~10 days)**: E3 — LLM agent task
   suite. The big one. Drives the paper's central claim.
3. **Phase C (deploy + compare, ~10 days)**: E2, E6, E7
4. **Phase D (closeout, ~5 days)**: E8, E10
5. **Phase E (optional, ~5 days)**: E9 (solar deploy)

**Required path total**: ~32 person-days experiments + ~10 days
writing = **~42 days post-thesis**.

**With E9**: ~47 days.

This is realistic for a 2-3 month post-defense window before the
next plausible workshop CFP.

## 5.ter What we do NOT need to run

To avoid scope creep, explicitly out-of-scope for this paper:
- Cross-MCU portability (RT1176 only)
- Multi-tenant scheduler for $exec calls
- MCP-bridge layer over AirREPL (interesting, but separate paper)
- Real-time closed-loop control via AirREPL (not its purpose;
  inner loops stay compiled C++)
- Mesh routing across multiple AirREPL nodes (separate paper)
- Live OTA firmware update via AirREPL (engineering work, not
  publication-novel)

These all go into "future work" section of the paper itself.

## 6. Paper structure (target ~10 pages workshop, expandable to
~14 pages short conference paper)

### §1 Introduction (1.5 p)
- Hook: edge sensors + LLM agent paradigm
- Problem: existing patterns assume cloud LLM + dumb endpoint
- Contribution: AirREPL inverts (LLM remote, runtime on MCU)
- Roadmap

### §2 Related Work (1.5 p)
- 2.1 LLM × drone command (MAVLink, MCP, TypeFly)
- 2.2 LLM code-gen for robotics (Code-as-Policies, PromptCraft)
- 2.3 LLM for embedded firmware (build-time, distinct from us)
- 2.4 Edge-AI sentinel sensors (solar, LoRa — currently
  non-LLM-controllable)
- 2.5 Positioning: clear table in §2.5 showing gap

### §3 AirREPL Design (2 p)
- 3.1 System model: actors (LLM agent / radio link / MCU runtime)
- 3.2 Protocol primitives: `$exec`, alias, exception, polling
- 3.3 Constraints: MTU, polling rate, MCU resources, power
- 3.4 ABI: ReplDriver (exec_repr / exec_int / exec_value)
- 3.5 Anti-brick architecture

### §4 Implementation (1 p)
- SentAI firmware overview (defer to thesis for depth)
- MicroPython embed port + sentai bindings
- Crazyflie CRTP transport layer
- ReplDriver host library

### §5 Evaluation (3 p)
- 5.1 Latency, throughput, RSSI sensitivity
- 5.2 LLM agent task suite results
- 5.3 Anti-brick fault injection
- 5.4 Energy per command + solar budget
- 5.5 Comparative table vs MAVLink, MCP

### §6 Discussion (1 p)
- Limitations (6 Hz ceiling, Python-only, single-tenant currently)
- Privacy-by-design implications
- When NOT to use AirREPL (high-bandwidth telemetry, real-time
  closed-loop control)
- Future: MCP bridge layer, multi-tenant scheduler, mesh routing

### §6.bis AirREPL vs MCP — positioning (CRITICAL for review)

This will be the #1 reviewer question. Address head-on, treat as
complementary not competitive.

**Stance**: AirREPL is the **substrate** (low-level Turing-complete
runtime), MCP is one possible **interface layer** above it.

**Pros MCP+Drone (don't strawman)**:
- Standard cross-vendor (Claude, GPT, Cursor, Llama clients all
  support MCP)
- Schema-bound type safety
- Auditability via central server log
- Discoverability (LLM enumerates tools on connect)
- Governance / rate-limiting at server layer
- Better compliance posture for enterprise

**Cons MCP+Drone**:
- Fixed vocabulary at design time — new op = server redeploy
- Stateless by convention; awkward to preserve interpreter state
- JSON-RPC is too fat for sub-100-byte radio MTU
- Requires intermediary server (LLM → server → drone)
- Can't introspect runtime state not anticipated ex-ante

**Pros AirREPL**:
- Turing-complete vocabulary (any Python expression)
- Stateful interpreter (aliases + variables persist)
- Compact wire format suited to constrained radio
- No middleware server — direct LLM ↔ radio ↔ MCU
- Live mid-mission re-tasking + introspection
- Architecturally privacy-by-construction (bandwidth forces
  semantic-only telemetry)

**Cons AirREPL**:
- Not yet a standard (custom integration per LLM)
- Python is dynamic — type safety only through anti-brick
- Auditability requires custom logging
- LLM must know Python (most do, but not formal contract)

**Where each wins**:
- MCP for enterprise drone with abundant bandwidth + strict
  compliance + well-defined operational vocabulary
- AirREPL for solar-constrained sensor MCU, ad-hoc / exploratory
  missions, low-MTU radio, runtime adaptability

**Long-term ideal: MCP bridge layer over AirREPL**:
```
LLM → MCP server (host-side, audit + schema)
            ↓
        AirREPL substrate (radio + MCU)
```
Analogous to: SSH vs REST API (general shell vs curated endpoints
— both valid, both deployed in real orgs).

This framing turns the "why not MCP?" question from a weakness
into a future-work strength: AirREPL is the **lower layer** on
which MCP can be built for sensor MCUs that MCP-over-MAVLink
can't reach (radio-constrained, MCU-only, server-less).

### §7 Conclusion (0.5 p)

### References (1-1.5 p)

## 7. Target venues + timeline

| Venue | Type | Deadline window | Notes |
|---|---|---|---|
| **arxiv** | Preprint | Any time | Submit first, no gatekeeping |
| ICRA Workshops (LLM+Robotics flavor) | Workshop | Annual, ~Jan deadline | Has been a staple since 2023 |
| HotMobile | Short paper | ~Sep/Oct | Strong embedded systems venue |
| SenSys | Short / poster | ~April | Sensor systems angle |
| NeurIPS Agent Benchmarks Workshop | Workshop | ~Aug/Sep | LLM agent angle |
| MLSys workshops | Workshop | varies | Systems-for-ML angle |

**Recommended order**:
1. arxiv preprint immediately on completion (citable artifact)
2. ICRA workshop submission (next available)
3. Iterate to full conference (HotMobile / SenSys) based on
   reviewer feedback

**Earliest realistic submission**: Q2 2027, assuming defense
finishes Q4 2026 and we then do the ~30 days of paper-prep.

## 8. Reproducibility plan (what reviewers can replicate)

- Hardware bill of materials (~$500: Coral $80 + Crazyflie 2.1
  $250 + Crazyradio $30 + cables + battery + optional solar)
- Single-command firmware build (`bash build.sh`)
- Single-command flash (`python3 scripts/flashtool.py -e
  sentai_runtime`)
- Single-command demo (`./reproduce.sh` does the rest)
- Pre-recorded LLM transcripts for 10-task suite (so reviewer
  doesn't need API keys to verify claim C2)
- Public repo with stable tag

## 9. Open questions / risks

1. **Is "Python REPL is the right ABI" defensible?** Reviewers
   may push back: why not just MCP-over-radio with a small fixed
   schema? Answer must be empirical (show tasks AirREPL can do
   that MCP can't, e.g., introspection, ad-hoc data shape change,
   chained operations in one packet).

2. **6 Hz polling is harsh for control loops.** Counter: AirREPL
   is for **mission-level tasking**, not inner-loop control.
   Inner loops stay on-MCU compiled C++ (the SentAI principle).

3. **Single-platform claim (Crazyflie radio).** Should we add a
   second transport (MAVLink radio, LoRa) to broaden the claim?
   This adds engineering effort but strengthens "transport-agnostic"
   framing.

4. **LLM evaluation reproducibility.** LLM outputs are stochastic.
   How do we make the eval reproducible? Pin model versions, fixed
   seeds where possible, multiple runs per task.

5. **Comparison fairness.** MAVLink + MCP can do some of these
   tasks too if you write the right server. Be careful not to
   strawman. Comparison should be "complexity to express new
   task" rather than "can this be done at all".

## 10. Notes from this conversation (May 2026)

- Operator's broader project framing (`SentAI` = Sentinel + AI):
  wireless-only sensor for distributed AI eyes/ears. Drone =
  stress-test of the platform, not the product. AirREPL is the
  **mechanism** for "AI commands edge sensors over radio".
- Power story is genuine: 200 mW continuous on a small solar panel
  in temperate climate is achievable for static deployment;
  AirREPL fits this envelope.
- Privacy story is genuine: bandwidth forces semantic-only
  telemetry. This is a real architectural property, not marketing.
- The agent paradigm in 2026 (Anthropic MCP, function-calling,
  tool-use) is the **timing context** that makes this publishable
  now and not 3 years ago.

## 11. Next checkpoint

Re-visit this file when:
- Thesis is defended (post-defense begin §5.1–5.9)
- A workshop CFP appears that fits cleanly
- Industry / collab interest emerges
- A major related paper publishes that changes positioning

## 12. References (working list, expand as written)

See `FutureWork.md` FW17 for the current vetted reference list. To
preserve here when paper writing begins, but no point duplicating
now.

---

**Living-doc rule**: every time we learn something material to this
paper (new experiment that strengthens a claim, a new related paper,
a venue change), edit this file and date the change. This is the
canonical AirREPL paper plan.
