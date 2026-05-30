# TD-S10-B6 - Research SentAI Prime Command Dictionary And Mission Executive

## Purpose

Research and evaluate the implementation directions proposed in
`TD-S10-A6_define_mission_level_robot_primitives.md`.

A6 remains the objective/guideline document.  B6 is the research chapter that
collects evidence from flight software, space operations, robotics executives,
and command/telemetry systems before we implement `sentai.prime`.

B6 should answer:

- what concepts we should borrow;
- what concepts are too heavy for SentAI;
- how those concepts map to our REPL/radio channel;
- how they map to B3/B4 migrated C++ controllers/tasks;
- what the first practical `sentai.prime` design should look like.

B6 is not an implementation task yet.  It may propose implementation slices,
but code changes should wait until the research conclusions are accepted.

## Current Hypothesis

SentAI needs a small, explicit mission-command layer:

```text
host/LLM/operator
  -> host-side primitive catalog
  -> compact REPL/radio command
  -> SentAI REPL / sentai.prime / sentai.explore
  -> primitive validates and runs in C++ or cooperative MP
  -> compact ACK/RUN/OK/ERR + sentai.fr evidence
  -> host-side summary/plots/MCP tools
```

The strongest emerging pattern is:

- document commands explicitly host-side, like F Prime / Mars rover command
  dictionaries, without forcing a formal onboard schema;
- execute long-running work asynchronously, like ROS Actions / spacecraft
  command sequences;
- keep a small observable/interruption state layer, like the useful core of
  Astrobee's executive;
- keep cyclic camera/control work in C++ tasks, like F Prime/cFS rate groups and
  our existing PrepTask model;
- keep radio messages compact and sequence-correlated, like CCSDS-style command
  and telemetry discipline;
- keep MCP and rich schemas host-side.

Terminology convention:

- **board-side** means the SentAI runtime on the device: MicroPython REPL,
  C++ tasks, PrepTask, onboard perception, local world model, safety, and
  primitive controllers.
- **host-side** means the external operator/LLM/agent environment: simulator
  analysis, artifact interpretation, MCP/tool schemas, natural-language
  reasoning, and command selection.

The board-side system is not a stateless executor.  It may maintain a local
world model that is updated continuously from camera, marker, flow, object, and
SLAM/place evidence.  Source review shows that the current model substrates are
already split across `sentai.objects`, `sentai.object_lifter`, `sentai.places`,
and `sentai.slam`.  B6 should keep that reality explicit before proposing any
future compact facade.

The radio dialogue should expose only what the host asks for:

```python
$status()
$what()
$find('cat')
$near('cat')
$ask('red_teddy_bear')
$hover('red_teddy_bear')
```

Internally those commands may query board-side object/place/world-model
snapshots.  The host receives compact answers and decides the next intent.

Board-side can also host high-level mission skills.  `sentai.explore` does not
need to be C++ by default.  It can be a set of MicroPython modules that compose
`sentai.prime` commands, query the board-side world model, and expose readable
mission verbs.  This is desirable because a coding-capable LLM/operator can
author or modify high-level skills much faster in MP/Python than in C++.

Safety boundary:

- MP/Python high-level skills may compose, query, sequence, and decide;
- MP/Python skills must not run camera-rate or control-rate loops;
- MP/Python skills must not stream low-level RPYT/hover setpoints directly;
- C++ controllers remain responsible for fast perception/control, command transport,
  watchdogs, abort, and terminal status;
- every MP high-level skill should be cancellable and should return quickly or
  run as a cooperative/tick-based mini-executive.

This gives a useful split:

```text
sentai.explore      -> board-side high-level MP mission skills
sentai.prime       -> board-side public action/query/recovery command surface
sentai.* C++ controllers -> board-side fast perception/control/safety services
host/LLM           -> host-side reasoning, coding, simulation, approval
```

Example future shape:

```python
# /lib/explore/local_search.py
META = {"name": "local_search", "kind": "mission_skill"}

def start(target='cat', radius='near', t=20):
    sentai.prime.hold(z=0.6)
    sentai.prime.find(target, t=5)
    # Later ticks inspect world-model updates and choose the next prime action.
```

The key is that `sentai.explore` can be programmable and expressive without
owning unsafe low-level loops.

## Two-Agent Workflow

B6 should distinguish two host-side agent roles:

| Role | Purpose | Writes code? | Sends live commands? |
| --- | --- | --- | --- |
| LLM Coding Agent | Designs, edits, tests, and promotes mission primitives/skills. | Yes, primarily in simulator. | Usually no live vehicle control except test harnesses. |
| LLM Command Agent | Operates the drone through REPL/radio using the accepted taxonomy. | No normal code edits; chooses commands/intents. | Yes, live commands and queries. |

Proposed workflow:

1. The LLM Coding Agent works in SIM first.
2. It writes or modifies MP/Python mission skills on the simulated SentAI FS.
3. It runs simulator missions and studies logs/plots/artifacts.
4. Once a skill is stable, it updates the host-side primitive catalog/taxonomy
   and promotes the skill as an accepted board-side primitive.
5. The accepted skill files are copied to the physical board FS.
6. The LLM Command Agent later uses only the accepted taxonomy and REPL/radio
   command surface to operate the real drone.

This keeps coding, experimentation, and verification separate from operational
commanding.  It also gives the host-side Command Agent a stable vocabulary:

```text
operator -> Command Agent -> $find('cat') / $hover('red_teddy_bear')
Coding Agent -> simulator FS -> skill file -> validated taxonomy -> board FS
```

Important guardrail: the Command Agent should not improvise new code on the
physical board during normal operation.  It may ask for status, call accepted
skills, cancel them, or enter recovery.  New skill authoring belongs to the
Coding Agent pipeline and should pass through SIM first.

This mirrors space/robot operations practice:

- mission planners/tooling build and validate command sequences before use;
- operators send accepted commands/sequences to the vehicle;
- onboard software executes a known, bounded command set and reports status.

## Research Matrix

| Direction | Source / system | What it teaches SentAI | Fit for B6 |
| --- | --- | --- | --- |
| Command dictionary | F Prime, Mars Pathfinder rover, Mars rover tools | Commands must have names, IDs, args, preconditions, generated/known telemetry, status, and errors. | Very strong.  This is likely the core of `sentai.prime`. |
| Events/telemetry/parameters | F Prime, cFS, CCSDS | Separate command intent from events, current telemetry/status, persisted tunables, and data products. | Very strong.  Maps to `sentai.fr`, status tuples, future defaults. |
| Executive command gate | Astrobee, rover sequence engines | Accept/reject commands based on operating state, mobility state, fault state, and controller conflicts. | Very strong.  This should exist before dispatch. |
| Command sequences | Voyager, Mars rovers, F Prime/cFS sequencers | Sequences are planned, uploaded, status-tracked, and often designed for delayed feedback. | Strong, but host-side first.  Onboard minimal sequencing only if needed. |
| Rate groups / cyclic work | F Prime, cFS | Time-critical loops belong to scheduled/cyclic C++ work, not interactive scripting. | Very strong.  Matches B5 migration. |
| Software bus / app framework | cFS | Decouple apps/services and route telemetry/commands through explicit channels. | Conceptually useful, but too heavy to copy. |
| Packet discipline | CCSDS | Include kind, controller/app, sequence ID, time, length/payload discipline. | Strong as compact convention; not full CCSDS. |
| Rover autonomy | Mars rovers | Human/agent selects intent; onboard autonomy handles local hazards/perception/closed-loop motion. | Strong.  SentAI should own high-level perception/visual-servo intent, not autopilot internals. |
| Ingenuity | F Prime on Mars helicopter | Reusable embedded flight software services: commanding, telemetry, parameters, sequencing. | Strong as validation of F Prime-like service split on a small flying robot. |
| Voyager | long-lived low-bandwidth mission ops | Baseline sequences, overlay/mini-sequences, fault protection, extreme command/result latency. | Useful mindset: small robust command set and autonomous safing. |
| ROS 2 Actions | robotics middleware | Goal/feedback/result/cancel lifecycle. | Strong semantics, but not transport/runtime. |
| Behavior Trees / Nav2 | robotics task composition | Actions/conditions/recovery and halt semantics. | Useful for future host-side planning; avoid onboard tree runtime for now. |
| MCP | agent tooling | Rich tool schema and resource discovery for LLMs. | Host-side only; translate to compact radio calls. |
| IAMSAR / USCG SAR | mission vocabulary | Search patterns, datum, contact, investigate, track, report, recovery. | Strong for high-level `sentai.explore`, after `sentai.prime`. |

## SOTA Comparison Table

This table compares SentAI's proposed B6 direction against nearby systems.  The
goal is orientation, not feature parity.

| System / literature | SOTA concept | Similarity to SentAI | What to adopt | What to avoid | B6 decision pressure |
| --- | --- | --- | --- | --- | --- |
| F Prime | Component-owned commands, events, telemetry channels, parameters, dictionaries. | Very close to desired `sentai.prime` command surface and host dictionary. | Explicit command metadata; controller/component, args, result fields, event names, parameter separation. | FPP/codegen and full GDS onboard in v1. | High.  Use as primary command-dictionary reference. |
| FPP | Source-of-truth modeling language for F Prime components/types/ports/commands. | Similar to possible future command schema generator. | Think in one source of truth that can generate C++/MP/host schemas. | Introducing a modeling language before the command set stabilizes. | Medium.  Revisit after first hand-written dictionary. |
| cFS | Core flight executive, software bus, command ingest, telemetry output, scheduler, health/safety, stored commands. | Similar service separation, but much larger. | Separate command ingest, safety, telemetry/status, recorder, sequence service. | Full software bus/app framework. | Medium-high.  Use for service naming and separation. |
| CCSDS | Packet discipline: kind/type, APID/app, sequence count, timestamp, length. | Similar to compact radio reply needs. | `#seq`, `CMD/ACK/RUN/OK/ERR/EVT/STAT`, controller/app IDs, optional timestamp. | Full CCSDS packet stack over CRTP text. | High.  Adopt the discipline, not the format. |
| Astrobee executive | Robot state machine accepts/rejects commands by operating/mobility/fault state and forwards to the active controller. | Very close to our minimal executive need. | `ready/running/holding/recovery/fault`, active controller, status/cancel/abort routing. | Full ROS/Astrobee plan executive. | High.  Best reference for the minimal executive. |
| Mars rovers | Ground builds sequences; onboard autonomy executes local closed-loop behavior and fault protection. | Very close host/board split: host plans, board protects/executes. | Host-side validation, onboard local autonomy, command/result evidence, visual odometry mindset. | Heavy onboard planners for v1. | High.  Good operational analogy for LLM Coding/Command Agents. |
| RP-check | Rule-based validation of command sequences before uplink. | Similar to host-side command compatibility and safety checks. | Host validates command combos before radio; onboard still rejects unsafe state. | Complex rule engine onboard. | Medium-high.  Host-side first. |
| CASPER / rover planning | Continuous planning/scheduling/execution for rover autonomy. | Similar long-term aspiration for host-side LLM planning. | Adaptive host-side planning ideas and artifact-driven replanning. | Onboard AI planner in B6. | Medium.  Inspires future Command Agent, not first board runtime. |
| Ingenuity | Small flight vehicle using F Prime services. | Validates F Prime-like command/telemetry/parameter discipline on a constrained flyer. | Keep services reusable; treat commanding/telemetry/parameters as core. | Conflating SentAI with flight autopilot. | Medium-high.  Good evidence for discipline on small aircraft. |
| Voyager operations | Baseline/overlay/mini sequences, fault protection, low-bandwidth delayed ops. | Similar narrow-link command discipline and autonomous safing needs. | Default baseline behavior, overlays, mini-sequences, link-loss recovery. | Deep-space complexity. | Medium.  Useful mindset for fallback and radio grammar. |
| ROS 2 Actions | Goal, feedback, result, cancel lifecycle. | Very close semantics for long-running primitive calls. | ACK/RUN/OK/ERR + cancel/result lifecycle. | ROS transport/DDS onboard. | High.  Use semantics, not stack. |
| Behavior Trees / Nav2 | Actions, conditions, running/success/failure, recovery/halt. | Similar host-side mission composition and recovery thinking. | Vocabulary for running/success/failure, recovery, halt/cancel. | Onboard BT runtime/XML in MP. | Medium.  Host-side planning reference. |
| State machines | Explicit states/transitions with deterministic behavior. | Very close to minimal board executive. | Small state machine for `ready/running/holding/recovery/fault`. | Huge mode matrix. | High.  Use a tiny explicit executive state machine. |
| MCP | Tool/resource schemas for agents. | Very close to host-side LLM integration. | Export the host-side primitive catalog as tools/resources. | MCP payloads over radio. | High host-side, low onboard. |
| IAMSAR / USCG SAR | High-level search and rescue vocabulary: datum, area, track, contact, investigate, recovery. | Close to future mission-language layer. | Put SAR/inspection verbs under `sentai.explore`. | Leaking SAR jargon into low-level runtime APIs. | Medium.  After `sentai.prime` stabilizes. |

Where SentAI appears close to SOTA:

- explicit host-side primitive catalog;
- command/result sequence IDs;
- state-gated command acceptance;
- async controller execution;
- command/event/status separation;
- host-side planning with onboard bounded autonomy;
- compact radio grammar with richer host schema.

Where SentAI intentionally differs:

- no heavy middleware onboard;
- no JSON/ROS/MCP over the radio path;
- no onboard general planner in B6;
- MicroPython is for cooperative skills and orchestration, not fast loops;
- C++ controllers retain camera-rate/control-rate work and safety-critical streams.

This comparison suggests the first implementable architecture should be:

```text
tiny executive state machine
  + host-side primitive catalog
  + active primitive/controller table
  + sequence-correlated ACK/RUN/OK/ERR
  + C++ async controllers
  + cooperative MP skills only where safe
  + host-side MCP/tool export
```

## Papers And Technical Reports To Read

B6 should include actual research papers and technical reports, not only
framework documentation.  The following papers are relevant and should be read
for ideas, vocabulary, and tradeoffs before implementation.

| Paper / report | Topic | Why it matters for SentAI |
| --- | --- | --- |
| **F Prime: An Open-Source Framework for Small-Scale Flight Software Systems** | F Prime architecture for CubeSats, SmallSats, instruments. | Direct inspiration for command dictionaries, reusable components, events, telemetry, parameters, and small-flight-system discipline. |
| **FPP: A Modeling Language for F Prime** | Modeling commands, ports, components, and types for F Prime. | Useful if we later want a source-of-truth format that can generate C++ tables, MP metadata, and host/MCP schemas. |
| **RP-check: An Architecture for Spaceflight Command Sequence Validation** | Checking rover command sequences against flight rules before uplink. | Important for SentAI command preconditions, safety rules, and host-side validation before radio transmission. |
| **Automatic Rover Command Generation / CASPER papers** | Continuous planning, scheduling, execution, and dynamic command sequence generation for rovers. | Helps decide what belongs host-side in the smart agent versus onboard in `sentai.prime`. |
| **Embedding a Scheduler in Execution for a Planetary Rover** | Scheduling during execution for Mars 2020-like rover operations. | Useful for future adaptive mission execution, but likely too heavy for onboard B6. |
| **SciBox: an end-to-end automated science planning and commanding system** | Automated planning, scheduling, command generation, validation, and operations workflow. | Strong host-side analogy for LLM/operator planning that generates compact SentAI commands. |
| **A Survey of Behavior Trees in Robotics and AI** | BT semantics, running/success/failure, actions, conditions, recovery, composition. | Good vocabulary for task status and recovery, but not a reason to run BTs onboard. |
| **Behavior Trees and State Machines in Robotics Applications** | DSL comparison between BTs and state machines in ROS-style robots. | Helps evaluate whether SentAI should use state machines, BT-like host plans, or command dictionaries. |
| **Formalisms for Robotic Mission Specification and Execution: A Comparative Analysis** | Compares BTs, state machines, HTNs, BPMN for robot missions. | Directly relevant to A6/B6: how to represent mission-level commands without overengineering. |
| **RoBen: benchmarking planetary rover planning and scheduling algorithms** | Rover planning/scheduling benchmark. | Useful later if `sentai.explore` becomes planning-heavy; likely not needed for first `sentai.prime`. |
| **Assurance for Autonomy -- JPL's past research, lessons learned, and future directions** | Assurance of autonomous robotic/space systems. | Useful for safety framing: bounded autonomy, checks, fallbacks, and evidence. |
| **SeqN / Aerie sequencing documentation** | Plaintext authoring format for spacecraft commands/sequences. | Closest analogue to our readable REPL/radio command grammar, but larger and ground-tool oriented. |

Initial paper-derived hypotheses:

- The onboard layer should stay closer to **callable primitives + minimal
  lifecycle/status/recovery discipline** than to a full planning system.
- Planning, validation, schedule generation, and rich schemas should mostly live
  host-side, where an LLM/MCP adapter can reason with more context.
- Onboard should own only what must remain responsive under link loss:
  safety, current controller, command status, cancel/abort/land, and local
  perception/control tasks.
- Every host-documented command should describe expected preconditions and
  telemetry/events,
  because that is the recurring pattern in rover/F Prime/cFS operations.
- A future sequence language should probably be a tiny subset of the REPL
  grammar, not a separate DSL.

Paper/source links:

- F Prime paper: <https://digitalcommons.usu.edu/smallsat/2018/all2018/328/>
- FPP repository: <https://github.com/nasa/fpp>
- RP-check: <https://robotics.jpl.nasa.gov/media/documents/rp_check_v4.pdf>
- CASPER / automated rover command generation:
  <https://ai.jpl.nasa.gov/public/documents/papers/spaceops00.pdf>
- Continuous planning and execution for an autonomous rover:
  <https://ai.jpl.nasa.gov/public/documents/papers/nasapswkshop02-estlin.pdf>
- Increased Mars Rover Autonomy using AI Planning:
  <https://ai.jpl.nasa.gov/public/documents/papers/estlin-icra07-marsrover.pdf>
- Embedding a Scheduler in Execution for a Planetary Rover:
  <https://ojs.aaai.org/index.php/ICAPS/article/view/13909>
- SciBox: <https://www.sciencedirect.com/science/article/pii/S0094576512003682>
- Behavior Trees survey:
  <https://www.sciencedirect.com/science/article/pii/S0921889022000513>,
  <https://arxiv.org/abs/2005.05842>
- Behavior Trees and State Machines in Robotics:
  <https://arxiv.org/abs/2208.04211>
- Robotic mission specification formalisms:
  <https://arxiv.org/abs/2603.15427>
- RoBen planetary rover planning benchmark:
  <https://openresearch.surrey.ac.uk/esploro/outputs/journalArticle/RoBen-Introducing-a-benchmarking-Tool-for/99513894402346>
- Assurance for autonomy:
  <https://arxiv.org/abs/2305.11902>
- SeqN / Aerie:
  <https://ammos.nasa.gov/aerie-docs/sequencing/seqn/>

## F Prime Evaluation

F Prime is a NASA/JPL C++ framework for embedded and spaceflight systems.  It
provides components, typed ports, queues/threads/OS abstraction, commands,
events, telemetry channels, parameters, dictionaries, autocoding, a ground data
system, and testing tools.

Why it matters for B6:

- it treats command surfaces as first-class artifacts;
- commands have typed arguments, names/IDs, dispatch paths, and response status;
- events and telemetry are separate from commands;
- parameters are stored tunables, not ad-hoc globals;
- a dictionary lets ground tools know exactly what the flight software supports;
- components own work, instead of arbitrary scripts mutating shared state.

What to borrow:

```text
command name
short radio spelling
kind
controller
args/defaults
preconditions
start/status/cancel/result hooks
terminal result fields
events emitted
```

What not to borrow:

- no onboard FPP/codegen dependency for v1;
- no full F Prime GDS;
- no heavy topology framework in MicroPython;
- no generic scheduler/OS in `sentai.prime`.

SentAI interpretation:

```python
{
    "name": "takeoff",
    "short": "to",
    "kind": "action",
    "controller": "sentai.servo.acquire",
    "args": (("target", "str", "m"), ("n", "int", 3), ("t", "float", 8.0)),
    "pre": ("camera_ready", "armed_or_armable", "controller_free_or_recovery"),
    "result": ("ok", "reason", "lock", "z", "thrust"),
}
```

This shape is illustrative.  B6 should decide whether the source of truth is
C++, Python `META`, or a host-side file that can generate both.

Sources:

- <https://nasa.github.io/fprime/>
- <https://fprime.jpl.nasa.gov/latest/docs/reference/system-functional/dictionary/>
- <https://fprime.jpl.nasa.gov/v3.6.1/docs/user-manual/overview/cmd-evt-chn-prm/>
- <https://fprime.jpl.nasa.gov/devel/docs/user-manual/framework/component-and-port-selection/>
- <https://github.com/nasa/fprime>
- <https://github.com/fprime-community/>

## cFS Evaluation

NASA cFS is a reusable flight software framework with platform support, OS
abstraction, a core flight executive, scheduling, inter-process communication,
error management, command ingest, telemetry output, health/safety, stored
commands, housekeeping, data storage, and common apps.

What to borrow:

- separate command ingest from task execution;
- separate telemetry/status output from event recording;
- separate health/safety from normal mission commands;
- treat stored/simple sequences as a service, not as arbitrary blocking script;
- keep runtime services as explicit controllers.

What not to borrow:

- no full cFS app framework;
- no software bus for B6;
- no mission-class framework complexity;
- no attempt to replace our existing FreeRTOS/sentai_runtime structure.

SentAI mapping:

| cFS-like concept | SentAI candidate |
| --- | --- |
| command ingest | REPL/radio dispatcher |
| telemetry output | compact status tuples / host plots |
| event/data storage | `sentai.fr` |
| health/safety | `sentai.safety` |
| scheduler/cyclic apps | PrepTask and C++ servo/flow/marker tasks |
| stored commands | future host-side or `/main.py` mini-sequences |

Sources:

- <https://etd.gsfc.nasa.gov/capabilities/capabilities-listing/cfs/>
- <https://github.com/nasa/cFS>

## CCSDS Evaluation

CCSDS is the standards family behind many space telemetry/telecommand packet
systems.  The useful lesson for SentAI is packet discipline, not the full
standard.

Useful fields:

| Field | SentAI equivalent |
| --- | --- |
| packet type | `CMD`, `ACK`, `RUN`, `OK`, `ERR`, `EVT`, `STAT` |
| APID/application ID | controller/module: `prime`, `servo`, `safety`, `fr`, `flow` |
| sequence count | `#17` command/result correlation |
| optional time | host/runtime/sim timestamp alignment |
| packet length | robust parser if we move beyond CRTP text |
| payload | compact key/value radio text now, binary later if needed |

SentAI should start with readable text:

```text
#17 CMD takeoff m n=3 t=8
#17 ACK takeoff ctrl=servo_acquire
#17 RUN takeoff phase=acquire n=5 z=0.42
#17 OK takeoff lock=7 z=0.61
#17 ERR takeoff reason=marker_timeout
```

Source:

- <https://docs.ccsdspy.org/en/latest/user-guide/ccsds.html>

## Astrobee Evaluation

Astrobee is the closest "robot executive" reference in this research set.  Its
executive tracks operating and mobility states, accepts or rejects commands
based on those states, forwards accepted commands to owner nodes, reports
command status, and reacts to fault/blocking conditions.

What to borrow:

- a small command acceptance gate before dispatch;
- explicit operating/mobility/perception/owner states;
- command rejection with reason, not silent failure;
- fault/recovery state that changes which commands are allowed;
- heartbeat/stale-owner thinking.

SentAI initial gate:

| State dimension | Examples | Command impact |
| --- | --- | --- |
| operating | boot, ready, active, fault, recovery | Reject normal motion in boot/fault; allow status/recovery. |
| mobility | grounded, armed, taking_off, hovering, landing | Reject `goto` before takeoff; allow `abort` anytime. |
| perception | camera_ready, marker_lock, flow_ready, object_ready | Gate marker/object/flow-dependent commands. |
| owner | none, calib, servo, landing, recovery | Prevent conflicting owners; allow compatible maintainers. |

Source:

- <https://nasa.github.io/astrobee/v/develop/executive.html>

## Mars Rover Evaluation

Mars rover operations are a strong analogy because the ground team issues
high-level intent and command sequences, while onboard software handles local
closed-loop behavior, hazard checks, visual odometry, and fault protection.

Relevant lessons:

- command sequences are built from dictionaries and checked before uplink;
- rover flight software receives sequence commands and generates high- and
  low-level motion behaviors;
- low-level motor control is separate and runs at high rate;
- autonomy extends directed drives into terrain not fully evaluated by humans;
- visual odometry is used as a safety-critical local estimator, not just a plot.

SentAI mapping:

| Rover pattern | SentAI mapping |
| --- | --- |
| rover planners choose route/intent | host/LLM/operator chooses primitive/goal |
| command sequence engine | future host-side or `/main.py` mini-sequencer |
| subsystem FSW owns behaviors | C++ task owners in `sentai.servo`, `sentai.flow`, `sentai.markers` |
| motor-control FSW owns fast loops | Crazyflie/PX4 autopilot and SentAI C++ streaming loops |
| visual odometry/slip detection | marker/flow/pose evidence and stale/loss gates |

Sources:

- <https://robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-science-laboratory/surface-system-software-and-rover-navigation/>
- <https://pds.nasa.gov/data/mpfr-m-rvrcam-5-midr-v1.0/mprv_0001/document/roverdoc/cmddict/cmddict.htm>
- <https://pds.nasa.gov/data/mpfr-m-apxs-5-ddr-v1.0/mprv_0001/document/roverdoc/cmddict/cmddesc.htm>

## Ingenuity Evaluation

Ingenuity matters because it is a small flying robot that used F Prime.  NASA
describes F Prime as providing reusable services such as commanding, telemetry,
parameters, and sequencing for spacecraft, and Ingenuity validates that this
style can apply to a constrained flight vehicle.

What to borrow:

- keep reusable services separated from vehicle-specific mission logic;
- command/telemetry/parameter/sequencing services are not "nice to have"; they
  are core operational infrastructure;
- flight-like experiments benefit from framework discipline even when the
  software is small.

What to avoid:

- do not claim SentAI is equivalent to Ingenuity's flight stack;
- do not push autopilot responsibilities into SentAI;
- do not introduce framework weight before B3/B4 primitives are stable.

Sources:

- <https://www.nasa.gov/solar-system/meet-the-open-source-software-powering-nasas-ingenuity-mars-helicopter/>
- <https://fprime.jpl.nasa.gov/overview/powerful-swa/>

## Voyager Evaluation

Voyager is relevant less for robotics and more for command discipline under
extreme constraints.  NASA describes baseline sequences, overlay sequences,
mini-sequences, long-term event tables, backup mission loads, and onboard fault
protection as core operating concepts.  The mission also demonstrates why every
command needs careful state reasoning when response latency is high.

What to borrow:

- keep a small robust baseline behavior active even if new commands stop;
- distinguish baseline, overlay, and one-shot/mini-sequences;
- make recovery/fault protection independent of operator round-trip time;
- keep sequence memory and command bandwidth constraints explicit;
- preserve documentation because long-lived systems outlive people's memory.

SentAI mapping:

| Voyager concept | SentAI candidate |
| --- | --- |
| baseline sequence | default safe services: camera/markers/flow/status/heartbeat |
| overlay sequence | temporary mission goal such as scan or hover |
| mini-sequence | one-shot command such as sample, land, report |
| backup mission load | safe recovery/land behavior if command link is lost |
| fault protection | `sentai.safety` stale camera, owner timeout, marker loss, safe stop |

Sources:

- <https://science.nasa.gov/mission/voyager/science/>
- <https://science.nasa.gov/blogs/voyager/2023/12/12/engineers-working-to-resolve-issue-with-voyager-1-computer-2/>
- <https://science.nasa.gov/learn/basics-of-space-flight/chapter11-1/>

## Robotics / ROS / Behavior Tree Evaluation

ROS 2 Actions and Behavior Trees give good semantics for long-running robot
tasks:

- goal/start;
- feedback/running status;
- result;
- cancel/halt;
- recovery branches;
- condition checks before actions.

What to borrow:

- the lifecycle vocabulary;
- "running/success/failure" status shape;
- explicit cancel/halt behavior;
- recovery as a first-class path.

What not to borrow:

- no ROS graph/DDS onboard;
- no XML/BT runtime over the radio;
- no generic behavior tree executor in MP for B6.

## MCP / Agent Evaluation

MCP fits the host side, not the radio side.  The host can expose SentAI
commands as tools/resources with schemas and descriptions.  The onboard system
should receive compact REPL-like commands.

Mapping:

| MCP concept | SentAI mapping |
| --- | --- |
| tool schema | host export of the SentAI primitive catalog |
| tool call | compact `$takeoff('m', n=3)` radio command |
| resource | logs, plots, artifacts, primitive catalog |
| prompt/skill | host-side mission planning guidance |

Important constraint: MCP payloads are too verbose for the normal onboard radio
path.  The host translates.

## IAMSAR / Mission Vocabulary Evaluation

IAMSAR and USCG SAR vocabulary is useful for high-level commands, not for B6's
first implementation layer.  It should inform `sentai.explore` after
`sentai.prime` is stable.

Useful terms:

- datum;
- commence search point;
- search area;
- track spacing;
- sweep width;
- coverage;
- parallel track;
- creeping line;
- expanding square;
- sector search;
- contact/sighting;
- investigate;
- overwatch/loiter;
- report/recovery.

SentAI should translate those into `sentai.prime` actions and perception
queries, rather than making the low-level runtime speak SAR jargon.

## Emerging B6 Design Constraints

B6 should recommend the following constraints before implementation:

- SentAI receives **intentions** over radio, not low-level pilot-control
  scripts.  Examples: "take off and stabilize", "what do you see?", "search
  near you for a cat", "what is near the cat?", "is there a red teddy bear?",
  "hover over the teddy bear".
- The host/LLM is the deliberative planner.  It interprets taxonomy, decides the
  next intent, and expects a compact response so it can continue reasoning
  host-side.
- MP/REPL must remain responsive while a long-running command is active: either
  by returning after starting an async owner, or by running a cooperative MP
  loop that reaches scheduler/executive checks often enough.
- Camera-rate and control-rate loops remain in C++ tasks.
- Every public command has a host-side catalog entry that reflects what is
  loaded on the current board image / FS.
- Every public command, including quick queries, has a sequence ID.
- Every accepted long-running command must create an active record immediately,
  so `$status()` and `$active()` can report that it has started, is ongoing, or
  is stopping/recovering.
- Every long-running command produces either immediate `ERR` or `ACK`, then
  terminal `OK` or `ERR`.  Immediate queries produce a sequence-correlated
  `OK`, `ERR`, `STAT`, or `ACTIVE` reply.
- `$status()` and `$active()` must remain available during active commands.
- Recovery commands bypass normal owner conflicts.
- `abort` / `cancel` / `land` semantics must be defined for every long-running
  owner, even if the first version only supports `abort -> safe recovery`.
- The drone should always have a safe fallback behavior: hold/stabilize when
  possible, otherwise return-home/land or safe land depending on available
  localization and link state.
- Host-side summaries come from `sentai.fr` and status samples, not onboard
  JSON.
- The host-side primitive catalog should be exportable to a future MCP adapter.

## Minimal Executive State Machine

B6 should define a small mission-executive state machine.  The goal is not to
invent an OS; it is to answer simple operational questions:

- is the drone waiting for a command?
- is a command running?
- what primitive/controller owns motion/perception/logging?
- can the command be cancelled or aborted?
- if interrupted, what state do we return to?
- what is the safe fallback if the command fails?

Initial high-level states:

| State | Meaning | Typical allowed commands |
| --- | --- | --- |
| `boot` | Runtime starting; services not ready. | `status`, diagnostics. |
| `ready` | Services ready, grounded or idle, waiting for command. | queries, setup, takeoff, log, calibration-safe commands. |
| `running` | A long-running action/goal/maintainer is active. | `status`, `active`, compatible queries, `cancel`, `abort`, `land`. |
| `holding` | Vehicle is stable/loitering/hovering after a command or fallback. | queries, next goal, release/land/abort. |
| `recovery` | Safe-stop, abort, return-home, or landing path is active. | `status`, emergency commands only. |
| `fault` | Something unsafe or inconsistent happened. | `status`, diagnostics, recovery/land if possible. |

Long-running command lifecycle:

```text
idle/ready
  -> accept command
  -> ACK + insert active record
  -> running phase updates
  -> OK -> holding or ready
  -> ERR -> holding/recovery/fault depending on failure
  -> cancel -> stopping -> holding or ready
  -> abort -> recovery -> holding/landed/fault
```

The active record is the bridge between the active primitive/controller and
REPL:

| Field | Purpose |
| --- | --- |
| `seq` | Correlate radio command and delayed result. |
| `active_primitive` | Public command name/spelling, e.g. `takeoff`, `hover`, `scan`. |
| `kind` | query, service, action, goal, maintainer, recovery. |
| `controller` | C++/MP controller that can report status/cancel. |
| `state` | accepted, running, stopping, recovering, done, error. |
| `phase` | Controller-specific phase, e.g. acquire, climb, center, descend. |
| `started_ms` | Runtime timestamp for timeout/status. |
| `timeout_ms` | Controller timeout or zero if not applicable. |
| `return_state` | Expected state after OK/cancel. |
| `fallback` | Recovery command/state if owner fails. |
| `cancel_mode` | none, cooperative, immediate_safe_stop, land. |
| `last_reason` | Last error/reject/progress reason. |

Status examples:

```text
#31 STAT op=ready active=0 mob=grounded cam=1 flow=1 world=ok
#32 STAT op=running active=1 #21:hover phase=center err=8.4 z=0.61
#33 ACTIVE n=1 #21:hover ctrl=servo state=running cancel=1 abort=1
```

Interrupt semantics to research/define:

| Interrupt | Meaning | Expected return |
| --- | --- | --- |
| `cancel` | Stop the current non-emergency command cooperatively. | `holding` or `ready`, based on mobility. |
| `release` | Stop a maintainer/constraint. | previous compatible mode or `holding`. |
| `abort` | Preempt current owner and enter safe recovery. | `recovery`, then `holding`, `landed`, or `fault`. |
| `land` | Preempt into landing if safe/possible. | `landed` or `fault`. |

This is the missing "state-machine" piece: not a large planner, but a minimal
mission-executive lifecycle so long-running work is visible, cancellable, and
recoverable.

## Current REPL Interrupt Behavior

Current `examples/sentai_runtime/micropython_task.c` already has a Ctrl-C
monitor while Python code executes:

- `ctrlc_monitor_task` polls console input while a script/line is running;
- byte `0x03` calls `mp_sched_keyboard_interrupt()`;
- the MicroPython VM raises `KeyboardInterrupt` when pending exceptions are
  handled;
- `sentai.rtos.sleep_ms` and similar chunked waits are expected to call
  `mp_handle_pending(...)` periodically.

So the current REPL can break some running Python code with Ctrl-C, but this is
not enough for B6's command model.

Important distinction:

| Case | Can Ctrl-C help? | B6 implication |
| --- | --- | --- |
| Pure Python loop that yields/checks pending often enough | Yes, raises `KeyboardInterrupt`. | Useful for diagnostics, not primary control. |
| Python code blocked in a C binding that does not check pending exceptions | Maybe not until the binding returns. | Long waits in bindings must be chunked or async. |
| C++ task started asynchronously, then REPL returns | Ctrl-C is irrelevant; use `$cancel()` / `$abort()` / `$land()`. | Preferred model for flight commands. |
| Python command starts C++ task then waits cooperatively | Ctrl-C may break the wait if the loop sleeps/yields.  Radio-injected commands can also run when the loop reaches scheduler/pending hooks. | Acceptable only if it pumps the executive and has explicit status/cancel/abort semantics. |
| C++ owner streams flight commands | Ctrl-C only affects MP caller, not necessarily the owner. | Owner must expose explicit abort/cancel/safe-stop. |

B6 conclusion: Ctrl-C is an emergency/developer convenience, not the command
interrupt model.  Normal long-running primitives should:

1. start a C++/service owner;
2. create an active record;
3. return to REPL quickly;
4. expose status through `$status()` / `$active()`;
5. expose explicit `$cancel()`, `$abort()`, `$land()` routes.

Important SentAI-specific nuance: the current runtime already supports a form
of radio/REPL injection while Python is sleeping.  `sentai.rtos.sleep_ms()`
splits long sleeps into small slices and calls `mp_handle_pending(true)`, so
scheduled radio dispatch can execute while a cooperative MP loop yields.  This
means the problem is not `while` by itself.  The problem is a non-cooperative
loop or blocking C binding that never reaches the MP scheduler, never updates an
active record, and cannot observe `$abort()` / `$cancel()` / `$land()`.

Therefore B6 should allow cooperative public MP primitives, but avoid
non-yielding blocking waits in operational commands except for explicit
debug/admin helpers.

## Cooperative MP Skills

Some high-level board-side skills may naturally be written in MicroPython first,
for example:

```python
def follow(obj):
    while True:
        # inspect world model / object snapshot
        # call sentai.prime.hover(...) or update a C++ owner target
        # decide whether target is still valid
        sentai.rtos.sleep_ms(50)
```

This is acceptable only if it becomes a **cooperative skill**, not a
non-yielding REPL trap.  The skill must periodically give MicroPython's
scheduler and the command executive a chance to process radio-injected
commands, and it must observe active-record state.

Required cooperative points:

- reach a scheduler/pending hook regularly, normally via
  `sentai.rtos.sleep_ms(...)`;
- check whether this command was cancelled;
- check whether `$abort()` or `$land()` was requested;
- update its active record phase/status;
- call only non-blocking or short bounded operations;
- sleep/yield in small chunks;
- leave the vehicle in a known state on exit, preferably `holding` or
  `recovery`.

Conceptual shape:

```python
def follow_tick(ctx):
    sentai.prime.pump()
    if ctx.abort_requested():
        sentai.prime.abort_owner(ctx.owner)
        return "recovery"
    if ctx.cancel_requested():
        sentai.prime.hover(ctx.last_safe_ref)
        return "holding"
    obj = sentai.objects.get(ctx.target)
    if obj is None:
        sentai.prime.hover(ctx.last_safe_ref)
        return "holding"
    sentai.prime.update_hover_target(obj)
    return "running"
```

Preferred shape: the outer loop is owned by the resident executive, not by a
REPL line that never yields:

```text
REPL/radio command -> start MP skill -> active record
executive tick -> skill.tick(ctx)
urgent radio abort -> ctx.abort_requested -> owner safe-stop
```

If v1 does not have a resident MP executive yet, then long-running MP skills
can still be written as cooperative loops, but they must sleep/yield often
enough for radio injection, status, cancel, and abort to work.  Prefer C++
async owners for flight-critical fast loops.

Exit policy examples:

| Running skill | Abort/cancel behavior | Return state |
| --- | --- | --- |
| `follow('cat')` | stop updating follow target; command hover/hold on last safe reference if possible | `holding` |
| `scan('area')` | stop scan motion; recenter/hold or land if localization is weak | `holding` or `recovery` |
| `investigate('o4')` | stop approach; hold current view/position | `holding` |
| `descend('slow')` | cancel only if safe; abort may continue landing | `recovery` or `landed` |

This is a key B6 design choice: MP can host high-level skills, and those skills
may use loops.  What is not acceptable for operational radio control is a
non-cooperative `while True` that never yields, never pumps pending radio work,
and never exposes active/status/cancel/abort.

## Operator / Host Assumptions

For B6, assume the host-side agent knows the public taxonomy and has learned
command compatibility from simulator runs, documentation, and previous mission
artifacts.  It should therefore avoid obviously conflicting commands.  The
onboard executive still needs a small safety gate because:

- the radio command may be stale, malformed, or out of order;
- the current onboard state may differ from the host's belief;
- a command can become unsafe due to marker loss, camera stale, estimator loss,
  low battery, or an active recovery owner;
- the host should receive a clear `ERR reason=...` rather than undefined
  behavior.

This means B6 should not try to build a huge onboard compatibility planner.
The drone only needs enough local judgment to reject impossible/unsafe commands
and enter a known holding/recovery mode.

Board-side state remains important even with a smart host.  The host may ask
"what do you see?" or "what is near the cat?", but the answer should come from
the current onboard world/perception model, not from re-running a full analysis
path in the host.  Host-side reasoning chooses intent; board-side memory
answers local perception/world-state queries and guards execution.

Namespace convention to settle before implementation:

| Concept | Candidate namespace | Note |
| --- | --- | --- |
| raw object detections / object memory | `sentai.objects` | Existing object table and object statuses. |
| object-to-world lifted targets | `sentai.object_lifter` | Bridge from image detections / tracklets to world positions. |
| local semantic map / hex places | `sentai.places` | Existing H3/place substrate; not a speculative future map namespace. |
| SLAM / landmark diagnostics | `sentai.slam` | Low-level diagnostics and pose/landmark source, not the public model-query grammar. |
| high-level exploration/SAR verbs | `sentai.explore` | Board-side MP mission skills; mission vocabulary, not the map artifact itself. |
| public short commands | `sentai.prime` | Radio-facing intent vocabulary. |

## Parameters And Mutability

Parameters should be classified by when they can safely change:

| Mutability | Meaning | Examples |
| --- | --- | --- |
| init-only | Changing requires peripheral/task reinitialization. | camera resolution, pixel format, model tensor shape, marker detector backend. |
| pre-arm | Safe before flight, risky during flight. | marker layout, intrinsics, estimator source selection, calibration contract. |
| inflight tunable | Safe to change while active if owner accepts it. | target altitude, speed limit, scan timeout, hover radius, confidence threshold. |
| command-local | Argument belongs only to one command invocation. | `$takeoff('m', n=3, t=8)`, `$find('cat', t=5)`. |

The host-side primitive catalog should expose mutability.  Board-side setters
or primitive owners should still reject init-only/pre-arm changes during flight
with a clear reason.

## Sequences And Learning

For now, B6 should assume **host-side sequencing**.  The host/LLM sends one
intent, waits for ACK/RUN/OK/ERR or polls status, then reasons about the next
intent.

Onboard mini-sequences are a future possibility for patterns that become common
and safety-critical, for example:

- takeoff -> marker acquire -> stabilize;
- search local area -> inspect top detection -> report;
- hover failure -> hold -> land;
- link loss -> hold timeout -> land.

Learning onboard is not a B6 goal.  The safe near-term version is:

- learn/evaluate patterns host-side from simulator and mission artifacts;
- promote stable patterns into explicit named primitives;
- keep the onboard version deterministic, bounded, inspectable, and cancellable.

So the "magic" version is possible later only as a workflow: host learns or
discovers a pattern, humans approve it, then it becomes a small explicit
primitive or mini-sequence.  It should not mutate flight behavior silently
onboard.

## Deliverable 1 - SOTA Comparison

Status: started.

The SOTA comparison table above is the first B6 deliverable.  It should remain
the orientation map for design decisions:

- F Prime is the command-dictionary reference.
- Astrobee is the executive-gate reference.
- CCSDS is the packet-discipline reference.
- Mars rovers / Voyager are the operational-model references.
- ROS Actions provide long-running action lifecycle vocabulary.
- MCP is host-side schema/tooling, not onboard protocol.
- IAMSAR is high-level mission vocabulary, not low-level runtime API.

Research conclusion so far:

```text
SentAI should implement a small command/executive layer, not import a framework.
```

D1 conclusion:

SentAI is closest to a **small spacecraft/robot command executive**, not to a
full robotics middleware stack.  The best SOTA alignment is a hybrid of:

- F Prime-style explicit host-side command documentation/catalogs;
- Astrobee-style minimal executive state and recovery tracking;
- ROS Action-style long-running command lifecycle;
- CCSDS-style sequence-correlated command/result discipline;
- rover/Voyager-style host-side planning with onboard bounded autonomy;
- MCP-style rich tooling only on the host.

Therefore the next architecture should not be "run ROS/F Prime/cFS onboard".
It should be:

```text
compact REPL/radio command
  -> REPL/function lookup
  -> primitive/controller validation
  -> active record
  -> C++ or cooperative MP controller
  -> ACK/RUN/OK/ERR
  -> host-side reasoning and artifact analysis
```

This keeps SentAI close to proven mission-operations practice while preserving
the constraints that matter here: tiny radio messages, responsive REPL, MP for
composition, C++ for fast/safety-critical loops, and simulator-first skill
authoring.

After D2, the important precision is that "responsive REPL" does not mean
"no long-running MP code".  SentAI already allows radio/REPL work to be
scheduled while MP code reaches pending hooks such as `sentai.rtos.sleep_ms()`.
So D1's architecture allows two safe controller shapes:

- asynchronous C++ controllers for fast or safety-critical loops;
- cooperative MP controllers for high-level skills, provided they yield/pump
  regularly and expose active/status/cancel/abort.

The rejected pattern is not "MP loop"; it is an unobservable,
non-cooperative MP loop.

D1 justification:

1. **Radio bandwidth and latency favor command intent, not rich middleware.**
   ROS 2, MCP, F Prime GDS, and cFS-style buses are too verbose for the normal
   CRTP/text radio path.  A compact REPL command plus sequence-correlated
   replies preserves the same operational semantics with far less payload.

2. **SentAI already has the right execution split.**  B5 moved camera-rate,
   marker, flow, estimator-feed, and control loops toward C++ controllers.  That
   matches F Prime/cFS rate-group thinking and avoids pretending that
   MicroPython can be a real-time flight-control runtime.

3. **The host/board split matches rover operations.**  The host/LLM can plan,
   validate, inspect artifacts, and choose the next intent.  Board-side should
   execute bounded local autonomy, answer from its current world model, and
   protect itself when the host is wrong or late.  The exact board-model query
   taxonomy is still experimental and should be validated in simulator before
   becoming a stable contract.

4. **A minimal executive is necessary for safety but a full planner is not.**
   Astrobee-style operating/mobility/fault awareness gives the minimum local
   structure needed for status, cancel, abort, and recovery.  Detailed
   primitive-specific checks remain inside the primitive/controller.  Full
   behavior trees, schedulers, or planners can stay host-side until the command
   layer is stable.

5. **A host-side primitive catalog makes LLM control inspectable.**  If every
   accepted public primitive has documented spelling, args, behavior, expected
   result, common failure reasons, and examples, the Command Agent can reason
   over a known capability set instead of improvising opaque Python snippets.
   The board implementation can stay as normal C++/MP functions that validate
   and reject at runtime.

6. **Simulator-first skill authoring needs a promotion boundary.**  The Coding
   Agent can create MP/Python skills in SIM, but operational board-side commands
   should be used by the Command Agent only after they appear in the host-side
   catalog/taxonomy for the current board image or FS.  This mirrors
   flight/rover practice: planning and validation happen before uplink.

7. **Cooperative MP fits the existing SentAI REPL model.**  Because
   `sentai.rtos.sleep_ms()` drains pending MP scheduler work, a high-level MP
   skill can run as a loop and still allow radio-injected `$status()`,
   `$active()`, `$cancel()`, and `$abort()` to be processed.  This is useful
   for `sentai.explore` skills that compose C++ controllers and query the local
   world model.  It is not a license for CPU-bound or non-yielding MP control
   loops.

8. **The chosen hybrid degrades safely.**  If a command fails, the board can
   report `ERR reason=...`, update active state, enter holding/recovery/land,
   and let the host reason again.  A heavier onboard framework would not remove
   the need for these local safing semantics.

D1 rejected alternatives:

| Alternative | Why rejected for B6 |
| --- | --- |
| Run ROS 2 onboard | Too heavy; wrong transport assumptions; unnecessary for current C++/MP runtime. |
| Port F Prime/cFS | Useful references, but framework/codegen/GDS overhead does not fit immediate SentAI constraints. |
| Use MCP over radio | Too verbose; MCP belongs host-side as schema/tool layer. |
| Keep ad-hoc REPL commands only | Not inspectable enough for LLM agents or safe mission execution. |
| Put a planner/BT runtime onboard now | Premature; host-side planning and small board-side minimal executive are enough for v1. |
| Let non-cooperative MP loops own flight behavior | Breaks status/abort responsiveness; unsafe for operational radio control. |

## Deliverable 2 - Minimal Command Lifecycle

Recommended lifecycle for public intention commands:

```text
received
  -> parsed
  -> REPL/function lookup
  -> primitive/controller validates or rejects
  -> ACK + active record
  -> controller running
  -> RUN updates / status polling
  -> OK or ERR terminal result
  -> return_state: ready / holding / recovery / fault
```

Immediate query lifecycle:

```text
received -> parsed -> function lookup -> execute getter -> #seq OK/ERR/STAT/ACTIVE
```

Rejected command lifecycle:

```text
received -> parsed -> unknown command or controller validation fail -> ERR reason=...
```

Interrupt lifecycle:

```text
running
  -> cancel/release/abort/land request
  -> controller receives stop request
  -> stopping/recovery phase
  -> OK/ERR terminal result
  -> holding / ready / landed / fault
```

B6 recommendation:

- every public command should have a sequence ID;
- every long-running command must ACK quickly or ERR immediately;
- every accepted long-running command must have an active record;
- terminal OK/ERR must include enough fields for host-side reasoning;
- Ctrl-C is a developer interrupt, not the operational lifecycle.

D2 conclusion:

The correct lifecycle is **observable and interruptible by default for anything
that can last longer than a quick query**.  The REPL/radio path must remain able
to handle `$status()`, `$active()`, `$cancel()`, `$abort()`, and `$land()` while
a command is active.  That can happen either because the command delegated to an
async C++ controller and returned, or because a cooperative MP controller yields/pumps
often enough for scheduled radio work and executive state checks.

Command classes:

| Class | Lifecycle | Active record? | Examples |
| --- | --- | --- | --- |
| query | immediate `#seq OK/ERR/STAT/ACTIVE` | no, except optional last-result | `$status()`, `$what()`, `$active()` |
| service enable/disable | immediate `OK/ERR`, may create/remove service record | yes for persistent services | `$log(on=1)`, `$flow.enable()` |
| action | `ACK -> RUN* -> OK/ERR` | yes | `$takeoff('m')`, `$descend('slow')` |
| goal | `ACK -> RUN* -> OK/ERR/cancel` | yes | `$goto('o4')`, `$find('cat')` if motion-owning |
| maintainer | `ACK -> RUN* until release/cancel/abort` | yes | `$hold(z=0.6)`, `$hover('m')`, `$track('o4')` |
| recovery | preemptive `ACK/ERR -> RUN* -> OK/ERR` | yes | `$abort()`, `$land()` |
| admin/debug | command-specific | maybe | `$sentai.calib.status_tuple()`, `$exec ...` |

The active record is created after the command is accepted and before the
controller starts doing dangerous or long-running work.  If controller start fails, return
`ERR` and do not leave a live active record.

D2 recommended state transition model:

```text
ready
  -- action/goal accepted --> running
running
  -- OK and vehicle stable --> holding
running
  -- OK and no hold needed --> ready
running
  -- recoverable ERR --> holding or recovery
running
  -- unrecoverable ERR --> fault
holding
  -- next command accepted --> running
holding
  -- release/land --> ready or landed
any non-fault state
  -- abort/land accepted --> recovery
recovery
  -- landed/safe --> landed or ready
recovery
  -- failed --> fault
```

D2 operational examples:

```text
$takeoff('m', n=3)
#12 ACK takeoff ctrl=servo_acquire
#12 RUN takeoff phase=acquire n=4 z=0.31
#12 OK takeoff lock=7 z=0.61 state=holding

$follow('cat')
#13 ACK follow ctrl=explore_follow
#13 RUN follow target=cat age=120ms err=9.2
$abort()
#14 ACK abort target=#13 ctrl=recovery
#13 ERR follow reason=aborted return=recovery
#14 OK abort state=holding

$status()
#15 STAT op=holding active=0 mob=hovering loc=flow_scaled cam=1 flow=1 world=ok
```

D2 design rule:

```text
No public long-running command may hide inside a non-cooperative REPL call.
```

Progress update policy:

- `ACK` is sent immediately after acceptance.
- `OK` or `ERR` is always sent when the command terminates.
- `RUN` updates are allowed but should be sparse and meaningful.
- Do not stream high-rate telemetry as radio replies.
- Prefer `RUN` on:
  - phase change;
  - important milestone;
  - warning/degradation;
  - long-duration heartbeat at a low rate if needed.
- Detailed progress remains pull-based through `$status()` and `$active()`.

Recommended default:

```text
push: ACK, sparse RUN, terminal OK/ERR
pull: $status(), $active(), controller-specific query, all sequence-correlated
logs: sentai.fr for full forensic detail
```

Allowed implementations:

- C++ async controller with `start/status/cancel/result`;
- MP cooperative skill registered with an active record and called by an
  executive tick;
- short query or bounded admin/debug helper.

Disallowed for operational commands:

- MP `while True` that never yields/pumps pending work and cannot observe
  active/cancel/abort state;
- C binding that blocks for long periods without pending/abort checks;
- command that starts motion but does not expose `status` and `abort/cancel`;
- command that can fail without a terminal `OK/ERR` event.

D2 justification:

- matches ROS Actions semantics without importing ROS;
- matches F Prime/cFS command dispatch and status discipline;
- matches rover/Voyager delayed-result operations;
- preserves radio responsiveness;
- makes host-side LLM reasoning reliable because every intent has an observable
  result or rejection.

## Deliverable 3 - Command Dictionary Schema

D3 goal:

Define where the knowledge about available commands/primitives lives, without
forcing the board to carry a heavy schema system.

The command dictionary is primarily a **host-side knowledge artifact**.  The
host should know what is loaded on the board, what each primitive does, how to
call it over REPL/radio, what result to expect, and how to sequence primitives
in a sensible order.

The board should remain simple:

- expose callable primitives through `sentai.prime`, `sentai.explore`, and
  explicit `sentai.*` namespaces;
- execute the primitive or return `ERR reason=...`;
- emit compact REPL/radio replies and detailed `sentai.fr` logs;
- answer `$status()` / `$active()` / `$what_loaded()` style queries as needed.

The useful split is:

```text
host-side command/primitive catalog
  -> knows what exists on the current board image / FS
  -> gives the LLM enough context to plan
  -> compact REPL spelling
  -> board primitive implementation validates and runs
  -> ACK/RUN/OK/ERR + sentai.fr evidence
```

D3 conclusion:

- Do **not** implement a formal board-side command schema in B6.
- Keep board-side primitives as normal C++/MP functions that validate their own
  inputs and runtime conditions.
- Keep the rich primitive catalog host-side, derived from the board image, the
  files copied to the board FS, docs, and simulator-validated skills.
- Let the LLM Command Agent inspect that host-side catalog before planning.
- If the host sends a command that the board cannot execute, the board replies
  with compact `ERR reason=...`.
- If needed, add a small board query such as `$what_loaded()` or
  `$sentai.prime.list()` later so the host can verify the current board surface.

This gives the desired operational model:

```text
Coding Agent validates/promotes primitives in SIM
  -> board image / board FS contains those primitive implementations
  -> host catalog mirrors what is loaded on that board
  -> Command Agent reads the catalog and plans an ordered sequence
  -> radio sends compact REPL calls
  -> board runs or rejects each call
```

The host-side catalog should document, at minimum:

- public spelling: `$takeoff('m', n=3)`;
- namespace/function: `sentai.prime.takeoff`;
- short description and examples;
- expected blocking/async/cooperative behavior;
- what state/result the LLM should expect after success;
- common failure reasons;
- whether the primitive is core C++/MP or loaded from board FS;
- which simulator/mission artifact validated it.

Radio spelling should stay close to REPL/Python:

```python
$takeoff('m', n=3)
$hover('red_teddy_bear')
$status()
$q(17).abort()
```

Deep or diagnostic calls may still use explicit namespaces:

```python
$sentai.calib.status_tuple()
$sentai.flow.enable()
```

D3 implementation recommendation for the first slice:

1. Keep D3 host-side for now.
2. Document the currently loaded board primitives and their REPL spellings.
3. Let board-side functions own their own validation, timeout handling, and
   logging.
4. Add only a lightweight `$what_loaded()` / `$sentai.prime.list()` query later
   if the host needs runtime verification.
5. Avoid a generic onboard schema engine.

## Deliverable 4 - WVMC State / Minimal Executive Model

D4 defines the small board-side state layer that makes D2 observable and
interruptible.  It keeps the useful Astrobee-style executive idea, but does not
become a planner, a large gate, or a duplicate of each primitive's validation.

Conceptual frame: **WVMC - World / View / Model / Controller**.

The order matters for SentAI: the real world is observed through sensor views;
those views update the board-side model; controllers observe that model and
command action.

| WVMC term | SentAI meaning |
| --- | --- |
| World | The real physical/simulated environment. |
| View | Sensor/perception pipelines: camera, markers, flow, IMU/estimator, EdgeTPU detections. |
| Model | Board-side estimated world/state: objects, lifted tracks, places/H3 cells, landmarks, localization quality, active command state. |
| Controller | Board-side primitives/controllers: C++ controllers and cooperative MP skills. |
| Agent/operator | Host-side LLM/operator that queries the model and sends compact radio commands. |

D4 terminology:

| Term | Meaning |
| --- | --- |
| `active_primitive` | The public command/primitive currently active, e.g. `takeoff('m')` or `follow('cat')`. |
| `controller` | The board-side implementation executing it: C++ controller or MP skill. |
| `minimal_executive` | Tiny runtime coordinator that tracks active state and routes status/cancel/abort/recovery. |

The host catalog from D3 tells the LLM what can be called and how to plan.
Primitive implementations remain authoritative for detailed validation.  The
minimal executive only answers:

- what is running now?
- who controls it?
- how do `$status()` and `$active()` describe it?
- where do `$cancel()`, `$abort()`, and `$land()` go?
- what state do we return to after OK/ERR/cancel?

Minimal shared state:

| State | Examples |
| --- | --- |
| operating | boot, ready, running, holding, recovery, fault |
| mobility | grounded, armed, taking_off, hovering, landing, landed |
| active controller | none, servo, calib, explore, landing, recovery |
| evidence | camera_ready, marker_lock, object_ready, flow_ready, localization quality |
| link | usb, radio, stale, lost |

Minimal active record:

| Field | Purpose |
| --- | --- |
| `seq` | Correlates replies with the host command. |
| `active_primitive` | Public primitive spelling/name. |
| `controller` | C++ controller or cooperative MP skill. |
| `state` | accepted, running, stopping, recovering, done, error. |
| `phase` | Short controller-defined phase for `$active()` / sparse `RUN`. |
| `can_cancel` / `can_abort` | Whether interruption is available. |
| `return_state` | ready, holding, landed, recovery, or fault. |
| `last_reason` | Last error/degradation reason. |

Recommended flow:

```text
radio command
  -> REPL/function lookup
  -> primitive validates or returns ERR reason=...
  -> minimal executive creates active record
  -> controller runs
  -> OK/ERR/cancel/abort updates state
```

D4 conclusion:

Keep a **minimal executive** for active primitive tracking, status, cancel,
abort, and recovery.  Do not centralize every precondition.  The host catalog
plans; the primitive validates; the minimal executive coordinates what is
currently active.

SOTA anchor:

- **Astrobee executive** supports the minimal-executive idea: command
  acceptance and execution are state-aware, but the executive is not the whole
  autonomy stack.
- **ROS Actions** supports the active-record shape: long-running work has goal,
  feedback/status, result, and cancel semantics.
- **F Prime/cFS** supports explicit component/controller ownership, events, and
  telemetry/status separation without requiring the full framework onboard.
- **Mars rover/Voyager operations** support the host/board split: host-side
  planning and validation, board-side bounded execution, status, and safing.

So D4 keeps the SOTA concept that matters here: a small, observable,
interruptible executive layer.  It rejects the heavier SOTA machinery that
would duplicate host planning or primitive-specific validation onboard.

## Deliverable 5 - Sequence-ID / Reply Format

D5 defines the compact radio/REPL reply discipline that lets the host-side
Command Agent reason about D2 lifecycle and D4 active state without receiving
the rich host catalog or JSON/MCP payloads over radio.

Principles:

- every public command, including quick queries, should have a host-visible
  sequence ID;
- replies should name the active primitive and, when useful, the controller;
- `RUN` is sparse progress, not telemetry streaming;
- detailed evidence stays in `sentai.fr` and host-side artifacts;
- field names should be stable, short, and easy to parse.

Recommended radio reply kinds:

| Kind | Meaning |
| --- | --- |
| `CMD` | Optional host-side notation for command sent, not required onboard. |
| `ACK` | Command accepted; primitive/controller started or query accepted. |
| `RUN` | Sparse progress/status update for an active primitive. |
| `OK` | Terminal success. |
| `ERR` | Immediate or terminal failure. |
| `EVT` | Asynchronous event. |
| `STAT` | Compact status snapshot. |
| `ACTIVE` | Compact active primitive/controller snapshot. |

Recommended text examples:

```text
$takeoff('m', n=3)
#17 ACK takeoff ctrl=servo_acquire
#17 RUN takeoff phase=acquire n=5 z=0.42
#17 OK takeoff lock=7 z=0.61 state=holding

$follow('cat')
#18 ACK follow ctrl=explore_follow
#18 RUN follow target=cat age=120ms err=9.2
$abort()
#19 ACK abort target=#18 ctrl=recovery
#18 ERR follow reason=aborted return=recovery
#19 OK abort state=holding

$status()
#20 STAT op=holding active=0 mob=hovering loc=flow_scaled cam=1 flow=1 world=ok

$active()
#21 ACTIVE n=1 #18:follow ctrl=explore_follow state=running phase=track cancel=1 abort=1
```

Constraints:

- keep single-radio-packet replies when possible;
- prefer stable short field names;
- sequence IDs are recommended for every public command, including queries;
- `RUN` replies are sparse operational updates, not high-rate telemetry;
- host may reconstruct rich summaries from `sentai.fr`;
- do not send host-side primitive catalog entries over radio.

Recommended field conventions:

| Field | Meaning |
| --- | --- |
| `#17` | Sequence ID assigned by host or board; must correlate replies. |
| `ctrl` | Board-side controller executing the active primitive. |
| `phase` | Short controller-defined phase. |
| `state` / `op` | Minimal executive state such as `running`, `holding`, `recovery`. |
| `reason` | Stable compact failure/degradation reason. |
| `return` | State/recovery path entered after cancel/abort/error. |
| `active` | Number of active primitives/controllers. |
| `loc` | Localization quality: `marker_lock`, `flow_scaled`, `unknown`, etc. |

D5 conclusion:

Use a compact, sequence-correlated text protocol: `ACK/RUN/OK/ERR` for command
lifecycle, `STAT/ACTIVE` for pull status, and `sentai.fr` for forensic detail.
This gives the host enough structure for reasoning while keeping radio traffic
small and board-side implementation simple.

SOTA anchor:

- **CCSDS** supports the packet-discipline idea: every reply has a kind,
  correlation identity, and compact payload, even if SentAI stays with readable
  text instead of full CCSDS packets.
- **ROS Actions** supports the lifecycle shape: accepted goal, sparse feedback,
  terminal result, and cancellation.
- **F Prime/cFS** supports separating command replies, events, telemetry/status,
  parameters, and data products.
- **Mars rover/Voyager operations** support sequence-correlated, low-bandwidth
  command/result discipline where the host reconstructs context from sparse
  replies and richer logs.

So D5 keeps the SOTA concept that matters here: disciplined command/result
correlation under a narrow link.  It rejects verbose middleware payloads and
high-rate telemetry over the radio reply path.

## Deliverable 6 - Board / Host / MCP Split

D6 defines the boundary implied by D1-D5.  The board executes compact,
validated intentions.  The host owns rich reasoning, catalogs, artifacts, and
tool schemas.

Recommended split:

| Layer | Owns |
| --- | --- |
| board-side C++ | Fast perception/control loops, PrepTask consumers, safety-critical streams, watchdogs, status tuples, async controllers. |
| board-side MP | REPL/radio dispatcher, `sentai.prime`, cooperative `sentai.explore` skills, minimal executive view, command composition. |
| board-side model | Current objects/lifted tracks/places/landmarks/localization quality and active primitive/controller state. |
| host-side catalog | The rich primitive dictionary: spelling, args, examples, expected state/result, common failures, validation artifacts. |
| host-side Coding Agent | Writes/tests skills in simulator FS, studies artifacts, promotes accepted board-side primitives/skills. |
| host-side Command Agent | Reads the catalog, sends compact intention commands, reasons from `ACK/RUN/OK/ERR`, `STAT/ACTIVE`, and artifacts. |
| host-side MCP/tools | Rich schemas, logs, plots, artifact analysis, simulator runners, catalog export. |

B6 recommendation:

- MCP and rich schemas never go over the normal radio path.
- The host translates rich tool calls into compact `$...` REPL/radio commands.
- Every public command, including queries, should be sequence-correlated.
- The board answers compactly from local state/model and logs detail to
  `sentai.fr`; model-query names/fields remain experimental until simulator
  missions show what host-side reasoning actually needs.
- C++ owns fast loops; MP owns cooperative composition only.
- New skills are developed in SIM first, then promoted to the physical board FS
  and host-side catalog.

Operational flow:

```text
host MCP/tool/schema call
  -> host catalog lookup and planning
  -> compact $command(...)
  -> board primitive/controller
  -> #seq ACK/RUN/OK/ERR or STAT/ACTIVE
  -> host artifact/log analysis
```

D6 conclusion:

Keep **MCP as the host-side affordance layer**, not the onboard protocol.
Keep **REPL/radio as a compact command/result layer**.  Keep **board-side state
and execution small, observable, and interruptible**.  This preserves the SOTA
split from F Prime/cFS/Mars rover operations while fitting SentAI's narrow radio
and MicroPython/C++ runtime.

## Deliverable 7 - Board Model Query Taxonomy

Status: speculative draft / wishful thinking.

D7 defines how the host reads the board-side world/perception model.  This is
separate from D6's board/host/MCP split and separate from D5's command reply
lifecycle.

Important caveat: D7 is **not yet an implementation contract**.  It is a
working sketch for how board-model queries might look.  Do not treat these
names, fields, or reply shapes as stable until we run several simulator
missions, inspect what the board model actually contains, and learn what the
host/LLM really needs to ask.

Board-model communication convention:

- `STAT` reports posture/health/executive state, not the whole world model.
- `ACTIVE` reports running primitives/controllers.
- Model queries are explicit pull requests, for example `$what()`,
  `$objects()`, `$places()`, `$near('cat')`, `$model()`, or domain-specific
  `sentai.*` queries.
- Replies should be compact summaries with stable IDs, not JSON dumps.
- Every model item should carry enough evidence for host reasoning:
  `id`, `cls/type`, `conf`, `age`, `frame`, and a small pose/region field when
  available.
- If the model is too large, return counts plus top-N items and let the host
  ask follow-up queries by ID.
- `sentai.fr` remains the forensic source for full detections, tracks,
  timestamps, and plots.

Example model replies:

```text
$what()
#31 OK what n=3 top=o4:cat:0.81,o7:red_teddy:0.74,m:marker:1.00 age=80ms

$objects(cls='cat')
#32 OK objects n=1 o4 cls=cat conf=0.81 age=95ms frame=cam bbox=104,66,42,39

$near('cat')
#33 OK near ref=o4 n=2 o7:red_teddy:0.74,o9:cup:0.62 frame=local age=140ms

$model()
#34 OK model objs=5 places=2 loc=flow_scaled marker=0 age=120ms rev=2841
```

Recommended model-field conventions:

| Field | Meaning |
| --- | --- |
| `id` / `o4` | Stable short object/place/track ID for follow-up commands. |
| `cls` / `type` | Object class or model item type. |
| `conf` | Confidence, usually 0..1 or compact percent. |
| `age` | Time since the evidence/model item was last updated. |
| `frame` | Coordinate/image frame: `cam`, `img`, `local`, `marker`, `world`. |
| `bbox` | Compact image region when pose is not available. |
| `pose` / `xy` / `z` | Optional compact metric estimate if valid. |
| `loc` | Localization quality used to interpret model geometry. |
| `rev` | Monotonic board-model revision for host cache invalidation. |

Board-model rule:

```text
query returns current compact snapshot
  -> host decides whether to ask more
  -> host sends next intent by object/place ID
  -> board validates that ID is still fresh enough
```

Progressive query principle:

The host question "what do you know?" can be much larger than one radio reply.
D7 should not invent a complete world-model DSL before we experiment.  The
first convention should be progressive:

```text
$model()                -> compact overview: counts, localization, revision
$objects()              -> top-N / filtered object summary
$places()               -> top-N / filtered place or map-cell summary
$detail('o4')           -> one object/place/track by stable ID
$sentai.slam.*          -> explicit low-level diagnostic query when needed
```

Because the current board already has richer substrates such as H3/place cells,
object tracks, lifted object positions, and SLAM landmarks, the host should
first ask for a compact index/summary and then drill down by ID.  This keeps
the radio protocol small while still allowing deep inspection through existing
or future low-level primitives.

Future idea, not B6 implementation:

- define a tiny introspection convention like `$model(index=1)` /
  `$detail('h23')` after we have real model data;
- allow host-side MCP tools to expose rich schemas for those details;
- keep the board reply compact and ID-based;
- avoid freezing a custom DSL until simulator experiments show what the host
  actually needs to ask.

D7 conclusion:

Communicate the board-side model through explicit compact pull queries with
stable IDs, confidence, age, frame, and revision, while keeping full evidence in
`sentai.fr`.  The host should cache model snapshots by `rev`, ask follow-up
queries by ID or explicit `sentai.*` low-level queries, and expect the board to
reject stale object/place IDs before motion commands use them.  Do not freeze a
world-model DSL in B6; treat D7 as the first compact query convention to test in
simulation.  Until those simulations exist, D7 remains a design hypothesis, not
a requirement.

SOTA anchor:

- **Mars rover operations** support the progressive-query idea: ground tools and
  operators reason from compact state/evidence products, then inspect richer
  artifacts or command products when needed.
- **F Prime/cFS** support separating status/telemetry/data products from
  commands.  D7 should behave like a compact status/data-product query surface,
  not a command DSL.
- **CCSDS packet discipline** supports returning small correlated summaries
  over narrow links and keeping larger data products out of the normal command
  response.
- **MCP** supports rich host-side schemas and resources for model details, but
  that richness should stay host-side and be translated into compact board
  queries.
- **SLAM/robot world-model practice** supports stable landmarks/object IDs and
  freshness/confidence metadata, but the exact SentAI representation must come
  from simulator evidence.

So D7 keeps the SOTA concept that matters here: progressive, evidence-based
model inspection over a narrow link.  It deliberately rejects a premature
onboard world-model DSL before real SentAI missions show what the model contains
and what the host actually needs.

## Deliverable 8 - Gap Analysis Against Current `sentai.*`

D8 checks whether D1-D7 cover the concepts that already exist in the current
`sentai.*` runtime.  It is not an implementation checklist.  Its job is to
catch missing vocabulary before B6 is accepted as research.

Observed runtime concepts already present in source:

| Existing concept | Current namespace | Gap against D1-D7 |
| --- | --- | --- |
| Camera / perception producer | `sentai.pipeline`, `sentai.camera`, PrepTask | D6 should treat this as the board-side View producer in WVMC.  Avoid duplicate preprocessing in consumers. |
| Object memory | `sentai.objects` | D7 mentions objects, but should recognize this existing object table as one board-model source. |
| Object-to-world lifting | `sentai.object_lifter` | D7 should account for lifted object IDs / tracklets as motion-command references. |
| Hex / place memory | `sentai.places` | The "hexagon world" is not only speculative: H3/place support already exists and should be named as an existing substrate. |
| SLAM / landmarks / pose | `sentai.slam` | D7 should distinguish low-level SLAM diagnostics from the compact model-query surface. |
| Low-level flight control | `sentai.servo`, `sentai.crazy` | D1-D4 cover this, but should keep the boundary clear: `prime` wraps intentions; `servo/crazy` remain runtime/control surfaces. |
| Existing mission FSM precedent | `sentai.explore` | D4 should treat this as a precedent for board-side executive/state-machine work, not necessarily the final generic executive. |
| Evidence/logging | `sentai.fr` | D5/D7 already align: compact replies over radio, forensic detail in logs. |
| Safety/fault handling | `sentai.safety`, `sentai.diag`, `sentai.sys` | D4/D5 should keep explicit safe fallback, abort, and health/status concepts. |

Missing or weakly defined concepts:

- no `sentai.prime` namespace exists yet;
- no generic active primitive/controller registry exists yet;
- no unified compact board-model facade exists yet;
- no shared conventions for model `id`, `age`, `frame`, `conf`, and `rev`
  across `objects`, `places`, `slam`, and `object_lifter`;
- no common ACK/RUN/OK/ERR reply layer exists yet;
- no host-visible primitive catalog exists yet;
- simulator and ARM namespace parity should be checked before D7 model-query
  experiments, especially around `sentai.slam`;
- communication paths exist beyond the narrow REPL/radio path (`link`, `mesh`,
  `uart`, `usb`), but B6 should keep the first command contract radio/REPL
  focused.

D8 conclusion:

D1-D7 cover the main architecture, but B6 should explicitly acknowledge the
existing board-model substrates: `sentai.pipeline` as View producer,
`sentai.objects` / `sentai.object_lifter` / `sentai.places` / `sentai.slam` as
Model sources, and `sentai.servo` / `sentai.crazy` / `sentai.explore` as
Controller or executive precedents.  The most important missed concept is the
already-existing H3/hex place layer in `sentai.places`.  D7 remains speculative
as a query grammar, but the underlying map/place substrate is real.

## Answered Questions

The following questions are answered well enough for B6 research:

| Question | Current answer |
| --- | --- |
| Should B6 implement code now? | No.  B6 is research/evaluation.  Implementation becomes a later task. |
| Where does reasoning live? | Host-side LLM/agent.  Board-side executes compact intentions and answers queries from local state/world model. |
| Are commands low-level controls? | No.  They are intentions: takeoff/stabilize, what do you see, find/hover/follow/investigate/report. |
| Does board-side have memory/state? | Yes.  Board-side maintains runtime state and a local world/perception model through existing substrates such as `sentai.objects`, `sentai.object_lifter`, `sentai.places`, and `sentai.slam`. |
| How does the host read the board model? | Through explicit compact pull queries such as `$what()`, `$objects()`, `$near(...)`, `$places()`, and `$model()`, with stable IDs, confidence, age, frame, and optional compact pose/region fields. |
| Can high-level explore skills live board-side? | Yes, likely in MP/Python as cooperative skills that compose `sentai.prime` and query the world model. |
| Can MP run long-running skills? | Yes, cooperatively.  No non-yielding REPL trap; skills must reach pending/executive checks and be status/cancel/abort aware, or delegate to async C++ owners. |
| Can REPL break running code? | Ctrl-C can raise `KeyboardInterrupt` for Python that checks pending exceptions, but normal operational abort/cancel must be explicit commands. |
| Where do fast loops live? | C++/FreeRTOS controllers, not MP. |
| Do we need onboard mini-sequences in v1? | Probably no.  Host-side sequencing first; onboard mini-sequences only after patterns are stable and approved. |
| Can parameters change inflight? | Some can.  Parameters need mutability classes: init-only, pre-arm, inflight tunable, command-local. |
| How does the physical board get new skills? | LLM Coding Agent writes/tests in SIM FS; accepted skill files are copied/promoted to physical board FS. |
| Does the live Command Agent write code? | No in normal operation.  It uses the accepted taxonomy through REPL/radio. |

## Remaining Open Questions

These are still real design questions or follow-up items before implementation:

| Question | Why it remains open |
| --- | --- |
| What exact fields belong in `$status()` and `$active()`? | Not clear yet.  Must be small enough for radio but rich enough for host-side reasoning. |
| What exact fields belong in board-model queries? | Not clear yet.  Need experience with object/place IDs, coordinate frames, confidence, age/freshness, and top-N truncation. |
| Do model queries use a compact facade or direct source namespaces? | Existing sources are `objects`, `object_lifter`, `places`, and `slam`; we still need simulator evidence before deciding whether `$model()` hides them or simply summarizes them. |
| How should H3/place memory be exposed compactly? | Premature to standardize.  The likely shape is ID/pointer-based: ask by place/cell/object ID, then drill down into associations such as objects detected in H3 cell `4434` with class `cat`. |
| How many radio event severities/kinds are needed? | Need enough observability without verbose protocol creep. |
| What is the exact fallback ladder? | Keep as a concrete todo: define sensor requirements for `hold -> return-home -> land -> motor stop`, including what happens without marker/flow/world lock. |
| How does host-side compatibility learning become tool metadata? | Simulator results should inform the LLM/MCP layer, but this is post-thesis / later-tooling work unless it becomes necessary sooner. |
| How do we handle incompatible commands? | Do not build a huge mode matrix.  The board returns explicit `ERR reason=...`; the host interprets and replans from status/artifacts. |
| What minimal onboard sequence executor is worth adding after v1? | Not needed now, but likely useful for common safing/mission patterns. |
| Which events/status fields are mandatory after each intention command? | Needed so the host can reason reliably after `OK/ERR`, especially for perception commands. |
| When does the board reject an object/place ID as stale? | Keep as a concrete todo so commands like `$hover('o4')` do not chase old detections. |
| What SIM/ARM namespace parity is required for B6/B7 experiments? | Concrete todo: reach parity for the namespaces needed by experiments, especially around `sentai.slam` and model sources. |

Deferred, not blocking the next implementation slice:

- exact host-side primitive catalog format; this can be discovered from source,
  simulator artifacts, and later host/MCP tooling after the thesis work;
- full standardization of board-model query grammar; B6 only records the
  direction and the existing substrates.

## Marker-Loss Fallback Sensing

Open safety question: what should SentAI trust when marker lock is lost?

Important observation: optical flow by itself is usually not enough to recover
metric Z.  Flow gives image displacement / angular motion.  To turn that into
metric lateral velocity or position, the estimator needs scale, usually from
altitude/range, known ground geometry, markers, barometer/range sensor, or a
strong ground-plane assumption.  Monocular optical flow can sometimes infer
time-to-contact/divergence, but that is not the same as a stable metric Z
source for landing or return-home.

Candidate fallback evidence ladder:

| Evidence | Useful for | Limits |
| --- | --- | --- |
| full marker constellation | best local pose/Z/centroid reference | can leave FOV during drift/low altitude |
| partial markers | coarse centroid / visual reacquire cue | ambiguous if too few markers; should be confidence-gated |
| optical flow + existing altitude estimate | lateral stabilization / drift reduction | needs scale; degrades over low texture or stale Z |
| barometer / estimator Z | short-term altitude continuity | drift/noise; may not align with marker/world Z |
| known ground objects / semantic anchors | reacquire reference, navigation intent, local map updates | object detections are not metric anchors unless size/pose/ground contact is known |
| object landmarks with known size or mapped position | possible fallback localization | requires prior map/calibration and confidence model |
| no reliable visual/localization evidence | safe hold briefly, then land | return-home may be unsafe without localization |

Design implication for B6/B7:

- `marker_loss` should not have one global behavior.
- The executive should classify localization quality:
  `marker_lock`, `partial_marker`, `flow_scaled`, `flow_unscaled`,
  `semantic_anchor`, `dead_reckoning`, `unknown`.
- Recovery actions should declare required evidence:
  - `hold` may accept flow + recent Z for a short timeout;
  - `return_home` likely requires a map/localization anchor;
  - `land` may require only safe descent evidence and obstacle assumptions;
  - `motor_stop` is last-resort fault behavior.
- If object anchors are used, they should become explicit board-side world-model
  entities with confidence, age, source, and whether they are metric anchors or
  only semantic detections.

This should become part of the fallback ladder research: flow can reduce drift,
but it should not be treated as a full replacement for marker-derived Z unless
another source provides scale.
