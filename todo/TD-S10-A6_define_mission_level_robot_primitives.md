# TD-S10-A6 - Define SentAI Primitive Taxonomies

## Goal

Define a compact taxonomy for SentAI primitives so future missions can be
written as readable REPL/radio commands while camera-rate perception,
estimator-feeding, and control loops stay in C++ tasks.

A6 is a design/research task before implementation.  It should produce:

- a low-level runtime taxonomy for what already exists in `sentai.*`;
- a flight/action taxonomy for simple reusable robot actions such as takeoff,
  land, hold, hover, descend, abort;
- a high-level SAR/inspection taxonomy under a dedicated `sentai.explore`
  namespace;
- a compact REPL-over-radio grammar;
- a host-side primitive catalog direction, without freezing the exact catalog
  format before source/simulator evidence shows what is loaded on the board;
- an experimental board-model query direction for asking "what do you know?"
  through compact, progressive, ID-based queries after simulator validation;
- a minimal board-side executive/state model for active primitive tracking,
  status, cancel, abort, and recovery;
- a cleanup plan for duplicate logic in `sentai.calib`, `sentai.markers`,
  `sentai.servo`, and `sentai.flow`;
- a PrepTask-centered frame/data ownership contract.

Important boundary: SentAI does not replace the autopilot.  Crazyflie/PX4-style
firmware owns attitude stabilization, mixing, EKF internals, motor safety, and
low-level flight-control loops.  SentAI owns the layer above that: perception,
mission intent, evidence, bounded visual-servo tasks, and orchestration.

## Taxonomy Layers

A6 should separate three layers instead of calling everything a primitive.

| Layer | Namespace | Purpose | Examples |
| --- | --- | --- | --- |
| Runtime primitives | `sentai.*` | Low-level device/runtime APIs and C++ task surfaces. | `sentai.camera`, `sentai.tpu`, `sentai.markers`, `sentai.flow`, `sentai.calib`, `sentai.servo`, `sentai.crazy`, `sentai.fr` |
| Flight/action primitives | `sentai.prime` | Short public actions/maintainers backed by C++ or local Python wrappers. | `$takeoff('m')`, `$hold(z=0.6)`, `$hover('m')`, `$descend('slow')`, `$abort()` |
| Mission/SAR primitives | `sentai.explore` | High-level mission vocabulary inspired by IAMSAR/USCG SAR and inspection workflows. | `$sentai.explore.scan('area1', p='parallel')`, `$sentai.explore.investigate('o4')`, `$sentai.explore.overwatch('o4')` |

The design order should be:

1. Inventory low-level runtime primitives that already exist.
2. Factor simple flight/action primitives into `sentai.prime`.
3. Build `sentai.explore` high-level mission primitives from `sentai.prime` and
   runtime services.
4. Only after simulator missions produce useful board-model data, decide
   whether compact model queries such as `$model()`, `$objects()`, and
   `$detail(id)` deserve stable names/fields.

## Low-Level Runtime Taxonomy

This layer describes what SentAI can already sense, compute, log, and command.
These APIs may be called directly for diagnostics, but they are not the normal
LLM/operator mission language.

| Family | Current modules | A6 question |
| --- | --- | --- |
| Camera/perception producer | `sentai.pipeline`, `sentai.camera`, PrepTask, SIM camera bridge | Ensure preprocessing happens once and consumers receive snapshots/pointers. |
| Edge inference | `sentai.tpu`, `sentai.tfl`, future perception tasks | Decide what becomes cached perception versus one-shot calls. |
| Board model substrates | `sentai.objects`, `sentai.object_lifter`, `sentai.places`, `sentai.slam` | Treat objects, lifted tracks, H3/places, and SLAM/landmarks as existing model sources before inventing a new facade. |
| Markers/layout/pose | `sentai.markers`, `sentai.calib` | Standardize marker observation snapshots, visual-Z, marker windows, pose validity. |
| Optical flow | `sentai.flow` | Make flow a shared producer/cache, not a private loop in each mission. |
| Calibration | `sentai.calib` | Keep calibration-specific tasks; move generic helpers to shared owners. |
| Servo/control tasks | `sentai.servo` | Split B4-specific FSM from reusable acquire/hold/handoff/motion/landing primitives. |
| Drone transport | `sentai.crazy`, `sentai.link` | Keep transport-only: RPYT, Generic Hover, ExtPos, flow packets, arm/disarm/release. |
| Evidence/logging | `sentai.fr` | Evidence only; host reconstructs summaries/plots. |
| Safety | `sentai.safety`, controller arbitration | Abort, stale camera, controller conflict, safe stop/land. |

PrepTask contract:

- camera/SIM bridge is the frame producer;
- marker, flow, object, and servo consumers read shared snapshots;
- control paths should not do repeated full-frame `memcpy`;
- recorder copies are allowed for evidence but not as control dependencies;
- SIM and ARM should follow the same producer/consumer contract.

## Flight / Action Taxonomy

`sentai.prime` is the short public namespace for simple robot actions and
maintainers.  It maps short REPL/radio calls to typed C++ task starters or
small Python wrappers.

Candidate core actions:

| Primitive | Kind | Notes |
| --- | --- | --- |
| `$status()` | query | Compact current state. |
| `$modes()` | query | Internal active guidance/controller status, for diagnostics. |
| `$active()` | query | List currently active primitives/controllers and their command IDs. |
| `$log(on=1)` | maintainer | Start/stop evidence logging. |
| `$takeoff('m', n=3)` | action | Take off until marker evidence exists. |
| `$hold(z=0.6)` | maintainer | Hold selected constraint such as altitude. |
| `$hover('m')` | goal/maintainer | Keep marker/target centered or hover over reference. |
| `$goto('o4')` | goal | Navigate toward a visual/object reference. |
| `$find('cup', t=5)` | action/query | Perception search without necessarily owning motion. |
| `$track('o4')` | maintainer | Keep a moving target in view. |
| `$descend('slow')` | action/recovery | Controlled descent. |
| `$land()` | recovery/action | Land using safest available controller. |
| `$abort()` | preempt/recovery | Stop current controller and enter safe recovery. |

The public grammar should stay mission-readable; internal implementation may
use guidance channels, primitive controllers, and C++ tasks.

## High-Level Mission Taxonomy

Use IAMSAR/USCG SAR terminology as the main reference for high-level mission
verbs.  These should live under `sentai.explore` or a closely related mission
namespace, not mixed into the low-level runtime modules.

IAMSAR/USCG-inspired terms to evaluate:

| Term | SentAI meaning |
| --- | --- |
| datum | Reference point/line/area such as marker, last-seen target, route, or region. |
| commence search point | Where a search pattern begins. |
| search area | Region to cover. |
| track spacing / sweep width | Search geometry tied to camera FOV, altitude, and detection probability. |
| coverage | Host-side measure of searched area/evidence. |
| parallel track search | Systematic area coverage. |
| creeping line search | Search biased along a track/drift direction. |
| expanding square search | Search around a likely last-known point. |
| sector search | Search around a small high-probability area. |
| track-line search | Search along a route. |
| sighting/contact | Candidate detection requiring investigation. |
| investigate | Move sensors/robot to gather evidence about a contact. |
| loiter/overwatch | Stay near a reference and maintain observation. |

Candidate `sentai.explore` calls:

```python
$sentai.explore.scan('area1', p='parallel')
$sentai.explore.scan('last_seen', p='square')
$sentai.explore.scan('route:r1', p='track')
$sentai.explore.investigate('o4')
$sentai.explore.track('o4')
$sentai.explore.overwatch('o4')
$sentai.explore.mark('o4')
$sentai.explore.report('o4')
$sentai.explore.return_home()
```

High-level mission primitives should compose `sentai.prime` actions and
runtime snapshots.  For example, `sentai.explore.scan(...)` may use a search
pattern generator, marker/flow/perception snapshots, `$hold(z=...)`, and
`$goto(...)`, but the user-facing term remains `scan`, not `lateral_guidance`.

## Radio / REPL Grammar

Normal agent traffic should be REPL-shaped, not a separate BASIC-like language.

```python
$takeoff('m', n=3)
$find('cup', t=5)
$q(17).abort()
$sentai.calib.status_tuple()
```

Resolution:

```text
$takeoff('m', n=3)        -> sentai.prime.takeoff('m', n=3)
$q(17).abort()            -> sentai.prime.q(17).abort()
$sentai.explore.scan(...)  -> sentai.explore.scan(...)
$sentai.calib.status()    -> sentai.calib.status()
$exec print('debug')      -> trusted raw admin path
```

Rules:

- `$` is the radio/tunnel envelope and is stripped before dispatch;
- short `$name(...)` calls resolve through `sentai.prime`;
- `$sentai.*` calls are explicit deep namespace calls for trusted operators;
- `$exec ...` is admin/debug, not normal agent operation;
- `q(seq)` is optional for humans and recommended for agents so delayed replies
  can be correlated;
- commands should fit one small radio text packet when possible;
- no JSON is parsed onboard in the normal flight command path.

Reply shape:

```text
#17 ACK abort
#17 RUN land phase=descend
#17 OK abort landed=1
#14 ERR find timeout
```

## Execution Model

MicroPython should be treated as a single cooperative orchestration loop, not
as the place where frame-rate asynchronous flight code runs.  A6 should assume
that MP can dispatch commands, poll status, and compose tasks, but should not
depend on MP running multiple concurrent loops reliably.

Hard invariant: normal primitive calls must remain observable and
interruptible from the REPL/radio path.  The system must remain able to answer
`$status()`, `$modes()`, `$active()`, `$prime()`, update/query calls, and
high-priority `$abort()` / `$land()` while a takeoff, scan, hover, or descend
primitive is active.

`start()` must therefore not mean "run a non-yielding infinite loop inside
MicroPython".  It should mean "request a controller and start work in the
correct runtime controller", either as a C++ async task or as a cooperative MP skill.

Execution locations:

| Primitive type | Where it executes | MP/REPL responsibility |
| --- | --- | --- |
| query | Immediate MP or C++ getter. | Return compact value quickly. |
| C++ action/goal/maintainer | FreeRTOS/C++ task or existing C++ service controller. | Call `*_start()`, then use `status()` / `result()` / `abort()`. |
| Python wrapper | MP function that starts/composes C++ primitives. | Do not contain frame-rate loops; may sequence starts and read statuses. |
| Python mission/executive | Resident `/main.py` command loop or `sentai.prime` dispatcher. | Parse radio/REPL lines, track active primitive/controller, answer status/cancel/abort. |
| host/LLM action | Host process/MCP adapter. | Plan from the host-side primitive catalog, send compact commands, analyze artifacts. |

Lifecycle verbs should be separated by intent:

| Verb pair | Meaning | Examples |
| --- | --- | --- |
| `enable()` / `disable()` | Turn a background service/producer on or off.  It may keep running until disabled. | `$camera.enable()`, `$flow.enable()`, `$log(on=1)`, `$sentai.markers.enable()` |
| `start()` / `cancel()` | Begin a bounded action/goal that eventually returns `OK/ERR` or can be cancelled. | `$takeoff('m')`, `$goto('o4')`, `$scan('cup')` |
| `hold()` / `release()` | Maintain a selected constraint until released or preempted. | `$hold(z=0.6)`, `$hold(y=0)` |
| `status()` / `result()` | Inspect active or completed work without changing ownership. | `$status()`, `$sample.status()` |
| `abort()` / `land()` | High-priority recovery/preemption. | `$abort()`, `$land()` |

This resolves the naming ambiguity:

- camera feed, marker observation, flow feed, and logging are services:
  `enable()`/`disable()`;
- takeoff, goto, scan, investigate, centered descend are actions/goals:
  `start()`/`cancel()` semantics, even if the radio shortcut is
  `$takeoff('m')`;
- altitude/axis/target invariants are maintainers:
  `hold()`/`release()` semantics.

Long-running loops belong in C++ tasks when they:

- read camera/marker/flow snapshots at frame rate;
- stream RPYT, Generic Hover, ExtPos, or flow packets;
- own safety-critical command outputs;
- need watchdog/timeout behavior independent of REPL responsiveness.

Python primitives may be long-lived as registered modules, but their `start()`
should normally return quickly:

```python
def start(target='m', n=3, t=8):
    rc = sentai.servo.marker_acquire_start(target, n, t)
    return "ACK sample" if rc == 0 else "ERR rc=%d" % rc
```

Status polling can happen in three ways:

1. external agent sends `$sample.status()` / `$status()`;
2. resident MP command loop periodically polls active controllers and emits compact
   `RUN/OK/ERR` replies/events;
3. host reconstructs detailed progress from `sentai.fr`.

Recommended pattern: **ACK fast, finish asynchronously, timeout in the controller**.

For actions like takeoff:

1. `$takeoff('m', t=8)` validates its required runtime conditions and asks the
   minimal executive to record the active primitive/controller.
2. If it cannot start, return immediate `ERR`.
3. If it starts, return immediate `ACK` with a sequence / controller ID.
4. The C++ task tracks timeout and terminal status internally.
5. Completion is exposed as:
   - compact event/reply from the resident executive, if enabled;
   - and/or `result()` / `$status()` polling;
   - and detailed `sentai.fr` evidence for host reconstruction.

Do not use a blocking MP pattern as the default:

```python
while not sentai.servo.done():
    sentai.rtos.sleep_ms(20)
```

That is acceptable only for one-shot experiment scripts when no radio command
loop needs to stay responsive.  It is not the normal A6 primitive pattern
because it prevents radio-driven status, update, query, cancel, and abort from
being serviced promptly.

Preferred result contract:

```text
#13 ACK takeoff ctrl=servo_acquire
#13 RUN takeoff phase=acquire n=5 z=0.42
#13 OK takeoff lock=7 z=0.61
#13 ERR takeoff reason=marker_timeout
```

Open A6 design point: decide whether the first implementation uses only
external polling, or whether `/main.py` should include a small resident
minimal-executive loop that polls active controllers between REPL/radio
messages and emits terminal events.

Practical MP constraint: if a Python primitive really needs to do multi-step
work itself, it should expose a cooperative `tick()` called by the resident
executive, not block inside `start()`:

```python
def start(...):
    state["active"] = True
    return "ACK"

def tick():
    if not state["active"]:
        return None
    # one small non-blocking step, then return
    return status()
```

But A6 should prefer C++ tasks for anything safety-critical, camera-rate, or
packet-streaming.

## Loadable Python Primitives

Use the simple module-as-primitive convention.  Avoid required OOP unless a
future primitive really needs it.

```python
# /lib/prime/sample.py

META = {
    "name": "sample",
    "calls": ("start", "status", "abort"),
    "kind": "action",
    "controller": "sentai.servo",
}

def start(target='m', n=3, t=8):
    return sentai.servo.marker_acquire_start(target, n, t)

def status():
    return sentai.servo.marker_status()

def abort():
    return sentai.servo.marker_abort()

def unload():
    abort()
```

Load/use/unload:

```python
$load('sample.py')
$sample.start('m', n=3)
$sample.status()
$unload('sample')
```

Rules:

- load into an isolated module dict;
- validate `META`;
- register as `sentai.prime.<META["name"]>`;
- do not copy module globals into REPL globals;
- reject name collisions unless `replace=1`;
- `unload()` removes registry entries, calls cleanup if present, refuses to
  unload running controllers unless `force=1`, then runs `gc.collect()`.

## Arbitration Model

Do not freeze an axis-ownership model yet.  Treat it as a design question.

Aviation guidance terminology is useful internally because it separates active
lateral guidance, vertical guidance, selected constraints, managed targets, and
annunciated modes.  But public mission DSL should not sound like an autopilot
manual.

Candidate internal concepts:

| Internal concept | Meaning |
| --- | --- |
| query | Immediate call with no command ownership. |
| action | Finite task with success/fail. |
| goal | Runs until target reached, may release or transition to a hold. |
| maintainer | Holds an invariant until cancelled/replaced. |
| recovery | High-priority safe preemption. |
| guidance channel | Internal active controller/channel: vertical, lateral, yaw, target, estimator feed, perception, logging, safety. |
| selected constraint | User-selected constraint such as `hold(y=0)` or `hold(z=0.6)`. |
| managed guidance | A planner/target primitive such as `goto('cat')` or `scan(...)`. |

Examples to resolve in A6:

| Active | New | Question |
| --- | --- | --- |
| `$hold(z=0.6)` | `$hover('cat')` | Compatible if hover does not need vertical ownership. |
| `$hover('cat')` | `$hover('dog')` | Conflict unless multi-target behavior is explicit. |
| `$hold(y=0)` | `$goto('cat')` | Compatible if `goto` accepts selected-y constraint. |
| `$hold(x=0, y=0)` | `$scan('cup')` | Conflict if scan needs lateral motion; compatible if scan is perception-only. |

A6 deliverable: clear command rejection semantics and compact `$modes()` /
`$status()` report format.  Do not build a large compatibility matrix yet:
the host should learn likely-compatible command sequences from simulator runs,
and the board should return explicit `ERR reason=...` when the current state
cannot accept a command.

Active primitive introspection is required, not optional.  Public command:

```python
$active()
```

Explicit API:

```python
sentai.prime.get_active()
```

Expected compact output shape:

```text
ACTIVE n=2 #13:takeoff ctrl=servo phase=acquire #8:log ctrl=fr
```

Each active entry should include:

| Field | Meaning |
| --- | --- |
| `cmd_seq` | Optional radio/agent sequence ID. |
| `name` | Primitive name, e.g. `takeoff`, `hold`, `log`. |
| `kind` | query/action/goal/maintainer/recovery. |
| `controller` / `ctrl` | C++ or MP controller, e.g. `servo`, `fr`, `flow`. |
| `phase` | Compact phase/status string. |
| `started_ms` | Runtime tick when accepted. |
| `timeout_ms` | Deadline if bounded. |
| `stop` | Supported stop command: `cancel`, `release`, `disable`, `abort`. |

Stop/control commands should be explicit:

```python
$cancel('#13')       # cancel a bounded goal/action
$release('hold')    # release a maintainer
$disable('flow')    # disable a background service
$abort()            # high-priority all-motion recovery
```

This is public enough for a radio agent because it answers "what is running?"
and "what can I stop?", while keeping deeper controller internals inside
`sentai.prime` / `sentai.*`.

Keep this deliberately small.  Do not invent an onboard OS or a generic
lifecycle framework.  The active registry should be a fixed-size table of
currently owned primitives, updated only at clear ownership transitions:

- `start`/`enable` accepted -> insert active entry;
- terminal `OK`/`ERR` -> remove active entry or keep a tiny last-result slot;
- `cancel`/`release`/`disable`/`abort` accepted -> mark stopping, then remove on
  terminal result;
- crash/stale controller -> mark `ERR stale_controller` and remove during executive
  cleanup.

Non-goals for A6:

- no dynamic dependency graph;
- no generic task scheduler in MP;
- no arbitrary nested primitive tree;
- no heap-heavy per-primitive objects unless a Python primitive was explicitly
  loaded;
- no attempt to mirror all FreeRTOS tasks.

The active table exists only so radio/REPL can answer "what is running?" and
"how do I stop it?".

## Research References

Use these as design references, not implementation dependencies.

| Reference family | What to borrow |
| --- | --- |
| IAMSAR / USCG SAR manuals | High-level mission vocabulary: datum, search area, track spacing, sweep width, coverage, parallel/creeping/sector/expanding-square/track-line search, contact, investigate, recovery. |
| ROS 2 Actions | `start/status/result/cancel` semantics for long-running tasks. |
| BehaviorTree.CPP / Nav2 BTs | Actions, conditions, recovery, running/success/failure, halting. |
| MAVSDK / Crazyflie HL Commander | Distinguish discrete public actions from continuous setpoint streaming. |
| FAA/EASA flight guidance / helicopter automation | Internal mode annunciation and lateral/vertical guidance separation. |
| F Prime / CCSDS / Mars Pathfinder / Aerie SeqN | Command dictionary, sequence IDs, compact telemetry, command/result correlation. |
| MCP | Host-side rich tool schema/resources/prompts; translate to compact REPL-radio calls. |

Source links already investigated:

- ROS 2 Actions: <https://docs.ros.org/en/rolling/Concepts/Basic/About-Actions.html>
- BehaviorTree.CPP: <https://www.behaviortree.dev/docs/intro/>
- Nav2 behavior trees: <https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html>
- MAVSDK Action/Offboard: <https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html>, <https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html>
- Crazyflie High Level Commander: <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_hl_commander/>
- MCP architecture/spec: <https://modelcontextprotocol.io/docs/learn/architecture>, <https://modelcontextprotocol.io/specification/2024-11-05/index>
- Mars Pathfinder command dictionary: <https://pds.nasa.gov/data/mpfr-m-rvrcam-5-midr-v1.0/mprv_0001/document/roverdoc/cmddict/cmddict.htm>
- Aerie SeqN: <https://ammos.nasa.gov/aerie-docs/sequencing/seqn/>
- F Prime dictionary: <https://fprime.jpl.nasa.gov/latest/docs/reference/system-functional/dictionary/>
- CCSDS overview: <https://docs.ccsdspy.org/en/latest/user-guide/ccsds.html>
- NASA cFS overview: <https://etd.gsfc.nasa.gov/capabilities/capabilities-listing/cfs/>
- NASA cFS GitHub: <https://github.com/nasa/cFS>
- NASA Astrobee executive: <https://nasa.github.io/astrobee/v/develop/executive.html>

## SOTA Alignment Snapshot

A6 is close to existing robotics/space-operation practice, but with an embedded
radio/REPL constraint.

| A6 concept | Similar established concept | Alignment | Difference / refinement |
| --- | --- | --- | --- |
| `sentai.*`, `sentai.prime`, `sentai.explore` layers | Robotics task/skill/primitive taxonomies; TAMP split between high-level reasoning and low-level feasibility. | Strong.  Capability/skill surveys recommend separating task, skill, and primitive. | Keep names local and concrete.  `sentai.explore` is mission vocabulary; `sentai.prime` is execution vocabulary; `sentai.*` is runtime surface. |
| `$takeoff('m')` with ACK/RUN/OK/ERR | ROS 2 Actions goal/feedback/result/cancel. | Strong.  Use the action lifecycle semantics. | Do not copy ROS graph/DDS/action transport; radio gets compact REPL calls and terse replies. |
| Non-blocking MP and C++ task controllers | MAVSDK Offboard start/stop and setpoint streaming; PX4 flight modes/tasks. | Strong.  Public action starts a mode/task; lower layer streams setpoints. | SentAI remains above autopilot.  Camera-rate and packet-streaming loops stay C++/FreeRTOS. |
| `$active()` / host catalog / result correlation | F Prime dictionaries; spacecraft command/event/telemetry separation; Astrobee executive. | Strong.  Commands should have IDs, kinds, events, and telemetry/status. | Keep onboard runtime tiny: fixed active primitive/controller table, no onboard OS or generic lifecycle framework. |
| `sentai.explore.scan/investigate/overwatch` | IAMSAR/USCG SAR planning vocabulary. | Strong for high-level mission DSL. | Must adapt maritime/aerial SAR patterns to tiny indoor visual-servo constraints and camera FOV. |
| Arbitration between hold/goto/hover | Behavior Trees, subsumption/behavior arbitration, flight guidance active modes. | Partial.  The need is real; the exact model is not settled. | Do not freeze axis ownership or a compatibility matrix yet.  Board-side should reject unsafe/conflicting commands with clear `ERR reason=...`; host-side should interpret and replan. |
| Loadable Python primitive files | Python internal DSL/plugin pattern; F Prime dictionaries as explicit command surfaces. | Moderate.  Good for experimentation. | Must be constrained: module-as-primitive, validate `META`, no globals leakage, no blocking loops. |
| Host-side LLM/MCP adapter | MCP tools/resources/prompts. | Strong for host agent integration. | MCP is too verbose onboard.  Use MCP offboard and translate to `$...` REPL-radio calls. |

What we should refine before implementation:

- rename "primitive" carefully in user-facing docs: `skill`, `action`, or
  `mission command` may be clearer depending on layer;
- define command kinds: query, service enable/disable, action/goal,
  maintainer, recovery;
- define a minimal active primitive/controller table and exact `$active()` output;
- define minimal command rejection reasons for primitives we actually have;
- decide whether `/main.py` has a resident executive tick loop or whether v1
  relies on explicit polling;
- ensure `sentai.explore` remains high-level and does not leak autopilot jargon;
- document what cannot be done over the small radio envelope and must remain
  host-side.

## F Prime Case Study

F Prime is useful for SentAI as an architecture reference, not as an onboard
dependency.  It is a NASA/JPL C++ framework for embedded and spaceflight systems
with component interfaces, queues/threads/OS abstraction, autocoded model
artifacts, flight-worthy service components, and testing tooling.

The closest F Prime concepts for A6/B6 are:

| F Prime concept | Meaning in F Prime | SentAI equivalent to consider |
| --- | --- | --- |
| Component | Unit with typed ports and optional commands/events/telemetry. | C++ task/service owner such as marker producer, flow producer, servo owner, recorder. |
| Topology | Explicit wiring between component ports. | SentAI runtime ownership graph: PrepTask/frame producer -> marker/flow/object snapshots -> servo consumers -> logger. |
| Command | Operator/ground-facing invocation with opcode, mnemonic, typed args, and sync/async/guarded kind. | `$takeoff('m')`, `$hold(z=0.6)`, `$abort()` documented in the host-side primitive catalog. |
| Command dispatcher | Routes incoming command buffers to owning component and receives status. | REPL/radio dispatcher mapping `$...` to `sentai.prime` or explicit `sentai.*`. |
| Command sequencer | Executes command sequences and gates the next command on prior status. | Future host-side or resident MP executive for simple mission scripts. |
| Event | Structured log with severity and arguments. | `sentai.fr` events, with host-side summary reconstruction. |
| Telemetry channel | Current typed value published for ground monitoring. | Compact status tuples and periodic `sentai.fr` / host plots. |
| Parameter | Stored tunable value, loadable and settable by command. | Future persisted primitive defaults, e.g. gains, timeout, marker thresholds. |
| Dictionary/catalog | Machine-readable host-side list of primitives, events, telemetry, parameters, IDs and types. | Host-side primitive catalog for agent/MCP tools; no heavy board-side schema in v1. |
| Rate group | Periodic cyclic execution for time-critical work. | FreeRTOS/C++ frame-rate loops and PrepTask consumers, not MP loops. |
| Passive/queued/active component choice | Pick execution model based on cyclic, event-driven, or background work. | Our rule: camera-rate/control loops in C++; MP only dispatches, polls, composes. |

What we should borrow:

- an explicit host-side primitive catalog: name, short radio spelling,
  arguments, defaults, expected behavior, common failure reasons, ACK/OK/ERR
  fields, cancel behavior, and simulator validation evidence;
- separation between commands, events, telemetry/status, parameters, and data
  products;
- command IDs / sequence IDs so delayed results correlate to the initiating
  radio command;
- component ownership thinking: one owner for each producer/control output;
- sync/async distinction: queries are immediate, actions are async controllers,
  critical recovery commands are prioritized;
- event severity levels, at least compactly: `cmd`, `act`, `warn`, `err`,
  `diag`;
- host-side catalog generation, so the LLM/MCP adapter knows what primitives
  exist without scraping Python code.

What we should not copy:

- no FPP/codegen dependency onboard for v1;
- no full F Prime GDS, XML/JSON-heavy ground protocol, or generic topology
  framework in MP;
- no attempt to turn `sentai.prime` into a generic OS or task scheduler;
- no dynamic port graph: keep fixed C++ producers/consumers and a small active
  primitive/controller table.

Proposed B6 interpretation:

1. Keep the host-side primitive catalog as a later artifact discovered from
   source, simulator evidence, and board FS contents, not a heavy board-side
   schema.
2. Keep board-side primitives as normal C++/MP functions that validate at
   runtime and return compact `ERR reason=...` on failure.
3. Add a minimal executive/state model to track active primitive/controller,
   status, cancel, abort, and recovery.
4. Keep detailed evidence in `sentai.fr`; generate host summaries/plots from
   events and telemetry-like status samples.
5. Add a host-side catalog export that can become an MCP tool schema later.
6. Keep onboard replies compact:

```text
#13 ACK takeoff ctrl=servo_acquire
#13 RUN takeoff phase=acquire n=5 z=0.42
#13 OK takeoff lock=7 z=0.61
#13 ERR takeoff reason=marker_timeout
```

Open design questions for B6:

- How many event severities do we actually need on the radio path?
- Should parameters be writable during flight, or only loaded before arming?
- Should command sequencing live only host-side in v1, or should `/main.py`
  support a minimal queued sequence executor?

## cFS / CCSDS / Astrobee Notes

F Prime is the closest conceptual match for a compact command dictionary, but
cFS, CCSDS, and Astrobee add three useful angles for B6:

| Reference | What it does | What SentAI should borrow |
| --- | --- | --- |
| NASA cFS | Reusable flight software framework with OS abstraction, platform support, core executive, scheduling, inter-process communication, error management, common apps for command ingest, telemetry output, health/safety, stored commands, housekeeping, data storage. | Treat SentAI runtime services as small "apps" with clear owners; keep command ingest, telemetry/status, health/safety, stored/simple sequences as separate concepts. |
| CCSDS Space Packet | Standard packet discipline for telecommand/telemetry, including packet type, APID, sequence count, optional time, and length fields. | Keep radio messages tiny but disciplined: command/report type, controller/application ID or mnemonic, sequence ID, optional timestamp, compact payload. |
| NASA Astrobee executive | Robot executive tracks operating/mobility state, accepts or rejects commands by state, forwards accepted commands to active controllers, reports command status, handles fault/blocking states and heartbeat. | Add a minimal SentAI executive: track active primitive/controller, keep `$active()` and `$status()` coherent, route cancel/abort/recovery, but leave detailed validation to primitives. |

The cFS lesson is not "adopt cFS".  It is that mature flight software keeps
command ingestion, telemetry output, health/safety, scheduling, and stored
commands as distinct services.  For SentAI this maps well to:

- `sentai.prime` / REPL dispatcher: command ingest;
- `sentai.fr`: event/data recording;
- compact status tuples: telemetry-like channels;
- `sentai.safety`: health/safety and command rejection;
- optional `/main.py` executive: stored/simple command sequences;
- C++ PrepTask and servo tasks: scheduled/cyclic work.

The CCSDS lesson is not "use full CCSDS packets over CRTP text".  The useful
minimum is:

```text
<seq> <kind> <name-or-controller> <small-fields>
```

Examples:

```text
#17 CMD takeoff m n=3 t=8
#17 ACK takeoff ctrl=servo_acquire
#17 RUN takeoff phase=acquire n=5 z=0.42
#17 OK takeoff lock=7 z=0.61
#17 ERR takeoff reason=marker_timeout
```

If we later need binary radio frames, the same concepts can become:

| Field | Purpose |
| --- | --- |
| packet kind | command, ack, run, ok, err, event, status |
| app/controller ID | prime, safety, marker, flow, servo, fr |
| sequence ID | correlate delayed replies |
| timestamp | align host/runtime/simulator logs |
| payload length | robust parsing |
| compact payload | key/value text now, binary later if needed |

The Astrobee lesson is especially relevant for B6 because it is a robot
executive rather than only a spacecraft framework.  We should define a minimal
executive/state model that tracks active primitives and routes status,
cancel/abort, and recovery:

| State dimension | Examples | Command impact |
| --- | --- | --- |
| operating state | boot, ready, running, holding, fault, recovery | Report global posture; allow recovery commands in unsafe states. |
| mobility state | grounded, armed, taking_off, hovering, landing | Give host and primitives compact flight context. |
| evidence/model state | camera_ready, marker_lock, flow_ready, object_ready | Report evidence quality; primitives decide whether they can run. |
| active primitive/controller | `takeoff('m')` / `sentai.servo.acquire`, `follow('cat')` / `sentai.explore.follow` | Route `$status()`, `$active()`, `$cancel()`, `$abort()`. |

This gives B6 a clean rule: **the host catalog plans, the primitive validates,
and the minimal executive coordinates what is currently active.**  MP/radio
stays responsive and the board does not duplicate every primitive-specific
precondition in a central gate.

B6 implementation candidates from this research:

1. Later, derive a host-side primitive catalog from source, board FS, and
   simulator artifacts.
2. Add a minimal executive inspired by Astrobee:
   active primitive/controller tracking, `$active()`, `$status()`, cancel,
   abort, and recovery.
3. Add sequence IDs and compact command/reply kinds inspired by CCSDS/F Prime:
   `CMD/ACK/RUN/OK/ERR/EVT/STAT`.
4. Keep cFS-like service separation in naming:
   command ingest, telemetry/status, health/safety, recorder, stored sequence.
5. Do not add software bus, full packet standard, generated topology, or a
   second runtime framework.

## A6 Work Plan

1. Inventory low-level runtime primitives in `sentai.calib`, `sentai.markers`,
   `sentai.servo`, `sentai.flow`, `sentai.crazy`, `sentai.fr`, and PrepTask.
2. Mark duplicate logic and controller/ownership problems: marker windows,
   visual-Z, estimator feed, flow pumping, command controllers, status tuples.
3. Document how a future host-side primitive catalog can be discovered from
   source, board FS, and simulator artifacts; do not freeze its exact format.
4. Define `sentai.prime` load/unload semantics for simple Python primitive
   files.
5. Define command rejection reasons plus `$active()`, `$modes()`, and
   `$status()` output.
6. Define the first `sentai.explore` high-level vocabulary and map it to
   `sentai.prime`.
7. Implement only after B3/B4-equivalent primitives are named and the data
   ownership contract is clear.

## Initial Low-Level Candidates

| Primitive candidate | Current source | Notes |
| --- | --- | --- |
| `marker_observation` | `sentai.markers`, `sentai.calib` | Latest marker count, centroid, visual-Z, pose validity. |
| `marker_window` | repeated B3/B4 logic | Shared running average/min/count. |
| `visual_z_hold` | B3/B4 | Vertical visual hold primitive. |
| `rpyt_acquire` | B3/B4 takeoff/acquire | Bounded thrust/RPYT acquisition. |
| `estimator_feed` | B4 | Flow + ExtPos feeding with stale counters. |
| `generic_hover_handoff` | B4 | Handoff from RPYT to Generic Hover. |
| `image_axis_motion` | B4 | Move to image min/max/center targets. |
| `centered_descend` | B3/B4 landing | Landing while maintaining visual center. |
| `safe_stop_land` | mission recovery paths | Stop controller and land/descent. |
