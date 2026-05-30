# TD-S10-A7 - Align Runtime Namespaces With SentAI Taxonomy

## Goal

Bring the existing `sentai.*` runtime closer to the A6/B6 taxonomy without
changing flight behavior.  A7 is the concrete bridge between the research
taxonomy and implementation work: name what already exists, identify where SIM
and ARM differ, and make future progress easy to reference.

A7 is mostly about naming, inventory, and safe introspection.  It should not
retune flight controllers, rewrite B5/B4 logic, or introduce a new mission
language yet.

## Context

A6/B6 established the working taxonomy:

- `sentai.*` is the low-level runtime surface;
- `sentai.prime` is the future short public action/query/recovery surface;
- `sentai.explore` is the future high-level mission vocabulary;
- `sentai.fr` remains the forensic evidence stream;
- host-side reasoning and MCP/tool schemas stay offboard;
- board-side C++ owns fast loops and safety-critical controller work;
- board-side MP may compose cooperative skills but must stay interruptible.

B6 D8 also identified the current runtime substrates:

| Concept | Current runtime owner |
| --- | --- |
| View / perception producer | `sentai.pipeline`, `sentai.camera`, PrepTask |
| Object memory | `sentai.objects` |
| Lifted object / track-to-world bridge | `sentai.object_lifter` |
| Hex / place memory | `sentai.places` |
| SLAM / landmarks / pose diagnostics | `sentai.slam` |
| Flight/control surfaces | `sentai.servo`, `sentai.crazy` |
| Existing mission FSM precedent | `sentai.explore` |
| Evidence | `sentai.fr` |
| Safety / health | `sentai.safety`, `sentai.diag`, `sentai.sys` |

## Non-Goals

- No new `sentai.prime` implementation in A7 unless a later B7 explicitly
  scopes a small non-invasive slice.
- No new world-model DSL.
- No compatibility matrix.
- No retuning of B5/s207 flight behavior.
- No breaking changes to existing mission APIs.
- No moving camera-rate, flow, marker, or control loops into MicroPython.

## Runtime Alignment Tasks

1. Inventory all exported `sentai.*` namespaces on ARM and SIM.
2. Mark which namespaces are production runtime, diagnostic, experimental, or
   legacy.
3. Identify SIM/ARM parity gaps that block future model-query or primitive
   experiments.
4. Document the canonical current model substrates:
   `objects`, `object_lifter`, `places`, and `slam`.
5. Document `sentai.pipeline` / PrepTask as the canonical View producer and
   frame preprocessing boundary.
6. Document `sentai.explore` as an existing mission-FSM precedent, not yet the
   final high-level taxonomy implementation.
7. Define a small set of safe read-only introspection checks that can run in
   SIM without changing flight state.
8. Keep B5/s207 as the regression mission: after A7/B7 changes, the migrated
   B4 mission must still run.

## Naming Conventions

Use these names when discussing future work:

| Name | Meaning |
| --- | --- |
| Runtime surface | Current `sentai.*` APIs exposed to MicroPython. |
| View producer | Camera / PrepTask / pipeline source that prepares shared snapshots. |
| Model substrate | Existing board-side memory source such as objects, places, lifted tracks, or SLAM landmarks. |
| Controller surface | Existing low-level action/control owner such as servo, crazy, calib, safety. |
| Public primitive surface | Future `sentai.prime` commands exposed as short REPL/radio calls. |
| Mission vocabulary | Future `sentai.explore` skills such as scan, investigate, overwatch, report. |
| Evidence stream | `sentai.fr`, consumed host-side for summaries and plots. |

Avoid introducing `sentai.map` or `sentai.world` as assumed namespaces for now.
The current concrete map/place substrate is `sentai.places`, and the current
world/perception model is distributed across existing runtime owners.

## Safe Introspection Direction

A7 should prefer read-only checks and host-side reports:

```text
list exported namespaces
compare SIM vs ARM namespace tables
verify model substrate modules exist
verify basic stats/status calls do not mutate flight state
record gaps as follow-up todos
```

Any future query facade such as `$model()`, `$objects()`, `$places()`, or
`$detail(id)` remains experimental until simulator missions show what the host
actually needs.

## Regression Rule

Every implementation step derived from A7 must preserve the ability to rerun the
stable B5/s207 mission.  If a change affects `sentai.servo`, `sentai.calib`,
`sentai.markers`, `sentai.flow`, `sentai.pipeline`, or `sentai.crazy`, verify
that the migrated B4 mission still behaves as before.

## Expected B7 Shape

B7 should be a small implementation task, not a rewrite:

1. Add or update host-side namespace inventory tooling.
2. Reach minimal SIM/ARM parity for namespaces needed by experiments.
3. Add read-only smoke checks for model substrates.
4. Add documentation generated from or verified against the runtime surface.
5. Run B5/s207 as regression.

The first B7 slice should make the runtime easier to inspect and reason about.
It should not yet build the full `sentai.prime` dispatcher.
