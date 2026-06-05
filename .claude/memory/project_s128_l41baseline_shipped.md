---
name: s128-l41baseline-shipped
description: "s128 L4.1Baseline (pre-seeded marker tour) — 2026-05-14 first PASS. Hybrid model: REPL drives sentai.servo intent + cflib MotionCommander flies cf2 in lockstep. Validates L2 + L4 + cf2/flow integrated. Currently 1-marker scope; multi-marker pending tighter dist tolerance."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**State 2026-05-14**: 1-marker tour passes end-to-end.
- mission_l41.py: spawns sentai_sim with bidirectional pipes,
  drives REPL + cflib in lockstep, journal'd via sentai.sim
- Result: trace_count=6, actions_ok=7, all fault counters=0,
  cf2 dist to marker=0.225 m (threshold 0.30 m)
- Path: `examples/sentai_runtime/experiments/s128_l41baseline_seeded/`

## Hybrid model (load-bearing)

Because L4's `sentai.servo` is still the SKELETON (Stage 4.A transport
wiring not shipped — see [[servo-l4-shipped]]), `servo.move()` records
intent into a trace ring but does NOT command cf2.  s128 runs the
intent recording AND the actual cf2 flight via cflib MotionCommander
in LOCKSTEP: each loop iteration calls both `repl.exec("sentai.servo.move(dx,dy,dz)")`
and `mc.move_distance(dx, dy, dz)`.

When Stage 4.A ships, the cflib calls become the legacy parallel path
and `sentai.servo.move()` becomes the only driver; this same harness
will still pass without change to the L4 verdict checks.

## Pass criteria (verdict.py)

L4-side: actions_ok==7, all faults==0, last_action==DISARM, last_result==0,
final armed==0/flight==GROUND, trace seq matches
`[ARM, TAKEOFF, MOVE, HOVER, LAND, DISARM]`.

L2-side: objects.count() survives flight.

Ground truth: `‖cf2.stateEstimate − target_xyz‖ < 0.30 m` at each hover.

## REPL drive mechanism (load-bearing)

ReplDriver spawns `sentai_sim` with `subprocess.Popen(stdin=PIPE, stdout=PIPE)`.
Key design notes:

- **sentinel marker for int returns**: `exec_int` wraps the cmd in
  `print('!RV…!', <expr>)` to ignore async stdout chatter from
  `camera_bridge` (rgb_crc debug, not gated by sentai.verbose).
- **fs.write for big dumps**: `exec_via_fs` uses `sentai.fs.write` to
  hand any return value through the SIM filesystem — immune to stdout
  noise.  `sentai.fs.write` returns `True`/`False` so the caller must
  use `exec_value` not `exec_int` (`int('True')` raises).
- **structured journal**: every action is mirrored to `sentai.sim.journal_*`
  via the `j_int` helper.  See [[sentai-sim-journal]].

## Outputs (under /tmp/s128_l41baseline/)

- `summary.json` — verdict reads this
- `journal.txt` — copy of REPL-written journal (LAST line = last
  completed step on crash — primary post-mortem artifact)
- `repl.transcript` — every stdin/stdout byte from sentai_sim
- `mission.log` — step-numbered host log
- `cf2_telemetry.json` — stateEstimate samples (50 Hz)
- `servo_status.json`, `servo_trace.json`, `objects_list.json`

## Known followups

- Multi-marker tour deferred until 1-marker is rock-solid.  Operator's
  call 2026-05-14 was "elimina zgomotul, navighează la un singur marker".
- WAYPOINT_NEAR_M=0.30 is loose; tighten once we see better cf2 settle
  after `move_distance`.  First run got dist=0.225 m which is right at
  the threshold edge.
- Bridge timing: gz_to_uds_bridge launches AFTER sentai_sim opens
  cam.sock.  Race condition would be: sim spawn → cam.sock listener →
  bridge attaches → flow_out.sock listener → flow_forwarder thread
  retries connect 20×0.25s.  Works today but fragile if any step
  slows down.

## L4.2Baseline (next, NOT shipped)

s129: same scaffolding but with live ArUco detection feeding
`sentai.objects.add` during flight (not pre-seeded).  Same verdict
matrix; if L4.1 passes and L4.2 fails, the regression is in the
perception pipeline (camera/tracker/PnP/anchor_pose) NOT L4 control.

Related: [[servo-l4-shipped]], [[objects-l2-shipped]],
[[flowbaseline-canonical-config]], [[sentai-sim-journal]],
[[sim-repl-test-recipe]], [[gazebo-gui-required]].
