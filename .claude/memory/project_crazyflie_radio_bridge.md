---
name: Crazyflie radio bridge SHIPPED (Pattern C)
description: Bidirectional host PC ↔ Crazyradio ↔ drone STM32 ↔ UART2 ↔ SentAI MicroPython REPL via custom Bitcraze deck driver. Pattern C dispatcher (C-side $-prefix exec, on_message handler, auto-fragmentation in C). Architecture, decisions, working build refs.
type: project
originSessionId: c7a210f2-a5f7-4a53-86b4-b15f5f9be074
---
2026-05-06 SHIPPED: end-to-end radio bridge between host PC (Crazyradio
PA) and the SentAI board's MicroPython REPL via a Crazyflie 2.1
brushless drone acting as relay. Pattern C dispatcher (board build
1183) replaces the original poll_event/`>>> ` listener loop with a
C-side dispatcher: built-in `$`-prefix REPL exec, single-handler
`sentai.crazy.on_message`, and auto-fragmentation in C for `link_send`.
Validated `cf.send_packet(port=0x0E, data=b'$1+1')` → `<-- ch=0 b'OK 2'`
round-trip in tens of ms, LED RED_R stays lit throughout, no scheduler
stall.

**Why:** Need a way to drive the SentAI MicroPython REPL remotely
without the board being USB-tethered, and a clean future path for the
board to issue control commands to the drone (board = master). Pattern
C removes the need for a Python listener loop on the board, so REPL
remains free for user work / main.py.

**How to apply:** When extending board↔drone comms, all detail lives
in `agent.md` §18 (constants, conventions, anti-patterns, build,
Pattern C dispatcher) and `experiment.md` "Crazyflie ⇄ SentAI radio
bridge" section. Drone-side fork:
`https://github.com/bogdannedelcu/crazyflie-firmware`, branch
`sentai-deck-driver` (rebased on bitcraze/master); board build ≥1183.

**Constants etched in stone**:
- CRTP_MAX_DATA_SIZE = 30 (radio limit, can't be raised)
- Wire format `[0xAA][LEN][CH][DATA][CRC]`, LEN ≤31, CRC = XOR
- Reserved CRTP port 0x0E for the bridge
- Channels: 0=REPL bidir / 1=flow inject board→drone (16B flow_pkt_t)
  / 2=reserved drone control / 3=reserved
- Per-fragment payload on ch=0 is 29 B (1 B `MF` prefix);
  link_send auto-fragments in C (transparent to caller)
- UART2 baudrate 576000 8N1
- `$` (single byte) prefix marks built-in REPL exec on ch=0
  (chosen over `>>> ` to save 3 of the 29 useful payload bytes)

**Pattern C dispatcher architecture**:
- SPSC FIFO `g_dispatch_q[8]` between rx task and MP VM (drop-newest
  with counter; 256 B max per slot)
- Single-shot `mp_sched_schedule(crazy_dispatch_drain_obj, ...)`
  (`g_crazy_drain_pending` flag dedupes — trampoline drains entire
  FIFO in one VM tick)
- `MICROPY_BEGIN/END_ATOMIC_SECTION` overridden to FreeRTOS
  `taskENTER/EXIT_CRITICAL` (default embed-port no-op is unsafe
  cross-task). Wrappers live in `mp_embed_safe.c` so QSTR pre-pass
  doesn't see FreeRTOS includes
- `crazy_msg_handler` registered as MP root pointer; `on_message(None)`
  detaches and frames are dropped silently
- C-side REPL exec uses stack-only `VSTR_FIXED(200)` for replies — no
  GC heap on the reply path; reply bounded at 200 B then auto-fragmented
- **CRITICAL: `mp_handle_pending(true)` is called from `repl_getchar`
  and `repl_getchar_timeout` in micropython_task.c.** Without this,
  the scheduled trampoline never fires while at the REPL prompt
  (MP VM is blocked on stdin, no bytecode running, scheduler
  dormant). Mainline MP ports do this in their `mp_hal_stdin_rx_chr`
  equivalents. With this hook in place, every 10 ms idle the
  scheduler queue drains — async events deliver within ~10 ms even
  when no user code is running.

**End-to-end validated 2026-05-06 via USB2serial probe** (board build #1202):
- Sent `$1+1` (AA 05 00 24 31 2B 31 A0) into board UART RX
- Got back `aa 06 00 00 OK 2 ba` (AA 06 00 00 4F 4B 20 32 BA) on board UART TX
- Same for `$2*3` → `OK 6` and `$5**2` → `OK 25`
- Same DURING `sentai.rtos.sleep_ms(8000)` — replies arrived at
  t=4.0/4.8/6.0 s while sleep was still running (chunked sleep
  drains scheduler every 10 ms)
- Round-trip latency: tens of ms; no LED stall, no scheduler stall

**End-to-end validated 2026-05-06 via Crazyradio (board ↔ drone ↔ host)**:
- `$1+1` → `OK 2` ✓
- `$sentai.version()` → `OK 'SentAI v1.0 build 1202 (...)'` (49 B reply)
  auto-fragmented in C, host reassembled via MF prefix ✓
- `$sentai.imu.read()` → `OK None` ✓
- `$1/0` → `ERR ZeroDivisionError: divide by zero` ✓ (exception caught)
- `$sentai.io.led(1, 1)` → `ERR AttributeError: ...` ✓ (exception caught)
- non-`$`-prefixed message with no on_message handler → silently dropped ✓

**MicroPython scheduler-drain coverage** (where mp_handle_pending fires):
- ✅ At REPL prompt waiting for keystroke — `repl_getchar` /
  `repl_getchar_timeout` in `micropython_task.c` call
  `mp_handle_pending(true)` between read attempts (every 10 ms)
- ✅ Running Python bytecode — VM calls `mp_handle_pending` at branch
  opcodes (FOR_ITER, JUMP, etc.) per upstream
- ✅ Inside `sentai.rtos.sleep_ms(N)` — chunked into 10 ms slices in
  `modsentai_rtos.c`, drains scheduler each slice
- ❌ Long C-side calls (TPU invoke, camera capture, fs writes): block
  for the duration. Bounded but the trampoline waits.
- ❌ Custom Python tight loops without I/O: scheduler fires only at
  branch opcodes inside the loop body. Usually fine.
- ❌ `sum(range(N))` / built-in C reductions: no Python branches → no
  scheduler tick. Use a `for` loop if you need scheduler responsiveness
  during heavy computation.

**Anti-patterns to never repeat**:
- CPX-over-UART for third-party decks (portMAX_DELAY in CPX_UART_TX,
  BSS overflow CPXPacket_t vs CPXRoutablePacket_t)
- `CONFIG_APP_ENABLE` + Appchannel polling in appMain (starves
  scheduler, LED RED_R goes dark on host open_link)
- Compression for short REPL replies (zlib/smaz both negative win
  under 100 B; help.txt 67KB → 50KB with smaz, 20KB with zlib but
  separate task from bridge)
- FreeRTOS includes inside `mpconfigport.h` (QSTR pre-pass cpp's it
  without firmware include paths and explodes — route via extern
  wrappers in mp_embed_safe.c)
- Two on_message handlers (Pattern C is intentionally single-handler;
  user routes by channel internally)
- **MOST IMPORTANT** (2026-05-06): Leaving `CONFIG_ENABLE_CPX=y`
  (default!) in the drone fork's `app-config`. Upstream Kconfig has
  `ENABLE_CPX` default `y` AND it `select`s `ENABLE_CPX_ON_UART2`,
  which compiles in `cpx_uart_transport.c`. That transport's CTR/TX
  task races our deck driver for the **shared static globals
  `txBuffer/txIdx/txSize`** in `uart2.c` — both call `uart2SendData`,
  the second overwrites the first's in-flight state, ISR feeds wrong
  bytes, TX_DONE never fires for our deck → callback wedges forever
  → CRTP RX task stalls → R2U stops incrementing.
  **Symptom**: bridge works for 3-5 packets then dies; only fix is
  drone power-cycle. **Real fix**: add `CONFIG_ENABLE_CPX=n` to
  `examples/app_sentai_bridge/app-config`. Verified 2026-05-06 via
  USB2serial probe at 576000 — drone TX is bit-perfect every send
  after disabling CPX.

**Drone-side debug recipe (USB2serial)**:
- CP2102 / FTDI / CH340 adapter at 576000 8N1, 3.3 V
- adapter RX ← drone PA2 (TX2 deck pad), adapter TX → drone PA3
  (RX2), shared GND mandatory
- Standard CF2.x deck pinout puts TX2/RX2 on the LEFT header
- Listen with `serial.Serial('/dev/ttyUSB0', 576000, timeout=0.4)`
- Valid frames look like `aa LL CH ... CRC` where CRC = XOR of all
  preceding bytes. `$1+1` reply frame: `aa050024312b31a0`

**Open work** (priority order, post-2026-05-06):
- Board → drone command injection on a new channel (3) — opcodes
  for takeoff / land / arm / setpoint via direct API on drone like
  `crtpCommanderHighLevelTakeoff`, `supervisorRequestArming`,
  `commanderSetSetpoint`. This is the "board commands the drone" path
  the project is heading toward.
- Host-side reassembler library for the MF prefix.
- `sentai.crazy.diag()` MP binding for the counters.
- Upstream PR after flight hours.

**Completed 2026-05-06 (Pattern C + telemetry)**:
- ✅ Pattern C dispatcher with `$`-prefix REPL exec
- ✅ `sentai.crazy.on_message(cb)` user handler path
- ✅ Auto-fragmentation in C for `link_send(0, …)`
- ✅ MicroPython scheduler-drain hooks (REPL stdin + chunked sleep_ms)
- ✅ Channel 2 telemetry — see opcode table in `agent.md` §18
- ✅ `CONFIG_ENABLE_CPX=n` on drone — kills the UART contention

**Completed 2026-05-07 (review fixes + extended telemetry)**:
- ✅ All 🔴/🟠 NASA-JPL review fixes (board: heap discipline, dead
  code removal, type safety, scheduler-drain semantics; drone:
  build-time CPX guard, no-cache-on-failure, dataLen tightening,
  endianness asserts)
- ✅ `uart2SendDataBounded()` — 50 ms timeout variant of
  `uart2SendData`; deck driver no longer wedges on TX_DONE stall.
  Counter `deck.sentaiTxTo` exposed for field diagnostics.
- ✅ Extended telemetry opcodes 0x10/11/12 (attitude),
  0x20/21/22 (velocity), 0x30/31/32 (canfly/isFlying/isTumbled).
  MP bindings `attitude_get()` / `velocity()` (return tuple) /
  `canfly()` / `is_flying()` / `is_tumbled()` (return bool).
- ✅ All committed + pushed to `bogdannedelcu/coralmicro` and
  `bogdannedelcu/crazyflie-firmware`. Board persistent flash #1209,
  drone fork tip `9d1f0b77`.

**Completed 2026-05-09 (radio-only operation)**:
- ✅ Drone build refreshed with all bridge work + kalman_core baro
  propwash fix (commit `53d72897`).  cf21bl.bin verified contains
  `bl <kalmanCoreUpdateWithBaro>` symbol; sentaiFlow/sentaiFlBad/
  sentaiTelem/sentaiTxTo PARAMs present; `CONFIG_ENABLE_CPX=n`
  preserved in app-config (anti-CPX-race guard from 2026-05-06).
- ✅ Board build #1220 persistent flash + `/main.py` auto-init
  (`crazy.init() + camera.init() + flow.enable() + flow.start(0)`)
  → board boots into radio-bridge-ready + flow-publishing state
  with ZERO USB connection required.  Tested: post-`sys.reset()`
  `sentai.crazy.baro()` returned 99.57 m within 200 ms; `flow.pub_stats()
  ['running'] = True`.
- ✅ End-to-end radio bridge re-validated post-reflash (build #1220 +
  drone @ 53d72897): 9 `$`-prefix REPL exec round-trips clean (`$1+1`
  → `OK 2`, `$sentai.version()` 49 B auto-fragmented, `$1/0` →
  `ERR ZeroDivisionError`); 12 telemetry queries via radio (`baro`,
  `altitude`, `battery`, `attitude_get`, `velocity`, etc.) all
  returned plausible values; 5 flow injects via radio
  (`$t=sentai.crazy.send_flow; $t(0,0,0.033,1)` × 5) advanced
  `deck.sentaiFlow` from 0 → 5, 0 rejected.
- ✅ Body_xform[0] = `(-1, 0, 0, +1)` empirically verified with
  controlled translation (forward 10 cm → dx=-222 mgp, back → +364,
  left → +233).  See `project_flow_body_frame_baseline.md`.
- ✅ Both repos pushed: `coralmicro@36f8a2c3` (7 new commits since
  last push), `crazyflie-firmware@53d72897` (3 new commits).

**Completed 2026-05-09 EOD (sync API for power-cycle durability)**:
- ✅ `sentai.fs.sync()` MP binding shipped (board build #1221+).  Wraps
  the existing `FxUserSync()` C API which calls `fx_media_flush()`.
  Returns True on success, False on mount/lock failure.  CRITICAL
  for any upload that needs to survive a power cycle (USB unplug,
  battery removal) — without it, files written via fs.write/append
  live in SDRAM cache and vanish at cold boot.
- ✅ `_host_upload_repl.py` auto-calls `fs.sync()` after every
  upload, with backward-compat fallback for older firmware
  (prints `sync=NO-API` warning so caller knows upload is
  sys.reset-durable but NOT power-cycle-durable).
- ✅ Repro confirmed before fix: uploaded /main.py via fs.append,
  `sys.reset()` → main.py ran, `baro=99.57m flow.running=True`.
  User power-cycled USB → next boot `sentai.fs.exists('/main.py') ==
  False`, bridge silent forever, `cf.send_packet(... b'$1+1')` → no
  reply.  Drone counters showed R2U +N (drone forwards radio→UART
  fine) but U2R==0 (board never replied) — diagnostic that points
  to "FAT lost the file" not "bridge wedged".
- Build #1221, push `coralmicro@2a2097ce`.

**Critical caveats discovered 2026-05-09**:
- ~~**Board reset → bridge state lost**~~ **OBSOLETED 2026-05-09 EOD**:
  fixed by moving `sentai_crazy_init(576000)` into firmware
  (`micropython_repl_task` claims UART2 BEFORE /main.py runs).  Now
  `g_crazy_running` is set unconditionally on every boot.  /main.py
  no longer needs `sentai.crazy.init()`.  Telem queries work
  immediately after any reset.
- **CRTP 30-byte limit**: `$sentai.crazy.send_flow(0,0,0.033,1)` is
  35 chars > 30.  Workaround: pre-set alias on board via radio:
  `$t=sentai.crazy.send_flow` (25 chars, fits) then call as
  `$t(0,0,0.033,1)` (15 chars, fits).  Aliases survive within a
  single boot (REPL globals dict).
- **Drone Kalman estimator default = 1 (Complementary)**, NOT 2
  (Kalman).  With Complementary active the EKF queue ignores
  flow injections — `kalman_pred.predNX/measNX` stay 0 even with
  `deck.sentaiFlow` incrementing.  Set per-boot via cflib:
  `cf.param.set_value('stabilizer.estimator', 2)`.  Not
  PARAM_PERSISTENT upstream so resets every drone boot.
- **`cf.param.get_value()` is cache-per-connect**, not live.  For
  monitoring counter changes during a test, either reconnect to
  refresh OR subscribe via LogConfig (push-based).  Build a
  pre/post snapshot for delta measurements -- mid-run polls won't
  see updates without reconnect.

**Completed 2026-05-07 (CH=1 flow injection end-to-end)**:
- ✅ MP binding `sentai.crazy.send_flow(dpx, dpy, dt, std)` — packs
  16-byte `flow_pkt_t` (4 × float32 LE), ships via existing
  `link_send(CH=1, ...)`. Single-frame, no fragmentation.
- ✅ MP binding `sentai.flow.period_ms()` — publisher cadence so
  callers can size `dt` clamps.
- ✅ Drone PARAM exports `deck.sentaiFlow` (injected) and
  `deck.sentaiFlBad` (rejected) for radio-side observability.
- ✅ Validated end-to-end with real `sentai.flow`: 6 s @ 30 Hz =
  180/180 packets, drone counter 0 → 180 over radio, 0 rejected,
  0 CRC errors. Second pass advanced 180 → 360 cleanly.
- ✅ Reference driver `diag/_t_flow_to_drone.py`; runnable via
  `_host_paste_bench.py --file diag/_t_flow_to_drone.py --fps 0`.
  (REPL paste path used because `/lib` FAT volume on board was
  found corrupt — `sentai.fs.format()` deferred to user; not a
  bridge issue.)
- ✅ Board build #1211 (RAM); drone fork tip rebuilt with PARAMs added.

**Completed 2026-05-09 → 05-10 (firmware auto-init + boot brick fix)**:
- ✅ Crazy bridge auto-init MOVED INTO FIRMWARE
  (`micropython_repl_task` calls `sentai_crazy_init(576000)` BEFORE
  /main.py runs).  Decouples radio recovery from /main.py state —
  if /main.py is corrupted, missing, or skipped by SAFE_MODE, board
  STILL boots radio-ready.  Anti-brick §M satisfied: only path that
  could leave board radio-deaf required USB to recover.  Now no
  matter what /main.py does, board enumerates radio bridge.
  Commit `c4577170` (NASA mount FSM + radio auto-init).
- ✅ /main.py simplified to MISSION-only: camera.init + flow.enable
  + flow.start(0).  No more crazy.init() in main.py.
- ✅ FxUserSync now flushes ALL 3 cache layers (FileX sector + FAT +
  LevelX log/wear-level via close+reopen).  Power-cycle durability
  validated.  Cost ~200-500 ms per explicit sync; zero on hot path.
- ✅ FxUserInit bounded retry (3×50 ms) + SAFE MODE on persistent
  failure (instead of silent auto-format that destroyed user data).
  Auto-format ONLY on virgin-NAND signature
  (LX_SYSTEM_INVALID_FORMAT / LX_NO_PAGES).
- ✅ **CRITICAL boot-brick fix** (commit `7a03d9a1`): the bounded
  retry loop above used `vTaskDelay(50)` PRE-SCHEDULER (LfsUserInit
  runs from CHECK in real_main:389 before vTaskStartScheduler at
  553).  vTaskDelay null-derefs pxCurrentTCB pre-scheduler →
  HardFault → boot loop → board enumerated as `1fc9:9307` HID-only,
  never reached `1fc9:c0a1` app mode.  Took ~3 hours to bisect via
  JTAG (g_boot_persist.progress=0x06, scheduler=1, app_main task
  never ran, GPR1 boot_attempts=0).  Fix: `bounded_delay_ms()`
  helper that gates on `xTaskGetSchedulerState()` — busy-spin
  pre-scheduler, vTaskDelay post-scheduler.  See
  `feedback_vtaskdelay_pre_scheduler.md`.
- ✅ End-to-end re-validated 2026-05-10 on drone battery (USB
  unplugged): radio sanity `$1+1`, cold-start camera+flow via
  inline `$exec`, LED-cued 8 s motion test producing 50/50 fresh
  flow frames (peak |dy|=168 mgp), REPL+flow concurrent at 30 fps
  publisher rate without starvation.  Build #1224, commits
  `7a03d9a1` + `ede34514` (radio MTU best-practice doc) +
  `922f7dec` (.gitmodules aifes registration).

**Open work** (priority order):
- Board → drone command injection on a new channel (3) — opcodes
  for takeoff / land / arm / setpoint via direct API on drone like
  `crtpCommanderHighLevelTakeoff`, `supervisorRequestArming`,
  `commanderSetSetpoint`.
- Defer drone-side `uart2SendData` to a TX queue/task (D-C2 from
  the review) — purely architectural now that bounded variant
  closes the wedge surface.
- Host-side reassembler library for the MF prefix.
- `sentai.crazy.diag()` MP binding for the counters.
- Upstream PR after flight hours.
- **Mâine 2026-05-10**: drone in flight, hover with flow injection live —
  validate `kalman_pred.measNX/Y` actually moves with `sentai.crazy.send_flow`
  pumped at 30 Hz; board does the publisher loop on-board to avoid radio-rate
  bottleneck (radio is for supervision, not real-time control —
  see `feedback_radio_no_file_transfer.md`).
