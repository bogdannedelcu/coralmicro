# s245 — camera TRIGGER hardware feasibility probe

Anchor: `TD-S10-A10` (paper §4.2) · EXP-s245 · follows
[s243](../s243_camera_alternation_yuv/) (free-running alternation) and
[s235/iter10](../s235_revalidate_paper_performance/iter10_visual_seam_validation/)
(per-frame single-source content verification).

**WBS**: TD-S10-A10
**Started**: 2026-09-04
**Status**: 🟡 in progress — trigger question closed (iter1-3); FPS question open (iter4)

## Why this experiment exists

s243 measured dual-camera throughput under the *free-running* alternation path:
`camera.ratio(a,b)` lets the CSI ISR flip the MUX on its own schedule and the
consumer takes whatever arrives. Result: 1:1 alternation costs **−44 % FPS**
(38.8 → 21.6 fps at VGA45) — almost exactly *half* the sensor rate.

The proposed change of approach is **demand-paced ("triggered") capture**: we ask
for a frame from camera X, we get exactly that frame, we switch, we ask again.
Before building that, this experiment establishes what the hardware can actually
do, because a source audit found the obvious path is not implemented.

## Source audit — what exists today (build 1593)

| Thing | State in tree | Evidence |
|---|---|---|
| `CameraMode::kTrigger` | enum defined, documented | `libs/camera/camera.h:42` |
| `CameraTask::Trigger()` | **empty stub** — does nothing | `libs/camera/camera.cc:746` |
| `kTrigger` gates in `GetFrame()` | **commented out** | `libs/camera/camera.cc:490-510` |
| `HandleEnableRequest(mode)` | **ignores `mode`**, always `CAMERA_RECEIVER_Start` | `libs/camera/camera.cc:896` |
| `camera.init(streaming=0)` | argument accepted, then discarded → streaming anyway | `sentai_runtime.cc:2775` |
| `Gpio::kCameraTrigger` | mapped GPIO8_IO27, pad `GPIO_EMC_B2_17`, output, level 0 | `libs/base/gpio.cc:41,71,148,383` |
| firmware ever drives that pin | **never** — all call sites commented | grep: only `camera.cc:501,510` |
| OV5640 FREX/strobe regs `0x3B00-0x3B0C` | only inside a register-dump range, never programmed | `libs/camera/camera_support.c:919` |

**Conclusion of the audit: there is no trigger path in this firmware.** Trigger
mode is inherited dead code from upstream coralmicro.

### GPIO electrical sanity (resolved from source)

`GPIO_EMC_B2_17` has `SEMC_DM03` as an alternate function — data mask 3, used only
by a 32-bit SDRAM port. This board configures `kSEMC_PortSize16Bit`
(`libs/nxp/rt1176-sdk/board_hardware.c:76`, 64 MB SDRAM), so only DM0/DM1 are in
use and **DM03 is free — driving this pad as GPIO cannot corrupt SDRAM.**

What source *cannot* answer: whether GPIO8_IO27 is physically routed to the OV5640
FREX (or strobe) pins on the SentAI board. The name is inherited from the
single-camera Coral Dev Board Micro. That is what iter1 probes.

## Claim

That the feasibility of a hardware-triggered dual-camera capture path on this
board can be decided from measurement, not assumption, by establishing:

1. whether the OV5640 FREX / snapshot block is reachable and configurable over
   I²C on **both** sensors (iter1, no firmware change), and
2. whether `GPIO8_IO27` is electrically connected to anything, i.e. whether an
   external hardware trigger line exists at all (iter2, needs firmware), and
3. whether any OV5640 output pad reaches that pin, tested functionally by making
   the sensor drive and watching whether the pin follows (iter3), and
4. what the ISR-level floor of alternating capture is — the true hardware cost of
   a MUX flip measured in µs, driven from the rendezvous `CSI_IRQHandler`
   already provides, not through the MicroPython/task round-trip that s243
   measured (iter4).

## Governing hypothesis (to be tested in iter4)

`CSI_IRQHandler` flips the MUX inside VBLANK, and build #932 measured ISR latency
at **2 µs peak vs ~3 ms of VBLANK at VGA45** — a 1500× margin
(`camera_support.c:363-530`). If the flip always lands in VBLANK, then the very
next DMA buffer is *already* 100 % from the new sensor, and 1:1 alternation should
sustain the **full sensor rate** (45 fps aggregate at VGA45 = 22.5 fps/camera)
with zero discarded frames.

s243 measured 21.6 fps aggregate — half of that. So roughly one good frame per
flip is being thrown away. Candidate causes, all visible in source:

- `g_cam_dirty_consecutive_n = 1` marks the next FB-done buffer dirty after a
  flip, which may be discarding a *clean* frame (`camera_support.c:521`);
- `switch_drain` adds a further whole-frame wait
  (`sentai_runtime.cc:2842+`, threshold default 1);
- both wait loops use `vTaskDelay(1 ms)` — tick quantization layered on a 22 ms
  frame period (`sentai_cam_switch`, `get_raw_with_recovery`).

If confirmed, there is up to a **2× headroom** on dual-camera capture.

## Pass criteria

- **iter1**: FREX register block `0x3B00-0x3B0C` read back from cam0 AND cam1
  with a non-error value; write-readback-restore on a scratch FREX register
  succeeds on both, proving the block is live (not a read-only shadow). Verdict
  recorded either way — a negative result is a valid result.
- **iter2**: pin state of GPIO8_IO27 classified as FLOATING (follows an internal
  weak pull in both directions → unrouted) or TIED/LOADED (does not follow),
  with an SDRAM integrity check passing across the whole toggle sequence.
- **iter3**: for every implemented `0x3016` pad-enable bit on both sensors, a
  recorded pin reading under sensor-drive-low and sensor-drive-high. A positive
  requires the pin to TRACK the driven level; the pre-flight must first prove
  the register writes take, or a "no response" would prove nothing.
- **iter4**: ISR-level histogram of arm→clean-frame latency in µs, N ≥ 500
  flips, reported as median/p95/max; plus the measured max sustained alternating
  capture rate compared against the sensor rate, with per-frame content
  verification so throughput is not won on dirty frames.
- HW: build counter verified post-flash + USB enumerated as `1fc9:c0a1`.

## Anti-cheat note

Not applicable — this is a bare-metal HW experiment on the physical board, no
`sentai_sim` and no Gazebo involved, so
[[sentai-sim-air-gapped-from-truth]] does not bind here. No ground-truth source
exists to leak from.

## How to run

```bash
python3 iter1_sensor_frex_probe/probe_frex.py                 # no firmware change needed
python3 iter2_gpio_routing_probe/probe_pin.py                 # needs build >= 1595
python3 iter3_frex_functional/probe_frex_functional.py        # needs build >= 1596
```

## Iter results table

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | OV5640 FREX block is reachable + writable over I²C on both sensors | 0 read errors, 0 cam0/cam1 divergences, FREX regs LIVE (write-readback-restore exact on both). Pads disabled (`0x3016`=0x00), strobe off (`0x3B00`=0x00). Found `0x3B08` FREX_REQUEST = a **software** trigger path that may not need the GPIO at all. | ✅ |
| iter2 | GPIO8_IO27 is / is not routed to the sensors | Pin under test follows both internal pulls (5/5 runs, stable) = **no driver on the net**; adjacent known-routed control `kEdgeTpuPgood` held against the pull = **method validated**. SDRAM canary clean 5/5, confirming DM03 is free. Cannot exclude a high-Z FREX input — that needs the functional test. | ✅ |
| iter3 | Functional test: does any OV5640 output pad reach GPIO8_IO27? | **No.** 16 configs × 2 sensors, pin never tracked `0x3019`. Pre-flight proved `0x3016` implements only bits 0-1, so every existing pad-enable bit was exercised. Two host-side bugs caught and fixed en route (invalid parse, false positive from the floating no-pull column). | ✅ |
| iter4 | ISR-level flip cost; 1:1 alternation can reach full sensor rate | not started | ⬜ |

## Result so far — the trigger question is closed

**There is no usable hardware trigger path on this board.**

1. **No firmware trigger** (source audit): `CameraMode::kTrigger` is inherited
   dead code — `Trigger()` is an empty stub, the gates in `GetFrame()` are
   commented out, and `HandleEnableRequest()` ignores its `mode` argument, so
   `camera.init(streaming=0)` silently gives streaming.
2. **No trigger pin** (iter2 + iter3): nothing drives `GPIO8_IO27`, and no
   OV5640 output pad reaches it on either sensor. `Gpio::kCameraTrigger` is a
   name inherited from the single-camera Coral Dev Board Micro. Only a FREX
   *input* remains unexcludable, and that needs a schematic or a scope, not
   more firmware.
3. **But a sensor-side software trigger exists** (iter1): the FREX block is
   live and writable on both sensors, including `0x3B08` FREX_REQUEST. That
   offers *determinism and cross-camera synchronisation* — not throughput,
   since the frame still traverses MIPI and I²C paces the request.

### What this means for the original question

The question that started s245 was "how many FPS can demand-paced dual-camera
capture reach?". The hardware cannot be triggered, so the answer has to come
from the free-running path — which is where the headroom appears to be anyway.

s243 measured 1:1 alternation at 21.6 fps aggregate, **exactly half** the 45 fps
sensor rate, while `CSI_IRQHandler` flips the MUX inside VBLANK with a 1500×
timing margin, meaning the next DMA buffer should already be clean. Roughly one
good frame per flip is being discarded. That is iter4, and it is worth up to
**2×**.
