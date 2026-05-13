# s112 — x86 SIM ArUco anchor shim (platform abstraction)

## What this proves

Brings up the same `sentai.flow.mode("anchor")` API on SIM and ARM,
mirroring the platform-abstraction pattern already used by
`sentai_pxp_shim.h` (PXP HW scale on ARM, pure-C area-average on SIM)
and `sentai_fft_shim.h` (CMSIS-DSP FFT on ARM, FFTW3 on SIM).

## Architecture

```
       ARM (board)                              SIM (Linux host)
   ──────────────────────                  ──────────────────────────────
   OV5640 → CSI ISR → SDRAM                Gazebo /downward_cam/image
        │                                       │
        │ gray80x60                              │  RGB 640×480 over UDS
        ▼                                       ▼
   flow_task.cc                            sim/scripts/aruco_pose_publisher.py
        │                                       │  (cv2.aruco + solvePnP +
   (M7 detector — s112+)                     estimate_drone_world_pose)
        │                                       │
        │ sentai_aruco_pose_t                   │  36-byte UDP-on-UDS pkt
        ▼                                       ▼
   sentai_aruco_shim_arm.cc            sim/sentai_aruco_shim_sim.c
       (today: stub)                  (drains UDS, caches latest pose)
        │                                       │
        └────────────► same C API ◄─────────────┘
                   sentai_aruco_get_latest()
                              │
                              ▼
                  sentai.flow.anchor_pose()  ← MicroPython REPL
                                              (identical dict on both)
```

## What ships in this experiment

| File | Purpose |
|---|---|
| `examples/sentai_runtime/sentai_aruco_shim.h` | C API contract (shared) |
| `examples/sentai_runtime/sentai_aruco_shim_arm.cc` | ARM stub (s112+ fills it in) |
| `sim/sentai_aruco_shim_sim.c` | SIM impl: SOCK_DGRAM UDS reader |
| `sim/scripts/aruco_pose_publisher.py` | Python sidecar: gz cam → cv2.aruco → UDS |
| `examples/sentai_runtime/modsentai_flow.c` | ARM MP bindings (`mode`, `anchor_pose`) |
| `sim/modsentai_sim.c` | SIM MP bindings — same names + dict fields |
| `test_anchor_wire.py` | End-to-end struct.pack wire test (this dir) |

## Wire format

C struct `aruco_wire_t` / Python `struct.pack("<II BB H ffff II", ...)`,
36 bytes total, little-endian:

| Offset | Field | Type |
|-------:|-------|------|
| 0  | magic = 0x41524332 ('ARC2') | uint32 |
| 4  | frame_seq | uint32 |
| 8  | detected (0/1) | uint8 |
| 9  | num_markers | uint8 |
| 10 | pad | uint16 |
| 12 | x_m | float32 |
| 16 | y_m | float32 |
| 20 | z_m | float32 |
| 24 | yaw_rad | float32 |
| 28 | detect_us | uint32 |
| 32 | src_ts_ms | uint32 |

UDS path:
- C shim binds & reads at `/tmp/sentai_aruco_pose_recv.sock`
- Python publisher sendto()s that path

`SOCK_DGRAM`, non-blocking, no framing/length prefix (one datagram = one
pose).  C side drains the queue on every read so the cached snapshot is
always the most recent one.

## How to run end-to-end

### 1. Wire-format smoke (no Gazebo, no OpenCV)

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s112_x86_anchor_shim/test_anchor_wire.py
# expect: "[test] PASS — wire format matches end-to-end"
```

### 2. Real anchor with Gazebo

```bash
# terminal A: start the SIM (Gazebo Garden + x500_sentai world)
make -C ... px4-sitl …

# terminal B: start the SIM firmware
./build-sim/sim/sentai_sim

# terminal C: start the Python publisher (separate gz subscriber)
distrobox enter crazysim-garden -- \
  python3 sim/scripts/aruco_pose_publisher.py -v

# terminal D: drive the SIM REPL
echo 'sentai.flow.mode("anchor"); print(sentai.flow.anchor_pose())' | \
  nc -U /tmp/sentai_sim_repl.sock
```

## Test result (2026-05-12, build #107)

`test_anchor_wire.py` end-to-end: **PASS**.  Sent pose (det=1, n=3,
x=1.25, y=-0.5, z=1.75, seq=42), received identical values in the
`sentai.flow.anchor_pose()` dict.

## What's NOT done yet (s113+)

- **ARM real detector** — `sentai_aruco_shim_arm.cc` is a stub.  The
  building blocks (PXP threshold 1.1 ms, edge 3.0 ms, PXP rectify
  0.04 ms) are shipped in s111; remaining is contour finder + quad
  decode + PnP.  ~5 ms/frame budget.
- **VPE forwarding integration** — when `mode("anchor")` is active,
  the flow path should auto-forward `anchor_pose()` to `sentai.link`
  MAVLink (PX4) or `sentai.crazy` CRTP (cf2) as
  VISION_POSITION_ESTIMATE.  Currently it's pull-only via REPL.
- **Direct cf2 parity** — same API needs verification on cf2 path
  (where pose forwarding goes via `cf.loc.send_external_position`).
- **gz subscription test in distrobox** — `subscribe_gz()` works in
  principle but hasn't been smoke-tested with a live Garden session
  this session.
