# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A fork of `coralmicro` (Coral Dev Board Micro firmware — NXP i.MX RT1176, Cortex-M7 @ 800 MHz + Cortex-M4 + EdgeTPU + dual OV5640 cameras). Upstream is a generic FreeRTOS+TFLite-Micro SDK; this fork's active deliverable is the **SentAI firmware** under `examples/sentai_runtime/`, which adds MicroPython REPL over USB CDC-ACM, an HTTP server over USB CDC-NCM, EdgeTPU pipelines, optical-flow on M7, FileX/LevelX user partition, and a Crazyflie radio bridge over UART2.

## Authoritative project guide — READ THIS FIRST

`examples/sentai_runtime/agent/agent.md` (~2800 lines) is the canonical handoff doc for the SentAI firmware. It supersedes anything generic in this file: load-bearing rules (anti-brick, ISR discipline, error codes), recovery recipes, REPL upload protocol, architecture map, error-code registry, and a full chronology of dead-ends to avoid. Companion docs in the same `agent/` dir: `embeded.md` (NASA/JPL discipline rules), `experiment.md` (chronological session log), `ov5640registers.md`. Long-form lab notes per subsystem live in `examples/sentai_runtime/paper/`.

Persistent cross-session memory built up over prior sessions lives in `/home/bogdan/.claude/projects/-home-bogdan-work-coralmicro/memory/` — `MEMORY.md` is the index, individual `.md` files cover specific decisions, dead-ends, and conventions. The harness auto-loads `MEMORY.md` into context at session start; the per-topic files can be read on demand. Past session transcripts (`.jsonl`) live alongside the memory dir but are not auto-loaded.

## Build & flash

```bash
# Full build (CMake → arm-none-eabi-gcc; toolchain auto-fetched on first configure)
bash build.sh                                          # writes to ./build
bash build.sh -n                                       # ninja generator → ./build-ninja

# Incremental rebuild of just the SentAI firmware
cmake --build build --target sentai_runtime

# Persistent flash (survives power cycle).  When the board is alive on USB
# (lsusb shows 1fc9:c0a1), flashtool auto-resets it into SDP via HID — NO
# manual USER+RESET button press needed.  Only ask the user for a physical
# button if the board is truly bricked (no enum at all and no 1fc9:013d).
python3 scripts/flashtool.py -e sentai_runtime

# RAM-only flash (fast unwedge; lost on power cycle; ALSO lost on sys.reset() —
# NVIC reset returns to ROM bootloader which runs whatever is in persistent flash)
python3 scripts/flashtool.py -e sentai_runtime --ram
```

`apps/CMakeLists.txt` has only `elf_loader` enabled — `mfg_test`, `multicore_model_cascade`, `rack_test`, `usb_drive`, and `my_project` are commented out (`.bss` overflows `m_data` once UMS is on). The vendor `examples/` tree builds independently.

`build.sh -a -s` wraps the Arduino sketch build via `arduino/package.py`; only needed when shipping the Arduino library archive, not for SentAI work.

## SDK patches (load-bearing)

`third_party/nxp/rt1176-sdk` is upstream-clean + 4 patches applied **at CMake configure time** by `scripts/apply_sdk_patches.sh` (idempotent). Patches live under `patches/coralmicro-rt1176-sdk/`:

| Patch | Symptom if missing |
|---|---|
| `0001-ov5640-vga-30fps-pclkperiod.patch` | D-PHY loses sync after frame 1; CSI timeouts |
| `0002-ehci-queue-depth-16.patch` | TPU USB transfer stalls under high pipeline load |
| `0003-fsl-csi-coralmicro-irq-hooks.patch` | flow / pipeline can't tag per-buffer cam_id |
| `0004-lwip-httpd-empty-body-post.patch` | `/api/write` HTTP returns 500 on empty body |

`git status` ALWAYS shows the SDK submodule as dirty (` m third_party/nxp/rt1176-sdk`) — **this is intentional**, the dirty state means patches are live. After any `git submodule update --init --recursive`, re-run `bash scripts/apply_sdk_patches.sh` (or just re-run cmake configure). Bypass with `-DSENTAI_SKIP_SDK_PATCHES=ON` only for CI sanity checks.

The other knob in the top-level `CMakeLists.txt` is `-DSENTAI_TPU_MULTI_EP=ON` to select the experimental multi-endpoint EdgeTPU firmware blob (default OFF = single-EP).

## Architecture map (where to look)

- `apps/elf_loader/` — bootstrap; the only currently-built app under `apps/`.
- `examples/sentai_runtime/` — the SentAI firmware. Top file: `sentai_runtime.cc:app_main`. MicroPython bindings live in `modsentai_*.c` (one per subsystem: `camera`, `tpu`, `flow`, `crazy`, `fs`, `pipeline`, `diag`, …); regen QSTRs after touching them (recipe in `agent/agent.md` §6). Linker script: `MIMXRT1176xxxxx_cm7_ram_mp.ld`.
- `libs/base/` — shared M7 platform: `main_freertos_m7.cc` (boot path), `filesystem.{cc,h}` (`LfsUser*` compat shims), `fx_user_fs.{cc,h}` (typed FileX API — preferred for new C++ code), `http_server.cc`, `ipc*.cc`. New code that writes the user partition should use `FxUser*`, not `LfsUser*` (raw `LfsUser()` accessor returns nullptr post-Phase-2).
- `libs/camera/` — OV5640 driver, CSI ISR (in `camera_support.c`), MUX glue (`cam_mux.h` is the polarity single-source-of-truth). ISR code is `__attribute__((section(".ramfunc")))` and MUST stay in ITCM — verify via `arm-none-eabi-objdump -h <elf> | grep ramfunc`.
- `libs/tpu/` — EdgeTPU driver + DFU + libedgetpu blobs. Apex firmware is `apex_latest_single_ep_bin.c` / `apex_latest_multi_ep_bin.c`.
- `libs/filex/`, `libs/levelx/` — Eclipse ThreadX FileX/LevelX (vendored 2026-04-27, replaces LittleFS on the user partition; system partition stays on LittleFS — see `agent/agent.md` §11 "Filesystem layout").
- `third_party/micropython/` — vendored MicroPython, embedded via the embed port. Embed sources are regenerated under `examples/sentai_runtime/micropython_embed/` by the QSTR regen recipe.
- `third_party/nxp/rt1176-sdk/` — vendor SDK; treat as read-only upstream + the 4 patches above.

## Where experiments live (DO NOT use `/tmp`)

Every host-side test, smoke check, or one-off experiment script goes
under `examples/sentai_runtime/experiments/sNNN_<name>/` (next free
number; latest at time of writing is `s086`). Each session folder is
self-contained: `README.md` (what it proves + how to run + pass
criteria), the script(s) themselves, and any captured outputs (CSVs,
JPEGs, logs).

**Why not `/tmp`**: `/tmp` is wiped on reboot, invisible to git, and
makes prior sessions unreproducible — every run becomes a fresh
discovery. Operator wants experiments to accumulate as durable lab
notes alongside the code, the way `s001`..`s085` already do (see
`examples/sentai_runtime/experiments/README.md` for the convention).
Anything previously living under `/tmp/test_*.py` should be moved into
a new `sNNN_<name>/` folder before being re-run.

For ON-BOARD diag drivers (the `_t_*.py` files pushed to `/lib/diag/`
via the chunked REPL uploader), the convention is `diag/_t_<name>.py`
under `examples/sentai_runtime/`. Host-only helpers live in
`diag/_host_*.py` and are never pushed to the board (the `_host_`
prefix is recognised by the uploader's exclusion rule).

## Working with the board (live HW)

Default boot exposes USB CDC-ACM at `/dev/ttyACM0` (REPL) and CDC-NCM at `10.0.0.1` (HTTP). `lsusb` should show NXP `1fc9:c0a1`; if it shows Google Coral `18d1:9307` the firmware is dead and only a physical button press (USER held during plug-in / reset) recovers it. See `agent/agent.md` §2 (rule 1 — "Never brick the board") and §4 (unwedge recipes) before touching boot/USB code.

Build counter on the board: `sentai.version()` returns `SentAI v1.0 build NNN (...)`. After flashing, verify the build # incremented vs expected — mismatch means stale firmware.

## Environment quirks

- The repo carries multiple Python venvs (`venv/`, `venv-coral/`, `venv_coral/`, `venv_edgetpu/`) for host-side EdgeTPU testing — see `reference_host_coral_testing.md` in user memory if you need to run pycoral locally.
- `setup.sh` installs Linux apt deps + the udev rule from `scripts/99-coral-micro.rules` (needed for non-root `flashtool.py`).
- `error.log` at the repo root is a build artifact and can be ignored.
