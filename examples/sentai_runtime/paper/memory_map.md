# Linker Memory Map Comparison

This note compares the memory organization used by the original repository example `examples/classify_images` with the current `examples/sentai_runtime` runtime.

For the original repository baseline, `classify_images` uses the default M7 application setup from the repository build system and therefore follows the standard linker layout in `libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm7_ram.ld`.

For the current runtime, the reference is the custom linker script `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`.

## Memory regions

The top-level memory regions are intentionally kept almost identical. The main changes are not the physical regions themselves, but which components are placed into OCRAM, SDRAM, and `.sdram_bss`.

| Region | Original repository (`classify_images`) | Current runtime (`sentai_runtime`) | Notes |
| --- | --- | --- | --- |
| `m_interrupts` | `0x00000800`, `0x00000400` | `0x00000800`, `0x00000400` | Vector table and startup region in internal RAM. |
| `m_text` | `0x00000c00`, `0x0003F400` | `0x00000c00`, `0x0003F400` | Main executable text/rodata region in tightly coupled on-chip memory. |
| `m_ncache` | `0x20000000`, `0x00008000` | `0x20000000`, `0x00008000` | Non-cacheable data region for DMA-sensitive data. |
| `m_data` | `0x20008000`, `0x00038000` | `0x20008000`, `0x00038000` | Main initialized data, BSS, and stack region. |
| `m_ocram` | `0x20240000`, `0x00080000` | `0x20240000`, `0x00080000` | Large on-chip RAM used for selected libraries and runtime code. |
| `rpmsg_sh_mem` | `0x202C0000`, `RPMSG_SHMEM_SIZE` | `0x202C0000`, `RPMSG_SHMEM_SIZE` | Shared-memory window for multicore/RPMsg use when enabled. |
| `m_heap` | `0x80000000`, `0x01000000` | `0x80000000`, `0x01000000` | SDRAM heap reservation; default is 16 MB. |
| `m_sdram` | `0x81000000`, `0x01000000` | `0x81000000`, `0x01000000` | SDRAM for large code/data objects after reserving the heap. |
| `m_ncamera` | `0x82000000`, `0x01000000` | `0x82000000`, `0x01000000` | Dedicated non-cacheable camera buffer region. |

## Section placement differences

The practical difference between the two linkers is the section-placement policy.

| Aspect | Original repository (`classify_images`) | Current runtime (`sentai_runtime`) | Why it changed |
| --- | --- | --- | --- |
| TensorFlow placement | `.tensorflow` placed in `m_ocram` | `.tensorflow` placed in `m_sdram` | Frees OCRAM for MicroPython, communication, and runtime services; accepts slightly slower access in exchange for capacity. |
| MicroPython runtime | Not present | Dedicated `.micropython` section in `m_ocram` | Keeps the interpreter and its hot code in on-chip RAM as a first-class runtime component. |
| `libm` for MicroPython | Not separated | Dedicated `.libm` section in `m_ocram` | Co-locates math support with MicroPython; avoids placing it in the main text region. |
| Audio runtime | Not separated | Dedicated `.audio` section in `m_ocram` | Makes onboard audio support explicit in the runtime layout. |
| MP3 encoding | Not present | Dedicated `.shine` section in `m_ocram` | Adds encoder support required by the microphone subsystem. |
| Embedded ML / on-device learning | Not present | Dedicated `.aifes` section in `m_ocram` | Supports the added AIfES-based learning functions. |
| Slow peripheral / telemetry services | Not present | Dedicated `.sentai_slow` section in `m_ocram` | Groups MAVLink, Meshtastic, HTTP, CPU-side TFL bridge, and Crazyflie support away from the main hot path. |
| USB networking | Not present | Dedicated `.cdc_ncm` section in `m_ocram` | Supports the added USB networking mode. |
| Camera service code | Not separated | Dedicated `.camera` section in `m_ocram` | Reflects the much larger camera subsystem in the runtime. |
| LWIP-related services | Standard LWIP placement in OCRAM | Extended LWIP placement including HTTP server support in OCRAM | Matches the richer onboard networking and web tooling. |
| Main `.bss` policy | Generic `.bss` stays in `m_data` | Selected large/runtime-heavy BSS objects are excluded from `m_data` | Reduces pressure on limited on-chip data RAM. |
| `.sdram_bss` contents | Mostly explicit `.sdram_bss*` plus Bluetooth BSS | `.sdram_bss*` plus MicroPython BSS, audio BSS, HTTP server BSS, and Bluetooth BSS | Moves bulky persistent state to SDRAM so the runtime can coexist with camera, inference, and scripting subsystems. |
| Camera DMA buffers | `NonCacheableCamera` region available | Same region, but central to the runtime design | In `sentai_runtime` the non-cacheable camera area becomes a core architectural element rather than a convenience. |
| Heap strategy | Standard SDRAM heap reservation | Same SDRAM heap reservation | The heap region itself is unchanged; the difference is that more long-lived state is kept out of the heap and moved into static sections. |

## Summary

The custom linker in `sentai_runtime` does not primarily change the physical memory map of the RT1176 board. Instead, it changes the allocation strategy:

| Design goal | Original repository (`classify_images`) | Current runtime (`sentai_runtime`) |
| --- | --- | --- |
| Baseline assumption | Single-purpose EdgeTPU demo application | Long-lived embedded runtime with scripting, telemetry, audio, camera switching, tracking, and auxiliary ML modules |
| OCRAM usage | Holds the standard application code and several bundled libraries, including TensorFlow | Reserved more aggressively for the interpreter, runtime services, and latency-sensitive subsystems |
| SDRAM usage | Heap plus some large sections and optional `.sdram_bss` users | Heap plus TensorFlow, tracker/runtime state, and several large BSS groups deliberately migrated from on-chip RAM |
| BSS philosophy | Mostly default placement | Explicit BSS migration to SDRAM to preserve scarce internal RAM |
| Overall effect | Suitable for a compact inference example | Suitable for a multi-service embedded AI runtime with substantially higher memory pressure |

In short, the original linker is organized like a conventional Coral Micro example application, while the `sentai_runtime` linker turns memory placement into an active systems-design tool. The key difference is not new memory hardware, but a more deliberate partitioning of OCRAM, SDRAM, and non-cacheable regions so that MicroPython, dual-camera perception, tracking, communication, and embedded-ML modules can coexist in one firmware image.