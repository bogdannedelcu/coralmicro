# Reverse-engineering `apex_latest_*.bin` — EdgeTPU USB controller firmware

**Date**: 2026-04-24. **Target**: `/home/bogdan/work/libedgetpu/driver/usb/apex_latest_single_ep.bin` (10,783 bytes) and `apex_latest_multi_ep.bin` (12,117 bytes). **Tools used**: custom 8051 disassembler at `scripts/d51.py`.

**Goal of the investigation**: determine whether the TPU firmware or silicon exposes any undocumented mechanism for caching compiled instruction bitstreams across `Invoke()` calls, analogous to how `PARAMETER_CACHING` retains weights.

**Verdict up front**: No. The firmware is a thin USB→DMA dispatcher. The whole software stack — compiler, libedgetpu API, apex firmware — has zero provisions for instruction caching on Beagle (the Coral chip family). The empirical fact that params *are* cached is handled below the API in silicon mask ROM.

---

## 1. File structure

Both blobs are raw 8051 (MCS-51) machine code for the TPU's embedded USB controller core. Entropy ≈ 6.5 bits/byte (normal for code, not compressed).

| Offset | Bytes | Meaning |
|---|---|---|
| `0x0000` | `02 1E 57` | LJMP 0x1E57 — reset vector → main |
| `0x0003` | `02 14 FA` | LJMP 0x14FA — EXT0 interrupt vector |
| `0x0013` | `02 00 16` | LJMP 0x0016 — EXT1 interrupt vector (USB SIE) |
| `0x0016` | `C0 E0 C0 F0 ...` | ISR prologue: PUSH ACC, B, DPH, DPL, PSW, R0..R7 |
| `0x0028` | `02 C0 03` | Vector → 0xC003 (outside file → mask ROM) |
| `0x189A` | `"GUC-Ver:"` + 4 bytes + `:END` | Firmware version string |
| `0x18AB` | `12 01 10 02 ...` | USB descriptor table (see §3) |
| `0x1E57` | `78 7F E4 F6 D8 FD ...` | main entry: clear internal RAM then init |

---

## 2. Architecture — hybrid patch + mask ROM

The 8051 makes many `LCALL` / `LJMP` jumps to addresses **outside** the 10.7 KB blob:

```
LJMP targets outside file (0x2A1F .. 0xFFFF):
  0x3C00, 0x6000, 0x6003 (×4), 0x6056, 0x6800, 0x7014, 0x7071,
  0x7E00, 0x8000, 0x9300 (×2), 0xAF03 (×2), 0xC003 (×2),
  0xC229, 0xD001, 0xE322, 0xE528, 0xE722, ...

LCALL targets outside file:
  0x3029, 0x7E19, 0xE530, 0xE541, 0xE546, 0xF012
```

These are **mask ROM** — factory-programmed code permanently etched in silicon. The `apex.bin` blob DFU-loaded via `edgetpu_dfu_task.cc` is just a **patch / bootstrap** layered over the much larger ROM library.

```
┌──────────────────────────────────────────────┐
│ 8051 core in EdgeTPU                         │
│ ┌────────────────────────────┐               │
│ │ Mask ROM (factory, ~50+ KB)│ ← invisible   │
│ │ 0x6000-0xFFFF approx       │   to us       │
│ └────────────▲───────────────┘               │
│              │ LCALL/LJMP                    │
│ ┌────────────┴───────────────┐               │
│ │ apex.bin (RAM, DFU'd)      │ ← our 10.7KB  │
│ │ 0x0000-0x2A1F              │               │
│ └────────────────────────────┘               │
└──────────────────────────────────────────────┘
```

Consequence: features we can't see in `apex.bin` might still exist in mask ROM — but the patch is the ONLY code we can modify, and it doesn't expose any hidden commands.

---

## 3. USB descriptor table

Parsed from `0x18AB` in `apex_latest_single_ep.bin`:

| Offset | Descriptor | Key fields |
|---|---|---|
| `+0` | DEVICE | USB 2.10, Vendor=`0x18D1` (Google), Product=`0x9302`, Class=per-interface |
| `+0x12` | DEVICE_QUALIFIER | USB 2.10 alternate-speed info |
| `+0x1C` | CONFIGURATION | wTotalLength=60, bmAttributes=0xC0 (self-powered), bMaxPower=98 mA |
| `+0x25` | OTHER_SPEED_CONFIG | Same as config |
| `+0x2E` | INTERFACE | 6 endpoints, class/subclass/protocol = `0xFF/0xFF/0xFF` (vendor-specific) |
| `+0x37` | ENDPOINT | `0x01` **bulk OUT** EP1, max 512 |
| `+0x3E` | ENDPOINT | `0x02` **bulk OUT** EP2, max 512 |
| `+0x45` | ENDPOINT | `0x03` **bulk OUT** EP3, max 512 |
| `+0x4C` | ENDPOINT | `0x81` **bulk IN** EP1, max 512 |
| `+0x53` | ENDPOINT | `0x82` **bulk IN** EP2, max 512 |
| `+0x5A` | ENDPOINT | `0x83` **interrupt IN** EP3, max 64 |
| `+0x61` | STRING | langID=0x0409 (en-US) |
| `+0x65` | DEVICE | USB 3.10 descriptor (fallback for SuperSpeed hosts) |

**Surprising**: both "single_ep" and "multi_ep" firmwares advertise **3 bulk-OUT endpoints**. The "single vs multi" naming refers to **routing behaviour**, not endpoint count: a `multi_bo_ep=1` CSR toggle (from libedgetpu) selects per-tag routing.

### GUC-Ver date codes

| Firmware | Bytes between `GUC-Ver:` and `:END` | Interpretation |
|---|---|---|
| `single_ep` | `18 09 10 05` | 2018-09-10, build 5 |
| `multi_ep` | `18 09 10 03` | 2018-09-10, build 3 |

Same date, different minor builds. Both old (2018).

---

## 4. DMA command register at XRAM 0x801C

The single most-referenced XRAM address (42 accesses) in the firmware. It is a **4-byte DMA command** where the first byte is the opcode:

```c
// canonical issue pattern, repeated for each of 7 opcodes
MOV DPTR, #0x801C
MOV A, #cmd          // 0x01, 0x02, 0x03, 0x80, 0x81, 0x82, 0x83
MOVX @DPTR, A        // byte 0 = opcode
INC DPTR; CLR A
MOVX @DPTR, A        // byte 1 = 0
INC DPTR
MOVX @DPTR, A        // byte 2 = 0
INC DPTR
MOVX @DPTR, A        // byte 3 = 0
```

### Command code mapping

| Code | Bit 7 (dir) | Bits 0-6 (endpoint) | Meaning |
|---|---|---|---|
| `0x01` | 0 (OUT) | 1 | Bulk-OUT EP1 transfer |
| `0x02` | 0 | 2 | Bulk-OUT EP2 |
| `0x03` | 0 | 3 | Bulk-OUT EP3 |
| `0x80` | 1 (IN) | 0 | Broadcast / interrupt / housekeeping (5× uses, distinctive) |
| `0x81` | 1 | 1 | Bulk-IN EP1 |
| `0x82` | 1 | 2 | Bulk-IN EP2 |
| `0x83` | 1 | 3 | Interrupt-IN EP3 |

**These 7 codes are the entire DMA command set**. Both `single_ep` and `multi_ep` firmwares use the identical set. No hidden `cache-hit`, `replay-previous`, or `ins-cache` opcode.

Bytes 1-3 of the 4-byte command are **always zero** in the firmware. The hardware at the other end might accept non-zero flags, but without custom firmware we can't test this. (Reserved for potential future exploit path.)

---

## 5. XRAM address map inferred from accesses

Unique addresses touched by the firmware (65 total):

| Page | Range | Count | Likely meaning |
|---|---|---|---|
| `0x00XX` | `0x0001-0x0020` | 16 | CPU/MCU control registers (port, flags) |
| `0x01XX` | `0x01A0-0x01C0` | 11 | USB endpoint FIFO / status registers (one set of 8 per EP) |
| `0x12XX-0x19XX` | sparse | 7 | Code ROM (MOVC lookup tables) |
| `0x29XX` | `0x2910, 0x29DD` | 2 | Embedded constant table (bit-counts / masks) |
| `0x80XX` | `0x8000-0x8042` | 15 | **DMA engine + USB SIE registers** (0x801C = command, 0x8040 = status) |
| `0x90XX` | `0x9004-0x9050` | 11 | Hardware debug/test registers |
| `0xA3XX, 0xFDXX` | sparse | 3 | PMIC / clock / misc |

Nothing that looks like a multi-MB buffer the 8051 could use as an instruction cache. The 8051's XRAM address space touches only control/status registers, not bulk memory.

---

## 6. Instruction frequency in `apex_latest_single_ep.bin`

Top 10 opcodes encountered by linear disasm:

| Mnemonic | Count |
|---|---|
| `MOVX @DPTR,A` | 603 |
| `INC DPTR` | 515 |
| `LCALL` | 344 |
| `CLR A` | 342 |
| `MOV DPTR,#imm16` | 333 |
| `MOV bit,C` | 276 |
| `MOV C,bit` | 274 |
| `MOV dir,A` | 183 |
| `MOVX A,@DPTR` | 171 |
| `CLR bit` | 162 |

Dominant pattern: write-XDATA-register, bit-manipulation, function call. Typical for a USB controller firmware that does lots of register-level housekeeping, not a data-processing firmware.

---

## 7. Conclusive evidence from libedgetpu source

Cross-checking the firmware analysis against the public libedgetpu source:

### 7.1 Only two TPU request types
[`api/request.h:41-44`](../../../../work/libedgetpu/api/request.h):
```cpp
enum class TpuRequestType {
    PARAMETER_CACHING,  // Request for parameter caching.
    INFERENCE           // Inference request, single hardware batch.
};
```
No `INSTRUCTION_CACHING`, `CACHED_INFERENCE`, etc.

### 7.2 Only three executable types
[`executable/executable.fbs:348-361`](../../../../work/libedgetpu/executable/executable.fbs):
```
enum ExecutableType : short {
  STAND_ALONE = 0,
  PARAMETER_CACHING = 1,
  EXECUTION_ONLY = 2,
}
```
The compiler can't emit an "INSTRUCTION_CACHING" executable because the schema doesn't have the type.

### 7.3 Beagle has a NullDramAllocator
[`beagle_usb_driver_provider.cc:344`](../../../../work/libedgetpu/driver/beagle/beagle_usb_driver_provider.cc):
```cpp
auto dram_allocator = gtl::MakeUnique<NullDramAllocator>();
```
[`null_dram_allocator.h`](../../../../work/libedgetpu/driver/memory/null_dram_allocator.h):
```cpp
// A DRAM allocator to be used for chips that do not have an on-chip DRAM.
StatusOr<shared_ptr<DramBuffer>> AllocateBuffer(size_t) override {
    return FailedPreconditionError("No on-chip DRAM available.");
}
```

Per-layer `cache_on_dram:bool` and per-executable `use_tpu_dram_for_parameters:bool` both **fail** on Coral Beagle. Those API paths exist for other Darwinn chips that have TPU DRAM; Beagle doesn't.

### 7.4 How params caching actually works on Beagle
Empirically it works (weights are not re-streamed). But via code inspection of [`package_registry.cc:664`](../../../../work/libedgetpu/driver/package_registry.cc):
```cpp
Status ExecutableReference::PrepareParameters() {
  if (!parameters_.IsDramType() || parameters_loaded_) {
    return OkStatus();   // ← no-op on Beagle (params are not DramType)
  }
  ...
}
```

So the driver layer does **nothing** to map/cache params on Beagle. The actual caching is handled **transparently by mask ROM + silicon DMA**: the `PARAMETER_CACHING` executable contains compiled TPU instructions that say "DMA 1.9 MB from host USB FIFO to on-chip parameter SRAM region X". After it runs once, the weights stay in that region (on-chip SRAM, somewhere in the ~8 MB pool estimated from datasheet + empirical benchmarks) until the next `PARAMETER_CACHING` invocation for a different model.

### 7.5 No vendor-specific USB control commands
`libusb_control_transfer` is called exactly 3 times in the entire libedgetpu codebase — all in `local_usb_device.cc` wrappers used only for standard USB and DFU class requests. The Darwinn ML protocol is 100% over bulk endpoints with tag-prefixed headers. No hidden backdoor control commands.

---

## 8. Why the hardware likely doesn't support instruction caching

Speculation grounded in what we can observe:

1. **Scalar-core instruction fetch is a streaming FIFO.** `instruction_queue_size` CSR at [`beagle_csr_offsets.h:821`](../../../../work/libedgetpu/driver/config/beagle/beagle_csr_offsets.h) is just a queue depth (256 entries). No "restart" or "rewind" CSR. Streaming instruction decoders typically don't have per-instruction cache tags because their ISA is optimised for throughput, not reuse.
2. **Params live in a random-access on-chip SRAM** (`192 KB narrow_memory` reported by `beagle_chip_structures.h` is clearly an undercount; the datasheet's 400-FPS MobileNet v2 implies multi-MB). That memory is addressable by MOVI+offset patterns encoded in the compiled bitstream.
3. **Google's compiler would use it if it existed.** The compiler produces `PARAMETER_CACHING` executables for params. It does NOT produce "INSTRUCTION_CACHING" executables. Strongest evidence that the chip has no facility for it.
4. **The asymmetry is architecturally sensible.** Params are read many times by tiles in a single inference (read-heavy random access); instructions are consumed once per invoke by the scalar core (sequential stream). Different access patterns → different memory tiers → only params get dedicated cache storage.

---

## 9. Remaining theoretical exploit paths (not attempted)

Ranked by feasibility and likely reward:

| # | Path | Feasibility | Expected reward |
|---|---|---|---|
| 1 | Pre-stage ins bytes in RT1176 OCRAM at load time (avoid SEMC) | ✅ trivial | reduces pipeline contention; no effect on pure invoke |
| 2 | Write custom 8051 firmware that probes undocumented DMA opcodes (0x04, 0x05, `CMD 01 00 00` etc.) | Hard (1-2 days: assemble, DFU, observe) | Uncertain — probably no secret cache exists |
| 3 | Decap the EdgeTPU ASIC and extract mask ROM | Infeasible (requires physical chip + microscopy lab) | Definitive answer, but enormous effort |
| 4 | Recompile the model with `batch_size > 1` | Medium (need edgetpu_compiler access) | Amortises ins across N inferences — helps throughput, hurts latency |
| 5 | Contact Google for undocumented APIs | Unlikely to yield results (product discontinued for new use-cases) | — |

**Path 1 is the only one that ships a concrete measurable win without further reverse-engineering.**

---

## 10. Tools left behind

- `scripts/d51.py` — self-contained 8051 disassembler (~10 KB, no dependencies). Usage: `python3 d51.py firmware.bin [--start 0x1E57] [--end 0x1F00]`
- This document (`paper/apex_fw.md`) — starting point if someone wants to pick up the thread
