# USB Mode-Switch Composite: Serial + IP / Serial + Mass Storage

## Goal

The SentAI board exposes a host-visible USB composite that can operate in
**two mutually-exclusive modes**, switched at boot via a persistent flag in
DTC-RAM that survives `NVIC_SystemReset` but is cleared by hardware POR:

| Mode | Interfaces | Trigger to enter | Trigger to exit |
|------|------------|------------------|-----------------|
| **Default (REPL+IP)** | `/dev/ttyACM0` (CDC-ACM REPL) + `enx…` (CDC-NCM Ethernet) | cold boot, hardware reset button, `'q'` byte sent on `/dev/ttyACM0` while in storage mode | — |
| **Storage** | `/dev/ttyACM0` (CDC-ACM, anti-brick — REPL is off) + `/dev/sda` (USB MSC, the LittleFS user partition as a raw 56 MB block device) | `sentai.usb.drive(1)` from REPL or pressing the user button | hardware reset button, `'q'` byte on `/dev/ttyACM0`, or `sentai.usb.drive(0)` from a host script that drives the byte itself |

The earlier ambition of running **all three** classes (ACM + NCM + MSC) in a
single composite descriptor is documented below for archaeological purposes.
That layout was abandoned: Linux's `usb-storage` and `cdc_ncm` race during
enumeration (cascade BOT-reset → bus reset at T+1 s) and the resulting
recovery storms make the board unstable. The mode-switch design eliminates
the conflict by ensuring `usb-storage` and `cdc_ncm` are never present in
the same descriptor.

## Mode-switch architecture (current)

### Persistence: DTC-RAM `noinit` struct

The boot-mode magic lives in a NOLOAD section carved into `m_data`
(DTC-RAM at `0x20000000`):

```ld
.noinit_boot_persist (NOLOAD) : ALIGN(4)
{
  __BOOT_PERSIST_START__ = .;
  *(.noinit.$boot_persist*)
  . = ALIGN(4);
  __BOOT_PERSIST_END__ = .;
} > m_data
```

Placed **before `.bss`** so the startup BSS-zero loop skips it. The struct is

```c
struct BootPersist {
  uint32_t magic;          // SENTAI_STORAGE_MAGIC = 0x57500001 ('SP01')
  uint32_t check;          // ~magic — guards against single-bit POR garbage
  uint32_t attempts;       // crash-loop counter (storage-mode boots)
  uint32_t progress;       // last-reached boot checkpoint (diagnostic)
  uint32_t prev_progress;  // snapshot of `progress` at start of THIS boot
};
```

DTC-RAM contents are preserved by `NVIC_SystemReset` and by watchdog/lockup
resets on RT1176, but cleared by a hardware POR (reset button, power
cycle). That gives us exactly the semantics we want:

* `drive(1)` writes the magic and warm-resets → next boot enters storage mode.
* `drive(0)` writes magic=0 and warm-resets → next boot enters default mode.
* Reset button → POR → DTC-RAM zeroed → default mode (anti-brick).

The empirical history that led to this design (and why SRC GPRs were
rejected as the primary mechanism) is in [Why DTC-RAM not SRC_GPR](#why-dtc-ram-not-src_gpr).

### Crash-loop guard

`attempts` is incremented at every storage-mode boot. After `kMaxStorageAttempts`
(=3) consecutive boots without reaching the safe checkpoint
`sentai_storage_boot_succeeded()`, `real_main` wipes the magic and forces
default mode, so a buggy storage-mode init can never brick the board across
power cycles.

### In-band exit

The storage-mode short-circuit in `sentai_runtime.cc::app_main` polls the
USB CDC-ACM RX buffer at 200 ms cadence. **Any** received byte triggers
`sentai_usb_drive_set(0)` → warm reset → default mode. This lets host
scripts return to default REPL+IP without physical access:

```bash
printf 'q\r\n' > /dev/ttyACM0   # board re-enumerates within ~6 s in default mode
```

### Watchdog: storage-mode bypass

The combined HTTP+REPL watchdog (`CombinedWatchdogTask`, see
[`watchdog.md`](watchdog.md)) monitors `g_http_last_activity` and
`g_repl_last_activity`.  In storage mode both interfaces are off **by
design**, so neither timestamp is ever updated and the naive idle
accounting would declare the board dead after `kDeadThresholdMs` (120 s)
+ WDOG1 timeout (30 s) ≈ 150 s into the storage session — exactly when
the user is busiest copying files.  Build ≤ #586 had this bug; users
saw their `/dev/sda` disconnect mid-transfer without any crash.

From build #587 onward the watchdog task checks
`sentai_storage_mode_active()` at the top of each tick and, while in
storage mode, only refreshes WDOG1 without running the dead-check.
CPU-lockup protection is preserved (if the task stops running entirely,
WDOG1 still fires in 30 s); the auto-reset on "no HTTP/REPL activity"
is simply not applied to a mode where no activity is possible.

Users exit storage mode only on their own initiative: `drive(0)` from a
host script, any byte on `/dev/ttyACM0`, user button, or hardware
RESET.  4+ minute storage sessions are now stable.

### Anti-brick: ConsoleM7's CDC-ACM is always present

In **both** modes, `ConsoleM7::Init()` registers a CDC-ACM interface (the
"anti-brick anchor"). This guarantees that `flashtool.py` can always find
the board on its NXP VID/PID and reflash it, even if every other subsystem
is broken. Storage mode also reuses this same `/dev/ttyACM0` for the
in-band exit listener — there is no second ACM interface.

### Diagnostic: boot-mode log line

On every default-mode boot, `app_main` logs a single line that captures
the full persistence state at boot time:

```
[boot-mode] storage=0 attempts=0 prev_progress=0x14
            sram=00000000/FFFFFFFF
            GPR9=00000000 GPR10=00000000 GPR11=00000000
            GPR12=00000000 GPR15=00000000 GPR16=00000000
```

* `storage` — current mode decision
* `attempts` — storage-mode boot attempts so far (for crash-loop debugging)
* `prev_progress` — last checkpoint reached by the previous boot (see
  `sentai_boot_progress_mark` codes in `main_freertos_m7.cc`)
* `sram` — magic / check pair from DTC-RAM (snapshot at boot, before clear)
* `GPR9..16` — six SRC_GPR registers used as redundant diagnostic
  persistence (none of them survived warm reset reliably on this silicon —
  see [Why DTC-RAM not SRC_GPR](#why-dtc-ram-not-src_gpr) below)

In recovery mode (boot-attempt counter ≥ 3 in `app_main`), the same data
is logged in the `[recovery]` line so post-mortem is possible after a hard
boot loop.

### Verified behaviour (build #582, 2026-04-19)

End-to-end test sequence, all green:

1. Cold boot → default mode, REPL + HTTP active.
2. Write file `/test_marker.txt = "hello sentai 2026-04-19 cycle1"` via REPL.
3. `sentai.usb.drive(1)` → board re-enumerates as ACM+MSC in <6 s.
4. Host sees `/dev/sda` (56 MB raw block device, LittleFS magic visible
   at offset 0x08 via `dd | xxd | head`).
5. `printf 'q\r\n' > /dev/ttyACM0` → board re-enumerates as ACM+NCM,
   `/dev/sda` gone, network up, HTTP responding.
6. `sentai.fs.read("/test_marker.txt")` returns the original 30 bytes —
   the LittleFS user partition is byte-preserved across the cycle.
7. Three back-to-back drive(1)/exit cycles all complete cleanly with
   `/dev/ttyACM0` always present (anti-brick honoured).

### Root cause of an earlier lockup loop

Initial implementations crashed (M7 LOCKUP) on the first storage-mode
boot. Diagnostic checkpoints in `g_boot_persist.progress` traced the
crash to `boot_log_fs_init()` calling `lfs_mkdir` on an uninitialised
`g_lfs_user`. Storage mode skips `LfsUserInit` (so the host has exclusive
NAND access), but `LfsUser()` still returned `&g_lfs_user`. The fix
(in `libs/base/filesystem.cc`) is a `g_lfs_user_initialized` flag —
`LfsUser()` returns `nullptr` until `LfsUserInit` actually mounts the
volume, and every existing caller already null-checks.

### Why DTC-RAM not SRC_GPR

The first attempt at the persistence flag used `SRC_GPR15`, then
`SRC_GPR9..12, 15, 16` simultaneously as a diagnostic. Empirically, on
this silicon, none of GPR9–16 reliably survived `NVIC_SystemReset`
within our boot path: `[boot-mode]` consistently showed
`GPR9=00000000 ... GPR16=00000000` after every warm reset, even
though the reference manual claims those registers are warm-reset
persistent. (`SRC_GPR1, 13, 14` *do* survive — they are used for
boot-attempt and watchdog/lockup counters in `sentai_runtime.cc` and
`reset.cc` respectively — but those slots are already taken.)

DTC-RAM (`m_data` at `0x20000000`) is non-cached on the M7 and its
contents survive every reset class except hardware POR. A NOLOAD
section placed before `.bss` is therefore both reliable and zero-cost
(the BSS-clear loop skips it; the C-startup data-init loop skips it).

The diagnostic SRC_GPR writes are still emitted every boot so any
future investigation has a side-by-side comparison between the two
mechanisms.

---

## Earlier "all three classes simultaneously" attempt (archived)

> The text below predates the mode-switch design. It described the
> attempt to expose CDC-ACM + CDC-NCM + USB MSC in a single composite
> descriptor. That layout is *not* what the firmware ships — it is kept
> here for context on why the cascade resets happen and what the
> three Linux drivers expect from each other.

---

## Hardware and stack

* **MCU**: NXP i.MX RT1176, Cortex-M7 core at 800 MHz.
* **USB controller**: USBPHY1 + USB1 EHCI (high-speed device, 480 Mb/s).
* **Device stack**: NXP MCUXpresso `usb_device_*` middleware, FreeRTOS task
  context (`kUsbDeviceTaskPriority`).
* **Host (target)**: Linux ≥ 5.x with `cdc_acm`, `cdc_ncm`, `usb_storage`,
  `xhci_hcd`. udev / NetworkManager / systemd-udevd active.

The composite descriptor is built at boot in
`libs/base/main_freertos_m7.cc::real_main()`. Each function class is
initialised by appending a `usb_device_class_config_struct_t` to the
`UsbDeviceTask` singleton. Interface and endpoint numbers are allocated
incrementally by `next_interface_value()` / `next_descriptor_value()`, in the
order the classes are added.

---

## Boot-time order

The classes are added in this order:

```c
ConsoleM7::Init()    // CDC-ACM      → interfaces 0, 1
InitializeCDCNCM()   // CDC-NCM      → interfaces 2, 3
InitializeUMS()      // MSC          → interface  4
```

The natural-looking optimisation — putting MSC at iface 2 (in front of NCM)
to make the Linux `usb-storage` probe finish before `cdc_ncm` enters its
sensitive bind window — turned out to break the **firmware-side DHCP
server** in subtle ways: the host's `cdc_ncm` interface comes up but never
gets an IPv4 lease (the firmware's DHCP responses do not reach the host).
REPL and link-state still work, but ping/HTTP both time out.

The root cause is not isolated yet — most likely an interaction between
`netif_default` selection in the lwIP startup and the order in which the
USB device task brings the bulk endpoints online — but rather than ship a
pretty descriptor that breaks IP, we keep HEAD ordering and pay the
cdc_ncm cascade once on first plug.

**Take-away on this stack**: the temptation to "fix" the descriptor by
changing init order has a high regression risk. Validate ping/HTTP after
every USB ordering change, not just enumeration / REPL.

---

## CDC-ACM serial — the "always-on" anchor

`ConsoleM7` (in `libs/base/console_m7.cc`) provides a CDC-ACM device at
interface 0 (control) / interface 1 (data). It is the **first** USB function
initialised, and it is the foundation of the anti-brick strategy:

> The CDC-ACM endpoints are armed by `UsbDeviceTask::Init()` *before*
> `vTaskStartScheduler()` runs. The host therefore sees the NXP USB ID
> (`1fc9:c0a1`) immediately after the chip leaves reset, regardless of
> whether `app_main()` later crashes. That visibility is what allows
> `flashtool.py` to recover the board over USB without pressing the
> hardware boot button.

Notes on the implementation:

* `console_m7.cc` patches the stock NXP CDC-ACM data stream so that any
  `printf`/`logf` from FreeRTOS is forwarded to the bulk-IN endpoint as well
  as to the boot-log RAM ring buffer.
* The hardware watchdog (WDOG1, 30 s) is configured early. If a runtime
  fault hangs FreeRTOS *after* USB-CDC has come up, the watchdog resets the
  CPU; the board re-enumerates with the same NXP ID and remains reflashable.

The only failure mode that defeats the anti-brick guarantee is a crash in
`real_main()` *before* `UsbDeviceTask::Init()` — most realistically a CHECK
failure in `LfsInit()`/`LfsUserInit()` because of a dead NAND. That requires
JTAG recovery; software cannot mitigate it.

---

## CDC-NCM IP networking

The `CdcNcm` class (`libs/cdc_ncm/`) implements the CDC Network Control Model
specification on top of the NXP CDC-ACM transport layer (NCM is "CDC + NTB
framing"). It exposes a 100 Mb/s virtual Ethernet device to the host:

* **Comm interface**: control + interrupt-IN endpoint, used to deliver
  `NETWORK_CONNECTION` and `CONNECTION_SPEED_CHANGE` notifications.
* **Data interface**: 2 alternate settings.
  * `alt=0` declares no endpoints (NCM convention — a quiesced state).
  * `alt=1` declares two bulk endpoints (IN/OUT) which carry NTB frames.

### Endpoint timing

The interrupt endpoint declares `bInterval = 32`.  This value is **invalid
for a high-speed interrupt endpoint** (the spec maximum is 16); Linux
auto-corrects to 9 and prints a one-line warning per enumeration.

Setting `bInterval = 9` directly in the descriptor (the obviously-correct
fix) silently breaks `/dev/ttyACM0`: the host enumerates ACM successfully
and `cdc_acm` binds, but the bulk-IN endpoint never delivers any data and
the REPL appears dead.  No bus resets, no kernel error, just zero bytes.
The root cause is not yet understood — most likely an interaction between
the changed descriptor layout and the patched NXP CDC-ACM TX path that we
inherited.  The Linux warning is harmless and `bInterval = 32` is left in
place until the underlying issue is diagnosed.

> **Engineering rule of thumb:** every USB descriptor change on this stack
> needs an explicit REPL-output regression test.  A descriptor that the
> Linux kernel "fixes up" silently can be safer than a textually-correct
> one whose host-side handling exposes a firmware bug.

### NTB parameters

The driver advertises `NTB_INPUT_SIZE = 16384` and `NTB-16 only` (no
NTB-32) via `GET_NTB_PARAMETERS`. The full 28-byte
`USB_CDC_NCM_NTB_PARAMETERS_LENGTH` structure is returned — the Linux
`cdc_ncm_init()` requires at least this length, otherwise it returns `-EIO`
and unbinds.

### Interaction with lwIP

A 16 KB TX/RX NTB pair sits in `.bss` and is shared with lwIP via
`netifapi_netif_add()`. The TX path aggregates up to 10 datagrams per
NTB to keep USB transactions large; the RX path parses NDP16 records and
delivers each datagram into `tcpip_input()`. A FreeRTOS queue
(`tx_queue_`, depth 10) buffers outbound packets between lwIP and the
USB sender to avoid blocking the lwIP thread on USB busy.

### DHCP server

After the netif comes up, `start_dhcp_server(usb_ip)` (in `dhcpserver.cc`)
publishes a single lease for the host. The board owns `10.0.0.1`
(configurable through `GetUsbIpAddress()`); the host receives `10.0.0.2`.
This is what lets `ssh root@10.0.0.1` or `curl http://10.0.0.1/` work
without any manual host-side IP configuration.

### Notifications

`SetConfiguration` triggers `SendConnectionNotification()` (carrier UP).
Sending it *here* (and not later in `SetInterface(alt=1)`) is required:
the Linux `cdc_ncm` driver starts polling the interrupt endpoint
immediately after enumeration and times out the interface if it does not
see `NETWORK_CONNECTION` quickly. Late notifications produced the same
9 ms register/unregister loop described above. `SPEED_CHANGE` then
follows in `SetInterface(alt=1)`.

---

## USB Mass Storage — exposing the user LittleFS

The `MscUms` class (`libs/msc_ums/`) implements the Bulk-Only Transport
profile (USB MSC subclass `06`, protocol `50`). It maps SCSI READ/WRITE
commands directly to NAND page operations on the *user* partition
(`kUserBaseBlock = 76`, `kUserBlockCount = 448`, page size 2048 → 56 MiB).

### Why direct NAND, not file-backed

The host sees the LittleFS image **as raw blocks**, not as files. On the
host, `littlefs-fuse` then mounts that block device as a filesystem. This
keeps the firmware path trivial — there is no SCSI-to-LFS translation
layer running on the MCU — and it lets the host use any LittleFS-aware
tooling.

### `SetUnitReady` and the UNIT_ATTENTION trap

The boot path calls `g_msc_ums.SetUnitReady(true)` so that `/dev/sda`
appears automatically the first time the cable is plugged. Earlier
versions of `SetUnitReady()` *also* armed `media_changed_=true`, intending
to comply with the SCSI rule that requires `UNIT_ATTENTION` (sense key
`0x06 / 0x28`) on the first not-ready→ready transition. In practice, on
the NXP MSC class layer this caused:

```
TUR  → kStatus_USB_Error (UNIT ATTENTION)
       → NXP class layer translates this into a bulk-IN STALL on the
         transport endpoint
       → usb-storage runs Bulk-Only Reset → Clear-Halt → port reset
       → USB bus reset cascade
```

The fix is to drop the auto-arm: `SetUnitReady()` now only flips the
ready flag, and the first TUR returns `GOOD` directly. Linux skips the
sense-recovery path entirely and binds the SCSI target on the first try.

### Write protection

`write_protected_` defaults to `true`. When set, `MODE SENSE (6)` returns
the WP bit; `WRITE(10)` is rejected with `kStatus_USB_InvalidRequest`. This
is the boot state — the host can read the user partition but cannot
modify it. To make the disk writable, the user calls
`sentai.usb.drive(1)` from the REPL.

### `sentai.usb.drive(1)` / `drive(0)`

This API is the only safe way to grant the host exclusive write access:

```python
sentai.usb.drive(1)   # firmware-side: drain lfs_task → unmount LFS →
                      #                clear write-protect → mark ready
sentai.usb.drive(0)   # firmware-side: mark not-ready → set write-protect
                      #                → remount LFS
```

The sequencing is critical:

1. **`drive(1)`** drains the `lfs_task` slot cache via
   `sentai_lfs_before_drive_change()`, takes the LFS mutex, calls
   `lfs_unmount()`, then *only after the firmware no longer touches NAND*
   flips `SetWriteProtect(false)` and `SetUnitReady(true)`. The host's
   subsequent SCSI WRITE commands have exclusive ownership of the NAND
   pages backing the user partition. IP and serial are not interrupted.
2. **`drive(0)`** calls `SetUnitReady(false)` *first*, so the host stops
   issuing further SCSI READs; only then is LFS remounted. The order
   prevents a race where a late SCSI READ would hit NAND while LFS is
   taking the mutex.

There is no USB bus reset on either transition. The host is informed of
the medium-attribute change via `MODE SENSE` only (the WP bit flips on
the next host poll). This was the second fix that eliminated cascade
resets — the original design issued `USB_DeviceStop()` /
`USB_DeviceRun()` to force a bus reset, which dropped IP and serial
along with MSC.

### LittleFS coexistence (read-only mode)

While `write_protected_=true` (the default), both the firmware's `lfs_task`
and the MSC handler can access NAND. They never run truly concurrently —
single-core FreeRTOS serialises tasks and the NAND driver is invoked from
task context — but two safety nets are in place:

* The MSC `Read` handler retries `Nand_Flash_Read_Page` up to 3 times
  before giving up and filling the buffer with `0xdeadbeef`.
* `lfs_task` flushes its slot cache before any drive transition
  (`sentai_lfs_before_drive_change()`).

---

## Suspicious paths that we *intentionally* did not change

A number of patches that look obvious in the abstract turned out to make
the cascade *worse* and were reverted. They are documented here so they
are not re-attempted:

* **Filtering `SET_INTERFACE` by interface and alt setting in NCM**
  (`if (iface != data_iface_ || alt != 1U) break;`): the parameter
  encoding from the patched NXP CH9 layer is not what the documentation
  suggests in all configurations; the filter never matched and the bulk
  OUT endpoint was never primed, so Linux saw a silent device and
  unregistered in a loop. The current code reacts to *any* `SET_INTERFACE`
  event and arms the bulk endpoint unconditionally. Spurious arms on
  disabled endpoints are silently dropped by the NXP stack.
* **`attached_=false` on `SET_INTERFACE(alt=0)`** combined with a
  re-prime guard in `RecvResponse`: the same root cause — `attached_`
  never went back to `true` because the alt=1 event was missed on some
  Linux kernel versions, and the bulk endpoint sat unprimed forever.
* **Sending `NETWORK_CONNECTION` only at `SET_INTERFACE(alt=1)`**: too
  late for the Linux `cdc_ncm` driver, which has already started a
  carrier-detect timer at the end of `bind_common()`.
* **Adding explicit `GET_NTB_FORMAT` / `GET_NTB_INPUT_SIZE` responders**:
  the patched NXP CDC-ACM layer does not always invoke our callback for
  these requests; setting `acm_param->buffer` in our handler had no
  effect, while the catch-all `kStatus_USB_Success` for `event >= 0x100`
  is sufficient and matches what Linux actually expects (a short or
  empty reply is graceful in `cdc_ncm_set_ntb_input_size()`).

The minimal patch that survives is just two changes from upstream:
`bInterval=9` (valid descriptor) and `SetUnitReady` no longer auto-arms
UA.

---

## Host-side notes

`/dev/sda` is recognised as `Direct-Access SENTAI FLASH STORAGE 0001`
(SCSI INQUIRY page 0). The SCSI removable bit is set so `udisks` /
`gnome-disk-utility` treat it like a USB stick.

Mount with the LittleFS-FUSE driver:

```bash
sudo littlefs-fuse \
  --block_size=131072 --read_size=2048 --prog_size=2048 \
  --block_count=448 --cache_size=2048 --lookahead_size=2048 \
  -o allow_other /dev/sda /mnt/coral
```

The numeric parameters must match the firmware's `g_lfs_user_config`
exactly — `kPageSize = 2048`, `block_size = 131072`, `kUserBlockCount =
448`. Mismatched values produce silent corruption; LittleFS does not
detect them at mount time.

For unprivileged user access (no `sudo`):

```bash
sudo usermod -aG disk bogdan       # allow read of /dev/sda
sudo tee /etc/udev/rules.d/98-sentai-sda.rules <<'EOF'
SUBSYSTEM=="block", KERNEL=="sda", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="c0a1", OWNER="bogdan", GROUP="bogdan", MODE="0660"
EOF
sudo sed -i 's/^#user_allow_other/user_allow_other/' /etc/fuse.conf
sudo udevadm control --reload-rules
```

After this `bogdan` can run `littlefs-fuse` directly.

---

## Diagnostics and what is still missing

* **Boot log** in `/log/boot.log` (RAM ring + LFS-backed file). Captures
  every `printf` from board init until the REPL takes over.
* **Crash log** in `/log/crash_NNN.log` rotated up to 10 files; written
  from `crash_log_write()` in `sentai_runtime.cc` and from the
  HardFault/BusFault/MemManage handlers via `sentai_fault_save()` (SRC
  GPR breadcrumbs).
* **Health/diag REPL** — `sentai.diag.health()`, `sentai.diag.sys_mode()`,
  `sentai.diag.crash_log()`, `sentai.diag.boot_log()`.
* **Anti-brick** — `boot_attempts` in `SRC_GPR1` survives warm reset; on
  the third consecutive crash before REPL the board enters
  `RECOVERY_MODE` (REPL only, no application code, USB always up).

What is **not yet** implemented and is on the short list:

> **In-RAM `dmesg`-like ring buffer.** A small SDRAM ring (≈16 KB) that
> captures every error report (`SERR_LOG(...)`, USB / IP command
> failures, transport stalls, NAND retries) with a millisecond
> timestamp and a level. Exposed via REPL as
> `sentai.diag.dmesg([level])`. Persists across REPL/HTTP commands,
> wiped only by reboot — the equivalent of Linux's `dmesg` for fault
> isolation when the host cannot reach the board over USB or IP. This
> is the natural next step now that the three-interface composite is
> stable: any future regression in MSC/NCM should leave a breadcrumb
> in this buffer rather than a silent failure.

---

## Summary of the fixes that made it work

| File | Change | Reason |
|------|--------|--------|
| `libs/base/main_freertos_m7.cc` | `InitializeUMS()` before `InitializeCDCNCM()` | MSC at iface 2 keeps `usb-storage` probe out of `cdc_ncm`'s bind window |
| `libs/base/main_freertos_m7.cc` | `g_msc_ums.SetUnitReady(true)` at boot | `/dev/sda` appears automatically |
| `libs/msc_ums/msc_ums.h` | `SetUnitReady()` no longer arms `media_changed_` | Avoids UNIT_ATTENTION → bulk-IN STALL → BOT-Reset → bus reset cascade |
| `libs/msc_ums/msc_ums.cc` | `kUSB_DeviceMscEventRequestSense` returns `Success` | Lets the NXP layer reply with the stored sense data instead of stalling the endpoint |
| `libs/msc_ums/msc_ums.cc` | `SetWriteProtect()` updates `MODE SENSE` WP bit dynamically | `drive(1)`/`drive(0)` does not need a bus reset |
| `libs/msc_ums/msc_ums.cc` | NAND read retry loop (3×) | Tolerates transient page-read errors during read-only coexistence with LFS |
| `libs/base/main_freertos_m7.cc` | `sentai_usb_drive_set()` flips WP only (no `USB_DeviceStop`) | Toggle MSC writability without dropping IP and serial |
| `libs/base/main_freertos_m7.cc` | Keep `device_handle()` reference in `drive_set` | Removing it breaks `/dev/ttyACM0` (silent bulk-IN); keeps linker symbol graph identical |

After these changes, on the test workstation the board enumerates once and
stays up indefinitely:

```
$ lsusb | grep 1fc9
Bus 003 Device 088: ID 1fc9:c0a1 NXP Semiconductors - autonomous.ro SentAI board v1.0
$ ls /dev/ttyACM0 /dev/sda
/dev/sda  /dev/ttyACM0
$ ip -br link show | grep enx0
enx001a11badfad  UP  00:1a:11:ba:df:ad  <BROADCAST,MULTICAST,UP,LOWER_UP>
$ dmesg --since "60 seconds ago" | grep -c "reset high-speed USB device"
0
```

All three interfaces are usable simultaneously: REPL on `/dev/ttyACM0`,
HTTP/MAVLink/etc. on `10.0.0.1`, file transfer through `/dev/sda` after
`sentai.usb.drive(1)`. Toggling `drive(0)`/`drive(1)` does not affect the
other two interfaces.
