# SentAI Runtime — Flash Partitions & USB Storage

## Flash Hardware

The Coral Micro board contains a **NAND flash** chip organized in erase blocks
of **128 KB** (64 pages × 2048 bytes/page).  Flash-ul nu este mapat în spațiul
de adrese al procesorului — accesul se face exclusiv prin driver-ul NAND, peste
care rulează **LittleFS**.

## Partition Layout

```text
Block    0          12                 76                              523
         ├──────────┼──────────────────┼────────────────────────────────┤
         │ Reserved │   System LFS     │          User LFS             │
         │ 1.5 MB   │     8 MB         │          ~56 MB               │
         │ (boot)   │  (64 blocks)     │        (448 blocks)           │
         └──────────┴──────────────────┴────────────────────────────────┘
```

| Region | Blocks | Size | Conținut |
|--------|--------|------|----------|
| **Reserved** | 0–11 | 1.5 MB | Bootloader, hardware config |
| **System LFS** | 12–75 (64 blocks) | 8 MB | Firmware (`default.elf`), system files |
| **User LFS** | 76–523 (448 blocks) | ~56 MB | Modele `.tflite`, scripturi Python, imagini, loguri |

Constantele definitorii se găsesc în `libs/base/filesystem.cc`:

```cpp
constexpr int kPagesPerBlock     = 64;
constexpr int kFilesystemBaseBlock = 12;
constexpr int kSystemBlockCount  = 64;     // 8 MB
constexpr int kUserBaseBlock     = 76;     // 12 + 64
constexpr int kUserBlockCount    = 448;    // ~56 MB
constexpr lfs_size_t kPageSize   = 2048;
```

## De ce LittleFS?

1. **Rezistență la power-loss** — LittleFS este un filesystem copy-on-write;
   o întrerupere de curent nu corupe structura.  Pe un dispozitiv embedded
   alimentat de la USB, acest lucru este critic.

2. **Wear leveling** — distribuie scrierile uniform pe blocuri
   (`block_cycles = 250`), prelungind viața NAND-ului.

3. **Footprint mic** — RAM minim (cache 2 KB, lookahead 2 KB), fără
   tabele FAT în memorie.

4. **Două instanțe separate** — separarea system/user previne ca un script
   Python greșit să corupă firmware-ul.  Userul nu poate scrie în partiția
   system.

## LittleFS Configuration

Ambele partiții (system și user) folosesc aceeași configurare de bază:

| Parametru | Valoare |
|-----------|---------|
| `read_size` | 2048 |
| `prog_size` | 2048 |
| `block_size` | 131072 (128 KB) |
| `block_cycles` | 250 |
| `cache_size` | 2048 |
| `lookahead_size` | 2048 |

## USB Mass Storage

Partiția **user** poate fi expusă ca disc USB (mass storage class) direct
de pe board, fără niciun tool extern.  Host-ul (PC/Mac/Linux) vede un disc
de ~56 MB cu conținut raw LittleFS.

### Cum funcționează

Board-ul rulează un **USB composite device** care combină pe același port:

- **CDC-ACM** — consolă serială (REPL)
- **CDC-NCM** — Ethernet over USB (rețea)
- **MSC (Mass Storage Class)** — disc flash (SCSI transparent, Bulk-Only)

MSC-ul expune doar blocurile 76–523 (user partition) ca 28 672 LBA-uri
de 2048 bytes.  Inquiry string: `SENTAI  FLASH STORAGE   0001`.

### Activare / dezactivare

```python
>>> sentai.usb.drive(1)    # activează — PC-ul vede discul
>>> sentai.usb.drive(0)    # dezactivează — revine la LFS local
```

Sau fizic: **butonul User** de pe board dezactivează automat USB drive-ul.

### Protocol intern

**Activare** (`drive(1)`):
1. `lfs_unmount()` — demontează user LFS (exclusivitate NAND)
2. `SetUnitReady(true)` — MSC raportează medium present + UNIT ATTENTION
3. USB bus reset — host-ul re-enumerează și montează discul

**Dezactivare** (`drive(0)`):
1. `SetUnitReady(false)` — MSC raportează medium removed
2. USB bus reset — host-ul vede eject
3. `LfsUserRemount()` — remontează LFS, preia modificările făcute de host

```text
Python: sentai.usb.drive(1)
  │
  ├─ lfs_unmount(user)          ← LFS offline, NAND liber
  ├─ MSC SetUnitReady(true)     ← SCSI: medium present
  └─ USB bus reset              ← host montează disc
       │
       │  ... host scrie fișiere ...
       │
Python: sentai.usb.drive(0)  (sau buton User)
  │
  ├─ MSC SetUnitReady(false)    ← SCSI: medium removed
  ├─ USB bus reset              ← host face eject
  └─ LfsUserRemount()          ← LFS online, vede fișierele noi
```

### Safety interlock

Cât timp USB drive-ul este activ, **orice operație `sentai.fs.*`** aruncă
`OSError("flash busy: call sentai.usb.drive(0) first")`.  Acest lucru previne
accesul simultan LFS + host USB pe aceleași blocuri NAND.

### Limitare: format raw

Host-ul vede blocuri raw LittleFS, **nu FAT32**.  Pe Linux se poate monta cu
`lfs-fuse`; pe Windows/macOS este nevoie de un tool dedicat (de ex.
`littlefs-fuse` sau `littlefs-python`).  Alternativa recomandată este
transferul de fișiere prin `sentai.fs` din REPL sau prin RPC/HTTP.

## Formatare

```python
>>> sentai.fs.format()     # reformatează partiția user (șterge tot)
```

Dezactivează automat USB drive-ul dacă este activ, apoi face `lfs_format()` +
`lfs_mount()` pe partiția user.
