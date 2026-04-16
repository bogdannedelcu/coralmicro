# SentAI Runtime — USB Access Modes

## Overview

Board-ul SentAI (Coral Micro, NXP i.MX RT1176) expune un singur port USB-C
care funcționează ca **USB composite device** (VID `0x1FC9`, PID `0xC0A1`).
Pe același cablu USB coexistă simultan trei clase USB:

| Interfață | Clasă USB | Funcție | Device pe host |
|-----------|-----------|---------|----------------|
| 0, 1 | CDC-ACM | Consolă serială / REPL | `ttyACM0` (Linux), COM port (Windows) |
| 2, 3 | CDC-NCM | Ethernet over USB | `usb0` (Linux), adaptor rețea (Windows) |
| 4 | MSC | Flash storage (dezactivat implicit) | Disc removable (când e activat) |

Toate trei sunt enumerate simultan de host la conectare.

---

## 1. Terminal Serial (CDC-ACM)

### Ce vezi pe host

Portul CDC-ACM apare ca un serial virtual (`/dev/ttyACM0` pe Linux, COM pe
Windows).  Nu are baud rate real — e USB bulk transfer.  Orice terminal
serial funcționează (minicom, PuTTY, `screen`, picocom, Arduino Serial Monitor).

### Ce primești

Imediat după boot apare REPL-ul MicroPython:

```text
[00:00:042] SentAI build #197 (2026-04-10 14:22:03)
[00:00:043] Boot logging to /log/boot.log
[00:00:115] Edge TPU opened (mode 3)
[main.py] Running (128 bytes, timeout 30s)...
[main.py] Finished OK.
[MicroPython] Boot log saved to /log/boot.log

MicroPython REPL on SentAI board v1.0  [build #197  2026-04-10 14:22:03]
Ctrl+C to interrupt running code or cancel input.

>>>
```

### Mod dual: REPL vs. Serial Data

Portul CDC-ACM are **două moduri** mutual exclusive:

| Mod | Descriere | Cum se activează |
|-----|-----------|------------------|
| **REPL** (implicit) | Terminal interactiv Python | Default la boot |
| **Serial bridge** | Transfer binar de date cu host-ul | `sentai.console('uart')` + `sentai.usb.open()` |

În modul serial bridge, REPL-ul trece pe UART (header J10), iar USB-ul
devine un canal de date raw:

```python
>>> sentai.console('uart')       # mută REPL pe UART
>>> sentai.usb.open()            # deschide CDC-ACM ca serial bridge
# ... host-ul trimite/primește date prin ttyACM0 ...
>>> sentai.usb.close()           # revine la normal
>>> sentai.console('usb')        # mută REPL înapoi pe USB
```

### Python API — `sentai.usb` (serial)

| Funcție | Descriere |
|---------|-----------|
| `sentai.usb.open()` | Deschide CDC-ACM pentru date. Necesită REPL pe UART. |
| `sentai.usb.close()` | Revine la modul consolă. |
| `sentai.usb.write(data)` | Trimite `str`/`bytes` către host. Returnează nr. bytes. |
| `sentai.usb.read(max=256, timeout_ms=1000)` | Citește până la `max` bytes de la host. |
| `sentai.usb.available()` | Bytes disponibili în buffer-ul RX. |

### Baud rate special: 1200

Setarea baud rate la **1200** de pe host declanșează `ResetToBootloader()` —
convenție Arduino pentru reflash.

---

## 2. Rețea IP prin USB (CDC-NCM)

### Ce este

CDC-NCM (Network Control Model) creează o interfață Ethernet virtuală între
board și host.  Board-ul rulează un **server DHCP** care alocă automat IP
host-ului.

### Configurare IP

| Rol | Adresă |
|-----|--------|
| **Board** | `10.0.0.1` (static, configurabil via `/usb_ip_address` pe LFS) |
| **Host** | `10.0.0.100+` (DHCP, automat) |
| **Mască** | `255.255.255.0` |

MAC adresa board-ului: `00:1A:11:BA:DF:AD`.

### Activare

CDC-NCM este activ la nivel USB imediat la boot, dar serverul HTTP trebuie
pornit explicit:

```python
>>> sentai.usb.ip(1)
USB Ethernet (CDC-NCM) active at 10.0.0.1 — http://10.0.0.1/
```

### Servicii disponibile

#### A. Web File Browser — `http://10.0.0.1/`

Interfață web pentru browse / upload / download / delete fișiere pe partiția
user.  Se servește din `/.sys/browser.html` (deployed automat la prima
activare).

| Endpoint | Metodă | Descriere |
|----------|--------|-----------|
| `/` | GET | Browser web |
| `/api/ls/<path>` | GET | Listing director (JSON array) |
| `/api/raw/<path>` | GET | Download fișier raw |
| `/api/write/<path>` | POST | Upload fișier (body = conținut) |
| `/api/mkdir/<path>` | POST | Creează director |
| `/api/rm/<path>` | POST | Șterge fișier |
| `/api/_pr` | GET | Rezultatul ultimei operații POST |

Toate path-urile sunt validate (trebuie să înceapă cu `/`, fără `..`).

#### B. JSON-RPC — `POST http://10.0.0.1/jsonrpc`

Endpoint JSON-RPC 2.0 pentru comenzi programatice.  Folosit de toolurile
standard coralmicro (camera streaming, model inference, etc.).

### Exemplu: download / upload din terminal

```bash
# Download un fișier
curl http://10.0.0.1/api/raw/models/ssd.tflite -o ssd.tflite

# Upload un script
curl -X POST http://10.0.0.1/api/write/main.py --data-binary @main.py

# Listing director
curl http://10.0.0.1/api/ls/
```

### Python API — `sentai.usb.ip()`

| Funcție | Descriere |
|---------|-----------|
| `sentai.usb.ip(1)` | Pornește HTTP server, returnează 1 |
| `sentai.usb.ip(0)` | Doar raportează status (NCM rămâne activ) |

---

## 3. USB Mass Storage (MSC)

### Ce este

Board-ul poate expune partiția user (~56 MB, LittleFS) ca un disc USB
removable.  Host-ul vede un disc SCSI cu inquiry string
`SENTAI FLASH STORAGE 0001`.

### Activare / dezactivare

```python
>>> sentai.usb.drive(1)    # host-ul vede discul
>>> sentai.usb.drive(0)    # eject — revine la LFS local
```

Butonul **User** de pe board dezactivează automat USB drive-ul.

### Protocol intern

```text
sentai.usb.drive(1)
  ├─ lfs_unmount(user)         ← LFS offline
  ├─ MSC: unit_ready = true    ← SCSI: medium present
  └─ USB bus reset              ← host re-enumerează

      ... host accesează discul ...

sentai.usb.drive(0)  (sau buton User)
  ├─ MSC: unit_ready = false   ← SCSI: medium removed
  ├─ USB bus reset              ← host face eject
  └─ LfsUserRemount()          ← LFS online cu modificările host-ului
```

### Limitare: format LittleFS

Discul conține blocuri raw LittleFS, **nu FAT32/exFAT**.  Asta înseamnă:

- **Linux**: montabil cu `littlefs-fuse` sau acces din Python cu `littlefs-python`
- **Windows/macOS**: necesită tool dedicat; OS-ul standard nu recunoaște formatul

**Alternativă recomandată**: pentru transfer simplu, folosiți Web File Browser
(`sentai.usb.ip(1)`) sau serial bridge (`sentai.usb.open()`).

### Safety interlock

Cât USB drive-ul e activ, **orice operație** `sentai.fs.*` aruncă:

```
OSError: flash busy: call sentai.usb.drive(0) first
```

Aceasta previne accesul simultan LFS + host USB pe aceleași blocuri NAND.

---

## Comparație: care mod să folosești?

| Criteriu | Terminal (CDC-ACM) | Rețea IP (CDC-NCM) | USB Drive (MSC) |
|----------|--------------------|---------------------|-----------------|
| **Viteză transfer** | ~500 KB/s | ~2-5 MB/s | depinde de tool LFS |
| **Ușurință** | Orice terminal serial | Browser web / curl | Necesită tool LFS |
| **Transfer fișiere** | Via `sentai.usb.write()` | Upload/download HTTP | Acces direct blocuri |
| **REPL interactiv** | Da (implicit) | Nu | Nu |
| **Acces simultan LFS** | Da | Da | **Nu** (LFS unmounted) |
| **Cerințe host** | Driver serial (standard) | Driver NCM (standard) | Tool LittleFS |

### Recomandări

- **Dezvoltare interactivă**: Terminal CDC-ACM (REPL)
- **Transfer fișiere / deploy**: Rețea IP cu `sentai.usb.ip(1)` + `curl` sau browser
- **Backup complet**: USB drive + `littlefs-python` (acces raw la toată partiția)
- **Comunicare aplicație ↔ PC**: Serial bridge (`sentai.usb.open()`)

---

## Diagrama de stare USB

```text
                    ┌─────────────────────────────────────────┐
                    │          USB Composite Device            │
                    │                                         │
                    │  ┌─────────┐  ┌─────────┐  ┌────────┐  │
  Host USB ────────►│  │ CDC-ACM │  │ CDC-NCM │  │  MSC   │  │
                    │  │ (REPL)  │  │ (IP)    │  │ (drive)│  │
                    │  └────┬────┘  └────┬────┘  └───┬────┘  │
                    │       │            │            │        │
                    │       ▼            ▼            ▼        │
                    │  Console M7   lwIP stack    NAND raw    │
                    │       │            │            │        │
                    │       ▼            ▼            ▼        │
                    │     REPL      HTTP server   User LFS    │
                    │   printf()    File browser  (unmounted) │
                    └─────────────────────────────────────────┘
```

## Surse relevante

| Fișier | Rol |
|--------|-----|
| `modsentai_usb.c` | Python API `sentai.usb.*` |
| `libs/base/main_freertos_m7.cc` | USB composite init, `sentai_usb_drive_set()` |
| `libs/base/console_m7.cc` | CDC-ACM consolă, TX/RX, target switching |
| `libs/cdc_ncm/cdc_ncm.cc` | CDC-NCM Ethernet, IP config, DHCP |
| `libs/msc_ums/msc_ums.cc` | MSC SCSI handler, NAND block I/O |
| `sentai_httpd.cc` | HTTP file browser endpoints |
