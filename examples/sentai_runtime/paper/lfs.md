# SentAI Runtime — LittleFS Architecture & Access Paths

Acest document descrie filesystem-ul LittleFS al board-ului SentAI: cum e
organizat, cine scrie în el și când, cum îl folosesc REPL-ul / HTTP-ul / USB
MSC-ul, precum și tranzițiile RW ↔ RO.  Partition layout-ul brut pe NAND
este tratat în [`partitions.md`](partitions.md); aici ne concentrăm pe
arhitectura software.

## 1. Two LittleFS instances

Firmware-ul montează **două** partiții LittleFS independente, cu mutex-uri
interne separate (LFS-nivel-bibliotecă):

| Instanță | Handle C++ | Blocuri NAND | Dimensiune | Rol |
|---------|-----------|--------------|-----------|-----|
| **System** | `coralmicro::Lfs()` → `g_lfs` | 12–75 (64) | 8 MB | Firmware (`default.elf`), fișiere system, `/.sys/*` |
| **User** | `coralmicro::LfsUser()` → `g_lfs_user` | 76–523 (448) | ~56 MB | `main.py`, modele `.tflite`, fișiere utilizator, `/log/*` |

Separarea previne ca scripturile Python să corupă firmware-ul — partiția
system este efectiv read-only din userspace (REPL/HTTP n-au API care să
scrie în ea, cu excepția `sentai_runtime`-ului la boot pentru
`/.sys/browser.html`).

Definițiile sunt în [`libs/base/filesystem.cc`](../../../libs/base/filesystem.cc):

```cpp
constexpr int       kPagesPerBlock       = 64;
constexpr int       kFilesystemBaseBlock = 12;
constexpr int       kSystemBlockCount    = 64;     // system LFS
constexpr int       kUserBaseBlock       = 76;
constexpr int       kUserBlockCount      = 448;    // user LFS
constexpr lfs_size_t kPageSize           = 2048;
// LFS per-instance config (identic pentru ambele):
//   read_size=2048, prog_size=2048, block_size=131072
//   block_cycles=250, cache_size=2048, lookahead_size=2048
```

`LfsUser()` returnează `nullptr` până când `LfsUserInit()` montează cu
succes partiția.  În **storage mode** (`sentai.usb.drive(1)`) inițializarea
este deliberat sărită — host-ul primește NAND-ul exclusiv — și `LfsUser()`
rămâne `nullptr`, așa că toți consumatorii (httpd, `boot_log_fs_init`,
`crash_log_*`) no-op în loc să facă crash pe `lfs_*` pe o partiție
nemontată.

## 2. App-level mutex: `s_lfs_mutex`

Pe lângă mutex-ul intern al LittleFS (folosit automat de fiecare apel
`lfs_*`), există un mutex **aplicativ** — `s_lfs_mutex` — definit în
[`sentai_lfs_task.cc`](../sentai_lfs_task.cc) și expus prin:

```c
int  sentai_lfs_lock(void);   // timeout 2000ms; returnează 1 / 0
void sentai_lfs_unlock(void);
```

Rolul său este să serializeze **secvențe logice** (open→read→close)
împotriva altora, pentru ca un reader HTTP să nu vadă un fișier pe
jumătate scris.  Mutex-ul intern al LittleFS nu e suficient: garantează
atomicitatea unui singur `lfs_*`, nu a unei secvențe.

**Regula de aur:** orice task care face mai mult decât un singur apel
`lfs_*` pe partiția user **trebuie** să ia `sentai_lfs_lock()` înainte.
Excepție deliberată: POST-urile HTTP (vezi §4.3).

## 3. Cine scrie și când

Întrebarea critică: **pe un board idle (după ce REPL a pornit), ce scrie
automat în LFS?**

Răspuns: **nimic**.  Nu există timer periodic, nu există task de health
care să scrie, `lfs_task` stă blocat pe `xQueueReceive`.  Toate scrierile
sunt **inițiate de utilizator sau de evenimente discrete**:

| Writer | Locație | Când | Frecvență |
|--------|---------|------|-----------|
| `boot_log_write` → `boot_log_flush_to_file` | [`sentai_runtime.cc:159`](../sentai_runtime.cc#L159) | Cât timp `g_boot_log_active == true`, când buffer-ul RAM (16 KB) se apropie de plin | **Doar în timpul boot-ului** |
| `boot_log_stop` (flush final + close) | [`sentai_runtime.cc:232`](../sentai_runtime.cc#L232) | Chemat din [`micropython_task.c:466`](../micropython_task.c#L466), imediat înainte ca REPL să afișeze promptul | **Single-shot la boot** |
| `crash_log_write` | [`sentai_runtime.cc:338`](../sentai_runtime.cc#L338) | HardFault, `HTTP_HANG`, watchdog trigger | **Doar pe crash** (rar) |
| `sentai.fs.write / mkdir / remove / format / ...` | [`modsentai_fs.c`](../modsentai_fs.c) | User script în REPL sau `main.py` | **On-demand** |
| POST `/api/write` / `/api/mkdir` / `/api/rm` | [`sentai_httpd.cc:248`](../sentai_httpd.cc#L248) | Request HTTP de la client | **On-demand** |
| Host prin USB MSC | direct pe NAND (fără LFS firmware) | Cât timp `sentai.usb.drive(1)` e activ | **On-demand** |

`boot_log_active` este `true` doar între `boot_log_init()` și
`sentai_boot_log_stop()` (imediat înainte de REPL).  După REPL start,
`boot_log_write()` iese instant fără să atingă LFS.  **Nu există log
rotation periodică, nu există heartbeat scris pe flash.**

### 3.1 Doar citesc

| Reader | Locație | Când |
|--------|---------|------|
| `sentai.fs.read / read_str / ls / exists / size` | [`modsentai_fs.c`](../modsentai_fs.c) | User script |
| GET `/api/ls` / `/api/raw` | [`sentai_httpd.cc:139`](../sentai_httpd.cc#L139) + [`sentai_lfs_task.cc`](../sentai_lfs_task.cc) | Request HTTP |
| `sentai_get_last_crash_log_path` | [`sentai_runtime.cc:418`](../sentai_runtime.cc#L418) | `sentai.diag.crash_log()` |
| `boot_log_fs_init` (rotation `boot.log` → `boot_old.log`) | [`sentai_runtime.cc`](../sentai_runtime.cc) | Single-shot la boot |
| `LoadBrowserHtmlCache` | [`sentai_httpd.cc:67`](../sentai_httpd.cc#L67) | Single-shot la boot |

## 4. Access paths

### 4.1 MicroPython REPL (`sentai.fs.*`)

Fiecare funcție publică din [`modsentai_fs.c`](../modsentai_fs.c) urmează
același model:

```c
static mp_obj_t mod_sentai_fs_read(mp_obj_t path_obj) {
    _fs_check_usb();                         // ref. §5 — RO în storage mode
    if (!sentai_lfs_lock()) {                // 2000ms timeout
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("lfs busy"));
    }
    // ... lfs_* calls protejate de mutex
    sentai_lfs_unlock();
    return result;
}
```

Dacă mutex-ul e contenționat >2s, Python primește `OSError("lfs busy")`
— wrapper defensiv, în practică nu se întâmplă pe un board sănătos.

### 4.2 HTTP GET (`/api/ls`, `/api/raw`) — fast-path arhitectură

GET-urile sunt arhitectura cea mai subtilă, pentru că rulează în
**`tcpip_thread`** (prioritatea cea mai mare, FreeRTOS 4).  Dacă acest
thread se blochează >5s, stack-ul USB NCM raportează "transmit queue
timed out" și placa pare moartă.  Designul trebuie să **nu blocheze
niciodată** `tcpip_thread` pe o scriere de flash lungă.

Implementarea curentă în [`sentai_lfs_task.cc`](../sentai_lfs_task.cc) are
trei căi, cascadate:

```text
GET /api/ls/foo
  │
  ▼
sentai_lfs_try_serve()
  │
  ├─ Step 1: slot SLOT_READY & path match?   → serve din cache  (rare hit)
  │
  ├─ Step 2: FAST PATH (uzual)
  │     xSemaphoreTake(s_lfs_mutex, 500ms)
  │     ├─ succes → DoLs/DoRaw inline în tcpip_thread → serve imediat
  │     │         (1 GET = 1 response)
  │     └─ timeout → cade la slow path
  │
  └─ Step 3: SLOW PATH (fallback când fast path nu poate)
        EnqueueLfsRequest() → lfs_task (prio 2) procesează asincron
        răspuns imediat: {"error":"lfs_busy"}
        browser.html retry 600ms → slot devine READY → serve
```

**De ce 500ms e safe ca timeout pe `tcpip_thread`:**
-   USB NCM transmit-timeout pe host ~5s → stall de 500ms e invizibil.
-   Odată ce `s_lfs_mutex` e luat, LFS-intern e garantat liber (pentru că
    toți ceilalți useri — MP, boot_log, crash_log, lfs_task — trec prin
    același mutex aplicativ).  Deci `lfs_dir_open` / `lfs_file_read` nu
    vor aștepta pe flash-GC în plus.
-   Worst-case real: MP face `lfs_file_close` (poate declanșa GC, ~700ms).
    Fast-path-ul așteaptă 500ms, nu prinde, cade la slow path; clientul
    retry după 600ms și de data asta fast path îl prinde.

**Latența măsurată pe idle (fără alt writer activ):**

| Endpoint | Latență |
|----------|---------|
| GET `/api/raw/small_file.txt` | ~40-85 ms |
| GET `/api/ls/` (root, ~12 entries) | ~230-400 ms |
| GET `/api/ls/log` (24 entries) | ~700-900 ms |

Aceștia sunt **timpii LittleFS pe NAND+FlexSPI**, nu overhead-ul
protocolar.  REPL-side `sentai.fs.ls` e chiar mai lent pentru aceleași
directoare (e.g. 504 ms vs. 383 ms pe root), pentru că trece și prin
costul de conversie Python + lock app-level.

**Slot state machine** (`sentai_lfs_task.cc`):

```text
    IDLE ──enqueue──▶ PENDING ──lfs_task done──▶ READY
     ▲                                              │
     │                                     try_serve match
     │                                              │
     └───────── FsCloseCustom (resp_done) ◀── SERVING
```

Câmpurile `s_slot_type` și `s_slot_path` (strncpy-ite de
`EnqueueLfsRequest`) disambiguează rezultatele stale când retry-urile
interacționează cu alte GET-uri diferite.

**Bug-fix-uri notabile** în history:
-   Build #584: lwIP rescrie URI-urile cu `/` terminal adăugând unul din
    [`httpd_default_filenames`](../../../third_party/nxp/rt1176-sdk/middleware/lwip/src/apps/http/httpd.c)
    (`index.shtml/ssi/shtm/html/htm`), deci `GET /api/ls/` sosea ca
    `/api/ls/index.shtml` și `DoLs("/index.shtml")` returna `[]`.
    `sentai_httpd.cc` tună acum aceste sufixe.
-   Build #585: designul async inițial obliga mereu **≥2 HTTP round-trips**
    per GET (primul răspundea `lfs_busy` prin construcție).  Fast path-ul
    elimină acest overhead artificial pentru cazul uzual (LFS idle).
-   Build #633 (2026-04-20): fast path-ul e restrâns la **doar `GET
    /api/raw`**; `GET /api/ls` merge întotdeauna prin `lfs_task` (slow
    path).  Motivul: empiric, `DoLs("/")` pe un filesystem vechi cu ~25
    intrări în rădăcină (inclusiv ~22 MB de modele `.tflite`) depășea
    frecvent 30 s de `lfs_dir_*` în `tcpip_thread`; asta ținea HTTP
    activitatea "idle" din punctul de vedere al watchdog-ului de rețea
    și placa ajungea în reset-loop la pragul de 2 min.  Per
    [embeded.md](../agent/embeded.md) §B *"Real-time and supervision
    rules"*, orice cod rulat în `tcpip_thread` trebuie să fie strict
    bounded — worst-case-ul LittleFS pe dir walk nu e.  RAW reads rămân
    pe fast path: citirile secvențiale de fișier au cost predictibil,
    mărginit de `kRespBufSize = 256 KB`.  Costul pentru LS: fiecare
    listing necesită ≥2 round-trip-uri (primul răspunde `lfs_busy`,
    al doilea servește din cache SLOT_READY după ce `lfs_task` a rulat
    `DoLs`).  La 600 ms retry în browser, latența totală vizibilă user
    e ~700 ms în loc de ~300 ms — compromis acceptabil față de riscul
    unui reset al plăcii.  Vezi
    [`sentai_lfs_task.cc:sentai_lfs_try_serve`](../sentai_lfs_task.cc)
    pentru detalii.

### 4.3 HTTP POST (`/api/write`, `/api/mkdir`, `/api/rm`)

POST-urile rulează tot în `tcpip_thread` dar **NU** iau `s_lfs_mutex`.
Se bazează pe mutex-ul intern LittleFS pentru atomicitatea per-call.

Motivul: POST-urile sunt inițiate de utilizator (click „Save"), foarte
rare, și worst-case stall-ul (`lfs_file_close` cu flash GC ~700 ms) e
sub toleranța USB NCM (~5 s).  Trade-off-ul deliberat e simplitatea în
schimbul unei teoretice race-condiții cu un GET fast-path pe același
fișier — nu s-a manifestat în testare.

Dacă vreodată apare o problemă de consistență vizibilă, remediul e să
se ia și `sentai_lfs_lock()` în handler-ele POST.

### 4.4 USB MSC (host mount)

Când `sentai.usb.drive(1)`:

1.   `lfs_unmount(g_lfs_user)` — eliberează exclusiv NAND-ul user-partition.
2.   MSC raportează medium-present → host Linux expune `/dev/sda`.
3.   Host-ul citește/scrie blocuri NAND direct, **fără ca firmware-ul să
     atingă LFS**.

Când `sentai.usb.drive(0)`:

1.   MSC raportează medium-removed → host face eject.
2.   `LfsUserRemount()` re-montează LFS-ul, care vede toate modificările
     făcute de host.

Implicație importantă: **`/dev/sda` expune blocurile LittleFS raw**
(28 672 × 2048 bytes = 56 MB); nu e FAT32.  Pe Linux e necesar
[`littlefs-fuse`](../../../third_party/littlefs-fuse/) cu parametrii
care trebuie să se potrivească EXACT cu config-ul firmware-ului:

```bash
sudo littlefs-fuse \
  --block_size=131072 --read_size=2048 --prog_size=2048 \
  --block_count=448 --cache_size=2048 --lookahead_size=2048 \
  /dev/sda /mnt/sentai
```

Valori nepotrivite → **corupție silențioasă** (LittleFS nu detectează
mismatch-ul la mount).

## 5. RW vs RO states

Partiția user LFS are trei stări distincte, controlate de `sentai.usb.drive`:

```text
┌─────────────────────────────────────────────────────────────────┐
│                   DEFAULT MODE  (drive(0))                      │
│                                                                  │
│   Firmware  :  LfsUser() valid, RW                              │
│   REPL      :  sentai.fs.read/write/ls/… OK                     │
│   HTTP GET  :  /api/ls, /api/raw OK                             │
│   HTTP POST :  /api/write, /api/mkdir, /api/rm OK               │
│   Host MSC  :  /dev/sda NU există                                │
│                                                                  │
│   Writer activ la un moment dat:  cel mult unul (mutex)         │
└─────────────────────────────────────────────────────────────────┘
                               │
             sentai.usb.drive(1)  (warm reset — storage persist)
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                   STORAGE MODE  (drive(1))                       │
│                                                                  │
│   Firmware  :  LfsUserInit() SĂRIT; LfsUser() returnează NULL    │
│   REPL      :  OFF (poll pe CDC-ACM pentru 'q' de exit)          │
│   HTTP      :  neaccesibil (CDC-NCM off)                         │
│   Host MSC  :  /dev/sda vizibil, RW pe blocuri raw               │
│                                                                  │
│   LFS partition nemontată pe firmware — exclusivitate host       │
└─────────────────────────────────────────────────────────────────┘
                               │
                       'q' pe tty / buton User
                               ▼
                       warm reset → default mode
                       (LfsUserRemount vede modificările host-ului)
```

### 5.1 Gardarea RO ("flash busy")

În default mode, dacă `sentai.usb.drive(1)` e activ (edge case: ordine
de init), MP funcțiile verifică `_fs_check_usb()` și raise:

```python
>>> sentai.fs.write('/foo.txt', b'bar')
OSError: flash busy: call sentai.usb.drive(0) first
```

Analog la nivel HTTP POST în [`sentai_httpd.cc:227`](../sentai_httpd.cc#L227):

```
{"ok":false,"error":"USB drive active — call sentai.usb.drive(0) first"}
```

GET-urile sunt permise și în storage mode, dar `DoLs`/`DoRaw` vor vedea
`LfsUser() == nullptr` și vor returna `[]` / 404 — comportament
defensiv, nu crash.

### 5.2 Storage-mode crash-loop guard

Dacă 3 boot-uri consecutive în storage mode eșuează fără să ajungă la
`sentai_storage_boot_succeeded()`, magic-ul din DTC-RAM
(`.noinit_boot_persist`) e șters și board-ul forțează default mode la
următorul reset — recuperare fără acces fizic.  Detalii în
[`usb.md`](usb.md) și [`boot.md`](boot.md).

### 5.3 Storage sessions: no auto-reset

Sesiunile în storage mode sunt **nelimitate ca durată** — user-ul iese
doar voluntar (`drive(0)`, byte pe `/dev/ttyACM0`, buton User, RESET).
Builds ≤ #586 aveau un bug prin care watchdog-ul de rețea omora
board-ul la ~2m30s (nici HTTP, nici REPL nu pot actualiza
`*_last_activity` în storage mode, deci idle-ul creștea monoton);
corectat în build #587 prin bypass în `CombinedWatchdogTask`.  Vezi
[`watchdog.md` §Storage-mode bypass](watchdog.md).

## 6. Operaționalizare & observații

**Performanța LittleFS pe acest hardware** (NAND prin FlexSPI,
block_size=128KB, cache=2KB) este inerent modestă:
-   ~30-60 ms per directory entry la `lfs_dir_read`
-   ~5-15 ms per 2KB page la `lfs_file_read`
-   ~100-700 ms la `lfs_file_close` pe un fișier scris (inclusiv GC)

Dacă latența devine problemă, knob-urile disponibile sunt:
-   Mărire `cache_size` (acum 2 KB) — mai mult RAM, mai puține re-reads pe
    metadata.
-   Mărire `lookahead_size` — alocarea de blocuri libere e mai rapidă.
-   Payload-uri mai mici la `DoLs` (pagination).
-   Mutare fișierelor mari pe partiții separate dacă vreodată se
    introduce un al treilea LFS.

**Formatare catastrofică**: `sentai.fs.format()` cheamă
`LfsUserInit(force_format=true)` — șterge TOT din user partition,
reinițializează.  Util pentru recovery după corupție (ex. mismatch de
parametri la `littlefs-fuse`).

**Nu există log-rotation automat** în firmware.  Dacă user scripts
acumulează fișiere (`/log/*` sau altundeva), trebuie șterse manual sau
printr-un cron in-script.

## 7. File map

```text
/                            root user partition
├── main.py                  script executat la boot (safe-boot protected)
├── .sys/
│   └── browser.html         UI web (servit la GET /)
├── log/
│   ├── boot.log             printf-urile de la power-on până la REPL
│   ├── boot_old.log         rotated (boot-ul precedent)
│   ├── crash_NNN.log        dump-uri de fault (rotate, max 10)
│   └── .boot_seq            counter boot (single-shot guard)
├── lib/                     module Python user
├── diags/                   fișiere diagnostic (opțional)
└── [user files]             modele .tflite, imagini, etc.
```

Toate path-urile sunt relative la LFS user partition.  Partiția system
nu e accesibilă user-ului.
