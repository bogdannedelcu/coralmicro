# SentAI Runtime — `sentai.flow` Optical Flow Engine

Acest document descrie arhitectura completă a `sentai.flow`: subsistemul
de optical flow construit deasupra camerelor OV5640 și rulând în **paralel
cu pipeline-ul de detecție TPU**, fără să atingă performanța acestuia.
Convenția de axe body-frame e tratată separat în
[`flow_body_frame.md`](flow_body_frame.md); matematica altitudinii, în
[`flow_altitude.md`](flow_altitude.md).

---

## 1. Obiectiv și constrângeri

Drona are nevoie de un semnal de **viteză orizontală** pentru autopilot
— o măsurătoare (Δx, Δy) în pixeli grid per frame, derivată din
deplasarea scenei între două cadre consecutive.  Cerințele user-ului au
fost stricte:

1.   Să ruleze **constant**, nu doar on-demand.
2.   **Să NU degradeze pipeline-ul de detecție** care rulează la
     ~24 fps alternant cam0/cam1.  Target: <2% scădere de fps.
3.   Să se poată porni/opri din MicroPython (`sentai.flow.m4_start`,
     `m4_stop`, `m4_read`).
4.   Datele (dx, dy, sad, confidence) disponibile la rata framerate-ului
     senzorului (25 Hz).
5.   Constrângeri NASA/JPL uzuale: bounded loops, zero heap runtime, ISR
     minimală, fault containment.

Soluția: **offload pe M4**.  RT1176 are un Cortex-M4 @ ~400 MHz lângă
M7-ul @ 800 MHz; M4 e dormant în firmware-ul stock.  Plasăm SAD block-
match-ul pe M4, frame-uri publicate de M7 prin shared memory OCRAM.

---

## 2. Arhitectură la nivel înalt

```
        ┌───────────────────────────────── M7 (800 MHz) ───────┐
        │                                                      │
        │   CSI ISR ─▶ PrepTask ─▶ InferTask ─▶ TPU ─▶ queue   │
        │                │                                     │
        │                │  (publish hook, dacă flow_enabled)  │
        │                ▼                                     │
        │     sentai_flow_m4_publish_frame(raw)                │
        │          step-16 decimation (raw → 40×30 gray)       │
        │          ~50 µs CPU, < 1% din bugetul PrepTask       │
        │                │                                     │
        └────────────────┼─────────────────────────────────────┘
                         │
                         ▼  non-cacheable OCRAM window
             ┌──────────────────────────┐
             │   flow_shared_t          │
             │   @ 0x202C1000 (4 KB)    │
             │   header + 40×30 gray    │
             │   cmd / cmd_seq / result │
             └──────────────────────────┘
                         │
        ┌────────────────┼─────────────────────────────────────┐
        │                ▼                                     │
        │    app_main on M4 (FreeRTOS task prio 2)             │
        │      poll frame_valid → memcpy → SAD block-match     │
        │      publish (dx, dy, sad, confidence)               │
        │                                                      │
        └──────────────────────── M4 (400 MHz) ────────────────┘

        MicroPython on M7 REPL:  sentai.flow.m4_read()
        = pure memory read from flow_shared_t (no IPC roundtrip)
```

**Lock-free, one-way data flow.**  M7 writes gray + frame_seq, M4 reads.
M4 writes result, M7 (via MP) reads.  Synchronization via volatile
fields + explicit `__DMB()` barriers on both sides.

---

## 3. Boot și bring-up M4

### 3.1 Binary packaging

M4 are un ELF separat la build (`sentai_flow_m4`), convertit în blob
binar (`sentai_flow_m4.bin`) și embed-at ca secțiune `.core1_code` în
ELF-ul M7 via `objcopy --rename-section .data=.core1_code`.  Linkerul
CMake definește simboluri `m4_binary_start`, `m4_binary_end`,
`m4_binary_size` ca pointer-i la blob.  Vezi [`cmake/toolchain-arm-
none-eabi-gcc.cmake`](../../../cmake/toolchain-arm-none-eabi-gcc.cmake)
linia 242 (`add_executable_m4`) și [`CMakeLists.txt`](../CMakeLists.txt)
linia 136.

**Flash**: un singur image (`image.srec`), M4-ul trăiește ca date în
interiorul M7.  La runtime, `IpcM7::StartM4()` copiază blobul la
0x20200000 (adresa de boot M4 în OCRAM) și apelează `MCMGR_StartCore()`.

### 3.2 Minimal main() pe M4 — de ce

Template-ul default `libs_base-m4_freertos` apelează
`BOARD_InitHardware(true)` în main-ul lui, care la rândul lui rulează:
`BOARD_InitBootPins()`, `BOARD_InitBootClocks()`, `BOARD_ConfigMPU()`,
`BOARD_InitNAND()`.  Problema: **`BOARD_InitBootPins` resetează pin-
mux-ul camerelor** pe care M7 le are deja configurate → CSI ISR rupe,
`E:0A01 CAM_SWITCH_FALLBACK` la fiecare cerere, pipeline-ul se blochează
la primele switch-uri.

Fix: override **weak main()** în [`flow_task_m4.cc`](../flow_task_m4.cc)
care face **doar**:

```c
extern "C" int main(int argc, char** argv) {
    BOARD_ConfigMPU();          // region 15 non-cacheable pe sh mem
    MCMGR_Init();               // MU inter-core signaling
    xTaskCreateStatic(app_main, ...);
    vTaskStartScheduler();
}
```

Zero pin/clock touch.  M7 nu e afectat.  Consecințe documentate:
- M4 FreeRTOS tick e nefiabil ca wall clock (nu s-a rulat
  `BOARD_InitBootClocks`); folosim **DWT_CYCCNT** pentru timing —
  publicăm cycles, nu µs false.
- `IpcM4::Init()` deliberat NU e apelat → `M4IsAlive()` din M7 întoarce
  întotdeauna timeout (-2); folosim shared magic în locul RPMsg pentru
  alive detection.

### 3.3 Startup opt-in

Boot-ul M7 NU pornește M4 automat.  User o face din MP:

```python
rc = sentai.flow.m4_enable()
# rc == 0 : M4 alive (shared magic seen within 2s)
# rc == -1: no M4 image linked
# rc == -3: magic not written within 2s (bring-up failed)
```

Ideempotent via shared magic check — apelurile ulterioare sunt instant 0.
Vezi [`sentai_runtime.cc:~1145`](../sentai_runtime.cc) pentru detalii.

---

## 4. Shared-memory protocol

### 4.1 Locația fizică

`flow_shared_t` locuieste la **adresă fixă hardcodată** `0x202C1000`,
în interiorul regiunii `rpmsg_sh_mem` (8 KB @ 0x202C0000), configurată
de `BOARD_ConfigMPU` ca **region 15 = device-type, non-cacheable, not
shareable**.  Ambele core-uri (M7 și M4) văd aceeași adresă fizică.

**De ce adresă fixă și nu section-based**: linker-ele M7 și M4 se
rulează independent.  Dacă aș declara struct-ul cu
`__attribute__((section(".rpmsg.flow")))`, fiecare binar l-ar plasa la
un offset diferit în funcție de ordinea simbolurilor → ambele core-uri
ar citi adrese diferite.  Adresa fixă elimină acest risc (NASA/JPL §E
— message-passing cu ownership explicit).

### 4.2 Layout

```c
typedef struct {
    // ── M4 → M7: heartbeat + status ─────────────── 0x00..0x17
    volatile uint32_t magic;             // 0x53464C57 = 'SFLW'
    volatile uint32_t version;           // 2 (bump la layout change)
    volatile uint32_t m4_heartbeat;      // ++ per 100ms din M4
    volatile uint32_t m4_tick_ms;        // tick M4 × portTICK_PERIOD_MS
    volatile uint32_t m4_boot_ts_ms;     // boot timestamp M4
    volatile uint32_t m4_state;          // IDLE / RUNNING

    // ── M4 → M7: ultimul rezultat SAD ───────────── 0x18..0x2B
    volatile int32_t  last_dx;           // în grid 40×30
    volatile int32_t  last_dy;
    volatile uint32_t last_sad;          // SAD la peak
    volatile uint32_t last_frame_seq;    // frame_seq de la care vine
    volatile uint8_t  last_confidence;   // 0..255

    // ── M4 → M7: statistici ─────────────────────── 0x2C..0x3B
    volatile uint32_t frames_processed;
    volatile uint32_t frames_dropped;
    volatile uint32_t last_compute_us;   // DE FAPT: DWT cycles (tick nefiabil)
    volatile uint32_t avg_compute_us;    // idem

    // ── M7 → M4: canal de comenzi ───────────────── 0x3C..0x47
    volatile uint32_t cmd;               // NOP / START / STOP
    volatile uint32_t cmd_seq;           // ++ la fiecare cmd nou
    volatile uint32_t cmd_ack_seq;       // M4 oglindește cmd_seq după apply

    // ── M7 → M4: publicare frame ──────────────── 0x48..0x53
    volatile uint32_t frame_seq;         // ++ după ce gray e umplut
    volatile uint32_t frame_valid;       // 1 = neconsumat, 0 = M4 a luat
    volatile uint8_t  frame_cam_id;      // 0 sau 1

    volatile uint32_t _reserved[5];      // pad la 0x60

    // ── M7 → M4: gray frame 40×30 = 1200 B ──────── 0x60..0x52F
    volatile uint8_t  gray[FLOW_GRAY_PIXELS];
} flow_shared_t;
```

Total: ~1300 bytes, mai puțin de 4 KB disponibile în fereastra rpmsg
(4 KB rămase după IPC queue-urile existente).

### 4.3 Memory barrier protocol

**Cross-core coherency** pe Cortex-M7/M4 cu region non-cacheable e
garantată de MPU, dar ordering-ul în load/store buffer-uri **nu** e
automat.  Protocolul:

```c
// M7 publish side (sentai_flow_m4_publish_frame):
//   1. umple bytes în sh->gray (fără barrier între iterații)
//   2. __DMB() ← release fence: comite toate gray writes
//   3. sh->frame_seq = ++counter
//   4. __DMB()
//   5. sh->frame_valid = 1  ← observatorul vede asta ULTIMUL

// M4 consume side (app_main):
//   1. if (sh->frame_valid) { ...
//   2.   __DMB() ← acquire fence: ÎNAINTE de citi gray/frame_seq
//   3.   uint32_t seq = sh->frame_seq;
//   4.   memcpy(s_gray[slot], sh->gray, FLOW_GRAY_PIXELS);
//   5.   sh->frame_valid = 0;   ← eliberează slotul pentru M7
```

Fără `__DMB()` pe M4 după check-ul `frame_valid`, Cortex-M4 poate
servi din load-buffer bytes stale din gray (vedere pre-M7-writes).
Analog pentru canalul de comenzi: M7 scrie `cmd` apoi `cmd_seq`, M4
citește `cmd_seq` apoi face `__DMB()` înainte de a citi `cmd`.

Asta e fix-ul NASA/JPL #1 + #3 aplicat în build #730.

---

## 5. M7 side: PrepTask publish hook

### 5.1 Inserția în pipeline

PrepTask (pipeline TPU) deja citește raw XRGB de la CSI și-l trimite
la PXP pentru scalarea în 512×512 pentru TPU.  Hook-ul flow se
adaugă **între PXP și cam_return_raw**:

```c
// detection_task.cc:~305
int rc = sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                          dst_buf, w, h);
// Hook nou: publică 40×30 gray în shared memory pentru M4.
sentai_flow_m4_publish_frame(raw, DEMO_CAMERA_WIDTH,
                             DEMO_CAMERA_HEIGHT,
                             sentai_cam_current_id());
sentai_cam_return_raw(idx);
```

### 5.2 Decimare CPU step-16

```c
// flow_task.cc:sentai_flow_m4_publish_frame
const int step_x = raw_w / FLOW_GRAY_W;   // 640/40 = 16
const int step_y = raw_h / FLOW_GRAY_H;   // 480/30 = 16
if (step_x < 1 || step_y < 1) return;     // fail closed (fix #2)

for (int y = 0; y < FLOW_GRAY_H; ++y) {
    const uint8_t* p = src_row;
    for (int x = 0; x < FLOW_GRAY_W; ++x) {
        // luma estimate robust la XRGB byte order:
        // toate 4 bytes sumati, shift 2.  Canalul X (constant)
        // adaugă un DC bias, invariant la SAD.
        const uint32_t sum = p[0] + p[1] + p[2] + p[3];
        *dst++ = (uint8_t)(sum >> 2);
        p += step_x * 4;
    }
    src_row += step_y * raw_w * 4;
}
```

Cost pe M7 măsurat: **~50 µs/frame** — 0.2% din bugetul de 22 ms per
frame al PrepTask.  Confirmat prin E26: pipeline fps = 24.66-24.70
cu flow activ vs 24.97 baseline (**delta < 2%**).

### 5.3 Early-return pe flag OFF

Hook-ul verifică un atomic flag la început:

```c
if (!__atomic_load_n(&s_flow_m4_publish_enabled, __ATOMIC_ACQUIRE))
    return;
```

Zero cost când flow e dezactivat (o singură citire volatilă ~1 ns).
`sentai.flow.m4_start()` face `__atomic_store_n(..., __ATOMIC_RELEASE)`
pe acest flag + scrie `sh->cmd = START` + bump `cmd_seq`.

---

## 6. M4 side: SAD block-match

### 6.1 Algoritm

Block de 16×16 pixeli centrat în grid-ul 40×30, căutare full pe o
fereastră de ±6 pixeli:

```c
constexpr int kBlockW      = 16;
constexpr int kBlockH      = 16;
constexpr int kSearchRange = 6;

// Worst case: 13² candidates × 16² pixels = 43 008 add/abs ops.
// Early-exit pe SAD > best: typical caz ~10-20% din worst.
// Runtime măsurat pe DWT: 200-400 µs pe M4.
```

Sub-pixel precision actualmente NU e aplicată (dy/dx returnate ca
întregi).  Îmbunătățire pe roadmap: **parabolic fit** pe SAD în jurul
minimului pentru subpixel, ~10× rezoluție pe axă.

### 6.2 Ping-pong buffers locale

M4 păstrează DOUĂ buffer-e gray locale:

```c
static uint8_t s_gray[2][FLOW_GRAY_PIXELS]
    __attribute__((aligned(4)));     // default section = cacheable OCRAM
```

Motivul: după ce M7 scrie un nou gray în shared mem, M4 îl copiază
într-unul din cele două sloturi locale `s_gray[curr]`, apoi SAD-uiește
`s_gray[curr]` vs `s_gray[prev]`.  Dacă M7 scrie iar înainte ca SAD
să se termine, frame-ul precedent e protejat.

**De ce e safe în cacheable OCRAM**: M7 NU citește și nu scrie în
`s_gray`; e pur M4-local.  Cache-ul M4 se comportă consistent pentru
single-writer/single-reader/same-core access.

### 6.3 Command/state machine

```c
while (true) {
    // heartbeat 10 Hz (neblocant — folosește tick_ms ca trigger)
    if (now_ms - last_hb >= 100) { update_heartbeat(); }

    // aplică comanda dacă cmd_seq != cmd_ack_seq
    if (sh->cmd_seq != sh->cmd_ack_seq) {
        __DMB();                     // fix #3: acquire fence
        apply_cmd(sh->cmd);          // START/STOP → m4_state
        sh->cmd_ack_seq = sh->cmd_seq;
    }

    // procesează frame dacă state=RUNNING și e unul disponibil
    if (sh->m4_state == RUNNING && sh->frame_valid) {
        __DMB();                     // fix #1: acquire fence
        copy + SAD + publish result
        sh->frame_valid = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1));    // 1000 Hz loop; camera e la 25 Hz
}
```

---

## 7. MicroPython API

| Funcție | Descriere | Alocare heap MP |
|---|---|---|
| `sentai.flow.m4_enable()` | Pornește M4 (prima apelare); idempotent | 0 |
| `sentai.flow.m4_start([cam_id=0])` | Comandă M4 să proceseze; activează publish hook | 0 |
| `sentai.flow.m4_stop()` | Dezactivează publish + trimite STOP la M4 | 0 |
| `sentai.flow.m4_heartbeat()` | Dict `{alive, heartbeat, tick_ms, ...}` | ~150 B |
| `sentai.flow.m4_read()` | Dict `{dx, dy, sad, confidence, frames_processed, ...}` | ~250 B |
| `sentai.flow.m4_body_read()` | `m4_read()` cu `(dx,dy) → (body_fw, body_left)` conversie | ~200 B |
| `sentai.flow.m4_gray_snap([buf])` | Snapshot 40×30 gray — bytes nou sau în buf preallocat | 0 dacă buf ≥1200B, else 1200 B |

Convenția body-frame e în [`flow_body_frame.md`](flow_body_frame.md).

**Regulă pentru autopilot**: folosește `m4_body_read()` (semn corect
aplicat intern) și poll-ează la 20-50 Hz.  NU folosi `m4_gray_snap()`
fără buffer preallocat într-un loop de producție — 1200 B alocat per
apel fragmentează heap-ul.

---

## 8. Performanță și metrici

Referință pe build #731, cam0 în jos, E26/E30 loop de 20 s:

| Metric | Valoare | Note |
|---|---|---|
| Pipeline TPU fps (cam0 fixed) | 24.70 fps | vs 24.97 baseline = -1.1% |
| Pipeline TPU fps (alt 1:1) | 21.56 fps | vs 21.73 baseline = -0.8% |
| M4 frame rate | ~25 fps | = camera framerate la VGA/45 |
| M7 decimation CPU cost | ~50 µs | 0.2% per frame PrepTask |
| M4 SAD compute | 200-400 µs | DWT cycles / 400 MHz estimate |
| Shared-mem publish→consume latency | <1 ms | poll 1 Hz pe M4 loop |
| REPL `m4_body_read()` roundtrip | <200 µs | pure memory read |

**Obiectivul "pipeline intact"**: satisfăcut.

---

## 9. Diagnostic și debugging

### 9.1 Probleme comune

| Simptom | Cauza probabilă | Fix |
|---|---|---|
| `m4_enable` returnează -3 | Magic neașteptat la 0x202C1000 — M4 nu a pornit sau nu a executat `app_main` | Verifică dacă M4 binary e linkat: `m4_binary_start != 0xdeadbeef` |
| `m4_read()` alive=False după enable OK | Regiune shared nu e non-cacheable; M4 a scris dar M7 vede stale | Verifică `BOARD_ConfigMPU` region 15 în board.c |
| `dy == 0` tot timpul, dx variază | Gray are structură doar pe axa X (scene uniform pe Y) OR mișcare sub 1 grid-px | Snapshot gray via `m4_gray_snap` + inspect; mișcă mai amplu |
| Pipeline fps scade >5% cu flow activ | Decimare mai scumpă decât așteptat (SEMC contention cu TPU) | Măsoară cost-ul hook-ului cu DWT pe M7; considerări PXP vs CPU |
| `last_compute_us` = 800 000 | Field-ul e de fapt **DWT cycles**, nu µs.  Împarte la Hz | Pe M4 @ ~240 MHz boot-default: cycles/240 = µs |

### 9.2 Inspectat vizual gray buffer-ul

```python
buf = bytearray(1200)
sentai.flow.m4_gray_snap(buf)
# Write PGM pentru a-l vedea pe host:
sentai.fs.write('/diags/gray.pgm', b'P5\n40 30\n255\n' + bytes(buf))
```

Apoi descarcă cu `curl http://10.0.0.1/api/raw/diags/gray.pgm` și
vizualizează cu `feh`, `eog`, sau convert cu `pil.Image.open`.

### 9.3 Crash log bridge — TODO

M4 **nu** e conectat la sistemul de crash logging al M7.  Dacă M4
crashează, M7 nu află (magic poate rămâne setat, dar stats se opresc).
Follow-up: un câmp `m4_crash_pc` în shared mem, populat dintr-un
HardFault handler pe M4.

---

## 10. Raport de mapare fișiere

| Fișier | Rol |
|---|---|
| [`flow_shared.h`](../flow_shared.h) | Layout shared struct, adresa fixă, constante FOV |
| [`flow_task_m4.cc`](../flow_task_m4.cc) | Firmware M4: weak main(), DWT init, SAD, protocol |
| [`flow_task.cc`](../flow_task.cc) | M7 side: publish hook, cmd start/stop, heartbeat |
| [`modsentai_flow.c`](../modsentai_flow.c) | Python API `sentai.flow.*` |
| [`detection_task.cc`](../detection_task.cc) | PrepTask hook inserting publish call |
| [`sentai_runtime.cc`](../sentai_runtime.cc) | `sentai_flow_m4_enable()` — opt-in bring-up |

Roadmap docs adiacente:
- [`flow_body_frame.md`](flow_body_frame.md) — axe body-frame și semne
- [`flow_altitude.md`](flow_altitude.md) — altitudine din flow + IMU

---

## 11. Design decisions documentate

1.   **De ce M4 și nu PXP second pass** — PXP e owner-ship single
     (atât de PrepTask) și un al doilea pass ar serializa cu primul.
     CPU decimation e ~50 µs, negligible.  PXP e rezervat pentru
     TPU input scaling.
2.   **De ce 40×30 și nu 80×60** — rpmsg shared window are 4 KB
     disponibile la offset 0x1000.  80×60 = 4800 B + header 96 B >
     4 KB.  40×30 = 1200 B încape confortabil.  Trade-off: rezoluție
     grossieră (step-16 pe raw 640×480).  Fix viitor posibil: mută
     buffer-ul în SDRAM cu MPU configurat non-cacheable.
3.   **De ce shared magic și nu RPMsg** — IpcM4::Init() face o
     negociere RPMsg care necesită `BOARD_InitBootPins` pe M4 (care
     ne-ar sparge camera).  Shared magic e mai simplu și suficient
     pentru alive detection.  RPMsg ar fi necesar doar pentru
     mesagerie bidirecțională complexă, care aici nu există.
4.   **De ce `sentai.flow` și `sentai.flow.m4_*`** — nume separat
     pentru două arhitecturi distincte:
     - `sentai.flow.start/stop/read/stats` rulează SAD pe M7 (legacy,
       folosit în E25); mutually exclusive cu pipeline-ul
     - `sentai.flow.m4_*` rulează SAD pe M4 în paralel cu pipeline-ul
     Cele două nu ar trebui combinate simultan.

---

## 12. Historical notes

- **Build #720-#726** — prima implementare completă: M4 boot, shared
  mem, PrepTask hook, SAD.  E26 confirmă pipeline intact (24.66 vs
  24.97 baseline), dar dy raportat 0 frecvent din cauza decimării
  prea agresive + mișcare user mică.
- **Build #730** — audit NASA/JPL identifică 5 probleme: 2 CRITICAL
  (memory ordering, bounds) și 3 MAJOR (cmd race, flag race, re-
  entrancy).  Toate fixate.  DWT cycles înlocuiesc tick-based µs
  (care erau bogus din cauza skip-ului `BOARD_InitBootClocks`).
  `m4_gray_snap` primește cale zero-alloc.  `sentai.diag.lfs_stats()`
  adăugat pentru depanare busy.
- **Build #731** — stabilizare.  Pipeline + flow în paralel la fps
  baseline, shared mem cu barrier-e corecte pe ambele sensuri.
