# SentAI Runtime — Task Architecture & MicroPython Bridge

## FreeRTOS pe Coral Micro

Runtime-ul rulează pe **FreeRTOS** cu 5 niveluri de prioritate (0–4).
Fiecare subsistem hardware (cameră, TPU, USB, consolă) este încapsulat
într-un **task singleton** care comunică prin cozi FreeRTOS.

---

## QueueTask — Pattern-ul Singleton

Biblioteca coralmicro definește un template `QueueTask<Request, Response, ...>`
care standardizează modul în care task-urile hardware funcționează:

```cpp
template <typename Request, typename Response, ...>
class QueueTask {
    void Init() {
        request_queue_ = xQueueCreate(QueueLength, sizeof(Request));
        xTaskCreateStatic(StaticTaskMain, Name, StackDepth, this, Priority, ...);
    }
    // Sync: creează semafor binar, trimite request, blochează pentru response
    Response SendRequest(Request& req);
    // Async: doar pune în coadă (safe din ISR)
    void SendRequestAsync(Request& req);

    void TaskMain() {
        TaskInit();
        while (true) {
            xQueueReceive(request_queue_, &msg, portMAX_DELAY);
            RequestHandler(&msg);  // virtual — implementat de fiecare subclasă
        }
    }
};
```

**Principiu:** apelantul (de ex. app_main) nu accesează direct hardware-ul.
Trimite un `Request` în coadă, task-ul hardware îl procesează și returnează
un `Response` prin semafor binar.

---

## Task-uri Singleton din Bibliotecă

| Clasă | Task name | Prioritate | Stack | Rol |
|-------|-----------|------------|-------|-----|
| `CameraTask` | `camera_task` | 4 | ×10 | CSI DMA, PXP, frame buffers |
| `EdgeTpuTask` | `edgetpu_task` | 3 | ×3 | TPU state machine, power |
| `EdgeTpuDfuTask` | `edgetpu_dfu_task` | 3 | ×3 | TPU firmware update |
| `UsbDeviceTask` | `usb_device_task` | 4 | ×10 | USB composite device loop |
| `ConsoleM7` | `m7_console_task_tx` | 3 | ×10 | Printf → USB/UART |
| — | `m7_console_task_rx` | 3 | ×10 | UART input |
| — | `m4_console_task` | 3 | ×10 | Forwarding M4 core |
| `PmicTask` | `pmic_task` | 4 | ×10 | Power management IC |

### CameraTask — detaliu

```cpp
class CameraTask : public QueueTask<camera::Request, camera::Response,
                                     "camera_task", STACK*10, prio=4, queue=4> {
    static CameraTask* GetSingleton();
    bool SetPower(bool);          // request kPower
    bool Enable(CameraMode);      // request kEnable
    void GetFrame(vector<CameraFrameFormat>&);  // request kFrame
    uint32_t* GetRawFrame(int*);  // request kFrame (raw)
    bool SwitchCamera(SwitchCameraId);  // request kSwitchCamera
};
```

Orice apel (de ex. `GetFrame`) se traduce într-un `SendRequest` sincron:
apelantul blochează până când task-ul camera procesează cererea pe task-ul
lui dedicat (prioritate 4).

---

## Task-uri SentAI Runtime

Pe lângă task-urile de bibliotecă, sentai_runtime creează task-uri proprii:

### Harta completă la runtime

```text
Prioritate 4:  camera_task, usb_device_task, pmic_task, IPC
Prioritate 3:  console_tx/rx, edgetpu_task, det_infer, app_main(parked)
Prioritate 2:  det_prep, ctrlc, tap_poll
Prioritate 1:  mp_repl, btn_usb
Prioritate 0:  IDLE
Timer:         Tmr Svc (prio 4), mainpy_to (one-shot 30s)
```

### Descriere per task

| Task | Fișier | Stack | Prio | Descriere |
|------|--------|-------|------|-----------|
| `app_main` | `sentai_runtime.cc` | — | 3 | Boot, init TPU, apoi `vTaskSuspend` |
| `btn_usb` | `sentai_runtime.cc` | ×4 | 1 | ISR buton → dezactivează USB drive |
| `mp_repl` | `micropython_task.c` | 16 KB | 1 | REPL interactiv + main.py |
| `ctrlc` | `micropython_task.c` | 512 B | 2 | Polling Ctrl+C pe serial |
| `det_prep` | `detection_task.cc` | ×8 | 2 | Camera → PXP → quant → staging |
| `det_infer` | `detection_task.cc` | ×12 | 3 | TPU invoke → NMS → queue |
| `tap_poll` | `modsentai_hal.cc` | 512 B | 2 | IMU tap gesture polling |
| `link_rx` | `sentai_link.cc` | 6 KB | — | MAVLink UART RX |
| `mesh_rx` | `sentai_mesh.cc` | 4 KB | — | Mesh radio RX |
| `crazy_rx` | `sentai_crazy.cc` | — | — | CrazyFlie RX |
| `crazy_cmd` | `sentai_crazy.cc` | — | — | CrazyFlie commands |

**HTTP server** (`sentai_httpd.cc`) — **nu** are task propriu; rulează pe
thread-ul lwIP `tcpip` (single-threaded).

---

## Comunicare între task-uri

### Mecanisme folosite

| Mecanism | Unde |
|----------|------|
| **FreeRTOS Queue** (typed) | CameraTask, EdgeTpuTask, ConsoleM7, detection results |
| **Binary Semaphore** | `staging_free` / `prep_done` (detection pipeline) |
| **Task Notification** | ISR buton → `btn_usb` (`vTaskNotifyGiveFromISR`) |
| **Shared volatile globals** | `g_tpu_ready`, `g_interpreter`, `g_cam_switch_pending` |
| **Stream Buffer** | Console log tee (`log_pipe_`) |
| **Mutex** | Console RX (`rx_mutex_`), USB serial (`usb_rx_sem_`) |

### Pipeline de detecție — semaphore handoff

```text
PrepTask                              InferTask
────────                              ─────────
take(staging_free)  ←──────────┐
cam_get_raw()                  │
PXP → staging_buf              │
uint8→int8 quant               │      take(prep_done)
give(prep_done) ──────────────►│      memcpy staging→tensor
                               │      give(staging_free) ──►┐
(ciclu: pregătește N+1)        │      TPU Invoke()          │
                               └───── NMS → Queue           │
                               ┌─────────────────────────────┘
                               ▼
                               (repeat)
```

### Protecție TPU

```cpp
extern "C" int sentai_tpu_invoke(void) {
    if (sentai_detection_is_running()) return -10;  // pipeline owns TPU
    return sentai_tpu_invoke_internal();
}
```

Python `sentai.tpu.invoke()` verifică dacă pipeline-ul de detecție rulează.
Dacă da, refuză invocarea (TPU-ul e ocupat de `det_infer`).

---

## Bridge-ul MicroPython → C/C++

### Arhitectura pe 3 straturi

```text
Python REPL              modsentai_*.c (C)             C++ implementation
───────────              ──────────────────             ──────────────────
sentai.camera.init(1)  → mod_sentai_cam_init()        → sentai_cam_init(1)
                          mp_obj_get_int(args[0])         CameraTask::SetPower()
                          MP_DEFINE_CONST_FUN_OBJ_1       CameraTask::Enable()
                          return mp_obj_new_int(rc)       SwitchCamera() × 2
```

### Pattern concret

**1. Funcția Python** — definită în `modsentai_*.c`:

```c
// modsentai_camera.c
static mp_obj_t mod_sentai_cam_init(mp_obj_t streaming_obj) {
    int streaming = mp_obj_get_int(streaming_obj);
    int rc = sentai_cam_init(streaming);     // apel extern C
    if (rc < 0) mp_raise_msg(&mp_type_OSError, ...);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_cam_init_obj, mod_sentai_cam_init);
```

**2. Declarația extern C** — în `modsentai.c`:

```c
extern int sentai_cam_init(int streaming);
extern int sentai_tpu_invoke(void);
extern int sentai_usb_drive_set(int on);
// ... toate funcțiile bridge
```

**3. Implementarea C++** — în `sentai_runtime.cc` sau `modsentai_hal.cc`:

```cpp
extern "C" int sentai_cam_init(int streaming) {
    auto* cam = CameraTask::GetSingleton();  // acces la singleton
    cam->SetPower(true);                     // SendRequest sincron
    cam->Enable(mode);                       // SendRequest sincron
    cam->SwitchCamera(kCameraBack);          // SendRequest sincron
    cam->SwitchCamera(kCameraFront);
    return 0;
}
```

### De ce 3 straturi?

- **MicroPython API** (`modsentai_*.c`) — cod C pur, face marshalling
  `mp_obj_t` ↔ tipuri C, gestionează excepții Python
- **`extern "C"` bridge** — permite ca C-ul din MicroPython să apeleze C++
  fără name mangling
- **C++ implementation** — acces la singleton-uri, shared_ptr, templates,
  STL (vector, etc.)

### Compilare: totul este `#include`

Toate fișierele `modsentai_*.c` sunt **incluse** din `modsentai.c`:

```c
// modsentai.c
#include "modsentai_fs.c"
#include "modsentai_camera.c"
#include "modsentai_usb.c"
#include "modsentai_io.c"
#include "modsentai_tfl.c"
#include "modsentai_rtos.c"
// ... etc.
```

Nu sunt compilate separat — rezultă o singură unitate de traducere.

---

## Modulul `sentai.rtos` — Introspecție Python

MicroPython poate inspecta și controla task-urile FreeRTOS:

```python
>>> sentai.rtos.tasks()
[('camera_task', 'Blocked', 4, 1856),
 ('mp_repl',     'Running', 1, 2048),
 ('det_prep',    'Blocked', 2, 512),
 ('det_infer',   'Blocked', 3, 768),
 ('IDLE',        'Ready',   0, 112)]

>>> sentai.rtos.cpu_usage()
{'det_infer': 42.1, 'det_prep': 8.3, 'camera_task': 5.2, ...}

>>> sentai.rtos.heap_info()
{'freertos_free': 245760, 'mp_gc_used': 12288, 'mp_gc_free': 53248}

>>> sentai.rtos.suspend('det_prep')   # oprește detection pipeline
>>> sentai.rtos.resume('det_prep')    # repornește
```

**Task-uri protejate** (nu pot fi suspendate): `mp_repl`, `IDLE`, `Tmr Svc`,
`ctrlc`, `console`, `usb_dev`.

---

## Diagrama completă

```text
┌─────────────────────────────────────────────────────────────────┐
│                        FreeRTOS Scheduler                       │
│                                                                 │
│  Prio 4 ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│          │ camera   │ │ usb_dev  │ │ pmic     │ │ IPC      │   │
│          └────┬─────┘ └──────────┘ └──────────┘ └──────────┘   │
│               │ Queue                                           │
│  Prio 3 ┌────┴─────┐ ┌──────────┐ ┌──────────┐                │
│          │console_tx│ │edgetpu   │ │det_infer │◄── sem_prep    │
│          └──────────┘ └──────────┘ └─────┬────┘    _done       │
│                                          │ Queue                │
│  Prio 2 ┌──────────┐ ┌──────────┐       │                     │
│          │ det_prep │─── sem_staging     │                     │
│          └──────────┘ │  ctrlc   │       │                     │
│                       └──────────┘       │                     │
│  Prio 1 ┌──────────┐ ┌──────────┐       │                     │
│          │ mp_repl  │ │ btn_usb  │◄── TaskNotify (ISR)         │
│          └─────┬────┘ └──────────┘                              │
│                │                                                │
│                ▼                                                │
│     modsentai_*.c  ──extern "C"──►  sentai_runtime.cc (C++)    │
│     (MP arg marshal)                (singleton access)          │
└─────────────────────────────────────────────────────────────────┘
```

## Surse relevante

| Fișier | Rol |
|--------|-----|
| `libs/base/queue_task.h` | Template QueueTask — baza singleton-urilor |
| `libs/base/tasks.h` | Constante prioritate per subsistem |
| `libs/camera/camera.h` | CameraTask: SetPower, Enable, GetFrame, SwitchCamera |
| `libs/tpu/edgetpu_task.h` | EdgeTpuTask: state machine, power |
| `libs/base/console_m7.h` | ConsoleM7: TX/RX tasks, REPL target |
| `sentai_runtime.cc` | app_main, btn_usb, TPU globals, _write override |
| `micropython_task.c` | mp_repl, ctrlc, safe boot, main.py timeout |
| `detection_task.cc` | det_prep + det_infer, double-buffered pipeline |
| `modsentai.c` | #include all modsentai_*.c, extern C declarations |
| `modsentai_rtos.c` | sentai.rtos.* — tasks(), cpu_usage(), suspend/resume |
