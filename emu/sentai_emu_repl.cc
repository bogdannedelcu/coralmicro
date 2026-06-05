// B8.3 ARM emulator REPL spike.
//
// Boots CM7 startup -> ARM FreeRTOS -> single REPL task on LPUART6 -> minimal
// MicroPython that can evaluate `1+1` and print `2`. No filesystem, no
// `sentai.*` hardware bindings, no USB CDC, no camera.
//
// The REPL loop is intentionally cherry-picked from
// `examples/sentai_runtime/micropython_task.c` (single-line variant only).
// History, escape sequences, multi-line blocks, Ctrl+C handling and the
// watchdog hook are all deferred — B8.3 only proves that:
//
//   * LPUART6 RX delivers bytes into mp_hal_stdin_rx_chr;
//   * mp_embed_init reaches a working VM;
//   * mp_embed_exec_str compiles and runs a single-line Python expression;
//   * mp_hal_stdout_tx_strn_cooked drives LPUART6 TX out to the file backend.

#include <stdint.h>
#include <string.h>

extern "C" {
#include "py/compile.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/stackctrl.h"
#include "port/micropython_embed.h"
}

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#if SENTAI_EMU_FX_FS
#include "libs/base/fx_user_fs.h"
#endif

extern "C" void sentai_emu_uart_init(void);

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;
extern "C" volatile uint32_t g_sentai_emu_last_tick;
extern "C" volatile uint32_t g_sentai_emu_repl_lines;
#if SENTAI_EMU_FX_FS
extern "C" volatile uint32_t g_sentai_emu_fs_smoke_ok;
extern "C" volatile uint32_t g_sentai_emu_fs_smoke_size;
#endif
#if SENTAI_EMU_PREP_RUNTIME
extern "C" void sentai_prep_init(void) __attribute__((weak));
#endif
#if SENTAI_EMU_GAZEBO_CAMERA_BRIDGE
extern "C" int sentai_emu_gazebo_camera_bridge_start(void);
#endif

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootTaskCreated = 0x0200;
constexpr uint32_t kBootReplRunning = 0x0400;
constexpr uint32_t kBootReplBanner = 0x0500;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootCreateTaskFailed = 0xEF00;

constexpr size_t kReplStackWords = 8 * 1024;        // 32 KiB stack
constexpr size_t kReplLineMax = 256;
#if SENTAI_EMU_FX_FS
constexpr size_t kGcHeapBytes = 128 * 1024;         // FS script + list/bytes
#elif SENTAI_EMU_MISSION
constexpr size_t kGcHeapBytes = 96 * 1024;          // larger heap for import
#else
constexpr size_t kGcHeapBytes = 32 * 1024;          // B8.3 minimum
#endif

#if SENTAI_EMU_LARGE_RUNTIME
#define SENTAI_EMU_REPL_BSS __attribute__((section(".sdram_bss"), aligned(8)))
#else
#define SENTAI_EMU_REPL_BSS __attribute__((aligned(8)))
#endif

StaticTask_t g_repl_tcb;
StackType_t g_repl_stack[kReplStackWords] __attribute__((aligned(8)));
uint8_t g_gc_heap[kGcHeapBytes] SENTAI_EMU_REPL_BSS;
char g_repl_line[kReplLineMax];

void ReplPutString(const char *s) {
    mp_hal_stdout_tx_strn(s, strlen(s));
}

int ReplReadLine(char *buf, size_t max_len) {
    size_t pos = 0;
    while (pos + 1 < max_len) {
        int ch = mp_hal_stdin_rx_chr();
        if (ch < 0) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            ReplPutString("\r\n");
            break;
        }
        if (ch == 0x7F || ch == '\b') {
            if (pos > 0) {
                --pos;
                ReplPutString("\b \b");
            }
            continue;
        }
        if (ch >= 0x20 && ch < 0x7F) {
            buf[pos++] = static_cast<char>(ch);
            char echo = static_cast<char>(ch);
            mp_hal_stdout_tx_strn(&echo, 1);
        }
    }
    buf[pos] = '\0';
    return static_cast<int>(pos);
}

void ReplTask(void *) {
    g_sentai_emu_boot_state = kBootReplRunning;
    sentai_emu_uart_init();
    ReplPutString("\r\nSentAI EMU REPL B8.3\r\n");

#if SENTAI_EMU_PREP_RUNTIME
    if (sentai_prep_init) {
        sentai_prep_init();
        ReplPutString("S233_PREP_INIT\r\n");
    }
#endif
#if SENTAI_EMU_GAZEBO_CAMERA_BRIDGE
    {
        int gz_rc = sentai_emu_gazebo_camera_bridge_start();
        ReplPutString(gz_rc == 0 ? "S233_GZ_CAM_START 0\r\n" :
                                   "S233_GZ_CAM_START -1\r\n");
    }
#endif

    // Stack-top hint: use a local variable address inside this task's stack.
    int stack_top_marker = 0;
    mp_stack_set_top(&stack_top_marker);
    mp_stack_set_limit((kReplStackWords - 256) * sizeof(StackType_t));
    mp_embed_init(g_gc_heap, sizeof(g_gc_heap), &stack_top_marker);

#if SENTAI_EMU_MISSION
    // B8.4: make the import machinery look at the in-firmware VFS at
    // emu/sentai_emu_fs.c.  The default sys.path is empty under the embed
    // port, so without this no `import` would ever call mp_import_stat.
    mp_embed_exec_str("import sys\nsys.path.append('')\n");
#endif

#if SENTAI_EMU_FX_FS && !SENTAI_EMU_FS_REPL_SMOKE
    // Mount an existing emulated user partition if present. The smoke script
    // below deliberately calls sentai.fs.format() to prove the destructive
    // path too; this mount keeps the interactive REPL usable without a format
    // when SENTAI_EMU_FS_REPL_SMOKE is disabled.
    (void)FxUserInit(0);
#endif

    ReplPutString("MicroPython embed ready\r\n");

#if SENTAI_EMU_FS_REPL_SMOKE
    mp_embed_exec_str(
        "import sentai\n"
        "print('FS_REPL_BEGIN')\n"
        "print('FS_FORMAT', sentai.fs.format())\n"
        "print('FS_MKDIR', sentai.fs.mkdir('/models'))\n"
        "print('FS_WRITE', sentai.fs.write('/models/a.txt', b'hello '))\n"
        "print('FS_APPEND', sentai.fs.append('/models/a.txt', b'world'))\n"
        "print('FS_SYNC', sentai.fs.sync())\n"
        "print('FS_SIZE', sentai.fs.size('/models/a.txt'))\n"
        "print('FS_READ', sentai.fs.read_str('/models/a.txt'))\n"
        "print('FS_EXISTS', sentai.fs.exists('/models/a.txt'))\n"
        "print('FS_LS', sentai.fs.ls('/models'))\n"
        "print('FS_REPL_DONE')\n");
    const ssize_t smoke_size = FxUserSize("/models/a.txt");
    g_sentai_emu_fs_smoke_size =
        smoke_size < 0 ? 0xFFFFFFFFu : static_cast<uint32_t>(smoke_size);
    g_sentai_emu_fs_smoke_ok =
        (smoke_size == 11 &&
         FxUserFileExists("/models/a.txt") &&
         FxUserDirExists("/models"))
            ? 1u
            : 0u;
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_FS_ASSET_CHECK
    mp_embed_exec_str(
        "import sentai\n"
        "print('FS_ASSET_CHECK_BEGIN')\n"
        "print('MODEL_SIZE', sentai.fs.size('/models/"
        "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite'))\n"
        "print('IMAGE_SIZE', sentai.fs.size('/images/cat_640x480.bmp'))\n"
        "print('MISSION_SIZE', sentai.fs.size('/mission.py'))\n"
        "print('MISSION_HEAD', sentai.fs.read_str('/mission.py')[:20])\n"
        "print('FS_ASSET_CHECK_DONE')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_CAT_AUTORUN
    mp_embed_exec_str("import mission\nmission.run()\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_FPS_AUTORUN
    mp_embed_exec_str("import mission\nmission.fps(5)\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_FPS_MEM_AUTORUN
    mp_embed_exec_str("import mission\nmission.fps_mem(5)\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_FPS_MEM_INVOKE_AUTORUN
    mp_embed_exec_str("import mission\nmission.fps_mem_invoke(5)\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_FPS_MEM_SESSION_AUTORUN
    mp_embed_exec_str("import mission\nmission.fps_mem_session(5)\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_TPU_TIMING_AUTORUN
    mp_embed_exec_str("import mission\nmission.timing_mem_session(3)\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_CRAZY_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "sentai.fr.init()\n"
        "sentai.fr.open('events', '/fr/crazy_events.csv')\n"
        "sentai.fr.open('scalars', '/fr/crazy_scalars.csv')\n"
        "print('CRAZY_MP_BEGIN')\n"
        "sentai.fr.push_event('crazy', 'mp_begin')\n"
        "sentai.crazy.debug(0)\n"
        "rc = sentai.crazy.init(576000)\n"
        "print('CRAZY_MP_INIT', rc)\n"
        "sentai.fr.push_scalar('crazy_init_rc', rc)\n"
        "pm = sentai.crazy.ping(1500)\n"
        "print('CRAZY_MP_PING_MS', pm)\n"
        "sentai.fr.push_scalar('crazy_ping_ms', pm)\n"
        "sr = sentai.crazy.stop()\n"
        "print('CRAZY_MP_STOP', sr)\n"
        "sentai.fr.push_scalar('crazy_stop_rc', sr)\n"
        "sentai.fr.push_event('crazy', 'mp_done')\n"
        "sentai.fr.task_stop()\n"
        "print('CRAZY_MP_DONE')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_CRAZY_CALIB_PRECURSOR_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "print('CALIB_GZ_BEGIN')\n"
        "sentai.fr.init()\n"
        "sentai.fr.open('events', '/fr/s229_events.csv')\n"
        "sentai.fr.open('scalars', '/fr/s229_scalars.csv')\n"
        "sentai.fr.push_event('s229', 'begin')\n"
        "sentai.crazy.debug(0)\n"
        "rc = sentai.crazy.init(576000)\n"
        "print('CALIB_GZ_INIT', rc)\n"
        "sentai.fr.push_scalar('crazy_init_rc', rc)\n"
        "pm = sentai.crazy.ping(1500)\n"
        "print('CALIB_GZ_PING_MS', pm)\n"
        "sentai.fr.push_scalar('crazy_ping_ms', pm)\n"
        "try:\n"
        "    r = sentai.calib.get_R_cam_to_body()\n"
        "    off = sentai.calib.get_cam_offset_B()\n"
        "    is_cal = sentai.calib.is_calibrated()\n"
        "    print('CALIB_GZ_CALIB_STUB', len(r), off, is_cal)\n"
        "    sentai.fr.push_scalar('calib_stub_len_R', len(r))\n"
        "except Exception as e:\n"
        "    print('CALIB_GZ_CALIB_ERR', str(e))\n"
        "sentai.fr.push_event('s229', 'fly_start')\n"
        "frc = sentai.crazy.fly(0.35, 1800, 2200, 2200)\n"
        "print('CALIB_GZ_FLY_RC', frc)\n"
        "sentai.fr.push_scalar('crazy_fly_rc', frc)\n"
        "try:\n"
        "    alt = sentai.crazy.altitude(400)\n"
        "    print('CALIB_GZ_ALT_AFTER', alt)\n"
        "    sentai.fr.push_scalar('alt_after', alt)\n"
        "except Exception as e:\n"
        "    print('CALIB_GZ_ALT_ERR', str(e))\n"
        "sr = sentai.crazy.stop()\n"
        "print('CALIB_GZ_STOP', sr)\n"
        "sentai.fr.push_scalar('crazy_stop_rc', sr)\n"
        "sentai.fr.push_event('s229', 'done')\n"
        "sentai.fr.task_stop()\n"
        "sentai.fs.mkdir('/b9')\n"
        "sentai.fs.write('/b9/s229_status.txt', 'done\\n')\n"
        "sentai.fs.sync()\n"
        "print('CALIB_GZ_DONE')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_CAMERA_MARKERS_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "print('CAM_MARK_BEGIN')\n"
        "sentai.fr.init()\n"
        "sentai.fr.open('events', '/fr/s230_events.csv')\n"
        "sentai.fr.open('scalars', '/fr/s230_scalars.csv')\n"
        "sentai.fs.mkdir('/b9')\n"
        "sentai.fr.push_event('s230', 'begin')\n"
        "model = '/markers/whycon_320x240.pgm'\n"
        "bmp = '/images/whycon_640x480.bmp'\n"
        "print('CAM_MARK_ASSET_PGM', sentai.fs.size(model))\n"
        "print('CAM_MARK_ASSET_BMP', sentai.fs.size(bmp))\n"
        "sentai.markers.clear()\n"
        "rc = sentai.markers.init('whycon')\n"
        "print('CAM_MARK_INIT', rc)\n"
        "sentai.markers.set_intrinsics(240.0, 240.0, 160.0, 120.0)\n"
        "sentai.markers.set_marker_size(0.08)\n"
        "pgm_n = sentai.markers.detect_pgm(model)\n"
        "print('CAM_MARK_PGM_N', pgm_n)\n"
        "print('CAM_MARK_PGM_DET0', sentai.markers.get_detection_tuple(0))\n"
        "sentai.fr.push_scalar('pgm_n', pgm_n)\n"
        "sel = sentai.camera.select(-1, bmp)\n"
        "prep = sentai.camera.prep_once()\n"
        "cam_n = sentai.markers.detect_from_camera()\n"
        "det0 = sentai.markers.get_detection_tuple(0)\n"
        "obs = sentai.markers.get_observation_tuple(320, 240, 0.0)\n"
        "print('CAM_MARK_SELECT', sel)\n"
        "print('CAM_MARK_PREP', prep)\n"
        "print('CAM_MARK_CAM_N', cam_n)\n"
        "print('CAM_MARK_CAM_DET0', det0)\n"
        "print('CAM_MARK_OBS', obs)\n"
        "sentai.fr.push_scalar('cam_n', cam_n)\n"
        "sentai.fr.push_scalar('prep_rc', prep)\n"
        "ok = 0\n"
        "frames = 10\n"
        "t0 = sentai.rtos.ticks_ms()\n"
        "for i in range(frames):\n"
        "    sentai.camera.prep_once()\n"
        "    n = sentai.markers.detect_from_camera()\n"
        "    if n > 0:\n"
        "        ok += 1\n"
        "t1 = sentai.rtos.ticks_ms()\n"
        "dt = t1 - t0\n"
        "fps_x100 = (frames * 100000) // dt if dt > 0 else 0\n"
        "print('CAM_MARK_LOOP', frames, ok, dt, fps_x100)\n"
        "sentai.fr.push_scalar('loop_frames', frames)\n"
        "sentai.fr.push_scalar('loop_ok', ok)\n"
        "sentai.fr.push_scalar('loop_dt_ms', dt)\n"
        "sentai.fr.push_scalar('loop_fps_x100', fps_x100)\n"
        "report = 'pgm_n=%d\\ncam_n=%d\\nloop_ok=%d\\nloop_dt_ms=%d\\nloop_fps_x100=%d\\n' % (pgm_n, cam_n, ok, dt, fps_x100)\n"
        "sentai.fs.write('/b9/s230_camera_markers_report.txt', report)\n"
        "sentai.fs.sync()\n"
        "sentai.fr.push_event('s230', 'done')\n"
        "sentai.fr.task_stop()\n"
        "print('CAM_MARK_DONE')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_GAZEBO_FLOW_WHYCON_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "print('S233_BEGIN')\n"
        "sentai.fr.init()\n"
        "sentai.fr.open('events', '/fr/s233_events.csv')\n"
        "sentai.fr.open('scalars', '/fr/s233_scalars.csv')\n"
        "sentai.fs.mkdir('/b9')\n"
        "sentai.fr.push_event('s233', 'begin')\n"
        "cam_rc = sentai.camera.init(1, 30)\n"
        "sentai.camera.prep_reset()\n"
        "try:\n"
        "    sentai.pipeline.prep_reset()\n"
        "except Exception:\n"
        "    pass\n"
        "gray_ref = sentai.camera.prep_enable(0)\n"
        "flow_ref = sentai.camera.prep_enable(4)\n"
        "try:\n"
        "    prep_fps = sentai.pipeline.prep_fps(30)\n"
        "except Exception:\n"
        "    prep_fps = -1\n"
        "prep_rc = sentai.pipeline.prep_start()\n"
        "flow_rc = sentai.flow.start(-1)\n"
        "sentai.markers.clear()\n"
        "markers_rc = sentai.markers.init('whycon')\n"
        "sentai.markers.set_intrinsics(288.3, 288.3, 160.0, 120.0)\n"
        "sentai.markers.set_marker_size(0.0544)\n"
        "sentai.markers.set_cam_extrinsics(-0.04, 0.0, -0.02, 0.0, 1.5707963, 3.1415927)\n"
        "print('S233_SETUP', cam_rc, gray_ref, flow_ref, prep_fps, prep_rc, flow_rc, markers_rc)\n"
        "sentai.fr.push_scalar('cam_rc', cam_rc)\n"
        "sentai.fr.push_scalar('gray_ref', gray_ref)\n"
        "sentai.fr.push_scalar('flow_ref', flow_ref)\n"
        "sentai.fr.push_scalar('prep_rc', prep_rc)\n"
        "sentai.fr.push_scalar('flow_rc', flow_rc)\n"
        "sentai.fr.push_scalar('markers_rc', markers_rc)\n"
        "frames = 0\n"
        "flow_seq0 = 0\n"
        "flow_seq_last = 0\n"
        "flow_nonzero = 0\n"
        "marker_samples = 0\n"
        "marker_hits = 0\n"
        "marker_best = 0\n"
        "crazy_init = -999\n"
        "crazy_ping = -999\n"
        "crazy_arm = -999\n"
        "crazy_takeoff = -999\n"
        "crazy_hl_stop = -999\n"
        "crazy_land = -999\n"
        "crazy_stop = -999\n"
        "t0 = sentai.rtos.ticks_ms()\n"
        "flow_phase_end = t0 + 3500\n"
        "next_mark = flow_phase_end + 500\n"
        "while sentai.rtos.ticks_ms() < flow_phase_end:\n"
        "    sentai.rtos.sleep_ms(50)\n"
        "    frames += 1\n"
        "    fr = sentai.flow.read()\n"
        "    seq = int(fr.get('frame_seq', 0))\n"
        "    if flow_seq0 == 0 and seq > 0:\n"
        "        flow_seq0 = seq\n"
        "    if seq > flow_seq_last:\n"
        "        flow_seq_last = seq\n"
        "    if int(fr.get('confidence', 0)) > 0:\n"
        "        flow_nonzero += 1\n"
        "print('S233_FLOW_BASELINE', frames, flow_seq_last, flow_nonzero)\n"
        "sentai.fr.push_scalar('flow_base_frames', frames)\n"
        "sentai.fr.push_scalar('flow_base_seq_last', flow_seq_last)\n"
        "try:\n"
        "    sentai.crazy.debug(0)\n"
        "    crazy_init = sentai.crazy.init(576000)\n"
        "    crazy_ping = sentai.crazy.ping(1500)\n"
        "    if crazy_init == 0:\n"
        "        crazy_arm = sentai.crazy.arm()\n"
        "        sentai.rtos.sleep_ms(300)\n"
        "        crazy_takeoff = sentai.crazy.takeoff(0.78, 2.0)\n"
        "        sentai.rtos.sleep_ms(3500)\n"
        "        try:\n"
        "            crazy_hl_stop = sentai.crazy.hl_stop()\n"
        "        except Exception:\n"
        "            crazy_hl_stop = -998\n"
        "        for _ in range(5):\n"
        "            try:\n"
        "                sentai.crazy.hover(0.0, 0.0, 0.0, 0.78)\n"
        "            except Exception:\n"
        "                pass\n"
        "            sentai.rtos.sleep_ms(30)\n"
        "except Exception as e:\n"
        "    print('S233_CRAZY_ERR', str(e))\n"
        "print('S233_CRAZY_SETUP', crazy_init, crazy_ping, crazy_arm, crazy_takeoff, crazy_hl_stop)\n"
        "sentai.fr.push_scalar('crazy_init', crazy_init)\n"
        "sentai.fr.push_scalar('crazy_ping', crazy_ping)\n"
        "sentai.fr.push_scalar('crazy_arm', crazy_arm)\n"
        "sentai.fr.push_scalar('crazy_takeoff', crazy_takeoff)\n"
        "why_t0 = sentai.rtos.ticks_ms()\n"
        "next_mark = why_t0\n"
        "while sentai.rtos.ticks_ms() - why_t0 < 9000:\n"
        "    try:\n"
        "        sentai.crazy.hover(0.0, 0.0, 0.0, 0.78)\n"
        "    except Exception:\n"
        "        pass\n"
        "    sentai.rtos.sleep_ms(80)\n"
        "    frames += 1\n"
        "    fr = sentai.flow.read()\n"
        "    seq = int(fr.get('frame_seq', 0))\n"
        "    if flow_seq0 == 0 and seq > 0:\n"
        "        flow_seq0 = seq\n"
        "    if seq > flow_seq_last:\n"
        "        flow_seq_last = seq\n"
        "    if int(fr.get('confidence', 0)) > 0:\n"
        "        flow_nonzero += 1\n"
        "    now = sentai.rtos.ticks_ms()\n"
        "    if now >= next_mark:\n"
        "        marker_samples += 1\n"
        "        try:\n"
        "            n = sentai.markers.detect_from_camera()\n"
        "        except Exception as e:\n"
        "            n = -1\n"
        "            print('S233_MARK_ERR', str(e))\n"
        "        if n > 0:\n"
        "            marker_hits += 1\n"
        "            if n > marker_best:\n"
        "                marker_best = n\n"
        "        print('S233_SAMPLE', frames, seq, fr.get('dx', 0), fr.get('dy', 0), fr.get('confidence', 0), n)\n"
        "        next_mark = now + 300\n"
        "t1 = sentai.rtos.ticks_ms()\n"
        "pst = sentai.pipeline.prep_stats()\n"
        "fst = sentai.flow.pub_stats()\n"
        "fperf = sentai.flow.perf()\n"
        "try:\n"
        "    crazy_land = sentai.crazy.land(0.0, 2.0)\n"
        "    sentai.rtos.sleep_ms(2400)\n"
        "except Exception as e:\n"
        "    print('S233_LAND_ERR', str(e))\n"
        "try:\n"
        "    crazy_stop = sentai.crazy.stop()\n"
        "except Exception:\n"
        "    crazy_stop = -998\n"
        "print('S233_CRAZY_DONE', crazy_land, crazy_stop)\n"
        "sentai.flow.stop()\n"
        "sentai.pipeline.prep_stop()\n"
        "sentai.camera.prep_disable(0)\n"
        "sentai.camera.prep_disable(4)\n"
        "dt = t1 - t0\n"
        "flow_frames = int(fst.get('frames', 0))\n"
        "flow_fps_x100 = (flow_frames * 100000) // dt if dt > 0 else 0\n"
        "prep_frames = int(pst.get('frames', 0))\n"
        "prep_fps_x100 = (prep_frames * 100000) // dt if dt > 0 else 0\n"
        "print('S233_SUMMARY', dt, prep_frames, prep_fps_x100, flow_frames, flow_fps_x100, marker_samples, marker_hits, marker_best, flow_seq_last)\n"
        "sentai.fr.push_scalar('dt_ms', dt)\n"
        "sentai.fr.push_scalar('prep_frames', prep_frames)\n"
        "sentai.fr.push_scalar('prep_fps_x100', prep_fps_x100)\n"
        "sentai.fr.push_scalar('flow_frames', flow_frames)\n"
        "sentai.fr.push_scalar('flow_fps_x100', flow_fps_x100)\n"
        "sentai.fr.push_scalar('marker_samples', marker_samples)\n"
        "sentai.fr.push_scalar('marker_hits', marker_hits)\n"
        "sentai.fr.push_scalar('marker_best', marker_best)\n"
        "sentai.fr.push_scalar('flow_seq_last', flow_seq_last)\n"
        "sentai.fr.push_event('s233', 'done')\n"
        "sentai.fr.task_stop()\n"
        "report = 'dt_ms=%d\\nprep_frames=%d\\nprep_fps_x100=%d\\nflow_frames=%d\\nflow_fps_x100=%d\\nmarker_samples=%d\\nmarker_hits=%d\\nmarker_best=%d\\nflow_seq_last=%d\\n' % (dt, prep_frames, prep_fps_x100, flow_frames, flow_fps_x100, marker_samples, marker_hits, marker_best, flow_seq_last)\n"
        "sentai.fs.write('/b9/s233_gazebo_flow_whycon_report.txt', report)\n"
        "sentai.fs.sync()\n"
        "print('S233_DONE')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_NAMESPACE_INVENTORY_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "lines = []\n"
        "def emit(s):\n"
        "    print(s)\n"
        "    lines.append(s)\n"
        "def clean(xs):\n"
        "    ys = []\n"
        "    for x in xs:\n"
        "        if len(x) and x[0] != '_':\n"
        "            ys.append(x)\n"
        "    ys.sort()\n"
        "    return ys\n"
        "def csv(xs):\n"
        "    return ','.join(clean(xs))\n"
        "def ns_line(name):\n"
        "    root = clean(dir(sentai))\n"
        "    if name in root:\n"
        "        try:\n"
        "            emit('NS|' + name + '|present|' + csv(dir(getattr(sentai, name))))\n"
        "        except Exception as e:\n"
        "            emit('NS|' + name + '|error|' + str(e))\n"
        "    else:\n"
        "        emit('NS|' + name + '|missing|')\n"
        "emit('INV_BEGIN')\n"
        "try:\n"
        "    emit('VERSION|' + str(sentai.version()))\n"
        "except Exception as e:\n"
        "    emit('VERSION_ERR|' + str(e))\n"
        "emit('ROOT|' + csv(dir(sentai)))\n"
        "for name in ['fs','fr','rtos','sys','io','tpu','pipeline','crazy',"
        "'camera','flow','markers','usb','uart','link','mesh','imu','mic',"
        "'sleep','safety','objects','places','slam','explore','tfl','aifes',"
        "'kmeans','pca','anomaly','dtw','hmm','rl','servo','object_lifter',"
        "'calib']:\n"
        "    ns_line(name)\n"
        "for topic in ['all','fs','fr','rtos','tpu','pipeline','crazy','sys','io']:\n"
        "    try:\n"
        "        emit('HELP|' + topic + '|begin')\n"
        "        if topic == 'all':\n"
        "            sentai.help()\n"
        "        else:\n"
        "            sentai.help(topic)\n"
        "        emit('HELP|' + topic + '|ok')\n"
        "    except Exception as e:\n"
        "        emit('HELP|' + topic + '|err|' + str(e))\n"
        "try:\n"
        "    try:\n"
        "        sentai.fs.ls('/')\n"
        "    except Exception:\n"
        "        sentai.fs.format()\n"
        "    sentai.fs.mkdir('/b9')\n"
        "    sentai.fs.write('/b9/s216_inventory.txt', '\\n'.join(lines) + '\\n')\n"
        "    sentai.fs.sync()\n"
        "    emit('FS_WRITE|' + str(sentai.fs.size('/b9/s216_inventory.txt')))\n"
        "except Exception as e:\n"
        "    emit('FS_WRITE_ERR|' + str(e))\n"
        "emit('INV_END')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_CORE_SMOKE_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "lines = []\n"
        "fails = 0\n"
        "def emit(s):\n"
        "    print(s)\n"
        "    lines.append(s)\n"
        "def check(name, cond, detail=''):\n"
        "    global fails\n"
        "    if cond:\n"
        "        emit('CHECK|' + name + '|PASS|' + str(detail))\n"
        "    else:\n"
        "        fails += 1\n"
        "        emit('CHECK|' + name + '|FAIL|' + str(detail))\n"
        "def call_ok(name, fn):\n"
        "    global fails\n"
        "    try:\n"
        "        r = fn()\n"
        "        emit('CHECK|' + name + '|PASS|' + str(r))\n"
        "        return r\n"
        "    except Exception as e:\n"
        "        fails += 1\n"
        "        emit('CHECK|' + name + '|FAIL|' + str(e))\n"
        "        return None\n"
        "emit('CORE_BEGIN')\n"
        "check('version', sentai.version() == 'SentAI EMU B8', sentai.version())\n"
        "prev = call_ok('verbose_set', lambda: sentai.verbose(0))\n"
        "call_ok('verbose_restore', lambda: sentai.verbose(prev))\n"
        "call_ok('debug', lambda: sentai.debug(1))\n"
        "check('console_default', sentai.console() in ('uart','usb'), sentai.console())\n"
        "check('console_uart', sentai.console('uart') == 'uart', sentai.console())\n"
        "call_ok('io_led_on', lambda: sentai.io.led_on())\n"
        "call_ok('io_led_off', lambda: sentai.io.led_off())\n"
        "check('sys_recovery', sentai.sys.recovery_mode() == False, sentai.sys.recovery_mode())\n"
        "check('sys_boot_attempts', sentai.sys.boot_attempts() == 0, sentai.sys.boot_attempts())\n"
        "t0 = sentai.rtos.ticks_ms()\n"
        "sentai.rtos.sleep_ms(25)\n"
        "sentai.rtos.repl_kick()\n"
        "t1 = sentai.rtos.ticks_ms()\n"
        "check('rtos_ticks_progress', t1 >= t0, str(t0) + '->' + str(t1))\n"
        "check('rtos_uptime', sentai.rtos.uptime() >= 0, sentai.rtos.uptime())\n"
        "sentai.fs.mkdir('/b9')\n"
        "sentai.fs.remove('/b9/s218.txt')\n"
        "sentai.fs.write('/b9/s218.txt', 'hello ')\n"
        "sentai.fs.append('/b9/s218.txt', 'world')\n"
        "check('fs_exists', sentai.fs.exists('/b9/s218.txt'), sentai.fs.size('/b9/s218.txt'))\n"
        "check('fs_size', sentai.fs.size('/b9/s218.txt') == 11, sentai.fs.size('/b9/s218.txt'))\n"
        "check('fs_read_str', sentai.fs.read_str('/b9/s218.txt') == 'hello world', sentai.fs.read_str('/b9/s218.txt'))\n"
        "check('fs_read_bytes', sentai.fs.read('/b9/s218.txt') == b'hello world', sentai.fs.read('/b9/s218.txt'))\n"
        "check('fs_b64', sentai.fs.read_base64('/b9/s218.txt') == 'aGVsbG8gd29ybGQ=', sentai.fs.read_base64('/b9/s218.txt'))\n"
        "check('fs_ls', len(sentai.fs.ls('/b9')) >= 1, sentai.fs.ls('/b9'))\n"
        "sentai.fs.write('/b9/s218_child.py', \"import sentai\\nsentai.fs.write('/b9/s218_run.txt', 'run-ok')\\n\")\n"
        "sentai.run('/b9/s218_child.py')\n"
        "check('sentai_run', sentai.fs.read_str('/b9/s218_run.txt') == 'run-ok', sentai.fs.read_str('/b9/s218_run.txt'))\n"
        "sentai.fr.init()\n"
        "check('fr_open_events', sentai.fr.open('events','/fr/s218_events.csv') == 0, sentai.fr.stats('events'))\n"
        "check('fr_open_scalars', sentai.fr.open('scalars','/fr/s218_scalars.csv') == 0, sentai.fr.stats('scalars'))\n"
        "check('fr_event', sentai.fr.push_event('s218','core') == 0, sentai.fr.stats('events'))\n"
        "check('fr_scalar', sentai.fr.push_scalar('core_ok', 1) == 0, sentai.fr.stats('scalars'))\n"
        "sentai.fr.task_stop()\n"
        "check('fr_events_file', sentai.fs.size('/fr/s218_events.csv') > 0, sentai.fs.size('/fr/s218_events.csv'))\n"
        "check('fr_scalars_file', sentai.fs.size('/fr/s218_scalars.csv') > 0, sentai.fs.size('/fr/s218_scalars.csv'))\n"
        "emit('CORE_FAILS|' + str(fails))\n"
        "sentai.fs.write('/b9/s218_core_report.txt', '\\n'.join(lines) + '\\n')\n"
        "sentai.fs.sync()\n"
        "emit('FS_WRITE|' + str(sentai.fs.size('/b9/s218_core_report.txt')))\n"
        "emit('CORE_END')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

#if SENTAI_EMU_HW_STUB_SMOKE_AUTORUN
    mp_embed_exec_str(
        "import sentai\n"
        "lines = []\n"
        "fails = 0\n"
        "def emit(s):\n"
        "    print(s)\n"
        "    lines.append(s)\n"
        "def check(name, cond, detail=''):\n"
        "    global fails\n"
        "    if cond:\n"
        "        emit('CHECK|' + name + '|PASS|' + str(detail))\n"
        "    else:\n"
        "        fails += 1\n"
        "        emit('CHECK|' + name + '|FAIL|' + str(detail))\n"
        "def call(name, fn):\n"
        "    global fails\n"
        "    try:\n"
        "        r = fn()\n"
        "        emit('CHECK|' + name + '|PASS|' + str(r))\n"
        "        return r\n"
        "    except Exception as e:\n"
        "        fails += 1\n"
        "        emit('CHECK|' + name + '|FAIL|' + str(e))\n"
        "        return None\n"
        "emit('HW_BEGIN')\n"
        "sentai.fr.init()\n"
        "sentai.fr.open('events','/fr/s228_events.csv')\n"
        "sentai.fr.open('scalars','/fr/s228_scalars.csv')\n"
        "sentai.fr.push_event('s228','begin')\n"
        "check('usb_drive_off', sentai.usb.drive(0) == 0)\n"
        "check('usb_drive_on_rejected', sentai.usb.drive(1) < 0)\n"
        "check('usb_ip_unavailable', sentai.usb.ip(0) < 0)\n"
        "check('usb_open_false', sentai.usb.open() == False)\n"
        "check('usb_available_zero', sentai.usb.available() == 0)\n"
        "check('usb_read_empty', sentai.usb.read(8, 0) == b'')\n"
        "sentai.usb.close()\n"
        "check('uart_open_false', sentai.uart.open(576000) == False)\n"
        "check('uart_available_zero', sentai.uart.available() == 0)\n"
        "check('uart_read_empty', sentai.uart.read(8, 0) == b'')\n"
        "sentai.uart.close()\n"
        "check('imu_init', sentai.imu.init() == 0)\n"
        "imu = sentai.imu.read()\n"
        "check('imu_sample', imu['x'] == 0 and imu['y'] == 0 and imu['z'] == 1000, imu)\n"
        "deg = sentai.imu.degrees()\n"
        "check('imu_degrees', deg['pitch'] == 0 and deg['roll'] == 0, deg)\n"
        "check('imu_tap_start_rejected', sentai.imu.tap_start() < 0)\n"
        "check('imu_tap_stop', sentai.imu.tap_stop() == 0)\n"
        "check('imu_poll_none', sentai.imu.poll_event(0) == None)\n"
        "check('mic_start_rejected', sentai.mic.start(1) < 0)\n"
        "check('mic_recording_false', sentai.mic.recording() == False)\n"
        "check('mic_samples_zero', sentai.mic.samples() == 0)\n"
        "check('mic_level_zero', sentai.mic.level() == 0)\n"
        "check('mic_stop', sentai.mic.stop() == 0)\n"
        "check('mic_save_none', sentai.mic.save_mp3() == None)\n"
        "t0 = sentai.rtos.ticks_ms()\n"
        "idle_rc = sentai.sleep.idle(70, 5, 0)\n"
        "t1 = sentai.rtos.ticks_ms()\n"
        "check('sleep_idle_timeout', idle_rc == 0 and t1 >= t0, str(t0) + '->' + str(t1))\n"
        "check('servo_init_sim', sentai.servo.init('sim') == 0)\n"
        "check('servo_arm_rejected', sentai.servo.arm() < 0)\n"
        "s = sentai.servo.status()\n"
        "check('servo_safe_status', s['armed'] == 0 and s['flight'] == sentai.servo.GROUND, s)\n"
        "check('servo_disarm', sentai.servo.disarm() == 0)\n"
        "check('servo_pose_none', sentai.servo.pose() == None)\n"
        "check('servo_pose_not_ready', sentai.servo.pose_ready() == 0)\n"
        "check('calib_identity_R', len(sentai.calib.get_R_cam_to_body()) == 9)\n"
        "check('calib_zero_offset', sentai.calib.get_cam_offset_B() == (0,0,0))\n"
        "check('calib_not_calibrated', sentai.calib.is_calibrated() == False)\n"
        "kr = sentai.calib.run_kabsch([])\n"
        "check('calib_kabsch_stub', len(kr[0]) == 9 and kr[1]['accepted'] == False, kr)\n"
        "check('lifter_count_zero', sentai.object_lifter.count() == 0)\n"
        "check('lifter_list_empty', sentai.object_lifter.list() == [])\n"
        "check('lifter_stats_zero', sentai.object_lifter.stats()['used'] == 0)\n"
        "check('lifter_clear', sentai.object_lifter.clear() == 0)\n"
        "check('safety_init', sentai.safety.init() == 0)\n"
        "check('safety_clear', sentai.safety.clear() == 0)\n"
        "check('safety_not_aborted', sentai.safety.aborted() == False)\n"
        "check('safety_task_start_rejected', sentai.safety.task_start() < 0)\n"
        "check('safety_task_stop', sentai.safety.task_stop() == 0)\n"
        "check('safety_enable_markers_rejected', sentai.safety.enable_markers() < 0)\n"
        "sentai.safety._test_push_aruco(3, 1, 0)\n"
        "check('safety_test_abort', sentai.safety.aborted() == True and sentai.safety.reason() == 'markers')\n"
        "sentai.fr.push_scalar('s228_fails', fails)\n"
        "sentai.fr.push_event('s228','done')\n"
        "sentai.fr.task_stop()\n"
        "check('fr_events_file', sentai.fs.size('/fr/s228_events.csv') > 0, sentai.fs.size('/fr/s228_events.csv'))\n"
        "check('fr_scalars_file', sentai.fs.size('/fr/s228_scalars.csv') > 0, sentai.fs.size('/fr/s228_scalars.csv'))\n"
        "emit('HW_FAILS|' + str(fails))\n"
        "sentai.fs.mkdir('/b9')\n"
        "sentai.fs.write('/b9/s228_hw_stub_report.txt', '\\n'.join(lines) + '\\n')\n"
        "sentai.fs.sync()\n"
        "emit('FS_WRITE|' + str(sentai.fs.size('/b9/s228_hw_stub_report.txt')))\n"
        "emit('HW_END')\n");
    ++g_sentai_emu_repl_lines;
    ++g_sentai_emu_heartbeat;
    g_sentai_emu_last_tick = xTaskGetTickCount();
#endif

    g_sentai_emu_boot_state = kBootReplBanner;

#if SENTAI_EMU_AUTORUN_HALT_AFTER
    while (true) {
        ++g_sentai_emu_heartbeat;
        g_sentai_emu_last_tick = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
#endif

    while (true) {
        ReplPutString(">>> ");
        int len = ReplReadLine(g_repl_line, sizeof(g_repl_line));
        g_sentai_emu_last_tick = xTaskGetTickCount();
        if (len <= 0) {
            continue;
        }
        ++g_sentai_emu_repl_lines;
        mp_embed_exec_str(g_repl_line);
        ++g_sentai_emu_heartbeat;
    }
}

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
volatile uint32_t g_sentai_emu_repl_lines = 0;
#if SENTAI_EMU_FX_FS
volatile uint32_t g_sentai_emu_fs_smoke_ok = 0;
volatile uint32_t g_sentai_emu_fs_smoke_size = 0;
#endif
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;

    TaskHandle_t task =
        xTaskCreateStatic(ReplTask, "emu_repl", kReplStackWords, nullptr,
                          tskIDLE_PRIORITY + 1, g_repl_stack, &g_repl_tcb);
    if (!task) {
        g_sentai_emu_boot_state = kBootCreateTaskFailed;
        while (true) {
        }
    }
    g_sentai_emu_boot_state = kBootTaskCreated;

    vTaskStartScheduler();

    g_sentai_emu_boot_state = kBootSchedulerReturned;
    while (true) {
    }
}
