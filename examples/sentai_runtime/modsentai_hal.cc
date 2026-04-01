// C++ HAL bridge for the 'sentai' MicroPython module
// Bridges MicroPython C code to coralmicro C++ APIs

#include "libs/base/console_m7.h"
#include "libs/base/filesystem.h"
#include "libs/base/led.h"
#include "libs/base/gpio.h"
#include "libs/lis2du12/lis2du12.h"
#include "libs/t5838/t5838.h"
#include "libs/base/main_freertos_m7.h"
#include "libs/audio/audio_driver.h"
#include "libs/audio/audio_service.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/littlefs/lfs.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpuart.h"

#include <cstring>
#include <vector>

// Shine fixed-point MP3 encoder (C library)
extern "C" {
#include "layer3.h"
}

extern "C" {

void sentai_led_set(int on) {
    coralmicro::LedSet(coralmicro::Led::kUser, on != 0);
}

void sentai_sleep_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t sentai_ticks_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

int sentai_console_read(char* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->Read(buf, size);
}

void sentai_console_write(const char* buf, int size) {
    coralmicro::ConsoleM7::GetSingleton()->Write(const_cast<char*>(buf), size);
}

// ===================== Filesystem bridge =====================
// All filesystem operations use the USER LFS partition.
// The system partition (with default.elf) is not accessible from Python.

// Force-format the user LFS partition. Returns 1 on success, 0 on failure.
int sentai_fs_format(void) {
    return coralmicro::LfsUserInit(/*force_format=*/true) ? 1 : 0;
}

// Read file into caller-provided buffer. Returns bytes read, or -1 on error.
int sentai_fs_read(const char* path, uint8_t* buf, int max_size) {
    size_t n = coralmicro::LfsUserReadFile(path, buf, (size_t)max_size);
    return (int)n;
}

// Get file size. Returns -1 if not found.
int sentai_fs_size(const char* path) {
    ssize_t s = coralmicro::LfsUserSize(path);
    return (int)s;
}

// Check if file exists.
int sentai_fs_file_exists(const char* path) {
    return coralmicro::LfsUserFileExists(path) ? 1 : 0;
}

// Check if directory exists.
int sentai_fs_dir_exists(const char* path) {
    return coralmicro::LfsUserDirExists(path) ? 1 : 0;
}

// Write buffer to file. Returns 1 on success, 0 on failure.
int sentai_fs_write(const char* path, const uint8_t* buf, int size) {
    return coralmicro::LfsUserWriteFile(path, buf, (size_t)size) ? 1 : 0;
}

// Remove file or empty directory. Returns 0 on success.
int sentai_fs_remove(const char* path) {
    return coralmicro::LfsUserRemove(path);
}

// Create directories (mkdir -p). Returns 1 on success.
int sentai_fs_makedirs(const char* path) {
    return coralmicro::LfsUserMakeDirs(path) ? 1 : 0;
}

// List directory entries. Calls callback for each entry.
// callback(name, type, size, user_data) - type: 1=file, 2=dir
// Returns number of entries, or -1 on error.
int sentai_fs_listdir(const char* path,
                     void (*callback)(const char* name, int type, int size, void* ud),
                     void* user_data) {
    lfs_dir_t dir;
    int err = lfs_dir_open(coralmicro::LfsUser(), &dir, path);
    if (err < 0) return -1;

    struct lfs_info info;
    int count = 0;
    while (lfs_dir_read(coralmicro::LfsUser(), &dir, &info) > 0) {
        // Skip . and ..
        if (info.name[0] == '.' &&
            (info.name[1] == '\0' || (info.name[1] == '.' && info.name[2] == '\0')))
            continue;
        int t = (info.type == LFS_TYPE_DIR) ? 2 : 1;
        callback(info.name, t, (int)info.size, user_data);
        count++;
    }
    lfs_dir_close(coralmicro::LfsUser(), &dir);
    return count;
}

// ===================== USB Serial bridge =====================
// Reuses the console's CDC ACM port (ttyACM0 on host).
// When open: printf goes to UART only, USB is reserved for Python.
// When closed: normal console (printf → UART + USB).

int sentai_usb_serial_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbSerialOpen() ? 1 : 0;
}

void sentai_usb_serial_close(void) {
    coralmicro::ConsoleM7::GetSingleton()->UsbSerialClose();
}

int sentai_usb_serial_is_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbSerialIsOpen() ? 1 : 0;
}

int sentai_usb_serial_write(const uint8_t* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbTransmit(buf, (size_t)size) ? size : -1;
}

int sentai_usb_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbRead(buf, max_size, timeout_ms);
}

int sentai_usb_serial_available(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UsbAvailable();
}

// ===================== REPL target control =====================

int sentai_console_set_target(int target) {
    using RT = coralmicro::ConsoleM7::ReplTarget;
    auto t = (target == 0) ? RT::kUsb : RT::kUart;
    coralmicro::ConsoleM7::GetSingleton()->SetReplTarget(t);
    return 0;
}

int sentai_console_get_target(void) {
    using RT = coralmicro::ConsoleM7::ReplTarget;
    return (coralmicro::ConsoleM7::GetSingleton()->GetReplTarget() == RT::kUsb) ? 0 : 1;
}

// ===================== UART Serial bridge =====================

int sentai_uart_serial_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartSerialOpen() ? 1 : 0;
}

void sentai_uart_serial_close(void) {
    coralmicro::ConsoleM7::GetSingleton()->UartSerialClose();
}

int sentai_uart_serial_is_open(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartSerialIsOpen() ? 1 : 0;
}

int sentai_uart_serial_write(const uint8_t* buf, int size) {
    return coralmicro::ConsoleM7::GetSingleton()->UartTransmit(buf, (size_t)size) ? size : -1;
}

int sentai_uart_serial_read(uint8_t* buf, int max_size, int timeout_ms) {
    return coralmicro::ConsoleM7::GetSingleton()->UartRead(buf, max_size, timeout_ms);
}

int sentai_uart_serial_available(void) {
    return coralmicro::ConsoleM7::GetSingleton()->UartAvailable();
}

// ===================== UART baudrate =====================
// LPUART6 is the M7 debug console UART.
// Clock source: OSC_RC_48M_DIV2 = 24 MHz (configured in board init).

#define LPUART6_CLOCK_FREQ  24000000U

void sentai_uart_set_baudrate(uint32_t baudrate) {
    LPUART_SetBaudRate(LPUART6, baudrate, LPUART6_CLOCK_FREQ);
}

void sentai_uart_restore_baudrate(void) {
    LPUART_SetBaudRate(LPUART6, 115200U, LPUART6_CLOCK_FREQ);
}

// ===================== Help file (system partition) =====================

int sentai_help_read(char* buf, int max_size) {
    size_t n = coralmicro::LfsReadFile(
        "examples/sentai_runtime/help.txt",
        reinterpret_cast<uint8_t*>(buf), (size_t)(max_size - 1));
    if (n > 0) {
        buf[n] = '\0';
        return (int)n;
    }
    return -1;
}

// ===================== IMU (LIS2DU12 accelerometer) =====================

static coralmicro::Lis2du12 g_imu;
static bool g_imu_initialized = false;

int sentai_imu_init(void) {
    if (g_imu_initialized) return 0;  // already initialized
    
    if (!g_imu.Init(I2C5Handle(), 0x19)) {
        return -1;  // init failed
    }
    
    g_imu_initialized = true;
    return 0;
}

int sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c) {
    if (!g_imu_initialized) return -1;
    
    bool ready = false;
    if (!g_imu.IsDataReady(&ready) || !ready) {
        return -2;  // data not ready
    }
    
    coralmicro::AccelData data;
    if (!g_imu.ReadData(&data)) {
        return -3;  // read failed
    }
    
    *x_mg = data.x_mg;
    *y_mg = data.y_mg;
    *z_mg = data.z_mg;
    *temp_c = data.temp_deg_c;
    return 0;
}

// ===================== Microphone (PDM → WAV) =====================

// Maximum recording: 10 seconds at 16 kHz = 160 000 samples × 2 bytes = 320 KB.
// Placed in SDRAM BSS to avoid overflowing the 224 KB DTCM m_data region.
static constexpr size_t kMicMaxSeconds = 10;
static constexpr int    kMicSampleRate = 16000;
static constexpr size_t kMicMaxSamples = kMicSampleRate * kMicMaxSeconds;

// DMA buffers for PDM audio — in SDRAM BSS (16 kHz is slow enough for SDRAM).
// 4 DMA buffers × 800 samples = 3200 int32 = 12.8 KB.
__attribute__((section(".sdram_bss")))
static coralmicro::AudioDriverBuffers<4, 3200> g_mic_audio_buffers;

// Static recording buffer in SDRAM BSS (320 KB).
__attribute__((section(".sdram_bss")))
static int16_t g_mic_rec_buf[kMicMaxSamples];
static volatile size_t g_mic_rec_pos = 0;      // write position (wraps in ring)
static volatile size_t g_mic_ring_size = 0;     // ring buffer size in samples
static volatile bool   g_mic_rec_wrapped = false; // true after first wrap
static volatile bool   g_mic_recording = false;

// RMS level — updated every DMA callback (~50 ms).  Stored as dB × 100 so
// MicroPython (no float) gets integer centibels.  Silence ≈ 0, loud ≈ 9000.
static volatile int g_mic_level_cdb = 0;

// Static AudioDriver (the object itself is tiny; DMA buffers are separate).
static coralmicro::AudioDriver g_mic_driver(g_mic_audio_buffers);
// AudioService must be constructed/destructed to start/stop the mic task.
// We use placement-new into a static aligned buffer to avoid heap.
__attribute__((section(".sdram_bss"), aligned(8)))
static uint8_t g_mic_svc_storage[sizeof(coralmicro::AudioService)];
static coralmicro::AudioService* g_mic_svc_ptr = nullptr;

// File counter for auto-incrementing filenames.
static int g_mic_file_counter = 0;

// Monitor-only mode: mic is on for level() but not recording.
static bool g_mic_monitor = false;

// Forward declaration for callback (used by mic_ensure_service).
static bool MicAudioCallback(void* ctx, const int32_t* samples,
                              size_t num_samples);

// Internal: ensure AudioService is running (for level monitoring or recording).
// Returns 0 on success, negative on error.
static int mic_ensure_service(void) {
    if (g_mic_svc_ptr) return 0;  // already running

    const coralmicro::AudioDriverConfig config{
        coralmicro::AudioSampleRate::k16000_Hz,
        /*num_dma_buffers=*/4,
        /*dma_buffer_size_ms=*/50};

    if (!g_mic_audio_buffers.CanHandle(config)) return -3;

    g_mic_svc_ptr = new (g_mic_svc_storage) coralmicro::AudioService(
        &g_mic_driver, config, /*task_priority=*/3,
        /*drop_first_samples_ms=*/150);
    g_mic_svc_ptr->AddCallback(nullptr, MicAudioCallback);
    return 0;
}

// Internal: stop AudioService entirely.
static void mic_stop_service(void) {
    if (!g_mic_svc_ptr) return;
    g_mic_svc_ptr->~AudioService();
    g_mic_svc_ptr = nullptr;
    g_mic_monitor = false;
}

// ---- integer log10 approximation for dB calculation ----
// Returns log10(x) × 1000 for x > 0, using integer math only.
// Accuracy: ~±1% for x > 100.
static int ilog10x1000(uint64_t x) {
    if (x == 0) return 0;
    // Count leading zeros to find the bit position.
    int bits = 63 - __builtin_clzll(x);  // floor(log2(x))
    // log10(x) = log2(x) / log2(10) ≈ bits × 301 / 1000
    return bits * 301;
}

// AudioService callback — runs from the audio FreeRTOS task.
// Accumulates int16 samples and computes RMS level every chunk.
//
// Cortex-M7 DSP optimizations:
//   RMS loop: __PKHTB packs top-16 of two int32 samples into one register,
//             __SMLALD does dual 16×16 signed MAC into 64-bit accumulator.
//             → processes 2 samples per iteration in a single cycle each.
//   Record loop: ring buffer with scalar writes (wraps at ring size).
static bool MicAudioCallback(void* ctx, const int32_t* samples,
                              size_t num_samples) {
    (void)ctx;

    // ---- RMS level (Cortex-M7 SIMD: 2 samples/iter) ----
    uint64_t sum = 0;
    size_t i = 0;
    // Process pairs: PKHTB extracts top halves, SMLALD squares+accumulates.
    for (; i + 1 < num_samples; i += 2) {
        // PKHTB(a, b, 16): result = [top16(a) | top16(b)]
        // Extracts the >>16 from both samples into one packed halfword register.
        uint32_t packed = __PKHTB(samples[i], samples[i + 1], 16);
        // SMLALD: sum += lo16(packed)^2 + hi16(packed)^2  (single cycle)
        sum = __SMLALD(packed, packed, sum);
    }
    // Handle odd remainder.
    if (i < num_samples) {
        int32_t s = samples[i] >> 16;
        sum += (int64_t)s * s;
    }
    // RMS in 16-bit scale: sqrt(sum / N).
    // dB = 20 × log10(rms).  We avoid float: store as centi-dB (× 100).
    // 20 × log10(rms) = 20 × 0.5 × log10(sum/N) = 10 × log10(sum/N).
    // Using ilog10x1000: result in milli-log10, so × 10 / 1000 = / 100.
    // We want centibels: 10 × log10(sum/N) × 100 = ilog10x1000(sum/N).
    if (num_samples > 0 && sum > 0) {
        g_mic_level_cdb = ilog10x1000(sum / num_samples);
    } else {
        g_mic_level_cdb = 0;
    }

    // Record into ring buffer (wraps to keep last N seconds).
    if (g_mic_recording) {
        size_t pos = g_mic_rec_pos;
        const size_t ring = g_mic_ring_size;
        for (size_t j = 0; j < num_samples; ++j) {
            g_mic_rec_buf[pos] = static_cast<int16_t>(samples[j] >> 16);
            if (++pos >= ring) {
                pos = 0;
                g_mic_rec_wrapped = true;
            }
        }
        g_mic_rec_pos = pos;
    }

    return true;
}

// sentai_mic_start(seconds) — start continuous ring-buffer recording.
// Ring keeps the last `seconds` of audio (default 10, max 10).
// If mic was already in monitor mode (from level()), just enables recording.
// Returns 0 on success, negative on error.
int sentai_mic_start(int max_seconds) {
    if (max_seconds < 1) max_seconds = 10;
    if (max_seconds > (int)kMicMaxSeconds) max_seconds = (int)kMicMaxSeconds;

    // If already recording (not just monitoring), reject.
    if (g_mic_svc_ptr && !g_mic_monitor) return -2;

    // Prepare ring buffer.
    g_mic_ring_size = (size_t)(kMicSampleRate * max_seconds);
    g_mic_rec_pos = 0;
    g_mic_rec_wrapped = false;
    g_mic_recording = true;
    g_mic_monitor = false;  // now in recording mode

    // Ensure mic service is running (may already be from monitor mode).
    int rc = mic_ensure_service();
    if (rc < 0) return rc;

    printf("[mic] Ring recording started (%d s ring)\r\n", max_seconds);
    return 0;
}

// sentai_mic_stop() — stop mic, return number of samples captured.
int sentai_mic_stop(void) {
    if (!g_mic_svc_ptr) return -1;

    g_mic_recording = false;

    // Fully stop AudioService (powers off mic).
    mic_stop_service();

    int n = (int)g_mic_rec_pos;
    printf("[mic] Stopped. %d samples\r\n", n);
    return n;
}

// sentai_mic_busy() — 1 = recording, 0 = done/stopped, -1 = never started.
int sentai_mic_busy(void) {
    if (!g_mic_svc_ptr) return (g_mic_rec_pos > 0) ? 0 : -1;
    return g_mic_recording ? 1 : 0;
}

// sentai_mic_samples() — number of samples available in ring.
int sentai_mic_samples(void) {
    if (g_mic_rec_wrapped) return (int)g_mic_ring_size;
    return (int)g_mic_rec_pos;
}

// sentai_mic_level() — current RMS level in centi-dB (0 = silence, ~9000 = loud).
// Auto-starts mic in monitor mode if not already running.
int sentai_mic_level(void) {
    if (!g_mic_svc_ptr) {
        g_mic_recording = false;
        g_mic_monitor = true;
        if (mic_ensure_service() < 0) return -1;
    }
    return g_mic_level_cdb;
}

// sentai_mic_save_l3() — save ring buffer as /audio/recNNN.mp3 (shine fixed-point MP3).
// Pauses recording while encoding, then resumes with a fresh ring.
// Can be called while mic is running — no need to stop() first.
// Returns file size on success, negative on error.
int sentai_mic_save_l3(char* out_name, int name_size) {
    // Pause recording for a consistent snapshot of the ring.
    bool was_recording = g_mic_recording;
    g_mic_recording = false;

    size_t pos   = g_mic_rec_pos;
    bool wrapped = g_mic_rec_wrapped;
    size_t ring  = g_mic_ring_size;

    // Determine sample count and start offset in ring.
    size_t n, start;
    if (wrapped) {
        n = ring;       // full ring
        start = pos;    // oldest sample is at write head
    } else {
        n = pos;        // haven't wrapped yet
        start = 0;
    }

    if (n == 0) {
        g_mic_recording = was_recording;
        return -3;  // nothing to save
    }

    // Ensure /audio/ directory exists.
    coralmicro::LfsUserMakeDirs("/audio");

    char path[48];
    snprintf(path, sizeof(path), "/audio/rec%03d.mp3", g_mic_file_counter);
    g_mic_file_counter++;

    // Configure shine encoder: 16 kHz mono, 64 kbps.
    shine_config_t cfg;
    shine_set_config_mpeg_defaults(&cfg.mpeg);
    cfg.wave.channels   = PCM_MONO;
    cfg.wave.samplerate = kMicSampleRate;
    cfg.mpeg.bitr       = 64;
    cfg.mpeg.mode       = MONO;

    shine_t enc = shine_initialise(&cfg);
    if (!enc) {
        g_mic_recording = was_recording;
        return -4;
    }

    int spp = shine_samples_per_pass(enc);

    // Estimate max MP3 size: bitrate * duration + padding.
    size_t max_mp3 = (size_t)(cfg.mpeg.bitr * 1000 / 8) * (n / kMicSampleRate + 2) + 4096;
    uint8_t* mp3 = reinterpret_cast<uint8_t*>(pvPortMalloc(max_mp3));
    if (!mp3) { shine_close(enc); g_mic_recording = was_recording; return -4; }
    size_t mp3_pos = 0;

    // Temporary frame buffer for ring → linear copy (576 samples for MPEG2 L3).
    int16_t frame_buf[1152];  // max possible spp

    // Encode frame by frame, reading from ring buffer.
    for (size_t i = 0; i < n; i += (size_t)spp) {
        size_t frame_len = ((n - i) >= (size_t)spp) ? (size_t)spp : (n - i);
        // Copy from ring into linear frame_buf.
        for (size_t k = 0; k < frame_len; ++k) {
            frame_buf[k] = g_mic_rec_buf[(start + i + k) % ring];
        }
        // Zero-pad if last frame is short.
        for (size_t k = frame_len; k < (size_t)spp; ++k) {
            frame_buf[k] = 0;
        }
        int written = 0;
        unsigned char* data = shine_encode_buffer_interleaved(enc, frame_buf, &written);
        if (data && written > 0 && mp3_pos + (size_t)written <= max_mp3) {
            memcpy(mp3 + mp3_pos, data, (size_t)written);
            mp3_pos += (size_t)written;
        }
    }

    // Flush remaining encoder data.
    int written = 0;
    unsigned char* data = shine_flush(enc, &written);
    if (data && written > 0 && mp3_pos + (size_t)written <= max_mp3) {
        memcpy(mp3 + mp3_pos, data, (size_t)written);
        mp3_pos += (size_t)written;
    }

    shine_close(enc);

    bool ok = coralmicro::LfsUserWriteFile(path, mp3, mp3_pos);
    vPortFree(mp3);

    if (!ok) {
        g_mic_recording = was_recording;
        return -5;
    }

    // Reset ring and resume recording if it was active.
    g_mic_rec_pos = 0;
    g_mic_rec_wrapped = false;
    g_mic_recording = was_recording;

    // Copy filename to output.
    int len = (int)strlen(path);
    if (len >= name_size) len = name_size - 1;
    memcpy(out_name, path, len);
    out_name[len] = '\0';

    printf("[mic] Saved %s (%u bytes MP3, %u samples)\r\n",
           path, (unsigned)mp3_pos, (unsigned)n);
    return (int)mp3_pos;
}

// ===================== Light Sleep with AAD wakeup =====================
// Uses T5838 AAD (Audio Activity Detection) to wake from light sleep.
// Unlike deep sleep, this keeps the system powered but in low-power idle.
// The T5838 AAD generates a GPIO interrupt when sound exceeds threshold.
// Also supports LIS2DU12 double-tap wakeup via INT2.
// Returns: 0 = timeout, 1 = mic wakeup, 2 = double-tap wakeup, -1 = error

static volatile bool g_idle_mic_triggered = false;
static volatile bool g_idle_tap_triggered = false;
static TaskHandle_t g_idle_task_handle = nullptr;

// ISR callback for MIC_CLK_FEEDBACK (AAD output)
static void idle_mic_isr_callback() {
    g_idle_mic_triggered = true;
    if (g_idle_task_handle) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(g_idle_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// ISR callback for accelerometer INT2 (double-tap)
static void idle_tap_isr_callback() {
    g_idle_tap_triggered = true;
    if (g_idle_task_handle) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(g_idle_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// Map dB threshold to T5838 AAD enum.
static coralmicro::T5838AadAThr DbToAadThr(int db) {
    using namespace coralmicro;
    if (db <= 60) return kT5838AadAThr60dB;
    if (db <= 65) return kT5838AadAThr65dB;
    if (db <= 70) return kT5838AadAThr70dB;
    if (db <= 75) return kT5838AadAThr75dB;
    if (db <= 80) return kT5838AadAThr80dB;
    if (db <= 85) return kT5838AadAThr85dB;
    if (db <= 90) return kT5838AadAThr90dB;
    return kT5838AadAThr95dB;
}

// Polling task for double-tap detection (INT2 not wired to GPIO)
static volatile bool g_tap_poll_running = false;
static void tap_poll_task(void* param) {
    (void)param;
    extern coralmicro::Lis2du12 g_imu;
    extern bool g_imu_initialized;
    
    while (g_tap_poll_running && !g_idle_tap_triggered) {
        if (g_imu_initialized) {
            lis2du12_all_sources_t sources{};
            if (lis2du12_all_sources_get(g_imu.GetDevCtx(), &sources) == 0) {
                if (sources.double_tap) {
                    printf("[tap_poll] Double-tap detected!\r\n");
                    g_idle_tap_triggered = true;
                    if (g_idle_task_handle) {
                        xTaskNotifyGive(g_idle_task_handle);
                    }
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Poll every 20ms
    }
    vTaskDelete(nullptr);
}

// Light sleep: wait for mic sound, double-tap, or timeout.
// threshold_db: 60, 65, 70, 75, 80, 85, 90, 95 (lower = more sensitive)
// timeout_ms: max wait time in milliseconds (0 = wait forever)
// enable_tap: if true, also wake on double-tap
// Returns: 0 = timeout, 1 = mic wakeup, 2 = double-tap wakeup, -1 = error
int sentai_sleep_idle(int threshold_db, int timeout_ms, int enable_tap) {
    using namespace coralmicro;
    extern Lis2du12 g_imu;
    extern bool g_imu_initialized;

    printf("[idle] Starting light sleep: threshold=%d dB, timeout=%d ms, tap=%d\r\n",
           threshold_db, timeout_ms, enable_tap);

    // Stop AudioService if running (releases PDM clock for AAD config).
    if (g_mic_svc_ptr) {
        g_mic_recording = false;
        g_mic_monitor = false;
        mic_stop_service();
        printf("[idle] Audio service stopped\r\n");
    }

    // Configure T5838 AAD
    T5838Aad mic_aad;
    if (!mic_aad.Init()) {
        printf("[idle] ERROR: T5838 init failed\r\n");
        return -1;
    }

    T5838AadAConf conf = {kT5838AadALpf4_4kHz, DbToAadThr(threshold_db)};
    if (!mic_aad.AadAModeSet(conf)) {
        printf("[idle] ERROR: Failed to configure AAD\r\n");
        return -1;
    }
    mic_aad.RestorePdmClk();
    printf("[idle] AAD configured: LPF=4.4kHz, threshold=%d dB\r\n", threshold_db);

    // Setup double-tap if enabled and IMU initialized
    TaskHandle_t tap_task_handle = nullptr;
    if (enable_tap) {
        if (!g_imu_initialized) {
            printf("[idle] WARNING: IMU not initialized, skipping double-tap\r\n");
        } else {
            // Configure double-tap on INT2 (internal to chip, polled via register)
            if (!g_imu.SetInt2DoubleTap()) {
                printf("[idle] WARNING: SetInt2DoubleTap failed\r\n");
            } else {
                printf("[idle] Double-tap detection enabled\r\n");
                g_tap_poll_running = true;
                g_idle_tap_triggered = false;
                xTaskCreate(tap_poll_task, "tap_poll", 512, nullptr, 2, &tap_task_handle);
            }
        }
    }

    // Setup state for ISR
    g_idle_mic_triggered = false;
    g_idle_task_handle = xTaskGetCurrentTaskHandle();

    // Register GPIO interrupt on MIC_CLK_FEEDBACK (rising edge = AAD triggered)
    GpioConfigureInterrupt(
        Gpio::kMicClkFeedback,
        GpioInterruptMode::kIntModeRising,
        idle_mic_isr_callback);

    printf("[idle] Waiting for sound%s...\r\n", enable_tap ? " or double-tap" : "");

    // Wait for notification (from ISR or tap poll task) or timeout
    TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    uint32_t notified = ulTaskNotifyTake(pdTRUE, wait_ticks);

    // Stop tap polling task
    g_tap_poll_running = false;
    if (tap_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(30));  // Let it exit
    }

    // Cleanup
    g_idle_task_handle = nullptr;

    // Determine wakeup cause
    int result;
    if (g_idle_mic_triggered) {
        printf("[idle] Woke up: MIC sound detected!\r\n");
        result = 1;
    } else if (g_idle_tap_triggered) {
        printf("[idle] Woke up: Double-tap detected!\r\n");
        result = 2;
    } else if (notified == 0) {
        printf("[idle] Woke up: timeout\r\n");
        result = 0;
    } else {
        printf("[idle] Woke up: unknown cause\r\n");
        result = 0;
    }

    // Disable AAD mode to allow normal mic use
    mic_aad.AadModeDisable();
    mic_aad.RestorePdmClk();

    return result;
}

}  // extern "C"
