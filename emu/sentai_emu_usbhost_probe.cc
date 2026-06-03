// B8.8 Phase 1 — USB host stack probe.
//
// Links the production `libs/usb/usb_host_task.cc` plus the minimum
// NXP RT1176 SDK USB middleware needed by `UsbHostTask::UsbHostTask()`
// into an emulator image, and exercises the constructor.  No Renode-side
// USB model is configured yet; the goal of P1 is to prove the
// production USB code COMPILES and LINKS against the emu profile, and
// to surface the complete set of SDK files pulled in transitively.
//
// The probe deliberately does NOT call `UsbHostTask::GetSingleton()` or
// `Init()` (which would create a FreeRTOS task) — that is Phase 2, once
// the Renode platform is wired up.  P1 is a one-shot constructor call
// from `main()` with progress markers on LPUART6.
//
// HARD RULE (per `feedback_emu_tpu_must_run_real_edgetpu_manager`):
// this target uses the production class verbatim, not an emu-only
// reimplementation.  The probe stays under emu/ so it does not affect
// the ARM `sentai_runtime` build.

#include <stdint.h>

// Production include order: usb_host_config.h must precede usb_host.h so the
// USB_HOST_CONFIG_* macros are visible inside the SDK headers.
#include "third_party/modified/nxp/rt1176-sdk/usb_host_config.h"
#include "libs/usb/usb_host_task.h"
#if SENTAI_EMU_EDGETPU_MANAGER_PROBE
#include "libs/tpu/edgetpu_manager.h"
#endif
#if SENTAI_EMU_EDGETPU_MMIO_SEND_BRIDGE_PROBE
#include "libs/tpu/edgetpu_driver.h"
#endif
#if SENTAI_EMU_EDGETPU_TASK_PROBE
#include "libs/tpu/edgetpu_task.h"
#endif
#if SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE
#include "libs/tpu/usb_host_edgetpu.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_devices.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb_spec.h"
#endif
#if SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE
#include "libs/tpu/usb_host_edgetpu.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_devices.h"
#include "third_party/nxp/rt1176-sdk/middleware/usb/include/usb_spec.h"
#endif
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "fsl_device_registers.h"

extern "C" volatile uint32_t g_sentai_emu_boot_state;
extern "C" volatile uint32_t g_sentai_emu_step;
extern "C" volatile uint32_t g_sentai_emu_heartbeat;  // referenced by FreeRTOS hooks
extern "C" volatile uint32_t g_sentai_emu_last_tick;

namespace {

constexpr uint32_t kBootEnteredMain = 0x0100;
constexpr uint32_t kBootBeforeUsbHostCtor = 0x0200;
constexpr uint32_t kBootAfterUsbHostCtor = 0x0300;
constexpr uint32_t kBootBeforeUsbHostInit = 0x0400;
constexpr uint32_t kBootAfterUsbHostInit = 0x0500;
constexpr uint32_t kBootHeartbeatRunning = 0x0600;
constexpr uint32_t kBootSchedulerNotStarted = 0xCAFE;
constexpr uint32_t kBootSchedulerReturned = 0xEE00;
constexpr uint32_t kBootHeartbeatCreateFailed = 0xEF00;

constexpr uint32_t kHeartbeatStackWords = configMINIMAL_STACK_SIZE * 2;

StaticTask_t g_heartbeat_tcb;
StackType_t g_heartbeat_stack[kHeartbeatStackWords] __attribute__((aligned(8)));
#if SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE
constexpr uint32_t kOpenDeviceStackWords = configMINIMAL_STACK_SIZE * 3;
StaticTask_t g_open_device_tcb;
StackType_t g_open_device_stack[kOpenDeviceStackWords] __attribute__((aligned(8)));
#endif
#if SENTAI_EMU_EDGETPU_MMIO_SEND_BRIDGE_PROBE
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_params_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_input_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_ins_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_output_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_event_calls;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_last_result;
extern "C" volatile uint32_t g_sentai_emu_tpu_bridge_output_sum;
#endif
#if SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE || SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE
void StoreLe16(uint8_t out[2], uint16_t v) {
    out[0] = static_cast<uint8_t>(v & 0xffu);
    out[1] = static_cast<uint8_t>((v >> 8) & 0xffu);
}
#endif

#if SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE
constexpr uint32_t kSyntheticEnumStackWords = configMINIMAL_STACK_SIZE * 3;
StaticTask_t g_synthetic_enum_tcb;
StackType_t g_synthetic_enum_stack[kSyntheticEnumStackWords]
    __attribute__((aligned(8)));

usb_descriptor_device_t g_synthetic_device_desc = {};
usb_descriptor_configuration_t g_synthetic_config_desc = {};
usb_descriptor_interface_t g_synthetic_interface_desc = {};
usb_descriptor_endpoint_t g_synthetic_eps[USB_EDGETPU_MIN_ENDPOINTS] = {};
usb_host_device_instance_t g_synthetic_device = {};
usb_host_configuration_t g_synthetic_config = {};

void InitSyntheticEdgeTpuDevice(usb_host_handle host_handle) {
    g_synthetic_device_desc.bLength = sizeof(g_synthetic_device_desc);
    g_synthetic_device_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE;
    StoreLe16(g_synthetic_device_desc.idVendor, coralmicro::kEdgeTpuVid);
    StoreLe16(g_synthetic_device_desc.idProduct, coralmicro::kEdgeTpuPid);
    g_synthetic_device_desc.bNumConfigurations = 1;

    g_synthetic_config_desc.bLength = sizeof(g_synthetic_config_desc);
    g_synthetic_config_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIGURE;
    g_synthetic_config_desc.bNumInterfaces = 1;
    g_synthetic_config_desc.bConfigurationValue = 1;
    g_synthetic_config_desc.bMaxPower = 250;

    g_synthetic_interface_desc.bLength = sizeof(g_synthetic_interface_desc);
    g_synthetic_interface_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE;
    g_synthetic_interface_desc.bInterfaceNumber = 0;
    g_synthetic_interface_desc.bAlternateSetting = 0;
    g_synthetic_interface_desc.bNumEndpoints = USB_EDGETPU_MIN_ENDPOINTS;
    g_synthetic_interface_desc.bInterfaceClass = USB_HOST_EDGETPU_CLASS_CODE;
    g_synthetic_interface_desc.bInterfaceSubClass =
        USB_HOST_EDGETPU_SUBCLASS_CODE;

    const uint8_t ep_addr[USB_EDGETPU_MIN_ENDPOINTS] = {0x81, 0x02, 0x83};
    const uint8_t ep_attr[USB_EDGETPU_MIN_ENDPOINTS] = {
        USB_ENDPOINT_BULK, USB_ENDPOINT_BULK, USB_ENDPOINT_INTERRUPT};
    for (int i = 0; i < USB_EDGETPU_MIN_ENDPOINTS; ++i) {
        g_synthetic_eps[i].bLength = sizeof(usb_descriptor_endpoint_t);
        g_synthetic_eps[i].bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_synthetic_eps[i].bEndpointAddress = ep_addr[i];
        g_synthetic_eps[i].bmAttributes = ep_attr[i];
        StoreLe16(g_synthetic_eps[i].wMaxPacketSize,
                  ep_attr[i] == USB_ENDPOINT_INTERRUPT ? 64 : 512);
        g_synthetic_eps[i].bInterval = 1;
    }

    g_synthetic_config.interfaceCount = 1;
    g_synthetic_config.configurationDesc = &g_synthetic_config_desc;
    usb_host_interface_t *iface = &g_synthetic_config.interfaceList[0];
    iface->interfaceDesc = &g_synthetic_interface_desc;
    iface->interfaceIndex = 0;
    iface->alternateSettingNumber = 1;
    iface->epCount = USB_EDGETPU_MIN_ENDPOINTS;
    for (int i = 0; i < USB_EDGETPU_MIN_ENDPOINTS; ++i) {
        iface->epList[i].epDesc = &g_synthetic_eps[i];
    }

    g_synthetic_device.hostHandle = host_handle;
    g_synthetic_device.configuration = g_synthetic_config;
    g_synthetic_device.deviceDescriptor = &g_synthetic_device_desc;
    g_synthetic_device.configurationDesc =
        reinterpret_cast<uint8_t *>(&g_synthetic_config_desc);
    g_synthetic_device.configurationLen = sizeof(g_synthetic_config_desc);
    g_synthetic_device.configurationValue = 1;
    g_synthetic_device.speed = USB_SPEED_HIGH;
    g_synthetic_device.setAddress = 1;
    g_synthetic_device.deviceAttachState = kStatus_device_Attached;
}
#endif
#if SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE
constexpr uint32_t kMmioEnumStackWords = configMINIMAL_STACK_SIZE * 3;
constexpr uintptr_t kCoralUsbBase = 0x40900400u;
constexpr uint32_t kCoralStatus = 0x00;
constexpr uint32_t kCoralVidPid = 0x04;
constexpr uint32_t kCoralIface = 0x08;
constexpr uint32_t kCoralEp0 = 0x0C;
constexpr uint32_t kCoralEp1 = 0x10;
constexpr uint32_t kCoralEp2 = 0x14;
constexpr uint32_t kCoralMaxp = 0x18;

StaticTask_t g_mmio_enum_tcb;
StackType_t g_mmio_enum_stack[kMmioEnumStackWords] __attribute__((aligned(8)));
TaskHandle_t g_mmio_enum_task = nullptr;

usb_descriptor_device_t g_mmio_device_desc = {};
usb_descriptor_configuration_t g_mmio_config_desc = {};
usb_descriptor_interface_t g_mmio_interface_desc = {};
usb_descriptor_endpoint_t g_mmio_eps[USB_EDGETPU_MIN_ENDPOINTS] = {};
usb_host_device_instance_t g_mmio_device = {};
usb_host_configuration_t g_mmio_config = {};

uint32_t CoralReg(uint32_t off) {
    return *reinterpret_cast<volatile uint32_t *>(kCoralUsbBase + off);
}

void CoralWrite(uint32_t off, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(kCoralUsbBase + off) = value;
}

void InitMmioEdgeTpuDevice(usb_host_handle host_handle) {
    const uint32_t vid_pid = CoralReg(kCoralVidPid);
    const uint32_t iface_word = CoralReg(kCoralIface);
    const uint32_t ep_words[USB_EDGETPU_MIN_ENDPOINTS] = {
        CoralReg(kCoralEp0), CoralReg(kCoralEp1), CoralReg(kCoralEp2)};
    const uint32_t maxp_word = CoralReg(kCoralMaxp);
    const uint16_t bulk_maxp = static_cast<uint16_t>(maxp_word & 0xffffu);
    const uint16_t intr_maxp = static_cast<uint16_t>((maxp_word >> 16) & 0xffffu);

    g_mmio_device_desc.bLength = sizeof(g_mmio_device_desc);
    g_mmio_device_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE;
    StoreLe16(g_mmio_device_desc.idVendor, static_cast<uint16_t>(vid_pid >> 16));
    StoreLe16(g_mmio_device_desc.idProduct, static_cast<uint16_t>(vid_pid));
    g_mmio_device_desc.bNumConfigurations = 1;

    g_mmio_config_desc.bLength = sizeof(g_mmio_config_desc);
    g_mmio_config_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIGURE;
    g_mmio_config_desc.bNumInterfaces = 1;
    g_mmio_config_desc.bConfigurationValue = 1;
    g_mmio_config_desc.bMaxPower = 250;

    g_mmio_interface_desc.bLength = sizeof(g_mmio_interface_desc);
    g_mmio_interface_desc.bDescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE;
    g_mmio_interface_desc.bInterfaceNumber = 0;
    g_mmio_interface_desc.bAlternateSetting = 0;
    g_mmio_interface_desc.bNumEndpoints =
        static_cast<uint8_t>((iface_word >> 16) & 0xffu);
    g_mmio_interface_desc.bInterfaceClass = static_cast<uint8_t>(iface_word);
    g_mmio_interface_desc.bInterfaceSubClass =
        static_cast<uint8_t>((iface_word >> 8) & 0xffu);

    for (int i = 0; i < USB_EDGETPU_MIN_ENDPOINTS; ++i) {
        const uint32_t ep = ep_words[i];
        const uint8_t attr = static_cast<uint8_t>((ep >> 8) & 0xffu);
        g_mmio_eps[i].bLength = sizeof(usb_descriptor_endpoint_t);
        g_mmio_eps[i].bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_mmio_eps[i].bEndpointAddress = static_cast<uint8_t>(ep);
        g_mmio_eps[i].bmAttributes = attr;
        StoreLe16(g_mmio_eps[i].wMaxPacketSize,
                  attr == USB_ENDPOINT_INTERRUPT ? intr_maxp : bulk_maxp);
        g_mmio_eps[i].bInterval = static_cast<uint8_t>((ep >> 24) & 0xffu);
    }

    g_mmio_config.interfaceCount = 1;
    g_mmio_config.configurationDesc = &g_mmio_config_desc;
    usb_host_interface_t *iface = &g_mmio_config.interfaceList[0];
    iface->interfaceDesc = &g_mmio_interface_desc;
    iface->interfaceIndex = 0;
    iface->alternateSettingNumber = 1;
    iface->epCount = g_mmio_interface_desc.bNumEndpoints;
    for (int i = 0; i < USB_EDGETPU_MIN_ENDPOINTS; ++i) {
        iface->epList[i].epDesc = &g_mmio_eps[i];
    }

    g_mmio_device.hostHandle = host_handle;
    g_mmio_device.configuration = g_mmio_config;
    g_mmio_device.deviceDescriptor = &g_mmio_device_desc;
    g_mmio_device.configurationDesc =
        reinterpret_cast<uint8_t *>(&g_mmio_config_desc);
    g_mmio_device.configurationLen = sizeof(g_mmio_config_desc);
    g_mmio_device.configurationValue = 1;
    g_mmio_device.speed = USB_SPEED_HIGH;
    g_mmio_device.setAddress = 1;
    g_mmio_device.deviceAttachState = kStatus_device_Attached;
}
#endif

void UartInit() {
    *reinterpret_cast<volatile uint32_t *>(0x40090018u) = (1u << 18) | (1u << 19);
}

void UartPutChar(char ch) {
    while ((*reinterpret_cast<volatile uint32_t *>(0x40090014u) & (1u << 23)) == 0u) {
    }
    *reinterpret_cast<volatile uint32_t *>(0x4009001Cu) = static_cast<uint8_t>(ch);
}

void UartWrite(const char *s) {
    while (*s) {
        UartPutChar(*s++);
    }
}

#if SENTAI_EMU_USBHOST_PROBE_START_TASK
void HeartbeatTask(void *) {
    g_sentai_emu_boot_state = kBootHeartbeatRunning;
    UartWrite("heartbeat task online\r\n");
    while (true) {
        ++g_sentai_emu_heartbeat;
        g_sentai_emu_last_tick = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#if SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE
void OpenDeviceTask(void *) {
    UartWrite("step 12: OpenDevice task calling manager\r\n");
    g_sentai_emu_step = 12;
    auto context = coralmicro::EdgeTpuManager::GetSingleton()->OpenDevice();
#if SENTAI_EMU_EDGETPU_MMIO_SEND_BRIDGE_PROBE
    if (!context) {
        g_sentai_emu_step = 23;
        UartWrite("step 23: OpenDevice unexpectedly returned nullptr\r\n");
    } else {
        g_sentai_emu_step = 22;
        UartWrite("step 22: OpenDevice returned context after MMIO enum\r\n");

        static const uint8_t params[] = {0x10, 0x11, 0x12, 0x13};
        static const uint8_t input[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};
        static const uint8_t instructions[] = {0x30, 0x31, 0x32, 0x33, 0x34};
        uint8_t output[16] = {};
        coralmicro::TpuDriver bridge_driver;
        const bool ok =
            bridge_driver.Initialize(nullptr, coralmicro::PerformanceMode::kLow) &&
            bridge_driver.SendParameters(params, sizeof(params)) &&
            bridge_driver.SendInputs(input, sizeof(input)) &&
            bridge_driver.SendInstructions(instructions, sizeof(instructions)) &&
            bridge_driver.GetOutputs(output, sizeof(output)) &&
            bridge_driver.ReadEvent();
        uint32_t sum = 0;
        for (uint8_t b : output) sum += b;
        g_sentai_emu_tpu_bridge_output_sum = sum;
        if (ok && output[0] == 0xA0 && output[15] == 0xAF) {
            g_sentai_emu_step = 30;
            UartWrite("step 30: TpuDriver Send* MMIO bridge smoke passed\r\n");
        } else {
            g_sentai_emu_step = 31;
            UartWrite("step 31: TpuDriver Send* MMIO bridge smoke failed\r\n");
        }
    }
#elif SENTAI_EMU_EDGETPU_MMIO_OPENDEVICE_PROBE
    if (context) {
        g_sentai_emu_step = 22;
        UartWrite("step 22: OpenDevice returned context after MMIO enum\r\n");
    } else {
        g_sentai_emu_step = 23;
        UartWrite("step 23: OpenDevice unexpectedly returned nullptr\r\n");
    }
#else
    if (context) {
        g_sentai_emu_step = 14;
        UartWrite("step 14: OpenDevice unexpectedly returned context\r\n");
    } else {
        g_sentai_emu_step = 13;
        UartWrite("step 13: OpenDevice returned nullptr after error\r\n");
    }
#endif
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE
void SyntheticEnumTask(void *param) {
    auto *host_task = static_cast<coralmicro::UsbHostTask *>(param);
    vTaskDelay(pdMS_TO_TICKS(100));
    UartWrite("step 15: synthetic EdgeTPU attach\r\n");
    g_sentai_emu_step = 15;
    InitSyntheticEdgeTpuDevice(host_task->host_handle());
    usb_status_t attach_status = host_task->HostEvent(
        reinterpret_cast<usb_device_handle>(&g_synthetic_device),
        reinterpret_cast<usb_host_configuration_handle>(&g_synthetic_config),
        kUSB_HostEventAttach);
    if (attach_status != kStatus_USB_Success) {
        g_sentai_emu_step = 17;
        UartWrite("step 17: synthetic EdgeTPU attach failed\r\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    UartWrite("step 16: synthetic EdgeTPU enum done\r\n");
    g_sentai_emu_step = 16;
    host_task->HostEvent(
        reinterpret_cast<usb_device_handle>(&g_synthetic_device),
        reinterpret_cast<usb_host_configuration_handle>(&g_synthetic_config),
        kUSB_HostEventEnumerationDone);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif
#if SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE
void MmioEnumTask(void *param) {
    auto *host_task = static_cast<coralmicro::UsbHostTask *>(param);
    vTaskDelay(pdMS_TO_TICKS(100));
    if ((CoralReg(kCoralStatus) & 1u) == 0u) {
        g_sentai_emu_step = 21;
        UartWrite("step 21: Renode Coral MMIO status missing\r\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    while (true) {
        UartWrite("step 18: Renode Coral MMIO attach\r\n");
        g_sentai_emu_step = 18;
        InitMmioEdgeTpuDevice(host_task->host_handle());
        CoralWrite(kCoralStatus, 0);
        usb_status_t attach_status = host_task->HostEvent(
            reinterpret_cast<usb_device_handle>(&g_mmio_device),
            reinterpret_cast<usb_host_configuration_handle>(&g_mmio_config),
            kUSB_HostEventAttach);
        if (attach_status != kStatus_USB_Success) {
            g_sentai_emu_step = 20;
            UartWrite("step 20: Renode Coral MMIO attach failed\r\n");
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        UartWrite("step 19: Renode Coral MMIO enum done\r\n");
        g_sentai_emu_step = 19;
        host_task->HostEvent(
            reinterpret_cast<usb_device_handle>(&g_mmio_device),
            reinterpret_cast<usb_host_configuration_handle>(&g_mmio_config),
            kUSB_HostEventEnumerationDone);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
#endif
#endif

}  // namespace

extern "C" {
volatile uint32_t g_sentai_emu_boot_state = 0;
volatile uint32_t g_sentai_emu_step = 0;
volatile uint32_t g_sentai_emu_heartbeat = 0;
volatile uint32_t g_sentai_emu_last_tick = 0;
#if SENTAI_EMU_EDGETPU_MMIO_SEND_BRIDGE_PROBE
volatile uint32_t g_sentai_emu_tpu_bridge_output_sum = 0;
#endif
}

extern "C" int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    g_sentai_emu_boot_state = kBootEnteredMain;
    UartInit();
    UartWrite("\r\nSentAI EMU USBHOST PROBE B8.8 P1\r\n");

    UartWrite("step 1: about to construct UsbHostTask\r\n");
    g_sentai_emu_boot_state = kBootBeforeUsbHostCtor;
    g_sentai_emu_step = 1;

#if SENTAI_EMU_USBHOST_PROBE_START_TASK
    coralmicro::UsbHostTask *host_task = coralmicro::UsbHostTask::GetSingleton();
#else
    // Production constructor.  Does: CLOCK_EnableUsbhs1PhyPllClock,
    // CLOCK_EnableUsbhs1Clock, USB_EhciLowPowerPhyInit, USB_HostInit.
    coralmicro::UsbHostTask host_task;
#endif

    g_sentai_emu_boot_state = kBootAfterUsbHostCtor;
    g_sentai_emu_step = 2;
    UartWrite("step 2: UsbHostTask constructor returned\r\n");

#if SENTAI_EMU_USBHOST_PROBE_START_TASK
    UartWrite("step 3: about to call UsbHostTask::Init\r\n");
    g_sentai_emu_boot_state = kBootBeforeUsbHostInit;
    g_sentai_emu_step = 3;

    host_task->Init();

    g_sentai_emu_boot_state = kBootAfterUsbHostInit;
    g_sentai_emu_step = 4;
    UartWrite("step 4: UsbHostTask::Init returned\r\n");

#if SENTAI_EMU_EDGETPU_MANAGER_PROBE
    UartWrite("step 6: about to construct EdgeTpuManager\r\n");
    g_sentai_emu_step = 6;
    coralmicro::EdgeTpuManager *manager =
        coralmicro::EdgeTpuManager::GetSingleton();
#if !SENTAI_EMU_EDGETPU_MMIO_OPENDEVICE_PROBE
    manager->NotifyError();
#else
    (void)manager;
#endif
    g_sentai_emu_step = 7;
    UartWrite("step 7: EdgeTpuManager singleton returned\r\n");
#endif

#if SENTAI_EMU_EDGETPU_TASK_PROBE
    UartWrite("step 9: about to init EdgeTpuTask\r\n");
    g_sentai_emu_step = 9;
    coralmicro::EdgeTpuTask::GetSingleton()->Init();
    g_sentai_emu_step = 10;
    UartWrite("step 10: EdgeTpuTask::Init returned\r\n");
#endif

    TaskHandle_t heartbeat = xTaskCreateStatic(
        HeartbeatTask, "emu_usbhost_hb", kHeartbeatStackWords, nullptr,
        tskIDLE_PRIORITY + 1, g_heartbeat_stack, &g_heartbeat_tcb);
    if (!heartbeat) {
        g_sentai_emu_boot_state = kBootHeartbeatCreateFailed;
            while (true) {
        }
    }

#if SENTAI_EMU_EDGETPU_OPENDEVICE_PROBE
    TaskHandle_t open_device = xTaskCreateStatic(
        OpenDeviceTask, "emu_tpu_open", kOpenDeviceStackWords, nullptr,
        tskIDLE_PRIORITY + 2, g_open_device_stack, &g_open_device_tcb);
    if (!open_device) {
        g_sentai_emu_boot_state = kBootHeartbeatCreateFailed;
        while (true) {
        }
    }
#endif

#if SENTAI_EMU_EDGETPU_SYNTH_ENUM_PROBE
    TaskHandle_t synthetic_enum = xTaskCreateStatic(
        SyntheticEnumTask, "emu_tpu_enum", kSyntheticEnumStackWords, host_task,
        tskIDLE_PRIORITY + 2, g_synthetic_enum_stack, &g_synthetic_enum_tcb);
    if (!synthetic_enum) {
        g_sentai_emu_boot_state = kBootHeartbeatCreateFailed;
        while (true) {
        }
    }
#endif
#if SENTAI_EMU_EDGETPU_MMIO_ENUM_PROBE
    g_mmio_enum_task = xTaskCreateStatic(
        MmioEnumTask, "emu_tpu_mmio", kMmioEnumStackWords, host_task,
        tskIDLE_PRIORITY + 2, g_mmio_enum_stack, &g_mmio_enum_tcb);
    if (!g_mmio_enum_task) {
        g_sentai_emu_boot_state = kBootHeartbeatCreateFailed;
        while (true) {
        }
    }
#endif

#if SENTAI_EMU_EDGETPU_MANAGER_PROBE
#if SENTAI_EMU_EDGETPU_TASK_PROBE
    g_sentai_emu_step = 11;
#else
    g_sentai_emu_step = 8;
#endif
#else
    g_sentai_emu_step = 5;
#endif
    UartWrite("step 5: starting scheduler\r\n");
    vTaskStartScheduler();

    g_sentai_emu_boot_state = kBootSchedulerReturned;
#else
    // P1 does NOT call Init() — that would create a FreeRTOS task and
    // start spinning the EHCI driver.  Just spin in main for the probe.
    g_sentai_emu_boot_state = kBootSchedulerNotStarted;
#endif
    while (true) {
    }
}
