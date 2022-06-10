#include "apps/MFGTest/mfg_test_iperf.h"
#include "libs/base/check.h"
#include "libs/base/filesystem.h"
#include "libs/base/gpio.h"
#include "libs/base/led.h"
#include "libs/base/mutex.h"
#include "libs/base/strings.h"
#include "libs/base/utils.h"
#include "libs/base/wifi.h"
#include "libs/rpc/rpc_http_server.h"
#include "libs/testlib/test_lib.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/nxp/rt1176-sdk/middleware/wiced/43xxx_BLE/bt_app_inc/wiced_bt_stack.h"
#include "third_party/nxp/rt1176-sdk/middleware/wiced/43xxx_Wi-Fi/WICED/WWD/wwd_wiced.h"
extern "C" {
#include "libs/nxp/rt1176-sdk/rtos/freertos/libraries/abstractions/wifi/include/iot_wifi.h"
}

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

namespace {
using coral::micro::testlib::JsonRpcGetStringParam;

void WiFiGetAP(struct jsonrpc_request* request) {
    std::string name;
    if (!JsonRpcGetStringParam(request, "name", &name)) return;

    WIFIReturnCode_t wifi_ret;
    constexpr int kNumResults = 50;
    WIFIScanResult_t xScanResults[kNumResults] = {0};
    wifi_ret = WIFI_Scan(xScanResults, kNumResults);

    if (wifi_ret != eWiFiSuccess) {
        jsonrpc_return_error(request, -1, "wifi scan failed", nullptr);
        return;
    }

    std::vector<int> scan_indices;
    for (int i = 0; i < kNumResults; ++i) {
        if (memcmp(xScanResults[i].cSSID, name.c_str(), name.size()) == 0) {
            scan_indices.push_back(i);
        }
    }

    if (scan_indices.empty()) {
        jsonrpc_return_error(request, -1, "network not found", nullptr);
        return;
    }

    int8_t best_rssi = SCHAR_MIN;
    for (auto scan_indice : scan_indices) {
        if (xScanResults[scan_indice].cRSSI > best_rssi) {
            best_rssi = xScanResults[scan_indice].cRSSI;
        }
    }

    jsonrpc_return_success(request, "{%Q:%d}", "signal_strength", best_rssi);
}

SemaphoreHandle_t ble_ready_mtx;
bool ble_ready = false;
SemaphoreHandle_t ble_scan_sema;
void BLEFind(struct jsonrpc_request* request) {
    {
        coral::micro::MutexLock lock(ble_ready_mtx);
        if (!ble_ready) {
            jsonrpc_return_error(request, -1, "bt not ready yet", nullptr);
            return;
        }
    }

    std::string address;
    if (!JsonRpcGetStringParam(request, "address", &address)) return;

    unsigned int a, b, c, d, e, f;
    int tokens = sscanf(address.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X", &a,
                        &b, &c, &d, &e, &f);
    if (tokens != 6) {
        jsonrpc_return_error(
            request, -1, "could not get six octets from 'address'", nullptr);
        return;
    }
    static int8_t rssi;
    static wiced_bt_device_address_t search_address;
    search_address[0] = static_cast<uint8_t>(a);
    search_address[1] = static_cast<uint8_t>(b);
    search_address[2] = static_cast<uint8_t>(c);
    search_address[3] = static_cast<uint8_t>(d);
    search_address[4] = static_cast<uint8_t>(e);
    search_address[5] = static_cast<uint8_t>(f);

    // This static here is because there isn't a way to pass a parameter into
    // the scan. Reset the value each time through the method.
    static bool found_match;
    found_match = false;
    wiced_result_t ret = wiced_bt_ble_observe(
        WICED_TRUE, 3,
        [](wiced_bt_ble_scan_results_t* p_scan_result, uint8_t* p_adv_data) {
            if (p_scan_result) {
                if (memcmp(search_address, p_scan_result->remote_bd_addr,
                           sizeof(wiced_bt_device_address_t)) == 0) {
                    found_match = true;
                    rssi = p_scan_result->rssi;
                }
            } else {
                CHECK(xSemaphoreGive(ble_scan_sema) == pdTRUE);
            }
        });
    if (ret != WICED_BT_PENDING) {
        jsonrpc_return_error(request, -1, "failed to initiate bt scan",
                             nullptr);
        return;
    }
    CHECK(xSemaphoreTake(ble_scan_sema, portMAX_DELAY) == pdTRUE);
    if (found_match) {
        jsonrpc_return_success(request, "{%Q:%d}", "signal_strength", rssi);
    } else {
        jsonrpc_return_error(request, -1, "failed to find 'address'", nullptr);
    }
}

void BLEScan(struct jsonrpc_request* request) {
    {
        coral::micro::MutexLock lock(ble_ready_mtx);
        if (!ble_ready) {
            jsonrpc_return_error(request, -1, "bt not ready yet", nullptr);
            return;
        }
    }

    static int8_t rssi;
    static char address[18] = "00:00:00:00:00:00";

    // This static here is because there isn't a way to pass a parameter into
    // the scan. Reset the value each time through the method.
    static bool found_address;
    found_address = false;
    wiced_result_t ret = wiced_bt_ble_observe(
        WICED_TRUE, 3,
        [](wiced_bt_ble_scan_results_t* p_scan_result, uint8_t* p_adv_data) {
            if (p_scan_result) {
                found_address = true;
                auto s = p_scan_result->remote_bd_addr;
                sprintf(address, "%02X:%02X:%02X:%02X:%02X:%02X", s[0], s[1],
                        s[2], s[3], s[4], s[5]);
                rssi = p_scan_result->rssi;
            } else {
                CHECK(xSemaphoreGive(ble_scan_sema) == pdTRUE);
            }
        });
    if (ret != WICED_BT_PENDING) {
        jsonrpc_return_error(request, -1, "failed to initiate bt scan",
                             nullptr);
        return;
    }
    CHECK(xSemaphoreTake(ble_scan_sema, portMAX_DELAY) == pdTRUE);
    if (found_address) {
        jsonrpc_return_success(request, "{%Q:%Q, %Q:%d}", "address", address,
                               "signal_strength", rssi);
    } else {
        jsonrpc_return_error(request, -1, "failed to find 'address'", nullptr);
    }
}

wiced_result_t ble_management_callback(
    wiced_bt_management_evt_t event,
    wiced_bt_management_evt_data_t* p_event_data) {
    switch (event) {
        case BTM_ENABLED_EVT: {
            coral::micro::MutexLock lock(ble_ready_mtx);
            if (((wiced_bt_dev_enabled_t*)(p_event_data))->status ==
                WICED_SUCCESS) {
                ble_ready = true;
            } else {
                ble_ready = false;
            }
        } break;
        case BTM_LPM_STATE_LOW_POWER:
            break;
        default:
            return WICED_BT_ERROR;
            break;
    }
    return WICED_BT_SUCCESS;
}
}  // namespace

extern unsigned char brcm_patchram_buf[];
extern unsigned int brcm_patch_ram_length;
extern "C" void app_main(void* param) {
    ble_scan_sema = xSemaphoreCreateBinary();
    CHECK(ble_scan_sema);
    ble_ready_mtx = xSemaphoreCreateMutex();
    CHECK(ble_ready_mtx);
    if (coral::micro::filesystem::ReadFile(
            "/third_party/cyw-bt-patch/BCM4345C0_003.001.025.0144.0266.1MW.hcd",
            brcm_patchram_buf,
            brcm_patch_ram_length) != brcm_patch_ram_length) {
        printf("Reading patchram failed\r\n");
        vTaskSuspend(nullptr);
    }

    if (coral::micro::TurnOnWiFi()) {
        if (coral::micro::ConnectWiFi()) {
            coral::micro::led::Set(coral::micro::led::LED::kUser, true);
        }
    } else {
        printf("Wi-Fi failed to come up (is the Wi-Fi board attached?\r\n");
        coral::micro::led::Set(coral::micro::led::LED::kPower, true);
        vTaskSuspend(nullptr);
    }
    coral::micro::gpio::SetGpio(coral::micro::gpio::kBtDevWake, false);
    wiced_bt_stack_init(ble_management_callback, &wiced_bt_cfg_settings,
                        wiced_bt_cfg_buf_pools);

    jsonrpc_init(nullptr, nullptr);
    jsonrpc_export(coral::micro::testlib::kMethodWiFiScan,
                   coral::micro::testlib::WiFiScan);
    jsonrpc_export("wifi_get_ap", WiFiGetAP);
    jsonrpc_export(coral::micro::testlib::kMethodWiFiSetAntenna,
                   coral::micro::testlib::WiFiSetAntenna);
    jsonrpc_export("ble_scan", BLEScan);
    jsonrpc_export("ble_find", BLEFind);
    IperfInit();
    coral::micro::UseHttpServer(new coral::micro::JsonRpcHttpServer);
    vTaskSuspend(nullptr);
}
