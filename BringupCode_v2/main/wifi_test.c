#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "wifi_test.h"

static const char *TAG = "WIFI_TEST";

esp_err_t wifi_test_run(void)
{
    ESP_LOGI(TAG, "SDIO slot=%d CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d RESET=%d",
             CONFIG_ESP_HOSTED_SDIO_SLOT, CONFIG_ESP_HOSTED_SDIO_PIN_CLK,
             CONFIG_ESP_HOSTED_SDIO_PIN_CMD, CONFIG_ESP_HOSTED_SDIO_PIN_D0,
             CONFIG_ESP_HOSTED_SDIO_PIN_D1, CONFIG_ESP_HOSTED_SDIO_PIN_D2,
             CONFIG_ESP_HOSTED_SDIO_PIN_D3, CONFIG_ESP_HOSTED_SDIO_GPIO_RESET_SLAVE);
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS initialization failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Network initialization failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop failed");

    int rc = esp_hosted_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "Hosted initialization failed: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connecting to existing C5 firmware...");
    rc = esp_hosted_connect_to_slave();
    if (rc != 0) {
        ESP_LOGE(TAG, "C5 connection failed: %d", rc);
        ESP_LOGE(TAG, "SD is not mounted in this test; this is a C5 transport failure.");
        ESP_LOGE(TAG, "Power-cycle the whole board. If it repeats, capture the C5 UART boot log.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "C5 transport connected");
    esp_hosted_coprocessor_fwver_t version = {0};
    rc = esp_hosted_get_coprocessor_fwversion(&version);
    if (rc == 0) {
        ESP_LOGI(TAG, "C5 firmware: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 version.major1, version.minor1, version.patch1);
    } else {
        ESP_LOGW(TAG, "Firmware version query failed: %d; testing Wi-Fi API next", rc);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "Wi-Fi initialization failed");
    esp_err_t ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret == ESP_OK) ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret == ESP_OK) ret = esp_wifi_start();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Scanning nearby Wi-Fi networks...");
        ret = esp_wifi_scan_start(NULL, true);
    }
    uint16_t total = 0;
    if (ret == ESP_OK) ret = esp_wifi_scan_get_ap_num(&total);
    if (ret == ESP_OK) {
        wifi_ap_record_t records[10] = {0};
        uint16_t count = 10;
        ret = esp_wifi_scan_get_ap_records(&count, records);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi scan PASS: %u networks found", (unsigned)total);
            for (uint16_t i = 0; i < count; ++i) {
                ESP_LOGI(TAG, "SSID: %.32s | RSSI: %d | channel: %u",
                         (const char *)records[i].ssid, records[i].rssi,
                         (unsigned)records[i].primary);
            }
        }
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(ret));
    esp_wifi_stop();
    esp_wifi_deinit();
    return ret;
}
