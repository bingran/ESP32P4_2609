#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "camera.h"
#include "camera_web.h"

static const char *TAG = "CAMERA_WEB";
#define CONNECTED_BIT BIT0
#define FAILED_BIT BIT1
static EventGroupHandle_t s_connection;
static unsigned s_retries;
static httpd_handle_t s_page_server;
static httpd_handle_t s_stream_server;

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (esp_wifi_connect() != ESP_OK) xEventGroupSetBits(s_connection, FAILED_BIT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = data;
        xEventGroupClearBits(s_connection, CONNECTED_BIT);
        ESP_LOGW(TAG, "Router disconnected, reason=%u", (unsigned)event->reason);
        if (s_retries++ < 5) {
            if (esp_wifi_connect() != ESP_OK) xEventGroupSetBits(s_connection, FAILED_BIT);
        } else {
            xEventGroupSetBits(s_connection, FAILED_BIT);
            ESP_LOGE(TAG, "Connection retries exhausted; check Wi-Fi settings and restart");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        s_retries = 0;
        xEventGroupClearBits(s_connection, FAILED_BIT);
        xEventGroupSetBits(s_connection, CONNECTED_BIT);
        ESP_LOGI(TAG, "Router connected. Camera address: http://" IPSTR "/",
                 IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t connect_router(void)
{
    const char *ssid = CONFIG_BRINGUP_WIFI_SSID;
    const char *password = CONFIG_BRINGUP_WIFI_PASSWORD;
    wifi_config_t wifi = {0};
    if (!ssid[0] || strlen(ssid) > sizeof(wifi.sta.ssid) ||
        strlen(password) > sizeof(wifi.sta.password)) {
        ESP_LOGE(TAG, "Set valid router credentials in menuconfig -> Router Wi-Fi settings");
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(wifi.sta.ssid, ssid, strlen(ssid));
    memcpy(wifi.sta.password, password, strlen(password));
    wifi.sta.pmf_cfg.capable = true;
    wifi.sta.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS initialization failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Network initialization failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop failed");
    s_connection = xEventGroupCreate();
    if (!s_connection) return ESP_ERR_NO_MEM;
    if (!esp_netif_create_default_wifi_sta()) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL),
                        TAG, "Wi-Fi event handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL),
                        TAG, "IP event handler failed");
    if (esp_hosted_init() != 0 || esp_hosted_connect_to_slave() != 0) {
        ESP_LOGE(TAG, "C5 connection failed");
        return ESP_FAIL;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "Wi-Fi initialization failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi), TAG, "Wi-Fi configuration failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Connecting to configured router...");
    EventBits_t bits = xEventGroupWaitBits(s_connection, CONNECTED_BIT | FAILED_BIT,
                                          pdFALSE, pdFALSE, pdMS_TO_TICKS(60000));
    return (bits & CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t page_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>P4 Camera</title><style>body{background:#151719;color:#eee;font:18px system-ui;"
        "text-align:center;margin:24px}img{max-width:100%;height:auto;background:#000}"
        "button{padding:10px 20px;font:inherit;margin:12px}</style></head><body>"
        "<h1>P4 Camera</h1><p>1920 &times; 1080 live view</p><img id='camera' alt='Camera stream'>"
        "<p><button onclick='start()'>Start / reconnect</button>"
        "<button onclick=\"document.getElementById('camera').removeAttribute('src')\">Stop</button></p>"
        "<p>One viewer at a time.</p><script>function start(){document.getElementById('camera').src="
        "'http://'+location.hostname+':81/stream?t='+Date.now()}start()</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_jpeg(const uint8_t *jpeg, size_t size, void *ctx)
{
    httpd_req_t *req = ctx;
    char header[128];
    int length = snprintf(header, sizeof(header),
        "\r\n--p4frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)size);
    if (length < 0 || length >= (int)sizeof(header)) return ESP_FAIL;
    esp_err_t ret = httpd_resp_send_chunk(req, header, length);
    if (ret == ESP_OK) ret = httpd_resp_send_chunk(req, (const char *)jpeg, size);
    return ret;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=p4frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Viewer connected; camera warming up");
    esp_err_t ret = camera_stream_jpeg(send_jpeg, req);
    ESP_LOGI(TAG, "Stream ended: %s", esp_err_to_name(ret));
    /* An error closes the session; a normal disconnect ends capture too. */
    return ESP_FAIL;
}

esp_err_t camera_web_start(void)
{
    ESP_RETURN_ON_ERROR(connect_router(), TAG, "Router connection failed");
    /* app_main owns hardware initialization and the initial SD snapshot. */
    if (!camera_is_initialized()) {
        ESP_LOGE(TAG, "Router connected, but camera initialization did not succeed");
        return ESP_ERR_INVALID_STATE;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    ESP_RETURN_ON_ERROR(httpd_start(&s_page_server, &config), TAG, "Web server failed");
    httpd_uri_t page = {.uri = "/", .method = HTTP_GET, .handler = page_handler};
    esp_err_t ret = httpd_register_uri_handler(s_page_server, &page);
    if (ret == ESP_OK) {
        config.server_port = 81;
        config.ctrl_port += 1;
        config.send_wait_timeout = 5;
        config.max_open_sockets = 1;
        config.lru_purge_enable = false;
        ret = httpd_start(&s_stream_server, &config);
    }
    if (ret == ESP_OK) {
        httpd_uri_t stream = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
        ret = httpd_register_uri_handler(s_stream_server, &stream);
    }
    if (ret != ESP_OK) {
        if (s_stream_server) httpd_stop(s_stream_server);
        httpd_stop(s_page_server);
        return ret;
    }
    ESP_LOGI(TAG, "Camera web server ready; open the printed address on the same network");
    return ESP_OK;
}
