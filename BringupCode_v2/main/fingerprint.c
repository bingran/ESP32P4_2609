#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fingerprint.h"

#define FP_UART UART_NUM_2
#define FP_TX_GPIO 29
#define FP_RX_GPIO 28
#define FP_EN_GPIO 31
#define FP_WAKEUP_GPIO 30
static const char *TAG = "R503";
static bool s_started;
/* Single UART-owner task; retain a small sample for failed-command diagnostics. */
static size_t s_rx_count;
static size_t s_rx_sample_size;
static uint8_t s_rx_sample[32];

static bool read_exact(uint8_t *out, size_t size, TickType_t start, TickType_t timeout)
{
    size_t done = 0;
    while (done < size) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) return false;
        int n = uart_read_bytes(FP_UART, out + done, size - done, timeout - elapsed);
        if (n < 0) return false;
        if (n > 0) {
            size_t copy = sizeof(s_rx_sample) - s_rx_sample_size;
            if (copy > (size_t)n) copy = n;
            memcpy(s_rx_sample + s_rx_sample_size, out + done, copy);
            s_rx_sample_size += copy;
            s_rx_count += n;
        }
        done += n;
    }
    return true;
}

/* All access is owned by fp_task. Validate address, ACK type, size and sum. */
static esp_err_t command(const uint8_t *payload, size_t size, uint8_t *reply, size_t *reply_size)
{
    if (!size || size > 8) return ESP_ERR_INVALID_ARG;
    uint8_t packet[19] = {0xef, 0x01, 0xff, 0xff, 0xff, 0xff, 0x01, 0, 0};
    packet[8] = size + 2;
    memcpy(packet + 9, payload, size);
    uint16_t sum = packet[6] + packet[7] + packet[8];
    for (size_t i = 0; i < size; ++i) sum += payload[i];
    packet[9 + size] = sum >> 8;
    packet[10 + size] = sum & 0xff;
    s_rx_count = 0;
    s_rx_sample_size = 0;
    uart_flush_input(FP_UART);
    if (uart_write_bytes(FP_UART, packet, size + 11) != (int)(size + 11)) return ESP_FAIL;
    TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(1200);
    uint8_t header[9] = {0};
    unsigned synced = 0;
    while (synced < 2) {
        uint8_t c;
        if (!read_exact(&c, 1, start, timeout)) return ESP_ERR_TIMEOUT;
        if (!synced) synced = c == 0xef ? 1 : 0;
        else if (c == 0x01) synced = 2;
        else synced = c == 0xef ? 1 : 0;
    }
    if (!read_exact(header + 2, 7, start, timeout)) return ESP_ERR_TIMEOUT;
    for (unsigned i = 2; i < 6; ++i) {
        if (header[i] != 0xff) return ESP_ERR_INVALID_RESPONSE;
    }
    unsigned length = ((unsigned)header[7] << 8) | header[8];
    uint8_t body[64];
    if (header[6] != 0x07 || length < 3 || length > sizeof(body)) return ESP_ERR_INVALID_RESPONSE;
    if (!read_exact(body, length, start, timeout)) return ESP_ERR_TIMEOUT;
    sum = header[6] + header[7] + header[8];
    for (unsigned i = 0; i < length - 2; ++i) sum += body[i];
    if (sum != (uint16_t)(((unsigned)body[length - 2] << 8) | body[length - 1])) {
        return ESP_ERR_INVALID_CRC;
    }
    if (*reply_size < length - 2) return ESP_ERR_INVALID_SIZE;
    *reply_size = length - 2;
    memcpy(reply, body, *reply_size);
    return ESP_OK;
}

static void fp_task(void *arg)
{
    (void)arg;
    /* Datasheet allows about 200 ms for power-on initialization. */
    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t startup[32];
    int startup_size = uart_read_bytes(FP_UART, startup, sizeof(startup), 0);
    ESP_LOGI(TAG, "Power-on RX: %d bytes; EN GPIO31=%d WAKEUP GPIO30=%d",
             startup_size, gpio_get_level(FP_EN_GPIO), gpio_get_level(FP_WAKEUP_GPIO));
    if (startup_size > 0) ESP_LOG_BUFFER_HEX(TAG, startup, startup_size);
    uint8_t reply[64];
    const uint8_t password[] = {0x13, 0, 0, 0, 0};
    for (;;) {
        size_t size = sizeof(reply);
        esp_err_t ret = command(password, sizeof(password), reply, &size);
        if (ret == ESP_OK && reply[0] == 0) {
            ESP_LOGI(TAG, "UART PASS: R503 default password verified at 9600 baud");
            break;
        }
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "Password verification rejected: code=0x%02x; check configured module password", reply[0]);
        } else {
            ESP_LOGW(TAG, "No valid password ACK: %s; check power, TX/RX, baud and module address",
                     esp_err_to_name(ret));
            ESP_LOGW(TAG, "Password command RX: %u bytes at 9600 baud", (unsigned)s_rx_count);
            if (s_rx_sample_size) ESP_LOG_BUFFER_HEX(TAG, s_rx_sample, s_rx_sample_size);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    const uint8_t parameters[] = {0x0f};
    size_t size = sizeof(reply);
    esp_err_t ret = command(parameters, sizeof(parameters), reply, &size);
    if (ret == ESP_OK && reply[0] == 0 && size >= 17) {
        unsigned capacity = ((unsigned)reply[5] << 8) | reply[6];
        unsigned security = ((unsigned)reply[7] << 8) | reply[8];
        unsigned baud_multiplier = ((unsigned)reply[15] << 8) | reply[16];
        ESP_LOGI(TAG, "System parameters: capacity=%u security=%u UART=%u baud",
                 capacity, security, baud_multiplier * 9600);
    } else ESP_LOGW(TAG, "System parameter query failed");
    const uint8_t templates[] = {0x1d};
    size = sizeof(reply);
    ret = command(templates, sizeof(templates), reply, &size);
    if (ret == ESP_OK && reply[0] == 0 && size >= 3) {
        ESP_LOGI(TAG, "Stored templates: %u (no changes made)", ((unsigned)reply[1] << 8) | reply[2]);
    }
    const uint8_t blue_led[] = {0x35, 0x03, 0x00, 0x02, 0x00};
    size = sizeof(reply);
    ret = command(blue_led, sizeof(blue_led), reply, &size);
    if (ret != ESP_OK || reply[0]) ESP_LOGW(TAG, "LED command not acknowledged");
    ESP_LOGI(TAG, "Place a finger on the sensor: image-capture test only, no enrollment or matching");
    int last_wakeup = -1;
    int last_code = -1;
    TickType_t last_report = 0;
    for (;;) {
        int wakeup = gpio_get_level(FP_WAKEUP_GPIO);
        if (wakeup != last_wakeup) {
            ESP_LOGI(TAG, "WAKEUP GPIO30 level=%d", wakeup);
            last_wakeup = wakeup;
        }
        const uint8_t capture[] = {0x01};
        size = sizeof(reply);
        ret = command(capture, sizeof(capture), reply, &size);
        TickType_t now = xTaskGetTickCount();
        int code = ret == ESP_OK ? reply[0] : -2;
        if (code != last_code || now - last_report >= pdMS_TO_TICKS(10000)) {
            if (ret != ESP_OK) ESP_LOGW(TAG, "Image command failed: %s", esp_err_to_name(ret));
            else if (!code) ESP_LOGI(TAG, "Finger image captured in module RAM: PASS (not identification)");
            else if (code == 0x02) ESP_LOGI(TAG, "Ready; no finger detected");
            else ESP_LOGW(TAG, "Image capture response=0x%02x", (unsigned)code);
            last_code = code;
            last_report = now;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t fingerprint_init(void)
{
    if (s_started) return ESP_OK;
    const gpio_config_t enable = {.pin_bit_mask = 1ULL << FP_EN_GPIO, .mode = GPIO_MODE_OUTPUT};
    const gpio_config_t wake = {.pin_bit_mask = 1ULL << FP_WAKEUP_GPIO, .mode = GPIO_MODE_INPUT};
    esp_err_t ret = gpio_config(&wake);
    if (ret == ESP_OK) ret = gpio_config(&enable);
    if (ret == ESP_OK) ret = gpio_set_level(FP_EN_GPIO, 1);
    if (ret != ESP_OK) return ret;
    const uart_config_t config = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ret = uart_driver_install(FP_UART, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) goto failed;
    ret = uart_param_config(FP_UART, &config);
    if (ret == ESP_OK) ret = uart_set_pin(FP_UART, FP_TX_GPIO, FP_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret == ESP_OK && xTaskCreate(fp_task, "r503_test", 4096, NULL, 5, NULL) != pdPASS) ret = ESP_ERR_NO_MEM;
    if (ret != ESP_OK) { uart_driver_delete(FP_UART); goto failed; }
    s_started = true;
    ESP_LOGI(TAG, "UART2 ESP TX=%d RX=%d; EN=%d HIGH; WAKEUP=%d input; 9600 8N1",
             FP_TX_GPIO, FP_RX_GPIO, FP_EN_GPIO, FP_WAKEUP_GPIO);
    return ESP_OK;
failed:
    gpio_set_level(FP_EN_GPIO, 0);
    return ret;
}
