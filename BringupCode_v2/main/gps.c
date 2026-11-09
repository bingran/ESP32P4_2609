#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"

#define GPS_UART UART_NUM_1
#define GPS_RX_GPIO 33
#define GPS_TX_GPIO 32
#define GPS_EN_GPIO 46
static const char *TAG = "GPS";
static bool s_initialized;
static TickType_t s_start_tick;
static bool s_first_fix;

/* Keep each talker/signal separate: adding GSV counts across signals would
 * double-count satellites. Collect all parts before reporting signal stats. */
typedef struct {
    char talker[3];
    unsigned signal, next_part, total_parts, with_signal, peak_cno;
    TickType_t last_report;
    bool used, reported;
} gps_gsv_t;
static gps_gsv_t s_gsv[16];

static void report_gsv(char **fields, unsigned count)
{
    if (count < 4) return;
    unsigned parts = (unsigned)strtoul(fields[1], NULL, 10);
    unsigned part = (unsigned)strtoul(fields[2], NULL, 10);
    unsigned in_view = (unsigned)strtoul(fields[3], NULL, 10);
    unsigned groups = (count - 4) / 4;
    unsigned signal = ((count - 4) % 4 == 1) ? (unsigned)strtoul(fields[count - 1], NULL, 10) : 0;
    if (!parts || !part || part > parts || groups > 4) return;
    gps_gsv_t *state = NULL;
    for (unsigned i = 0; i < 16; ++i) {
        if (s_gsv[i].used && s_gsv[i].talker[0] == fields[0][1] &&
            s_gsv[i].talker[1] == fields[0][2] && s_gsv[i].signal == signal) {
            state = &s_gsv[i];
            break;
        }
    }
    if (!state) {
        for (unsigned i = 0; i < 16; ++i) {
            if (!s_gsv[i].used) {
                state = &s_gsv[i];
                state->used = true;
                state->talker[0] = fields[0][1];
                state->talker[1] = fields[0][2];
                state->signal = signal;
                break;
            }
        }
    }
    if (!state) return;
    if (part == 1) {
        state->with_signal = 0;
        state->peak_cno = 0;
        state->next_part = 1;
        state->total_parts = parts;
    }
    if (state->next_part != part || state->total_parts != parts) return;
    for (unsigned i = 0; i < groups; ++i) {
        const char *cno = fields[4 + i * 4 + 3];
        if (*fields[4 + i * 4] && *cno) {
            unsigned value = (unsigned)strtoul(cno, NULL, 10);
            if (value > 0) {
                ++state->with_signal;
                if (value > state->peak_cno) state->peak_cno = value;
            }
        }
    }
    ++state->next_part;
    TickType_t now = xTaskGetTickCount();
    if (part == parts && (!state->reported || now - state->last_report >= pdMS_TO_TICKS(5000))) {
        state->reported = true;
        state->last_report = now;
        ESP_LOGI(TAG, "Sky %s signal=%u: in_view=%u with_signal=%u strongest_C/N0=%u dB-Hz",
                 state->talker, signal, in_view, state->with_signal, state->peak_cno);
    }
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool valid_nmea(const char *line)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) != 3) return false;
    int high = hex_digit(star[1]), low = hex_digit(star[2]);
    if (high < 0 || low < 0) return false;
    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; ++p) checksum ^= (uint8_t)*p;
    return checksum == (uint8_t)((high << 4) | low);
}

static bool coordinate(const char *value, const char *hemisphere, bool latitude, double *out)
{
    if (!*value || strlen(hemisphere) != 1) return false;
    char h = hemisphere[0];
    if (latitude ? (h != 'N' && h != 'S') : (h != 'E' && h != 'W')) return false;
    char *end;
    double packed = strtod(value, &end);
    if (*end || !(packed >= 0.0 && packed <= 18000.0)) return false;
    int degrees = (int)(packed / 100.0);
    double minutes = packed - degrees * 100.0;
    if (minutes >= 60.0) return false;
    double result = degrees + minutes / 60.0;
    if (result > (latitude ? 90.0 : 180.0)) return false;
    if (h == 'S' || h == 'W') result = -result;
    *out = result;
    return true;
}

static void report_nmea(char *line)
{
    char *fields[24];
    unsigned count = 1;
    fields[0] = line;
    /* Preserve empty fields: no-fix NMEA commonly omits coordinates. */
    for (char *p = line; *p; ++p) {
        if (*p == '*') { *p = '\0'; break; }
        if (*p == ',') {
            *p = '\0';
            if (count < 24) fields[count++] = p + 1;
            else break;
        }
    }
    if (strlen(fields[0]) != 6) return;
    static TickType_t last_gga, last_rmc;
    static bool seen_gga, seen_rmc;
    TickType_t now = xTaskGetTickCount();
    if (!strcmp(fields[0] + 3, "GSV")) {
        report_gsv(fields, count);
    } else if (!strcmp(fields[0] + 3, "GGA") && count >= 11) {
        if (seen_gga && now - last_gga < pdMS_TO_TICKS(5000)) return;
        last_gga = now;
        seen_gga = true;
        unsigned quality = (unsigned)strtoul(fields[6], NULL, 10);
        unsigned satellites = (unsigned)strtoul(fields[7], NULL, 10);
        double lat, lon;
        if (quality && coordinate(fields[2], fields[3], true, &lat) &&
            coordinate(fields[4], fields[5], false, &lon)) {
            if (!s_first_fix) {
                ESP_LOGI(TAG, "First GGA position fix observed %lu seconds after GPS startup",
                         (unsigned long)((now - s_start_tick) / configTICK_RATE_HZ));
                s_first_fix = true;
            }
            ESP_LOGI(TAG, "FIX quality=%u satellites_used=%u HDOP=%s lat=%.6f lon=%.6f altitude=%s%s UTC=%s",
                     quality, satellites, fields[8], lat, lon, fields[9], fields[10], fields[1]);
        } else {
            ESP_LOGI(TAG, "NO FIX: satellites_used=%u HDOP=%s uptime=%lus; see Sky reports for reception",
                     satellites, *fields[8] ? fields[8] : "unavailable",
                     (unsigned long)((now - s_start_tick) / configTICK_RATE_HZ));
        }
    } else if (!strcmp(fields[0] + 3, "RMC") && count >= 10) {
        if (seen_rmc && now - last_rmc < pdMS_TO_TICKS(5000)) return;
        last_rmc = now;
        seen_rmc = true;
        if (!strcmp(fields[2], "A")) {
            double lat, lon;
            if (coordinate(fields[3], fields[4], true, &lat) &&
                coordinate(fields[5], fields[6], false, &lon)) {
                ESP_LOGI(TAG, "Position valid: lat=%.6f lon=%.6f speed=%.1f km/h date(ddmmyy)=%s UTC=%s",
                         lat, lon, strtod(fields[7], NULL) * 1.852, fields[9], fields[1]);
            }
        }
    }
}

static void gps_task(void *arg)
{
    (void)arg;
    const int baud_rates[] = {9600, 38400};
    unsigned baud_index = 0;
    bool locked = false;
    char line[256];
    size_t used = 0;
    uint32_t received = 0;
    TickType_t last_valid = xTaskGetTickCount();
    TickType_t probe_start = last_valid;
    for (;;) {
        uint8_t bytes[128];
        int n = uart_read_bytes(GPS_UART, bytes, sizeof(bytes), pdMS_TO_TICKS(200));
        if (n > 0) received += n;
        for (int i = 0; i < n; ++i) {
            char c = (char)bytes[i];
            if (c == '$') { used = 0; line[used++] = c; }
            else if (c == '\n') {
                line[used] = '\0';
                if (valid_nmea(line)) {
                    last_valid = xTaskGetTickCount();
                    if (!locked) {
                        ESP_LOGI(TAG, "UART PASS: checksum-valid NMEA at %d baud", baud_rates[baud_index]);
                        locked = true;
                    }
                    report_nmea(line);
                }
                used = 0;
            } else if (c != '\r' && used) {
                if (c >= 32 && c <= 126 && used < sizeof(line) - 1) line[used++] = c;
                else used = 0;
            }
        }
        TickType_t now = xTaskGetTickCount();
        if (!locked && now - probe_start >= pdMS_TO_TICKS(10000)) {
            ESP_LOGW(TAG, "No valid NMEA at %d baud (%lu bytes); trying alternate baud",
                     baud_rates[baud_index], (unsigned long)received);
            baud_index ^= 1;
            esp_err_t ret = uart_set_baudrate(GPS_UART, baud_rates[baud_index]);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Baud change failed: %s", esp_err_to_name(ret));
                vTaskDelete(NULL);
                return;
            }
            uart_flush_input(GPS_UART);
            used = 0;
            received = 0;
            probe_start = now;
        } else if (locked && now - last_valid >= pdMS_TO_TICKS(10000)) {
            ESP_LOGW(TAG, "No valid NMEA for 10 seconds; check GPS power and UART connection");
            last_valid = now;
        }
    }
}

esp_err_t gps_init(void)
{
    if (s_initialized) return ESP_OK;
    const gpio_config_t enable = {
        .pin_bit_mask = 1ULL << GPS_EN_GPIO, .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret = gpio_config(&enable);
    if (ret != ESP_OK) return ret;
    ret = gpio_set_level(GPS_EN_GPIO, 1);
    if (ret != ESP_OK) return ret;
    s_start_tick = xTaskGetTickCount();
    const uart_config_t config = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ret = uart_driver_install(GPS_UART, 4096, 0, 0, NULL, 0);
    if (ret != ESP_OK) goto failed;
    ret = uart_param_config(GPS_UART, &config);
    if (ret == ESP_OK) ret = uart_set_pin(GPS_UART, GPS_TX_GPIO, GPS_RX_GPIO,
                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret == ESP_OK && xTaskCreate(gps_task, "gps_nmea", 6144, NULL, 5, NULL) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
    }
    if (ret != ESP_OK) { uart_driver_delete(GPS_UART); goto failed; }
    s_initialized = true;
    ESP_LOGI(TAG, "MAX-M10S UART1: ESP RX=GPIO%d TX=GPIO%d EN=GPIO%d HIGH; 9600 8N1",
             GPS_RX_GPIO, GPS_TX_GPIO, GPS_EN_GPIO);
    ESP_LOGI(TAG, "Waiting for NMEA; UART reception and satellite fix are separate checks");
    return ESP_OK;
failed:
    gpio_set_level(GPS_EN_GPIO, 0);
    return ret;
}
