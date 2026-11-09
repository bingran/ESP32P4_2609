#include <stdint.h>
#include "battery.h"
#include "adc_shared.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BATTERY_GPIO 49
#define SAMPLE_COUNT 32
// Schematic R7 = 100k, R8 = 100k. VBATT = VADC * (R7 + R8) / R8.
#define DIVIDER_TOP_OHM 100000
#define DIVIDER_BOTTOM_OHM 100000

static const char *TAG = "BATTERY";
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;

esp_err_t battery_init(void)
{
    if (s_cali) return ESP_OK;
    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BATTERY_GPIO, &unit, &s_channel),
                        TAG, "GPIO mapping failed");
    ESP_RETURN_ON_ERROR(adc_shared_get_unit(unit, &s_adc), TAG, "ADC init failed");
    const adc_oneshot_chan_cfg_t channel = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_oneshot_config_channel(s_adc, s_channel, &channel);
    if (ret == ESP_OK) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        const adc_cali_curve_fitting_config_t cal = {
            .unit_id = unit, .chan = s_channel,
            .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cal, &s_cali);
#else
        ret = ESP_ERR_NOT_SUPPORTED;
#endif
    }
    if (ret != ESP_OK) {
        s_adc = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "GPIO%d calibrated ADC; divider 100k/100k (x2)", BATTERY_GPIO);
    return ESP_OK;
}

esp_err_t battery_read(battery_reading_t *reading)
{
    if (!reading) return ESP_ERR_INVALID_ARG;
    if (!s_adc || !s_cali) return ESP_ERR_INVALID_STATE;
    int raw_sum = 0, mv_sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        int raw, mv;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, s_channel, &raw), TAG, "Read failed");
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, raw, &mv), TAG, "Calibration failed");
        raw_sum += raw;
        mv_sum += mv;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    reading->raw = raw_sum / SAMPLE_COUNT;
    reading->adc_mv = mv_sum / SAMPLE_COUNT;
    reading->battery_mv = (int)(((int64_t)mv_sum * (DIVIDER_TOP_OHM + DIVIDER_BOTTOM_OHM)) /
                               ((int64_t)SAMPLE_COUNT * DIVIDER_BOTTOM_OHM));
    return ESP_OK;
}
