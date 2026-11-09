
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "thermistor.h"
#include "adc_shared.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define TAG                 "THERMISTOR"

#define THERM_ADC_UNIT      ADC_UNIT_2
#define THERM_ADC_CHANNEL   ADC_CHANNEL_1

#define VCC_MV              3336.0f
#define R_FIXED             10000.0f
#define R_NOMINAL           10000.0f
#define BETA                3950.0f
#define T_NOMINAL_K         298.15f

#define SAMPLE_COUNT        32

// =====================================================
// GLOBAL HANDLES
// =====================================================

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// =====================================================
// ADC CALIBRATION
// =====================================================

static esp_err_t thermistor_calibration_init(void)
{
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    adc_cali_curve_fitting_config_t curve_config = {
        .unit_id = THERM_ADC_UNIT,
        .chan = THERM_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_cali_create_scheme_curve_fitting(
        &curve_config,
        &adc_cali_handle
    );

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Curve fitting calibration enabled");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Curve fitting unavailable: %s",
             esp_err_to_name(ret));

#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    adc_cali_line_fitting_config_t line_config = {
        .unit_id = THERM_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_cali_create_scheme_line_fitting(
        &line_config,
        &adc_cali_handle
    );

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Line fitting calibration enabled");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Line fitting unavailable: %s",
             esp_err_to_name(ret));

#endif

    return ret;
}

// =====================================================
// INITIALIZATION
// =====================================================

esp_err_t thermistor_init(void)
{
    int gpio_num = -1;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_channel_to_io(
            THERM_ADC_UNIT,
            THERM_ADC_CHANNEL,
            &gpio_num
        ),
        TAG,
        "ADC GPIO mapping failed"
    );

    ESP_LOGI(TAG, "ADC channel mapped to GPIO%d", gpio_num);

    ESP_RETURN_ON_ERROR(
        adc_shared_get_unit(
            THERM_ADC_UNIT,
            &adc_handle
        ),
        TAG,
        "ADC initialization failed"
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(
            adc_handle,
            THERM_ADC_CHANNEL,
            &channel_config
        ),
        TAG,
        "ADC channel configuration failed"
    );

    ESP_RETURN_ON_ERROR(
        thermistor_calibration_init(),
        TAG,
        "ADC calibration initialization failed"
    );

    ESP_LOGI(TAG, "Thermistor initialized");

    return ESP_OK;
}

// =====================================================
// MEASUREMENT
// =====================================================

esp_err_t thermistor_read(thermistor_reading_t *reading)
{
    if (reading == NULL || adc_handle == NULL ||
        adc_cali_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int voltage_mv = 0;

    int64_t raw_sum = 0;
    int64_t voltage_sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {

        ESP_RETURN_ON_ERROR(
            adc_oneshot_read(
                adc_handle,
                THERM_ADC_CHANNEL,
                &raw
            ),
            TAG,
            "ADC read failed"
        );

        ESP_RETURN_ON_ERROR(
            adc_cali_raw_to_voltage(
                adc_cali_handle,
                raw,
                &voltage_mv
            ),
            TAG,
            "ADC voltage conversion failed"
        );

        raw_sum += raw;
        voltage_sum += voltage_mv;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    reading->raw = (int)(raw_sum / SAMPLE_COUNT);
    reading->voltage_mv = (int)(voltage_sum / SAMPLE_COUNT);

    if (reading->voltage_mv <= 2 ||
        reading->voltage_mv >= (int)VCC_MV - 2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Wiring:
    // VCC -> 10k fixed resistor -> ADC -> NTC -> GND

    reading->resistance_ohm =
        R_FIXED *
        (
            (float)reading->voltage_mv /
            (VCC_MV - (float)reading->voltage_mv)
        );

    float temperature_k = 1.0f /
        (
            (1.0f / T_NOMINAL_K) +
            (
                logf(reading->resistance_ohm / R_NOMINAL) /
                BETA
            )
        );

    reading->temperature_c = temperature_k - 273.15f;

    return ESP_OK;
}
