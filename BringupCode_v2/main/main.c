
/*
 * ESP32-P4C5 Hardware Diagnostic
 * ESP-IDF v5.5.5
 *
 * Flash: 16MB BOYA
 * Thermistor: 10k NTC, Beta 3950
 * ADC: GPIO50 / ADC2_CHANNEL_1
 *
 * Wiring:
 * 3.336V
 *    |
 *  10k fixed resistor
 *    |
 *    +------ GPIO50 (ADC2_CHANNEL_1)
 *    |
 *  10k NTC (Beta 3950)
 *    |
 *   GND
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define TAG                  "HW_TEST"

#define THERM_ADC_UNIT       ADC_UNIT_2
#define THERM_ADC_CHANNEL    ADC_CHANNEL_1

// Measured with multimeter
#define VCC_MV               3336.0f

#define R_FIXED              10000.0f
#define R_NOMINAL            10000.0f
#define BETA                 3950.0f
#define T_NOMINAL_K          298.15f

#define SAMPLE_COUNT         32

// =====================================================
// GLOBAL HANDLES
// =====================================================

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// =====================================================
// ADC CALIBRATION INITIALIZATION
// =====================================================

static esp_err_t thermistor_calibration_init(void)
{
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    printf("Trying ADC curve fitting calibration...\n");

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
        printf("ADC calibration: Curve fitting enabled\n");
        return ESP_OK;
    }

    printf("Curve fitting unavailable: %s\n",
           esp_err_to_name(ret));

#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    printf("Trying ADC line fitting calibration...\n");

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
        printf("ADC calibration: Line fitting enabled\n");
        return ESP_OK;
    }

    printf("Line fitting unavailable: %s\n",
           esp_err_to_name(ret));

#endif

    printf("ERROR: No usable ADC calibration scheme!\n");

    return ret;
}

// =====================================================
// SYSTEM INFORMATION
// =====================================================

static void print_system_info(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size = 0;

    esp_chip_info(&chip_info);

    printf("\n");
    printf("========================================\n");
    printf("       ESP32-P4C5 HARDWARE TEST\n");
    printf("========================================\n");

    printf("Target             : %s\n", CONFIG_IDF_TARGET);
    printf("CPU cores          : %d\n", chip_info.cores);

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;

    printf("Silicon revision   : v%d.%d\n",
           major_rev, minor_rev);

    printf("CPU frequency      : %d MHz\n",
           CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);

    if (chip_info.features & CHIP_FEATURE_WIFI_BGN)
        printf("WiFi               : Supported\n");

    if (chip_info.features & CHIP_FEATURE_BT)
        printf("Bluetooth          : Supported\n");

    if (chip_info.features & CHIP_FEATURE_BLE)
        printf("BLE                : Supported\n");

    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        printf("Flash capacity     : %" PRIu32 " MB\n",
               flash_size / (1024 * 1024));
    } else {
        printf("Flash capacity     : Read failed\n");
    }

    printf("Flash manufacturer : BOYA (from boot detection)\n");

    printf("Minimum free heap  : %" PRIu32 " bytes\n",
           esp_get_minimum_free_heap_size());

    printf("========================================\n");
}

// =====================================================
// THERMISTOR INITIALIZATION
// =====================================================

static esp_err_t thermistor_init(void)
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

    printf("ADC mapping: ADC2_CHANNEL_1 -> GPIO%d\n",
           gpio_num);

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = THERM_ADC_UNIT,
    };

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(
            &init_config,
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

    // Initialize calibration AFTER configuring the ADC channel
    ESP_RETURN_ON_ERROR(
        thermistor_calibration_init(),
        TAG,
        "ADC calibration initialization failed"
    );

    return ESP_OK;
}

// =====================================================
// THERMISTOR MEASUREMENT
// =====================================================

static esp_err_t thermistor_read(
    int *raw_average,
    int *voltage_average_mv,
    float *resistance,
    float *temperature_c
)
{
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
            "ADC voltage calibration failed"
        );

        raw_sum += raw;
        voltage_sum += voltage_mv;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *raw_average =
        (int)(raw_sum / SAMPLE_COUNT);

    *voltage_average_mv =
        (int)(voltage_sum / SAMPLE_COUNT);

    // Check for invalid ADC voltage
    if (*voltage_average_mv <= 2 ||
        *voltage_average_mv >= (int)VCC_MV - 2) {

        return ESP_ERR_INVALID_RESPONSE;
    }

    /*
     * Divider formula:
     *
     * VCC -> R_FIXED -> ADC -> R_NTC -> GND
     *
     * R_NTC = R_FIXED * V_ADC / (VCC - V_ADC)
     */

    *resistance = R_FIXED *
        (
            (float)*voltage_average_mv /
            (VCC_MV - (float)*voltage_average_mv)
        );

    // Beta equation
    float temperature_k = 1.0f /
        (
            (1.0f / T_NOMINAL_K) +
            (
                logf(*resistance / R_NOMINAL) /
                BETA
            )
        );

    *temperature_c = temperature_k - 273.15f;

    return ESP_OK;
}

// =====================================================
// APPLICATION ENTRY
// =====================================================

void app_main(void)
{
    print_system_info();

    printf("\n");
    printf("Initializing thermistor...\n");

    ESP_ERROR_CHECK(thermistor_init());

    printf("ADC initialized: GPIO50 / ADC2_CHANNEL_1\n");
    printf("NTC: 10k, Beta 3950\n");
    printf("Fixed resistor: 10k\n");
    printf("Supply voltage reference: %.0f mV\n", VCC_MV);

    printf("\n");
    printf("Starting continuous measurement...\n");
    printf("----------------------------------------\n");

    while (1) {

        int raw = 0;
        int voltage_mv = 0;

        float resistance = 0.0f;
        float temperature = 0.0f;

        esp_err_t err = thermistor_read(
            &raw,
            &voltage_mv,
            &resistance,
            &temperature
        );

        if (err == ESP_OK) {

            printf(
                "GPIO50 | Raw: %4d | Voltage: %4d mV | "
                "NTC: %8.0f Ohm | Temp: %.2f C\n",
                raw,
                voltage_mv,
                resistance,
                temperature
            );

        } else if (err == ESP_ERR_INVALID_RESPONSE) {

            printf(
                "GPIO50 | Raw: %4d | Voltage: %4d mV | "
                "Invalid reading - check wiring\n",
                raw,
                voltage_mv
            );

        } else {

            printf(
                "Thermistor error: %s\n",
                esp_err_to_name(err)
            );
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
