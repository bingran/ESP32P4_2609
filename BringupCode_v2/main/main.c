
/*
 * ESP32-P4C5 Hardware Diagnostic
 * ESP-IDF v5.5.5
 *
 * Flash: 16MB BOYA
 * Thermistor: 10k NTC, Beta 3950
 * ADC: GPIO50 / ADC2_CHANNEL_1
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

#define TAG "HW_TEST"

// Thermistor configuration
#define THERM_ADC_UNIT       ADC_UNIT_2
#define THERM_ADC_CHANNEL    ADC_CHANNEL_1

#define R_FIXED              10000.0f
#define R_NOMINAL            10000.0f
#define BETA                 3950.0f
#define T_NOMINAL_K          298.15f

#define ADC_MAX_VALUE        4095.0f
#define SAMPLE_COUNT         32

static adc_oneshot_unit_handle_t adc_handle;


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

    printf("CPU frequency     : %d MHz\n",
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
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = THERM_ADC_UNIT,
    };

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&init_config, &adc_handle),
        TAG, "ADC initialization failed"
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
        TAG, "ADC channel configuration failed"
    );

    return ESP_OK;
}


// =====================================================
// THERMISTOR MEASUREMENT
// =====================================================

static esp_err_t thermistor_read(
    int *raw_average,
    float *resistance,
    float *temperature_c)
{
    int raw;
    int64_t sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {

        ESP_RETURN_ON_ERROR(
            adc_oneshot_read(
                adc_handle,
                THERM_ADC_CHANNEL,
                &raw
            ),
            TAG, "ADC read failed"
        );

        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *raw_average = (int)(sum / SAMPLE_COUNT);

    if (*raw_average <= 2 || *raw_average >= 4093) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // NTC connected between 3.3V and ADC.
    // Fixed resistor connected between ADC and GND.

    *resistance = R_FIXED *
        ((ADC_MAX_VALUE / (float)(*raw_average)) - 1.0f);

    float temperature_k = 1.0f /
        ((1.0f / T_NOMINAL_K) +
         (logf(*resistance / R_NOMINAL) / BETA));

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

    printf("\n");
    printf("Starting continuous measurement...\n");
    printf("----------------------------------------\n");

    while (1) {

        int raw = 0;
        float resistance = 0;
        float temperature = 0;

        esp_err_t err = thermistor_read(
            &raw,
            &resistance,
            &temperature
        );

        if (err == ESP_OK) {

            printf(
                "GPIO50 | ADC: %4d | NTC: %8.0f Ohm | Temp: %.2f C\n",
                raw,
                resistance,
                temperature
            );

        } else if (err == ESP_ERR_INVALID_RESPONSE) {

            printf(
                "GPIO50 | ADC: %4d | Invalid reading - check wiring\n",
                raw
            );

        } else {

            printf("Thermistor error: %s\n",
                   esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
