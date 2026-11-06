
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "thermistor.h"
#include "sd_card.h"

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
// APPLICATION ENTRY
// =====================================================

void app_main(void)
{
    print_system_info();

    // -----------------------------------------------
    // Thermistor initialization
    // -----------------------------------------------

    printf("\n");
    printf("Initializing thermistor...\n");

    ESP_ERROR_CHECK(thermistor_init());

    printf("Thermistor initialized successfully.\n");

    // -----------------------------------------------
    // SD card test
    // -----------------------------------------------

    printf("\n");
    printf("Starting SD card test...\n");

    esp_err_t sd_ret = sd_card_test();

    if (sd_ret == ESP_OK) {
        printf("\nSD CARD: PASS\n");
    } else {
        printf("\nSD CARD: FAIL (%s)\n",
               esp_err_to_name(sd_ret));
    }

    // -----------------------------------------------
    // Continuous thermistor measurement
    // -----------------------------------------------

    printf("\n");
    printf("Starting continuous thermistor measurement...\n");
    printf("----------------------------------------\n");

    while (1) {

        thermistor_reading_t reading = {0};

        esp_err_t ret = thermistor_read(&reading);

        if (ret == ESP_OK) {

            printf(
                "GPIO50 | Raw: %4d | Voltage: %4d mV | "
                "NTC: %8.0f Ohm | Temp: %.2f C\n",
                reading.raw,
                reading.voltage_mv,
                reading.resistance_ohm,
                reading.temperature_c
            );

        } else {

            printf("Thermistor error: %s\n",
                   esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
