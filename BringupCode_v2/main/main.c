
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "thermistor.h"
#include "sd_card.h"
#include "imu.h"
#include "audio.h"

static const char *TAG = "MAIN";

uint32_t int1_count = 0;
uint32_t int2_count = 0;

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
// CONTINUOUS SENSOR MONITORING TASK
// =====================================================

static void sensor_monitor_task(void *pvParameters)
{
    while (1) {

        // Thermistor
        thermistor_reading_t reading = {0};

        esp_err_t ret = thermistor_read(&reading);

        if (ret == ESP_OK) {
            ESP_LOGI("THERMISTOR",
                "GPIO50 | Raw: %4d | Voltage: %4d mV | "
                "NTC: %8.0f Ohm | Temp: %.2f C",
                reading.raw,
                reading.voltage_mv,
                reading.resistance_ohm,
                reading.temperature_c
            );
        } else {
            ESP_LOGE("THERMISTOR", "Read failed: %s",
                     esp_err_to_name(ret));
        }

        // IMU
        imu_data_t imu = {0};

        ret = imu_read(&imu);

        if (ret == ESP_OK) {
            ESP_LOGI("IMU",
                "ACC[g] X=%.3f Y=%.3f Z=%.3f | "
                "GYRO[dps] X=%.2f Y=%.2f Z=%.2f",
                imu.accel_x_g,
                imu.accel_y_g,
                imu.accel_z_g,
                imu.gyro_x_dps,
                imu.gyro_y_dps,
                imu.gyro_z_dps
            );
        }

        imu_get_interrupt_counts(&int1_count, &int2_count);

        ESP_LOGI("IMU",
            "INTERRUPTS | INT1: %" PRIu32 " | INT2: %" PRIu32,
            int1_count,
            int2_count
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =====================================================
// APPLICATION ENTRY
// =====================================================

void app_main(void)
{
    print_system_info();

    esp_err_t ret;

    // -----------------------------------------------
    // THERMISTOR
    // -----------------------------------------------

    printf("\nInitializing thermistor...\n");

    ret = thermistor_init();

    if (ret == ESP_OK) {
        printf("Thermistor initialized successfully.\n");
    } else {
        ESP_LOGE(TAG, "Thermistor initialization failed: %s",
                 esp_err_to_name(ret));
    }

    // -----------------------------------------------
    // IMU
    // -----------------------------------------------

    printf("\nInitializing IMU...\n");

    esp_err_t imu_ret = imu_init();

    if (imu_ret == ESP_OK) {
        printf("IMU: PASS\n");
    } else {
        printf("IMU: FAIL (%s)\n", esp_err_to_name(imu_ret));
    }

    // -----------------------------------------------
    // SD CARD
    // -----------------------------------------------

    printf("\nStarting SD card test...\n");

    esp_err_t sd_ret = sd_card_test();

    if (sd_ret == ESP_OK) {
        printf("SD CARD: PASS\n");
    } else {
        printf("SD CARD: FAIL (%s)\n",
               esp_err_to_name(sd_ret));
    }

    // -----------------------------------------------
    // AUDIO INITIALIZATION
    // -----------------------------------------------

    printf("\nInitializing audio...\n");

    esp_err_t audio_ret = audio_init();

    if (audio_ret == ESP_OK) {
        ESP_LOGI(TAG, "Audio initialization PASS");
    } else {
        ESP_LOGE(TAG, "Audio initialization failed: %s",
                 esp_err_to_name(audio_ret));
    }

    // -----------------------------------------------
    // AUDIO TESTS
    // -----------------------------------------------

    if (audio_ret == ESP_OK) {

        // Speaker beep test
        printf("\nStarting speaker test...\n");

        ret = audio_play_test_tone(1000, 1000);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Speaker test PASS");
        } else {
            ESP_LOGE(TAG, "Speaker test failed: %s",
                     esp_err_to_name(ret));
        }

        // -------------------------------------------
        // MICROPHONE RECORDING TEST
        // -------------------------------------------

        if (sd_ret == ESP_OK) {

            ESP_LOGI(TAG, "Recording microphone for 5 seconds");

            ret = audio_record_wav_file(
                "/sdcard/MIC_TEST.WAV",
                5000
            );

            if (ret == ESP_OK) {

                ESP_LOGI(TAG, "Microphone recording PASS");

                vTaskDelay(pdMS_TO_TICKS(1000));

                ESP_LOGI(TAG, "Playing recorded microphone WAV");

                ret = audio_play_wav_file(
                    "/sdcard/MIC_TEST.WAV"
                );

                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Microphone playback PASS");
                } else {
                    ESP_LOGE(TAG, "Microphone playback failed: %s",
                             esp_err_to_name(ret));
                }

            } else {
                ESP_LOGE(TAG, "Microphone recording failed: %s",
                         esp_err_to_name(ret));
            }

            // Start sensor monitoring in background
            ESP_LOGI(TAG, "Starting background sensor monitoring...");

            xTaskCreate(
                sensor_monitor_task,
                "sensor_monitor",
                4096,
                NULL,
                5,
                NULL
            );

            // ---------------------------------------
            // ORIGINAL MUSIC PLAYBACK
            // ---------------------------------------

            ESP_LOGI(TAG, "Playing original music: TEST.WAV");

            vTaskDelay(pdMS_TO_TICKS(1000));

            ret = audio_play_wav_file("/sdcard/TEST.WAV");

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Original music playback PASS");
            } else {
                ESP_LOGE(TAG, "Original music playback failed: %s",
                         esp_err_to_name(ret));
            }

        } else {
            ESP_LOGW(TAG, "Skipping SD audio tests: SD card unavailable");
        }

    } else {
        ESP_LOGW(TAG, "Skipping audio tests: audio unavailable");
    }

    // -----------------------------------------------
    // CONTINUOUS SENSOR MONITORING
    // -----------------------------------------------

    printf("\n");
    printf("========================================\n");
    printf("       CONTINUOUS SENSOR MONITOR\n");
    printf("========================================\n");

    printf("Thermistor and IMU measurements every second.\n");
    printf("----------------------------------------\n");

    while(1);
}
