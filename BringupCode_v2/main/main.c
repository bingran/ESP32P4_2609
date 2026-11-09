
#include <stdio.h>
#include <inttypes.h>

#include "sdkconfig.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "thermistor.h"
#include "sd_card.h"
#include "imu.h"
#include "audio.h"
#include "camera.h"
#include "wifi_test.h"


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

static void print_memory_info(void)
{
    printf("\n========================================\n");
    printf("          MEMORY DIAGNOSTIC\n");
    printf("========================================\n");

    printf("Total heap:       %u bytes\n",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_DEFAULT));

    printf("Free heap:        %u bytes\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    printf("Total PSRAM:      %u bytes\n",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));

    printf("Free PSRAM:       %u bytes\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    printf("Total DMA memory: %u bytes\n",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_DMA));

    printf("Free DMA memory:  %u bytes\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));

    printf("Largest DMA:      %u bytes\n",
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

    printf("========================================\n\n");
}

// =====================================================
// CONTINUOUS SENSOR MONITORING TASK
// =====================================================

static void sensor_monitor_task(void *pvParameters)
{
    while (1) {

        // ---------------------------------------------
        // THERMISTOR
        // ---------------------------------------------

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

        // ---------------------------------------------
        // IMU
        // ---------------------------------------------

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
    print_memory_info();
#if CONFIG_BRINGUP_WIFI_TEST_ONLY
    ESP_LOGI(TAG, "Wi-Fi test mode: skipping SD, camera, audio and sensors");
    esp_err_t wifi_ret = wifi_test_run();
    ESP_LOGI(TAG, "Wi-Fi diagnostic: %s", esp_err_to_name(wifi_ret));
    return;
#endif
    esp_err_t ret;

    // -----------------------------------------------
    // THERMISTOR
    // -----------------------------------------------

    printf("\nInitializing thermistor...\n");

    ret = thermistor_init();

    if (ret == ESP_OK) {
        printf("Thermistor: PASS\n");
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
    // CAMERA INITIALIZATION
    // -----------------------------------------------

    printf("\nInitializing SC2336 MIPI CSI camera...\n");

    esp_err_t camera_ret = ESP_ERR_INVALID_STATE;

    if (audio_ret == ESP_OK) {

        /*
         * Camera shares the I2C master bus initialized by audio_init().
         * Therefore audio_init() must run first.
         */
        camera_ret = camera_init();

        if (camera_ret == ESP_OK) {
            ESP_LOGI(TAG, "Starting SC2336 frame capture test...");

            esp_err_t capture_ret = camera_capture_test();

            if (capture_ret == ESP_OK) {
                ESP_LOGI(TAG, "SC2336 frame capture PASS");
            } else {
                ESP_LOGE(TAG, "SC2336 frame capture FAIL: %s",
                        esp_err_to_name(capture_ret));
            }
        }

    } else {
        ESP_LOGW(TAG,
                 "Skipping camera test: shared audio I2C bus unavailable");
    }

    // -----------------------------------------------
    // AUDIO TESTS
    // -----------------------------------------------

    if (audio_ret == ESP_OK) {

        // -------------------------------------------
        // SPEAKER BEEP TEST
        // -------------------------------------------

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

            // ---------------------------------------
            // BACKGROUND SENSOR MONITORING
            // ---------------------------------------

            ESP_LOGI(TAG, "Starting background sensor monitoring...");

            BaseType_t task_ret = xTaskCreate(
                sensor_monitor_task,
                "sensor_monitor",
                4096,
                NULL,
                5,
                NULL
            );

            if (task_ret == pdPASS) {
                ESP_LOGI(TAG, "Sensor monitoring task started");
            } else {
                ESP_LOGE(TAG, "Failed to create sensor monitoring task");
            }

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
    // BOARD BRING-UP SUMMARY
    // -----------------------------------------------

    printf("\n");
    printf("========================================\n");
    printf("       BOARD BRING-UP SUMMARY\n");
    printf("========================================\n");

    printf("Thermistor : %s\n",
           "See initialization log");

    printf("IMU        : %s\n",
           (imu_ret == ESP_OK) ? "PASS" : "FAIL");

    printf("SD Card    : %s\n",
           (sd_ret == ESP_OK) ? "PASS" : "FAIL");

    printf("Audio      : %s\n",
           (audio_ret == ESP_OK) ? "PASS" : "FAIL");

    printf("Camera     : %s\n",
           (camera_ret == ESP_OK) ? "PASS" : "FAIL");

    printf("========================================\n");

    // -----------------------------------------------
    // IDLE
    // -----------------------------------------------

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
