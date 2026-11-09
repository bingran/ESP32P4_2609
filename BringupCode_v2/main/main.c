
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
#include "camera_web.h"
#include "gps.h"
#include "fingerprint.h"


static const char *TAG = "MAIN";
static bool s_thermistor_ready;
static bool s_imu_ready;
static volatile bool s_web_start_finished;
static esp_err_t s_web_result = ESP_ERR_INVALID_STATE;

static void camera_web_task(void *arg)
{
    (void)arg;
    s_web_result = camera_web_start();
    s_web_start_finished = true;
    ESP_LOGI(TAG, "Router camera startup: %s", esp_err_to_name(s_web_result));
    vTaskDelete(NULL);
}

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
    unsigned log_cycles = 0;
    while (1) {
        const bool log_sensor = log_cycles < 5;

        // ---------------------------------------------
        // THERMISTOR
        // ---------------------------------------------

        thermistor_reading_t reading = {0};

        esp_err_t ret = s_thermistor_ready ? thermistor_read(&reading) : ESP_ERR_INVALID_STATE;

        if (log_sensor && ret == ESP_OK) {
            ESP_LOGI("THERMISTOR",
                "GPIO50 | Raw: %4d | Voltage: %4d mV | "
                "NTC: %8.0f Ohm | Temp: %.2f C",
                reading.raw,
                reading.voltage_mv,
                reading.resistance_ohm,
                reading.temperature_c
            );
        } else if (log_sensor && s_thermistor_ready) {
            ESP_LOGE("THERMISTOR", "Read failed: %s",
                     esp_err_to_name(ret));
        }

        // ---------------------------------------------
        // IMU
        // ---------------------------------------------

        imu_data_t imu = {0};

        ret = s_imu_ready ? imu_read(&imu) : ESP_ERR_INVALID_STATE;

        if (log_sensor && ret == ESP_OK) {
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

        if (s_imu_ready) imu_get_interrupt_counts(&int1_count, &int2_count);

        if (log_sensor && s_imu_ready) ESP_LOGI("IMU",
            "INTERRUPTS | INT1: %" PRIu32 " | INT2: %" PRIu32,
            int1_count,
            int2_count
        );

        if (log_sensor && ++log_cycles == 5) {
            ESP_LOGI(TAG, "ADC and IMU periodic logs stopped after 5 cycles; sensor polling continues");
        }
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
    esp_err_t ret;

    // -----------------------------------------------
    // THERMISTOR
    // -----------------------------------------------

    printf("\nInitializing thermistor...\n");

    ret = thermistor_init();
    s_thermistor_ready = (ret == ESP_OK);

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
    s_imu_ready = (imu_ret == ESP_OK);

    if (imu_ret == ESP_OK) {
        printf("IMU: PASS\n");
    } else {
        printf("IMU: FAIL (%s)\n", esp_err_to_name(imu_ret));
    }

    // -----------------------------------------------
    // GPS uses its own UART and background receiver alongside all modules.
    esp_err_t gps_ret = gps_init();
    if (gps_ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS UART/task initialization failed: %s", esp_err_to_name(gps_ret));
    }

    esp_err_t fp_ret = fingerprint_init();
    if (fp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Fingerprint UART/task initialization failed: %s", esp_err_to_name(fp_ret));
    }

    // Sensor monitoring runs independently of SD, audio and Wi-Fi.
    if (s_thermistor_ready || s_imu_ready) {
        if (xTaskCreate(sensor_monitor_task, "sensor_monitor", 4096, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create sensor monitoring task");
        }
    }

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

        if (camera_ret == ESP_OK && sd_ret == ESP_OK) {
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
    // Start networking independently; audio tests below continue while the
    // router connects. The one-shot capture above has released the camera.
    if (xTaskCreate(camera_web_task, "camera_web_start", 8192, NULL, 5, NULL) != pdPASS) {
        s_web_result = ESP_ERR_NO_MEM;
        s_web_start_finished = true;
        ESP_LOGE(TAG, "Failed to create router camera task");
    }

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
    printf("GPS        : %s\n", gps_ret == ESP_OK ? "UART/TASK READY (see GPS fix log)" : "INIT FAIL");
    printf("R503       : %s\n", fp_ret == ESP_OK ? "UART/TASK READY (see R503 log)" : "INIT FAIL");
    printf("Router/web : %s\n", !s_web_start_finished ? "STARTING (see later log)" :
           (s_web_result == ESP_OK ? "PASS" : "FAIL (see log)"));

    printf("========================================\n");

    // -----------------------------------------------
    // IDLE
    // -----------------------------------------------

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
