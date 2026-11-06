
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "sd_card.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define TAG                     "SD_CARD"

#define SD_DET_GPIO             GPIO_NUM_45

#define SD_CMD_GPIO             GPIO_NUM_44
#define SD_CLK_GPIO             GPIO_NUM_43

#define SD_D0_GPIO              GPIO_NUM_39
#define SD_D1_GPIO              GPIO_NUM_40
#define SD_D2_GPIO              GPIO_NUM_41
#define SD_D3_GPIO              GPIO_NUM_42

#define SD_MOUNT_POINT          "/sdcard"
#define SD_TEST_FILE SD_MOUNT_POINT "/P4TEST.TXT"

#define SD_MAX_FILES            5
#define SD_ALLOC_UNIT           (16 * 1024)

// Your current board configuration:
// GPIO45 HIGH = card inserted
#define SD_CARD_INSERTED_LEVEL  1


// =====================================================
// CARD DETECTION
// =====================================================

static esp_err_t sd_card_detect_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << SD_DET_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&config);
}

static bool sd_card_is_inserted(void)
{
    return gpio_get_level(SD_DET_GPIO) ==
           SD_CARD_INSERTED_LEVEL;
}

// =====================================================
// FILE SYSTEM DIAGNOSTICS
// =====================================================

static void sd_log_errno(const char *operation)
{
    int error_number = errno;

    ESP_LOGE(TAG, "%s failed", operation);
    ESP_LOGE(TAG, "errno = %d", error_number);
    ESP_LOGE(TAG, "Reason: %s", strerror(error_number));
}

static esp_err_t sd_check_mount_point(void)
{
    struct stat st;

    if (stat(SD_MOUNT_POINT, &st) != 0) {
        sd_log_errno("Mount point stat");
        return ESP_FAIL;
    }

    if (!S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "%s is not a directory", SD_MOUNT_POINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Mount point exists: %s", SD_MOUNT_POINT);

    return ESP_OK;
}

// =====================================================
// FILE WRITE / READ TEST
// =====================================================

static esp_err_t sd_card_file_test(void)
{
    const char *test_data =
        "ESP32-P4 SD CARD TEST\r\n"
        "SDMMC 4-bit mode\r\n"
        "Read and write verification PASS\r\n";

    char read_buffer[160] = {0};

    size_t expected_size = strlen(test_data);
    size_t written = 0;
    size_t bytes_read = 0;

    FILE *file = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "Starting file read/write test");

    // Check mount point
    if (sd_check_mount_point() != ESP_OK) {
        return ESP_FAIL;
    }

    // -------------------------------------------------
    // WRITE TEST
    // -------------------------------------------------

    ESP_LOGI(TAG, "Opening file for writing: %s", SD_TEST_FILE);

    errno = 0;
    file = fopen(SD_TEST_FILE, "w");

    if (file == NULL) {
        sd_log_errno("File open for writing");
        return ESP_FAIL;
    }

    written = fwrite(
        test_data,
        1,
        expected_size,
        file
    );

    if (written != expected_size) {
        sd_log_errno("File write");
        fclose(file);
        return ESP_FAIL;
    }

    // Force buffered data to filesystem
    if (fflush(file) != 0) {
        sd_log_errno("File flush");
        fclose(file);
        return ESP_FAIL;
    }

    if (fclose(file) != 0) {
        sd_log_errno("File close after writing");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File write PASS: %u bytes",
             (unsigned)written);

    // -------------------------------------------------
    // READ TEST
    // -------------------------------------------------

    ESP_LOGI(TAG, "Opening file for reading");

    errno = 0;
    file = fopen(SD_TEST_FILE, "r");

    if (file == NULL) {
        sd_log_errno("File open for reading");
        return ESP_FAIL;
    }

    bytes_read = fread(
        read_buffer,
        1,
        sizeof(read_buffer) - 1,
        file
    );

    if (ferror(file)) {
        sd_log_errno("File read");
        fclose(file);
        return ESP_FAIL;
    }

    if (fclose(file) != 0) {
        sd_log_errno("File close after reading");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File read: %u bytes",
             (unsigned)bytes_read);

    // -------------------------------------------------
    // DATA VERIFICATION
    // -------------------------------------------------

    if (bytes_read != expected_size) {
        ESP_LOGE(TAG,
                 "Size mismatch: expected=%u actual=%u",
                 (unsigned)expected_size,
                 (unsigned)bytes_read);

        return ESP_FAIL;
    }

    if (memcmp(read_buffer, test_data, expected_size) != 0) {
        ESP_LOGE(TAG, "Read-back data mismatch");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File read PASS");
    ESP_LOGI(TAG, "Data verification PASS");

    ret = ESP_OK;

    return ret;
}

// =====================================================
// SD CARD TEST
// =====================================================

esp_err_t sd_card_test(void)
{
    esp_err_t ret;
    sdmmc_card_t *card = NULL;

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "       SD CARD DIAGNOSTIC");
    ESP_LOGI(TAG, "================================");

    // -------------------------------------------------
    // CARD DETECTION
    // -------------------------------------------------

    ret = sd_card_detect_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Card detect GPIO init failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(30));

    int detect_level = gpio_get_level(SD_DET_GPIO);

    ESP_LOGI(TAG, "SD_DET GPIO45 level: %d", detect_level);

    if (!sd_card_is_inserted()) {
        ESP_LOGW(TAG, "No SD card detected");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "SD card detected");

    // -------------------------------------------------
    // SDMMC HOST CONFIGURATION
    // -------------------------------------------------

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config =
        SDMMC_SLOT_CONFIG_DEFAULT();

    slot_config.width = 4;

    slot_config.clk = SD_CLK_GPIO;
    slot_config.cmd = SD_CMD_GPIO;

    slot_config.d0 = SD_D0_GPIO;
    slot_config.d1 = SD_D1_GPIO;
    slot_config.d2 = SD_D2_GPIO;
    slot_config.d3 = SD_D3_GPIO;

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Initializing SDMMC 4-bit interface");
    ESP_LOGI(TAG, "CLK=43 CMD=44 D0=39 D1=40 D2=41 D3=42");

    // -------------------------------------------------
    // FAT FILESYSTEM MOUNT
    // -------------------------------------------------

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = SD_ALLOC_UNIT,
    };

    ret = esp_vfs_fat_sdmmc_mount(
        SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &card
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");

    // -------------------------------------------------
    // CARD INFORMATION
    // -------------------------------------------------

    sdmmc_card_print_info(stdout, card);

    // -------------------------------------------------
    // FILESYSTEM TEST
    // -------------------------------------------------

    ret = sd_card_file_test();

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "================================");
        ESP_LOGI(TAG, "       SD CARD TEST: PASS");
        ESP_LOGI(TAG, "================================");
    } else {
        ESP_LOGE(TAG, "SD CARD TEST: FAIL");
    }

    // -------------------------------------------------
    // UNMOUNT
    // -------------------------------------------------

    esp_err_t unmount_ret =
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);

    if (unmount_ret != ESP_OK) {
        ESP_LOGE(TAG, "SD unmount failed: %s",
                 esp_err_to_name(unmount_ret));

        if (ret == ESP_OK) {
            ret = unmount_ret;
        }
    } else {
        ESP_LOGI(TAG, "SD card unmounted successfully");
    }

    return ret;
}
