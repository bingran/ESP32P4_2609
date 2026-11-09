
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
#define SD_TEST_FILE            SD_MOUNT_POINT "/P4TEST.TXT"

#define SD_MAX_FILES            5
#define SD_ALLOC_UNIT           (16 * 1024)

// GPIO45 HIGH = card inserted
#define SD_CARD_INSERTED_LEVEL  1

// =====================================================
// GLOBAL STATE
// =====================================================

static sdmmc_card_t *sd_card_handle = NULL;
static bool sd_mounted = false;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

static esp_err_t sd_card_detect_init(void);
static bool sd_card_is_inserted(void);
static void sd_log_errno(const char *operation);
static esp_err_t sd_check_mount_point(void);
static esp_err_t sd_card_file_test(void);

// =====================================================
// PUBLIC STATUS API
// =====================================================

bool sd_card_is_mounted(void)
{
    return sd_mounted;
}

// =====================================================
// SD CARD INITIALIZATION / MOUNT
// =====================================================

esp_err_t sd_card_init(void)
{
    if (sd_mounted) {
        ESP_LOGI(TAG, "SD card already mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SD card detection");

    esp_err_t ret = sd_card_detect_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD detect GPIO initialization failed: %s",
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

    // -------------------------------------------------
    // SDMMC HOST CONFIGURATION
    // -------------------------------------------------

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    /* GPIO39-44 are slot 0's fixed pins; slot 1 is reserved for C5 SDIO. */
    host.slot = SDMMC_HOST_SLOT_0;
    /* Diagnose slot-0 write CRC errors at a more conservative bus clock. */
    host.max_freq_khz = 10000;
    host.flags &= ~(SDMMC_HOST_FLAG_8BIT | SDMMC_HOST_FLAG_DDR);
    host.flags |= SDMMC_HOST_FLAG_4BIT;

    // -------------------------------------------------
    // SDMMC SLOT CONFIGURATION
    // -------------------------------------------------

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

    // -------------------------------------------------
    // FATFS MOUNT CONFIGURATION
    // -------------------------------------------------

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = SD_ALLOC_UNIT,
    };

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "       SD CARD INITIALIZATION");
    ESP_LOGI(TAG, "================================");

    ESP_LOGI(TAG, "Mount point: %s", SD_MOUNT_POINT);
    ESP_LOGI(TAG, "SDMMC slot: %d", host.slot);
    ESP_LOGI(TAG, "Bus width: %d-bit", slot_config.width);
    ESP_LOGI(TAG, "Clock: %d kHz", host.max_freq_khz);

    // -------------------------------------------------
    // MOUNT SD CARD
    // -------------------------------------------------

    ESP_LOGI(TAG, "Mounting SD card...");

    ret = esp_vfs_fat_sdmmc_mount(
        SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &sd_card_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s",
                 esp_err_to_name(ret));

        sd_card_handle = NULL;
        sd_mounted = false;

        return ret;
    }

    /* Restore DAT3's pull-up after the temporary GPIO-high initialization.
     * This alone did not resolve the observed 4-bit write CRC errors.
     * Keep peripheral routing and direction unchanged. */
    ret = gpio_pullup_en(SD_D3_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore DAT3 pull-up: %s", esp_err_to_name(ret));
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card_handle);
        sd_card_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "Slot 0 DAT3 GPIO42 pull-up restored after 4-bit initialization");

    /* Controlled comparison with slot 1: the IDF IO_MUX path forces drive
     * strength 3, unlike the GPIO-matrix path. Try default strength 2 on
     * the same wires without changing bus width, clock or pin routing. */
    const gpio_num_t sd_pins[] = {
        SD_CLK_GPIO, SD_CMD_GPIO, SD_D0_GPIO, SD_D1_GPIO, SD_D2_GPIO, SD_D3_GPIO,
    };
    for (size_t i = 0; i < sizeof(sd_pins) / sizeof(sd_pins[0]); ++i) {
        ret = gpio_set_drive_capability(sd_pins[i], GPIO_DRIVE_CAP_2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set SD drive strength: %s", esp_err_to_name(ret));
            esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card_handle);
            sd_card_handle = NULL;
            return ret;
        }
    }
    ESP_LOGI(TAG, "SD drive strength set to 2 for slot-0 comparison");

    sd_mounted = true;

    ESP_LOGI(TAG, "SD card mounted successfully");

    sdmmc_card_print_info(stdout, sd_card_handle);
    ESP_LOGI(TAG, "SD pin configuration after mount (GPIO39-44):");
    gpio_dump_io_configuration(stdout,
        (1ULL << SD_D0_GPIO) | (1ULL << SD_D1_GPIO) |
        (1ULL << SD_D2_GPIO) | (1ULL << SD_D3_GPIO) |
        (1ULL << SD_CLK_GPIO) | (1ULL << SD_CMD_GPIO));

    return ESP_OK;
}

// =====================================================
// SD CARD UNMOUNT
// =====================================================

esp_err_t sd_card_deinit(void)
{
    if (!sd_mounted || sd_card_handle == NULL) {
        ESP_LOGI(TAG, "SD card is not mounted");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Unmounting SD card...");

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(
        SD_MOUNT_POINT,
        sd_card_handle
    );

    if (ret == ESP_OK) {

        sd_card_handle = NULL;
        sd_mounted = false;

        ESP_LOGI(TAG, "SD card unmounted successfully");

    } else {

        ESP_LOGE(TAG, "SD card unmount failed: %s",
                 esp_err_to_name(ret));
    }

    return ret;
}

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
// ERROR DIAGNOSTICS
// =====================================================

static void sd_log_errno(const char *operation)
{
    int error_number = errno;

    ESP_LOGE(TAG, "%s failed", operation);
    ESP_LOGE(TAG, "errno = %d", error_number);
    ESP_LOGE(TAG, "Reason: %s", strerror(error_number));
}

// =====================================================
// MOUNT POINT CHECK
// =====================================================

static esp_err_t sd_check_mount_point(void)
{
    struct stat st;

    if (stat(SD_MOUNT_POINT, &st) != 0) {
        sd_log_errno("Mount point stat");
        return ESP_FAIL;
    }

    if (!S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "%s is not a directory",
                 SD_MOUNT_POINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Mount point exists: %s",
             SD_MOUNT_POINT);

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

    ESP_LOGI(TAG, "Starting file read/write test");

    // -------------------------------------------------
    // CHECK MOUNT POINT
    // -------------------------------------------------

    if (sd_check_mount_point() != ESP_OK) {
        return ESP_FAIL;
    }

    // -------------------------------------------------
    // WRITE TEST
    // -------------------------------------------------

    ESP_LOGI(TAG, "Opening file for writing: %s",
             SD_TEST_FILE);

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

    return ESP_OK;
}

// =====================================================
// SD CARD DIAGNOSTIC
// =====================================================

esp_err_t sd_card_test(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "       SD CARD DIAGNOSTIC");
    ESP_LOGI(TAG, "================================");

    // Mount if not already mounted.
    esp_err_t ret = sd_card_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Run read/write verification.
    ret = sd_card_file_test();

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "================================");
        ESP_LOGI(TAG, "       SD CARD TEST: PASS");
        ESP_LOGI(TAG, "================================");
        ESP_LOGI(TAG, "SD card remains mounted at %s",
                 SD_MOUNT_POINT);
    } else {
        ESP_LOGE(TAG, "SD CARD TEST: FAIL");
    }

    // IMPORTANT:
    // Do NOT unmount here.
    // WAV playback needs /sdcard to remain mounted.

    return ret;
}
