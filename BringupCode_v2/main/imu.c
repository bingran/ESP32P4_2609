#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "imu.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define TAG                 "IMU"

#define IMU_I2C_PORT        I2C_NUM_1
#define IMU_SDA_GPIO        GPIO_NUM_20
#define IMU_SCL_GPIO        GPIO_NUM_21

#define IMU_INT2_GPIO       GPIO_NUM_22
#define IMU_INT1_GPIO       GPIO_NUM_23

#define IMU_I2C_FREQ_HZ     100000

#define IMU_ADDR_LOW        0x6A
#define IMU_ADDR_HIGH       0x6B

#define IMU_WHO_AM_I_REG    0x0F
#define IMU_WHO_AM_I_VALUE  0x6B

// ISM330DHCX registers
#define INT1_CTRL           0x0D
#define INT2_CTRL           0x0E

#define CTRL1_XL            0x10
#define CTRL2_G             0x11
#define CTRL3_C             0x12

#define STATUS_REG          0x1E

#define OUTX_L_G            0x22

// Interrupt routing
#define INT1_DRDY_XL        0x01
#define INT2_DRDY_G         0x02

// =====================================================
// GLOBAL HANDLES
// =====================================================

static i2c_master_bus_handle_t imu_bus = NULL;
static i2c_master_dev_handle_t imu_device = NULL;

// Interrupt counters
static volatile uint32_t int1_interrupt_count = 0;
static volatile uint32_t int2_interrupt_count = 0;

// =====================================================
// INTERRUPT HANDLERS
// =====================================================

static void IRAM_ATTR imu_int1_isr_handler(void *arg)
{
    (void)arg;
    int1_interrupt_count++;
}

static void IRAM_ATTR imu_int2_isr_handler(void *arg)
{
    (void)arg;
    int2_interrupt_count++;
}

// =====================================================
// I2C REGISTER ACCESS
// =====================================================

static esp_err_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        imu_device,
        data,
        sizeof(data),
        1000
    );
}

static esp_err_t imu_read_regs(
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    return i2c_master_transmit_receive(
        imu_device,
        &reg,
        1,
        data,
        length,
        1000
    );
}

static esp_err_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    return imu_read_regs(reg, value, 1);
}

// =====================================================
// I2C INITIALIZATION
// =====================================================

static esp_err_t imu_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(
        &bus_config,
        &imu_bus
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized: SDA=%d SCL=%d",
             IMU_SDA_GPIO, IMU_SCL_GPIO);

    return ESP_OK;
}

// =====================================================
// IMU DEVICE DETECTION
// =====================================================

static esp_err_t imu_detect(void)
{
    const uint8_t addresses[] = {
        IMU_ADDR_LOW,
        IMU_ADDR_HIGH
    };

    for (size_t i = 0;
         i < sizeof(addresses) / sizeof(addresses[0]);
         i++) {

        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addresses[i],
            .scl_speed_hz = IMU_I2C_FREQ_HZ,
        };

        imu_device = NULL;

        esp_err_t ret = i2c_master_bus_add_device(
            imu_bus,
            &device_config,
            &imu_device
        );

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cannot add I2C device 0x%02X",
                     addresses[i]);
            continue;
        }

        uint8_t who_am_i = 0;

        ret = imu_read_reg(IMU_WHO_AM_I_REG, &who_am_i);

        if (ret == ESP_OK &&
            who_am_i == IMU_WHO_AM_I_VALUE) {

            ESP_LOGI(TAG, "ISM330DHCX detected!");
            ESP_LOGI(TAG, "I2C address: 0x%02X", addresses[i]);
            ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);

            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "Address 0x%02X: WHO_AM_I=0x%02X",
                 addresses[i],
                 who_am_i);

        i2c_master_bus_rm_device(imu_device);
        imu_device = NULL;
    }

    ESP_LOGE(TAG, "ISM330DHCX not detected");

    return ESP_ERR_NOT_FOUND;
}

// =====================================================
// INTERRUPT GPIO INITIALIZATION
// =====================================================

static esp_err_t imu_interrupt_gpio_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask =
            (1ULL << IMU_INT1_GPIO) |
            (1ULL << IMU_INT2_GPIO),

        .mode = GPIO_MODE_INPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_POSEDGE,
    };

    esp_err_t ret = gpio_config(&config);

    if (ret != ESP_OK) {
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);

    // ISR service may already have been installed elsewhere
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service installation failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Register INT1 handler
    ret = gpio_isr_handler_add(
        IMU_INT1_GPIO,
        imu_int1_isr_handler,
        NULL
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT1 ISR registration failed");
        return ret;
    }

    // Register INT2 handler
    ret = gpio_isr_handler_add(
        IMU_INT2_GPIO,
        imu_int2_isr_handler,
        NULL
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT2 ISR registration failed");
        gpio_isr_handler_remove(IMU_INT1_GPIO);
        return ret;
    }

    ESP_LOGI(TAG, "INT1 GPIO23 configured with rising-edge ISR");
    ESP_LOGI(TAG, "INT2 GPIO22 configured with rising-edge ISR");

    return ESP_OK;
}

// =====================================================
// SENSOR CONFIGURATION
// =====================================================

static esp_err_t imu_configure(void)
{
    esp_err_t ret;

    // Block data update + automatic register increment
    ret = imu_write_reg(CTRL3_C, 0x44);

    if (ret != ESP_OK) {
        return ret;
    }

    // Accelerometer:
    // ODR = 104 Hz
    // Full scale = +/- 2g
    ret = imu_write_reg(CTRL1_XL, 0x40);

    if (ret != ESP_OK) {
        return ret;
    }

    // Gyroscope:
    // ODR = 104 Hz
    // Full scale = +/- 250 dps
    ret = imu_write_reg(CTRL2_G, 0x40);

    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Route accelerometer data-ready to INT1
    ret = imu_write_reg(INT1_CTRL, INT1_DRDY_XL);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT1 routing configuration failed");
        return ret;
    }

    // Route gyroscope data-ready to INT2
    ret = imu_write_reg(INT2_CTRL, INT2_DRDY_G);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT2 routing configuration failed");
        return ret;
    }

    ESP_LOGI(TAG, "Accelerometer configured: 104 Hz, +/-2g");
    ESP_LOGI(TAG, "Gyroscope configured: 104 Hz, +/-250 dps");

    ESP_LOGI(TAG, "INT1 routing: Accelerometer data ready");
    ESP_LOGI(TAG, "INT2 routing: Gyroscope data ready");

    return ESP_OK;
}

// =====================================================
// IMU INITIALIZATION
// =====================================================

esp_err_t imu_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "       IMU DIAGNOSTIC");
    ESP_LOGI(TAG, "================================");

    int1_interrupt_count = 0;
    int2_interrupt_count = 0;

    ret = imu_i2c_init();

    if (ret != ESP_OK) {
        return ret;
    }

    ret = imu_detect();

    if (ret != ESP_OK) {
        return ret;
    }

    // Configure ESP32 interrupt GPIOs first
    ret = imu_interrupt_gpio_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Interrupt GPIO initialization failed");
        return ret;
    }

    // Configure sensor and route its interrupt signals
    ret = imu_configure();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor configuration failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "INT1 GPIO23 initial level: %d",
             gpio_get_level(IMU_INT1_GPIO));

    ESP_LOGI(TAG, "INT2 GPIO22 initial level: %d",
             gpio_get_level(IMU_INT2_GPIO));

    ESP_LOGI(TAG, "IMU initialization PASS");

    return ESP_OK;
}

// =====================================================
// INTERRUPT COUNTER ACCESS
// =====================================================

void imu_get_interrupt_counts(
    uint32_t *int1_count,
    uint32_t *int2_count)
{
    if (int1_count != NULL) {
        *int1_count = int1_interrupt_count;
    }

    if (int2_count != NULL) {
        *int2_count = int2_interrupt_count;
    }
}

// =====================================================
// IMU DATA READ
// =====================================================

static int16_t imu_convert_raw(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

esp_err_t imu_read(imu_data_t *data)
{
    if (data == NULL || imu_device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[12];

    esp_err_t ret = imu_read_regs(
        OUTX_L_G,
        buffer,
        sizeof(buffer)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    // Gyroscope: registers 0x22 - 0x27
    data->gyro_x = imu_convert_raw(buffer[0], buffer[1]);
    data->gyro_y = imu_convert_raw(buffer[2], buffer[3]);
    data->gyro_z = imu_convert_raw(buffer[4], buffer[5]);

    // Accelerometer: registers 0x28 - 0x2D
    data->accel_x = imu_convert_raw(buffer[6], buffer[7]);
    data->accel_y = imu_convert_raw(buffer[8], buffer[9]);
    data->accel_z = imu_convert_raw(buffer[10], buffer[11]);

    // Sensitivity:
    // +/-250 dps = 8.75 mdps/LSB
    // +/-2g = 0.061 mg/LSB

    data->gyro_x_dps = data->gyro_x * 0.00875f;
    data->gyro_y_dps = data->gyro_y * 0.00875f;
    data->gyro_z_dps = data->gyro_z * 0.00875f;

    data->accel_x_g = data->accel_x * 0.000061f;
    data->accel_y_g = data->accel_y * 0.000061f;
    data->accel_z_g = data->accel_z * 0.000061f;

    return ESP_OK;
}