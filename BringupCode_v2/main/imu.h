#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} imu_data_t;

esp_err_t imu_init(void);

esp_err_t imu_read(imu_data_t *data);

// Get interrupt event counters
void imu_get_interrupt_counts(
    uint32_t *int1_count,
    uint32_t *int2_count
);