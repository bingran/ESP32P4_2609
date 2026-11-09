
#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include "esp_err.h"

/* Initialize the SC2336 MIPI CSI camera. */
esp_err_t camera_init(void);

/* Capture one frame and save it to /sdcard/SC2336_TEST.bin. */
esp_err_t camera_capture_test(void);

/* Deinitialize ESP-Video without deleting the shared audio I2C bus. */
esp_err_t camera_deinit(void);

/* Check whether camera initialization succeeded. */
bool camera_is_initialized(void);

#endif
