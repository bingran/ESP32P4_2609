
#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Initialize the SC2336 MIPI CSI camera. */
esp_err_t camera_init(void);

/* Capture one 1920x1080 frame and save /sdcard/CAM1080.bin and CAM1080.jpg. */
esp_err_t camera_capture_test(void);

/* Deinitialize ESP-Video without deleting the shared audio I2C bus. */
esp_err_t camera_deinit(void);

/* Check whether camera initialization succeeded. */
bool camera_is_initialized(void);

/* Single caller only. Runs until the sink returns an error or capture fails.
 * JPEG data is valid only during the callback; the sink must send/copy it. */
typedef esp_err_t (*camera_jpeg_sink_t)(const uint8_t *jpeg, size_t size, void *ctx);
esp_err_t camera_stream_jpeg(camera_jpeg_sink_t sink, void *ctx);

#endif
