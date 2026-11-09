
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/jpeg_encode.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio.h"
#include "sd_card.h"
#include "camera.h"

#define CAMERA_RESET_GPIO       26
#define CAMERA_CSI_EN_GPIO      27
#define CAMERA_SCCB_FREQ        100000

#define CAMERA_DEVICE_PATH      ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define CAMERA_FRAME_PATH      "/sdcard/CAM1080.bin"
#define CAMERA_JPEG_PATH       "/sdcard/CAM1080.jpg"

#define CAMERA_BUFFER_COUNT     2

#define CAMERA_CAPTURE_WIDTH    1920
#define CAMERA_CAPTURE_HEIGHT   1080

/* Start with YUV422 packed format. */
#define CAMERA_CAPTURE_FORMAT   V4L2_PIX_FMT_UYVY

static const char *TAG = "CAMERA";

static bool s_camera_initialized = false;

/* Keep the camera MMAP buffer separate from JPEG DMA memory. */
static esp_err_t camera_save_jpeg(const uint8_t *frame, size_t frame_size,
                                  const struct v4l2_pix_format *pix)
{
    const size_t row_size = CAMERA_CAPTURE_WIDTH * 2;
    const size_t input_size = row_size * CAMERA_CAPTURE_HEIGHT;
    const size_t output_capacity = CAMERA_CAPTURE_WIDTH * CAMERA_CAPTURE_HEIGHT * 3 + 1024;
    if (pix->width != CAMERA_CAPTURE_WIDTH || pix->height != CAMERA_CAPTURE_HEIGHT ||
        pix->pixelformat != V4L2_PIX_FMT_UYVY || pix->bytesperline < row_size ||
        frame_size < (CAMERA_CAPTURE_HEIGHT - 1) * pix->bytesperline + row_size) {
        ESP_LOGE(TAG, "Unsupported or incomplete UYVY frame for JPEG");
        return ESP_ERR_INVALID_SIZE;
    }

    jpeg_encoder_handle_t encoder = NULL;
    size_t input_allocated = 0, output_allocated = 0;
    const jpeg_encode_memory_alloc_cfg_t input_mem = {
        .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
    };
    const jpeg_encode_memory_alloc_cfg_t output_mem = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    uint8_t *input = jpeg_alloc_encoder_mem(input_size, &input_mem, &input_allocated);
    uint8_t *output = jpeg_alloc_encoder_mem(output_capacity, &output_mem, &output_allocated);
    esp_err_t ret = ESP_ERR_NO_MEM;
    if (!input || !output) {
        ESP_LOGE(TAG, "Cannot allocate JPEG buffers");
        goto done;
    }
    /* IDF's documented YUV422 order is U Y0 V Y1. Remove any row padding. */
    for (size_t y = 0; y < CAMERA_CAPTURE_HEIGHT; ++y) {
        memcpy(input + y * row_size, frame + y * pix->bytesperline, row_size);
    }
    const jpeg_encode_engine_cfg_t engine_cfg = {
        .timeout_ms = 1000,
    };
    ret = jpeg_new_encoder_engine(&engine_cfg, &encoder);
    if (ret != ESP_OK) {
        goto done;
    }
    const jpeg_encode_cfg_t cfg = {
        .src_type = JPEG_ENCODE_IN_FORMAT_YUV422,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = 80,
        .width = CAMERA_CAPTURE_WIDTH,
        .height = CAMERA_CAPTURE_HEIGHT,
        .pixel_reverse = false,
    };
    uint32_t jpeg_size = 0;
    ret = jpeg_encoder_process(encoder, &cfg, input, input_size,
                               output, output_allocated, &jpeg_size);
    if (ret != ESP_OK) {
        goto done;
    }
    if (jpeg_size < 4 || jpeg_size > output_allocated ||
        output[0] != 0xff || output[1] != 0xd8) {
        ret = ESP_FAIL;
        goto done;
    }
    FILE *file = fopen(CAMERA_JPEG_PATH, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Cannot create %s: errno=%d", CAMERA_JPEG_PATH, errno);
        ret = ESP_FAIL;
        goto done;
    }
    size_t written = fwrite(output, 1, jpeg_size, file);
    int close_result = fclose(file);
    if (written != jpeg_size || close_result != 0) {
        remove(CAMERA_JPEG_PATH);
        ret = ESP_FAIL;
        goto done;
    }
    ESP_LOGI(TAG, "JPEG saved: %s (%lu bytes), quality=80", CAMERA_JPEG_PATH,
             (unsigned long)jpeg_size);

done:
    if (encoder) {
        jpeg_del_encoder_engine(encoder);
    }
    free(input);
    free(output);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG save failed: %s; raw frame retained", esp_err_to_name(ret));
    }
    return ret;
}

static esp_video_init_csi_config_t s_csi_config = {
    .sccb_config = {
        .init_sccb = false,
        .freq = CAMERA_SCCB_FREQ,
    },
    .reset_pin = CAMERA_RESET_GPIO,
    .pwdn_pin = -1,
    .dont_init_ldo = false,
};

/* -------------------------------------------------------
 * CAMERA INITIALIZATION
 * ------------------------------------------------------- */

esp_err_t camera_init(void)
{
    if (s_camera_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "SC2336 MIPI CSI camera initialization");
    ESP_LOGI(TAG, "====================================");

    /* Reuse the I2C bus created by audio_init(). */
    i2c_master_bus_handle_t i2c_handle =
        audio_get_i2c_bus_handle();

    if (i2c_handle == NULL) {
        ESP_LOGE(TAG, "Audio I2C bus is not available");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Using shared audio I2C bus");
    ESP_LOGI(TAG, "SCCB SDA: GPIO7");
    ESP_LOGI(TAG, "SCCB SCL: GPIO8");
    ESP_LOGI(TAG, "Camera RESET: GPIO%d", CAMERA_RESET_GPIO);
    ESP_LOGI(TAG, "Camera XCLK: External 24 MHz crystal");

    gpio_config_t csi_en_config = {
        .pin_bit_mask = (1ULL << CAMERA_CSI_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&csi_en_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI_EN GPIO configuration failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_set_level(CAMERA_CSI_EN_GPIO, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable camera power");
        return ret;
    }

    ESP_LOGI(TAG, "CSI_EN GPIO27 HIGH - camera power enabled");

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Attach the existing audio I2C bus. */
    s_csi_config.sccb_config.init_sccb = false;
    s_csi_config.sccb_config.i2c_handle = i2c_handle;

    const esp_video_init_config_t video_config = {
        .csi = &s_csi_config,
    };

    ESP_LOGI(TAG, "Initializing ESP-Video MIPI CSI...");
#if CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER
    ESP_LOGI(TAG, "ISP automatic pipeline controller enabled (exposure / white balance)");
#else
    ESP_LOGW(TAG, "ISP automatic pipeline controller disabled");
#endif

    ret = esp_video_init(&video_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-Video initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    s_camera_initialized = true;

    ESP_LOGI(TAG, "ESP-Video initialization successful");
    ESP_LOGI(TAG, "Camera initialization completed");

    ESP_LOGI(TAG, "========== MEMORY BEFORE CAPTURE ==========");
    ESP_LOGI(TAG, "Free heap:       %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    ESP_LOGI(TAG, "Largest block:   %lu bytes",
             (unsigned long)heap_caps_get_largest_free_block(
                 MALLOC_CAP_8BIT));

    ESP_LOGI(TAG, "Free PSRAM:      %lu bytes",
             (unsigned long)heap_caps_get_free_size(
                 MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "Largest PSRAM:   %lu bytes",
             (unsigned long)heap_caps_get_largest_free_block(
                 MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "Free DMA memory: %lu bytes",
             (unsigned long)heap_caps_get_free_size(
                 MALLOC_CAP_DMA));

    ESP_LOGI(TAG, "Largest DMA:     %lu bytes",
             (unsigned long)heap_caps_get_largest_free_block(
                 MALLOC_CAP_DMA));

    return ESP_OK;
}

/* -------------------------------------------------------
 * LOG PIXEL FORMAT
 * ------------------------------------------------------- */

static void camera_log_pixel_format(uint32_t format)
{
    ESP_LOGI(TAG, "Pixel format: %c%c%c%c (0x%08" PRIx32 ")",
             (char)(format & 0xFF),
             (char)((format >> 8) & 0xFF),
             (char)((format >> 16) & 0xFF),
             (char)((format >> 24) & 0xFF),
             format);
}

/* -------------------------------------------------------
 * CAPTURE ONE FRAME AND SAVE TO SD CARD
 * ------------------------------------------------------- */

esp_err_t camera_capture_test(void)
{
    if (!s_camera_initialized) {
        ESP_LOGE(TAG, "Camera is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!sd_card_is_mounted()) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "SC2336 single-frame capture test");
    ESP_LOGI(TAG, "====================================");

    esp_err_t ret = ESP_FAIL;
    int fd = -1;
    bool streaming = false;
    bool buffers_requested = false;

    uint8_t *buffers[CAMERA_BUFFER_COUNT] = { NULL };
    size_t buffer_lengths[CAMERA_BUFFER_COUNT] = { 0 };

    uint32_t allocated_count = 0;

    const enum v4l2_buf_type type =
        V4L2_BUF_TYPE_VIDEO_CAPTURE;

    /* OPEN DEVICE */

    fd = open(CAMERA_DEVICE_PATH, O_RDONLY);

    if (fd < 0) {
        ESP_LOGE(TAG, "Cannot open %s: errno=%d",
                 CAMERA_DEVICE_PATH, errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Opened camera device: %s",
             CAMERA_DEVICE_PATH);

    /* QUERY CAPABILITIES */

    struct v4l2_capability capability = { 0 };

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QUERYCAP failed: errno=%d", errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Camera driver: %s", capability.driver);
    ESP_LOGI(TAG, "Camera card: %s", capability.card);

    /* ENUMERATE SUPPORTED FORMATS */

    /* ENUMERATE SUPPORTED PIXEL FORMATS */

    ESP_LOGI(TAG, "Enumerating supported pixel formats...");

    for (uint32_t i = 0; ; i++) {
        struct v4l2_fmtdesc desc = {
            .index = i,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FMT, &desc) != 0) {
            break;
        }

        ESP_LOGI(TAG, "Format[%lu]: %c%c%c%c",
                 (unsigned long)i,
                 (char)(desc.pixelformat & 0xFF),
                 (char)((desc.pixelformat >> 8) & 0xFF),
                 (char)((desc.pixelformat >> 16) & 0xFF),
                 (char)((desc.pixelformat >> 24) & 0xFF));
    }

    /* ---------------------------------------------
     * ENUMERATE SUPPORTED FRAME SIZES
     * --------------------------------------------- */

    ESP_LOGI(TAG, "Enumerating supported frame sizes...");

    for (uint32_t fmt_index = 0; ; fmt_index++) {

        struct v4l2_fmtdesc fmt_desc = {
            .index = fmt_index,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmt_desc) != 0) {
            break;
        }

        ESP_LOGI(TAG, "Frame sizes for format %c%c%c%c:",
                 (char)(fmt_desc.pixelformat & 0xFF),
                 (char)((fmt_desc.pixelformat >> 8) & 0xFF),
                 (char)((fmt_desc.pixelformat >> 16) & 0xFF),
                 (char)((fmt_desc.pixelformat >> 24) & 0xFF));

        for (uint32_t size_index = 0; ; size_index++) {

            struct v4l2_frmsizeenum size_desc = {
                .index = size_index,
                .pixel_format = fmt_desc.pixelformat,
            };

            if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size_desc) != 0) {
                break;
            }

            if (size_desc.type == V4L2_FRMSIZE_TYPE_DISCRETE) {

                ESP_LOGI(TAG, "  %lu x %lu",
                         (unsigned long)size_desc.discrete.width,
                         (unsigned long)size_desc.discrete.height);

            } else if (size_desc.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
                       size_desc.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {

                ESP_LOGI(TAG, "  Range: %lu x %lu to %lu x %lu",
                         (unsigned long)size_desc.stepwise.min_width,
                         (unsigned long)size_desc.stepwise.min_height,
                         (unsigned long)size_desc.stepwise.max_width,
                         (unsigned long)size_desc.stepwise.max_height);
            }
        }
    }
    /* SET EXPLICIT CAPTURE FORMAT */

    struct v4l2_format format = {
        .type = type,
    };

    format.fmt.pix.width = CAMERA_CAPTURE_WIDTH;
    format.fmt.pix.height = CAMERA_CAPTURE_HEIGHT;
    format.fmt.pix.pixelformat = CAMERA_CAPTURE_FORMAT;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.bytesperline = CAMERA_CAPTURE_WIDTH * 2;
    format.fmt.pix.sizeimage =
        CAMERA_CAPTURE_WIDTH * CAMERA_CAPTURE_HEIGHT * 2;

    ESP_LOGI(TAG, "Requesting capture format:");
    ESP_LOGI(TAG, "Resolution: %u x %u",
             (unsigned)format.fmt.pix.width,
             (unsigned)format.fmt.pix.height);

    camera_log_pixel_format(format.fmt.pix.pixelformat);

    ESP_LOGI(TAG, "Requested bytes per line: %u",
             (unsigned)format.fmt.pix.bytesperline);

    ESP_LOGI(TAG, "Requested image size: %u",
             (unsigned)format.fmt.pix.sizeimage);

    if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed: errno=%d", errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Driver negotiated format:");

    ESP_LOGI(TAG, "Resolution: %u x %u",
             (unsigned)format.fmt.pix.width,
             (unsigned)format.fmt.pix.height);

    camera_log_pixel_format(format.fmt.pix.pixelformat);

    ESP_LOGI(TAG, "Bytes per line: %u",
             (unsigned)format.fmt.pix.bytesperline);

    ESP_LOGI(TAG, "Image size: %u bytes",
             (unsigned)format.fmt.pix.sizeimage);

    /*
     * Do not continue if the driver did not accept
     * a usable image layout.
     */
    if (format.fmt.pix.width == 0 ||
        format.fmt.pix.height == 0 ||
        format.fmt.pix.bytesperline == 0 ||
        format.fmt.pix.sizeimage == 0) {

        ESP_LOGE(TAG, "Invalid negotiated image layout");
        goto cleanup;
    }

    if (format.fmt.pix.pixelformat != CAMERA_CAPTURE_FORMAT) {
        ESP_LOGE(TAG, "Driver returned a different pixel format");
        goto cleanup;
    }

    /* REQUEST MMAP BUFFERS */

    struct v4l2_requestbuffers request = {
        .count = CAMERA_BUFFER_COUNT,
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (ioctl(fd, VIDIOC_REQBUFS, &request) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: errno=%d", errno);
        goto cleanup;
    }

    buffers_requested = true;
    allocated_count = request.count;

    ESP_LOGI(TAG, "Driver allocated %lu buffers",
             (unsigned long)allocated_count);

    if (allocated_count == 0 ||
        allocated_count > CAMERA_BUFFER_COUNT) {

        ESP_LOGE(TAG, "Unexpected buffer count: %lu",
                 (unsigned long)allocated_count);
        goto cleanup;
    }

    /* MAP AND QUEUE BUFFERS */

    for (uint32_t i = 0; i < allocated_count; i++) {
        struct v4l2_buffer buffer = {
            .type = type,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };

        if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) != 0) {
            ESP_LOGE(TAG,
                     "VIDIOC_QUERYBUF failed: index=%lu errno=%d",
                     (unsigned long)i, errno);
            goto cleanup;
        }

        if (buffer.length == 0) {
            ESP_LOGE(TAG, "Buffer %lu has zero length",
                     (unsigned long)i);
            goto cleanup;
        }

        void *mapped = mmap(
            NULL,
            buffer.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            buffer.m.offset
        );

        if (mapped == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap failed: index=%lu errno=%d",
                     (unsigned long)i, errno);
            goto cleanup;
        }

        buffers[i] = (uint8_t *)mapped;
        buffer_lengths[i] = buffer.length;

        ESP_LOGI(TAG, "Buffer %lu mapped: %lu bytes",
                 (unsigned long)i,
                 (unsigned long)buffer.length);

        if (ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: index=%lu errno=%d",
                     (unsigned long)i, errno);
            goto cleanup;
        }
    }

    /* START STREAMING */

    if (ioctl(fd, VIDIOC_STREAMON, (void *)&type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d", errno);
        goto cleanup;
    }

    streaming = true;

    ESP_LOGI(TAG, "CSI streaming started");
    ESP_LOGI(TAG, "Waiting for first frame...");

    /* DEQUEUE FIRST FRAME */

    struct v4l2_buffer captured = {
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (ioctl(fd, VIDIOC_DQBUF, &captured) != 0) {
        ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
        goto cleanup;
    }

    if (captured.index >= allocated_count) {
        ESP_LOGE(TAG, "Invalid buffer index: %u",
                 captured.index);
        goto cleanup;
    }

    if (captured.flags & V4L2_BUF_FLAG_ERROR) {
        ESP_LOGE(TAG, "Captured frame has error flag");
        goto cleanup;
    }

    if (captured.bytesused == 0 ||
        captured.bytesused > buffer_lengths[captured.index]) {

        ESP_LOGE(TAG, "Invalid frame size: %lu",
                 (unsigned long)captured.bytesused);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Frame captured successfully");
    ESP_LOGI(TAG, "Buffer index: %u", captured.index);
    ESP_LOGI(TAG, "Frame size: %lu bytes",
             (unsigned long)captured.bytesused);

    /* SAVE RAW FRAME TO SD CARD */

    FILE *file = fopen(CAMERA_FRAME_PATH, "wb");

    if (file == NULL) {
        ESP_LOGE(TAG, "Cannot create %s: errno=%d",
                 CAMERA_FRAME_PATH, errno);
        goto cleanup;
    }

    /* Diagnose slot-0 multi-block writes without changing the 4-bit bus.
     * Unbuffered, sector-sized requests prevent stdio merging this frame
     * into large bursts. This is a diagnostic, not a throughput target. */
    if (setvbuf(file, NULL, _IONBF, 0) != 0) {
        fclose(file);
        ESP_LOGE(TAG, "Cannot configure unbuffered raw frame output");
        goto cleanup;
    }
    ESP_LOGI(TAG, "Raw save diagnostic: 512-byte file writes, SD remains 4-bit");
    size_t written = 0;
    while (written < captured.bytesused) {
        size_t chunk = captured.bytesused - written;
        if (chunk > 512) chunk = 512;
        size_t count = fwrite(buffers[captured.index] + written, 1, chunk, file);
        written += count;
        if (count != chunk) break;
    }

    int close_result = fclose(file);

    if (written != captured.bytesused || close_result != 0) {
        ESP_LOGE(TAG, "File write failed: %lu / %lu bytes",
                 (unsigned long)written,
                 (unsigned long)captured.bytesused);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Frame saved: %s", CAMERA_FRAME_PATH);
    ESP_LOGI(TAG, "Saved %lu bytes", (unsigned long)written);

    ret = camera_save_jpeg(buffers[captured.index], captured.bytesused, &format.fmt.pix);

cleanup:

    /* STOP STREAMING */

    if (streaming) {
        if (ioctl(fd, VIDIOC_STREAMOFF, (void *)&type) != 0) {
            ESP_LOGW(TAG, "VIDIOC_STREAMOFF failed: errno=%d",
                     errno);
            if (ret == ESP_OK) {
                ret = ESP_FAIL;
            }
        } else {
            ESP_LOGI(TAG, "CSI streaming stopped");
        }
    }

    /* UNMAP BUFFERS */

    for (uint32_t i = 0; i < CAMERA_BUFFER_COUNT; i++) {
        if (buffers[i] != NULL) {
            munmap(buffers[i], buffer_lengths[i]);
            buffers[i] = NULL;
        }
    }

    /* RELEASE DRIVER BUFFERS */

    if (fd >= 0) {
        if (buffers_requested) {
            struct v4l2_requestbuffers release = {
                .count = 0,
                .type = type,
                .memory = V4L2_MEMORY_MMAP,
            };

            if (ioctl(fd, VIDIOC_REQBUFS, &release) != 0) {
                ESP_LOGW(TAG, "Failed to release camera buffers");
            }
        }

        close(fd);
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SC2336 frame capture PASS");
    } else {
        ESP_LOGE(TAG, "SC2336 frame capture FAIL");
    }

    return ret;
}

/* -------------------------------------------------------
 * CAMERA DEINITIALIZATION
 * ------------------------------------------------------- */

esp_err_t camera_deinit(void)
{
    if (!s_camera_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing camera...");

    esp_err_t ret = esp_video_deinit();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinitialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    s_camera_initialized = false;

    gpio_set_level(CAMERA_CSI_EN_GPIO, 0);

    /* The I2C master bus belongs to audio.c. */
    /* Do NOT delete the shared I2C bus here. */

    ESP_LOGI(TAG, "Camera deinitialized");

    return ESP_OK;
}

bool camera_is_initialized(void)
{
    return s_camera_initialized;
}

esp_err_t camera_stream_jpeg(camera_jpeg_sink_t sink, void *ctx)
{
    if (!s_camera_initialized || !sink) return ESP_ERR_INVALID_STATE;
    const enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    const size_t row_bytes = CAMERA_CAPTURE_WIDTH * 2;
    const size_t raw_bytes = row_bytes * CAMERA_CAPTURE_HEIGHT;
    uint8_t *mapped[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};
    uint8_t *input = NULL, *output = NULL;
    size_t input_size = 0, output_size = 0;
    jpeg_encoder_handle_t encoder = NULL;
    bool streaming = false, requested = false;
    esp_err_t ret = ESP_FAIL;
    int fd = open(CAMERA_DEVICE_PATH, O_RDONLY);
    if (fd < 0) return ESP_FAIL;

    struct v4l2_format format = {.type = type};
    format.fmt.pix.width = CAMERA_CAPTURE_WIDTH;
    format.fmt.pix.height = CAMERA_CAPTURE_HEIGHT;
    format.fmt.pix.pixelformat = CAMERA_CAPTURE_FORMAT;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.bytesperline = row_bytes;
    format.fmt.pix.sizeimage = raw_bytes;
    if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) goto done;
    if (format.fmt.pix.width != CAMERA_CAPTURE_WIDTH ||
        format.fmt.pix.height != CAMERA_CAPTURE_HEIGHT ||
        format.fmt.pix.pixelformat != CAMERA_CAPTURE_FORMAT ||
        format.fmt.pix.bytesperline < row_bytes) {
        ESP_LOGE(TAG, "Unsupported stream layout");
        goto done;
    }
    struct v4l2_requestbuffers request = {
        .count = CAMERA_BUFFER_COUNT, .type = type, .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &request) != 0) goto done;
    requested = true;
    if (!request.count || request.count > CAMERA_BUFFER_COUNT) goto done;
    for (uint32_t i = 0; i < request.count; ++i) {
        struct v4l2_buffer buffer = {.type = type, .memory = V4L2_MEMORY_MMAP, .index = i};
        if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) != 0 || !buffer.length) goto done;
        void *ptr = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
        if (ptr == MAP_FAILED) goto done;
        mapped[i] = ptr;
        lengths[i] = buffer.length;
        if (ioctl(fd, VIDIOC_QBUF, &buffer) != 0) goto done;
    }
    const jpeg_encode_memory_alloc_cfg_t in_mem = {.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER};
    const jpeg_encode_memory_alloc_cfg_t out_mem = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    input = jpeg_alloc_encoder_mem(raw_bytes, &in_mem, &input_size);
    output = jpeg_alloc_encoder_mem(CAMERA_CAPTURE_WIDTH * CAMERA_CAPTURE_HEIGHT * 3 + 1024,
                                     &out_mem, &output_size);
    if (!input || !output) {
        ret = ESP_ERR_NO_MEM;
        goto done;
    }
    const jpeg_encode_engine_cfg_t engine = {.timeout_ms = 1000};
    ret = jpeg_new_encoder_engine(&engine, &encoder);
    if (ret != ESP_OK) goto done;
    ret = ESP_FAIL;
    if (ioctl(fd, VIDIOC_STREAMON, (void *)&type) != 0) goto done;
    streaming = true;
    const jpeg_encode_cfg_t cfg = {
        .width = CAMERA_CAPTURE_WIDTH, .height = CAMERA_CAPTURE_HEIGHT,
        .src_type = JPEG_ENCODE_IN_FORMAT_YUV422, .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = 80, .pixel_reverse = false,
    };
    const TickType_t start = xTaskGetTickCount();
    TickType_t last_frame = start;
    unsigned sent = 0;
    for (;;) {
        struct v4l2_buffer frame = {.type = type, .memory = V4L2_MEMORY_MMAP};
        ret = ESP_FAIL;
        if (ioctl(fd, VIDIOC_DQBUF, &frame) != 0) break;
        if (frame.index >= request.count) break;
        TickType_t now = xTaskGetTickCount();
        /* Recycle frames throughout warm-up and between sends (target at most 10 fps). */
        if ((frame.flags & V4L2_BUF_FLAG_ERROR) ||
            now - start < pdMS_TO_TICKS(2000) || now - last_frame < pdMS_TO_TICKS(100)) {
            if (ioctl(fd, VIDIOC_QBUF, &frame) != 0) break;
            continue;
        }
        size_t needed = (CAMERA_CAPTURE_HEIGHT - 1) * format.fmt.pix.bytesperline + row_bytes;
        if (frame.bytesused < needed || frame.bytesused > lengths[frame.index]) {
            ESP_LOGE(TAG, "Incomplete streaming frame");
            break;
        }
        for (size_t y = 0; y < CAMERA_CAPTURE_HEIGHT; ++y) {
            memcpy(input + y * row_bytes,
                   mapped[frame.index] + y * format.fmt.pix.bytesperline, row_bytes);
        }
        if (ioctl(fd, VIDIOC_QBUF, &frame) != 0) break;
        uint32_t jpeg_size = 0;
        ret = jpeg_encoder_process(encoder, &cfg, input, raw_bytes, output, output_size, &jpeg_size);
        if (ret != ESP_OK) break;
        if (jpeg_size < 4 || jpeg_size > output_size || output[0] != 0xff || output[1] != 0xd8) {
            ret = ESP_FAIL;
            break;
        }
        ret = sink(output, jpeg_size, ctx);
        if (ret != ESP_OK) break;
        last_frame = now;
        if (++sent == 1 || sent % 100 == 0) {
            ESP_LOGI(TAG, "Stream sent %u frames; last JPEG %lu bytes", sent, (unsigned long)jpeg_size);
        }
    }

done:
    if (streaming) ioctl(fd, VIDIOC_STREAMOFF, (void *)&type);
    for (unsigned i = 0; i < CAMERA_BUFFER_COUNT; ++i) {
        if (mapped[i]) munmap(mapped[i], lengths[i]);
    }
    if (requested) {
        struct v4l2_requestbuffers release = {.count = 0, .type = type, .memory = V4L2_MEMORY_MMAP};
        ioctl(fd, VIDIOC_REQBUFS, &release);
    }
    close(fd);
    if (encoder) jpeg_del_encoder_engine(encoder);
    free(input);
    free(output);
    return ret;
}
