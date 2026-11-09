
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

#include "audio.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define TAG                     "AUDIO"

#define CODEC_EN_GPIO           GPIO_NUM_5
#define PA_CTRL_GPIO            GPIO_NUM_4

#define AUDIO_I2C_PORT          I2C_NUM_0
#define AUDIO_I2C_SDA           GPIO_NUM_7
#define AUDIO_I2C_SCL           GPIO_NUM_8
#define AUDIO_I2C_FREQ_HZ       100000

#define AUDIO_I2S_PORT          I2S_NUM_0

#define AUDIO_MCLK_GPIO         GPIO_NUM_13
#define AUDIO_BCLK_GPIO         GPIO_NUM_12
#define AUDIO_DIN_GPIO          GPIO_NUM_11
#define AUDIO_LRCK_GPIO         GPIO_NUM_10
#define AUDIO_DOUT_GPIO         GPIO_NUM_9

#define AUDIO_SAMPLE_RATE       16000
#define AUDIO_BIT_WIDTH         I2S_DATA_BIT_WIDTH_16BIT

#define AUDIO_BEEP_VOLUME       40
#define AUDIO_MUSIC_VOLUME      70

#define AUDIO_MCLK_MULTIPLE     I2S_MCLK_MULTIPLE_256

#define AUDIO_TONE_AMPLITUDE    28000

// Microphone gain baseline.
// Start at +20 dB to avoid unnecessarily amplifying background noise.
#define MIC_INPUT_GAIN_DB       20.0f

// =====================================================
// GLOBAL HANDLES
// =====================================================

static i2c_master_bus_handle_t audio_i2c_bus = NULL;

static i2s_chan_handle_t audio_tx_handle = NULL;
static i2s_chan_handle_t audio_rx_handle = NULL;

static const audio_codec_data_if_t *audio_data_if = NULL;
static const audio_codec_ctrl_if_t *audio_ctrl_if = NULL;
static const audio_codec_gpio_if_t *audio_gpio_if = NULL;
static const audio_codec_if_t *audio_codec_if = NULL;

static esp_codec_dev_handle_t audio_codec = NULL;

static bool audio_initialized = false;

// =====================================================
// OUTPUT VOLUME CONTROL
// =====================================================

static esp_err_t audio_set_output_volume(int volume)
{
    if (audio_codec == NULL) {
        ESP_LOGE(TAG, "Cannot set volume: codec not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_codec_dev_set_out_vol(audio_codec, volume);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set volume to %d: %s",
                 volume, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Output volume set to %d/100", volume);

    return ESP_OK;
}

// =====================================================
// POWER INITIALIZATION
// =====================================================

static esp_err_t audio_power_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask =
            (1ULL << CODEC_EN_GPIO) |
            (1ULL << PA_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&config);

    if (ret != ESP_OK) {
        return ret;
    }

    // Keep speaker amplifier disabled.
    ret = gpio_set_level(PA_CTRL_GPIO, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    // Power-cycle codec and microphone rail.
    ret = gpio_set_level(CODEC_EN_GPIO, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    ret = gpio_set_level(CODEC_EN_GPIO, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "CODEC_EN GPIO5 HIGH");
    ESP_LOGI(TAG, "Codec/microphone supply enabled");
    ESP_LOGI(TAG, "PA_CTRL GPIO4 LOW");

    return ESP_OK;
}

// =====================================================
// I2C INITIALIZATION
// =====================================================

static esp_err_t audio_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = AUDIO_I2C_PORT,
        .sda_io_num = AUDIO_I2C_SDA,
        .scl_io_num = AUDIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(
        &bus_config,
        &audio_i2c_bus
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized: SDA=%d SCL=%d",
             AUDIO_I2C_SDA, AUDIO_I2C_SCL);

    return ESP_OK;
}

// =====================================================
// I2S INITIALIZATION
// =====================================================

static esp_err_t audio_i2s_init(void)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            AUDIO_I2S_PORT,
            I2S_ROLE_MASTER
        );

    channel_config.auto_clear = true;

    esp_err_t ret = i2s_new_channel(
        &channel_config,
        &audio_tx_handle,
        &audio_rx_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S channel creation failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(
            AUDIO_SAMPLE_RATE
        ),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            AUDIO_BIT_WIDTH,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
            .mclk = AUDIO_MCLK_GPIO,
            .bclk = AUDIO_BCLK_GPIO,
            .ws = AUDIO_LRCK_GPIO,
            .dout = AUDIO_DOUT_GPIO,
            .din = AUDIO_DIN_GPIO,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    standard_config.clk_cfg.mclk_multiple =
        AUDIO_MCLK_MULTIPLE;

    ret = i2s_channel_init_std_mode(
        audio_tx_handle,
        &standard_config
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S TX initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_init_std_mode(
        audio_rx_handle,
        &standard_config
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S RX initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(audio_tx_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S TX enable failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(audio_rx_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S RX enable failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S full-duplex initialized");
    ESP_LOGI(TAG, "Sample rate: %d Hz", AUDIO_SAMPLE_RATE);
    ESP_LOGI(TAG, "Bits per sample: 16");
    ESP_LOGI(TAG, "Channels: Stereo");
    ESP_LOGI(TAG, "MCLK: GPIO13");
    ESP_LOGI(TAG, "BCLK: GPIO12");
    ESP_LOGI(TAG, "LRCK: GPIO10");
    ESP_LOGI(TAG, "DOUT: GPIO9");
    ESP_LOGI(TAG, "DIN: GPIO11");

    return ESP_OK;
}

// =====================================================
// ES8311 INITIALIZATION
// =====================================================

static esp_err_t audio_codec_init(void)
{
    audio_codec_i2s_cfg_t i2s_config = {
        .port = AUDIO_I2S_PORT,
        .tx_handle = audio_tx_handle,
        .rx_handle = audio_rx_handle,
    };

    audio_data_if = audio_codec_new_i2s_data(&i2s_config);

    if (audio_data_if == NULL) {
        ESP_LOGE(TAG, "Cannot create I2S data interface");
        return ESP_FAIL;
    }

    audio_codec_i2c_cfg_t i2c_config = {
        .port = AUDIO_I2C_PORT,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = audio_i2c_bus,
    };

    audio_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_config);

    if (audio_ctrl_if == NULL) {
        ESP_LOGE(TAG, "Cannot create ES8311 I2C interface");
        return ESP_FAIL;
    }

    audio_gpio_if = audio_codec_new_gpio();

    if (audio_gpio_if == NULL) {
        ESP_LOGE(TAG, "Cannot create codec GPIO interface");
        return ESP_FAIL;
    }

    es8311_codec_cfg_t codec_config = {
        .ctrl_if = audio_ctrl_if,
        .gpio_if = audio_gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .no_dac_ref = true,

        // ESP32 is I2S master.
        .master_mode = false,

        .use_mclk = true,

        // NS4150B amplifier control.
        .pa_pin = PA_CTRL_GPIO,
        .pa_reverted = false,

        // LMA3729T421-OA5 is an analog-output microphone.
        .digital_mic = false,

        .invert_mclk = false,
        .invert_sclk = false,
        .mclk_div = 256,
    };

    audio_codec_if = es8311_codec_new(&codec_config);

    if (audio_codec_if == NULL) {
        ESP_LOGE(TAG, "ES8311 driver creation failed");
        return ESP_FAIL;
    }

    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = audio_codec_if,
        .data_if = audio_data_if,
    };

    audio_codec = esp_codec_dev_new(&device_config);

    if (audio_codec == NULL) {
        ESP_LOGE(TAG, "ES8311 codec device creation failed");
        return ESP_FAIL;
    }

    esp_codec_dev_sample_info_t sample_config = {
        .sample_rate = AUDIO_SAMPLE_RATE,
        .channel = 2,
        .channel_mask = 0,
        .bits_per_sample = 16,
        .mclk_multiple = 256,
    };

    // Open codec first.
    esp_err_t ret = esp_codec_dev_open(
        audio_codec,
        &sample_config
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 open failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Configure microphone gain only after successful open.
    int gain_ret = esp_codec_dev_set_in_gain(
        audio_codec,
        MIC_INPUT_GAIN_DB
    );

    if (gain_ret == ESP_CODEC_DEV_OK) {
        ESP_LOGI(TAG, "Microphone input gain configured: %.1f dB",
                 (double)MIC_INPUT_GAIN_DB);
    } else {
        ESP_LOGE(TAG, "Microphone input gain configuration failed: %d",
                 gain_ret);
    }

    ret = audio_set_output_volume(AUDIO_BEEP_VOLUME);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initial volume configuration failed");
        return ret;
    }

    ESP_LOGI(TAG, "ES8311 initialized successfully");
    ESP_LOGI(TAG, "ES8311 address: 0x%02X",
             ES8311_CODEC_DEFAULT_ADDR);
    ESP_LOGI(TAG, "Beep volume: %d/100", AUDIO_BEEP_VOLUME);
    ESP_LOGI(TAG, "Music volume: %d/100", AUDIO_MUSIC_VOLUME);

    return ESP_OK;
}

// =====================================================
// AUDIO INITIALIZATION
// =====================================================

esp_err_t audio_init(void)
{
    if (audio_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "       AUDIO DIAGNOSTIC");
    ESP_LOGI(TAG, "================================");

    esp_err_t ret = audio_power_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = audio_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = audio_i2s_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = audio_codec_init();
    if (ret != ESP_OK) {
        return ret;
    }

    audio_initialized = true;

    ESP_LOGI(TAG, "Audio initialization PASS");

    return ESP_OK;
}

// =====================================================
// SHARED I2C BUS HANDLE
// =====================================================

i2c_master_bus_handle_t audio_get_i2c_bus_handle(void)
{
    return audio_i2c_bus;
}

// =====================================================
// TEST TONE GENERATION
// =====================================================

esp_err_t audio_play_test_tone(
    uint32_t frequency_hz,
    uint32_t duration_ms)
{
    if (!audio_initialized || audio_codec == NULL) {
        ESP_LOGE(TAG, "Audio is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (frequency_hz == 0 ||
        frequency_hz >= AUDIO_SAMPLE_RATE / 2 ||
        duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Playing test tone: %lu Hz, %lu ms",
             (unsigned long)frequency_hz,
             (unsigned long)duration_ms);

    esp_err_t ret = audio_set_output_volume(AUDIO_BEEP_VOLUME);

    if (ret != ESP_OK) {
        return ret;
    }

    gpio_set_level(PA_CTRL_GPIO, 1);
    ESP_LOGI(TAG, "PA_CTRL HIGH (amplifier enabled)");

    vTaskDelay(pdMS_TO_TICKS(100));

    int16_t pcm_buffer[256 * 2];
    const float two_pi = 6.28318530718f;

    uint32_t total_frames =
        (AUDIO_SAMPLE_RATE * duration_ms) / 1000;

    uint32_t frames_written = 0;

    while (frames_written < total_frames) {

        uint32_t frames_this_block = 256;

        if (frames_written + frames_this_block > total_frames) {
            frames_this_block = total_frames - frames_written;
        }

        for (uint32_t i = 0; i < frames_this_block; i++) {

            uint32_t sample_number = frames_written + i;

            float phase =
                two_pi * frequency_hz *
                sample_number / AUDIO_SAMPLE_RATE;

            int16_t sample =
                (int16_t)(sinf(phase) * AUDIO_TONE_AMPLITUDE);

            pcm_buffer[i * 2] = sample;
            pcm_buffer[i * 2 + 1] = sample;
        }

        ret = esp_codec_dev_write(
            audio_codec,
            (void *)pcm_buffer,
            frames_this_block * 2 * sizeof(int16_t)
        );

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Audio write failed: %s",
                     esp_err_to_name(ret));
            break;
        }

        frames_written += frames_this_block;
    }

    gpio_set_level(PA_CTRL_GPIO, 0);
    ESP_LOGI(TAG, "PA_CTRL LOW (amplifier disabled)");

    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Test tone playback completed");

    return ESP_OK;
}

// =====================================================
// WAV FILE PLAYBACK
// =====================================================

#define WAV_TAG                 "WAV"
#define WAV_SAMPLE_RATE         16000
#define WAV_BITS_PER_SAMPLE     16
#define WAV_BUFFER_FRAMES       256

static uint16_t wav_read_u16(FILE *file, bool *ok)
{
    uint8_t b[2];

    if (fread(b, 1, 2, file) != 2) {
        *ok = false;
        return 0;
    }

    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t wav_read_u32(FILE *file, bool *ok)
{
    uint8_t b[4];

    if (fread(b, 1, 4, file) != 4) {
        *ok = false;
        return 0;
    }

    return (uint32_t)b[0] |
           ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
}

esp_err_t audio_play_wav_file(const char *path)
{
    if (!audio_initialized || audio_codec == NULL) {
        ESP_LOGE(WAV_TAG, "Audio is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(path, "rb");

    if (file == NULL) {
        ESP_LOGE(WAV_TAG, "Cannot open WAV file: %s", path);
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    bool ok = true;

    uint8_t riff[4];
    uint8_t wave[4];

    if (fread(riff, 1, 4, file) != 4 ||
        wav_read_u32(file, &ok) == 0 ||
        fread(wave, 1, 4, file) != 4 ||
        !ok ||
        memcmp(riff, "RIFF", 4) != 0 ||
        memcmp(wave, "WAVE", 4) != 0) {

        ESP_LOGE(WAV_TAG, "Invalid RIFF/WAVE file");
        goto cleanup;
    }

    uint16_t audio_format = 0;
    uint16_t channels = 0;
    uint32_t sample_rate = 0;
    uint16_t bits_per_sample = 0;
    uint16_t block_align = 0;

    uint32_t data_size = 0;
    bool fmt_found = false;
    bool data_found = false;

    while (!feof(file)) {

        uint8_t chunk_id[4];
        size_t n = fread(chunk_id, 1, 4, file);

        if (n == 0) {
            break;
        }

        if (n != 4) {
            ESP_LOGE(WAV_TAG, "Incomplete WAV chunk header");
            goto cleanup;
        }

        uint32_t chunk_size = wav_read_u32(file, &ok);

        if (!ok) {
            ESP_LOGE(WAV_TAG, "Cannot read chunk size");
            goto cleanup;
        }

        if (memcmp(chunk_id, "fmt ", 4) == 0) {

            if (chunk_size < 16) {
                ESP_LOGE(WAV_TAG, "Invalid fmt chunk");
                goto cleanup;
            }

            audio_format = wav_read_u16(file, &ok);
            channels = wav_read_u16(file, &ok);
            sample_rate = wav_read_u32(file, &ok);

            (void)wav_read_u32(file, &ok);

            block_align = wav_read_u16(file, &ok);
            bits_per_sample = wav_read_u16(file, &ok);

            if (!ok) {
                ESP_LOGE(WAV_TAG, "Cannot read fmt chunk");
                goto cleanup;
            }

            if (fseek(file, (long)(chunk_size - 16), SEEK_CUR) != 0) {
                ESP_LOGE(WAV_TAG, "Cannot skip fmt extension");
                goto cleanup;
            }

            fmt_found = true;
        }
        else if (memcmp(chunk_id, "data", 4) == 0) {

            data_size = chunk_size;
            data_found = true;
            break;
        }
        else {
            if (fseek(file, (long)chunk_size, SEEK_CUR) != 0) {
                ESP_LOGE(WAV_TAG, "Cannot skip WAV chunk");
                goto cleanup;
            }
        }

        if (chunk_size & 1) {
            if (fseek(file, 1, SEEK_CUR) != 0) {
                ESP_LOGE(WAV_TAG, "Cannot skip chunk padding");
                goto cleanup;
            }
        }
    }

    if (!fmt_found || !data_found) {
        ESP_LOGE(WAV_TAG, "WAV fmt/data chunk missing");
        goto cleanup;
    }

    ESP_LOGI(WAV_TAG, "WAV format:");
    ESP_LOGI(WAV_TAG, "  Sample rate: %" PRIu32, sample_rate);
    ESP_LOGI(WAV_TAG, "  Channels: %u", channels);
    ESP_LOGI(WAV_TAG, "  Bits: %u", bits_per_sample);
    ESP_LOGI(WAV_TAG, "  Data size: %" PRIu32, data_size);

    if (audio_format != 1 ||
        (channels != 1 && channels != 2) ||
        sample_rate != WAV_SAMPLE_RATE ||
        bits_per_sample != WAV_BITS_PER_SAMPLE ||
        block_align != channels * 2) {

        ESP_LOGE(WAV_TAG,
                 "Unsupported WAV format. "
                 "Expected PCM, 16kHz, 16-bit, mono/stereo.");
        goto cleanup;
    }

    if (data_size == 0 || data_size % block_align != 0) {
        ESP_LOGE(WAV_TAG, "Invalid WAV data size");
        goto cleanup;
    }

    int16_t input_buffer[WAV_BUFFER_FRAMES * 2];
    int16_t stereo_buffer[WAV_BUFFER_FRAMES * 2];

    uint32_t bytes_remaining = data_size;
    uint32_t frames_played = 0;

    ESP_LOGI(WAV_TAG, "Starting WAV playback: %s", path);

    ret = audio_set_output_volume(AUDIO_MUSIC_VOLUME);

    if (ret != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(PA_CTRL_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = ESP_OK;

    while (bytes_remaining > 0) {

        uint32_t frames = bytes_remaining / block_align;

        if (frames > WAV_BUFFER_FRAMES) {
            frames = WAV_BUFFER_FRAMES;
        }

        size_t input_bytes = frames * block_align;

        size_t bytes_read = fread(
            input_buffer,
            1,
            input_bytes,
            file
        );

        if (bytes_read != input_bytes) {
            ESP_LOGE(WAV_TAG, "WAV read error");
            ret = ESP_FAIL;
            break;
        }

        if (channels == 1) {
            for (uint32_t i = 0; i < frames; i++) {
                stereo_buffer[i * 2] = input_buffer[i];
                stereo_buffer[i * 2 + 1] = input_buffer[i];
            }
        } else {
            memcpy(stereo_buffer, input_buffer, input_bytes);
        }

        ret = esp_codec_dev_write(
            audio_codec,
            (void *)stereo_buffer,
            frames * 2 * sizeof(int16_t)
        );

        if (ret != ESP_OK) {
            ESP_LOGE(WAV_TAG, "Audio write failed: %s",
                     esp_err_to_name(ret));
            break;
        }

        bytes_remaining -= input_bytes;
        frames_played += frames;
    }

    gpio_set_level(PA_CTRL_GPIO, 0);

    if (ret == ESP_OK) {
        ESP_LOGI(WAV_TAG,
                 "Playback completed: %" PRIu32 " frames",
                 frames_played);
    }

cleanup:

    fclose(file);

    if (ret != ESP_OK) {
        ESP_LOGE(WAV_TAG, "WAV playback failed");
    }

    return ret;
}

// =====================================================
// MICROPHONE WAV RECORDING
// =====================================================

#define MIC_TAG                     "MIC"

#define MIC_RECORD_SAMPLE_RATE      16000
#define MIC_RECORD_CHANNELS         1
#define MIC_RECORD_BITS             16

#define MIC_RECORD_BUFFER_FRAMES    256

static void mic_write_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void mic_write_le32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
    p[2] = (uint8_t)((value >> 16) & 0xFF);
    p[3] = (uint8_t)((value >> 24) & 0xFF);
}

static esp_err_t mic_write_wav_header(
    FILE *file,
    uint32_t data_size)
{
    uint8_t header[44] = {0};

    uint32_t byte_rate =
        MIC_RECORD_SAMPLE_RATE *
        MIC_RECORD_CHANNELS *
        MIC_RECORD_BITS / 8;

    uint16_t block_align =
        MIC_RECORD_CHANNELS *
        MIC_RECORD_BITS / 8;

    memcpy(&header[0], "RIFF", 4);
    mic_write_le32(&header[4], 36 + data_size);

    memcpy(&header[8], "WAVE", 4);

    memcpy(&header[12], "fmt ", 4);
    mic_write_le32(&header[16], 16);

    mic_write_le16(&header[20], 1);
    mic_write_le16(&header[22], MIC_RECORD_CHANNELS);

    mic_write_le32(&header[24], MIC_RECORD_SAMPLE_RATE);
    mic_write_le32(&header[28], byte_rate);

    mic_write_le16(&header[32], block_align);
    mic_write_le16(&header[34], MIC_RECORD_BITS);

    memcpy(&header[36], "data", 4);
    mic_write_le32(&header[40], data_size);

    if (fseek(file, 0, SEEK_SET) != 0) {
        return ESP_FAIL;
    }

    if (fwrite(header, 1, sizeof(header), file) != sizeof(header)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

// =====================================================
// MICROPHONE RECORDING FUNCTION
// =====================================================

esp_err_t audio_record_wav_file(
    const char *path,
    uint32_t duration_ms)
{
    if (!audio_initialized || audio_codec == NULL) {
        ESP_LOGE(MIC_TAG, "Audio is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (path == NULL || duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(MIC_TAG, "================================");
    ESP_LOGI(MIC_TAG, "      MICROPHONE RECORDING");
    ESP_LOGI(MIC_TAG, "================================");

    ESP_LOGI(MIC_TAG, "Output: %s", path);
    ESP_LOGI(MIC_TAG, "Duration: %" PRIu32 " ms", duration_ms);
    ESP_LOGI(MIC_TAG, "Sample rate: %d Hz", MIC_RECORD_SAMPLE_RATE);
    ESP_LOGI(MIC_TAG, "Format: 16-bit PCM mono");
    ESP_LOGI(MIC_TAG, "Input gain: %.1f dB",
             (double)MIC_INPUT_GAIN_DB);

    FILE *file = fopen(path, "wb+");

    if (file == NULL) {
        ESP_LOGE(MIC_TAG, "Cannot create WAV file: %s", path);
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Disable speaker amplifier during recording.
    gpio_set_level(PA_CTRL_GPIO, 0);

    ret = mic_write_wav_header(file, 0);

    if (ret != ESP_OK) {
        ESP_LOGE(MIC_TAG, "Cannot write WAV header");
        fclose(file);
        return ret;
    }

    int16_t stereo_buffer[MIC_RECORD_BUFFER_FRAMES * 2];
    int16_t mono_buffer[MIC_RECORD_BUFFER_FRAMES];

    uint32_t total_frames =
        (MIC_RECORD_SAMPLE_RATE * duration_ms) / 1000;

    uint32_t frames_recorded = 0;
    uint32_t data_size = 0;

    int16_t left_min = INT16_MAX;
    int16_t left_max = INT16_MIN;
    int16_t right_min = INT16_MAX;
    int16_t right_max = INT16_MIN;

    uint64_t left_sum_sq = 0;
    uint64_t right_sum_sq = 0;

    int64_t left_sum = 0;
    int64_t right_sum = 0;

    uint32_t left_nonzero = 0;
    uint32_t right_nonzero = 0;
    uint32_t diagnostic_frames = 0;

    ESP_LOGI(MIC_TAG, "Recording started...");

    while (frames_recorded < total_frames) {

        uint32_t frames_this_block = MIC_RECORD_BUFFER_FRAMES;

        if (frames_recorded + frames_this_block > total_frames) {
            frames_this_block = total_frames - frames_recorded;
        }

        size_t input_bytes =
            frames_this_block * 2 * sizeof(int16_t);

        int read_ret = esp_codec_dev_read(
            audio_codec,
            stereo_buffer,
            input_bytes
        );

        if (read_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGE(MIC_TAG, "Microphone read failed: %d", read_ret);
            ret = ESP_FAIL;
            break;
        }

        // Analyse both channels.
        for (uint32_t i = 0; i < frames_this_block; i++) {

            int16_t left = stereo_buffer[i * 2];
            int16_t right = stereo_buffer[i * 2 + 1];

            if (left < left_min) left_min = left;
            if (left > left_max) left_max = left;

            if (right < right_min) right_min = right;
            if (right > right_max) right_max = right;

            left_sum += left;
            right_sum += right;

            left_sum_sq += (int64_t)left * left;
            right_sum_sq += (int64_t)right * right;

            if (left != 0) left_nonzero++;
            if (right != 0) right_nonzero++;
        }

        diagnostic_frames += frames_this_block;

        // Save left channel as mono WAV.
        for (uint32_t i = 0; i < frames_this_block; i++) {
            mono_buffer[i] = stereo_buffer[i * 2];
        }

        size_t output_bytes =
            frames_this_block * sizeof(int16_t);

        size_t bytes_written = fwrite(
            mono_buffer,
            1,
            output_bytes,
            file
        );

        if (bytes_written != output_bytes) {
            ESP_LOGE(MIC_TAG, "SD card write failed");
            ret = ESP_FAIL;
            break;
        }

        data_size += output_bytes;
        frames_recorded += frames_this_block;

        if ((frames_recorded % 16000) < frames_this_block) {
            ESP_LOGI(MIC_TAG,
                     "Recorded %" PRIu32 " / %" PRIu32 " frames",
                     frames_recorded,
                     total_frames);
        }
    }

    gpio_set_level(PA_CTRL_GPIO, 0);

    esp_err_t header_ret = mic_write_wav_header(file, data_size);

    if (header_ret != ESP_OK) {
        ESP_LOGE(MIC_TAG, "Failed to finalize WAV header");
        ret = ESP_FAIL;
    }

    // Microphone diagnostics.
    if (diagnostic_frames > 0) {

        double left_mean =
            (double)left_sum / diagnostic_frames;

        double right_mean =
            (double)right_sum / diagnostic_frames;

        // Remove DC offset before calculating AC RMS.
        double left_mean_sq =
            (double)left_sum_sq / diagnostic_frames;

        double right_mean_sq =
            (double)right_sum_sq / diagnostic_frames;

        double left_rms =
            sqrt(fmax(0.0, left_mean_sq - left_mean * left_mean));

        double right_rms =
            sqrt(fmax(0.0, right_mean_sq - right_mean * right_mean));

        ESP_LOGI(MIC_TAG,
                 "========== MIC SAMPLE DIAGNOSTIC ==========");

        ESP_LOGI(MIC_TAG,
                 "LEFT : min=%d max=%d nonzero=%" PRIu32,
                 left_min, left_max, left_nonzero);

        ESP_LOGI(MIC_TAG,
                 "RIGHT: min=%d max=%d nonzero=%" PRIu32,
                 right_min, right_max, right_nonzero);

        ESP_LOGI(MIC_TAG, "LEFT DC OFFSET : %.2f", left_mean);
        ESP_LOGI(MIC_TAG, "RIGHT DC OFFSET: %.2f", right_mean);

        ESP_LOGI(MIC_TAG, "LEFT AC RMS : %.2f", left_rms);
        ESP_LOGI(MIC_TAG, "RIGHT AC RMS: %.2f", right_rms);

        ESP_LOGI(MIC_TAG,
                 "Diagnostic frames: %" PRIu32,
                 diagnostic_frames);

        ESP_LOGI(MIC_TAG,
                 "===========================================");
    }

    fflush(file);
    fclose(file);

    if (ret == ESP_OK) {
        ESP_LOGI(MIC_TAG, "Recording completed!");
        ESP_LOGI(MIC_TAG, "Frames recorded: %" PRIu32,
                 frames_recorded);
        ESP_LOGI(MIC_TAG, "Audio data: %" PRIu32 " bytes",
                 data_size);
        ESP_LOGI(MIC_TAG, "WAV file: %s", path);
    } else {
        ESP_LOGE(MIC_TAG, "Recording failed");
    }

    return ret;
}
