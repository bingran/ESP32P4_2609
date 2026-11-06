#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialize ES8311 audio playback.
 */
esp_err_t audio_init(void);

/**
 * @brief Play a sine-wave test tone.
 *
 * @param frequency_hz Tone frequency in Hz.
 * @param duration_ms Playback duration in milliseconds.
 */
esp_err_t audio_play_test_tone(
    uint32_t frequency_hz,
    uint32_t duration_ms
);

/**
 * @brief Get the shared I2C master bus handle.
 *
 * Used by other devices sharing GPIO7/GPIO8.
 */
i2c_master_bus_handle_t audio_get_i2c_bus_handle(void);

/**
 * @brief Play a PCM WAV file from the SD card.
 *
 * Supported formats:
 * - PCM (uncompressed)
 * - 16 kHz
 * - 16-bit
 * - Mono or stereo
 *
 * Mono audio is duplicated to both speaker channels.
 *
 * @param path Full path, e.g. "/sdcard/TEST.WAV".
 */
esp_err_t audio_play_wav_file(const char *path);
esp_err_t audio_record_wav_file(const char *path, uint32_t duration_ms);

#endif // AUDIO_H