idf_component_register(
    SRCS
        "main.c"
        "thermistor.c"
        "sd_card.c"
        "imu.c"
        "audio.c"

    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        esp_adc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_sdmmc
        esp_codec_dev
        spi_flash
        sdmmc
        fatfs
)