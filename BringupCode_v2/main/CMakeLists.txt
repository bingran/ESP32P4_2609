idf_component_register(
    SRCS
        "main.c"
        "thermistor.c"
        "sd_card.c"

    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        esp_adc
        esp_driver_gpio
        esp_driver_sdmmc
        sdmmc
        fatfs
        spi_flash
)
