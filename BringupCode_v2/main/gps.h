#pragma once
#include "esp_err.h"

/* Start UART reception. ESP_OK confirms task/UART setup, not a satellite fix. */
esp_err_t gps_init(void);
