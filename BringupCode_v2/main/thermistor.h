
#pragma once

#include "esp_err.h"

typedef struct {
    int raw;
    int voltage_mv;
    float resistance_ohm;
    float temperature_c;
} thermistor_reading_t;

esp_err_t thermistor_init(void);

esp_err_t thermistor_read(thermistor_reading_t *reading);
