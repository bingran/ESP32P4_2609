#pragma once

#include "esp_err.h"

typedef struct {
    int raw;
    int adc_mv;
    int battery_mv;
} battery_reading_t;

// GPIO49 must be connected to the battery divider.
// Shares ADC_UNIT_2 with the GPIO50 thermistor through adc_shared.
esp_err_t battery_init(void);
esp_err_t battery_read(battery_reading_t *reading);
