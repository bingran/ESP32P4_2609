#pragma once

#include "esp_adc/adc_oneshot.h"

// Initialize channels sequentially during startup, before starting readers.
esp_err_t adc_shared_get_unit(adc_unit_t unit, adc_oneshot_unit_handle_t *handle);
