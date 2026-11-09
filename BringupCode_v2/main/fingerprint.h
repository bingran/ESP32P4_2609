#pragma once
#include "esp_err.h"

/* Start non-destructive R503 diagnostics. ESP_OK means task/UART ready. */
esp_err_t fingerprint_init(void);
