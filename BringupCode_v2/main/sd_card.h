#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t sd_card_init(void);
esp_err_t sd_card_test(void);
esp_err_t sd_card_deinit(void);
bool sd_card_is_mounted(void);

#endif // SD_CARD_H