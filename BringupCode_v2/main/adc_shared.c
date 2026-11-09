#include "adc_shared.h"
#include "soc/soc_caps.h"

static adc_oneshot_unit_handle_t s_units[SOC_ADC_PERIPH_NUM];

esp_err_t adc_shared_get_unit(adc_unit_t unit, adc_oneshot_unit_handle_t *handle)
{
    if (!handle || (unsigned)unit >= SOC_ADC_PERIPH_NUM) return ESP_ERR_INVALID_ARG;
    if (!s_units[unit]) {
        const adc_oneshot_unit_init_cfg_t config = {.unit_id = unit};
        esp_err_t ret = adc_oneshot_new_unit(&config, &s_units[unit]);
        if (ret != ESP_OK) return ret;
    }
    *handle = s_units[unit];
    return ESP_OK;
}
