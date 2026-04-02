#include "param_store.h"

#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "param_store";

esp_err_t param_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t param_store_load_defaults_if_missing(void)
{
    ESP_LOGI(TAG, "NVS ready, default motor params loaded by motor_manager");
    return ESP_OK;
}
