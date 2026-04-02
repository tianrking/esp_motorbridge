#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t param_store_init(void);
esp_err_t param_store_load_defaults_if_missing(void);

#ifdef __cplusplus
}
#endif
