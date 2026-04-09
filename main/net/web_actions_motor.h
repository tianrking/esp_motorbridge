#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t web_actions_handle_motor(const char *action,
                                   const char *query,
                                   const uint8_t *ids,
                                   int n,
                                   int max_id,
                                   char *resp,
                                   size_t resp_len,
                                   bool *handled);
