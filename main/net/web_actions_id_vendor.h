#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t web_actions_handle_id_vendor_pre_ids(const char *action,
                                                const char *query,
                                                char *resp,
                                                size_t resp_len,
                                                bool *handled);

esp_err_t web_actions_handle_id_vendor_post_ids(const char *action,
                                                 const char *query,
                                                 const uint8_t *ids,
                                                 int n,
                                                 char *resp,
                                                 size_t resp_len,
                                                 bool *handled);
