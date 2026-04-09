#pragma once

#include <stddef.h>

#include "esp_err.h"

esp_err_t web_control_execute_action_from_query(const char *query, char *resp, size_t resp_len);
