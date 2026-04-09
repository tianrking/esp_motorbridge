#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

esp_err_t web_actions_handle_demo(const char *action,
                                  const char *query,
                                  char *resp,
                                  size_t resp_len,
                                  bool *handled);
