#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

bool web_control_demo_running(void);
int web_control_demo_id(void);

esp_err_t web_control_demo_start(int demo_id, char *resp, size_t resp_len);
void web_control_demo_request_stop(void);
void web_control_demo_wait_stopped(int timeout_ms);
esp_err_t web_control_demo_reset(char *resp, size_t resp_len);
