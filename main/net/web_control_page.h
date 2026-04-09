#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t web_control_root_get_handler(httpd_req_t *req);
esp_err_t web_control_favicon_get_handler(httpd_req_t *req);
