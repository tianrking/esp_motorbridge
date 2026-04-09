#include "net/web_actions_demo.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"

#include "net/web_control_demo.h"

esp_err_t web_actions_handle_demo(const char *action,
                                  const char *query,
                                  char *resp,
                                  size_t resp_len,
                                  bool *handled)
{
    *handled = false;

    if (strcmp(action, "demo_start") == 0 || strcmp(action, "demo1_start") == 0) {
        int demo_id = 1;
        if (strcmp(action, "demo_start") == 0) {
            char demo_id_s[8] = {0};
            if (httpd_query_key_value(query, "demo_id", demo_id_s, sizeof(demo_id_s)) == ESP_OK) {
                demo_id = atoi(demo_id_s);
            }
        }
        *handled = true;
        return web_control_demo_start(demo_id, resp, resp_len);
    }

    if (strcmp(action, "demo_status") == 0) {
        snprintf(resp, resp_len, "ok demo running=%d id=%d", web_control_demo_running() ? 1 : 0, web_control_demo_id());
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "demo_stop") == 0 || strcmp(action, "demo1_stop") == 0) {
        web_control_demo_request_stop();
        snprintf(resp, resp_len, "ok demo stopping");
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "demo_reset") == 0) {
        *handled = true;
        return web_control_demo_reset(resp, resp_len);
    }

    return ESP_OK;
}
