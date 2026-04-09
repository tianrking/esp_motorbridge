#include "net/web_actions_id_vendor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/motor_manager.h"
#include "net/web_control_demo.h"
#include "net/web_control_dispatch.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

esp_err_t web_actions_handle_id_vendor_pre_ids(const char *action,
                                                const char *query,
                                                char *resp,
                                                size_t resp_len,
                                                bool *handled)
{
    *handled = false;

    if (strcmp(action, "set_id") == 0) {
        char old_s[8] = {0};
        char new_s[8] = {0};
        if (web_control_demo_running()) {
            snprintf(resp, resp_len, "err stop demo first");
            *handled = true;
            return ESP_FAIL;
        }
        if (httpd_query_key_value(query, "id_old", old_s, sizeof(old_s)) != ESP_OK ||
            httpd_query_key_value(query, "id_new", new_s, sizeof(new_s)) != ESP_OK) {
            snprintf(resp, resp_len, "err missing id_old/id_new");
            *handled = true;
            return ESP_FAIL;
        }

        int old_id = atoi(old_s);
        int new_id = atoi(new_s);
        if (old_id < MOTORBRIDGE_MIN_MOTOR_ID || old_id > motor_manager_count() ||
            new_id < MOTORBRIDGE_MIN_MOTOR_ID || new_id > motor_manager_count() ||
            old_id == new_id) {
            snprintf(resp, resp_len, "err bad id");
            *handled = true;
            return ESP_FAIL;
        }

        motor_state_t old_m = {0};
        if (!motor_manager_get_state((uint8_t)old_id, &old_m) ||
            old_m.vendor == NULL ||
            old_m.vendor->name == NULL ||
            strcmp(old_m.vendor->name, "damiao") != 0) {
            snprintf(resp, resp_len, "err old id is not damiao");
            *handled = true;
            return ESP_FAIL;
        }

        motor_state_t new_m = {0};
        if (motor_manager_get_state((uint8_t)new_id, &new_m) && new_m.online) {
            snprintf(resp, resp_len, "err new id online");
            *handled = true;
            return ESP_FAIL;
        }

        web_control_dispatch_admin((uint8_t)old_id, 5, 0);
        vTaskDelay(pdMS_TO_TICKS(30));
        web_control_dispatch_set_ids((uint8_t)old_id, (uint8_t)new_id, true, true);
        vTaskDelay(pdMS_TO_TICKS(40));
        (void)motor_manager_set_vendor((uint8_t)new_id, "damiao");
        can_manager_trigger_scan(new_id, new_id);

        snprintf(resp, resp_len, "ok set_id %d->%d requested", old_id, new_id);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "enable_all") == 0) {
        web_control_dispatch_admin(0, 4, 0);
        snprintf(resp, resp_len, "ok enable_all");
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "disable_all") == 0) {
        web_control_dispatch_admin(0, 5, 0);
        snprintf(resp, resp_len, "ok disable_all");
        *handled = true;
        return ESP_OK;
    }

    return ESP_OK;
}

esp_err_t web_actions_handle_id_vendor_post_ids(const char *action,
                                                 const char *query,
                                                 const uint8_t *ids,
                                                 int n,
                                                 char *resp,
                                                 size_t resp_len,
                                                 bool *handled)
{
    *handled = false;

    if (strcmp(action, "set_vendor") != 0) {
        return ESP_OK;
    }

    char vendor[16] = {0};
    if (!web_control_parse_vendor_name(query, vendor, sizeof(vendor))) {
        snprintf(resp, resp_len, "err bad vendor");
        *handled = true;
        return ESP_FAIL;
    }

    int ok_count = 0;
    for (int i = 0; i < n; ++i) {
        if (motor_manager_set_vendor(ids[i], vendor)) {
            ok_count++;
        }
        web_control_pace_then_next(i, n);
    }

    snprintf(resp, resp_len, "ok set_vendor=%s count=%d", vendor, ok_count);
    *handled = true;
    return ESP_OK;
}
