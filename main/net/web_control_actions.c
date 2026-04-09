#include "net/web_control_actions.h"

#include <stdio.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_timer.h"

#include "config/app_config.h"
#include "net/web_actions_demo.h"
#include "net/web_actions_id_vendor.h"
#include "net/web_actions_motor.h"
#include "net/web_control_demo.h"
#include "net/web_control_dispatch.h"

static char s_last_query[256];
static int64_t s_last_query_us = 0;
static const int64_t WEB_DUPLICATE_GUARD_US = 800000;

static void remember_query(const char *query)
{
    strlcpy(s_last_query, query, sizeof(s_last_query));
    s_last_query_us = esp_timer_get_time();
}

static bool is_duplicate_query(const char *query)
{
    if (query == NULL) {
        return false;
    }
    int64_t now_us = esp_timer_get_time();
    if (s_last_query_us <= 0) {
        return false;
    }
    if ((now_us - s_last_query_us) >= WEB_DUPLICATE_GUARD_US) {
        return false;
    }
    return strncmp(query, s_last_query, sizeof(s_last_query)) == 0;
}

static bool is_blocked_while_demo_running(const char *action)
{
    return strcmp(action, "enable") == 0 ||
           strcmp(action, "disable") == 0 ||
           strcmp(action, "clear_error") == 0 ||
           strcmp(action, "set_zero") == 0 ||
           strcmp(action, "set_gains") == 0 ||
           strcmp(action, "set_vendor") == 0 ||
           strcmp(action, "set_mode") == 0 ||
           strcmp(action, "apply") == 0;
}

esp_err_t web_control_execute_action_from_query(const char *query, char *resp, size_t resp_len)
{
    char action[24] = {0};
    char ids_csv[96] = {0};
    uint8_t ids[32] = {0};

    if (httpd_query_key_value(query, "action", action, sizeof(action)) != ESP_OK) {
        snprintf(resp, resp_len, "err missing action");
        return ESP_FAIL;
    }

    bool handled = false;
    esp_err_t err = web_actions_handle_demo(action, query, resp, resp_len, &handled);
    if (handled) {
        if (err == ESP_OK) {
            remember_query(query);
        }
        return err;
    }

    err = web_actions_handle_id_vendor_pre_ids(action, query, resp, resp_len, &handled);
    if (handled) {
        if (err == ESP_OK) {
            remember_query(query);
        }
        return err;
    }

    if (httpd_query_key_value(query, "ids", ids_csv, sizeof(ids_csv)) != ESP_OK) {
        snprintf(resp, resp_len, "err missing ids");
        return ESP_FAIL;
    }

    if (is_duplicate_query(query)) {
        snprintf(resp, resp_len, "ok duplicate ignored");
        return ESP_OK;
    }

    int n = web_control_parse_ids(ids_csv, ids, (int)(sizeof(ids) / sizeof(ids[0])));
    const app_config_t *cfg = app_config_get();
    int max_id = cfg->max_motors;
    int filtered = 0;
    for (int i = 0; i < n; ++i) {
        if (ids[i] <= max_id) {
            ids[filtered++] = ids[i];
        }
    }
    n = filtered;

    if (n <= 0 && strcmp(action, "estop") != 0) {
        snprintf(resp, resp_len, "err invalid ids");
        return ESP_FAIL;
    }

    if (web_control_demo_running() && is_blocked_while_demo_running(action)) {
        snprintf(resp, resp_len, "err demo running, stop demo first");
        return ESP_FAIL;
    }

    err = web_actions_handle_id_vendor_post_ids(action, query, ids, n, resp, resp_len, &handled);
    if (handled) {
        if (err == ESP_OK) {
            remember_query(query);
        }
        return err;
    }

    err = web_actions_handle_motor(action, query, ids, n, max_id, resp, resp_len, &handled);
    if (handled) {
        if (err == ESP_OK) {
            remember_query(query);
        }
        return err;
    }

    snprintf(resp, resp_len, "err unknown action");
    return ESP_FAIL;
}
