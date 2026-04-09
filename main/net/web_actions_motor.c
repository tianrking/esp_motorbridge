#include "net/web_actions_motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"

#include "core/motor_manager.h"
#include "net/web_control_dispatch.h"

esp_err_t web_actions_handle_motor(const char *action,
                                   const char *query,
                                   const uint8_t *ids,
                                   int n,
                                   int max_id,
                                   char *resp,
                                   size_t resp_len,
                                   bool *handled)
{
    (void)max_id;
    *handled = false;

    if (strcmp(action, "enable") == 0) {
        web_control_log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_admin(ids[i], 4, 0);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok enable count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "disable") == 0) {
        web_control_log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_admin(ids[i], 5, 0);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok disable count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "clear_error") == 0) {
        web_control_log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_admin(ids[i], 7, 0);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok clear_error count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "set_zero") == 0) {
        web_control_log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_admin(ids[i], 6, 0);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok set_zero count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "estop") == 0) {
        web_control_log_action(action, ids, n, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        web_control_dispatch_admin(0, 2, 0);
        snprintf(resp, resp_len, "ok estop");
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "state") == 0) {
        uint8_t id = ids[0];
        motor_state_t m;
        if (!motor_manager_get_state(id, &m)) {
            snprintf(resp, resp_len, "err state not found");
            *handled = true;
            return ESP_FAIL;
        }
        snprintf(resp,
                 resp_len,
                 "ok state id=%u pos=%.5f vel=%.5f online=%d mode=%d",
                 (unsigned int)id,
                 m.position,
                 m.speed,
                 m.online ? 1 : 0,
                 (int)m.mode);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "set_gains") == 0) {
        float kp = 0.0f;
        float kd = 0.0f;
        (void)web_control_parse_float(query, "kp", &kp);
        (void)web_control_parse_float(query, "kd", &kd);
        web_control_log_action(action, ids, n, 0, kp, kd, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_set_gains(ids[i], kp, kd);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok set_gains count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    char mode_s[8] = {0};
    if (httpd_query_key_value(query, "mode", mode_s, sizeof(mode_s)) != ESP_OK) {
        return ESP_OK;
    }
    int mode = atoi(mode_s);
    if (mode < MOTOR_MODE_MIT || mode > MOTOR_MODE_FORCE_POS) {
        snprintf(resp, resp_len, "err bad mode");
        *handled = true;
        return ESP_FAIL;
    }

    if (strcmp(action, "set_mode") == 0) {
        web_control_log_action(action, ids, n, mode, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n; ++i) {
            web_control_dispatch_admin(ids[i], 1, (uint8_t)mode);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok set_mode count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    if (strcmp(action, "apply") == 0) {
        float pos = 0.0f;
        float vel = 0.0f;
        float tau = 0.0f;
        float vlim = 0.0f;
        float ratio = 0.1f;
        (void)web_control_parse_float(query, "pos", &pos);
        (void)web_control_parse_float(query, "vel", &vel);
        (void)web_control_parse_float(query, "tau", &tau);
        (void)web_control_parse_float(query, "vlim", &vlim);
        (void)web_control_parse_float(query, "ratio", &ratio);
        if ((mode == MOTOR_MODE_POS_VEL || mode == MOTOR_MODE_FORCE_POS) && vlim <= 0.0f) {
            snprintf(resp, resp_len, "err vlim must be > 0 in this mode");
            *handled = true;
            return ESP_FAIL;
        }
        web_control_log_action(action, ids, n, mode, pos, vel, tau, vlim, ratio);

        for (int i = 0; i < n; ++i) {
            web_control_dispatch_mode(ids[i], (uint8_t)mode, pos, vel, tau, vlim, ratio);
            web_control_pace_then_next(i, n);
        }
        snprintf(resp, resp_len, "ok apply count=%d", n);
        *handled = true;
        return ESP_OK;
    }

    return ESP_OK;
}
