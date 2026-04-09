#include "net/web_actions_motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"

#include "core/motor_manager.h"
#include "net/web_control_dispatch.h"

typedef struct {
    char *resp;
    size_t cap;
    size_t used;
    int count;
    int max_id;
} state_all_ctx_t;

static void append_state_item_cb(motor_state_t *m, void *ctx)
{
    state_all_ctx_t *c = (state_all_ctx_t *)ctx;
    if (m == NULL || c == NULL || c->used >= c->cap) {
        return;
    }
    if ((int)m->id > c->max_id) {
        return;
    }

    int n = snprintf(c->resp + c->used,
                     c->cap - c->used,
                     "%sid=%u,online=%d,mode=%d,pos=%.5f,vel=%.5f,tau=%.5f",
                     (c->count > 0) ? ";" : "",
                     (unsigned int)m->id,
                     m->online ? 1 : 0,
                     (int)m->mode,
                     m->position,
                     m->speed,
                     m->torque);
    if (n <= 0) {
        return;
    }
    size_t step = (size_t)n;
    if (step >= (c->cap - c->used)) {
        c->used = c->cap - 1;
        c->resp[c->used] = '\0';
        return;
    }
    c->used += step;
    c->count++;
}

esp_err_t web_actions_handle_motor(const char *action,
                                   const char *query,
                                   const uint8_t *ids,
                                   int n,
                                   int max_id,
                                   char *resp,
                                   size_t resp_len,
                                   bool *handled)
{
    *handled = false;

    if (strcmp(action, "state_all") == 0) {
        int hdr = snprintf(resp, resp_len, "ok state_all ");
        if (hdr < 0 || (size_t)hdr >= resp_len) {
            snprintf(resp, resp_len, "err state_all overflow");
            *handled = true;
            return ESP_FAIL;
        }
        state_all_ctx_t ctx = {
            .resp = resp,
            .cap = resp_len,
            .used = (size_t)hdr,
            .count = 0,
            .max_id = max_id,
        };
        motor_manager_for_each(append_state_item_cb, &ctx);
        snprintf(resp + ctx.used, resp_len - ctx.used, " (count=%d)", ctx.count);
        *handled = true;
        return ESP_OK;
    }

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
