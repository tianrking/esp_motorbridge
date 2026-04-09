#include "net/web_actions_motor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "net/web_control_dispatch.h"

typedef struct {
    char *resp;
    size_t cap;
    size_t used;
    int count;
    int max_id;
    bool full;
} state_emit_ctx_t;

typedef struct {
    bool valid;
    bool online;
    uint8_t mode;
    float pos;
    float vel;
    float tau;
} state_cache_t;

static state_cache_t s_delta_cache[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static uint32_t s_delta_seq = 0;

static bool state_changed(const state_cache_t *c, const motor_state_t *m)
{
    if (c == NULL || m == NULL || !c->valid) {
        return true;
    }
    const float eps = 0.001f;
    if (c->online != m->online || c->mode != (uint8_t)m->mode) {
        return true;
    }
    if (fabsf(c->pos - m->position) > eps || fabsf(c->vel - m->speed) > eps || fabsf(c->tau - m->torque) > eps) {
        return true;
    }
    return false;
}

static void update_cache(uint8_t id, const motor_state_t *m)
{
    if (id > MOTORBRIDGE_MAX_MOTOR_ID || m == NULL) {
        return;
    }
    s_delta_cache[id].valid = true;
    s_delta_cache[id].online = m->online;
    s_delta_cache[id].mode = (uint8_t)m->mode;
    s_delta_cache[id].pos = m->position;
    s_delta_cache[id].vel = m->speed;
    s_delta_cache[id].tau = m->torque;
}

static void append_state_item(state_emit_ctx_t *c, const motor_state_t *m)
{
    if (c == NULL || m == NULL || c->used >= c->cap) {
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

static void append_state_item_cb(motor_state_t *m, void *ctx)
{
    state_emit_ctx_t *c = (state_emit_ctx_t *)ctx;
    if (m == NULL || c == NULL) {
        return;
    }
    if ((int)m->id > c->max_id) {
        return;
    }

    bool include = c->full;
    if (!include) {
        include = state_changed(&s_delta_cache[m->id], m);
    }
    if (include) {
        append_state_item(c, m);
    }
    update_cache(m->id, m);
}

static esp_err_t handle_state_all(int max_id, char *resp, size_t resp_len, bool *handled)
{
    int hdr = snprintf(resp, resp_len, "ok state_all ");
    if (hdr < 0 || (size_t)hdr >= resp_len) {
        snprintf(resp, resp_len, "err state_all overflow");
        *handled = true;
        return ESP_FAIL;
    }
    state_emit_ctx_t ctx = {
        .resp = resp,
        .cap = resp_len,
        .used = (size_t)hdr,
        .count = 0,
        .max_id = max_id,
        .full = true,
    };
    motor_manager_for_each(append_state_item_cb, &ctx);
    snprintf(resp + ctx.used, resp_len - ctx.used, " (count=%d)", ctx.count);
    *handled = true;
    return ESP_OK;
}

static esp_err_t handle_state_delta(const char *query, int max_id, char *resp, size_t resp_len, bool *handled)
{
    char full_s[8] = {0};
    char reset_s[8] = {0};
    bool full = (httpd_query_key_value(query, "full", full_s, sizeof(full_s)) == ESP_OK && atoi(full_s) > 0);
    bool reset = (httpd_query_key_value(query, "reset", reset_s, sizeof(reset_s)) == ESP_OK && atoi(reset_s) > 0);

    if (reset) {
        memset(s_delta_cache, 0, sizeof(s_delta_cache));
    }

    int hdr = snprintf(resp, resp_len, "ok state_delta seq=%lu ", (unsigned long)(++s_delta_seq));
    if (hdr < 0 || (size_t)hdr >= resp_len) {
        snprintf(resp, resp_len, "err state_delta overflow");
        *handled = true;
        return ESP_FAIL;
    }
    state_emit_ctx_t ctx = {
        .resp = resp,
        .cap = resp_len,
        .used = (size_t)hdr,
        .count = 0,
        .max_id = max_id,
        .full = full,
    };
    motor_manager_for_each(append_state_item_cb, &ctx);
    if (ctx.count == 0) {
        snprintf(resp + ctx.used, resp_len - ctx.used, "none (count=0)");
    } else {
        snprintf(resp + ctx.used, resp_len - ctx.used, " (count=%d)", ctx.count);
    }
    *handled = true;
    return ESP_OK;
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
        return handle_state_all(max_id, resp, resp_len, handled);
    }

    if (strcmp(action, "state_delta") == 0) {
        return handle_state_delta(query, max_id, resp, resp_len, handled);
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
