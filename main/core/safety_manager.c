#include "safety_manager.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

static const char *TAG = "safety_manager";
static bool s_offline_estop_latched = false;
// Burn-in policy: keep running even when some motors timeout.
static const bool s_offline_estop_enabled = false;
static bool s_logged_estop_disabled = false;

typedef struct {
    int64_t now_us;
    int32_t timeout_ms;
    uint8_t ids[MOTORBRIDGE_MAX_MOTOR_ID + 1];
    int count;
} stale_ctx_t;

static void collect_stale_cb(motor_state_t *m, void *ctx)
{
    stale_ctx_t *c = (stale_ctx_t *)ctx;
    if (m == NULL || c == NULL || !m->online || m->last_seen_us <= 0) {
        return;
    }
    // Only "participating" motors are considered for offline protection.
    if (!m->enabled || m->mode == MOTOR_MODE_DISABLED) {
        return;
    }
    int64_t age_us = c->now_us - m->last_seen_us;
    if (age_us > ((int64_t)c->timeout_ms * 1000LL)) {
        if (c->count < (int)(sizeof(c->ids) / sizeof(c->ids[0]))) {
            c->ids[c->count++] = m->id;
        }
    }
}

void safety_manager_tick(void)
{
    const app_config_t *cfg = app_config_get();
    if (cfg == NULL || cfg->offline_timeout_ms <= 0) {
        return;
    }

    stale_ctx_t stale = {
        .now_us = esp_timer_get_time(),
        .timeout_ms = cfg->offline_timeout_ms,
    };
    motor_manager_for_each(collect_stale_cb, &stale);

    if (stale.count > 0 && s_offline_estop_enabled) {
        if (!s_offline_estop_latched) {
            s_offline_estop_latched = true;
            ESP_LOGW(TAG, "offline protection triggered, E-STOP ALL (offline_count=%d)", stale.count);
        }
    } else if (s_offline_estop_latched) {
        s_offline_estop_latched = false;
        ESP_LOGI(TAG, "offline protection latch cleared");
    }

    if (!s_offline_estop_enabled && !s_logged_estop_disabled) {
        ESP_LOGW(TAG, "offline E-STOP disabled for burn-in; timeout only marks offline");
        s_logged_estop_disabled = true;
    }

    motor_manager_mark_offline_by_timeout(stale.now_us, stale.timeout_ms);
}
