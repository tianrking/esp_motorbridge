#include "safety_manager.h"

#include <stdint.h>

#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "app/command_router.h"
#include "config/app_config.h"
#include "core/motor_manager.h"

static const char *TAG = "safety_manager";
static bool s_offline_estop_latched = false;

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
    int64_t age_us = c->now_us - m->last_seen_us;
    if (age_us > ((int64_t)c->timeout_ms * 1000LL)) {
        if (c->count < (int)(sizeof(c->ids) / sizeof(c->ids[0]))) {
            c->ids[c->count++] = m->id;
        }
    }
}

static void send_estop_all(void)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = 2; // HOST_OP_ESTOP
    msg.data[1] = 0;
    msg.data[2] = 0;
    command_router_handle_can_rx(&msg);
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

    if (stale.count > 0) {
        if (!s_offline_estop_latched) {
            send_estop_all();
            s_offline_estop_latched = true;
            ESP_LOGW(TAG, "offline protection triggered, E-STOP ALL (offline_count=%d)", stale.count);
        }
    } else if (s_offline_estop_latched) {
        s_offline_estop_latched = false;
        ESP_LOGI(TAG, "offline protection latch cleared");
    }

    motor_manager_mark_offline_by_timeout(stale.now_us, stale.timeout_ms);
}
