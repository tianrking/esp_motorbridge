#include "control_loop.h"

#include "esp_timer.h"
#include "esp_log.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

typedef struct {
    float dt_s;
} control_ctx_t;

static const char *TAG = "control_loop";
static int64_t s_last_tx_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

static void control_one(motor_state_t *m, void *ctx)
{
    (void)ctx;
    const app_config_t *cfg = app_config_get();

    if (!m->enabled || m->mode == MOTOR_MODE_DISABLED || m->vendor == NULL) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t min_interval_us = (int64_t)cfg->min_tx_interval_ms * 1000;
    if (min_interval_us > 0 && (now_us - s_last_tx_us[m->id]) < min_interval_us) {
        return;
    }

    if (m->vendor->build_control_frame != NULL) {
        twai_message_t tx;
        if (m->vendor->build_control_frame(m->id, m->mode, &m->cmd, &m->params, &tx)) {
            (void)can_manager_send(&tx, 8);
            s_last_tx_us[m->id] = now_us;
        }
    }
}

void control_loop_tick(float dt_s)
{
    control_ctx_t ctx = {
        .dt_s = dt_s,
    };
    motor_manager_for_each(control_one, &ctx);
    ESP_LOGD(TAG, "control tick dt=%.4f", dt_s);
}
