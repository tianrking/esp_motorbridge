#include "control_loop.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "control_loop";
static int64_t s_last_tx_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

void control_loop_tick(float dt_s)
{
    (void)dt_s;
    const app_config_t *cfg = app_config_get();
    const int max_id = motor_manager_count();

    const int64_t now_us = esp_timer_get_time();
    const int64_t min_interval_us = (int64_t)cfg->min_tx_interval_ms * 1000;
    for (int id = MOTORBRIDGE_MIN_MOTOR_ID; id <= max_id; ++id) {
        motor_state_t m = {0};
        if (!motor_manager_get_state((uint8_t)id, &m)) {
            continue;
        }
        if (!m.enabled || m.mode == MOTOR_MODE_DISABLED || m.vendor == NULL || m.vendor->build_control_frame == NULL) {
            continue;
        }

        if (min_interval_us > 0 && (now_us - s_last_tx_us[m.id]) < min_interval_us) {
            continue;
        }

        twai_message_t tx = {0};
        if (m.vendor->build_control_frame(m.id, m.mode, &m.cmd, &m.params, &tx)) {
            (void)can_manager_send(&tx, 8);
            s_last_tx_us[m.id] = now_us;
        }
    }

    ESP_LOGD(TAG, "control tick dt=%.4f", dt_s);
}
