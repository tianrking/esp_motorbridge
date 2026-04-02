#include "safety_manager.h"

#include "esp_timer.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

void safety_manager_tick(void)
{
    const app_config_t *cfg = app_config_get();
    int64_t now_us = esp_timer_get_time();
    motor_manager_mark_offline_by_timeout(now_us, cfg->offline_timeout_ms);
}
