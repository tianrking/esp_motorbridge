#include <inttypes.h>
#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "app/serial_cli.h"
#include "config/app_config.h"
#include "core/control_loop.h"
#include "core/motor_manager.h"
#include "core/safety_manager.h"
#include "net/wifi_udp_bridge.h"
#include "net/web_control.h"
#include "storage/param_store.h"
#include "transport/can_manager.h"
#include "vendors/damiao/damiao_protocol.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "motorbridge";

static void task_control(void *arg)
{
    const app_config_t *cfg = app_config_get();
    TickType_t period_ticks = pdMS_TO_TICKS(cfg->control_period_ms);
    if (period_ticks == 0) {
        period_ticks = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();
    const float dt_s = (float)cfg->control_period_ms / 1000.0f;

    while (1) {
        vTaskDelayUntil(&last_wake, period_ticks);
        control_loop_tick(dt_s);
    }
}

static void task_safety(void *arg)
{
    while (1) {
        safety_manager_tick();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void log_motor_cb(motor_state_t *m, void *ctx)
{
    (void)ctx;
    if (!m->online) {
        return;
    }
    ESP_LOGI(TAG,
             "id=%u online=%d en=%d mode=%d pos=%.3f vel=%.3f torq=%.3f cmd(pos=%.3f vel=%.3f tau=%.3f vlim=%.3f r=%.3f) kp=%.2f kd=%.3f st=0x%02X",
             m->id,
             m->online,
             m->enabled,
             (int)m->mode,
             m->position,
             m->speed,
             m->torque,
             m->cmd.pos,
             m->cmd.vel,
             m->cmd.tau,
             m->cmd.vlim,
             m->cmd.ratio,
             m->params.kp,
             m->params.kd,
             m->status);
}

static void task_telemetry(void *arg)
{
    const app_config_t *cfg = app_config_get();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(cfg->log_period_ms));
    }
}

static void apply_default_damiao_id_map(void)
{
    const app_config_t *cfg = app_config_get();
    if (!cfg->apply_default_id_map_at_boot) {
        return;
    }

    // default map: motor 1..7 -> feedback 0x11..0x17
    const int map_count = cfg->max_motors < 7 ? cfg->max_motors : 7;
    for (int i = 1; i <= map_count; ++i) {
        const uint8_t motor_id = (uint8_t)i;
        const uint8_t feedback_id = (uint8_t)(0x10 + i);

        twai_message_t tx = {0};
        damiao_build_register_write_cmd(motor_id, 7, (uint32_t)feedback_id, &tx);
        (void)can_manager_send(&tx, 10);

        damiao_build_register_write_cmd(motor_id, 8, (uint32_t)motor_id, &tx);
        (void)can_manager_send(&tx, 10);

        damiao_build_store_params_cmd(motor_id, &tx);
        (void)can_manager_send(&tx, 10);
    }

    ESP_LOGI(TAG, "default Damiao ID map applied for motors 1..%d", map_count);
}

void app_main(void)
{
    const app_config_t *cfg = app_config_get();
    const motor_vendor_ops_t *vendor = motor_vendor_active();

    ESP_LOGI(TAG, "MotorBridge startup vendor=%s", vendor->name);
    ESP_LOGI(TAG,
             "CAN tx=%d rx=%d bitrate=%d motors=%d ctrl_period=%dms tx_min=%dms offline_to=%dms default_kp=%.2f default_kd=%.3f",
             cfg->tx_gpio,
             cfg->rx_gpio,
             cfg->bitrate,
             cfg->max_motors,
             cfg->control_period_ms,
             cfg->min_tx_interval_ms,
             cfg->offline_timeout_ms,
             cfg->default_kp,
             cfg->default_kd);
    if (cfg->offline_timeout_ms < 1000) {
        ESP_LOGW(TAG, "offline timeout=%dms is shorter than typical Damiao feedback period", cfg->offline_timeout_ms);
    }

    motor_manager_init(cfg->max_motors);

    ESP_ERROR_CHECK(param_store_init());
    ESP_ERROR_CHECK(param_store_load_defaults_if_missing());

    ESP_ERROR_CHECK(can_manager_init(cfg->tx_gpio, cfg->rx_gpio, cfg->bitrate));
    can_manager_set_rx_callback(command_router_handle_can_rx);
    ESP_ERROR_CHECK(can_manager_start());
    apply_default_damiao_id_map();
    ESP_ERROR_CHECK(wifi_udp_bridge_start());
    ESP_ERROR_CHECK(web_control_start());
    ESP_ERROR_CHECK(serial_cli_start());

    xTaskCreatePinnedToCore(task_control, "control", 4096, NULL, 20, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_safety, "safety", 3072, NULL, 18, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_telemetry, "telemetry", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "startup scan disabled; use serial CLI 'scan <min> <max>' when needed");
}
