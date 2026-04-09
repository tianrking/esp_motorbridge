#include "app/command_router.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router_internal.h"
#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "vendors/damiao/damiao_protocol.h"

reg_cache_t g_cmd_reg_cache[256];
bool g_cmd_verify_pending = false;
uint8_t g_cmd_verify_expect_motor_id = 0;
uint8_t g_cmd_verify_expect_feedback_id = 0;
int64_t g_cmd_seen_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
uint8_t g_cmd_damiao_mode_written[MOTORBRIDGE_MAX_MOTOR_ID + 1];
int64_t g_cmd_damiao_mode_written_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

void command_router_handle_can_rx(const twai_message_t *msg)
{
    if (command_router_try_handle_register_reply(msg)) {
        return;
    }

    host_admin_cmd_t admin;
    if (host_protocol_parse_admin_cmd(msg, &admin)) {
        command_router_apply_admin_cmd(&admin);
        return;
    }

    uint8_t id = 0;
    motor_mode_t mode = MOTOR_MODE_DISABLED;
    damiao_cmd_t cmd = {0};
    if (host_protocol_parse_mode_frame(msg, &id, &mode, &cmd)) {
        if (id >= MOTORBRIDGE_MIN_MOTOR_ID && id <= motor_manager_count()) {
            command_router_apply_mode_cmd(id, mode, &cmd);
        }
        return;
    }

    (void)command_router_try_handle_feedback(msg);
}

int command_router_collect_seen_ids(int min_id, int max_id, uint8_t *out_ids, int max_ids, int32_t recent_ms)
{
    if (out_ids == NULL || max_ids <= 0) {
        return 0;
    }
    if (min_id < MOTORBRIDGE_MIN_MOTOR_ID) {
        min_id = MOTORBRIDGE_MIN_MOTOR_ID;
    }
    if (max_id > MOTORBRIDGE_MAX_MOTOR_ID) {
        max_id = MOTORBRIDGE_MAX_MOTOR_ID;
    }
    if (min_id > max_id) {
        return 0;
    }

    int count = 0;
    int64_t now_us = esp_timer_get_time();
    int64_t window_us = (int64_t)recent_ms * 1000;
    for (int i = min_id; i <= max_id && count < max_ids; ++i) {
        if (g_cmd_seen_us[i] > 0 && (now_us - g_cmd_seen_us[i]) <= window_us) {
            out_ids[count++] = (uint8_t)i;
        }
    }
    return count;
}

bool command_router_wait_feedback(uint8_t id, int32_t timeout_ms)
{
    if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > MOTORBRIDGE_MAX_MOTOR_ID || timeout_ms <= 0) {
        return false;
    }

    int64_t start_seen = g_cmd_seen_us[id];
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline_us) {
        if (g_cmd_seen_us[id] > start_seen) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}
