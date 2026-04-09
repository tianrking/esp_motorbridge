#include "app/command_router_internal.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/damiao/damiao_protocol.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "cmd_router";

static void send_damiao_reg_write(uint8_t id, uint8_t rid, uint32_t value)
{
    twai_message_t tx;
    damiao_build_register_write_cmd(id, rid, value, &tx);
    (void)can_manager_send(&tx, 10);
}

static void send_damiao_reg_read(uint8_t id, uint8_t rid)
{
    twai_message_t tx;
    damiao_build_register_read_cmd(id, rid, &tx);
    (void)can_manager_send(&tx, 10);
}

void command_router_ensure_damiao_ctrl_mode(uint8_t id, motor_mode_t mode)
{
    if (mode < MOTOR_MODE_MIT || mode > MOTOR_MODE_FORCE_POS) {
        return;
    }

    motor_state_t m;
    if (!motor_manager_get_state(id, &m) || m.vendor == NULL || m.vendor->name == NULL) {
        return;
    }
    if (strcmp(m.vendor->name, "damiao") != 0) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    const uint8_t desired = (uint8_t)mode;
    const bool same_mode = g_cmd_damiao_mode_written[id] == desired;
    const bool too_soon = (now - g_cmd_damiao_mode_written_us[id]) < 300000;
    if (same_mode && too_soon) {
        return;
    }

    send_damiao_reg_write(id, 10, (uint32_t)desired);
    g_cmd_damiao_mode_written[id] = desired;
    g_cmd_damiao_mode_written_us[id] = now;
}

bool command_router_try_handle_register_reply(const twai_message_t *msg)
{
    uint8_t rid = 0;
    uint32_t reg_value = 0;
    if (!damiao_parse_register_reply(msg, &rid, &reg_value)) {
        return false;
    }

    g_cmd_reg_cache[rid].valid = true;
    g_cmd_reg_cache[rid].value = reg_value;
    if (rid == 7 || rid == 8) {
        ESP_LOGI(TAG, "register reply rid=%u value=0x%08" PRIX32, rid, reg_value);
        if (g_cmd_verify_pending && g_cmd_reg_cache[7].valid && g_cmd_reg_cache[8].valid) {
            uint8_t got_feedback = (uint8_t)g_cmd_reg_cache[7].value;
            uint8_t got_motor = (uint8_t)g_cmd_reg_cache[8].value;
            if (got_feedback == g_cmd_verify_expect_feedback_id && got_motor == g_cmd_verify_expect_motor_id) {
                ESP_LOGI(TAG, "set-ids verify ok motor=0x%02X feedback=0x%02X", got_motor, got_feedback);
            } else {
                ESP_LOGW(TAG,
                         "set-ids verify mismatch expected motor=0x%02X feedback=0x%02X got motor=0x%02X feedback=0x%02X",
                         g_cmd_verify_expect_motor_id,
                         g_cmd_verify_expect_feedback_id,
                         got_motor,
                         got_feedback);
            }
            g_cmd_verify_pending = false;
        }
    }
    return true;
}

void command_router_apply_set_ids(uint8_t old_id, uint8_t new_motor_id, uint8_t new_feedback_id, bool store_after_set, bool verify_after_set)
{
    ESP_LOGI(TAG,
             "set-ids old=0x%02X -> new_motor=0x%02X new_feedback=0x%02X store=%d verify=%d",
             old_id,
             new_motor_id,
             new_feedback_id,
             store_after_set,
             verify_after_set);
    send_damiao_reg_write(old_id, 7, (uint32_t)new_feedback_id);
    send_damiao_reg_write(old_id, 8, (uint32_t)new_motor_id);

    if (store_after_set) {
        twai_message_t tx;
        // Store must be sent to the currently-addressable node (old ID) before remap fully takes effect.
        damiao_build_store_params_cmd(old_id, &tx);
        (void)can_manager_send(&tx, 10);
    }

    if (verify_after_set) {
        g_cmd_verify_pending = true;
        g_cmd_verify_expect_motor_id = new_motor_id;
        g_cmd_verify_expect_feedback_id = new_feedback_id;
        g_cmd_reg_cache[7].valid = false;
        g_cmd_reg_cache[8].valid = false;
        send_damiao_reg_read(new_motor_id, 7);
        send_damiao_reg_read(new_motor_id, 8);
    }
}
