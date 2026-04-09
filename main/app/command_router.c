#include "app/command_router.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "transport/can_manager.h"
#include "vendors/damiao/damiao_protocol.h"
#include "vendors/motor_vendor.h"

typedef struct {
    bool valid;
    uint32_t value;
} reg_cache_t;

static const char *TAG = "cmd_router";
static reg_cache_t s_reg_cache[256];
static bool s_verify_pending = false;
static uint8_t s_verify_expect_motor_id = 0;
static uint8_t s_verify_expect_feedback_id = 0;
static int64_t s_seen_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static uint8_t s_damiao_mode_written[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int64_t s_damiao_mode_written_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

static void send_admin_to_motor(uint8_t id, host_admin_opcode_t op)
{
    motor_state_t m;
    if (!motor_manager_get_state(id, &m) || m.vendor == NULL || m.vendor->build_admin_frame == NULL) {
        return;
    }
    twai_message_t tx;
    m.vendor->build_admin_frame(id, op, &tx);
    (void)can_manager_send(&tx, 10);
}

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

static void send_damiao_store_params(uint8_t id)
{
    twai_message_t tx;
    damiao_build_store_params_cmd(id, &tx);
    (void)can_manager_send(&tx, 10);
}

static void ensure_damiao_ctrl_mode(uint8_t id, motor_mode_t mode)
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
    const bool same_mode = s_damiao_mode_written[id] == desired;
    const bool too_soon = (now - s_damiao_mode_written_us[id]) < 300000; // 300ms
    if (same_mode && too_soon) {
        return;
    }

    // Damiao control mode register: rid=10 (1=MIT,2=POS_VEL,3=VEL,4=FORCE_POS)
    send_damiao_reg_write(id, 10, (uint32_t)desired);
    s_damiao_mode_written[id] = desired;
    s_damiao_mode_written_us[id] = now;
}

static void apply_set_ids(uint8_t old_id, uint8_t new_motor_id, uint8_t new_feedback_id, bool store_after_set, bool verify_after_set)
{
    // Damiao: rid=7 MST_ID (feedback), rid=8 ESC_ID (motor)
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
        send_damiao_store_params(new_motor_id);
    }

    if (verify_after_set) {
        s_verify_pending = true;
        s_verify_expect_motor_id = new_motor_id;
        s_verify_expect_feedback_id = new_feedback_id;
        s_reg_cache[7].valid = false;
        s_reg_cache[8].valid = false;
        send_damiao_reg_read(new_motor_id, 7);
        send_damiao_reg_read(new_motor_id, 8);
    }
}

static void apply_admin_cmd(const host_admin_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    const app_config_t *cfg = app_config_get();

    if (cmd->op == HOST_OP_SCAN) {
        can_manager_trigger_scan(MOTORBRIDGE_MIN_MOTOR_ID, cfg->max_motors);
        return;
    }

    if (cmd->op == HOST_OP_ESTOP) {
        motor_manager_estop_all();
        for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
            send_admin_to_motor((uint8_t)i, HOST_OP_DISABLE);
        }
        return;
    }

    if (cmd->op == HOST_OP_SET_IDS) {
        uint8_t old_id = cmd->motor_id == 0 ? cfg->default_motor_id : cmd->motor_id;
        uint8_t new_motor_id = cmd->new_motor_id == 0 ? cfg->default_motor_id : cmd->new_motor_id;
        uint8_t new_feedback_id = cmd->new_feedback_id == 0 ? cfg->default_feedback_id : cmd->new_feedback_id;
        apply_set_ids(old_id, new_motor_id, new_feedback_id, cmd->store_after_set, cmd->verify_after_set);
        return;
    }

    int start = (cmd->motor_id == 0) ? MOTORBRIDGE_MIN_MOTOR_ID : cmd->motor_id;
    int end = (cmd->motor_id == 0) ? motor_manager_count() : cmd->motor_id;

    for (int i = start; i <= end; ++i) {
        uint8_t id = (uint8_t)i;
        switch (cmd->op) {
        case HOST_OP_SET_MODE:
            ensure_damiao_ctrl_mode(id, cmd->mode);
            (void)motor_manager_set_mode(id, cmd->mode);
            break;
        case HOST_OP_SET_GAINS:
            (void)motor_manager_set_gains(id, cmd->kp, cmd->kd);
            break;
        case HOST_OP_ENABLE:
            (void)motor_manager_set_enabled(id, true);
            send_admin_to_motor(id, HOST_OP_ENABLE);
            break;
        case HOST_OP_DISABLE:
            (void)motor_manager_set_enabled(id, false);
            send_admin_to_motor(id, HOST_OP_DISABLE);
            break;
        case HOST_OP_SET_ZERO:
            send_admin_to_motor(id, HOST_OP_SET_ZERO);
            break;
        case HOST_OP_CLEAR_ERROR:
            send_admin_to_motor(id, HOST_OP_CLEAR_ERROR);
            break;
        default:
            break;
        }
    }
}

static void apply_mode_cmd(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    ensure_damiao_ctrl_mode(id, mode);

    switch (mode) {
    case MOTOR_MODE_MIT:
        (void)motor_manager_set_mit_cmd(id, cmd->pos, cmd->vel, cmd->tau);
        break;
    case MOTOR_MODE_POS_VEL:
        (void)motor_manager_set_pos_vel_cmd(id, cmd->pos, cmd->vlim);
        break;
    case MOTOR_MODE_VEL:
        (void)motor_manager_set_vel_cmd(id, cmd->vel);
        break;
    case MOTOR_MODE_FORCE_POS:
        (void)motor_manager_set_force_pos_cmd(id, cmd->pos, cmd->vlim, cmd->ratio);
        break;
    default:
        break;
    }
}

void command_router_handle_can_rx(const twai_message_t *msg)
{
    uint8_t rid = 0;
    uint32_t reg_value = 0;
    if (damiao_parse_register_reply(msg, &rid, &reg_value)) {
        s_reg_cache[rid].valid = true;
        s_reg_cache[rid].value = reg_value;
        if (rid == 7 || rid == 8) {
            ESP_LOGI(TAG, "register reply rid=%u value=0x%08" PRIX32, rid, reg_value);
            if (s_verify_pending && s_reg_cache[7].valid && s_reg_cache[8].valid) {
                uint8_t got_feedback = (uint8_t)s_reg_cache[7].value;
                uint8_t got_motor = (uint8_t)s_reg_cache[8].value;
                if (got_feedback == s_verify_expect_feedback_id && got_motor == s_verify_expect_motor_id) {
                    ESP_LOGI(TAG, "set-ids verify ok motor=0x%02X feedback=0x%02X", got_motor, got_feedback);
                } else {
                    ESP_LOGW(TAG,
                             "set-ids verify mismatch expected motor=0x%02X feedback=0x%02X got motor=0x%02X feedback=0x%02X",
                             s_verify_expect_motor_id,
                             s_verify_expect_feedback_id,
                             got_motor,
                             got_feedback);
                }
                s_verify_pending = false;
            }
        }
        return;
    }

    host_admin_cmd_t admin;
    if (host_protocol_parse_admin_cmd(msg, &admin)) {
        apply_admin_cmd(&admin);
        return;
    }

    uint8_t id = 0;
    motor_mode_t mode = MOTOR_MODE_DISABLED;
    damiao_cmd_t cmd = {0};
    if (host_protocol_parse_mode_frame(msg, &id, &mode, &cmd)) {
        if (id >= MOTORBRIDGE_MIN_MOTOR_ID && id <= motor_manager_count()) {
            apply_mode_cmd(id, mode, &cmd);
        }
        return;
    }

    float pos = 0, vel = 0, torque = 0, t_mos = 0, t_rotor = 0;
    uint8_t status = 0;
    uint8_t parsed_id = 0;

    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
        motor_state_t m;
        if (motor_manager_get_state((uint8_t)i, &m) && m.vendor != NULL && m.vendor->parse_feedback != NULL) {
            if (m.vendor->parse_feedback(msg, &parsed_id, &pos, &vel, &torque, &t_mos, &t_rotor, &status)) {
                if (parsed_id == i) {
                    motor_manager_update_feedback(parsed_id, pos, vel, torque, t_mos, t_rotor, status);

                    if (parsed_id < sizeof(s_seen_us)/sizeof(s_seen_us[0])) {
                        s_seen_us[parsed_id] = esp_timer_get_time();
                    }
                    return;
                }
            }
        }
    }
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
        if (s_seen_us[i] > 0 && (now_us - s_seen_us[i]) <= window_us) {
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

    int64_t start_seen = s_seen_us[id];
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline_us) {
        if (s_seen_us[id] > start_seen) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}
