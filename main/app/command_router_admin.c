#include "app/command_router_internal.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

void command_router_ensure_damiao_ctrl_mode(uint8_t id, motor_mode_t mode);
void command_router_apply_set_ids(uint8_t old_id,
                                  uint8_t new_motor_id,
                                  uint8_t new_feedback_id,
                                  bool store_after_set,
                                  bool verify_after_set);

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

void command_router_apply_admin_cmd(const host_admin_cmd_t *cmd)
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
        command_router_apply_set_ids(old_id, new_motor_id, new_feedback_id, cmd->store_after_set, cmd->verify_after_set);
        return;
    }

    int start = (cmd->motor_id == 0) ? MOTORBRIDGE_MIN_MOTOR_ID : cmd->motor_id;
    int end = (cmd->motor_id == 0) ? motor_manager_count() : cmd->motor_id;

    for (int i = start; i <= end; ++i) {
        uint8_t id = (uint8_t)i;
        switch (cmd->op) {
        case HOST_OP_SET_MODE:
            command_router_ensure_damiao_ctrl_mode(id, cmd->mode);
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

void command_router_apply_mode_cmd(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    command_router_ensure_damiao_ctrl_mode(id, mode);

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
