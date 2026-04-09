#include "vendors/damiao/damiao_protocol.h"

#include <string.h>

#include "vendors/damiao/damiao_codec_utils.h"

bool damiao_build_control_frame(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out_msg)
{
    if (out_msg == NULL || cmd == NULL || params == NULL) {
        return false;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;

    switch (mode) {
    case MOTOR_MODE_MIT: {
        out_msg->identifier = id;
        uint32_t pos_u = damiao_float_to_uint(cmd->pos, params->p_min, params->p_max, 16);
        uint32_t vel_u = damiao_float_to_uint(cmd->vel, params->v_min, params->v_max, 12);
        uint32_t kp_u = damiao_float_to_uint(params->kp, 0.0f, 500.0f, 12);
        uint32_t kd_u = damiao_float_to_uint(params->kd, 0.0f, 5.0f, 12);
        uint32_t tau_u = damiao_float_to_uint(cmd->tau, params->t_min, params->t_max, 12);

        out_msg->data[0] = (uint8_t)((pos_u >> 8) & 0xFF);
        out_msg->data[1] = (uint8_t)(pos_u & 0xFF);
        out_msg->data[2] = (uint8_t)((vel_u >> 4) & 0xFF);
        out_msg->data[3] = (uint8_t)(((vel_u & 0xF) << 4) | ((kp_u >> 8) & 0xF));
        out_msg->data[4] = (uint8_t)(kp_u & 0xFF);
        out_msg->data[5] = (uint8_t)((kd_u >> 4) & 0xFF);
        out_msg->data[6] = (uint8_t)(((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF));
        out_msg->data[7] = (uint8_t)(tau_u & 0xFF);
        break;
    }
    case MOTOR_MODE_POS_VEL:
        out_msg->identifier = 0x100 + id;
        memcpy(&out_msg->data[0], &cmd->pos, sizeof(float));
        memcpy(&out_msg->data[4], &cmd->vlim, sizeof(float));
        break;
    case MOTOR_MODE_VEL:
        out_msg->identifier = 0x200 + id;
        memcpy(&out_msg->data[0], &cmd->vel, sizeof(float));
        break;
    case MOTOR_MODE_FORCE_POS: {
        out_msg->identifier = 0x300 + id;
        memcpy(&out_msg->data[0], &cmd->pos, sizeof(float));
        uint16_t v_des = (uint16_t)(cmd->vlim < 0.0f ? 0.0f : (cmd->vlim > 100.0f ? 10000.0f : cmd->vlim * 100.0f));
        float ratio = cmd->ratio;
        if (ratio < 0.0f) {
            ratio = 0.0f;
        }
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        uint16_t i_des = (uint16_t)(ratio * 10000.0f);
        memcpy(&out_msg->data[4], &v_des, sizeof(uint16_t));
        memcpy(&out_msg->data[6], &i_des, sizeof(uint16_t));
        break;
    }
    case MOTOR_MODE_DISABLED:
    default:
        return false;
    }

    return true;
}
