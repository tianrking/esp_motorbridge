#include "vendors/damiao/damiao_protocol.h"

#include <string.h>

#include "config/app_config.h"

static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float clipped = x;
    if (clipped < x_min) {
        clipped = x_min;
    }
    if (clipped > x_max) {
        clipped = x_max;
    }
    return (uint32_t)(((clipped - x_min) * ((float)((1u << bits) - 1u))) / span);
}

static float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x) * span / ((float)((1u << bits) - 1u)) + x_min;
}

void damiao_build_scan_request(uint8_t id, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0xCC;
}

void damiao_build_admin_frame(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = id;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    memset(out_msg->data, 0xFF, sizeof(out_msg->data));

    switch (op) {
    case HOST_OP_ENABLE:
        out_msg->data[7] = 0xFC;
        break;
    case HOST_OP_DISABLE:
        out_msg->data[7] = 0xFD;
        break;
    case HOST_OP_SET_ZERO:
        out_msg->data[7] = 0xFE;
        break;
    case HOST_OP_CLEAR_ERROR:
        out_msg->data[7] = 0xFB;
        break;
    default:
        out_msg->data[7] = 0xFD;
        break;
    }
}

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
        uint32_t pos_u = float_to_uint(cmd->pos, params->p_min, params->p_max, 16);
        uint32_t vel_u = float_to_uint(cmd->vel, params->v_min, params->v_max, 12);
        uint32_t kp_u = float_to_uint(params->kp, 0.0f, 500.0f, 12);
        uint32_t kd_u = float_to_uint(params->kd, 0.0f, 5.0f, 12);
        uint32_t tau_u = float_to_uint(cmd->tau, params->t_min, params->t_max, 12);

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

bool damiao_parse_feedback(const twai_message_t *msg,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint8_t *status)
{
    if (msg == NULL || msg->data_length_code < 8) {
        return false;
    }

    if (msg->identifier == MOTORBRIDGE_HOST_ADMIN_ID) {
        return false;
    }

    uint8_t can_id = msg->data[0] & 0x0F;
    uint8_t st = msg->data[0] >> 4;

    uint32_t pos_i = ((uint32_t)msg->data[1] << 8) | (uint32_t)msg->data[2];
    uint32_t vel_i = ((uint32_t)msg->data[3] << 4) | ((uint32_t)msg->data[4] >> 4);
    uint32_t torq_i = (((uint32_t)msg->data[4] & 0x0F) << 8) | (uint32_t)msg->data[5];

    if (id != NULL) {
        *id = can_id;
    }
    if (status != NULL) {
        *status = st;
    }
    if (pos != NULL) {
        *pos = uint_to_float(pos_i, -12.5f, 12.5f, 16);
    }
    if (vel != NULL) {
        *vel = uint_to_float(vel_i, -30.0f, 30.0f, 12);
    }
    if (torque != NULL) {
        *torque = uint_to_float(torq_i, -10.0f, 10.0f, 12);
    }
    if (t_mos != NULL) {
        *t_mos = (float)msg->data[6];
    }
    if (t_rotor != NULL) {
        *t_rotor = (float)msg->data[7];
    }

    return can_id > 0;
}

void damiao_build_register_write_cmd(uint8_t motor_id, uint8_t rid, uint32_t value, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0x55;
    out_msg->data[3] = rid;
    memcpy(&out_msg->data[4], &value, sizeof(uint32_t));
}

void damiao_build_register_read_cmd(uint8_t motor_id, uint8_t rid, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0x33;
    out_msg->data[3] = rid;
}

void damiao_build_store_params_cmd(uint8_t motor_id, twai_message_t *out_msg)
{
    if (out_msg == NULL) {
        return;
    }

    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = 0x7FF;
    out_msg->extd = 0;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = motor_id;
    out_msg->data[1] = 0;
    out_msg->data[2] = 0xAA;
    out_msg->data[3] = 0x01;
}

bool damiao_parse_register_reply(const twai_message_t *msg, uint8_t *rid, uint32_t *value)
{
    if (msg == NULL || msg->data_length_code < 8) {
        return false;
    }

    if (msg->data[2] != 0x33) {
        return false;
    }

    if (rid != NULL) {
        *rid = msg->data[3];
    }
    if (value != NULL) {
        uint32_t v = 0;
        memcpy(&v, &msg->data[4], sizeof(uint32_t));
        *value = v;
    }
    return true;
}
