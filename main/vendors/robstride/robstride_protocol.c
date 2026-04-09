#include "vendors/robstride/robstride_protocol.h"
#include <string.h>

#define RS_COMM_GET_DEVID 0
#define RS_COMM_CONTROL   1
#define RS_COMM_STATUS    2
#define RS_COMM_ENABLE    3
#define RS_COMM_DISABLE   4
#define RS_COMM_SET_ZERO  6
#define RS_COMM_CLEAR_ERR 4 // Fallback to disable on clear error

static uint32_t build_ext_id(uint32_t comm, uint16_t extra, uint8_t id) {
    return (comm << 24) | (((uint32_t)extra) << 8) | ((uint32_t)id);
}

void robstride_build_scan_request(uint8_t id, twai_message_t *out_msg) {
    if (!out_msg) return;
    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->identifier = build_ext_id(RS_COMM_GET_DEVID, 0x00FF, id);
    out_msg->extd = 1;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
}

void robstride_build_admin_frame(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg) {
    if (!out_msg) return;
    memset(out_msg, 0, sizeof(*out_msg));
    out_msg->extd = 1;
    out_msg->rtr = 0;
    out_msg->data_length_code = 0;
    
    uint32_t comm = RS_COMM_DISABLE;
    switch(op) {
        case HOST_OP_ENABLE: comm = RS_COMM_ENABLE; break;
        case HOST_OP_DISABLE: comm = RS_COMM_DISABLE; break;
        case HOST_OP_SET_ZERO: comm = RS_COMM_SET_ZERO; break;
        case HOST_OP_CLEAR_ERROR: comm = RS_COMM_CLEAR_ERR; break;
        default: break;
    }
    out_msg->identifier = build_ext_id(comm, 0, id);
}

static float clampf(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

bool robstride_build_control_frame(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd, const motor_params_t *params, twai_message_t *out_msg) {
    if (!out_msg || !cmd || !params) return false;
    // RS only supports MIT mode via main control frame in this sdk
    if (mode != MOTOR_MODE_MIT) return false; 
    
    float pmax = params->p_max > 0.0f ? params->p_max : 12.5f; 
    float vmax = params->v_max > 0.0f ? params->v_max : 10.0f;
    float tmax = params->t_max > 0.0f ? params->t_max : 28.0f;
    float kp_max = 500.0f;
    float kd_max = 5.0f;

    uint16_t pos_u16 = (uint16_t)(((clampf(cmd->pos, -pmax, pmax) / pmax) + 1.0f) * 32767.0f);
    uint16_t vel_u16 = (uint16_t)(((clampf(cmd->vel, -vmax, vmax) / vmax) + 1.0f) * 32767.0f);
    uint16_t kp_u16 = (uint16_t)((clampf(params->kp, 0.0f, kp_max) / kp_max) * 65535.0f);
    uint16_t kd_u16 = (uint16_t)((clampf(params->kd, 0.0f, kd_max) / kd_max) * 65535.0f);
    uint16_t torq_u16 = (uint16_t)(((clampf(cmd->tau, -tmax, tmax) / tmax) + 1.0f) * 32767.0f);

    out_msg->identifier = build_ext_id(RS_COMM_CONTROL, torq_u16, id);
    out_msg->extd = 1;
    out_msg->rtr = 0;
    out_msg->data_length_code = 8;
    out_msg->data[0] = pos_u16 >> 8;
    out_msg->data[1] = pos_u16 & 0xFF;
    out_msg->data[2] = vel_u16 >> 8;
    out_msg->data[3] = vel_u16 & 0xFF;
    out_msg->data[4] = kp_u16 >> 8;
    out_msg->data[5] = kp_u16 & 0xFF;
    out_msg->data[6] = kd_u16 >> 8;
    out_msg->data[7] = kd_u16 & 0xFF;
    return true;
}

bool robstride_parse_feedback(const twai_message_t *msg, uint8_t *id, float *pos, float *vel, float *torque, float *t_mos, float *t_rotor, uint8_t *status) {
    if (!msg || msg->extd == 0 || msg->data_length_code < 8) return false;
    
    uint32_t comm_type = (msg->identifier >> 24) & 0x1F;
    uint16_t extra_data = (msg->identifier >> 8) & 0xFFFF;
    
    if (comm_type == RS_COMM_GET_DEVID) {
        if (id) *id = (uint8_t)(extra_data & 0xFF);
        if (pos) *pos = 0.0f;
        if (vel) *vel = 0.0f;
        if (torque) *torque = 0.0f;
        if (t_mos) *t_mos = 0.0f;
        if (t_rotor) *t_rotor = 0.0f;
        if (status) *status = 0;
        return true;
    }
    
    if (comm_type != RS_COMM_STATUS) return false;
    
    uint8_t node_id = msg->identifier & 0xFF;
    
    uint16_t pos_u16 = (msg->data[0] << 8) | msg->data[1];
    uint16_t vel_u16 = (msg->data[2] << 8) | msg->data[3];
    uint16_t torq_u16 = (msg->data[4] << 8) | msg->data[5];
    uint16_t temp_u16 = (msg->data[6] << 8) | msg->data[7];

    float pmax = 12.5f; 
    float vmax = 10.0f;
    float tmax = 28.0f;

    if (id) *id = node_id;
    if (pos) *pos = ((float)pos_u16 / 32767.0f - 1.0f) * pmax;
    if (vel) *vel = ((float)vel_u16 / 32767.0f - 1.0f) * vmax;
    if (torque) *torque = ((float)torq_u16 / 32767.0f - 1.0f) * tmax;
    if (t_mos) *t_mos = (float)temp_u16 * 0.1f;
    if (t_rotor) *t_rotor = 0.0f;
    
    if (status) {
        uint8_t st = 0;
        if (extra_data & (1<<12)) st |= 0x01; // stall
        if (extra_data & (1<<10)) st |= 0x02; // overtemp
        *status = st;
    }
    
    return true;
}
