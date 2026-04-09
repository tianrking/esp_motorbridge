#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/twai.h"
#include "protocol/host_protocol.h"
#include "core/motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

void robstride_build_scan_request(uint8_t id, twai_message_t *out_msg);
void robstride_build_admin_frame(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg);
bool robstride_build_control_frame(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd, const motor_params_t *params, twai_message_t *out_msg);
bool robstride_parse_feedback(const twai_message_t *msg, uint8_t *id, float *pos, float *vel, float *torque, float *t_mos, float *t_rotor, uint8_t *status);

#ifdef __cplusplus
}
#endif
