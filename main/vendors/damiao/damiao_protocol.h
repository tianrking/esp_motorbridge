#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void damiao_build_scan_request(uint8_t id, twai_message_t *out_msg);
void damiao_build_admin_frame(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg);
bool damiao_build_control_frame(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out_msg);
bool damiao_parse_feedback(const twai_message_t *msg,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint8_t *status);
void damiao_build_register_write_cmd(uint8_t motor_id, uint8_t rid, uint32_t value, twai_message_t *out_msg);
void damiao_build_register_read_cmd(uint8_t motor_id, uint8_t rid, twai_message_t *out_msg);
void damiao_build_store_params_cmd(uint8_t motor_id, twai_message_t *out_msg);
bool damiao_parse_register_reply(const twai_message_t *msg, uint8_t *rid, uint32_t *value);

#ifdef __cplusplus
}
#endif
