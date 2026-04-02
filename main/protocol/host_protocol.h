#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOST_OP_SET_MODE = 1,
    HOST_OP_ESTOP = 2,
    HOST_OP_SCAN = 3,
    HOST_OP_ENABLE = 4,
    HOST_OP_DISABLE = 5,
    HOST_OP_SET_ZERO = 6,
    HOST_OP_CLEAR_ERROR = 7,
    HOST_OP_SET_GAINS = 8,
    HOST_OP_SET_IDS = 9,
} host_admin_opcode_t;

typedef struct {
    host_admin_opcode_t op;
    uint8_t motor_id;
    motor_mode_t mode;
    float kp;
    float kd;
    uint8_t new_motor_id;
    uint8_t new_feedback_id;
    bool store_after_set;
    bool verify_after_set;
} host_admin_cmd_t;

bool host_protocol_parse_admin_cmd(const twai_message_t *msg, host_admin_cmd_t *out_cmd);
bool host_protocol_parse_mode_frame(const twai_message_t *msg, uint8_t *id, motor_mode_t *mode, damiao_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif
