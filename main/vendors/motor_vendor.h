#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;
    void (*build_scan_request)(uint8_t id, twai_message_t *out_msg);
    void (*build_admin_frame)(uint8_t id, host_admin_opcode_t op, twai_message_t *out_msg);
    bool (*build_control_frame)(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out_msg);
    bool (*parse_feedback)(const twai_message_t *msg,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint8_t *status);
} motor_vendor_ops_t;

const motor_vendor_ops_t *motor_vendor_active(void);

#ifdef __cplusplus
}
#endif
