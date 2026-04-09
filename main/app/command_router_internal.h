#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "config/app_config.h"
#include "protocol/host_protocol.h"

typedef struct {
    bool valid;
    uint32_t value;
} reg_cache_t;

extern reg_cache_t g_cmd_reg_cache[256];
extern bool g_cmd_verify_pending;
extern uint8_t g_cmd_verify_expect_motor_id;
extern uint8_t g_cmd_verify_expect_feedback_id;
extern int64_t g_cmd_seen_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
extern uint8_t g_cmd_damiao_mode_written[MOTORBRIDGE_MAX_MOTOR_ID + 1];
extern int64_t g_cmd_damiao_mode_written_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

bool command_router_try_handle_register_reply(const twai_message_t *msg);
void command_router_apply_admin_cmd(const host_admin_cmd_t *cmd);
void command_router_apply_mode_cmd(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd);
bool command_router_try_handle_feedback(const twai_message_t *msg);
