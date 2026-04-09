#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/app_config.h"

struct motor_vendor_ops_s;
typedef struct motor_vendor_ops_s motor_vendor_ops_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pos;
    float vel;
    float tau;
    float vlim;
    float ratio;
} damiao_cmd_t;

typedef struct {
    float kp;
    float kd;
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float t_min;
    float t_max;
} motor_params_t;

typedef struct {
    uint8_t id;
    bool online;
    bool enabled;
    motor_mode_t mode;
    damiao_cmd_t cmd;
    float position;
    float speed;
    float torque;
    float t_mos;
    float t_rotor;
    uint8_t status;
    int64_t last_seen_us;
    motor_params_t params;
    const motor_vendor_ops_t *vendor;
} motor_state_t;

void motor_manager_init(int max_motors);
int motor_manager_count(void);

bool motor_manager_set_vendor(uint8_t id, const char *vendor_name);

bool motor_manager_set_mode(uint8_t id, motor_mode_t mode);
bool motor_manager_set_enabled(uint8_t id, bool enabled);

bool motor_manager_set_mit_cmd(uint8_t id, float pos, float vel, float tau);
bool motor_manager_set_pos_vel_cmd(uint8_t id, float pos, float vlim);
bool motor_manager_set_vel_cmd(uint8_t id, float vel);
bool motor_manager_set_force_pos_cmd(uint8_t id, float pos, float vlim, float ratio);
bool motor_manager_set_gains(uint8_t id, float kp, float kd);

bool motor_manager_get_state(uint8_t id, motor_state_t *out);
void motor_manager_update_feedback(uint8_t id, float pos, float speed, float torque, float t_mos, float t_rotor, uint8_t status);
void motor_manager_mark_seen(uint8_t id);

void motor_manager_mark_offline_by_timeout(int64_t now_us, int32_t timeout_ms);
void motor_manager_estop_all(void);

void motor_manager_for_each(void (*fn)(motor_state_t *m, void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif
