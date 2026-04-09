#include "motor_manager.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "vendors/motor_vendor.h"

static motor_state_t s_motors[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int s_max_motors = 0;
static SemaphoreHandle_t s_lock;
static const char *TAG = "motor_manager";

static bool valid_id(uint8_t id)
{
    return id >= MOTORBRIDGE_MIN_MOTOR_ID && id <= s_max_motors;
}

static void init_default_params(motor_params_t *p)
{
    const app_config_t *cfg = app_config_get();
    p->kp = cfg->default_kp;
    p->kd = cfg->default_kd;

    // 4340/4340P defaults; tune per model if needed.
    p->p_min = -12.5f;
    p->p_max = 12.5f;
    p->v_min = -10.0f;
    p->v_max = 10.0f;
    p->t_min = -28.0f;
    p->t_max = 28.0f;
}

void motor_manager_init(int max_motors)
{
    if (max_motors > MOTORBRIDGE_MAX_MOTOR_ID) {
        max_motors = MOTORBRIDGE_MAX_MOTOR_ID;
    }
    if (max_motors < MOTORBRIDGE_MIN_MOTOR_ID) {
        max_motors = MOTORBRIDGE_MIN_MOTOR_ID;
    }

    s_lock = xSemaphoreCreateMutex();
    s_max_motors = max_motors;
    memset(s_motors, 0, sizeof(s_motors));

    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->id = (uint8_t)i;
        m->mode = MOTOR_MODE_DISABLED;
        m->enabled = false;
        m->vendor = NULL;
        m->cmd.ratio = 0.1f;
        init_default_params(&m->params);
    }
}

int motor_manager_count(void)
{
    return s_max_motors;
}

bool motor_manager_set_vendor(uint8_t id, const char *vendor_name)
{
    if (!valid_id(id)) {
        return false;
    }
    const motor_vendor_ops_t *vendor = motor_vendor_get(vendor_name);
    if (vendor == NULL) {
        ESP_LOGE(TAG, "Unknown motor vendor: %s", vendor_name);
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].vendor = vendor;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_mode(uint8_t id, motor_mode_t mode)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].mode = mode;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_enabled(uint8_t id, bool enabled)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].enabled = enabled;
    // Keep last mode while disabled, so a later enable can resume control intent.
    // Hard stop semantics are handled by motor_manager_estop_all().
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_mit_cmd(uint8_t id, float pos, float vel, float tau)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].mode = MOTOR_MODE_MIT;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vel = vel;
    s_motors[id].cmd.tau = tau;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_pos_vel_cmd(uint8_t id, float pos, float vlim)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].mode = MOTOR_MODE_POS_VEL;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_vel_cmd(uint8_t id, float vel)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].mode = MOTOR_MODE_VEL;
    s_motors[id].cmd.vel = vel;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_force_pos_cmd(uint8_t id, float pos, float vlim, float ratio)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].mode = MOTOR_MODE_FORCE_POS;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    s_motors[id].cmd.ratio = ratio;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_gains(uint8_t id, float kp, float kd)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].params.kp = kp;
    s_motors[id].params.kd = kd;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_get_state(uint8_t id, motor_state_t *out)
{
    if (!valid_id(id) || out == NULL) {
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_motors[id];
    xSemaphoreGive(s_lock);
    return true;
}

void motor_manager_update_feedback(uint8_t id, float pos, float speed, float torque, float t_mos, float t_rotor, uint8_t status)
{
    if (!valid_id(id)) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_state_t *m = &s_motors[id];
    m->online = true;
    m->position = pos;
    m->speed = speed;
    m->torque = torque;
    m->t_mos = t_mos;
    m->t_rotor = t_rotor;
    m->status = status;
    m->last_seen_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
}

void motor_manager_mark_seen(uint8_t id)
{
    if (!valid_id(id)) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].online = true;
    s_motors[id].last_seen_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
}

void motor_manager_mark_offline_by_timeout(int64_t now_us, int32_t timeout_ms)
{
    int64_t timeout_us = (int64_t)timeout_ms * 1000;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        if (m->online && (now_us - m->last_seen_us > timeout_us)) {
            ESP_LOGW(TAG,
                     "id=%u offline timeout (%" PRId64 "ms > %dms), mark offline only",
                     m->id,
                     (now_us - m->last_seen_us) / 1000,
                     timeout_ms);
            m->online = false;
        }
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_estop_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->mode = MOTOR_MODE_DISABLED;
        m->enabled = false;
        memset(&m->cmd, 0, sizeof(m->cmd));
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_clear_online_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->online = false;
        m->last_seen_us = 0;
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_for_each(void (*fn)(motor_state_t *m, void *ctx), void *ctx)
{
    if (fn == NULL) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        fn(&s_motors[i], ctx);
    }
    xSemaphoreGive(s_lock);
}
