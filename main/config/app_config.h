#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MOTORBRIDGE_HOST_ADMIN_ID 0x050

#define MOTORBRIDGE_HOST_MIT_BASE_ID 0x100
#define MOTORBRIDGE_HOST_POS_VEL_BASE_ID 0x140
#define MOTORBRIDGE_HOST_VEL_BASE_ID 0x180
#define MOTORBRIDGE_HOST_FORCE_POS_BASE_ID 0x1C0

#define MOTORBRIDGE_MIN_MOTOR_ID 1
#define MOTORBRIDGE_MAX_MOTOR_ID 32

typedef enum {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_MIT = 1,
    MOTOR_MODE_POS_VEL = 2,
    MOTOR_MODE_VEL = 3,
    MOTOR_MODE_FORCE_POS = 4,
} motor_mode_t;

typedef struct {
    int tx_gpio;
    int rx_gpio;
    int bitrate;
    int control_period_ms;
    int min_tx_interval_ms;
    int offline_timeout_ms;
    int log_period_ms;
    int max_motors;
    float default_kp;
    float default_kd;
    uint8_t default_motor_id;
    uint8_t default_feedback_id;
    bool apply_default_id_map_at_boot;
    bool wifi_udp_enabled;
    const char *wifi_ssid;
    const char *wifi_password;
    int udp_port;
} app_config_t;

const app_config_t *app_config_get(void);
