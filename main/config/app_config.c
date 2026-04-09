#include "app_config.h"

#include "sdkconfig.h"

#ifndef CONFIG_MOTORBRIDGE_APPLY_DEFAULT_ID_MAP_AT_BOOT
#define CONFIG_MOTORBRIDGE_APPLY_DEFAULT_ID_MAP_AT_BOOT 0
#endif

static const app_config_t s_cfg = {
    .tx_gpio = CONFIG_MOTORBRIDGE_CAN_TX_GPIO,
    .rx_gpio = CONFIG_MOTORBRIDGE_CAN_RX_GPIO,
    .bitrate = CONFIG_MOTORBRIDGE_CAN_BITRATE,
    .control_period_ms = CONFIG_MOTORBRIDGE_CONTROL_PERIOD_MS,
    .min_tx_interval_ms = CONFIG_MOTORBRIDGE_MIN_TX_INTERVAL_MS,
    .offline_timeout_ms = CONFIG_MOTORBRIDGE_OFFLINE_TIMEOUT_MS,
    .log_period_ms = CONFIG_MOTORBRIDGE_LOG_PERIOD_MS,
    .max_motors = CONFIG_MOTORBRIDGE_MAX_MOTORS,
    .default_kp = (float)CONFIG_MOTORBRIDGE_DEFAULT_KP_X100 / 100.0f,
    .default_kd = (float)CONFIG_MOTORBRIDGE_DEFAULT_KD_X1000 / 1000.0f,
    .default_motor_id = CONFIG_MOTORBRIDGE_DEFAULT_MOTOR_ID,
    .default_feedback_id = CONFIG_MOTORBRIDGE_DEFAULT_FEEDBACK_ID,
    .apply_default_id_map_at_boot = CONFIG_MOTORBRIDGE_APPLY_DEFAULT_ID_MAP_AT_BOOT,
#if CONFIG_MOTORBRIDGE_ENABLE_WIFI_UDP
    .wifi_udp_enabled = true,
    .wifi_ssid = CONFIG_MOTORBRIDGE_WIFI_SSID,
    .wifi_password = CONFIG_MOTORBRIDGE_WIFI_PASSWORD,
    .udp_port = CONFIG_MOTORBRIDGE_UDP_PORT,
#else
    .wifi_udp_enabled = false,
    .wifi_ssid = "",
    .wifi_password = "",
    .udp_port = 0,
#endif
};

const app_config_t *app_config_get(void)
{
    return &s_cfg;
}
