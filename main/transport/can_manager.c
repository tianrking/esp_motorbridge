#include "can_manager.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "config/app_config.h"
#include "transport/can_manager_internal.h"

const char *g_can_manager_tag = "can_manager";

can_rx_callback_t g_can_rx_cb = NULL;
int g_scan_min = MOTORBRIDGE_MIN_MOTOR_ID;
int g_scan_max = MOTORBRIDGE_MAX_MOTOR_ID;
bool g_dump_enabled = false;
int64_t g_scan_until_us = 0;
bool g_recovery_in_progress = false;
int64_t g_last_rx_us = 0;
int64_t g_last_qfull_log_us = 0;
int64_t g_last_hard_recover_us = 0;
int g_throttle_streak = 0;
int64_t g_tx_block_until_us = 0;
int64_t g_last_tx_fail_log_us = 0;
uint32_t g_tx_fail_suppressed = 0;
int64_t g_last_status_log_us = 0;
int64_t g_recovery_start_us = 0;
int64_t g_last_busoff_reset_us = 0;

twai_timing_config_t can_manager_timing_for_bitrate(int bitrate)
{
    twai_timing_config_t cfg;
    switch (bitrate) {
    case 250000:
        cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
        cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 800000:
        cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        break;
    case 1000000:
    default:
        cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        break;
    }
    return cfg;
}

esp_err_t can_manager_init(int tx_gpio, int rx_gpio, int bitrate)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = can_manager_timing_for_bitrate(bitrate);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.tx_queue_len = 64;
    g_config.rx_queue_len = 64;

    ESP_LOGI(g_can_manager_tag, "twai install tx=%d rx=%d bitrate=%d", tx_gpio, rx_gpio, bitrate);
    return twai_driver_install(&g_config, &t_config, &f_config);
}

esp_err_t can_manager_start(void)
{
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        return err;
    }

    can_manager_start_rx_task();
    can_manager_start_scan_task();
    return ESP_OK;
}

void can_manager_set_rx_callback(can_rx_callback_t cb)
{
    g_can_rx_cb = cb;
}

void can_manager_trigger_scan(int min_id, int max_id)
{
    if (min_id < MOTORBRIDGE_MIN_MOTOR_ID) {
        min_id = MOTORBRIDGE_MIN_MOTOR_ID;
    }
    if (max_id > MOTORBRIDGE_MAX_MOTOR_ID) {
        max_id = MOTORBRIDGE_MAX_MOTOR_ID;
    }
    if (min_id > max_id) {
        return;
    }

    g_scan_min = min_id;
    g_scan_max = max_id;
    g_scan_until_us = esp_timer_get_time() + 2000000LL;
    ESP_LOGI(g_can_manager_tag, "scan range updated [%d, %d]", g_scan_min, g_scan_max);
}

void can_manager_set_dump(bool enabled)
{
    g_dump_enabled = enabled;
    ESP_LOGI(g_can_manager_tag, "candump %s", enabled ? "on" : "off");
}

bool can_manager_recent_rx(int32_t within_ms)
{
    if (within_ms <= 0) {
        return false;
    }
    int64_t last = g_last_rx_us;
    if (last <= 0) {
        return false;
    }
    return (esp_timer_get_time() - last) <= ((int64_t)within_ms * 1000);
}
