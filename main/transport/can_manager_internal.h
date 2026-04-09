#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"

#include "transport/can_manager.h"

extern const char *g_can_manager_tag;

extern can_rx_callback_t g_can_rx_cb;
extern int g_scan_min;
extern int g_scan_max;
extern bool g_dump_enabled;
extern int64_t g_scan_until_us;
extern bool g_recovery_in_progress;
extern int64_t g_last_rx_us;
extern int64_t g_last_qfull_log_us;
extern int64_t g_last_hard_recover_us;
extern int g_throttle_streak;
extern int64_t g_tx_block_until_us;
extern int64_t g_last_tx_fail_log_us;
extern uint32_t g_tx_fail_suppressed;
extern int64_t g_last_status_log_us;
extern int64_t g_recovery_start_us;
extern int64_t g_last_busoff_reset_us;

void can_manager_tx_backoff_ms(int32_t ms);
void can_manager_maybe_log_twai_status(const twai_status_info_t *st, bool force);
void can_manager_try_recover_on_timeout(void);
void can_manager_try_hard_recover(const twai_status_info_t *st);

void can_manager_start_rx_task(void);
void can_manager_start_scan_task(void);

twai_timing_config_t can_manager_timing_for_bitrate(int bitrate);
