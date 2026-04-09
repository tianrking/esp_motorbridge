#include "transport/can_manager_internal.h"

#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void can_manager_tx_backoff_ms(int32_t ms)
{
    if (ms <= 0) {
        return;
    }
    int64_t until = esp_timer_get_time() + (int64_t)ms * 1000;
    if (until > g_tx_block_until_us) {
        g_tx_block_until_us = until;
    }
}

void can_manager_maybe_log_twai_status(const twai_status_info_t *st, bool force)
{
    if (st == NULL) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (force || (now - g_last_status_log_us) > 300000) {
        ESP_LOGW(g_can_manager_tag,
                 "twai status: state=%d txq=%" PRIu32 " rxq=%" PRIu32
                 " tec=%" PRIu32 " rec=%" PRIu32 " tx_failed=%" PRIu32 " bus_err=%" PRIu32,
                 (int)st->state,
                 st->msgs_to_tx,
                 st->msgs_to_rx,
                 st->tx_error_counter,
                 st->rx_error_counter,
                 st->tx_failed_count,
                 st->bus_error_count);
        g_last_status_log_us = now;
    }
}

void can_manager_try_recover_on_timeout(void)
{
    twai_status_info_t st = {0};
    if (twai_get_status_info(&st) != ESP_OK) {
        return;
    }
    can_manager_maybe_log_twai_status(&st, false);

    if (st.msgs_to_tx > 0) {
        (void)twai_clear_transmit_queue();
    }

    if (st.state == TWAI_STATE_BUS_OFF && !g_recovery_in_progress) {
        if (twai_initiate_recovery() == ESP_OK) {
            g_recovery_in_progress = true;
            g_recovery_start_us = esp_timer_get_time();
            ESP_LOGW(g_can_manager_tag, "TWAI bus-off: recovery initiated");
            can_manager_tx_backoff_ms(120);
        }
    } else if (st.state == TWAI_STATE_BUS_OFF && g_recovery_in_progress) {
        const int64_t now = esp_timer_get_time();
        if ((now - g_recovery_start_us) > 600000 && (now - g_last_busoff_reset_us) > 1000000) {
            g_last_busoff_reset_us = now;
            ESP_LOGW(g_can_manager_tag, "bus-off recovery timeout, force stop/start");
            (void)twai_clear_transmit_queue();
            (void)twai_stop();
            vTaskDelay(pdMS_TO_TICKS(30));
            (void)twai_start();
            g_recovery_in_progress = false;
            can_manager_tx_backoff_ms(220);
        }
    } else if (st.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) {
            ESP_LOGW(g_can_manager_tag, "TWAI was stopped, restarted");
            can_manager_tx_backoff_ms(80);
        }
    } else if (st.state == TWAI_STATE_RUNNING) {
        g_recovery_in_progress = false;
        g_recovery_start_us = 0;
    }
}

void can_manager_try_hard_recover(const twai_status_info_t *st)
{
    if (st == NULL) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (now - g_last_hard_recover_us < 1000000LL) {
        return;
    }

    if (st->msgs_to_tx < 16 || st->tx_error_counter < 96) {
        return;
    }

    g_last_hard_recover_us = now;
    ESP_LOGW(g_can_manager_tag,
             "hard recover: stop/start TWAI (state=%d txq=%" PRIu32 " tec=%" PRIu32 " bus_err=%" PRIu32 ")",
             (int)st->state,
             st->msgs_to_tx,
             st->tx_error_counter,
             st->bus_error_count);
    (void)twai_clear_transmit_queue();
    (void)twai_stop();
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)twai_start();
    g_recovery_in_progress = false;
    can_manager_tx_backoff_ms(200);
}
