#include "transport/can_manager_internal.h"

#include <inttypes.h>
#include <stdarg.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static int s_busoff_level = 0;
static int64_t s_next_recover_attempt_us = 0;
static int64_t s_last_recovery_log_us = 0;
static uint32_t s_recovery_log_suppressed = 0;

static int32_t recovery_backoff_ms_for_level(int level)
{
    if (level < 0) {
        level = 0;
    }
    int32_t ms = 120;
    while (level-- > 0 && ms < 2000) {
        ms <<= 1;
    }
    return (ms > 2000) ? 2000 : ms;
}

static void maybe_log_recovery(const char *fmt, ...)
{
    const int64_t now = esp_timer_get_time();
    if ((now - s_last_recovery_log_us) < 1000000LL) {
        s_recovery_log_suppressed++;
        return;
    }

    if (s_recovery_log_suppressed > 0) {
        ESP_LOGW(g_can_manager_tag, "recovery log suppressed=%" PRIu32, s_recovery_log_suppressed);
        s_recovery_log_suppressed = 0;
    }

    va_list ap;
    va_start(ap, fmt);
    esp_log_writev(ESP_LOG_WARN, g_can_manager_tag, fmt, ap);
    va_end(ap);
    s_last_recovery_log_us = now;
}

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
    if (force || (now - g_last_status_log_us) > 1000000) {
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
        const int64_t now = esp_timer_get_time();
        if (now < s_next_recover_attempt_us) {
            return;
        }

        if (twai_initiate_recovery() == ESP_OK) {
            g_recovery_in_progress = true;
            g_recovery_start_us = now;
            int32_t backoff_ms = recovery_backoff_ms_for_level(s_busoff_level);
            s_next_recover_attempt_us = now + (int64_t)backoff_ms * 1000;
            maybe_log_recovery("TWAI bus-off: recovery initiated level=%d backoff=%dms", s_busoff_level, (int)backoff_ms);
            can_manager_tx_backoff_ms(backoff_ms);
        }
    } else if (st.state == TWAI_STATE_BUS_OFF && g_recovery_in_progress) {
        const int64_t now = esp_timer_get_time();
        if ((now - g_recovery_start_us) > 600000 && (now - g_last_busoff_reset_us) > 1000000 && now >= s_next_recover_attempt_us) {
            g_last_busoff_reset_us = now;
            maybe_log_recovery("bus-off recovery timeout, force stop/start level=%d", s_busoff_level);
            (void)twai_clear_transmit_queue();
            (void)twai_stop();
            vTaskDelay(pdMS_TO_TICKS(30));
            (void)twai_start();
            g_recovery_in_progress = false;
            s_busoff_level = (s_busoff_level < 6) ? (s_busoff_level + 1) : 6;
            int32_t backoff_ms = recovery_backoff_ms_for_level(s_busoff_level);
            s_next_recover_attempt_us = now + (int64_t)backoff_ms * 1000;
            can_manager_tx_backoff_ms(backoff_ms);
        }
    } else if (st.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) {
            maybe_log_recovery("TWAI was stopped, restarted");
            can_manager_tx_backoff_ms(80);
        }
    } else if (st.state == TWAI_STATE_RUNNING) {
        g_recovery_in_progress = false;
        g_recovery_start_us = 0;
        s_busoff_level = 0;
        s_next_recover_attempt_us = 0;
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
