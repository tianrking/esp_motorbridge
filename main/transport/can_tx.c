#include "transport/can_manager_internal.h"

#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void maybe_log_tx_fail(uint32_t id, esp_err_t err)
{
    const int64_t now = esp_timer_get_time();
    if ((now - g_last_tx_fail_log_us) > 200000) {
        if (g_tx_fail_suppressed > 0) {
            ESP_LOGW(g_can_manager_tag, "tx fail storm: suppressed=%" PRIu32, g_tx_fail_suppressed);
            g_tx_fail_suppressed = 0;
        }
        ESP_LOGW(g_can_manager_tag, "tx failed id=0x%03" PRIX32 " err=0x%x", id, (unsigned int)err);
        g_last_tx_fail_log_us = now;
    } else {
        g_tx_fail_suppressed++;
    }
}

esp_err_t can_manager_send(const twai_message_t *msg, int timeout_ms)
{
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_dump_enabled) {
        ESP_LOGI(g_can_manager_tag,
                 "tx id=0x%03" PRIX32 " dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                 msg->identifier,
                 msg->data_length_code,
                 msg->data[0],
                 msg->data[1],
                 msg->data[2],
                 msg->data[3],
                 msg->data[4],
                 msg->data[5],
                 msg->data[6],
                 msg->data[7]);
    }

    if (timeout_ms <= 0) {
        timeout_ms = 1;
    }

    int64_t now_us = esp_timer_get_time();
    if (now_us < g_tx_block_until_us) {
        int32_t wait_ms = (int32_t)((g_tx_block_until_us - now_us + 999) / 1000);
        if (wait_ms > timeout_ms) {
            wait_ms = timeout_ms;
        }
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }

    twai_status_info_t st = {0};
    if (twai_get_status_info(&st) == ESP_OK && st.state != TWAI_STATE_RUNNING) {
        can_manager_maybe_log_twai_status(&st, false);
        can_manager_try_recover_on_timeout();
        can_manager_tx_backoff_ms(60);
        return ESP_ERR_INVALID_STATE;
    }

    int waited_ms = 0;
    while (twai_get_status_info(&st) == ESP_OK && st.msgs_to_tx >= 16 && waited_ms < timeout_ms) {
        int64_t now = esp_timer_get_time();
        if (now - g_last_qfull_log_us > 500000) {
            ESP_LOGW(g_can_manager_tag,
                     "tx throttled: queue high txq=%" PRIu32 " state=%d tec=%" PRIu32 " bus_err=%" PRIu32,
                     st.msgs_to_tx,
                     (int)st.state,
                     st.tx_error_counter,
                     st.bus_error_count);
            g_last_qfull_log_us = now;
        }
        can_manager_try_hard_recover(&st);
        vTaskDelay(pdMS_TO_TICKS(1));
        waited_ms += 1;
    }

    if (twai_get_status_info(&st) == ESP_OK && st.msgs_to_tx >= 16) {
        g_throttle_streak++;
        if (g_throttle_streak >= 6) {
            can_manager_try_hard_recover(&st);
            g_throttle_streak = 0;
        }
        can_manager_try_hard_recover(&st);
        can_manager_tx_backoff_ms(60);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    for (int attempt = 0; attempt < 3; ++attempt) {
        err = twai_transmit(msg, pdMS_TO_TICKS(timeout_ms + (attempt * 4)));
        if (err == ESP_OK) {
            break;
        }

        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
            can_manager_try_recover_on_timeout();
            if (err == ESP_ERR_INVALID_STATE) {
                twai_status_info_t st_now = {0};
                if (twai_get_status_info(&st_now) == ESP_OK) {
                    can_manager_try_hard_recover(&st_now);
                }
                can_manager_tx_backoff_ms(120);
            }
            if (attempt < 2) {
                vTaskDelay(pdMS_TO_TICKS(3 + attempt * 3));
                continue;
            }
        }
        break;
    }

    if (err != ESP_OK) {
        maybe_log_tx_fail(msg->identifier, err);
    } else {
        g_throttle_streak = 0;
        if (g_tx_fail_suppressed > 0 && (esp_timer_get_time() - g_last_tx_fail_log_us) > 500000) {
            ESP_LOGI(g_can_manager_tag, "tx recovered after suppressing %" PRIu32 " fail logs", g_tx_fail_suppressed);
            g_tx_fail_suppressed = 0;
        }
    }
    return err;
}
