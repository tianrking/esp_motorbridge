#include "can_manager.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config/app_config.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "can_manager";

static can_rx_callback_t s_rx_cb;
static int s_scan_min = MOTORBRIDGE_MIN_MOTOR_ID;
static int s_scan_max = MOTORBRIDGE_MAX_MOTOR_ID;
static bool s_dump_enabled = false;
static int64_t s_scan_until_us = 0;
static bool s_recovery_in_progress = false;
static int64_t s_last_rx_us = 0;
static int64_t s_last_qfull_log_us = 0;
static int64_t s_last_hard_recover_us = 0;
static int s_throttle_streak = 0;
static int64_t s_tx_block_until_us = 0;
static int64_t s_last_tx_fail_log_us = 0;
static uint32_t s_tx_fail_suppressed = 0;
static int64_t s_last_status_log_us = 0;
static int64_t s_recovery_start_us = 0;
static int64_t s_last_busoff_reset_us = 0;

static void tx_backoff_ms(int32_t ms)
{
    if (ms <= 0) {
        return;
    }
    int64_t until = esp_timer_get_time() + (int64_t)ms * 1000;
    if (until > s_tx_block_until_us) {
        s_tx_block_until_us = until;
    }
}

static void maybe_log_tx_fail(uint32_t id, esp_err_t err)
{
    const int64_t now = esp_timer_get_time();
    if ((now - s_last_tx_fail_log_us) > 200000) {
        if (s_tx_fail_suppressed > 0) {
            ESP_LOGW(TAG, "tx fail storm: suppressed=%" PRIu32, s_tx_fail_suppressed);
            s_tx_fail_suppressed = 0;
        }
        ESP_LOGW(TAG, "tx failed id=0x%03" PRIX32 " err=0x%x", id, (unsigned int)err);
        s_last_tx_fail_log_us = now;
    } else {
        s_tx_fail_suppressed++;
    }
}

static void maybe_log_twai_status(const twai_status_info_t *st, bool force)
{
    if (st == NULL) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (force || (now - s_last_status_log_us) > 300000) {
        ESP_LOGW(TAG,
                 "twai status: state=%d txq=%" PRIu32 " rxq=%" PRIu32
                 " tec=%" PRIu32 " rec=%" PRIu32 " tx_failed=%" PRIu32 " bus_err=%" PRIu32,
                 (int)st->state,
                 st->msgs_to_tx,
                 st->msgs_to_rx,
                 st->tx_error_counter,
                 st->rx_error_counter,
                 st->tx_failed_count,
                 st->bus_error_count);
        s_last_status_log_us = now;
    }
}

static void try_recover_on_timeout(void)
{
    twai_status_info_t st = {0};
    if (twai_get_status_info(&st) != ESP_OK) {
        return;
    }
    maybe_log_twai_status(&st, false);

    if (st.msgs_to_tx > 0) {
        (void)twai_clear_transmit_queue();
    }

    if (st.state == TWAI_STATE_BUS_OFF && !s_recovery_in_progress) {
        if (twai_initiate_recovery() == ESP_OK) {
            s_recovery_in_progress = true;
            s_recovery_start_us = esp_timer_get_time();
            ESP_LOGW(TAG, "TWAI bus-off: recovery initiated");
            tx_backoff_ms(120);
        }
    } else if (st.state == TWAI_STATE_BUS_OFF && s_recovery_in_progress) {
        const int64_t now = esp_timer_get_time();
        if ((now - s_recovery_start_us) > 600000 && (now - s_last_busoff_reset_us) > 1000000) {
            s_last_busoff_reset_us = now;
            ESP_LOGW(TAG, "bus-off recovery timeout, force stop/start");
            (void)twai_clear_transmit_queue();
            (void)twai_stop();
            vTaskDelay(pdMS_TO_TICKS(30));
            (void)twai_start();
            s_recovery_in_progress = false;
            tx_backoff_ms(220);
        }
    } else if (st.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) {
            ESP_LOGW(TAG, "TWAI was stopped, restarted");
            tx_backoff_ms(80);
        }
    } else if (st.state == TWAI_STATE_RUNNING) {
        s_recovery_in_progress = false;
        s_recovery_start_us = 0;
    }
}

static void try_hard_recover(const twai_status_info_t *st)
{
    if (st == NULL) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (now - s_last_hard_recover_us < 1000000LL) {
        return;
    }

    // High queue + elevated TEC means bus is unhealthy but not yet bus-off.
    if (st->msgs_to_tx < 16 || st->tx_error_counter < 96) {
        return;
    }

    s_last_hard_recover_us = now;
    ESP_LOGW(TAG,
             "hard recover: stop/start TWAI (state=%d txq=%" PRIu32 " tec=%" PRIu32 " bus_err=%" PRIu32 ")",
             (int)st->state,
             st->msgs_to_tx,
             st->tx_error_counter,
             st->bus_error_count);
    (void)twai_clear_transmit_queue();
    (void)twai_stop();
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)twai_start();
    s_recovery_in_progress = false;
    tx_backoff_ms(200);
}

static twai_timing_config_t timing_for_bitrate(int bitrate)
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

static void task_can_rx(void *arg)
{
    (void)arg;
    twai_message_t msg;
    while (1) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            s_last_rx_us = esp_timer_get_time();
            if (s_dump_enabled) {
                ESP_LOGI(TAG,
                         "rx id=0x%03" PRIX32 " dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                         msg.identifier,
                         msg.data_length_code,
                         msg.data[0],
                         msg.data[1],
                         msg.data[2],
                         msg.data[3],
                         msg.data[4],
                         msg.data[5],
                         msg.data[6],
                         msg.data[7]);
            }
            if (s_rx_cb != NULL) {
                s_rx_cb(&msg);
            }
        }
    }
}

#include "core/motor_manager.h"

static void task_scan(void *arg)
{
    (void)arg;
    while (1) {
        int64_t now = esp_timer_get_time();
        if (now >= s_scan_until_us) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        for (int id = s_scan_min; id <= s_scan_max && esp_timer_get_time() < s_scan_until_us; ++id) {
            motor_state_t m;
            if (motor_manager_get_state(id, &m) && m.vendor != NULL && m.vendor->build_scan_request != NULL) {
                twai_message_t msg;
                m.vendor->build_scan_request((uint8_t)id, &msg);
                (void)twai_transmit(&msg, pdMS_TO_TICKS(8));
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

esp_err_t can_manager_init(int tx_gpio, int rx_gpio, int bitrate)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = timing_for_bitrate(bitrate);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.tx_queue_len = 64;
    g_config.rx_queue_len = 64;

    ESP_LOGI(TAG, "twai install tx=%d rx=%d bitrate=%d", tx_gpio, rx_gpio, bitrate);
    return twai_driver_install(&g_config, &t_config, &f_config);
}

esp_err_t can_manager_start(void)
{
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        return err;
    }

    xTaskCreatePinnedToCore(task_can_rx, "can_rx", 4096, NULL, 21, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_scan, "can_scan", 3072, NULL, 6, NULL, tskNO_AFFINITY);
    return ESP_OK;
}

void can_manager_set_rx_callback(can_rx_callback_t cb)
{
    s_rx_cb = cb;
}

esp_err_t can_manager_send(const twai_message_t *msg, int timeout_ms)
{
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dump_enabled) {
        ESP_LOGI(TAG,
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
    if (now_us < s_tx_block_until_us) {
        int32_t wait_ms = (int32_t)((s_tx_block_until_us - now_us + 999) / 1000);
        if (wait_ms > timeout_ms) {
            wait_ms = timeout_ms;
        }
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }

    twai_status_info_t st = {0};
    if (twai_get_status_info(&st) == ESP_OK && st.state != TWAI_STATE_RUNNING) {
        maybe_log_twai_status(&st, false);
        try_recover_on_timeout();
        tx_backoff_ms(60);
        return ESP_ERR_INVALID_STATE;
    }

    int waited_ms = 0;
    while (twai_get_status_info(&st) == ESP_OK && st.msgs_to_tx >= 16 && waited_ms < timeout_ms) {
        int64_t now = esp_timer_get_time();
        if (now - s_last_qfull_log_us > 500000) {
            ESP_LOGW(TAG,
                     "tx throttled: queue high txq=%" PRIu32 " state=%d tec=%" PRIu32 " bus_err=%" PRIu32,
                     st.msgs_to_tx,
                     (int)st.state,
                     st.tx_error_counter,
                     st.bus_error_count);
            s_last_qfull_log_us = now;
        }
        try_hard_recover(&st);
        vTaskDelay(pdMS_TO_TICKS(1));
        waited_ms += 1;
    }
    if (twai_get_status_info(&st) == ESP_OK && st.msgs_to_tx >= 16) {
        s_throttle_streak++;
        if (s_throttle_streak >= 6) {
            try_hard_recover(&st);
            s_throttle_streak = 0;
        }
        try_hard_recover(&st);
        tx_backoff_ms(60);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    for (int attempt = 0; attempt < 3; ++attempt) {
        err = twai_transmit(msg, pdMS_TO_TICKS(timeout_ms + (attempt * 4)));
        if (err == ESP_OK) {
            break;
        }

        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
            try_recover_on_timeout();
            if (err == ESP_ERR_INVALID_STATE) {
                twai_status_info_t st_now = {0};
                if (twai_get_status_info(&st_now) == ESP_OK) {
                    try_hard_recover(&st_now);
                }
                tx_backoff_ms(120);
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
        s_throttle_streak = 0;
        if (s_tx_fail_suppressed > 0 && (esp_timer_get_time() - s_last_tx_fail_log_us) > 500000) {
            ESP_LOGI(TAG, "tx recovered after suppressing %" PRIu32 " fail logs", s_tx_fail_suppressed);
            s_tx_fail_suppressed = 0;
        }
    }
    return err;
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

    s_scan_min = min_id;
    s_scan_max = max_id;
    s_scan_until_us = esp_timer_get_time() + 2000000LL; // keep scan active for 2s
    ESP_LOGI(TAG, "scan range updated [%d, %d]", s_scan_min, s_scan_max);
}

void can_manager_set_dump(bool enabled)
{
    s_dump_enabled = enabled;
    ESP_LOGI(TAG, "candump %s", enabled ? "on" : "off");
}

bool can_manager_recent_rx(int32_t within_ms)
{
    if (within_ms <= 0) {
        return false;
    }
    int64_t last = s_last_rx_us;
    if (last <= 0) {
        return false;
    }
    return (esp_timer_get_time() - last) <= ((int64_t)within_ms * 1000);
}
