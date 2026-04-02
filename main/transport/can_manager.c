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

static void try_recover_on_timeout(void)
{
    twai_status_info_t st = {0};
    if (twai_get_status_info(&st) != ESP_OK) {
        return;
    }

    ESP_LOGW(TAG,
             "twai status: state=%d txq=%" PRIu32 " rxq=%" PRIu32
             " tec=%" PRIu32 " rec=%" PRIu32 " tx_failed=%" PRIu32 " bus_err=%" PRIu32,
             (int)st.state,
             st.msgs_to_tx,
             st.msgs_to_rx,
             st.tx_error_counter,
             st.rx_error_counter,
             st.tx_failed_count,
             st.bus_error_count);

    if (st.msgs_to_tx > 0) {
        (void)twai_clear_transmit_queue();
    }

    if (st.state == TWAI_STATE_BUS_OFF && !s_recovery_in_progress) {
        if (twai_initiate_recovery() == ESP_OK) {
            s_recovery_in_progress = true;
            ESP_LOGW(TAG, "TWAI bus-off: recovery initiated");
        }
    } else if (st.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) {
            ESP_LOGW(TAG, "TWAI was stopped, restarted");
        }
    } else if (st.state == TWAI_STATE_RUNNING) {
        s_recovery_in_progress = false;
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

static void task_scan(void *arg)
{
    (void)arg;
    const motor_vendor_ops_t *vendor = motor_vendor_active();
    while (1) {
        int64_t now = esp_timer_get_time();
        if (now >= s_scan_until_us) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        for (int id = s_scan_min; id <= s_scan_max && esp_timer_get_time() < s_scan_until_us; ++id) {
            twai_message_t msg;
            vendor->build_scan_request((uint8_t)id, &msg);
            (void)twai_transmit(&msg, pdMS_TO_TICKS(8));
            vTaskDelay(pdMS_TO_TICKS(2));
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

    twai_status_info_t st = {0};
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
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = twai_transmit(msg, pdMS_TO_TICKS(timeout_ms));
    if (err == ESP_ERR_TIMEOUT) {
        // Retry once with a slightly longer wait to reduce transient queue/bus contention.
        err = twai_transmit(msg, pdMS_TO_TICKS(timeout_ms + 6));
        if (err == ESP_ERR_TIMEOUT) {
            try_recover_on_timeout();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tx failed id=0x%03" PRIX32 " err=0x%x", msg->identifier, (unsigned int)err);
    } else {
        s_throttle_streak = 0;
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
