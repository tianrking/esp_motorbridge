#include "can_manager.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config/app_config.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "can_manager";

static can_rx_callback_t s_rx_cb;
static int s_scan_min = MOTORBRIDGE_MIN_MOTOR_ID;
static int s_scan_max = MOTORBRIDGE_MAX_MOTOR_ID;
static bool s_dump_enabled = false;

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
        for (int id = s_scan_min; id <= s_scan_max; ++id) {
            twai_message_t msg;
            vendor->build_scan_request((uint8_t)id, &msg);
            (void)twai_transmit(&msg, pdMS_TO_TICKS(10));
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    esp_err_t err = twai_transmit(msg, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tx failed id=0x%03" PRIX32 " err=0x%x", msg->identifier, (unsigned int)err);
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
    ESP_LOGI(TAG, "scan range updated [%d, %d]", s_scan_min, s_scan_max);
}

void can_manager_set_dump(bool enabled)
{
    s_dump_enabled = enabled;
    ESP_LOGI(TAG, "candump %s", enabled ? "on" : "off");
}
