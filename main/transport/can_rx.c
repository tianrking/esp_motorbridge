#include "transport/can_manager_internal.h"

#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void task_can_rx(void *arg)
{
    (void)arg;
    twai_message_t msg;
    while (1) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            g_last_rx_us = esp_timer_get_time();
            if (g_dump_enabled) {
                ESP_LOGI(g_can_manager_tag,
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
            if (g_can_rx_cb != NULL) {
                g_can_rx_cb(&msg);
            }
        }
    }
}

void can_manager_start_rx_task(void)
{
    xTaskCreatePinnedToCore(task_can_rx, "can_rx", 4096, NULL, 21, NULL, tskNO_AFFINITY);
}
