#include "transport/can_manager_internal.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static void task_scan(void *arg)
{
    (void)arg;
    while (1) {
        int64_t now = esp_timer_get_time();
        if (now >= g_scan_until_us) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        for (int id = g_scan_min; id <= g_scan_max && esp_timer_get_time() < g_scan_until_us; ++id) {
            motor_state_t m;
            if (motor_manager_get_state(id, &m) && m.vendor != NULL && m.vendor->build_scan_request != NULL) {
                twai_message_t msg;
                m.vendor->build_scan_request((uint8_t)id, &msg);
                (void)can_manager_send(&msg, 8);
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

void can_manager_start_scan_task(void)
{
    xTaskCreatePinnedToCore(task_scan, "can_scan", 3072, NULL, 6, NULL, tskNO_AFFINITY);
}
