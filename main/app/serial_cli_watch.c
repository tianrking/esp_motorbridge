#include "app/serial_cli_watch.h"

#include "driver/twai.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "app/serial_cli_internal.h"
#include "app/serial_cli_output.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

typedef struct {
    uint8_t ids[MOTORBRIDGE_MAX_MOTOR_ID + 1];
    int count;
} id_list_ctx_t;

static void collect_online_ids_cb(motor_state_t *m, void *ctx)
{
    id_list_ctx_t *c = (id_list_ctx_t *)ctx;
    if (m == NULL || c == NULL || !m->online) {
        return;
    }
    if (c->count < (int)(sizeof(c->ids) / sizeof(c->ids[0]))) {
        c->ids[c->count++] = m->id;
    }
}

bool serial_cli_active_poll_id(uint8_t id, int wait_ms)
{
    motor_state_t m = {0};
    if (!motor_manager_get_state(id, &m) || m.vendor == NULL || m.vendor->build_scan_request == NULL) {
        return false;
    }

    twai_message_t req = {0};
    m.vendor->build_scan_request(id, &req);
    if (can_manager_send(&req, 8) != ESP_OK) {
        return false;
    }
    return command_router_wait_feedback(id, wait_ms);
}

void serial_cli_active_poll_online_states(int wait_ms)
{
    id_list_ctx_t c = {0};
    motor_manager_for_each(collect_online_ids_cb, &c);
    for (int i = 0; i < c.count; ++i) {
        (void)serial_cli_active_poll_id(c.ids[i], wait_ms);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void serial_cli_watch_tick(void)
{
    if (!g_serial_cli_watch_enabled) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (g_serial_cli_last_watch_us != 0 &&
        (now_us - g_serial_cli_last_watch_us) < ((int64_t)g_serial_cli_watch_period_ms * 1000LL)) {
        return;
    }

    if (g_serial_cli_watch_active_poll) {
        serial_cli_active_poll_online_states(g_serial_cli_watch_poll_wait_ms);
    }
    serial_cli_log_online_states();
    g_serial_cli_last_watch_us = now_us;
}
