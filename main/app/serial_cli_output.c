#include "app/serial_cli_output.h"

#include <inttypes.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "app/command_router.h"
#include "app/serial_cli_internal.h"
#include "config/app_config.h"
#include "core/motor_manager.h"

typedef struct {
    int min_id;
    int max_id;
    char ids_buf[128];
    size_t used;
    int count;
} list_ctx_t;

static void append_id(char *buf, size_t cap, size_t *used, uint8_t id)
{
    if (buf == NULL || used == NULL || *used >= cap) {
        return;
    }
    int w = snprintf(buf + *used, cap - *used, "%s%u", (*used == 0) ? "" : ",", id);
    if (w > 0) {
        *used += (size_t)w;
        if (*used > cap) {
            *used = cap;
        }
    }
}

static void collect_online_cb(motor_state_t *m, void *ctx)
{
    list_ctx_t *c = (list_ctx_t *)ctx;
    if (m == NULL || c == NULL) {
        return;
    }
    if (m->online && m->id >= c->min_id && m->id <= c->max_id) {
        append_id(c->ids_buf, sizeof(c->ids_buf), &c->used, m->id);
        c->count++;
    }
}

void serial_cli_log_online_ids(int min_id, int max_id)
{
    list_ctx_t c = {
        .min_id = min_id,
        .max_id = max_id,
    };
    motor_manager_for_each(collect_online_cb, &c);
    ESP_LOGI(g_serial_cli_tag, "online ids in [%d,%d]: %s (count=%d)", min_id, max_id, c.count > 0 ? c.ids_buf : "none", c.count);
}

void serial_cli_log_seen_ids(int min_id, int max_id, int32_t recent_ms)
{
    uint8_t ids[MOTORBRIDGE_MAX_MOTOR_ID] = {0};
    int n = command_router_collect_seen_ids(min_id, max_id, ids, MOTORBRIDGE_MAX_MOTOR_ID, recent_ms);
    char buf[128] = {0};
    size_t used = 0;
    for (int i = 0; i < n; ++i) {
        append_id(buf, sizeof(buf), &used, ids[i]);
    }
    ESP_LOGI(g_serial_cli_tag, "scan seen ids in [%d,%d] (last %dms): %s (count=%d)", min_id, max_id, recent_ms, n > 0 ? buf : "none", n);
}

typedef struct {
    char line[512];
    size_t used;
    int count;
    int64_t now_us;
    int32_t stale_ms;
} state_dump_ctx_t;

static void append_text(char *buf, size_t cap, size_t *used, const char *text)
{
    if (buf == NULL || used == NULL || text == NULL || *used >= cap) {
        return;
    }
    int w = snprintf(buf + *used, cap - *used, "%s", text);
    if (w > 0) {
        *used += (size_t)w;
        if (*used > cap) {
            *used = cap;
        }
    }
}

static void collect_state_cb(motor_state_t *m, void *ctx)
{
    state_dump_ctx_t *c = (state_dump_ctx_t *)ctx;
    if (m == NULL || c == NULL || !m->online) {
        return;
    }

    char item[96];
    int64_t age_ms = 0;
    if (m->last_seen_us > 0 && c->now_us > m->last_seen_us) {
        age_ms = (c->now_us - m->last_seen_us) / 1000;
    }
    const bool stale = (c->stale_ms > 0) && (age_ms > c->stale_ms);
    snprintf(item, sizeof(item), "%s%u:pos=%.3f vel=%.3f m=%d age=%" PRId64 "ms%s",
             (c->count == 0) ? "" : " | ",
             m->id,
             m->position,
             m->speed,
             (int)m->mode,
             age_ms,
             stale ? " stale" : "");
    append_text(c->line, sizeof(c->line), &c->used, item);
    c->count++;
}

void serial_cli_log_online_states(void)
{
    const app_config_t *cfg = app_config_get();
    state_dump_ctx_t c = {
        .now_us = esp_timer_get_time(),
        .stale_ms = (cfg != NULL) ? cfg->offline_timeout_ms : 0,
    };
    motor_manager_for_each(collect_state_cb, &c);
    ESP_LOGI(g_serial_cli_tag, "online states: %s (count=%d)", c.count > 0 ? c.line : "none", c.count);
}

void serial_cli_log_one_state(uint8_t id)
{
    motor_state_t m = {0};
    if (!motor_manager_get_state(id, &m)) {
        ESP_LOGW(g_serial_cli_tag, "state id=%u unavailable", (unsigned int)id);
        return;
    }
    int64_t now_us = esp_timer_get_time();
    int64_t age_ms = (m.last_seen_us > 0 && now_us > m.last_seen_us) ? (now_us - m.last_seen_us) / 1000 : 0;
    ESP_LOGI(g_serial_cli_tag,
             "state id=%u online=%d en=%d mode=%d pos=%.5f vel=%.5f tau=%.5f age=%" PRId64 "ms",
             m.id,
             m.online ? 1 : 0,
             m.enabled ? 1 : 0,
             (int)m.mode,
             m.position,
             m.speed,
             m.torque,
             age_ms);
}

void serial_cli_print_help(void)
{
    ESP_LOGI(g_serial_cli_tag, "commands:");
    ESP_LOGI(g_serial_cli_tag, "  help");
    ESP_LOGI(g_serial_cli_tag, "  scan [vendor] <min_id> <max_id> e.g. scan robstride 1 10");
    ESP_LOGI(g_serial_cli_tag, "  show                        list online motor ids");
    ESP_LOGI(g_serial_cli_tag, "  show_state                  list online ids with pos/vel/mode");
    ESP_LOGI(g_serial_cli_tag, "  state <id>                  show one motor state");
    ESP_LOGI(g_serial_cli_tag, "  watch <on|off> [ms] [poll]  periodic show_state; add 'poll' for active query");
    ESP_LOGI(g_serial_cli_tag, "  poll <id|all> [wait_ms]     actively request feedback then print");
    ESP_LOGI(g_serial_cli_tag, "  clear_online                clear online cache (then rescan)");
    ESP_LOGI(g_serial_cli_tag, "  candump <on|off>            print raw CAN RX frames");
    ESP_LOGI(g_serial_cli_tag, "  enable <id|all>");
    ESP_LOGI(g_serial_cli_tag, "  disable <id|all>");
    ESP_LOGI(g_serial_cli_tag, "  mode <id|all> <0..4>        1=MIT 2=PosVel 3=Vel 4=ForcePos");
    ESP_LOGI(g_serial_cli_tag, "  setid <old_id> <new_id>     Damiao only, store+verify");
}
