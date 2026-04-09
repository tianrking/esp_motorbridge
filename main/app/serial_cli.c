#include "app/serial_cli.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "serial_cli";
static volatile bool s_watch_enabled = false;
static int s_watch_period_ms = 1000;
static int64_t s_last_watch_us = 0;
static bool s_watch_active_poll = false;
static int s_watch_poll_wait_ms = 20;

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

static void log_online_ids(int min_id, int max_id)
{
    list_ctx_t c = {
        .min_id = min_id,
        .max_id = max_id,
    };
    motor_manager_for_each(collect_online_cb, &c);
    ESP_LOGI(TAG, "online ids in [%d,%d]: %s (count=%d)", min_id, max_id, c.count > 0 ? c.ids_buf : "none", c.count);
}

static void log_seen_ids(int min_id, int max_id, int32_t recent_ms)
{
    uint8_t ids[MOTORBRIDGE_MAX_MOTOR_ID] = {0};
    int n = command_router_collect_seen_ids(min_id, max_id, ids, MOTORBRIDGE_MAX_MOTOR_ID, recent_ms);
    char buf[128] = {0};
    size_t used = 0;
    for (int i = 0; i < n; ++i) {
        append_id(buf, sizeof(buf), &used, ids[i]);
    }
    ESP_LOGI(TAG, "scan seen ids in [%d,%d] (last %dms): %s (count=%d)", min_id, max_id, recent_ms, n > 0 ? buf : "none", n);
}

static void send_admin_cmd(uint8_t op, uint8_t motor_id, uint8_t mode)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = op;
    msg.data[1] = motor_id;
    msg.data[2] = mode;
    // Route host-style admin cmd locally so command_router can translate it
    // into the bound vendor protocol frame for each motor.
    command_router_handle_can_rx(&msg);
}

static int parse_int(const char *s, int def)
{
    if (s == NULL || *s == '\0') {
        return def;
    }
    return (int)strtol(s, NULL, 10);
}

static void print_help(void)
{
    ESP_LOGI(TAG, "commands:");
    ESP_LOGI(TAG, "  help");
    ESP_LOGI(TAG, "  scan [vendor] <min_id> <max_id> e.g. scan robstride 1 10");
    ESP_LOGI(TAG, "  show                        list online motor ids");
    ESP_LOGI(TAG, "  show_state                  list online ids with pos/vel/mode");
    ESP_LOGI(TAG, "  state <id>                  show one motor state");
    ESP_LOGI(TAG, "  watch <on|off> [ms] [poll]  periodic show_state; add 'poll' for active query");
    ESP_LOGI(TAG, "  poll <id|all> [wait_ms]     actively request feedback then print");
    ESP_LOGI(TAG, "  clear_online                clear online cache (then rescan)");
    ESP_LOGI(TAG, "  candump <on|off>            print raw CAN RX frames");
    ESP_LOGI(TAG, "  enable <id|all>");
    ESP_LOGI(TAG, "  disable <id|all>");
    ESP_LOGI(TAG, "  mode <id|all> <0..4>        1=MIT 2=PosVel 3=Vel 4=ForcePos");
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

static void log_online_states(void)
{
    const app_config_t *cfg = app_config_get();
    state_dump_ctx_t c = {
        .now_us = esp_timer_get_time(),
        .stale_ms = (cfg != NULL) ? cfg->offline_timeout_ms : 0,
    };
    motor_manager_for_each(collect_state_cb, &c);
    ESP_LOGI(TAG, "online states: %s (count=%d)", c.count > 0 ? c.line : "none", c.count);
}

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

static bool active_poll_id(uint8_t id, int wait_ms)
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

static void active_poll_online_states(int wait_ms)
{
    id_list_ctx_t c = {0};
    motor_manager_for_each(collect_online_ids_cb, &c);
    for (int i = 0; i < c.count; ++i) {
        (void)active_poll_id(c.ids[i], wait_ms);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void handle_line(char *line)
{
    while (*line == '\\' || *line == '/') {
        line++;
    }

    char *save = NULL;
    char *cmd = strtok_r(line, " \t", &save);
    if (cmd == NULL) {
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        print_help();
        return;
    }

    if (strcmp(cmd, "scan") == 0) {
        char *arg1 = strtok_r(NULL, " \t", &save);
        int min_id = 1;
        int max_id = 10;
        const char *vendor = NULL;

        if (arg1 != NULL) {
            if (strcmp(arg1, "damiao") == 0 || strcmp(arg1, "robstride") == 0) {
                vendor = arg1;
                min_id = parse_int(strtok_r(NULL, " \t", &save), 1);
                max_id = parse_int(strtok_r(NULL, " \t", &save), 10);
            } else {
                min_id = parse_int(arg1, 1);
                max_id = parse_int(strtok_r(NULL, " \t", &save), 10);
            }
        }

        if (min_id < MOTORBRIDGE_MIN_MOTOR_ID) {
            min_id = MOTORBRIDGE_MIN_MOTOR_ID;
        }
        if (max_id > MOTORBRIDGE_MAX_MOTOR_ID) {
            max_id = MOTORBRIDGE_MAX_MOTOR_ID;
        }
        if (min_id > max_id) {
            ESP_LOGW(TAG, "bad range: %d > %d", min_id, max_id);
            return;
        }

        if (vendor != NULL) {
            int changed = 0;
            for (int i = min_id; i <= max_id; ++i) {
                if (motor_manager_set_vendor((uint8_t)i, vendor)) {
                    changed++;
                }
            }
            ESP_LOGI(TAG, "dynamically mapped %d slots to vendor '%s'", changed, vendor);
        }

        can_manager_trigger_scan(min_id, max_id);
        ESP_LOGI(TAG, "scan start [%d,%d], waiting 1200ms...", min_id, max_id);
        vTaskDelay(pdMS_TO_TICKS(1200));
        log_seen_ids(min_id, max_id, 2000);
        return;
    }

    if (strcmp(cmd, "show") == 0) {
        const app_config_t *cfg = app_config_get();
        log_online_ids(MOTORBRIDGE_MIN_MOTOR_ID, cfg->max_motors);
        return;
    }
    if (strcmp(cmd, "show_state") == 0) {
        log_online_states();
        return;
    }
    if (strcmp(cmd, "state") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int id = parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(TAG, "usage: state <id>");
            return;
        }
        motor_state_t m = {0};
        if (!motor_manager_get_state((uint8_t)id, &m)) {
            ESP_LOGW(TAG, "state id=%d unavailable", id);
            return;
        }
        ESP_LOGI(TAG,
                 "state id=%u online=%d en=%d mode=%d pos=%.5f vel=%.5f tau=%.5f",
                 m.id,
                 m.online ? 1 : 0,
                 m.enabled ? 1 : 0,
                 (int)m.mode,
                 m.position,
                 m.speed,
                 m.torque);
        return;
    }
    if (strcmp(cmd, "watch") == 0) {
        char *arg = strtok_r(NULL, " \t", &save);
        if (arg == NULL) {
            ESP_LOGW(TAG, "usage: watch <on|off> [ms] [poll]");
            return;
        }
        if (strcmp(arg, "on") == 0) {
            int ms = parse_int(strtok_r(NULL, " \t", &save), 1000);
            char *mode_s = strtok_r(NULL, " \t", &save);
            if (ms < 100) {
                ms = 100;
            }
            s_watch_period_ms = ms;
            s_last_watch_us = 0;
            s_watch_active_poll = (mode_s != NULL && strcmp(mode_s, "poll") == 0);
            s_watch_enabled = true;
            ESP_LOGI(TAG,
                     "watch on, period=%dms, mode=%s",
                     s_watch_period_ms,
                     s_watch_active_poll ? "active-poll" : "cache");
            return;
        }
        if (strcmp(arg, "off") == 0) {
            s_watch_enabled = false;
            ESP_LOGI(TAG, "watch off");
            return;
        }
        ESP_LOGW(TAG, "usage: watch <on|off> [ms] [poll]");
        return;
    }
    if (strcmp(cmd, "poll") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int wait_ms = parse_int(strtok_r(NULL, " \t", &save), 20);
        if (wait_ms < 5) {
            wait_ms = 5;
        }
        if (wait_ms > 200) {
            wait_ms = 200;
        }
        if (id_s == NULL) {
            ESP_LOGW(TAG, "usage: poll <id|all> [wait_ms]");
            return;
        }
        if (strcmp(id_s, "all") == 0) {
            active_poll_online_states(wait_ms);
            log_online_states();
            return;
        }
        int id = parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(TAG, "usage: poll <id|all> [wait_ms]");
            return;
        }
        (void)active_poll_id((uint8_t)id, wait_ms);
        motor_state_t m = {0};
        if (motor_manager_get_state((uint8_t)id, &m)) {
            int64_t now_us = esp_timer_get_time();
            int64_t age_ms = (m.last_seen_us > 0 && now_us > m.last_seen_us) ? (now_us - m.last_seen_us) / 1000 : 0;
            ESP_LOGI(TAG,
                     "state id=%u online=%d mode=%d pos=%.5f vel=%.5f age=%" PRId64 "ms",
                     m.id,
                     m.online ? 1 : 0,
                     (int)m.mode,
                     m.position,
                     m.speed,
                     age_ms);
        }
        return;
    }
    if (strcmp(cmd, "clear_online") == 0) {
        motor_manager_clear_online_all();
        ESP_LOGI(TAG, "online cache cleared; run scan <vendor> <min> <max>");
        return;
    }

    if (strcmp(cmd, "candump") == 0) {
        char *arg = strtok_r(NULL, " \t", &save);
        if (arg == NULL) {
            ESP_LOGW(TAG, "usage: candump <on|off>");
            return;
        }
        if (strcmp(arg, "on") == 0) {
            can_manager_set_dump(true);
        } else if (strcmp(arg, "off") == 0) {
            can_manager_set_dump(false);
        } else {
            ESP_LOGW(TAG, "usage: candump <on|off>");
        }
        return;
    }

    if (strcmp(cmd, "enable") == 0 || strcmp(cmd, "disable") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        if (id_s == NULL) {
            ESP_LOGW(TAG, "missing id, use <id|all>");
            return;
        }
        uint8_t op = (strcmp(cmd, "enable") == 0) ? 4 : 5;
        if (strcmp(id_s, "all") == 0) {
            for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
                send_admin_cmd(op, (uint8_t)i, 0);
            }
            ESP_LOGI(TAG, "%s all", cmd);
            return;
        }
        int id = parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(TAG, "id out of range: %d", id);
            return;
        }
        send_admin_cmd(op, (uint8_t)id, 0);
        ESP_LOGI(TAG, "%s id=%d", cmd, id);
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int mode = parse_int(strtok_r(NULL, " \t", &save), -1);
        if (id_s == NULL || mode < 0 || mode > 4) {
            ESP_LOGW(TAG, "usage: mode <id|all> <0..4>");
            return;
        }
        if (strcmp(id_s, "all") == 0) {
            for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
                send_admin_cmd(1, (uint8_t)i, (uint8_t)mode);
            }
            ESP_LOGI(TAG, "set mode=%d for all", mode);
            return;
        }
        int id = parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(TAG, "id out of range: %d", id);
            return;
        }
        send_admin_cmd(1, (uint8_t)id, (uint8_t)mode);
        ESP_LOGI(TAG, "set mode=%d id=%d", mode, id);
        return;
    }

    ESP_LOGW(TAG, "unknown cmd: %s", cmd);
    print_help();
}

static void task_serial_cli(void *arg)
{
    (void)arg;
    char line[128] = {0};
    size_t idx = 0;
    ESP_LOGI(TAG, "ready. type 'help' or 'scan 1 10'");

    while (1) {
        if (s_watch_enabled) {
            int64_t now_us = esp_timer_get_time();
            if (s_last_watch_us == 0 || (now_us - s_last_watch_us) >= ((int64_t)s_watch_period_ms * 1000LL)) {
                if (s_watch_active_poll) {
                    active_poll_online_states(s_watch_poll_wait_ms);
                }
                log_online_states();
                s_last_watch_us = now_us;
            }
        }

        int ch = getchar();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            putchar('\n');
            fflush(stdout);
            if (idx == 0) {
                continue;
            }
            line[idx] = '\0';
            handle_line(line);
            idx = 0;
            line[0] = '\0';
            continue;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (idx > 0) {
                idx--;
                line[idx] = '\0';
                putchar('\b');
                putchar(' ');
                putchar('\b');
                fflush(stdout);
            }
            continue;
        }

        if (idx + 1 < sizeof(line)) {
            line[idx++] = (char)ch;
            line[idx] = '\0';
            putchar(ch);
            fflush(stdout);
        }
    }
}

esp_err_t serial_cli_start(void)
{
    if (xTaskCreatePinnedToCore(task_serial_cli, "serial_cli", 4096, NULL, 4, NULL, tskNO_AFFINITY) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
