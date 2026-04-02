#include "app/serial_cli.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"

static const char *TAG = "serial_cli";

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
    (void)can_manager_send(&msg, 10);
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
    ESP_LOGI(TAG, "  scan <min_id> <max_id>      e.g. scan 1 10");
    ESP_LOGI(TAG, "  show                        list online motor ids");
    ESP_LOGI(TAG, "  candump <on|off>            print raw CAN RX frames");
    ESP_LOGI(TAG, "  enable <id|all>");
    ESP_LOGI(TAG, "  disable <id|all>");
    ESP_LOGI(TAG, "  mode <id|all> <0..4>        1=MIT 2=PosVel 3=Vel 4=ForcePos");
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
        int min_id = parse_int(strtok_r(NULL, " \t", &save), 1);
        int max_id = parse_int(strtok_r(NULL, " \t", &save), 10);
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
        int ch = getchar();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
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
            }
            continue;
        }

        if (idx + 1 < sizeof(line)) {
            line[idx++] = (char)ch;
            line[idx] = '\0';
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
