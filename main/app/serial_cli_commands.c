#include "app/serial_cli_commands.h"

#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "app/serial_cli_internal.h"
#include "app/serial_cli_output.h"
#include "app/serial_cli_watch.h"
#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static void send_admin_cmd(uint8_t op, uint8_t motor_id, uint8_t mode)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = op;
    msg.data[1] = motor_id;
    msg.data[2] = mode;
    command_router_handle_can_rx(&msg);
}

static void send_set_id_cmd(uint8_t old_id, uint8_t new_id, bool store, bool verify)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = 9; // HOST_OP_SET_IDS
    msg.data[1] = old_id;
    msg.data[2] = new_id;
    msg.data[3] = new_id;
    msg.data[4] = (store ? 0x01 : 0) | (verify ? 0x02 : 0);
    command_router_handle_can_rx(&msg);
}

void serial_cli_handle_line(char *line)
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
        serial_cli_print_help();
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
                min_id = serial_cli_parse_int(strtok_r(NULL, " \t", &save), 1);
                max_id = serial_cli_parse_int(strtok_r(NULL, " \t", &save), 10);
            } else {
                min_id = serial_cli_parse_int(arg1, 1);
                max_id = serial_cli_parse_int(strtok_r(NULL, " \t", &save), 10);
            }
        }

        if (min_id < MOTORBRIDGE_MIN_MOTOR_ID) {
            min_id = MOTORBRIDGE_MIN_MOTOR_ID;
        }
        if (max_id > MOTORBRIDGE_MAX_MOTOR_ID) {
            max_id = MOTORBRIDGE_MAX_MOTOR_ID;
        }
        if (min_id > max_id) {
            ESP_LOGW(g_serial_cli_tag, "bad range: %d > %d", min_id, max_id);
            return;
        }

        if (vendor != NULL) {
            int changed = 0;
            for (int i = min_id; i <= max_id; ++i) {
                if (motor_manager_set_vendor((uint8_t)i, vendor)) {
                    changed++;
                }
            }
            ESP_LOGI(g_serial_cli_tag, "dynamically mapped %d slots to vendor '%s'", changed, vendor);
        }

        can_manager_trigger_scan(min_id, max_id);
        ESP_LOGI(g_serial_cli_tag, "scan start [%d,%d], waiting 1200ms...", min_id, max_id);
        vTaskDelay(pdMS_TO_TICKS(1200));
        serial_cli_log_seen_ids(min_id, max_id, 2000);
        return;
    }

    if (strcmp(cmd, "show") == 0) {
        const app_config_t *cfg = app_config_get();
        serial_cli_log_online_ids(MOTORBRIDGE_MIN_MOTOR_ID, cfg->max_motors);
        return;
    }

    if (strcmp(cmd, "show_state") == 0) {
        serial_cli_log_online_states();
        return;
    }

    if (strcmp(cmd, "state") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int id = serial_cli_parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(g_serial_cli_tag, "usage: state <id>");
            return;
        }
        serial_cli_log_one_state((uint8_t)id);
        return;
    }

    if (strcmp(cmd, "watch") == 0) {
        char *arg = strtok_r(NULL, " \t", &save);
        if (arg == NULL) {
            ESP_LOGW(g_serial_cli_tag, "usage: watch <on|off> [ms] [poll]");
            return;
        }
        if (strcmp(arg, "on") == 0) {
            int ms = serial_cli_parse_int(strtok_r(NULL, " \t", &save), 1000);
            char *mode_s = strtok_r(NULL, " \t", &save);
            if (ms < 100) {
                ms = 100;
            }
            g_serial_cli_watch_period_ms = ms;
            g_serial_cli_last_watch_us = 0;
            g_serial_cli_watch_active_poll = (mode_s != NULL && strcmp(mode_s, "poll") == 0);
            g_serial_cli_watch_enabled = true;
            ESP_LOGI(g_serial_cli_tag,
                     "watch on, period=%dms, mode=%s",
                     g_serial_cli_watch_period_ms,
                     g_serial_cli_watch_active_poll ? "active-poll" : "cache");
            return;
        }
        if (strcmp(arg, "off") == 0) {
            g_serial_cli_watch_enabled = false;
            ESP_LOGI(g_serial_cli_tag, "watch off");
            return;
        }
        ESP_LOGW(g_serial_cli_tag, "usage: watch <on|off> [ms] [poll]");
        return;
    }

    if (strcmp(cmd, "poll") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int wait_ms = serial_cli_parse_int(strtok_r(NULL, " \t", &save), 20);
        if (wait_ms < 5) {
            wait_ms = 5;
        }
        if (wait_ms > 200) {
            wait_ms = 200;
        }
        if (id_s == NULL) {
            ESP_LOGW(g_serial_cli_tag, "usage: poll <id|all> [wait_ms]");
            return;
        }
        if (strcmp(id_s, "all") == 0) {
            serial_cli_active_poll_online_states(wait_ms);
            serial_cli_log_online_states();
            return;
        }
        int id = serial_cli_parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(g_serial_cli_tag, "usage: poll <id|all> [wait_ms]");
            return;
        }
        (void)serial_cli_active_poll_id((uint8_t)id, wait_ms);
        serial_cli_log_one_state((uint8_t)id);
        return;
    }

    if (strcmp(cmd, "clear_online") == 0) {
        motor_manager_clear_online_all();
        ESP_LOGI(g_serial_cli_tag, "online cache cleared; run scan <vendor> <min> <max>");
        return;
    }

    if (strcmp(cmd, "candump") == 0) {
        char *arg = strtok_r(NULL, " \t", &save);
        if (arg == NULL) {
            ESP_LOGW(g_serial_cli_tag, "usage: candump <on|off>");
            return;
        }
        if (strcmp(arg, "on") == 0) {
            can_manager_set_dump(true);
        } else if (strcmp(arg, "off") == 0) {
            can_manager_set_dump(false);
        } else {
            ESP_LOGW(g_serial_cli_tag, "usage: candump <on|off>");
        }
        return;
    }

    if (strcmp(cmd, "enable") == 0 || strcmp(cmd, "disable") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        if (id_s == NULL) {
            ESP_LOGW(g_serial_cli_tag, "missing id, use <id|all>");
            return;
        }
        uint8_t op = (strcmp(cmd, "enable") == 0) ? 4 : 5;
        if (strcmp(id_s, "all") == 0) {
            for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
                send_admin_cmd(op, (uint8_t)i, 0);
            }
            ESP_LOGI(g_serial_cli_tag, "%s all", cmd);
            return;
        }
        int id = serial_cli_parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(g_serial_cli_tag, "id out of range: %d", id);
            return;
        }
        send_admin_cmd(op, (uint8_t)id, 0);
        ESP_LOGI(g_serial_cli_tag, "%s id=%d", cmd, id);
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char *id_s = strtok_r(NULL, " \t", &save);
        int mode = serial_cli_parse_int(strtok_r(NULL, " \t", &save), -1);
        if (id_s == NULL || mode < 0 || mode > 4) {
            ESP_LOGW(g_serial_cli_tag, "usage: mode <id|all> <0..4>");
            return;
        }
        if (strcmp(id_s, "all") == 0) {
            for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
                send_admin_cmd(1, (uint8_t)i, (uint8_t)mode);
            }
            ESP_LOGI(g_serial_cli_tag, "set mode=%d for all", mode);
            return;
        }
        int id = serial_cli_parse_int(id_s, -1);
        if (id < MOTORBRIDGE_MIN_MOTOR_ID || id > motor_manager_count()) {
            ESP_LOGW(g_serial_cli_tag, "id out of range: %d", id);
            return;
        }
        send_admin_cmd(1, (uint8_t)id, (uint8_t)mode);
        ESP_LOGI(g_serial_cli_tag, "set mode=%d id=%d", mode, id);
        return;
    }

    if (strcmp(cmd, "setid") == 0) {
        int old_id = serial_cli_parse_int(strtok_r(NULL, " \t", &save), -1);
        int new_id = serial_cli_parse_int(strtok_r(NULL, " \t", &save), -1);
        if (old_id < MOTORBRIDGE_MIN_MOTOR_ID || old_id > motor_manager_count() ||
            new_id < MOTORBRIDGE_MIN_MOTOR_ID || new_id > motor_manager_count() ||
            old_id == new_id) {
            ESP_LOGW(g_serial_cli_tag, "usage: setid <old_id> <new_id>");
            return;
        }

        motor_state_t old_m = {0};
        if (!motor_manager_get_state((uint8_t)old_id, &old_m) ||
            old_m.vendor == NULL ||
            old_m.vendor->name == NULL ||
            strcmp(old_m.vendor->name, "damiao") != 0) {
            ESP_LOGW(g_serial_cli_tag, "setid only supports damiao-mapped old_id");
            return;
        }

        motor_state_t new_m = {0};
        if (motor_manager_get_state((uint8_t)new_id, &new_m) && new_m.online) {
            ESP_LOGW(g_serial_cli_tag, "new_id=%d appears online; abort to avoid ID conflict", new_id);
            return;
        }

        send_admin_cmd(5, (uint8_t)old_id, 0);
        vTaskDelay(pdMS_TO_TICKS(30));
        send_set_id_cmd((uint8_t)old_id, (uint8_t)new_id, true, true);
        vTaskDelay(pdMS_TO_TICKS(40));
        (void)motor_manager_set_vendor((uint8_t)new_id, "damiao");
        can_manager_trigger_scan(new_id, new_id);
        ESP_LOGI(g_serial_cli_tag, "setid requested old=%d -> new=%d (damiao, store+verify)", old_id, new_id);
        return;
    }

    ESP_LOGW(g_serial_cli_tag, "unknown cmd: %s", cmd);
    serial_cli_print_help();
}
