#include "app/serial_cli.h"

#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/serial_cli_commands.h"
#include "app/serial_cli_internal.h"
#include "app/serial_cli_output.h"
#include "app/serial_cli_watch.h"

const char *g_serial_cli_tag = "serial_cli";
volatile bool g_serial_cli_watch_enabled = false;
int g_serial_cli_watch_period_ms = 1000;
int64_t g_serial_cli_last_watch_us = 0;
bool g_serial_cli_watch_active_poll = false;
int g_serial_cli_watch_poll_wait_ms = 20;

int serial_cli_parse_int(const char *s, int def)
{
    if (s == NULL || *s == '\0') {
        return def;
    }
    return (int)strtol(s, NULL, 10);
}

static void task_serial_cli(void *arg)
{
    (void)arg;
    char line[128] = {0};
    size_t idx = 0;

    ESP_LOGI(g_serial_cli_tag, "ready. type 'help' or 'scan 1 10'");

    while (1) {
        serial_cli_watch_tick();

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
            serial_cli_handle_line(line);
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
    serial_cli_print_help();
    if (xTaskCreatePinnedToCore(task_serial_cli, "serial_cli", 4096, NULL, 4, NULL, tskNO_AFFINITY) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
