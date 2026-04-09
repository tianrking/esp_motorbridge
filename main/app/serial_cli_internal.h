#pragma once

#include <stdbool.h>
#include <stdint.h>

extern const char *g_serial_cli_tag;
extern volatile bool g_serial_cli_watch_enabled;
extern int g_serial_cli_watch_period_ms;
extern int64_t g_serial_cli_last_watch_us;
extern bool g_serial_cli_watch_active_poll;
extern int g_serial_cli_watch_poll_wait_ms;

int serial_cli_parse_int(const char *s, int def);
