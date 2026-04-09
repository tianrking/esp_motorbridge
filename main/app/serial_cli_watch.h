#pragma once

#include <stdbool.h>
#include <stdint.h>

bool serial_cli_active_poll_id(uint8_t id, int wait_ms);
void serial_cli_active_poll_online_states(int wait_ms);
void serial_cli_watch_tick(void);
