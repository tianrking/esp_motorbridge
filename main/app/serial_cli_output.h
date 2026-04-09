#pragma once

#include <stdint.h>

void serial_cli_print_help(void);
void serial_cli_log_online_ids(int min_id, int max_id);
void serial_cli_log_seen_ids(int min_id, int max_id, int32_t recent_ms);
void serial_cli_log_online_states(void);
void serial_cli_log_one_state(uint8_t id);
