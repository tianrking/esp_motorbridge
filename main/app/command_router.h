#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

void command_router_handle_can_rx(const twai_message_t *msg);
int command_router_collect_seen_ids(int min_id, int max_id, uint8_t *out_ids, int max_ids, int32_t recent_ms);
bool command_router_wait_feedback(uint8_t id, int32_t timeout_ms);

#ifdef __cplusplus
}
#endif
