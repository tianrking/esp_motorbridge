#pragma once

#include <stdbool.h>

#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*can_rx_callback_t)(const twai_message_t *msg);

esp_err_t can_manager_init(int tx_gpio, int rx_gpio, int bitrate);
esp_err_t can_manager_start(void);
void can_manager_set_rx_callback(can_rx_callback_t cb);
esp_err_t can_manager_send(const twai_message_t *msg, int timeout_ms);
void can_manager_trigger_scan(int min_id, int max_id);
void can_manager_set_dump(bool enabled);

#ifdef __cplusplus
}
#endif
