#include "safety_manager.h"

#include "esp_log.h"

static const char *TAG = "safety_manager";
static bool s_logged_disabled = false;

void safety_manager_tick(void)
{
    // User policy: do not auto-disable/mark-offline motors based on feedback timeout.
    // Keep safety task alive for future checks (over-temp, range, etc.), but timeout check is disabled.
    if (!s_logged_disabled) {
        ESP_LOGW(TAG, "offline timeout safety check disabled");
        s_logged_disabled = true;
    }
}
