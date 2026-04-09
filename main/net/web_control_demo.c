#include "net/web_control_demo.h"

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "net/web_control_dispatch.h"

static volatile bool s_demo_running = false;
static volatile bool s_demo_stop_req = false;
static volatile int s_demo_id = 0;
static TaskHandle_t s_demo_task = NULL;

static float triangle_0_to_3(float phase)
{
    float p = fmodf(phase, 2.0f);
    if (p < 0.0f) {
        p += 2.0f;
    }
    if (p < 1.0f) {
        return p * 3.0f;
    }
    return (2.0f - p) * 3.0f;
}

void web_control_demo_wait_stopped(int timeout_ms)
{
    int waited = 0;
    while (s_demo_running && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void prepare_demo_all_posvel(int max_id)
{
    for (int id = 1; id <= max_id; ++id) {
        web_control_dispatch_admin((uint8_t)id, 5, 0);
        vTaskDelay(pdMS_TO_TICKS(8));
        web_control_dispatch_admin((uint8_t)id, 7, 0);
        vTaskDelay(pdMS_TO_TICKS(8));
        web_control_dispatch_admin((uint8_t)id, 1, MOTOR_MODE_POS_VEL);
        vTaskDelay(pdMS_TO_TICKS(8));
        web_control_dispatch_admin((uint8_t)id, 4, 0);
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

static void demo_task(void *arg)
{
    (void)arg;
    const app_config_t *cfg = app_config_get();
    const int max_id = cfg->max_motors;
    const float vlim = 1.0f;
    const int demo_id = s_demo_id;
    float t = 0.0f;

    prepare_demo_all_posvel(max_id);

    while (!s_demo_stop_req) {
        for (int id = 1; id <= max_id; ++id) {
            float pos = 0.0f;
            if (demo_id == 1) {
                pos = triangle_0_to_3(t);
            } else if (demo_id == 2) {
                const float phase = t * 1.2f + ((float)(id - 1) * 0.45f);
                pos = 1.5f + 1.45f * sinf(phase);
            } else {
                const float phase = t + ((id % 2 == 0) ? 1.0f : 0.0f);
                pos = triangle_0_to_3(phase);
            }
            web_control_dispatch_mode((uint8_t)id, MOTOR_MODE_POS_VEL, pos, 0.0f, 0.0f, vlim, 0.1f);
        }
        t += 0.05f;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    s_demo_running = false;
    s_demo_stop_req = false;
    s_demo_id = 0;
    s_demo_task = NULL;
    vTaskDelete(NULL);
}

bool web_control_demo_running(void)
{
    return s_demo_running;
}

int web_control_demo_id(void)
{
    return (int)s_demo_id;
}

esp_err_t web_control_demo_start(int demo_id, char *resp, size_t resp_len)
{
    if (demo_id < 1 || demo_id > 3) {
        snprintf(resp, resp_len, "err bad demo_id");
        return ESP_FAIL;
    }

    if (s_demo_running) {
        s_demo_stop_req = true;
        web_control_demo_wait_stopped(1500);
    }
    if (s_demo_running) {
        snprintf(resp, resp_len, "err demo busy, stop timeout");
        return ESP_FAIL;
    }

    s_demo_stop_req = false;
    s_demo_id = demo_id;
    s_demo_running = true;
    if (xTaskCreatePinnedToCore(demo_task, "demo_task", 4096, NULL, 6, &s_demo_task, tskNO_AFFINITY) != pdPASS) {
        s_demo_running = false;
        s_demo_id = 0;
        snprintf(resp, resp_len, "err demo start failed");
        return ESP_FAIL;
    }

    snprintf(resp, resp_len, "ok demo%d started", demo_id);
    return ESP_OK;
}

void web_control_demo_request_stop(void)
{
    s_demo_stop_req = true;
}

esp_err_t web_control_demo_reset(char *resp, size_t resp_len)
{
    const app_config_t *cfg = app_config_get();
    const int max_id = cfg->max_motors;

    s_demo_stop_req = true;
    web_control_demo_wait_stopped(1500);

    for (int id = 1; id <= max_id; ++id) {
        web_control_dispatch_admin((uint8_t)id, 5, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        web_control_dispatch_admin((uint8_t)id, 7, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        web_control_dispatch_admin((uint8_t)id, 1, MOTOR_MODE_POS_VEL);
        vTaskDelay(pdMS_TO_TICKS(10));
        web_control_dispatch_admin((uint8_t)id, 4, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        web_control_dispatch_mode((uint8_t)id, MOTOR_MODE_POS_VEL, 0.0f, 0.0f, 0.0f, 0.6f, 0.1f);
        vTaskDelay(pdMS_TO_TICKS(12));
    }

    snprintf(resp, resp_len, "ok demo reset count=%d", max_id);
    return ESP_OK;
}
