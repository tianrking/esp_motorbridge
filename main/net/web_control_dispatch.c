#include "net/web_control_dispatch.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/command_router.h"
#include "config/app_config.h"

static const char *TAG = "web_control";
static const int INTER_ID_DELAY_MS = 10;

static int16_t clamp_i16_from_f(float x, float scale)
{
    float v = x * scale;
    if (v > 32767.0f) {
        v = 32767.0f;
    }
    if (v < -32768.0f) {
        v = -32768.0f;
    }
    return (int16_t)v;
}

static uint16_t clamp_u16_from_f(float x, float scale)
{
    float v = x * scale;
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > 65535.0f) {
        v = 65535.0f;
    }
    return (uint16_t)v;
}

bool web_control_parse_float(const char *query, const char *key, float *out)
{
    if (query == NULL || out == NULL) {
        return false;
    }
    char tmp[32] = {0};
    if (httpd_query_key_value(query, key, tmp, sizeof(tmp)) != ESP_OK) {
        return false;
    }
    *out = strtof(tmp, NULL);
    return true;
}

bool web_control_parse_vendor_name(const char *query, char *out, size_t out_len)
{
    if (query == NULL || out == NULL || out_len == 0) {
        return false;
    }
    if (httpd_query_key_value(query, "vendor", out, out_len) != ESP_OK) {
        return false;
    }
    return strcmp(out, "damiao") == 0 || strcmp(out, "robstride") == 0;
}

int web_control_parse_ids(const char *csv, uint8_t *ids, int max_ids)
{
    if (csv == NULL || ids == NULL || max_ids <= 0) {
        return 0;
    }

    int count = 0;
    const char *p = csv;
    while (*p != '\0' && count < max_ids) {
        while (*p == ' ' || *p == ',') {
            p++;
        }
        if (!isdigit((unsigned char)*p)) {
            if (*p == '\0') {
                break;
            }
            p++;
            continue;
        }
        long v = strtol(p, (char **)&p, 10);
        if (v >= MOTORBRIDGE_MIN_MOTOR_ID && v <= MOTORBRIDGE_MAX_MOTOR_ID) {
            ids[count++] = (uint8_t)v;
        }
        while (*p != '\0' && *p != ',') {
            p++;
        }
    }
    return count;
}

void web_control_dispatch_admin(uint8_t id, uint8_t op, uint8_t mode)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = op;
    msg.data[1] = id;
    msg.data[2] = mode;
    command_router_handle_can_rx(&msg);
}

static void inter_id_delay(int idx, int total)
{
    if (idx < total - 1) {
        vTaskDelay(pdMS_TO_TICKS(INTER_ID_DELAY_MS));
    }
}

void web_control_pace_then_next(int idx, int total)
{
    inter_id_delay(idx, total);
}

void web_control_dispatch_set_gains(uint8_t id, float kp, float kd)
{
    twai_message_t msg = {0};
    if (kp < 0.0f) {
        kp = 0.0f;
    }
    if (kd < 0.0f) {
        kd = 0.0f;
    }

    uint16_t kp_u = (uint16_t)clamp_u16_from_f(kp, 100.0f);
    uint16_t kd_u = (uint16_t)clamp_u16_from_f(kd, 1000.0f);

    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = 8; // HOST_OP_SET_GAINS
    msg.data[1] = id;
    msg.data[2] = (uint8_t)(kp_u & 0xFF);
    msg.data[3] = (uint8_t)((kp_u >> 8) & 0xFF);
    msg.data[4] = (uint8_t)(kd_u & 0xFF);
    msg.data[5] = (uint8_t)((kd_u >> 8) & 0xFF);
    command_router_handle_can_rx(&msg);
}

void web_control_dispatch_set_ids(uint8_t old_id, uint8_t new_id, bool store_after_set, bool verify_after_set)
{
    twai_message_t msg = {0};
    msg.identifier = MOTORBRIDGE_HOST_ADMIN_ID;
    msg.data_length_code = 8;
    msg.data[0] = 9; // HOST_OP_SET_IDS
    msg.data[1] = old_id;
    msg.data[2] = new_id; // new motor id
    msg.data[3] = new_id; // new feedback id
    msg.data[4] = (store_after_set ? 0x01 : 0) | (verify_after_set ? 0x02 : 0);
    command_router_handle_can_rx(&msg);
}

void web_control_dispatch_mode(uint8_t id, uint8_t mode, float pos, float vel, float tau, float vlim, float ratio)
{
    twai_message_t msg = {0};
    msg.data_length_code = 8;
    int16_t a = 0;
    int16_t b = 0;
    uint16_t c = 0;

    switch (mode) {
    case MOTOR_MODE_MIT:
        msg.identifier = MOTORBRIDGE_HOST_MIT_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vel, 100.0f);
        c = (uint16_t)clamp_i16_from_f(tau, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        msg.data[4] = (uint8_t)(c & 0xFF);
        msg.data[5] = (uint8_t)((c >> 8) & 0xFF);
        break;
    case MOTOR_MODE_POS_VEL:
        msg.identifier = MOTORBRIDGE_HOST_POS_VEL_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vlim, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        break;
    case MOTOR_MODE_VEL:
        msg.identifier = MOTORBRIDGE_HOST_VEL_BASE_ID + id;
        a = clamp_i16_from_f(vel, 100.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        break;
    case MOTOR_MODE_FORCE_POS:
        msg.identifier = MOTORBRIDGE_HOST_FORCE_POS_BASE_ID + id;
        a = clamp_i16_from_f(pos, 1000.0f);
        b = clamp_i16_from_f(vlim, 100.0f);
        c = clamp_u16_from_f(ratio, 10000.0f);
        msg.data[0] = (uint8_t)(a & 0xFF);
        msg.data[1] = (uint8_t)((a >> 8) & 0xFF);
        msg.data[2] = (uint8_t)(b & 0xFF);
        msg.data[3] = (uint8_t)((b >> 8) & 0xFF);
        msg.data[4] = (uint8_t)(c & 0xFF);
        msg.data[5] = (uint8_t)((c >> 8) & 0xFF);
        break;
    default:
        return;
    }

    command_router_handle_can_rx(&msg);
}

void web_control_log_action(const char *action,
                            const uint8_t *ids,
                            int n,
                            int mode,
                            float pos,
                            float vel,
                            float tau,
                            float vlim,
                            float ratio)
{
    char ids_buf[96] = {0};
    size_t used = 0;
    for (int i = 0; i < n && used + 6 < sizeof(ids_buf); ++i) {
        int w = snprintf(ids_buf + used, sizeof(ids_buf) - used, "%s%u", (i == 0) ? "" : ",", ids[i]);
        if (w <= 0) {
            break;
        }
        used += (size_t)w;
    }

    if (strcmp(action, "set_gains") == 0) {
        ESP_LOGI(TAG, "web action=set_gains ids=[%s] kp=%.3f kd=%.3f", ids_buf, pos, vel);
        return;
    }
    ESP_LOGI(TAG,
             "web action=%s ids=[%s] mode=%d pos=%.3f vel=%.3f tau=%.3f vlim=%.3f ratio=%.4f",
             action,
             ids_buf,
             mode,
             pos,
             vel,
             tau,
             vlim,
             ratio);
}
