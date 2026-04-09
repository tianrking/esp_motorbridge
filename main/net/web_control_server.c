#include "net/web_control_server.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "net/web_control_actions.h"
#include "net/web_control_page.h"

static const char *TAG = "web_control";
static httpd_handle_t s_server = NULL;

static esp_err_t control_get_handler(httpd_req_t *req)
{
    char query[256] = {0};
    char resp[1024] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query");
        return ESP_FAIL;
    }

    esp_err_t err = web_control_execute_action_from_query(query, resp, sizeof(resp));
    if (err == ESP_OK) {
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, resp);
    }
    return err;
}

#if CONFIG_HTTPD_WS_SUPPORT
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t *payload = calloc(1, frame.len + 1);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }

    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        free(payload);
        return err;
    }
    payload[frame.len] = '\0';

    char *resp = calloc(1, 1024);
    if (resp == NULL) {
        free(payload);
        return ESP_ERR_NO_MEM;
    }
    (void)web_control_execute_action_from_query((char *)payload, resp, 1024);

    char rid[24] = {0};
    bool has_rid = (httpd_query_key_value((char *)payload, "rid", rid, sizeof(rid)) == ESP_OK);
    free(payload);

    char *out_buf = calloc(1, 1200);
    if (out_buf == NULL) {
        free(resp);
        return ESP_ERR_NO_MEM;
    }
    if (has_rid) {
        snprintf(out_buf, 1200, "rid=%s;%s", rid, resp);
    } else {
        strlcpy(out_buf, resp, 1200);
    }
    free(resp);

    httpd_ws_frame_t out = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)out_buf,
        .len = strlen(out_buf),
    };
    esp_err_t send_err = httpd_ws_send_frame(req, &out);
    free(out_buf);
    return send_err;
}
#endif

esp_err_t web_control_server_start(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.server_port = 80;
    cfg.stack_size = 8192;

    ESP_LOGI(TAG, "starting web control http://192.168.4.1/");
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        return ESP_FAIL;
    }

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = web_control_root_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t control = {
        .uri = "/api/control",
        .method = HTTP_GET,
        .handler = control_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = web_control_favicon_get_handler,
        .user_ctx = NULL,
    };
#if CONFIG_HTTPD_WS_SUPPORT
    const httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
#endif

    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &control);
    httpd_register_uri_handler(s_server, &favicon);
#if CONFIG_HTTPD_WS_SUPPORT
    httpd_register_uri_handler(s_server, &ws);
#else
    ESP_LOGW(TAG, "WebSocket disabled in sdkconfig, fallback to HTTP GET control");
#endif

    return ESP_OK;
}
